//! The encoder against the archive: the same streams through both, and what each holds compared
//! sample by sample, pin by pin, loss by loss and marker by marker.

use super::*;
use crate::capture::{Archive, CaptureWriter, Interval};
use crate::stream::{DioLost, Edge, Edges, End, Marker, Pair};
use escapement_tickwave_format::decode::{BlockChannel, Footer};
use escapement_tickwave_format::header::ChannelKind;
use escapement_tickwave_format::model::sample_tick;
use std::sync::atomic::AtomicBool;

const CLOCK: u32 = 96_000_000;

/// The firmware's time for a count of its clock: rounded down to ns.
fn ns(count: u64) -> u64 {
    (u128::from(count) * 1_000_000_000 / u128::from(CLOCK)) as u64
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    fn one_in(&mut self, n: u64) -> bool {
        self.below(n) == 0
    }
}

#[derive(Clone, Copy)]
struct Shape {
    seed: u64,
    rate: u32,
    first_sequence: u32,
    dio_mask: u8,
    messages: usize,
    end: bool,
}

fn header(shape: Shape) -> Header {
    Header {
        capture_id: None,
        clock_hz: CLOCK,
        period: CLOCK / shape.rate,
        setting_ma: Some(350),
        full_scale_mv: 3300,
        first_sequence: shape.first_sequence,
        previous_low: true,
        low: vec![
            CalibrationPoint {
                code: 0,
                current_nanoamps: 0,
            },
            CalibrationPoint {
                code: 65535,
                current_nanoamps: 100_000,
            },
        ],
        high: vec![
            CalibrationPoint {
                code: 0,
                current_nanoamps: 0,
            },
            CalibrationPoint {
                code: 65535,
                current_nanoamps: 350_000_000,
            },
        ],
        dio_mask: shape.dio_mask,
        dio_levels: 0b0101 & shape.dio_mask,
        edge_resolution_ns: 11,
        switch_weight_q16: 12_000,
        rearm_between_pairs: false,
        previous_shunt_nanoamps: None,
        high_range_only: false,
    }
}

/// A stream as the probe sends one: pairs at the period with losses, pauses and holds, range
/// switches, DIO edges (pulses at one time among them) and loss reports, and markers.
fn stream(shape: Shape) -> Vec<Message> {
    let mut rng = Rng(shape.seed | 1);
    let header = header(shape);
    let period = u64::from(header.period);
    let mut out = vec![Message::Header(header.clone())];
    let mut sequence = shape.first_sequence;
    // An attached stream starts well after its capture did.
    let mut count = if shape.first_sequence > 1 {
        period * u64::from(shape.first_sequence)
    } else {
        0
    };
    let mut low = true;
    let mut levels = header.dio_levels;
    // The last DIO count taken in: an edge, or a loss report's end. An attached stream times
    // edges from its request, after the pair before its first.
    let mut dio_after = count.saturating_sub(if shape.first_sequence > 1 { period } else { 0 });
    if rng.one_in(3) {
        // The probe dropped the first pairs messages after the header.
        let lost = 1 + rng.below(40);
        sequence = sequence.wrapping_add(lost as u32);
        count += lost * period;
    }
    for m in 0..shape.messages {
        let mut flags = 0;
        if m > 0 {
            match rng.below(12) {
                0 => {
                    let lost = 1 + rng.below(300);
                    sequence = sequence.wrapping_add(lost as u32);
                    count += lost * period;
                }
                1 => {
                    flags = Pairs::PAUSED;
                    count += period * (1 + rng.below(50)) + rng.below(period);
                }
                2 => {
                    flags = Pairs::HELD;
                    count += period * (1 + rng.below(50)) + rng.below(period);
                }
                _ => {}
            }
        }
        let k = 1 + rng.below(99) as usize;
        let first = count;
        let pairs: Vec<Pair> = (0..k)
            .map(|_| {
                if rng.one_in(9) {
                    low = !low;
                }
                Pair {
                    voltage_code: 30_000 + rng.below(2_000) as u16,
                    current_code: rng.below(65_536) as u16,
                    low,
                    under_range: !low && rng.one_in(5),
                }
            })
            .collect();
        out.push(Message::Pairs(Pairs::new(
            flags,
            sequence,
            ns(first),
            pairs,
        )));
        sequence = sequence.wrapping_add(k as u32);
        count += k as u64 * period;
        let last = first + (k as u64 - 1) * period;
        // The edges and loss reports up to the last pair, before the next pairs message.
        if shape.dio_mask != 0 {
            let mut edges = Vec::new();
            let mut at = dio_after;
            while at < last {
                at += 1 + rng.below(3 * period);
                if at > last {
                    break;
                }
                if rng.one_in(15) {
                    if !edges.is_empty() {
                        out.push(Message::Edges(Edges::new_at(std::mem::take(&mut edges))));
                    }
                    let to = at + 1 + rng.below(2 * period);
                    levels = rng.below(16) as u8 & shape.dio_mask;
                    out.push(Message::DioLost(DioLost {
                        from_ns: ns(at),
                        to_ns: ns(to),
                        levels,
                    }));
                    at = to;
                    dio_after = to;
                    continue;
                }
                let pin = 1 << rng.below(4);
                if pin & shape.dio_mask == 0 {
                    continue;
                }
                levels ^= pin;
                edges.push((at, levels));
                if rng.one_in(6) {
                    // A pulse between two runs of the interrupt: two edges at one time.
                    levels ^= pin;
                    edges.push((at, levels));
                }
                dio_after = at;
            }
            if !edges.is_empty() {
                out.push(Message::Edges(Edges::new_at(edges)));
            }
            // The probe has sent every edge up to the last pair before the next pairs message.
            dio_after = dio_after.max(last);
        }
        if rng.one_in(4) {
            let n = 1 + rng.below(3);
            let markers: Vec<Marker> = (0..n)
                .map(|i| Marker {
                    time_ns: ns(first + rng.below(last - first + 1)),
                    number: (m as u32) * 10 + i as u32,
                    label: if rng.one_in(5) {
                        vec![b'x', 0xff, b'\\']
                    } else {
                        format!("m{m}.{i}").into_bytes()
                    },
                })
                .collect();
            let mut markers = markers;
            if rng.one_in(3) && markers.len() > 1 {
                markers[1].time_ns = markers[0].time_ns;
            }
            markers.sort_by_key(|marker| marker.time_ns);
            let flags = if rng.one_in(4) { Markers::LOST } else { 0 };
            out.push(Message::Markers(Markers::new(flags, markers)));
        } else if rng.one_in(12) {
            // The probe lost markers and has none to send.
            out.push(Message::Markers(Markers::new(Markers::LOST, vec![])));
        }
    }
    if shape.end {
        out.push(Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: sequence.wrapping_sub(1).wrapping_add(rng.below(3) as u32),
        }));
    }
    out
}

/// `edges` as the probe sends an edges message: one first time, then offsets rounded down.
#[allow(clippy::needless_pass_by_value)]
fn edges_message(edges: Vec<(u64, u8)>) -> Vec<Edge> {
    let first = edges[0].0;
    edges
        .iter()
        .map(|&(at, levels)| Edge {
            time_ns: ns(first) + ns(at - first),
            levels,
        })
        .collect()
}

impl Edges {
    fn new_at(counts: Vec<(u64, u8)>) -> Edges {
        Edges::new(0, edges_message(counts))
    }
}

/// What a finished recording holds, decoded.
struct Decoded {
    header: escapement_tickwave_format::header::Header,
    edges: Vec<Vec<i64>>,
    codes: Vec<Vec<u64>>,
    annotations: Vec<serde_json::Map<String, serde_json::Value>>,
}

fn decode(file: &[u8]) -> Decoded {
    let head = Footer::parse_file(file).unwrap();
    let n = head.header.channels.len();
    let (mut edges, mut codes) = (vec![Vec::new(); n], vec![Vec::new(); n]);
    for i in 0..head.rows.len() {
        let (offset, len) = head.block_location(i);
        let block = head
            .decode_block(i, &file[offset as usize..][..len as usize])
            .unwrap();
        for (c, channel) in block.channels.into_iter().enumerate() {
            match channel {
                BlockChannel::Digital { edges: e, .. } => edges[c].extend(e),
                BlockChannel::Analog { values, .. } => {
                    codes[c].extend(values.iter().map(|sample| match sample {
                        escapement_tickwave_format::model::Sample::U64(code) => code,
                        other => panic!("{other:?}"),
                    }))
                }
            }
        }
    }
    let mut annotations = Vec::new();
    for (name, text) in head.header.annotations.iter().flatten() {
        if name.ends_with(".jsonl") {
            for line in text.lines().skip(1) {
                annotations.push(serde_json::from_str(line).unwrap());
            }
        }
    }
    Decoded {
        header: head.header,
        edges,
        codes,
        annotations,
    }
}

/// A digital channel's level at `tick`, from its initial level, edges and gaps; none inside a gap.
fn level_at(
    header: &escapement_tickwave_format::header::Header,
    edges: &[i64],
    c: usize,
    tick: i64,
) -> Option<bool> {
    let ChannelKind::Digital { initial, gaps, .. } = &header.channels[c].data else {
        panic!("digital");
    };
    let mut level = *initial;
    let mut gaps = gaps.iter().peekable();
    let mut edges = edges.iter().peekable();
    loop {
        let edge = edges.peek().map(|&&t| t).filter(|&t| t <= tick);
        let gap = gaps.peek().filter(|g| g.from <= tick);
        match (edge, gap) {
            (Some(e), Some(g)) if g.from < e => {
                if tick < g.to {
                    return None;
                }
                level = g.level;
                gaps.next();
            }
            (Some(_), _) => {
                level = !level;
                edges.next();
            }
            (None, Some(g)) => {
                if tick < g.to {
                    return None;
                }
                level = g.level;
                gaps.next();
            }
            (None, None) => return Some(level),
        }
    }
}

/// One pin's history as events: an edge, the start of what is unknown, and the level known again.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Event {
    Edge(i64),
    Unknown(i64),
    Known(i64, bool),
}

fn pin_events_tickwave(d: &Decoded, c: usize) -> Vec<Event> {
    let ChannelKind::Digital { gaps, .. } = &d.header.channels[c].data else {
        panic!("digital");
    };
    let mut events: Vec<Event> = d.edges[c].iter().map(|&t| Event::Edge(t)).collect();
    for g in gaps {
        events.push(Event::Unknown(g.from));
        events.push(Event::Known(g.to, g.level));
    }
    events.sort_by_key(|e| match *e {
        Event::Edge(t) => (t, 1),
        Event::Unknown(t) => (t, 0),
        Event::Known(t, _) => (t, 2),
    });
    events
}

fn pin_events_archive(archive: &Archive, pin: u8, clock_hz: u32, base: i64) -> Vec<Event> {
    let cancel = AtomicBool::new(false);
    let mut events = Vec::new();
    let mut level: Option<bool> = None;
    archive
        .digital_spans(Interval::new(0, u64::MAX / 2), &cancel, |span| {
            let at = tick(span.interval.start_ns, clock_hz) - base;
            let new = span.levels.map(|levels| levels & (1 << pin) != 0);
            match (level, new) {
                (Some(a), Some(b)) if a != b => events.push(Event::Edge(at)),
                (Some(_), None) => events.push(Event::Unknown(at)),
                (None, Some(b)) if !events.is_empty() || at > 0 => {
                    events.push(Event::Known(at, b));
                }
                _ => {}
            }
            level = new;
            Ok(())
        })
        .unwrap();
    events
}

fn encode(messages: &[Message]) -> Vec<u8> {
    let mut encoder = Encoder::new(Context::default());
    let mut file = Vec::new();
    for message in messages {
        encoder.ingest(message).unwrap();
        file.extend(encoder.take_output());
        encoder.flush().unwrap();
        file.extend(encoder.take_output());
    }
    let finished = encoder.finish("capture").unwrap().unwrap();
    file.extend(finished.tail);
    file
}

fn archive(dir: &Path, messages: &[Message]) -> Archive {
    let path = dir.join("capture.vergecapture");
    let mut writer = CaptureWriter::create(&path).unwrap();
    for message in messages {
        writer.ingest(message.clone()).unwrap();
    }
    writer.finish().unwrap();
    Archive::open(&path).unwrap()
}

/// What one comparison covered.
#[derive(Default, Debug)]
struct Tally {
    samples: usize,
    lost_spans: usize,
    range_gaps: usize,
    dio_events: usize,
    dio_gaps: usize,
    markers: usize,
    after_loss: usize,
    attached: usize,
}

fn compare(shape: Shape) -> Tally {
    let mut tally = Tally::default();
    let messages = stream(shape);
    let dir = tempfile::tempdir().unwrap();
    let archive = archive(dir.path(), &messages);
    let file = encode(&messages);
    let d = decode(&file);
    let cancel = AtomicBool::new(false);
    let header = header(shape);
    let period = i64::from(header.period);
    // The capture's tick at the recording's tick 0.
    let base = -d.header.sources[0].origin;

    // Samples: the same ticks, readings and flags.
    let mut stored = Vec::new();
    archive
        .overlapping(Interval::new(0, u64::MAX / 2), &cancel, |block| {
            stored.extend(block.into_iter().map(|s| s.sample));
            Ok(())
        })
        .unwrap();
    let ChannelKind::Analog {
        first,
        breaks,
        count,
        ..
    } = &d.header.channels[V].data
    else {
        panic!("analog");
    };
    assert_eq!(*count as usize, stored.len(), "{}", shape.seed);
    let mut runs = vec![escapement_tickwave_format::model::Break {
        index: 0,
        tick: *first,
        lost: 0,
    }];
    runs.extend(breaks.iter().copied());
    let mut run = 0;
    for (i, s) in stored.iter().enumerate() {
        while run + 1 < runs.len() && runs[run + 1].index <= i as u64 {
            run += 1;
        }
        let r = runs[run];
        let at = sample_tick(
            r.tick,
            Ratio::integer(header.period as u64).unwrap(),
            i as u64 - r.index,
        );
        assert_eq!(
            at,
            tick(s.time_ns, CLOCK) - base,
            "sample {i}, seed {}",
            shape.seed
        );
        assert_eq!(d.codes[V][i], u64::from(s.millivolts));
        assert_eq!(d.codes[I][i], u64::from(s.nanoamps));
        assert_eq!(
            level_at(&d.header, &d.edges[LOW_RANGE], LOW_RANGE, at),
            Some(s.range == Some(CurrentRange::Low)),
            "low_range at sample {i}, seed {}",
            shape.seed
        );
        assert_eq!(
            level_at(&d.header, &d.edges[UNDER_RANGE], UNDER_RANGE, at),
            Some(s.under_range),
            "under_range at sample {i}"
        );
    }

    // Losses: the same spans, to the ns the archive's rounded-up period moves them.
    let lost = archive
        .lost_spans(Interval::new(0, u64::MAX / 2), &cancel)
        .unwrap();
    let mine: Vec<(i64, i64)> = runs
        .windows(2)
        .filter(|w| w[1].lost > 0)
        .filter_map(|w| {
            let n = w[1].index;
            let last = sample_tick(
                w[0].tick,
                Ratio::integer(header.period as u64).unwrap(),
                n - 1 - w[0].index,
            );
            let from = last + period;
            let to = (from + w[1].lost as i64 * period).min(w[1].tick);
            (to > from).then_some((from, to))
        })
        .collect();
    assert_eq!(lost.len(), mine.len(), "lost spans, seed {}", shape.seed);
    tally.samples = stored.len();
    tally.lost_spans = lost.len();
    // Every pair lost: in breaks, and before the first or after the last, which the extra counts.
    let extra = &d.header.sources[0].extra;
    let edges_lost: u64 = ["pairs_lost_before", "pairs_lost_after"]
        .iter()
        .map(|key| extra[*key].parse::<u64>().unwrap())
        .sum();
    let view = archive
        .view(Interval::new(0, u64::MAX / 2), 1, &cancel)
        .unwrap();
    assert_eq!(
        runs.iter().map(|r| r.lost).sum::<u64>() + edges_lost,
        view.lost_pairs,
        "pairs lost, seed {}",
        shape.seed
    );

    // The range channels: unknown before the first sample, and over each break that lost pairs.
    let mut expected = Vec::new();
    if *count > 0 && *first > 0 {
        expected.push((0, *first));
    }
    for w in runs.windows(2).filter(|w| w[1].lost > 0) {
        let n = w[1].index;
        let last = sample_tick(
            w[0].tick,
            Ratio::integer(header.period as u64).unwrap(),
            n - 1 - w[0].index,
        );
        if w[1].tick > last + period {
            expected.push((last + period, w[1].tick));
        }
    }
    for c in [LOW_RANGE, UNDER_RANGE] {
        let ChannelKind::Digital { gaps, .. } = &d.header.channels[c].data else {
            panic!("digital");
        };
        let gaps: Vec<(i64, i64)> = gaps.iter().map(|g| (g.from, g.to)).collect();
        assert_eq!(
            gaps, expected,
            "range gaps, channel {c}, seed {}",
            shape.seed
        );
        tally.range_gaps += gaps.len();
    }
    for (span, &(from, to)) in lost.iter().zip(&mine) {
        assert!(
            span.start_ns.abs_diff(nanos(from + base, CLOCK)) <= 1,
            "{span:?} {from}"
        );
        assert!(
            span.end_ns.abs_diff(nanos(to + base, CLOCK)) <= 1 + (span.duration_ns() / 1000),
            "{span:?} {to}"
        );
    }

    // DIO, pin by pin: the same edges and unknown spans.
    let pins: Vec<u8> = (0..4).filter(|k| shape.dio_mask & (1 << k) != 0).collect();
    for (i, &pin) in pins.iter().enumerate() {
        let c = FIRST_DIO + i;
        let mut mine = pin_events_tickwave(&d, c);
        if shape.first_sequence > 1 {
            // An attached stream's pins are unknown until it started; the archive takes them as
            // known from tick 0.
            assert_eq!(mine.first(), Some(&Event::Unknown(0)));
            let Some(&Event::Known(to, _)) = mine.get(1) else {
                panic!("an attached stream's first gap, seed {}", shape.seed);
            };
            // Up to the first pair, and before the first edge or loss report.
            assert!(*count == 0 || to <= *first, "seed {}", shape.seed);
            if let Some(next) = mine.get(2) {
                let at = match *next {
                    Event::Edge(t) | Event::Unknown(t) | Event::Known(t, _) => t,
                };
                assert!(to < at, "seed {}", shape.seed);
            }
            mine.drain(..2);
            tally.attached += 1;
        }
        let theirs = pin_events_archive(&archive, pin, CLOCK, base);
        assert_eq!(mine, theirs, "DIO{pin}, seed {}", shape.seed);
        tally.dio_events += mine.len();
        tally.dio_gaps += mine
            .iter()
            .filter(|e| matches!(e, Event::Unknown(_)))
            .count();
    }

    // Markers: the same times, numbers, labels and loss flags, in order.
    let page = archive
        .markers(Interval::new(0, u64::MAX / 2), None, 1 << 20, &cancel)
        .unwrap();
    assert_eq!(
        page.entries.len(),
        d.annotations.len(),
        "markers, seed {}",
        shape.seed
    );
    let mut annotations = d.annotations.clone();
    annotations.sort_by_key(|a| a["start"].as_i64().unwrap());
    for (entry, a) in page.entries.iter().zip(&annotations) {
        assert_eq!(
            a["start"].as_i64().unwrap(),
            tick(entry.marker.time_ns, CLOCK) - base
        );
        assert_eq!(
            a["values"]["number"].as_str().unwrap(),
            entry.marker.number.to_string()
        );
        assert_eq!(
            a["label"].as_str().unwrap(),
            crate::record::marker_label(&entry.marker.label)
        );
        assert_eq!(
            a["kind"].as_str().unwrap(),
            if entry.lost_before {
                "marker-after-loss"
            } else {
                "marker"
            }
        );
    }
    let ids: std::collections::HashSet<_> = d
        .annotations
        .iter()
        .map(|a| a["id"].as_str().unwrap().to_owned())
        .collect();
    assert_eq!(ids.len(), d.annotations.len(), "marker ids are unique");
    tally.markers = page.entries.len();
    tally.after_loss = page.entries.iter().filter(|e| e.lost_before).count();

    // The header round-trips through the source's extra.
    assert_eq!(header_from(&d.header.sources[0].extra), Some(header));
    tally
}

fn shapes() -> Vec<Shape> {
    let mut shapes = Vec::new();
    for seed in 1..=40u64 {
        shapes.push(Shape {
            seed,
            // A stream attached past the sequence's wrap comes 12 hours into a 100k capture; at
            // a slower rate the wrap is years away.
            rate: if seed % 7 == 0 {
                100_000
            } else {
                [1_000, 10_000, 100_000, 7][seed as usize % 4]
            },
            first_sequence: if seed % 7 == 0 {
                u32::MAX - 2
            } else if seed % 3 == 0 {
                5_000
            } else {
                1
            },
            dio_mask: [0b1111, 0b0101, 0, 0b1000][seed as usize % 4],
            messages: 5 + (seed as usize * 13) % 200,
            end: seed % 5 != 0,
        });
    }
    shapes
}

#[test]
fn the_encoder_holds_what_the_archive_holds() {
    let mut total = Tally::default();
    for shape in shapes() {
        let t = compare(shape);
        total.samples += t.samples;
        total.lost_spans += t.lost_spans;
        total.range_gaps += t.range_gaps;
        total.dio_events += t.dio_events;
        total.dio_gaps += t.dio_gaps;
        total.markers += t.markers;
        total.after_loss += t.after_loss;
        total.attached += t.attached;
    }
    eprintln!("{total:?}");
    // A comparison that checked nothing passes too: each kind must have come up.
    assert!(total.samples > 100_000 && total.lost_spans > 20 && total.range_gaps > 20);
    assert!(total.dio_events > 1_000 && total.dio_gaps > 20);
    assert!(total.markers > 50 && total.after_loss > 5 && total.attached > 3);
}

#[test]
fn ticks_round_trip_through_the_probes_ns() {
    for count in (0..10_000_000u64)
        .step_by(7)
        .chain([u64::from(u32::MAX) * 97, 1 << 50])
    {
        assert_eq!(tick(ns(count), CLOCK), count as i64);
        assert_eq!(nanos(count as i64, CLOCK), ns(count));
    }
    // An edge's time is its message's first time plus an offset, each rounded down.
    for first in (0..100_000u64).step_by(13) {
        for offset in [0, 1, 2, 95, 96, 97, 1_000_001] {
            assert_eq!(tick(ns(first) + ns(offset), CLOCK), (first + offset) as i64);
        }
    }
}

/// Each sample's tick and codes, and each marker's start, from a finished file.
fn content(file: &[u8]) -> (Vec<(i64, u64, u64)>, Vec<i64>, u32) {
    let d = decode(file);
    let ChannelKind::Analog {
        first,
        breaks,
        period,
        ..
    } = &d.header.channels[V].data
    else {
        panic!("analog");
    };
    let samples = (0..d.codes[V].len())
        .map(|i| {
            let at = escapement_tickwave_format::model::run_sample_tick(
                *first, *period, breaks, i as u64,
            );
            (at, d.codes[V][i], d.codes[I][i])
        })
        .collect();
    let mut markers: Vec<i64> = d
        .annotations
        .iter()
        .map(|a| a["start"].as_i64().unwrap())
        .collect();
    markers.sort_unstable();
    (samples, markers, d.header.blocks)
}

fn write_with_file_writer(path: &Path, messages: &[Message], finish: bool) {
    let mut writer = FileWriter::create(path, Context::default())
        .unwrap()
        .flushing_always();
    for message in messages {
        writer.ingest(message).unwrap();
    }
    assert!(
        writer.snapshots().lock().unwrap().is_some(),
        "a snapshot was published"
    );
    if finish {
        writer.finish().unwrap();
    }
}

#[test]
fn a_file_writer_flushes_then_compacts_to_the_same_content() {
    let shape = shapes()[13];
    let messages = stream(shape);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.tickwave");
    write_with_file_writer(&path, &messages, false);
    let flushed = std::fs::read(&path).unwrap();
    let mut reader = escapement_tickwave_format::reader::Reader::default();
    reader.feed(&flushed);
    let mut blocks = 0;
    while let Some(event) = reader.next_event().unwrap() {
        blocks += usize::from(matches!(
            event,
            escapement_tickwave_format::reader::Event::Block(_)
        ));
    }
    assert!(blocks > 20, "a block a message: {blocks}");

    std::fs::remove_file(&path).unwrap();
    write_with_file_writer(&path, &messages, true);
    let compacted = std::fs::read(&path).unwrap();
    let whole = encode(&messages);
    assert_eq!(
        Footer::parse_file(&compacted).unwrap().header.content_id,
        Footer::parse_file(&whole).unwrap().header.content_id
    );
    let (samples, markers, count) = content(&compacted);
    assert!(count < 5, "compacted to full-size blocks: {count}");
    let (whole_samples, whole_markers, _) = content(&whole);
    assert_eq!((samples, markers), (whole_samples, whole_markers));
    // Nothing is left beside it.
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}

#[test]
fn a_recording_cut_off_anywhere_recovers_to_a_prefix() {
    let shape = shapes()[13];
    let messages = stream(shape);
    let dir = tempfile::tempdir().unwrap();
    let full = dir.path().join("full.tickwave");
    write_with_file_writer(&full, &messages, false);
    let bytes = std::fs::read(&full).unwrap();
    let (all, all_markers, _) = content(&encode(&messages));
    let mut recovered = 0;
    for cut in (1..=16).map(|k| bytes.len() * k / 16) {
        let path = dir.path().join(format!("cut-{cut}.tickwave"));
        std::fs::write(&path, &bytes[..cut]).unwrap();
        match compact(&path) {
            Ok(()) => {}
            // Too little for a declaration, or nothing after it.
            Err(Error::Recut(_)) if cut < bytes.len() / 8 => {
                continue;
            }
            Err(error) => {
                if let Ok(dir) = std::env::var("TICKWAVE_REPRO") {
                    std::fs::write(format!("{dir}/cut-{cut}.tickwave"), &bytes[..cut]).unwrap();
                }
                panic!("cut at {cut} of {}: {error}", bytes.len())
            }
        }
        let (samples, markers, _) = content(&std::fs::read(&path).unwrap());
        assert_eq!(samples[..], all[..samples.len()], "cut at {cut}");
        assert!(
            markers.iter().all(|m| all_markers.contains(m)),
            "cut at {cut}"
        );
        recovered += 1;
    }
    assert!(recovered >= 12, "{recovered}");
    // A finished file is only compacted, to the same content.
    let finished = dir.path().join("finished.tickwave");
    write_with_file_writer(&finished, &messages, true);
    let before = std::fs::read(&finished).unwrap();
    compact(&finished).unwrap();
    assert_eq!(
        Footer::parse_file(&std::fs::read(&finished).unwrap())
            .unwrap()
            .header
            .content_id,
        Footer::parse_file(&before).unwrap().header.content_id
    );
}

#[test]
fn an_attached_streams_recording_starts_at_its_request_and_refuses_anything_before() {
    let shape = Shape {
        seed: 3,
        rate: 1_000,
        first_sequence: 5_000,
        dio_mask: 0b0001,
        messages: 3,
        end: true,
    };
    let messages = stream(shape);
    let d = decode(&encode(&messages));
    let period = i64::from(header(shape).period);
    assert_eq!(d.header.sources[0].origin, -4_998 * period);
    // An edge stamped before the pair ahead of the first one breaks the stream's rule.
    let mut early = vec![messages[0].clone()];
    early.push(Message::Edges(Edges::new_at(vec![(
        4_998 * period as u64 - 1,
        1,
    )])));
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&early[0]).unwrap();
    assert!(matches!(encoder.ingest(&early[1]), Err(Error::Timestamps)));
}
