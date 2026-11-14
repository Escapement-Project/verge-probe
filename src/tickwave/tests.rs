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
                BlockChannel::Digital {
                    edges: e, source, ..
                } => edges[c].extend(source.unwrap_or(e)),
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
            let at = tick(span.interval.start_ns, clock_hz);
            let new = span.levels.map(|levels| levels & (1 << pin) != 0);
            match (level, new) {
                (Some(a), Some(b)) if a != b => events.push(Event::Edge(at)),
                (Some(_), None) => events.push(Event::Unknown(at)),
                (None, Some(b)) if !events.is_empty() || at > base => {
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
    // The capture's tick at the recording's tick 0. The header and blocks hold the capture's
    // ticks, the annotations the recording's.
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
            tick(s.time_ns, CLOCK),
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
    if *count > 0 && *first > base {
        expected.push((base, *first));
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
            span.start_ns.abs_diff(nanos(from, CLOCK)) <= 1,
            "{span:?} {from}"
        );
        assert!(
            span.end_ns.abs_diff(nanos(to, CLOCK)) <= 1 + (span.duration_ns() / 1000),
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
            // known from the recording's start.
            assert_eq!(mine.first(), Some(&Event::Unknown(base)));
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

/// An export that starts where a DIO loss ends starts as the levels arrive: unknown just before
/// it, known at it. The sigrok session's first sample reads them as the CSV's first row does,
/// whether a pair is at the start or the export opens with the pair before repeated there.
#[test]
fn an_export_from_where_a_dio_loss_ends_knows_its_first_samples_dio() {
    use escapement_tickwave_view::{Cache, Recording};
    let shape = Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 1,
        dio_mask: 0b0100,
        messages: 2,
        end: true,
    };
    let header = header(shape);
    let p = u64::from(header.period);
    let pair = Pair {
        voltage_code: 30_000,
        current_code: 1_000,
        low: true,
        under_range: false,
    };
    // The levels known again at a pair, and inside pairs the probe lost (from 10p to 15p).
    for (known_at, resume, repeated) in [(5 * p, 10 * p, 0), (12 * p, 15 * p, 1)] {
        let sequence = 1 + (resume / p) as u32;
        let messages = [
            Message::Header(header.clone()),
            Message::Pairs(Pairs::new(0, 1, ns(0), vec![pair; 10])),
            Message::DioLost(DioLost {
                from_ns: ns(2 * p + 1),
                to_ns: ns(known_at),
                levels: 0b0100,
            }),
            Message::Pairs(Pairs::new(0, sequence, ns(resume), vec![pair; 10])),
            Message::End(End {
                reason: EndReason::Stopped,
                last_sequence: sequence + 9,
            }),
        ];
        let recording =
            Recording::open(std::sync::Arc::new(encode(&messages)), Cache::new(64 << 20)).unwrap();
        let dir = tempfile::tempdir().unwrap();
        let cancel = AtomicBool::new(false);
        let (csv, sr) = (dir.path().join("first.csv"), dir.path().join("first.sr"));
        let ticks = known_at as i64..(resume + 10 * p) as i64;
        export(&recording, &csv, true, ticks.clone(), &cancel).unwrap();
        export(&recording, &sr, false, ticks, &cancel).unwrap();

        let rows = csv_table(&csv);
        assert_eq!(rows[0][5..], ["dio2", "dio_gap"]);
        assert_eq!(rows[1][1], ns(known_at).to_string());
        assert_eq!(
            rows[1][5..],
            ["1", "0"],
            "the CSV's first row, known at {known_at}"
        );
        let mut zip = zip::ZipArchive::new(std::fs::File::open(sr).unwrap()).unwrap();
        let mut logic = Vec::new();
        std::io::Read::read_to_end(&mut zip.by_name("logic-1-1").unwrap(), &mut logic).unwrap();
        // GAP, DIO2, DIO_GAP, UNDER: DIO2 high and known, GAP high on the repeated pair.
        assert_eq!(
            logic[0],
            0b0010 | repeated,
            "the first sample, known at {known_at}"
        );
    }
}

/// Each export's rows, split as CSV.
fn csv_table(path: &Path) -> Vec<Vec<String>> {
    csv::ReaderBuilder::new()
        .has_headers(false)
        .flexible(true)
        .from_path(path)
        .unwrap()
        .records()
        .map(|r| r.unwrap().iter().map(str::to_owned).collect())
        .collect()
}

/// The rows, the same field for field but for times, which may differ by 1 ns: a pair's time in
/// the stream is its message's first time plus a floored offset. Returns how many rows.
fn same_csv(archive: &Path, recording: &Path, case: &str) -> usize {
    let (a, b) = (csv_table(archive), csv_table(recording));
    let times: Vec<bool> = a
        .first()
        .map(|h| h.iter().map(|c| c.contains("time")).collect())
        .unwrap_or_default();
    let mut bad = Vec::new();
    for (n, (ra, rb)) in a.iter().zip(&b).enumerate() {
        let same = ra.len() == rb.len()
            && ra.iter().zip(rb).enumerate().all(|(k, (x, y))| {
                x == y
                    || times.get(k).copied().unwrap_or(false)
                        && matches!((x.parse::<i64>(), y.parse::<i64>()), (Ok(x), Ok(y)) if (x - y).abs() <= 1)
            });
        if !same {
            bad.push(format!("row {n}: {ra:?} / {rb:?}"));
        }
    }
    assert!(
        bad.is_empty() && a.len() == b.len(),
        "{case}: {} rows against {}, {} differ:\n{}",
        a.len(),
        b.len(),
        bad.len(),
        bad.iter().take(8).cloned().collect::<Vec<_>>().join("\n")
    );
    a.len()
}

/// The same text but for integers that differ by at most 1: times in ns.
fn within_a_ns(a: &[u8], b: &[u8]) -> bool {
    let split = |t: &[u8]| {
        let text = String::from_utf8_lossy(t).into_owned();
        let mut out = Vec::new();
        let mut word = String::new();
        for ch in text.chars() {
            if ch.is_ascii_digit() != word.chars().next().is_some_and(|c| c.is_ascii_digit())
                && !word.is_empty()
            {
                out.push(std::mem::take(&mut word));
            }
            word.push(ch);
        }
        out.push(word);
        out
    };
    let (a, b) = (split(a), split(b));
    a.len() == b.len() && a.iter().zip(&b).all(|(x, y)| {
        x == y
            || matches!((x.parse::<i64>(), y.parse::<i64>()), (Ok(x), Ok(y)) if (x - y).abs() <= 1)
    })
}

/// The sessions' members, byte for byte, but for times in the text ones. Returns how many members.
fn same_sigrok(archive: &Path, recording: &Path, case: &str) -> usize {
    let read = |path: &Path| {
        let mut zip = zip::ZipArchive::new(std::fs::File::open(path).unwrap()).unwrap();
        (0..zip.len())
            .map(|i| {
                let mut member = zip.by_index(i).unwrap();
                let mut bytes = Vec::new();
                std::io::Read::read_to_end(&mut member, &mut bytes).unwrap();
                (member.name().unwrap().into_owned(), bytes)
            })
            .collect::<Vec<_>>()
    };
    let (a, b) = (read(archive), read(recording));
    let names = |m: &[(String, Vec<u8>)]| m.iter().map(|(n, _)| n.clone()).collect::<Vec<_>>();
    assert_eq!(names(&a), names(&b), "{case}: members");
    let differ: Vec<String> = a
        .iter()
        .zip(&b)
        .filter(|(x, y)| {
            x.1 != y.1 && !(x.0 == "metadata" || x.0 == "markers.csv") || !within_a_ns(&x.1, &y.1)
        })
        .map(|(x, y)| {
            let at = x.1.iter().zip(&y.1).position(|(p, q)| p != q);
            let text = |b: &[u8]| {
                if b.len() < 2000 {
                    String::from_utf8_lossy(b).into_owned()
                } else {
                    String::new()
                }
            };
            format!(
                "{} ({} against {} bytes, first at {at:?})\n{}\n---\n{}",
                x.0,
                x.1.len(),
                y.1.len(),
                text(&x.1),
                text(&y.1)
            )
        })
        .collect();
    assert!(differ.is_empty(), "{case}: {}", differ.join(", "));
    a.len()
}

/// One stream through both writers, exported whole and over selections from both, compared.
/// Returns the CSV rows, sigrok members and selections compared.
fn compare_exports(seed: u64, messages: &[Message]) -> (usize, usize, usize) {
    use escapement_tickwave_view::{Cache, Recording};
    let cancel = AtomicBool::new(false);
    let (mut rows, mut members, mut selections) = (0, 0, 0);
    let dir = tempfile::tempdir().unwrap();
    let file = encode(messages);
    let archive = archive(dir.path(), messages);
    let recording = Recording::open(std::sync::Arc::new(file), Cache::new(64 << 20)).unwrap();
    let source = &recording.sources()[0];
    let header = header_from(&source.extra).unwrap();
    let base = -source.origin;
    let half = i64::from(header.period) / 2;
    let runs = recording.runs(recording.channel("V").unwrap()).unwrap();
    let case = |t0: i64, t1: i64| {
        (
            Interval::new(nanos(t0 + base, CLOCK), nanos(t1 + base, CLOCK)),
            t0..t1,
        )
    };
    let mut cases = vec![(Interval::new(0, u64::MAX / 4), 0..i64::MAX / 4)];
    // From where no pair is, so the export opens with the pair before moved there, or with none:
    // inside a span the probe lost or paused, and before the first pair. And to inside one, so a
    // sigrok session repeats the last pair up to the end.
    let holes: Vec<i64> = (0..runs.breaks.len())
        .filter(|&k| runs.breaks[k].index > 0)
        .filter_map(|k| {
            let end = escapement_tickwave_view::runs::place_after(&runs, runs.breaks[k].index - 1);
            let at = runs.break_tick(k);
            (at > end + 1).then(|| end + (at - end) / 2)
        })
        .collect();
    if runs.count > 0 && runs.start() > 1 {
        cases.push(case(runs.start() / 2, runs.tick(runs.count / 2) + half));
    }
    for pair in holes.windows(2).take(2) {
        cases.push(case(pair[0], pair[1]));
    }
    // Between pairs, each end inside one's support.
    if runs.count > 4 {
        let mut rng = Rng(seed);
        for _ in 0..2 {
            let a = rng.below(runs.count - 2);
            let b = a + 1 + rng.below(runs.count - a - 2);
            cases.push(case(runs.tick(a) + half, runs.tick(b) + half));
        }
    }
    selections += cases.len() - 1;
    for (k, (ns, ticks)) in cases.into_iter().enumerate() {
        for csv in [true, false] {
            let name = format!(
                "seed {seed} case {k} {}",
                if csv { "csv" } else { "sigrok" }
            );
            let ext = if csv { "csv" } else { "sr" };
            let a = dir.path().join(format!("archive-{k}.{ext}"));
            let b = dir.path().join(format!("recording-{k}.{ext}"));
            archive.export(&a, csv, ns, &cancel).unwrap();
            export(&recording, &b, csv, ticks.clone(), &cancel).unwrap();
            if csv {
                rows += same_csv(&a, &b, &name);
            } else {
                members += same_sigrok(&a, &b, &name);
            }
        }
    }
    (rows, members, selections)
}

#[test]
fn exports_from_a_recording_match_the_archives() {
    let (mut rows, mut members, mut selections) = (0, 0, 0);
    let mut add = |(r, m, s): (usize, usize, usize)| {
        rows += r;
        members += m;
        selections += s;
    };
    for shape in shapes() {
        add(compare_exports(shape.seed, &stream(shape)));
    }
    // A capture whose only marker message is a loss report with no marker in it.
    let mut messages: Vec<Message> = stream(shapes()[0])
        .into_iter()
        .filter(|m| !matches!(m, Message::Markers(_)))
        .collect();
    let at = messages.len() - usize::from(matches!(messages.last(), Some(Message::End(_))));
    messages.insert(at, Message::Markers(Markers::new(Markers::LOST, vec![])));
    add(compare_exports(0, &messages));
    eprintln!("rows {rows}, sigrok members {members}, selections {selections}");
    assert!(rows > 100_000 && members > 100 && selections > 80);
}

/// A value a corrupt or foreign stream could carry: the ends of the range as often as the middle.
fn extreme(rng: &mut Rng) -> u64 {
    match rng.below(6) {
        0 => 0,
        1 => 1,
        2 => u64::MAX,
        3 => u64::MAX / 2,
        4 => i64::MAX as u64,
        _ => rng.next(),
    }
}

fn mutate(message: &mut Message, rng: &mut Rng) {
    match message {
        Message::Header(header) => {
            header.clock_hz = extreme(rng) as u32;
            header.period = extreme(rng) as u32;
            header.first_sequence = extreme(rng) as u32;
        }
        Message::Pairs(pairs) => {
            pairs.first_sequence = extreme(rng) as u32;
            pairs.time_ns = extreme(rng);
            pairs.flags = rng.next() as u8;
        }
        Message::Edges(edges) => {
            for edge in &mut edges.edges {
                if rng.one_in(2) {
                    edge.time_ns = extreme(rng);
                }
                edge.levels = rng.next() as u8;
            }
        }
        Message::DioLost(lost) => {
            lost.from_ns = extreme(rng);
            lost.to_ns = extreme(rng);
            lost.levels = rng.next() as u8;
        }
        Message::Markers(markers) => {
            markers.flags = rng.next() as u8;
            for marker in &mut markers.markers {
                marker.time_ns = extreme(rng);
                marker.number = extreme(rng) as u32;
            }
        }
        Message::End(end) => end.last_sequence = extreme(rng) as u32,
    }
}

/// Whatever a corrupt transfer, a foreign firmware or a message left from an earlier stream hands
/// the recorder comes back as an error, never a panic: random bytes of every kind through
/// `stream::parse` and on into the Encoder, and real streams with their fields set to anything,
/// through to the finish. Each failure names its seed.
#[test]
fn nothing_from_the_probe_panics_the_recorder() {
    use crate::stream::{self, parse};
    use std::panic::{AssertUnwindSafe, catch_unwind};
    let shape = |seed: u64| Shape {
        seed,
        rate: [1_000, 10_000, 100_000, 7][seed as usize % 4],
        first_sequence: if seed.is_multiple_of(5) {
            u32::MAX - 2
        } else {
            1
        },
        dio_mask: 0x0f,
        messages: 40,
        end: seed.is_multiple_of(2),
    };
    for (clock_hz, period) in [(0, 0), (0, 1), (CLOCK, 0)] {
        let mut encoder = Encoder::new(Context::default());
        let bad = Header {
            clock_hz,
            period,
            ..header(shape(1))
        };
        assert!(
            encoder.ingest(&Message::Header(bad)).is_err(),
            "clock {clock_hz}, period {period}"
        );
    }
    // Stamps the pairs can't account for are refused rather than placed, and their neighbours
    // just inside the bounds are taken.
    let fresh = || {
        let mut encoder = Encoder::new(Context::default());
        encoder.ingest(&Message::Header(header(shape(1)))).unwrap();
        encoder
    };
    let period = u64::from(header(shape(1)).period);
    let minute = 60 * u64::from(CLOCK);
    let pairs_at = |first_sequence: u32, at: u64| {
        Message::Pairs(Pairs {
            flags: 0,
            first_sequence,
            time_ns: ns(at),
            pairs: vec![Pair {
                voltage_code: 1000,
                current_code: 1000,
                low: true,
                under_range: false,
            }],
        })
    };
    let edge_at = |at: u64| {
        Message::Edges(stream::Edges {
            flags: 0,
            edges: vec![stream::Edge {
                time_ns: ns(at),
                levels: 1,
            }],
        })
    };
    assert!(
        fresh().ingest(&pairs_at(1, minute + 1)).is_err(),
        "first pairs past a minute"
    );
    assert!(
        fresh().ingest(&pairs_at(1, minute)).is_ok(),
        "first pairs at a minute"
    );
    let mut encoder = fresh();
    encoder.ingest(&pairs_at(1, 0)).unwrap();
    assert!(
        encoder.ingest(&pairs_at(2, period - 1)).is_err(),
        "pairs before their sequence"
    );
    let mut encoder = fresh();
    encoder.ingest(&pairs_at(1, 0)).unwrap();
    assert!(
        encoder.ingest(&pairs_at(5, 4 * period + minute)).is_ok(),
        "pairs after a loss and a pause"
    );
    let mut encoder = fresh();
    encoder.ingest(&pairs_at(1, 0)).unwrap();
    assert!(
        encoder
            .ingest(&pairs_at(5, 4 * period + minute + 1))
            .is_err(),
        "pairs past a loss and a minute"
    );
    let mut encoder = fresh();
    encoder.ingest(&pairs_at(1, 0)).unwrap();
    assert!(
        encoder.ingest(&edge_at(minute + 1)).is_err(),
        "an edge past a minute ahead"
    );
    let mut encoder = fresh();
    encoder.ingest(&pairs_at(1, 0)).unwrap();
    assert!(
        encoder.ingest(&edge_at(minute)).is_ok(),
        "an edge a minute ahead"
    );
    let kinds = [
        stream::PAIRS,
        stream::HEADER,
        stream::END,
        stream::EDGES,
        stream::MARKS,
        stream::DIO_LOST,
        0x00,
        0xff,
    ];
    let mut parsed = 0;
    for seed in 1..=300u64 {
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1);
            let mut encoder = Encoder::new(Context::default());
            let _ = encoder.ingest(&Message::Header(header(shape(seed))));
            let mut parsed = 0;
            for _ in 0..200 {
                let len = rng.below(600) as usize;
                let mut bytes: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
                if let Some(kind) = bytes.first_mut() {
                    *kind = kinds[rng.below(kinds.len() as u64) as usize];
                }
                // Counts that fit the length, half the time, so whole messages get through.
                if rng.one_in(2) {
                    match bytes.first() {
                        Some(&stream::PAIRS) if len >= 16 => {
                            let k = ((len - 16) / 5).clamp(1, 99) as u16;
                            bytes[14..16].copy_from_slice(&k.to_le_bytes());
                        }
                        Some(&stream::EDGES) if len >= 12 => {
                            let e = ((len - 12) / 5).clamp(1, 99) as u16;
                            bytes[10..12].copy_from_slice(&e.to_le_bytes());
                        }
                        Some(&stream::HEADER) if len >= 2 => bytes[1] = 3 + rng.below(2) as u8,
                        _ => {}
                    }
                }
                if let Ok(message) = parse(&bytes) {
                    parsed += 1;
                    let _ = encoder.ingest(&message);
                    let _ = encoder.take_output();
                }
            }
            let _ = encoder.finish("random");
            parsed
        }));
        match outcome {
            Ok(n) => parsed += n,
            Err(_) => panic!("random bytes, seed {seed}"),
        }
    }
    assert!(parsed > 10_000, "only {parsed} random messages parsed");
    for seed in 1..=200u64 {
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut rng = Rng(seed.wrapping_mul(0xbf58_476d_1ce4_e5b9) | 1);
            let mut messages = stream(shape(seed));
            for message in messages.iter_mut().skip(1) {
                if rng.one_in(4) {
                    mutate(message, &mut rng);
                }
            }
            if rng.one_in(3) && messages.len() > 2 {
                let (a, b) = (
                    rng.below(messages.len() as u64),
                    rng.below(messages.len() as u64),
                );
                messages.swap(a as usize, b as usize);
            }
            let mut encoder = Encoder::new(Context::default());
            for message in &messages {
                let _ = encoder.ingest(message);
                let _ = encoder.take_output();
            }
            let _ = encoder.finish("mutated");
        }));
        assert!(outcome.is_ok(), "mutated stream, seed {seed}");
    }
}
