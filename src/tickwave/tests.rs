//! Streams through the encoder and the file writer, and what the recording, its recovery and its
//! exports hold.

use super::*;
use crate::stream::{DioHeader, DioLost, DioTime, Edge, Edges, End, Marker, Pair};
use escapement_tickwave_format::decode::{BlockChannel, Footer, Index, from_memory};
use escapement_tickwave_format::header::ChannelKind;
use escapement_tickwave_format::model::{Break, Grid};
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
            time_ns: None,
        }));
    }
    out
}

/// A DIO stream as the probe sends one (`0x90` mode 15): edges, some at one time, and loss reports
/// on the pins of `mask`, in time order, then an end past the last.
fn dio_stream(seed: u64, mask: u8, messages: usize) -> Vec<Message> {
    let clock = 150_000_000;
    let ns = |count: u64| (u128::from(count) * 1_000_000_000 / u128::from(clock)) as u64;
    let mut rng = Rng(seed | 1);
    let mut levels = 0b0110 & mask;
    let mut out = vec![Message::DioHeader(DioHeader {
        clock_hz: clock,
        dio_mask: mask,
        dio_levels: levels,
        edge_resolution_ns: 7,
    })];
    let mut at = 0;
    for _ in 0..messages {
        at += 1 + rng.below(1 << 24);
        if rng.one_in(8) {
            let to = at + 1 + rng.below(1 << 20);
            levels = rng.below(16) as u8 & mask;
            out.push(Message::DioLost(DioLost {
                from_ns: ns(at),
                to_ns: ns(to),
                levels,
            }));
            at = to;
            continue;
        }
        let mut edges = Vec::new();
        for _ in 0..1 + rng.below(20) {
            let pin = 1 << rng.below(4);
            if pin & mask != 0 {
                levels ^= pin;
                edges.push(Edge {
                    time_ns: ns(at),
                    levels,
                });
            }
            at += rng.below(1 << 16);
        }
        if !edges.is_empty() {
            out.push(Message::Edges(Edges::new(0, edges)));
        }
        // A quiet stretch: the probe says how far it has got, after everything it sent.
        if rng.one_in(3) {
            at += 1 + rng.below(1 << 24);
            out.push(Message::DioTime(DioTime { time_ns: ns(at) }));
        }
    }
    out.push(Message::End(End {
        reason: EndReason::Asked,
        last_sequence: 0,
        time_ns: Some(ns(at + rng.below(1 << 24))),
    }));
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
    /// Each digital channel's gaps, inline or paged.
    gaps: Vec<Vec<Gap>>,
    /// Each analog channel's breaks, inline or paged.
    breaks: Vec<Vec<Break>>,
    /// Each channel's blocks.
    blocks: Vec<usize>,
    annotations: Vec<serde_json::Map<String, serde_json::Value>>,
}

/// Read block by block through the format crate, apart from the view crate the export reads with.
fn decode(file: &[u8]) -> Decoded {
    let index = Index::of_file(file).unwrap();
    let n = index.header().channels.len();
    let (mut edges, mut codes) = (vec![Vec::new(); n], vec![Vec::new(); n]);
    for c in 0..n {
        for block in index.channel_blocks(file, c).unwrap() {
            match &block.data {
                BlockChannel::Digital { .. } => {
                    edges[c].extend_from_slice(block.data.source_ticks().unwrap());
                }
                BlockChannel::Analog { values, .. } => {
                    codes[c].extend(values.iter().map(|sample| match sample {
                        escapement_tickwave_format::model::Sample::U64(code) => code,
                        other => panic!("{other:?}"),
                    }))
                }
            }
        }
    }
    let footer = &index.footer;
    let gaps = (0..n)
        .map(|c| footer.gaps(c).unwrap_or_default().to_vec())
        .collect();
    let breaks = (0..n)
        .map(|c| footer.breaks(c).unwrap_or_default().to_vec())
        .collect();
    let mut annotations = Vec::new();
    let sidecar = footer.read_sidecar(from_memory(file)).unwrap();
    if let Some(sidecar) = &sidecar {
        assert_eq!(
            sidecar.left_out,
            Vec::<String>::new(),
            "annotations left out"
        );
    }
    for (name, text) in sidecar.iter().flat_map(|s| &s.files) {
        if name.ends_with(".jsonl") {
            for line in text.lines().skip(1) {
                annotations.push(serde_json::from_str(line).unwrap());
            }
        }
    }
    Decoded {
        header: footer.header.clone(),
        edges,
        codes,
        gaps,
        breaks,
        blocks: index.blocks.iter().map(Vec::len).collect(),
        annotations,
    }
}

/// A digital channel's level at `tick`, from its initial level, edges and gaps; none inside a gap.
fn level_at(d: &Decoded, c: usize, tick: i64) -> Option<bool> {
    let ChannelKind::Digital { initial, .. } = &d.header.channels[c].data else {
        panic!("digital");
    };
    let mut level = *initial;
    let mut gaps = d.gaps[c].iter().peekable();
    let mut edges = d.edges[c].iter().peekable();
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

/// The view a finished file stores, as a viewer elsewhere reads it.
fn stored_view(file: Vec<u8>) -> Option<View> {
    use escapement_tickwave_view::{Cache, Recording};
    Recording::open(std::sync::Arc::new(file), Cache::new(1 << 20))
        .unwrap()
        .stored_view()
        .unwrap()
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
fn content(file: &[u8]) -> (Vec<(i64, u64, u64)>, Vec<i64>, usize) {
    let d = decode(file);
    let ChannelKind::Analog { first, period, .. } = &d.header.channels[V].data else {
        panic!("analog");
    };
    let breaks = &d.breaks[V];
    let samples = (0..d.codes[V].len())
        .map(|i| {
            let at = Grid::new(*period, None)
                .unwrap()
                .sample_tick(*first, breaks, i as u64);
            (at, d.codes[V][i], d.codes[I][i])
        })
        .collect();
    let mut markers: Vec<i64> = d
        .annotations
        .iter()
        .map(|a| a["start"].as_i64().unwrap())
        .collect();
    markers.sort_unstable();
    (samples, markers, d.blocks[V])
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
fn a_file_writer_publishes_again_only_once_the_view_took_the_last_snapshot() {
    let messages = stream(shapes()[13]);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.tickwave");
    let mut writer = FileWriter::create(&path, Context::default())
        .unwrap()
        .flushing_always();
    let slot = writer.snapshots();
    let through = || {
        slot.lock()
            .unwrap()
            .as_ref()
            .map(|p| p.snapshot.known_through())
    };
    let before = Instant::now();
    let mut rest = messages.iter();
    while through().is_none() {
        writer.ingest(rest.next().unwrap()).unwrap();
    }
    let first = through().unwrap();
    assert!(slot.lock().unwrap().as_ref().unwrap().at >= before);
    let (last, untaken) = rest.as_slice().split_last().unwrap();
    assert!(untaken.len() >= 2, "{} messages left", untaken.len());
    for message in untaken {
        writer.ingest(message).unwrap();
    }
    assert_eq!(through(), Some(first), "refilled before the view took it");

    slot.lock().unwrap().take();
    let taken = Instant::now();
    writer.ingest(last).unwrap();
    let next = slot
        .lock()
        .unwrap()
        .take()
        .expect("a snapshot once the last was taken");
    assert!(next.snapshot.known_through() > first);
    assert!(next.at >= taken);
    // The end it carries is where a view of that snapshot sees the data end.
    let bytes = std::sync::Arc::new(std::fs::read(&path).unwrap());
    let view = escapement_tickwave_view::Recording::live(
        next.snapshot,
        bytes,
        escapement_tickwave_view::Cache::new(64 << 20),
    )
    .unwrap();
    assert_eq!(next.end, view.extent().map(|e| e.end));
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
    // Each channel's overview follows its own rate: the flags change far less often than pairs
    // come, so theirs is coarser.
    let header = Footer::parse_file(&compacted).unwrap().header;
    let shifts = shifts_for(&header);
    for (c, &shift) in shifts.iter().enumerate() {
        assert_eq!(header.shift(c), Some(shift), "channel {c}");
    }
    assert!(shifts[LOW_RANGE] > shifts[V], "{shifts:?}");
    let (whole_samples, whole_markers, _) = content(&whole);
    assert_eq!((samples, markers), (whole_samples, whole_markers));
    assert_eq!(stored_view(compacted), Some(verge_view()));
    // Nothing is left beside it.
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}

#[test]
fn a_closed_recording_is_compacted_beside_itself_and_replaced_only_on_persist() {
    let messages = stream(shapes()[13]);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.tickwave");
    let mut writer = FileWriter::create(&path, Context::default())
        .unwrap()
        .flushing_always();
    for message in &messages {
        writer.ingest(message).unwrap();
    }
    assert!(writer.close().unwrap(), "a recording is left");
    let closed = std::fs::read(&path).unwrap();
    assert!(content(&closed).2 > 20, "left uncompacted");

    let copy = compacted(&path).unwrap();
    assert_eq!(
        std::fs::read(&path).unwrap(),
        closed,
        "the recording is untouched until persist"
    );
    assert_eq!(
        std::fs::read_dir(dir.path()).unwrap().count(),
        2,
        "the copy is beside it"
    );
    copy.persist().unwrap();
    let persisted = std::fs::read(&path).unwrap();
    let whole = encode(&messages);
    assert_eq!(
        Footer::parse_file(&persisted).unwrap().header.content_id,
        Footer::parse_file(&whole).unwrap().header.content_id
    );
    assert!(
        content(&persisted).2 < 5,
        "compacted: {}",
        content(&persisted).2
    );
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);

    // A copy dropped unpersisted leaves the recording as it was and nothing beside it.
    drop(compacted(&path).unwrap());
    assert_eq!(std::fs::read(&path).unwrap(), persisted);
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}

#[test]
fn closing_a_recording_whose_header_never_came_leaves_no_file() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.tickwave");
    let writer = FileWriter::create(&path, Context::default()).unwrap();
    assert!(!writer.close().unwrap());
    assert!(!path.exists());
}

#[test]
fn a_stalled_streams_last_blocks_reach_the_file_on_the_recorders_clock() {
    let messages = stream(Shape {
        seed: 5,
        rate: 100_000,
        first_sequence: 1,
        dio_mask: 0,
        messages: 400,
        end: false,
    });
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.tickwave");
    let mut writer = FileWriter::create(&path, Context::default()).unwrap();
    writer.flush_every = Duration::MAX;
    for message in &messages {
        writer.ingest(message).unwrap();
    }
    // The stream stalls with more in its open blocks than one call compresses.
    writer.flush_every = Duration::ZERO;
    writer.write().unwrap();
    assert!(writer.encoder.pending());
    writer.flush().unwrap();
    assert!(!writer.encoder.pending());
}

#[test]
fn an_end_reporting_more_lost_pairs_than_a_recording_holds_is_refused_and_the_rest_kept() {
    let messages = stream(Shape {
        seed: 9,
        rate: 7,
        first_sequence: 1,
        dio_mask: 0,
        messages: 5,
        end: false,
    });
    let next = messages
        .iter()
        .rev()
        .find_map(|m| match m {
            Message::Pairs(p) => Some(p.first_sequence.wrapping_add(p.pairs.len() as u32)),
            _ => None,
        })
        .unwrap();
    let mut encoder = Encoder::new(Context::default());
    for message in &messages {
        encoder.ingest(message).unwrap();
    }
    // 2^31 − 1 pairs at 7 a second are ten years: past what the axis holds at the probe's clock.
    let end = Message::End(End {
        reason: EndReason::Stopped,
        last_sequence: next.wrapping_add(0x7fff_fffe),
        time_ns: None,
    });
    assert!(matches!(
        encoder.ingest(&end),
        Err(Error::LostAfter(0x7fff_ffff))
    ));
    let finished = encoder.finish("capture").unwrap().unwrap();
    let ChannelKind::Analog {
        count, lost_after, ..
    } = finished.header.channels[V].data
    else {
        panic!("analog");
    };
    assert!(count > 0);
    assert_eq!(lost_after, 0);
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
                time_ns: None,
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

/// A file whose marker bundle doesn't resolve still opens, and its export is refused rather than
/// written without the markers: over the pairs, and over an interval that meets nothing else,
/// where the markers it can't place might be.
#[test]
fn an_export_of_a_file_whose_markers_dont_resolve_is_refused() {
    use escapement_tickwave_view::{Cache, Recording};
    let header = header(Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 1,
        dio_mask: 0,
        messages: 1,
        end: true,
    });
    let p = u64::from(header.period);
    let pair = Pair {
        voltage_code: 30_000,
        current_code: 1_000,
        low: true,
        under_range: false,
    };
    let marker = Marker {
        time_ns: ns(3 * p),
        number: 1,
        label: b"m".to_vec(),
    };
    let messages = [
        Message::Header(header),
        Message::Pairs(Pairs::new(0, 1, ns(0), vec![pair; 10])),
        Message::Markers(Markers::new(0, vec![marker])),
        Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: 10,
            time_ns: None,
        }),
    ];
    let mut file = encode(&messages);
    let (at, len) = Index::of_file(&file).unwrap().footer.bundle.unwrap();
    file[(at + u64::from(len) / 2) as usize] ^= 0xff;
    let recording = Recording::open(std::sync::Arc::new(file), Cache::new(64 << 20)).unwrap();
    assert!(recording.has_markers());
    let dir = tempfile::tempdir().unwrap();
    let cancel = AtomicBool::new(false);
    for (name, csv) in [("markers.csv", true), ("markers.sr", false)] {
        for interval in [0..i64::MAX / 4, i64::MIN / 4..0] {
            let path = dir.path().join(name);
            let error = export(&recording, &path, csv, interval.clone(), &cancel).unwrap_err();
            assert!(
                matches!(
                    error,
                    Error::View(escapement_tickwave_view::Error::Annotations(_))
                ),
                "{name} {interval:?}: {error}"
            );
            assert!(!path.exists(), "{name} {interval:?}");
        }
    }
}

/// An export reads the marker parts its interval meets and no other: one over a part that doesn't
/// read is refused, and one past it is written.
#[test]
fn an_export_reads_only_the_marker_parts_it_meets() {
    use escapement_tickwave_view::{Cache, Recording};
    let header = header(Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 1,
        dio_mask: 0,
        messages: 1,
        end: true,
    });
    let p = u64::from(header.period);
    let pair = Pair {
        voltage_code: 30_000,
        current_code: 1_000,
        low: true,
        under_range: false,
    };
    let marker = Marker {
        time_ns: ns(3 * p),
        number: 1,
        label: b"m".to_vec(),
    };
    let messages = [
        Message::Header(header),
        Message::Pairs(Pairs::new(0, 1, ns(0), vec![pair; 10])),
        Message::Markers(Markers::new(0, vec![marker])),
        Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: 10,
            time_ns: None,
        }),
    ];
    let mut file = encode(&messages);
    let footer = Index::of_file(&file).unwrap().footer;
    let rows = footer.part_rows(from_memory(&file)).unwrap();
    let [row] = rows[..] else { panic!("{rows:?}") };
    file[(row.offset + u64::from(row.length) / 2) as usize] ^= 0xff;
    let recording = Recording::open(std::sync::Arc::new(file), Cache::new(64 << 20)).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let cancel = AtomicBool::new(false);
    for (name, csv) in [("markers.csv", true), ("markers.sr", false)] {
        let path = dir.path().join(format!("over-{name}"));
        let error = export(&recording, &path, csv, 0..i64::MAX / 4, &cancel).unwrap_err();
        assert!(matches!(error, Error::View(_)), "{name}: {error}");
        assert!(!path.exists(), "{name}");
        let path = dir.path().join(format!("past-{name}"));
        export(&recording, &path, csv, row.end + 1..i64::MAX / 4, &cancel).unwrap();
    }
}

/// An export reaches past the pairs for what the recording holds there: a marker before the
/// first pair, and the pins of a recording with no pair.
#[test]
fn an_export_reaches_the_markers_and_pins_outside_the_pairs() {
    use escapement_tickwave_view::{Cache, Recording};
    let shape = |dio_mask| Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 1,
        dio_mask,
        messages: 1,
        end: true,
    };
    let p = u64::from(header(shape(0)).period);
    let pair = Pair {
        voltage_code: 30_000,
        current_code: 1_000,
        low: true,
        under_range: false,
    };
    let marker = Marker {
        time_ns: ns(2 * p),
        number: 7,
        label: b"early".to_vec(),
    };
    let early = [
        Message::Header(header(shape(0))),
        Message::Markers(Markers::new(0, vec![marker])),
        // The first four pairs lost.
        Message::Pairs(Pairs::new(0, 5, ns(4 * p), vec![pair; 10])),
        Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: 14,
            time_ns: None,
        }),
    ];
    let unpaired = [
        Message::Header(header(shape(0b0001))),
        Message::Edges(Edges::new_at(vec![(5 * p, 0)])),
        Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: 0,
            time_ns: None,
        }),
    ];
    let dir = tempfile::tempdir().unwrap();
    let cancel = AtomicBool::new(false);
    let rows = |name: &str, messages: &[Message]| {
        let recording =
            Recording::open(std::sync::Arc::new(encode(messages)), Cache::new(64 << 20)).unwrap();
        let path = dir.path().join(name);
        export(&recording, &path, true, 0..i64::MAX / 4, &cancel).unwrap();
        csv_table(&path)
    };
    let early = rows("early.csv", &early);
    assert_eq!(early[1][1..], ["2000000", "", "", "", "7", "early", "0"]);
    assert_eq!(early[2][..2], ["5", "4000000"]);
    let unpaired = rows("unpaired.csv", &unpaired);
    assert_eq!(unpaired[1][1..], ["0", "", "", "", "1", "0"]);
    assert_eq!(unpaired[2][1], ns(5 * p).to_string());
    assert_eq!(unpaired[2][2..], ["", "", "", "0", "0"]);
    assert_eq!(unpaired.len(), 3);
}

/// A value a corrupt or foreign stream could carry: the ends of the range as often as the middle,
/// and a time of about 1.9 years, near the furthest a recording takes (2^53 ticks at 150 MHz).
fn extreme(rng: &mut Rng) -> u64 {
    match rng.below(7) {
        0 => 0,
        1 => 1,
        2 => u64::MAX,
        3 => u64::MAX / 2,
        4 => i64::MAX as u64,
        5 => 60_000_000_000_000_000,
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
        Message::End(end) => {
            end.last_sequence = extreme(rng) as u32;
            end.time_ns = rng.one_in(2).then(|| extreme(rng));
        }
        Message::DioHeader(header) => {
            header.clock_hz = extreme(rng) as u32;
            header.dio_mask = rng.next() as u8;
            header.dio_levels = rng.next() as u8;
        }
        Message::DioTime(time) => time.time_ns = extreme(rng),
    }
}

/// Whatever a corrupt transfer, a foreign firmware or a message left from an earlier stream hands
/// the recorder comes back as an error, never a panic or a hang: random bytes of every kind through
/// `stream::parse` and on into the Encoder after a capture's or a DIO stream's header, and real
/// streams of both kinds with their fields, headers included, set to anything, through to the
/// finish. Each failure names its seed, as does a case that runs too long.
#[test]
fn nothing_from_the_probe_panics_the_recorder() {
    use crate::stream::{self, parse};
    use std::panic::{AssertUnwindSafe, catch_unwind};
    // Far past what a case takes, about 7 ms; the hang a5d9514 fixed took hours.
    const CASE_LIMIT: Duration = Duration::from_secs(5);
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
    // A DIO stream has no pairs to bound its stamps by. The furthest edge a recording can end
    // after, with its length one past it below 2^53 ticks, finishes as fast as a near one.
    let started = Instant::now();
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&dio_stream(1, 0x0f, 0)[0]).unwrap();
    let far = ((1u128 << 53) - 2) * 1_000_000_000 / 150_000_000;
    encoder
        .ingest(&Message::Edges(Edges::new(
            0,
            vec![Edge {
                time_ns: far as u64,
                levels: 1,
            }],
        )))
        .unwrap();
    encoder.flush().unwrap();
    encoder.finish("far").unwrap();
    assert!(started.elapsed() < CASE_LIMIT, "{:?}", started.elapsed());
    // A tick further on is refused as it comes, edge or DIO time, and what came before finishes.
    let past = (((1u128 << 53) - 1) * 1_000_000_000 / 150_000_000) as u64;
    for message in [
        Message::Edges(Edges::new(
            0,
            vec![Edge {
                time_ns: past,
                levels: 1,
            }],
        )),
        Message::DioTime(DioTime { time_ns: past }),
    ] {
        let mut encoder = Encoder::new(Context::default());
        encoder.ingest(&dio_stream(1, 0x0f, 0)[0]).unwrap();
        encoder
            .ingest(&Message::DioTime(DioTime { time_ns: 1_000 }))
            .unwrap();
        assert!(matches!(encoder.ingest(&message), Err(Error::Timestamps)));
        encoder.flush().unwrap();
        encoder.finish("past").unwrap();
    }
    let kinds = [
        stream::PAIRS,
        stream::HEADER,
        stream::END,
        stream::EDGES,
        stream::MARKS,
        stream::DIO_LOST,
        stream::DIO_TIME,
        0x00,
        0xff,
    ];
    let mut parsed = 0;
    for seed in 1..=300u64 {
        let started = Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1);
            let mut encoder = Encoder::new(Context::default());
            let first = if seed.is_multiple_of(2) {
                Message::Header(header(shape(seed)))
            } else {
                dio_stream(seed, 0x0f, 0).swap_remove(0)
            };
            let _ = encoder.ingest(&first);
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
                        Some(&stream::HEADER) if len >= 2 => bytes[1] = 3 + rng.below(3) as u8,
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
        let took = started.elapsed();
        assert!(took < CASE_LIMIT, "random bytes, seed {seed}: {took:?}");
    }
    assert!(parsed > 10_000, "only {parsed} random messages parsed");
    for seed in 1..=300u64 {
        let started = Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut rng = Rng(seed.wrapping_mul(0xbf58_476d_1ce4_e5b9) | 1);
            let mut messages = if seed.is_multiple_of(3) {
                dio_stream(seed, [0x0f, 0x01, 0x0a][(seed / 3) as usize % 3], 40)
            } else {
                stream(shape(seed))
            };
            // The header less often, so that most streams get past it.
            for (i, message) in messages.iter_mut().enumerate() {
                if rng.one_in(if i == 0 { 8 } else { 4 }) {
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
                let _ = encoder.flush();
            }
            let _ = encoder.finish("mutated");
        }));
        assert!(outcome.is_ok(), "mutated stream, seed {seed}");
        let took = started.elapsed();
        assert!(took < CASE_LIMIT, "mutated stream, seed {seed}: {took:?}");
    }
}

/// A DIO stream's recording holds its pins alone, under `verge-dio`: no V, I, flags or power,
/// each pin's edges where the stream put them, a loss as a gap on every pin, and a length that
/// reaches the end message's time past the last edge. The FileWriter's compaction keeps it.
#[test]
fn a_dio_streams_recording_holds_its_pins_alone() {
    let clock = 150_000_000;
    let ns = |count: u64| (u128::from(count) * 1_000_000_000 / u128::from(clock)) as u64;
    let edges = |at: u64, levels: u8| {
        Message::Edges(Edges::new(
            0,
            vec![Edge {
                time_ns: ns(at),
                levels,
            }],
        ))
    };
    let messages = vec![
        Message::DioHeader(DioHeader {
            clock_hz: clock,
            dio_mask: 0b0101,
            dio_levels: 0b1001,
            edge_resolution_ns: 7,
        }),
        // DIO0 falls and DIO2 rises; then DIO0 rises; DIO1 and DIO3 are not reported.
        edges(1000, 0b0110),
        edges(2000, 0b0101),
        Message::DioLost(DioLost {
            from_ns: ns(3000),
            to_ns: ns(4000),
            levels: 0b0000,
        }),
        edges(5000, 0b0001),
        Message::End(End {
            reason: EndReason::Asked,
            last_sequence: 0,
            time_ns: Some(ns(10_000)),
        }),
    ];
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("dio.tickwave");
    write_with_file_writer(&path, &messages, true);
    for file in [encode(&messages), std::fs::read(&path).unwrap()] {
        let d = decode(&file);
        let names: Vec<&str> = d.header.channels.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["DIO0", "DIO2"]);
        assert!(d.header.derived.is_empty());
        assert_eq!(
            d.header.sources[0].format,
            SourceFormat::Other(DIO_FORMAT.into())
        );
        assert_eq!(d.header.length, 10_000);
        assert_eq!(d.edges, [vec![1000, 2000, 5000], vec![1000]]);
        let lost = Gap {
            from: 3000,
            to: 4000,
            level: false,
        };
        assert_eq!(d.gaps, [vec![lost], vec![lost]]);
        assert_eq!(level_at(&d, 0, 0), Some(true));
        assert_eq!(level_at(&d, 1, 9_999), Some(false));
        assert_eq!(stored_view(file), Some(verge_view()));
    }
}

/// A DIO stream reports at least one pin, sends no pairs, and its times never go back.
#[test]
fn a_dio_stream_refuses_what_it_cannot_record() {
    let header = |mask| {
        Message::DioHeader(DioHeader {
            clock_hz: 150_000_000,
            dio_mask: mask,
            dio_levels: 0,
            edge_resolution_ns: 7,
        })
    };
    let mut encoder = Encoder::new(Context::default());
    assert!(matches!(encoder.ingest(&header(0)), Err(Error::NoPins)));
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&header(1)).unwrap();
    assert!(matches!(
        encoder.ingest(&Message::Pairs(Pairs {
            flags: 0,
            first_sequence: 1,
            time_ns: 0,
            pairs: vec![Pair {
                voltage_code: 0,
                current_code: 0,
                low: false,
                under_range: false
            }],
        })),
        Err(Error::DioPairs)
    ));
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&header(1)).unwrap();
    let edge = |ns, levels| {
        Message::Edges(Edges::new(
            0,
            vec![Edge {
                time_ns: ns,
                levels,
            }],
        ))
    };
    encoder.ingest(&edge(2_000, 1)).unwrap();
    assert!(matches!(
        encoder.ingest(&edge(1_000, 0)),
        Err(Error::Timestamps)
    ));
    // Edges stamped together come in one message, so a later one can't share its time.
    assert!(matches!(
        encoder.ingest(&edge(2_000, 0)),
        Err(Error::Timestamps)
    ));
    assert!(matches!(encoder.ingest(&header(1)), Err(Error::Restarted)));
    // A DIO time comes after everything sent; an edge can come at it.
    let time = |ns| Message::DioTime(DioTime { time_ns: ns });
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&header(1)).unwrap();
    encoder.ingest(&edge(2_000, 1)).unwrap();
    assert!(matches!(
        encoder.ingest(&time(2_000)),
        Err(Error::Timestamps)
    ));
    let mut encoder = Encoder::new(Context::default());
    encoder.ingest(&header(1)).unwrap();
    encoder.ingest(&edge(2_000, 1)).unwrap();
    // 2,001 ns is the tick after the edge's: the promise.
    encoder.ingest(&time(2_001)).unwrap();
    encoder.ingest(&time(2_007)).unwrap();
    encoder.ingest(&edge(2_007, 0)).unwrap();
    assert!(matches!(
        encoder.ingest(&time(2_000)),
        Err(Error::Timestamps)
    ));
    // A loss that spans nothing gives the levels from its time on, and an edge after it comes
    // later: 7 ns is one count.
    for (edge_ns, taken) in [(3_000, false), (3_007, true)] {
        let mut encoder = Encoder::new(Context::default());
        encoder.ingest(&header(1)).unwrap();
        encoder
            .ingest(&Message::DioLost(DioLost {
                from_ns: 3_000,
                to_ns: 3_000,
                levels: 1,
            }))
            .unwrap();
        let outcome = encoder.ingest(&edge(edge_ns, 0));
        assert_eq!(outcome.is_ok(), taken, "edge at {edge_ns} ns: {outcome:?}");
        assert!(taken || matches!(outcome, Err(Error::Timestamps)));
    }
}

/// A crash keeps every edge and loss the stream sent before the last flush, the latest included
/// however long the pins stay quiet after it, and the quiet the probe's last DIO time covers; a file
/// cut anywhere recovers to a prefix.
#[test]
fn a_dio_recording_cut_off_keeps_its_latest_edge_and_recovers_to_a_prefix() {
    #[derive(Clone, Copy, PartialEq)]
    enum Last {
        Edges,
        Loss,
        Quiet,
    }
    for (seed, mask, last) in [
        (3, 0x0f, Last::Edges),
        (8, 0x0a, Last::Loss),
        (5, 0x0f, Last::Quiet),
    ] {
        let mut messages = dio_stream(seed, mask, 40);
        // The recorder stops without an end, after its clock's flush.
        messages.pop();
        let after = messages
            .iter()
            .filter_map(|m| match m {
                Message::Edges(e) => e.edges.last().map(|e| e.time_ns),
                Message::DioLost(l) => Some(l.to_ns),
                Message::DioTime(t) => Some(t.time_ns),
                _ => None,
            })
            .max()
            .unwrap();
        messages.retain(|m| last != Last::Edges || !matches!(m, Message::DioTime(_)));
        match last {
            Last::Edges => {}
            Last::Loss => messages.push(Message::DioLost(DioLost {
                from_ns: after + 1_000,
                to_ns: after + 2_000,
                levels: 0b0010,
            })),
            // Five seconds of quiet pins.
            Last::Quiet => messages.push(Message::DioTime(DioTime {
                time_ns: after + 5_000_000_000,
            })),
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("dio.tickwave");
        let mut writer = FileWriter::create(&path, Context::default())
            .unwrap()
            .flushing_always();
        for message in &messages {
            writer.ingest(message).unwrap();
        }
        writer.flush().unwrap();
        drop(writer);
        let bytes = std::fs::read(&path).unwrap();
        let mut whole = messages.clone();
        whole.push(Message::End(End {
            reason: EndReason::Asked,
            last_sequence: 0,
            time_ns: None,
        }));
        let all = decode(&encode(&whole));
        compact(&path).unwrap();
        let recovered = decode(&std::fs::read(&path).unwrap());
        assert_eq!(recovered.edges, all.edges, "seed {seed}");
        assert_eq!(recovered.gaps, all.gaps, "seed {seed}");
        if last == Last::Quiet {
            let quiet_to = (after + 5_000_000_000) as i64 * 150 / 1000;
            assert!(
                recovered.header.length >= quiet_to,
                "seed {seed}: {}",
                recovered.header.length
            );
        }
        for cut in (1..16).map(|k| bytes.len() * k / 16) {
            let path = dir.path().join(format!("cut-{cut}.tickwave"));
            std::fs::write(&path, &bytes[..cut]).unwrap();
            match compact(&path) {
                Ok(()) => {}
                // Too little for a declaration, or nothing after it.
                Err(Error::Recut(_)) if cut < bytes.len() / 4 => continue,
                Err(error) => panic!("seed {seed}, cut at {cut} of {}: {error}", bytes.len()),
            }
            let cut_off = decode(&std::fs::read(&path).unwrap());
            for (c, edges) in cut_off.edges.iter().enumerate() {
                assert_eq!(
                    edges[..],
                    all.edges[c][..edges.len()],
                    "seed {seed}, cut at {cut}"
                );
                let gaps = &cut_off.gaps[c];
                assert_eq!(
                    gaps[..],
                    all.gaps[c][..gaps.len()],
                    "seed {seed}, cut at {cut}"
                );
            }
        }
    }
}

/// A crash before the first edge leaves a file with no block, which recovers to the probe's last
/// DIO time before the last flush, every pin at its starting level.
#[test]
fn a_dio_recording_cut_off_before_its_first_edge_recovers_to_its_last_dio_time() {
    let (mask, levels) = (0x0f, 0b0110);
    let last_ns: u64 = 2_600_000_000;
    let messages = [
        Message::DioHeader(DioHeader {
            clock_hz: 150_000_000,
            dio_mask: mask,
            dio_levels: levels,
            edge_resolution_ns: 7,
        }),
        Message::DioTime(DioTime {
            time_ns: 200_000_000,
        }),
        Message::DioTime(DioTime { time_ns: last_ns }),
    ];
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("quiet.tickwave");
    let mut writer = FileWriter::create(&path, Context::default())
        .unwrap()
        .flushing_always();
    for message in &messages {
        writer.ingest(message).unwrap();
    }
    writer.flush().unwrap();
    drop(writer);
    compact(&path).unwrap();
    let recovered = decode(&std::fs::read(&path).unwrap());
    let last = last_ns as i64 * 150 / 1000;
    assert!(
        recovered.header.length >= last,
        "{}",
        recovered.header.length
    );
    let mut pins = 0;
    for (c, channel) in recovered.header.channels.iter().enumerate() {
        let Some(pin) = channel.name.strip_prefix("DIO") else {
            continue;
        };
        let pin: u32 = pin.parse().unwrap();
        assert!(recovered.edges[c].is_empty(), "{}", channel.name);
        assert_eq!(
            level_at(&recovered, c, last - 1),
            Some(levels >> pin & 1 == 1),
            "{}",
            channel.name
        );
        pins += 1;
    }
    assert_eq!(pins, 4);
}

/// A crash while the finish is being written leaves a file cut inside its tail: past every block
/// and record the recording wrote while it ran.
#[test]
fn a_recording_cut_inside_its_finish_recovers() {
    for k in [2, 13, 22, 33] {
        let messages = stream(shapes()[k]);
        let mut encoder = Encoder::new(Context::default());
        let mut file = Vec::new();
        for message in &messages {
            encoder.ingest(message).unwrap();
            file.extend(encoder.take_output());
            encoder.flush().unwrap();
            file.extend(encoder.take_output());
        }
        let body = file.len();
        file.extend(encoder.finish("capture").unwrap().unwrap().tail);
        let footer = u64::from_le_bytes(file[file.len() - 24..file.len() - 16].try_into().unwrap());
        let footer = usize::try_from(footer).unwrap();
        let (all, all_markers, _) = content(&file);
        let dir = tempfile::tempdir().unwrap();
        let step = ((file.len() - body) / 128).max(1);
        let cuts: std::collections::BTreeSet<usize> = (body..file.len())
            .step_by(step)
            .chain([footer, file.len() - 1])
            .chain(footer.saturating_sub(if k == 13 { 4096 } else { 0 })..footer)
            .collect();
        let mut failed = Vec::new();
        for &cut in &cuts {
            let path = dir.path().join(format!("cut-{cut}.tickwave"));
            std::fs::write(&path, &file[..cut]).unwrap();
            if let Err(error) = compact(&path) {
                failed.push(format!("{}: {error}", cut - body));
                continue;
            }
            let (samples, markers, _) = content(&std::fs::read(&path).unwrap());
            assert_eq!(samples[..], all[..samples.len()], "cut at {cut}");
            assert!(
                markers.iter().all(|m| all_markers.contains(m)),
                "cut at {cut}"
            );
            if cut == footer {
                assert_eq!(samples, all, "cut at the footer");
                assert_eq!(markers.len(), all_markers.len(), "cut at the footer");
            }
        }
        eprintln!(
            "shape {k}: {} markers, {} samples; tail {} bytes from {body}, footer at +{}, {} cuts",
            all_markers.len(),
            all.len(),
            file.len() - body,
            footer - body,
            cuts.len()
        );
        assert!(
            failed.is_empty(),
            "shape {k}: {} of {} cuts failed: {failed:#?}",
            failed.len(),
            cuts.len()
        );
    }
}

/// Clock counts between pairs at 1,000 pairs a second.
const P: u64 = CLOCK as u64 / 1_000;

/// An attached stream at 1,000 pairs a second whose first pairs message holds `first` pairs, then
/// `dio`, then a third message and the end.
fn attached(first: u32, dio: Message) -> Vec<Message> {
    let header = header(Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 5_000,
        dio_mask: 0b0001,
        messages: 3,
        end: true,
    });
    let pair = Pair {
        voltage_code: 30_000,
        current_code: 1_000,
        low: true,
        under_range: false,
    };
    let at = |sequence: u32| ns(u64::from(sequence) * P);
    let second = 5_000 + first;
    vec![
        Message::Header(header),
        Message::Pairs(Pairs::new(0, 5_000, at(5_000), vec![pair; first as usize])),
        Message::Pairs(Pairs::new(0, second, at(second), vec![pair; 10])),
        dio,
        Message::Pairs(Pairs::new(0, second + 10, at(second + 10), vec![pair; 10])),
        Message::End(End {
            reason: EndReason::Stopped,
            last_sequence: second + 19,
            time_ns: None,
        }),
    ]
}

fn spans(gaps: &[Gap]) -> Vec<(i64, i64, bool)> {
    gaps.iter().map(|g| (g.from, g.to, g.level)).collect()
}

#[test]
fn a_dio_loss_from_before_the_first_pair_starts_past_the_first_gap() {
    let messages = attached(
        10,
        Message::DioLost(DioLost {
            from_ns: ns(4_999 * P),
            to_ns: ns(5_012 * P),
            levels: 0,
        }),
    );
    let p = P as i64;
    assert_eq!(
        spans(&decode(&encode(&messages)).gaps[FIRST_DIO]),
        [
            (4_998 * p, 5_000 * p, true),
            (5_000 * p + 1, 5_012 * p, false)
        ]
    );
}

#[test]
fn a_dio_loss_that_ends_before_the_latest_promise_is_refused() {
    // The second pairs message promises everything before the first message's last pair.
    for (to, refused) in [(5_008, true), (5_009, false)] {
        let messages = attached(
            10,
            Message::DioLost(DioLost {
                from_ns: ns(5_003 * P),
                to_ns: ns(to * P),
                levels: 0,
            }),
        );
        let mut encoder = Encoder::new(Context::default());
        let outcome = messages[..4]
            .iter()
            .try_for_each(|message| encoder.ingest(message).map(drop));
        assert_eq!(outcome.is_err(), refused, "a loss to {to}");
    }
}

#[test]
fn an_edge_at_a_lone_first_pair_comes_after_the_first_gap() {
    let messages = attached(1, Message::Edges(Edges::new_at(vec![(5_000 * P, 0)])));
    let d = decode(&encode(&messages));
    let p = P as i64;
    assert_eq!(
        spans(&d.gaps[FIRST_DIO]),
        [(4_998 * p, 5_000 * p - 1, true)]
    );
    assert_eq!(d.edges[FIRST_DIO], [5_000 * p]);
}

/// A recording laid out as the Recorder lays one out, its pairs `period` apart and `phase` past
/// their ticks.
fn laid_out(period: Ratio, phase: Option<Ratio>) -> Vec<u8> {
    let header = header(Shape {
        seed: 0,
        rate: 1_000,
        first_sequence: 1,
        dio_mask: 0,
        messages: 1,
        end: true,
    });
    let timebase = Ratio::integer(u64::from(CLOCK)).unwrap();
    let mut channels = vec![
        analog("V", "V", period, int(1000)),
        analog("I", "A", period, int(1_000_000_000)),
        digital("low_range", false),
        digital("under_range", false),
    ];
    for channel in &mut channels[..2] {
        if let DeclaredKind::Analog { phase: p, .. } = &mut channel.data {
            *p = phase;
        }
    }
    let mut source = SourceHeader::new(SourceFormat::Other(FORMAT.into()), timebase);
    source.extra = header_extra(&header, &Context::default());
    let declaration = Declaration {
        timebase,
        channels,
        derived: Vec::new(),
        sources: vec![source],
        title: None,
        meta: BTreeMap::new(),
        overview_shift: overview_shift(period),
        requires: Vec::new(),
    };
    let mut writer = Writer::new(declaration, write_options()).unwrap();
    for c in [V, I] {
        writer.run(c, 0, 0).unwrap();
        for _ in 0..10 {
            writer.sample(c, 1u64).unwrap();
        }
    }
    let end = 11 * P as i64;
    let mut file = writer.take_output();
    file.extend(
        writer
            .finish(end, &[end, end], Finish::default())
            .unwrap()
            .tail,
    );
    file
}

#[test]
fn an_export_refuses_pairs_off_whole_ticks() {
    use escapement_tickwave_view::{Cache, Recording};
    let whole = Ratio::integer(P).unwrap();
    let cases = [
        (whole, None, true),
        (Ratio::new(2 * P + 1, 2).unwrap(), None, false),
        (whole, Ratio::new(1, 2), false),
    ];
    let dir = tempfile::tempdir().unwrap();
    let cancel = AtomicBool::new(false);
    for (k, (period, phase, exports)) in cases.into_iter().enumerate() {
        let recording = Recording::open(
            std::sync::Arc::new(laid_out(period, phase)),
            Cache::new(1 << 20),
        )
        .unwrap();
        let path = dir.path().join(format!("{k}.csv"));
        let outcome = export(&recording, &path, true, 0..i64::MAX / 4, &cancel);
        assert_eq!(
            outcome.is_ok(),
            exports,
            "{period:?} {phase:?}: {:?}",
            outcome.err()
        );
        if !exports {
            assert!(matches!(outcome, Err(Error::NotVerge)));
        }
    }
}

#[test]
fn a_dio_recordings_published_end_is_its_promise() {
    let messages = dio_stream(3, 0b1111, 40);
    let dir = tempfile::tempdir().unwrap();
    let mut writer = FileWriter::create(&dir.path().join("dio.tickwave"), Context::default())
        .unwrap()
        .flushing_always();
    let slot = writer.snapshots();
    let mut checked = 0;
    for message in &messages[..messages.len() - 1] {
        writer.ingest(message).unwrap();
        let Some(published) = slot.lock().unwrap().take() else {
            continue;
        };
        // The levels hold through the promise, a DIO time's included.
        let promise = published.snapshot.promise();
        assert_eq!(
            published.end,
            (promise > 0).then_some(promise),
            "snapshot {checked}"
        );
        checked += 1;
    }
    assert!(checked > 10, "{checked} snapshots checked");
}

#[test]
fn a_directorys_recordings_are_listed_and_a_cut_off_one_is_finished_by_compacting() {
    let messages = stream(shapes()[13]);
    let dir = tempfile::tempdir().unwrap();
    let whole = encode(&messages);
    let done = dir.path().join("a.tickwave");
    let cut = dir.path().join("b.tickwave");
    std::fs::write(&done, &whole).unwrap();
    std::fs::write(&cut, &whole[..whole.len() / 2]).unwrap();
    std::fs::write(dir.path().join("a.tickwave.x1y2.partial"), b"copy").unwrap();
    std::fs::write(dir.path().join("notes.txt"), b"").unwrap();

    assert_eq!(recordings(dir.path()).unwrap(), [done.clone(), cut.clone()]);
    assert!(finished(&done).unwrap());
    assert!(!finished(&cut).unwrap());
    compact(&cut).unwrap();
    assert!(finished(&cut).unwrap());
    assert!(recordings(&dir.path().join("none")).unwrap().is_empty());
}

/// Sends a stream through a recorder, waiting out its bounded queue.
fn record(path: &Path, messages: &[Message]) -> Recorder {
    let mut recorder = Recorder::create(path, Context::default()).unwrap();
    for message in messages {
        // The recorder's queue is bounded; it is behind while its thread catches up.
        while let Err(error) = recorder.send(message.clone()) {
            assert!(matches!(error, Error::Behind), "{error}");
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    recorder
}

/// An ended recorder finishes its file on its own thread: once it says it has, the file is whole
/// before anyone closes it.
#[test]
fn an_ended_recorder_finishes_its_file_on_its_own() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ended.tickwave");
    // A stream without its End message: the writer stops only on `end`.
    let mut recorder = record(&path, &stream(shapes()[4]));
    recorder.end();
    let start = std::time::Instant::now();
    while !recorder.is_finished() {
        assert!(start.elapsed().as_secs() < 10, "the writer never finished");
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    assert!(finished(&path).unwrap());
    recorder.close().unwrap();
}

#[test]
fn a_recorder_finishes_its_recording_compacted_and_closes_it_without() {
    let messages = stream(shapes()[13]);
    // The same name in two directories: the file's name is part of what it holds.
    let (one, other) = (tempfile::tempdir().unwrap(), tempfile::tempdir().unwrap());
    let closed = one.path().join("recorded.tickwave");
    record(&closed, &messages).close().unwrap();
    assert!(finished(&closed).unwrap());
    let finished_path = other.path().join("recorded.tickwave");
    record(&finished_path, &messages).finish().unwrap();
    let before = std::fs::read(&closed).unwrap();
    compact(&closed).unwrap();
    let compacted = std::fs::read(&closed).unwrap();
    assert_ne!(compacted, before, "close left it uncompacted");
    assert_eq!(
        std::fs::read(&finished_path).unwrap(),
        compacted,
        "finish compacted it"
    );
}
