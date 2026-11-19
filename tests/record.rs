//! Sigrok sessions and CSV, from samples and from a stream's messages, and `record::Writer`.

mod common;

use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::time::Duration;

use common::{end, header, pairs};
use escapement_verge_probe::record::{Format, RecordError, Sink, Writer, samplerate};
use escapement_verge_probe::stream::{self, CurrentRange, DioLost, Edge, Edges, Sample};

const CHUNK: usize = 1 << 20;

fn sample(sequence: u32, time_ms: u64, millivolts: u32, nanoamps: u32) -> Sample {
    Sample {
        sequence,
        time_ns: time_ms * 1_000_000,
        millivolts,
        nanoamps,
        range: CurrentRange::Low,
        under_range: false,
    }
}

fn read(zip: &mut zip::ZipArchive<File>, name: &str) -> Vec<u8> {
    let mut bytes = Vec::new();
    zip.by_name(name).unwrap().read_to_end(&mut bytes).unwrap();
    bytes
}

fn floats(bytes: &[u8]) -> Vec<f32> {
    bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect()
}

/// A pause between pairs 3 and 4 repeats pair 3 across it with GAP high, so the time axis stays
/// true.
#[test]
fn a_gap_repeats_the_last_pair_with_gap_high() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gap.sr");
    let mut sink = Sink::create(&path, false).unwrap();
    sink.begin(1000, 0, 0).unwrap();
    sink.samples(&[
        sample(1, 0, 3300, 20_000),
        sample(2, 1, 3300, 5_000_000),
        sample(3, 2, 3290, 1_000),
        sample(4, 5, 3280, 2_000),
        sample(5, 6, 3280, 3_000),
    ])
    .unwrap();
    sink.finish().unwrap();

    let mut zip = zip::ZipArchive::new(File::open(&path).unwrap()).unwrap();
    assert_eq!(read(&mut zip, "version"), b"2");
    assert_eq!(read(&mut zip, "logic-1-1"), [0, 0, 0, 1, 1, 0, 0]);
    assert_eq!(
        floats(&read(&mut zip, "analog-1-3-1")),
        [3.3, 3.3, 3.29, 3.29, 3.29, 3.28, 3.28]
    );
    assert_eq!(
        floats(&read(&mut zip, "analog-1-4-1")),
        [0.02, 5.0, 0.001, 0.001, 0.001, 0.002, 0.003]
    );
    let metadata = String::from_utf8(read(&mut zip, "metadata")).unwrap();
    assert!(metadata.contains("samplerate=1 kHz\n"), "{metadata}");
}

/// A pair read under the high range's floor is high on UNDER, the last logic channel; CSV names
/// its range `under`.
#[test]
fn a_pair_under_the_high_range_is_marked() {
    let dir = tempfile::tempdir().unwrap();
    let mut samples = [sample(1, 0, 3300, 20_000), sample(2, 1, 3300, 5_000_000)];
    for s in &mut samples {
        s.range = CurrentRange::High;
    }
    samples[0].under_range = true;
    let path = dir.path().join("under.sr");
    let mut sink = Sink::create(&path, false).unwrap();
    sink.begin(1000, 0, 0).unwrap();
    sink.samples(&samples).unwrap();
    sink.finish().unwrap();
    let mut zip = zip::ZipArchive::new(File::open(&path).unwrap()).unwrap();
    assert_eq!(read(&mut zip, "logic-1-1"), [2, 0]);
    let metadata = String::from_utf8(read(&mut zip, "metadata")).unwrap();
    assert!(metadata.contains("probe2=UNDER\n"), "{metadata}");
    let path = dir.path().join("under.csv");
    let mut sink = Sink::create(&path, true).unwrap();
    sink.begin(1000, 0, 0).unwrap();
    sink.samples(&samples).unwrap();
    sink.finish().unwrap();
    let csv = std::fs::read_to_string(&path).unwrap();
    assert!(
        csv.contains("1,0,3300,20000,under\n") && csv.contains("2,1000000,3300,5000000,high\n"),
        "{csv}"
    );
}

fn edge(time_ns: u64, levels: u8) -> Edge {
    Edge { time_ns, levels }
}

/// Each pin reported is a logic channel after GAP, its level at each sample, the analog channels
/// numbered on after them. A pulse between two samples shows in the second; a gap's repeated
/// samples take the levels at their own times.
#[test]
fn dio_pins_are_logic_channels_at_each_sample() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("dio.sr");
    let mut sink = Sink::create(&path, false).unwrap();
    sink.begin(1000, 0x09, 0x0F).unwrap();
    sink.edges(vec![
        // DIO0 falls just after the sample at 1 ms.
        edge(1_000_100, 0x08),
        // DIO3 pulses low and back between 2 and 3 ms, so reads low at 3.
        edge(2_000_300, 0x00),
        edge(2_000_400, 0x08),
        // DIO0 rises in the gap, at 3.5 ms.
        edge(3_500_000, 0x09),
    ]);
    sink.samples(&[
        sample(1, 0, 3300, 1),
        sample(2, 1, 3300, 1),
        sample(3, 2, 3300, 1),
        sample(4, 3, 3300, 1),
        sample(5, 6, 3300, 1),
    ])
    .unwrap();
    sink.finish().unwrap();

    let mut zip = zip::ZipArchive::new(File::open(&path).unwrap()).unwrap();
    // Bit 0 GAP, bit 1 DIO0, bit 2 DIO3, bit 3 UNDER.
    assert_eq!(read(&mut zip, "logic-1-1"), [6, 6, 4, 0, 7, 7, 6]);
    assert_eq!(floats(&read(&mut zip, "analog-1-5-1")).len(), 7);
    assert_eq!(floats(&read(&mut zip, "analog-1-6-1")).len(), 7);
    let metadata = String::from_utf8(read(&mut zip, "metadata")).unwrap();
    assert!(
        metadata.contains(
            "total probes=4\nsamplerate=1 kHz\ntotal analog=2\nprobe1=GAP\nprobe2=DIO0\n\
             probe3=DIO3\nprobe4=UNDER\nanalog5=V\nanalog6=mA\nunitsize=1\n"
        ),
        "{metadata}"
    );
}

/// CSV puts each edge on a line of its own, among the pairs by time, and each pair's levels after
/// it; edges after the last pair come last.
#[test]
fn csv_lines_up_edges_and_pairs_by_time() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("dio.csv");
    let mut sink = Sink::create(&path, true).unwrap();
    sink.begin(1000, 0x0A, 0x02).unwrap();
    sink.edges(vec![edge(1_000_000, 0x0A), edge(1_500_000, 0x08)]);
    sink.samples(&[sample(1, 0, 3300, 7), sample(2, 1, 3300, 8)])
        .unwrap();
    sink.edges(vec![edge(2_200_000, 0x00)]);
    sink.samples(&[sample(3, 2, 3300, 9)]).unwrap();
    sink.finish().unwrap();
    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        "sequence,time_ns,millivolts,nanoamps,range,dio1,dio3\n\
         1,0,3300,7,low,1,0\n\
         ,1000000,,,,1,1\n\
         2,1000000,3300,8,low,1,1\n\
         ,1500000,,,,0,1\n\
         3,2000000,3300,9,low,0,1\n\
         ,2200000,,,,0,0\n"
    );
}

/// Past a chunk's worth, the samples go on in the next chunk file of each channel.
#[test]
fn a_long_recording_is_written_in_chunks() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("long.sr");
    let mut sink = Sink::create(&path, false).unwrap();
    sink.begin(100_000, 0, 0).unwrap();
    let samples: Vec<Sample> = (0..CHUNK as u32 + 3)
        .map(|n| Sample {
            sequence: n,
            time_ns: u64::from(n) * 10_000,
            millivolts: 1800,
            nanoamps: n,
            range: CurrentRange::High,
            under_range: false,
        })
        .collect();
    sink.samples(&samples).unwrap();
    sink.finish().unwrap();

    let mut zip = zip::ZipArchive::new(File::open(&path).unwrap()).unwrap();
    assert_eq!(read(&mut zip, "logic-1-1").len(), CHUNK);
    assert_eq!(read(&mut zip, "logic-1-2").len(), 3);
    assert_eq!(
        floats(&read(&mut zip, "analog-1-4-2")),
        [
            CHUNK as f32 / 1e6,
            (CHUNK + 1) as f32 / 1e6,
            (CHUNK + 2) as f32 / 1e6
        ]
    );
    assert!(zip.by_name("logic-1-3").is_err());
}

#[test]
fn rates_read_as_libsigrok_writes_them() {
    assert_eq!(samplerate(1), "1 Hz");
    assert_eq!(samplerate(1000), "1 kHz");
    assert_eq!(samplerate(1500), "1500 Hz");
    assert_eq!(samplerate(100_000), "100 kHz");
}

#[test]
fn a_files_extension_names_its_format_in_any_case() {
    for (name, format) in [
        ("run.tickwave", Some(Format::Tickwave)),
        ("run.SR", Some(Format::Sigrok)),
        ("run.Csv", Some(Format::Csv)),
        ("run.txt", None),
        ("run", None),
    ] {
        assert_eq!(Format::named_by(Path::new(name)), format, "{name}");
    }
}

/// Writes `messages`, as a probe sends them, to a CSV file, and returns its lines.
fn csv_of(messages: &[Vec<u8>]) -> Vec<String> {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("stream.csv");
    let mut writer = Writer::create(&path, Format::Csv, Default::default()).unwrap();
    for message in messages {
        writer.send(stream::parse(message).unwrap()).unwrap();
    }
    writer.finish().unwrap();
    std::fs::read_to_string(&path)
        .unwrap()
        .lines()
        .map(str::to_owned)
        .collect()
}

/// The pairs either side of a gap in the sequence are written as they came.
#[test]
fn a_writer_keeps_the_pairs_either_side_of_a_gap() {
    let lines = csv_of(&[header(0, 0), pairs(1, 3), pairs(6, 2), end(2, 7)]);
    let sequences: Vec<&str> = lines[1..]
        .iter()
        .map(|line| line.split(',').next().unwrap())
        .collect();
    assert_eq!(sequences, ["1", "2", "3", "6", "7"]);
}

/// A marker that comes after the pairs around it is merged in by its time.
#[test]
fn late_markers_are_written_in_time_order() {
    let mut marker = vec![0xa4, 1, 1];
    marker.extend_from_slice(&500_000u64.to_le_bytes());
    marker.extend_from_slice(&42u32.to_le_bytes());
    marker.push(8);
    marker.extend_from_slice(b"step,one");
    let lines = csv_of(&[header(0, 0), pairs(1, 3), pairs(4, 3), marker, end(2, 6)]);
    assert!(lines[0].contains("marker_number,marker_label,marker_lost"));
    assert!(
        lines
            .iter()
            .any(|line| line == ",500000,,,,42,\"step,one\",1"),
        "{lines:?}"
    );
    let times: Vec<u64> = lines[1..]
        .iter()
        .map(|row| row.split(',').nth(1).unwrap().parse().unwrap())
        .collect();
    assert!(times.windows(2).all(|pair| pair[0] <= pair[1]), "{lines:?}");
}

/// A pairs message waits for the next message, so an edge inside it that comes after it still
/// lands among its pairs.
#[test]
fn edges_after_their_pairs_land_among_them() {
    let edges = |edges: &[(u64, u8)]| {
        Edges::new(
            0,
            edges
                .iter()
                .map(|&(time_ns, levels)| edge(time_ns, levels))
                .collect(),
        )
        .to_bytes()
    };
    let lost = DioLost {
        from_ns: 3_100_000,
        to_ns: 3_200_000,
        levels: 0x00,
    };
    let lines = csv_of(&[
        header(0x01, 0x00),
        pairs(1, 3),
        edges(&[(1_500_000, 0x01)]),
        pairs(4, 2),
        lost.to_bytes(),
        edges(&[(4_000_000, 0x01)]),
        end(2, 5),
    ]);
    let lines: Vec<String> = lines
        .iter()
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            format!("{}@{}={}/{}", fields[0], fields[1], fields[5], fields[6])
        })
        .collect();
    assert_eq!(
        lines,
        [
            "sequence@time_ns=dio0/dio_gap",
            "1@0=0/0",
            "2@1000000=0/0",
            "@1500000=1/0",
            "3@2000000=1/0",
            "4@3000000=1/0",
            "@3100000=/1",
            "@3200000=0/0",
            "@4000000=1/0",
            "5@4000000=1/0",
        ]
    );
}

/// A sigrok session or CSV records energy, so a DIO stream alone is refused; the writer stops and
/// says why.
#[test]
fn a_sink_refuses_a_dio_stream() {
    let dir = tempfile::tempdir().unwrap();
    let mut writer =
        Writer::create(&dir.path().join("dio.csv"), Format::Csv, Default::default()).unwrap();
    let dio_header = vec![0xA1, 5, 0, 0x6C, 0xDC, 0x02, 0x0F, 0, 7, 0];
    writer.send(stream::parse(&dio_header).unwrap()).unwrap();
    let mut refused = Ok(());
    for _ in 0..100 {
        refused = writer.send(stream::parse(&pairs(1, 1)).unwrap());
        if refused.is_err() {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(matches!(refused, Err(RecordError::Stopped)), "{refused:?}");
    assert!(matches!(writer.finish(), Err(RecordError::DioOnly)));
}

/// A file that stops taking pairs fails the writer with its own error.
#[cfg(target_os = "linux")]
#[test]
fn a_full_disk_fails_the_writer() {
    let mut writer =
        Writer::create(Path::new("/dev/full"), Format::Csv, Default::default()).unwrap();
    writer.send(stream::parse(&header(0, 0)).unwrap()).ok();
    for first in (1..2000).step_by(100) {
        writer.send(stream::parse(&pairs(first, 100)).unwrap()).ok();
    }
    let error = writer.finish().expect_err("the disk is full");
    assert!(error.to_string().contains("No space left"), "{error}");
}

/// A tickwave recording from the simulated Pro holds every pair the reader counted, its channels,
/// and where it came from; finishing it leaves nothing beside it.
#[cfg(feature = "tickwave")]
#[test]
fn a_tickwave_writer_holds_every_pair_and_its_header() {
    use escapement_verge_probe::Bus;
    use escapement_verge_probe::sim::SimBus;
    use escapement_verge_probe::stream::{EnergyStream, Message, Reader};
    use escapement_verge_probe::tickwave::format::decode::Footer;
    use escapement_verge_probe::tickwave::format::header::ChannelKind;
    use escapement_verge_probe::verge_aux::Aux;
    let bus = SimBus::scenario("ours-pro").unwrap();
    let device = escapement_verge_probe::detect(&bus)
        .unwrap()
        .remove(0)
        .device;
    let mut reader = Reader::new(Aux::open(bus.open_aux(&device).unwrap()).unwrap());
    reader
        .start_energy(&EnergyStream::new(1000).dio(0b0101))
        .unwrap();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("run.tickwave");
    let mut writer = Writer::create(&path, Format::Tickwave, reader.context()).unwrap();
    let until = bus.now() + Duration::from_millis(300);
    for message in reader.messages(|| bus.now() >= until) {
        let message: Message = message.unwrap();
        writer.send(message).unwrap();
    }
    writer.finish().unwrap();
    let pairs = reader.counts().total.pairs;
    assert!(pairs > 100, "{pairs}");
    let file = std::fs::read(&path).unwrap();
    let header = Footer::parse_file(&file).unwrap().header;
    let ChannelKind::Analog { count, .. } = header.channels[0].data else {
        panic!("V is analog");
    };
    assert_eq!(count, pairs);
    let names: Vec<&str> = header.channels.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        names,
        ["V", "I", "low_range", "under_range", "DIO0", "DIO2"]
    );
    let source = &header.sources[0];
    assert_eq!(source.software, reader.context().software);
    assert!(escapement_verge_probe::tickwave::header_from(&source.extra).is_some());
    assert_eq!(source.extra["end_reason"], "stopped");
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}
