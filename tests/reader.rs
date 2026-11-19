//! `stream::Reader` on the simulated bench, and on played-back streams.

mod common;

use std::collections::VecDeque;
use std::time::Duration;

use common::{end, header, pairs};

use escapement_verge_probe::dap::LinkError;
use escapement_verge_probe::sim::{SimAux, SimBus};
use escapement_verge_probe::stream::{
    Began, DioLost, Edge, Edges, EndReason, EnergyStream, Marker, Markers, Message, Reader,
    ReaderError,
};
use escapement_verge_probe::verge_aux::{Aux, AuxLink};
use escapement_verge_probe::{
    Bus, DapTransport, EnergyCaptureSample, EnergyRanges, EnergyTotal, UsbDevice, VergeControl,
    detect,
};

fn unit(scenario: &str) -> (SimBus, UsbDevice) {
    let bus = SimBus::scenario(scenario).unwrap();
    let device = detect(&bus).unwrap().remove(0).device;
    (bus, device)
}

fn reader(bus: &SimBus, device: &UsbDevice) -> Reader<SimAux> {
    Reader::new(Aux::open(bus.open_aux(device).unwrap()).unwrap())
}

/// The requests sent to Verge Aux, in hex.
fn aux_log(bus: &SimBus) -> Vec<String> {
    bus.log()
        .into_iter()
        .filter_map(|line| line.split_once("AUX ").map(|(_, hex)| hex.to_owned()))
        .collect()
}

/// The stream's messages, asking for its end once `after` has passed on the bench's clock.
fn record<L: AuxLink>(reader: &mut Reader<L>, bus: &SimBus, after: Duration) -> Vec<Message> {
    let until = bus.now() + after;
    reader
        .messages(|| bus.now() >= until)
        .collect::<Result<_, _>>()
        .unwrap()
}

/// Runs `test` on a thread of its own, failing it if it has not finished in a minute: a deadline
/// that stops working would otherwise hang the test rather than fail it.
fn within_a_minute(test: impl FnOnce() + Send + 'static) {
    let (done, finished) = std::sync::mpsc::channel();
    let thread = std::thread::spawn(move || {
        test();
        let _ = done.send(());
    });
    match finished.recv_timeout(Duration::from_secs(60)) {
        Ok(()) => thread.join().unwrap(),
        // The test panicked: join gives its panic.
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => thread.join().unwrap(),
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!("the test did not finish"),
    }
}

fn capturing(reader: &mut Reader<impl AuxLink>) -> bool {
    !matches!(
        reader.control().latest_energy_sample(),
        Ok(EnergyCaptureSample::Stopped)
    )
}

/// A capture the reader starts runs from its header to its end, and stops with the stream: the
/// stop names the capture.
#[test]
fn a_started_stream_runs_from_its_header_to_its_end_and_stops_its_capture() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    let began = reader
        .start_energy(&EnergyStream::new(1000).dio(0b0101))
        .unwrap();
    assert!(matches!(began, Began::Started { replaced: None, .. }));
    assert!(reader.owns_capture());
    let messages = record(&mut reader, &bus, Duration::from_millis(300));
    assert!(matches!(messages.first(), Some(Message::Header(_))));
    let Some(Message::End(end)) = messages.last() else {
        panic!("the stream's end comes last");
    };
    assert_eq!(end.reason, EndReason::Stopped);
    let counts = reader.counts();
    assert!(counts.total.pairs > 100, "{counts:?}");
    assert_eq!(counts.lost, 0);
    assert_eq!(counts.pins, reader.header().unwrap().dio_mask);
    assert_eq!(counts.end, Some(*end));
    assert!(!reader.active());
    assert!(!capturing(&mut reader));
    let log = aux_log(&bus);
    assert!(
        log.iter()
            .any(|hex| hex.starts_with("900b") && hex.len() == 52),
        "{log:?}"
    );
}

/// A running capture is followed, not restarted: the stream ends by the capture's identity, and
/// the capture runs on.
#[test]
fn a_joined_stream_ends_and_leaves_its_capture_running() {
    let (bus, device) = unit("ours-pro-energy");
    let mut reader = reader(&bus, &device);
    let began = reader.start_energy(&EnergyStream::new(5000)).unwrap();
    assert_eq!(began, Began::Joined);
    assert!(!reader.owns_capture());
    let messages = record(&mut reader, &bus, Duration::from_millis(300));
    let Some(Message::End(end)) = messages.last() else {
        panic!("the stream's end comes last");
    };
    assert_eq!(end.reason, EndReason::Asked);
    assert_eq!(reader.header().unwrap().whole_rate(), 1000);
    assert!(capturing(&mut reader));
    let log = aux_log(&bus);
    assert!(log.iter().any(|hex| hex.starts_with("900d")), "{log:?}");
    assert!(
        !log.iter()
            .any(|hex| hex.starts_with("900b") || hex == "9002"),
        "{log:?}"
    );
}

/// A failed capture counts as none running: the start replaces it, and says what it had counted.
/// With joining off, a running one is replaced too.
#[test]
fn a_start_replaces_a_failed_capture_or_one_it_may_not_join() {
    let (bus, device) = unit("ours-pro-energy");
    bus.fail_energy(0);
    let mut reader = reader(&bus, &device);
    let began = reader.start_energy(&EnergyStream::new(1000)).unwrap();
    assert!(
        matches!(
            began,
            Began::Started {
                replaced: Some(EnergyTotal::Failed(_)),
                ..
            }
        ),
        "{began:?}"
    );
    record(&mut reader, &bus, Duration::ZERO);

    let (bus, device) = unit("ours-pro-energy");
    let mut reader = self::reader(&bus, &device);
    let began = reader
        .start_energy(&EnergyStream::new(1000).join(false))
        .unwrap();
    assert!(
        matches!(
            began,
            Began::Started {
                replaced: Some(EnergyTotal::Total(_)),
                ..
            }
        ),
        "{began:?}"
    );
}

/// A start sets the ranges it was given, whatever another host asked for before; a join keeps the
/// running capture's.
#[test]
fn a_start_uses_its_ranges_and_a_join_keeps_the_captures() {
    let header = |scenario: &str, ranges: EnergyRanges| {
        let (bus, device) = unit(scenario);
        let mut reader = reader(&bus, &device);
        reader
            .control()
            .set_energy_ranges(EnergyRanges::High)
            .unwrap();
        reader
            .control()
            .set_energy_ranges(match ranges {
                EnergyRanges::High => EnergyRanges::Auto,
                _ => EnergyRanges::High,
            })
            .unwrap();
        reader
            .start_energy(&EnergyStream::new(1000).ranges(ranges))
            .unwrap();
        record(&mut reader, &bus, Duration::ZERO);
        reader.header().unwrap().high_range_only
    };
    assert!(header("ours-pro", EnergyRanges::High));
    assert!(!header("ours-pro", EnergyRanges::Auto));
    assert!(!header("ours-pro-energy", EnergyRanges::High));
}

/// Another host's start ends this stream; the reader leaves the new capture alone, on the way out
/// and when dropped.
#[test]
fn a_restart_by_another_host_ends_the_stream_and_leaves_the_new_capture() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    let Some(Message::Header(_)) = reader.read(Duration::from_secs(1)).unwrap() else {
        panic!("the header comes first");
    };
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let other = VergeControl::new(&mut transport)
        .start_energy_capture_identified(2000)
        .unwrap()
        .capture_id
        .unwrap();
    let messages = record(&mut reader, &bus, Duration::from_secs(5));
    let Some(Message::End(end)) = messages.last() else {
        panic!("the stream's end comes last");
    };
    assert_eq!(end.reason, EndReason::Restarted);
    assert!(!reader.active());
    drop(reader);
    let status = VergeControl::new(&mut transport)
        .energy_capture_status()
        .unwrap();
    assert!(status.running);
    assert_eq!(status.capture_id, Some(other));
}

/// Asking for the end after another host restarted the capture is no error: the stream ends by
/// itself.
#[test]
fn an_end_asked_after_a_restart_waits_for_the_streams_own() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    VergeControl::new(&mut transport)
        .start_energy_capture_identified(2000)
        .unwrap();
    reader.end().unwrap();
    let messages = record(&mut reader, &bus, Duration::ZERO);
    assert!(matches!(messages.last(), Some(Message::End(_))));
}

/// Dropping a reader mid-stream stops the capture it started.
#[test]
fn a_dropped_reader_stops_its_own_capture() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    reader.read(Duration::from_secs(1)).unwrap();
    drop(reader);
    let mut reader = self::reader(&bus, &device);
    assert!(!capturing(&mut reader));
}

/// A second start while a stream is under way is refused, and a refused start leaves the reader
/// idle.
#[test]
fn a_start_is_refused_while_a_stream_runs_or_by_the_probe() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    assert!(matches!(
        reader.start_energy(&EnergyStream::new(1000)),
        Err(ReaderError::Busy)
    ));
    assert!(matches!(reader.start_dio(1), Err(ReaderError::Busy)));
    record(&mut reader, &bus, Duration::ZERO);
    assert!(matches!(
        reader.start_energy(&EnergyStream::new(0)),
        Err(ReaderError::Start(_))
    ));
    assert!(!reader.active());
}

/// The edges of a standalone MCU-Link's pins, alone: each pin's own, and the stream's end time.
#[test]
fn a_dio_stream_counts_each_pins_edges_and_ends_when_asked() {
    let (bus, device) = unit("ours-plain-busy");
    let mut reader = reader(&bus, &device);
    reader.start_dio(0x0F).unwrap();
    let messages = record(&mut reader, &bus, Duration::from_millis(500));
    assert!(matches!(messages.first(), Some(Message::DioHeader(_))));
    let Some(Message::End(end)) = messages.last() else {
        panic!("the stream's end comes last");
    };
    assert_eq!(end.reason, EndReason::Asked);
    assert!(end.time_ns.is_some());
    let counts = reader.counts();
    assert_eq!(counts.pins, 0x0F);
    assert!(counts.edges[1] >= 8, "{counts:?}");
    assert_eq!([counts.edges[0], counts.edges[2], counts.edges[3]], [0; 3]);
    assert_eq!(counts.total.pairs, 0);
    assert!(aux_log(&bus).iter().any(|hex| hex == "9007"));
}

#[test]
fn a_dio_stream_counts_its_losses() {
    let (bus, device) = unit("ours-plain-edge-loss");
    let mut reader = reader(&bus, &device);
    reader.start_dio(0x0F).unwrap();
    record(&mut reader, &bus, Duration::from_millis(500));
    assert!(reader.counts().edge_losses >= 1, "{:?}", reader.counts());
}

/// Wraps the simulator's link, as a test needs it changed.
struct Changed<F> {
    link: SimAux,
    bus: SimBus,
    change: F,
}

impl<F: FnMut(Vec<u8>) -> Option<Vec<u8>>> AuxLink for Changed<F> {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError> {
        self.link.write(request)
    }

    /// A message the change drops is a read that timed out. The simulator's clock stands still
    /// while no stream runs, so a read that times out moves it on.
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError> {
        let changed = match self.link.read(timeout) {
            Ok(message) => (self.change)(message),
            Err(LinkError::Timeout) => None,
            Err(error) => return Err(error),
        };
        changed.ok_or_else(|| {
            self.bus.sleep(timeout);
            LinkError::Timeout
        })
    }

    fn now(&self) -> Duration {
        self.link.now()
    }
}

fn changed<F: FnMut(Vec<u8>) -> Option<Vec<u8>>>(
    bus: &SimBus,
    device: &UsbDevice,
    change: F,
) -> Reader<Changed<F>> {
    Reader::new(
        Aux::open(Changed {
            link: bus.open_aux(device).unwrap(),
            bus: bus.clone(),
            change,
        })
        .unwrap(),
    )
}

/// The header has two seconds, however many pairs come meanwhile; then the capture the reader
/// started stops.
#[test]
fn a_missing_header_fails_the_stream_even_while_pairs_arrive() {
    within_a_minute(missing_header);
}

fn missing_header() {
    let (bus, device) = unit("ours-pro");
    let mut reader = changed(&bus, &device, |message| {
        Some(if message.first() == Some(&0xA1) {
            pairs(1, 1)
        } else {
            message
        })
    });
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    let started = bus.now();
    let result: Result<Vec<Message>, _> = reader.messages(|| false).collect();
    assert!(matches!(result, Err(ReaderError::NoHeader)), "{result:?}");
    assert!(bus.now() - started < Duration::from_secs(3));
    assert!(!reader.active());
    assert!(aux_log(&bus).iter().any(|hex| hex.starts_with("900b")));
}

/// An end asked for and not sent within two seconds fails the stream.
#[test]
fn an_end_two_seconds_late_fails_the_stream() {
    within_a_minute(end_late);
}

fn end_late() {
    let (bus, device) = unit("ours-pro");
    let mut reader = changed(&bus, &device, |message| {
        (message.first() != Some(&0xA2)).then_some(message)
    });
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    let result: Result<Vec<Message>, _> = reader.messages(|| true).collect();
    assert!(matches!(result, Err(ReaderError::NotEnded)), "{result:?}");
}

/// A header or an end that has come is taken however late it is read: a file that takes long to
/// make between the start and the first read fails nothing.
#[test]
fn a_header_or_an_end_read_late_is_taken() {
    within_a_minute(|| {
        let (bus, device) = unit("ours-pro");
        let mut reader = reader(&bus, &device);
        reader.start_energy(&EnergyStream::new(1000)).unwrap();
        bus.sleep(Duration::from_secs(3));
        assert!(matches!(
            reader.read(Duration::ZERO),
            Ok(Some(Message::Header(_)))
        ));
        reader.end().unwrap();
        bus.sleep(Duration::from_secs(3));
        let mut ended = false;
        while let Some(message) = reader.read(Duration::ZERO).unwrap() {
            ended = matches!(message, Message::End(_));
        }
        assert!(ended && !reader.active());
    });
}

/// A link that fails every read once `dead` is set, as an unplugged unit's does.
struct Dying {
    link: SimAux,
    dead: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

impl AuxLink for Dying {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError> {
        self.link.write(request)
    }

    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError> {
        if self.dead.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(LinkError::Disconnected);
        }
        self.link.read(timeout)
    }

    fn now(&self) -> Duration {
        self.link.now()
    }
}

/// A link that fails while a joined stream waits for its header still leaves the reader idle, so
/// the next start is not refused as busy.
#[test]
fn abandoning_over_a_dead_link_leaves_the_reader_idle() {
    within_a_minute(|| {
        let (bus, device) = unit("ours-pro-energy");
        let dead = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let link = Dying {
            link: bus.open_aux(&device).unwrap(),
            dead: dead.clone(),
        };
        let mut reader = Reader::new(Aux::open(link).unwrap());
        assert_eq!(
            reader.start_energy(&EnergyStream::new(1000)).unwrap(),
            Began::Joined
        );
        dead.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(matches!(reader.abandon(), Err(ReaderError::Read(_))));
        assert!(!reader.active());
        assert!(!matches!(
            reader.start_energy(&EnergyStream::new(1000)),
            Err(ReaderError::Busy)
        ));
    });
}

/// A joined stream's end needs the capture's identity, which only its header gives: one asked for,
/// or abandoned, before the header waits for it.
#[test]
fn a_joined_stream_ended_before_its_header_ends_by_its_identity() {
    for abandon in [false, true] {
        let (bus, device) = unit("ours-pro-energy");
        let mut held = None;
        let mut reader = changed(&bus, &device, move |message| {
            if message.first() == Some(&0xA1) {
                held = Some(message);
                return None;
            }
            // The header comes once the next message would have.
            Some(held.take().unwrap_or(message))
        });
        reader.start_energy(&EnergyStream::new(1000)).unwrap();
        assert!(reader.capture_id().is_none());
        if abandon {
            reader.abandon().unwrap();
        } else {
            reader.end().unwrap();
            assert!(reader.ending());
            record(&mut reader, &bus, Duration::ZERO);
        }
        let log = aux_log(&bus);
        assert!(
            log.iter()
                .any(|hex| hex.starts_with("900d") && hex.len() == 52),
            "{log:?}"
        );
        assert!(!log.iter().any(|hex| hex.starts_with("900b")), "{log:?}");
        assert!(capturing(&mut reader));
    }
}

/// Plays back stream messages, as a probe whose stream ends by itself would send them, for a
/// reader that joins its capture. `None` is a read that times out, as the ones that end the drains
/// on opening.
struct Playback(VecDeque<Option<Vec<u8>>>);

impl AuxLink for Playback {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError> {
        let reply = match request {
            [0x82] => {
                // Info format 6: a Pro with energy capture, its total and capture identities.
                let mut info = b"\x82CJTAG\x06\x00\x00\x01".to_vec();
                info.extend_from_slice(&0u32.to_le_bytes());
                info.extend_from_slice(&261u32.to_le_bytes());
                info
            }
            // A capture runs, with no pair yet.
            [0x90, 1] => [&[0x90, 0, 2][..], &[0; 8]].concat(),
            [0x90, 10, _] => vec![0x90, 0, 0],
            _ => return Ok(()),
        };
        self.0.push_front(Some(reply));
        Ok(())
    }

    fn read(&mut self, _: Duration) -> Result<Vec<u8>, LinkError> {
        self.0.pop_front().flatten().ok_or(LinkError::Timeout)
    }
}

/// A reader that has joined the capture `messages` play back.
fn played(messages: Vec<Vec<u8>>) -> Reader<Playback> {
    // Opening drains until a read times out, asks for mode 7, then drains again.
    let opening = [None, Some(vec![0x90, 0, 0]), None];
    let link = Playback(
        opening
            .into_iter()
            .chain(messages.into_iter().map(Some))
            .collect(),
    );
    let mut reader = Reader::new(Aux::open(link).unwrap());
    assert_eq!(
        reader.start_energy(&EnergyStream::new(1000)).unwrap(),
        Began::Joined
    );
    reader
}

fn read_all(reader: &mut Reader<Playback>) -> Vec<Message> {
    let mut messages = Vec::new();
    while let Some(message) = reader.read(Duration::ZERO).unwrap() {
        messages.push(message);
    }
    messages
}

/// Pairs the sequence skips are lost, and so are those just before the end, which shows only in
/// its last sequence. What comes before the header is a replaced stream's, and is dropped.
#[test]
fn skipped_sequences_count_as_lost_and_a_replaced_streams_tail_is_dropped() {
    let mut reader = played(vec![
        pairs(30, 2),
        end(1, 40),
        header(0, 0),
        pairs(1, 3),
        pairs(6, 2),
        end(2, 10),
    ]);
    let messages = read_all(&mut reader);
    assert_eq!(messages.len(), 4);
    assert!(matches!(messages[0], Message::Header(_)));
    let counts = reader.counts();
    assert_eq!((counts.total.pairs, counts.lost), (5, 2 + 3));
    assert_eq!(counts.end.unwrap().reason, EndReason::Stopped);
    assert!(!reader.active());
}

/// Each pin's edges count, from the levels the header gave; a loss counts once and gives the
/// levels known after it.
#[test]
fn edges_count_per_pin_and_losses_reset_the_levels() {
    let edges = |edges: &[(u64, u8)]| {
        Edges::new(
            0,
            edges
                .iter()
                .map(|&(time_ns, levels)| Edge { time_ns, levels })
                .collect(),
        )
        .to_bytes()
    };
    let lost = DioLost {
        from_ns: 3_100_000,
        to_ns: 3_200_000,
        levels: 0x08,
    };
    let mut reader = played(vec![
        header(0x09, 0x00),
        pairs(1, 3),
        edges(&[(1_500_000, 0x01), (1_600_000, 0x09)]),
        lost.to_bytes(),
        edges(&[(4_000_000, 0x09)]),
        Markers::new(
            Markers::LOST,
            vec![Marker {
                time_ns: 4_100_000,
                number: 1,
                label: b"one".to_vec(),
            }],
        )
        .to_bytes(),
        end(2, 3),
    ]);
    read_all(&mut reader);
    let counts = reader.counts();
    assert_eq!(counts.pins, 0x09);
    // DIO0 rose twice: before the loss and after it.
    assert_eq!(counts.edges, [2, 0, 0, 1]);
    assert_eq!(counts.edge_losses, 1);
    assert_eq!(counts.levels, 0x09);
    assert_eq!(counts.marker_losses, 1);
}

/// A second header, or a DIO stream's, does not belong in an energy stream.
#[test]
fn a_second_header_or_a_dio_header_fails_an_energy_stream() {
    let mut reader = played(vec![header(0, 0), header(0, 0)]);
    reader.read(Duration::ZERO).unwrap();
    assert!(matches!(
        reader.read(Duration::ZERO),
        Err(ReaderError::SecondStream)
    ));
    let dio_header = vec![0xA1, 5, 0, 0x6C, 0xDC, 0x02, 0x0F, 0, 7, 0];
    let mut reader = played(vec![dio_header]);
    assert!(matches!(
        reader.read(Duration::ZERO),
        Err(ReaderError::DioInEnergy)
    ));
}

/// The context names the probe and its firmware, for a recording of the stream.
#[test]
fn the_context_names_the_probe_and_its_firmware() {
    let (bus, device) = unit("ours-pro");
    let mut reader = reader(&bus, &device);
    assert_eq!(reader.context().instrument, None);
    reader.start_energy(&EnergyStream::new(1000)).unwrap();
    let context = reader.context();
    assert_eq!(context.instrument.as_deref(), Some("MCU-Link Pro"));
    let firmware = detect(&bus).unwrap()[0].verge().unwrap().firmware.clone();
    assert_eq!(context.software, Some(firmware));
    assert!(context.started_utc.is_some());
}
