//! Verge Aux on a simulated Pro: the supply and energy commands beside CMSIS-DAP, and the energy
//! stream through every way it ends.

use std::time::Duration;

use escapement_verge_probe::dap::{DapLink, Ours};
use escapement_verge_probe::sim::SimBus;
use escapement_verge_probe::stream::{EndReason, Message};
use escapement_verge_probe::verge_aux::Aux;
use escapement_verge_probe::{
    Bus, CommandError, DapTransport, EnergyTotal, Marked, VergeControl, detect,
};

fn pro(bus: &SimBus) -> escapement_verge_probe::UsbDevice {
    let unit = bus.add_ours("26ea505");
    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
    detect(bus).unwrap().remove(0).device
}

/// Messages until `want` matches one, or the stream goes quiet for a second of simulated time.
fn until(
    aux: &mut Aux<escapement_verge_probe::sim::SimAux>,
    want: impl Fn(&Message) -> bool,
) -> Vec<Message> {
    let mut seen = Vec::new();
    while let Some(message) = aux.message(Duration::from_secs(1)).unwrap() {
        let done = want(&message);
        seen.push(message);
        if done {
            break;
        }
    }
    seen
}

fn rate(rate: u32) -> Vec<u8> {
    let mut request = vec![0x90, 9];
    request.extend_from_slice(&rate.to_le_bytes());
    request.push(0);
    request
}

/// A Pro and a standalone MCU-Link running Verge offer it; an on-board MCU-Link does not.
#[test]
fn a_pro_and_a_standalone_mcu_link_offer_it() {
    let bus = SimBus::default();
    let device = pro(&bus);
    assert!(device.aux);
    for board in [Ours::BOARD_MCU_LINK, Ours::BOARD_ON_BOARD] {
        let unit = bus.add_ours("26ea505");
        bus.set_board(unit, board);
    }
    let units = detect(&bus).unwrap();
    let board = |unit: &escapement_verge_probe::Unit| unit.verge().and_then(|ours| ours.board);
    for unit in &units {
        let offers = board(unit) != Some(escapement_verge_probe::BoardKind::OnBoard);
        assert_eq!(unit.device.aux, offers, "{:?}", board(unit));
        assert_eq!(
            bus.open_aux(&unit.device).is_ok(),
            offers,
            "{:?}",
            board(unit)
        );
    }
}

/// The typed commands run over it as over CMSIS-DAP, and one capture answers on both.
#[test]
fn the_commands_answer_as_on_cmsis_dap() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut verge = VergeControl::new(&mut aux);
    assert!(verge.info().unwrap().unwrap().supports_energy_capture());
    assert!(verge.set_target_power(true).unwrap());
    assert_eq!(
        verge
            .start_energy_capture(1000)
            .unwrap()
            .actual_rate_pairs_per_second,
        1000
    );
    drop(verge);
    bus.sleep(Duration::from_millis(500));

    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut cmsis_dap = VergeControl::new(&mut transport);
    assert!(
        matches!(cmsis_dap.read_energy_accumulator().unwrap(), EnergyTotal::Total(total) if total.integrated_pairs >= 500)
    );
    // Anything else gets [0xff].
    assert_eq!(aux.exchange(&[0x00, 0x01]).unwrap(), [0xFF]);
    assert!(
        link.exchange(&[0x90, 9, 0xE8, 3, 0, 0, 0]).unwrap()[1] == 0xFF,
        "mode 9 is Verge Aux's alone"
    );
}

/// Mode 9: the reply, then the header, then every pair in order, then the end at a stop.
#[test]
fn a_stream_carries_every_pair_to_its_end() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    aux.exchange(&[0x8F, 1]).unwrap();
    let reply = aux.exchange(&rate(1000)).unwrap();
    assert_eq!(&reply[..3], [0x90, 0, 0]);
    let Some(Message::Header(header)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no header first");
    };
    assert_eq!(
        (header.rate(), header.first_sequence, header.setting_ma),
        (1000.0, 1, Some(350))
    );
    let mut next = 1;
    for message in until(
        &mut aux,
        |m| matches!(m, Message::Pairs(p) if p.first_sequence > 2_000),
    ) {
        let Message::Pairs(pairs) = message else {
            panic!("{message:?} mid-stream");
        };
        assert_eq!(pairs.first_sequence, next, "a gap in the sequence");
        assert!(!pairs.pairs.is_empty() && pairs.pairs.len() <= 99);
        next += pairs.pairs.len() as u32;
    }
    assert_eq!(aux.exchange(&[0x90, 2]).unwrap(), [0x90, 0, 0]);
    let tail = until(&mut aux, |m| matches!(m, Message::End(_)));
    let Some(Message::End(end)) = tail.last() else {
        panic!("no end: {tail:?}");
    };
    assert_eq!(end.reason, EndReason::Stopped);
    let streamed = tail
        .iter()
        .filter_map(|m| {
            if let Message::Pairs(p) = m {
                Some(p.pairs.len() as u32)
            } else {
                None
            }
        })
        .sum::<u32>();
    assert_eq!(end.last_sequence, next - 1 + streamed);
}

/// Mode 10 streams the running capture from its next pair; mode 7 ends it and the capture runs on;
/// a start on CMSIS-DAP restarts the capture and ends a stream with reason 3.
#[test]
fn a_stream_ends_as_its_capture_does() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    aux.exchange(&[0x8F, 1]).unwrap();
    assert_eq!(
        aux.exchange(&[0x90, 10, 0]).unwrap(),
        [0x90, 0, 1],
        "nothing runs yet"
    );
    let mut link = bus.open_dap(&device).unwrap();
    link.exchange(&[0x90, 0, 0xE8, 3, 0, 0]).unwrap();
    bus.sleep(Duration::from_millis(250));

    assert_eq!(aux.exchange(&[0x90, 10, 0]).unwrap(), [0x90, 0, 0]);
    let Some(Message::Header(header)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no header");
    };
    assert!(
        header.first_sequence > 200,
        "streams from the next pair: {}",
        header.first_sequence
    );
    assert_eq!(aux.exchange(&[0x90, 7]).unwrap(), [0x90, 0, 0]);
    let ended = until(&mut aux, |m| matches!(m, Message::End(_)));
    assert!(matches!(ended.last(), Some(Message::End(end)) if end.reason == EndReason::Asked));
    assert_eq!(
        aux.exchange(&[0x90, 1]).unwrap()[2],
        0,
        "the capture runs on"
    );

    aux.exchange(&[0x90, 10, 0]).unwrap();
    link.exchange(&[0x90, 0, 0xE8, 3, 0, 0]).unwrap();
    let ended = until(&mut aux, |m| matches!(m, Message::End(_)));
    assert!(
        matches!(ended.last(), Some(Message::End(end)) if end.reason == EndReason::Restarted),
        "{ended:?}"
    );
}

/// What the program before this one left waiting is dropped as the interface is opened.
#[test]
fn opening_drops_what_the_last_program_left() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut first = bus.open_aux(&device).unwrap();
    escapement_verge_probe::verge_aux::AuxLink::write(&mut first, &[0x82]).unwrap();
    drop(first);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    assert_eq!(aux.exchange(&[0x8F, 2]).unwrap()[0], 0x8F);
}

/// A program that asks while a stream message waits unread, and goes before its reply, leaves the
/// probe's OUT endpoint shut until a host reads what waits: the opener reads before it writes.
#[test]
fn opening_reads_past_a_reply_held_behind_a_stream_message() {
    use escapement_verge_probe::verge_aux::AuxLink;
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut first = bus.open_aux(&device).unwrap();
    first.write(&rate(1000)).unwrap();
    assert_eq!(first.read(Duration::from_secs(1)).unwrap()[..2], [0x90, 0]);
    bus.sleep(Duration::from_millis(50));
    first.write(&[0x90, 1]).unwrap();
    drop(first);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    assert_eq!(aux.exchange(&[0x90, 1]).unwrap()[..3], [0x90, 0, 0]);
}

/// The typed stream calls: modes 9, 10 and 7 through `VergeControl`, and the messages through its
/// transport between them. CMSIS-DAP refuses them.
#[test]
fn the_typed_stream_calls_drive_the_stream() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut verge = VergeControl::new(&mut aux);
    assert!(matches!(
        verge.stream_energy_capture_identified(0),
        Err(escapement_verge_probe::CommandError::Firmware(
            escapement_verge_probe::EnergyError::Stopped
        ))
    ));
    assert_eq!(
        verge
            .start_energy_stream_identified(2000, 0)
            .unwrap()
            .actual_rate_pairs_per_second,
        2000
    );
    assert!(matches!(
        verge.transport().message(Duration::from_secs(1)).unwrap(),
        Some(Message::Header(header)) if header.rate() == 2000.0
    ));
    verge.end_energy_stream().unwrap();
    let ended = until(verge.transport(), |m| matches!(m, Message::End(_)));
    assert!(matches!(ended.last(), Some(Message::End(end)) if end.reason == EndReason::Asked));
    drop(verge);

    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    assert!(
        VergeControl::new(&mut transport)
            .start_energy_stream_identified(1000, 0)
            .is_err(),
        "mode 9 is Verge Aux's alone"
    );
}

#[test]
fn driving_a_pin_during_a_stream_reports_its_edge() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut control = VergeControl::new(&mut aux);
    control.start_energy_stream_identified(1000, 7).unwrap();
    control.access_dio(7, 7, 1).unwrap();
    control.access_dio(1, 1, 0).unwrap();
    control.end_energy_stream().unwrap();
    let messages = until(control.transport(), |m| matches!(m, Message::End(_)));
    let levels: Vec<_> = messages
        .iter()
        .filter_map(|m| match m {
            Message::Edges(edges) => Some(edges.edges.iter().map(|e| e.levels)),
            _ => None,
        })
        .flatten()
        .collect();
    assert_eq!(levels, [1, 0]);
}

/// The simulated target marks its 10 ms awake every 100 ms on DIO3. A stream that asks for it
/// gets those edges, each after the pairs message whose last pair it does not come after, and
/// before the next.
#[test]
fn a_stream_reports_the_edges_it_asked_for_after_their_pairs() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut verge = VergeControl::new(&mut aux);
    verge.set_target_power(true).unwrap();
    verge.start_energy_stream_identified(1000, 0x0A).unwrap();
    let header = match verge.transport().message(Duration::from_secs(1)).unwrap() {
        Some(Message::Header(header)) => header,
        other => panic!("{other:?}"),
    };
    assert_eq!(
        (
            header.dio_mask,
            header.dio_levels,
            header.edge_resolution_ns
        ),
        (0x0A, 0x08, 11)
    );
    bus.sleep(Duration::from_millis(350));
    verge.end_energy_stream().unwrap();
    let messages = until(verge.transport(), |m| matches!(m, Message::End(_)));
    let edges: Vec<escapement_verge_probe::stream::Edge> = messages
        .iter()
        .filter_map(|message| match message {
            Message::Edges(edges) => {
                assert_eq!(edges.flags, 0);
                Some(edges.edges.clone())
            }
            _ => None,
        })
        .flatten()
        .collect();
    // When each pairs message arrives, every edge up to the last pair before it has arrived.
    let (mut received, mut through) = (0, None);
    for message in &messages {
        match message {
            Message::Pairs(pairs) => {
                if let Some(through) = through {
                    let due = edges.iter().filter(|edge| edge.time_ns <= through).count();
                    assert!(
                        received >= due,
                        "{received} edges before pairs needing {due}"
                    );
                }
                through = Some(pairs.time_of(pairs.pairs.len() - 1, &header));
            }
            Message::Edges(message) => received += message.edges.len(),
            _ => {}
        }
    }
    let times: Vec<(u64, u8)> = edges
        .iter()
        .map(|edge| (edge.time_ns / 1_000_000, edge.levels))
        .collect();
    assert_eq!(
        times,
        [
            (10, 0),
            (100, 8),
            (110, 0),
            (200, 8),
            (210, 0),
            (300, 8),
            (310, 0)
        ][..times.len()],
        "falls at 10 ms past each 100 and rises on each 100, DIO1 never moving"
    );
    assert!(times.len() >= 5, "{times:?}");
}

/// At a slow rate a pairs message spans seconds, and the edges after it more than an edges
/// message's offsets reach: they come in more messages, none spanning more than 2^32 ns.
#[test]
fn edges_of_a_slow_stream_split_by_span() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut verge = VergeControl::new(&mut aux);
    verge.set_target_power(true).unwrap();
    verge.start_energy_stream_identified(10, 0x08).unwrap();
    bus.sleep(Duration::from_secs(12));
    verge.end_energy_stream().unwrap();
    let messages = until(verge.transport(), |m| matches!(m, Message::End(_)));
    let mut edges = 0;
    for message in &messages {
        if let Message::Edges(message) = message {
            let span = message.edges.last().unwrap().time_ns - message.edges[0].time_ns;
            assert!(span <= u64::from(u32::MAX), "{span} ns in one message");
            edges += message.edges.len();
        }
    }
    assert!(edges >= 200, "{edges} edges");
}

/// A stream the last program left running, as one killed mid-recording leaves it, is ended as
/// the interface is opened: nothing else can hold the interface, so nothing else reads it.
#[test]
fn opening_ends_a_stream_the_last_program_left_running() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut first = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    first.exchange(&rate(1000)).unwrap();
    drop(first);
    bus.sleep(Duration::from_millis(500));
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    assert_eq!(
        aux.message(Duration::from_millis(500)).unwrap(),
        None,
        "the stream still runs"
    );
    assert_eq!(
        aux.exchange(&[0x90, 1]).unwrap()[2],
        0,
        "the capture runs on"
    );
}

/// A stream so slow that it stays quiet through the drain is ended too: the opener asks whether
/// or not one shows.
#[test]
fn opening_ends_a_stream_too_slow_to_show() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut first = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    first.exchange(&rate(1)).unwrap();
    assert!(matches!(
        first.message(Duration::from_secs(1)).unwrap(),
        Some(Message::Header(_))
    ));
    drop(first);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    assert_eq!(
        aux.message(Duration::from_secs(3)).unwrap(),
        None,
        "the slow stream still runs"
    );
}

/// The probe keeps 14 pairs messages for a host that does not read: the rest are lost, which the
/// sequence shows, and the end still says which pair was the last.
#[test]
fn pairs_the_host_did_not_read_in_time_are_lost() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    aux.exchange(&[0x8F, 1]).unwrap();
    aux.exchange(&rate(100_000)).unwrap();
    bus.sleep(Duration::from_millis(500));
    aux.exchange(&[0x90, 2]).unwrap();
    let mut streamed = 0u32;
    let mut last = 0;
    let ended = until(&mut aux, |m| matches!(m, Message::End(_)));
    for message in &ended {
        if let Message::Pairs(pairs) = message {
            streamed += pairs.pairs.len() as u32;
            last = pairs.first_sequence + pairs.pairs.len() as u32 - 1;
        }
    }
    let Some(Message::End(end)) = ended.last() else {
        panic!("no end");
    };
    assert!(streamed <= 14 * 99, "{streamed} pairs kept");
    assert!(
        end.last_sequence > last && end.last_sequence >= 49_000,
        "{end:?} after {last}"
    );
}

/// A start on CMSIS-DAP ends the stream after the old capture's last pairs, with the last of them
/// in the end.
#[test]
fn a_restart_streams_the_old_captures_last_pairs_first() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    aux.exchange(&rate(1000)).unwrap();
    let _ = aux.message(Duration::from_secs(1)).unwrap();
    bus.sleep(Duration::from_millis(300));
    let mut link = bus.open_dap(&device).unwrap();
    link.exchange(&[0x90, 0, 0xE8, 3, 0, 0]).unwrap();
    let ended = until(&mut aux, |m| matches!(m, Message::End(_)));
    let mut next = 1;
    for message in &ended {
        if let Message::Pairs(pairs) = message {
            assert_eq!(pairs.first_sequence, next, "a gap");
            next += pairs.pairs.len() as u32;
        }
    }
    let Some(Message::End(end)) = ended.last() else {
        panic!("no end");
    };
    assert_eq!(end.reason, EndReason::Restarted);
    assert_eq!(end.last_sequence, next - 1);
    assert!(end.last_sequence >= 300, "{end:?}");
}

/// A reply the last program never read waits on the interface for the next one, which drops it.
#[test]
fn a_reply_left_unread_outlasts_its_program() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut first = bus.open_aux(&device).unwrap();
    escapement_verge_probe::verge_aux::AuxLink::write(&mut first, &[0x82]).unwrap();
    drop(first);
    let mut raw = bus.open_aux(&device).unwrap();
    assert_eq!(
        escapement_verge_probe::verge_aux::AuxLink::read(&mut raw, Duration::from_millis(50))
            .unwrap()[0],
        0x82,
        "the probe keeps it"
    );
}

/// A marker from either interface goes into the stream at the moment it ran; without a stream it
/// goes nowhere and says why.
#[test]
fn markers_reach_the_stream_from_either_interface() {
    let bus = SimBus::default();
    let device = pro(&bus);
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut control = VergeControl::new(&mut aux);
    assert_eq!(control.mark(1, b"early").unwrap(), Marked::NoCapture);
    assert!(control.info().unwrap().unwrap().supports_markers());
    assert!(matches!(
        control.mark(1, &[0; 33]),
        Err(CommandError::InvalidRequest(_))
    ));
    control.start_energy_capture(1000).unwrap();
    assert_eq!(control.mark(2, b"").unwrap(), Marked::NoStream);
    control.stream_energy_capture_identified(0).unwrap();
    assert_eq!(control.mark(3, b"from aux").unwrap(), Marked::Sent);
    drop(control);
    {
        let mut link = bus.open_dap(&device).unwrap();
        let mut transport = DapTransport(&mut link);
        let mut debugger = VergeControl::new(&mut transport);
        assert_eq!(debugger.mark(4, b"from cmsis-dap").unwrap(), Marked::Sent);
    }
    let mut control = VergeControl::new(&mut aux);
    control.end_energy_stream().unwrap();
    let messages = until(control.transport(), |m| matches!(m, Message::End(_)));
    let markers: Vec<_> = messages
        .iter()
        .filter_map(|m| match m {
            Message::Markers(markers) => Some(markers.markers.iter()),
            _ => None,
        })
        .flatten()
        .map(|m| (m.number, m.label.clone(), m.time_ns))
        .collect();
    assert_eq!(
        markers
            .iter()
            .map(|(n, l, _)| (*n, l.as_slice()))
            .collect::<Vec<_>>(),
        [(3, &b"from aux"[..]), (4, &b"from cmsis-dap"[..])]
    );
    assert!(markers[0].2 < markers[1].2, "times in the order they ran");
}

#[test]
fn firmware_without_markers_is_told_apart() {
    let bus = SimBus::scenario("ours-pro-old-aux").unwrap();
    let device = detect(&bus).unwrap().remove(0).device;
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut control = VergeControl::new(&mut aux);
    assert!(!control.info().unwrap().unwrap().supports_markers());
    assert!(matches!(
        control.mark(1, b"x"),
        Err(CommandError::Unsupported(_))
    ));
}
