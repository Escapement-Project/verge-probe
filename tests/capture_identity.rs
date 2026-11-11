use escapement_verge_probe::aux::{Aux, AuxLink};
use escapement_verge_probe::dap::{self, DapLink, Ours};
use escapement_verge_probe::sim::{SimAux, SimBus};
use escapement_verge_probe::stream::Message;
use escapement_verge_probe::{Bus, CommandError, DapTransport, EnergyError, VergeControl, detect};
use std::time::Duration;

fn pro() -> (SimBus, escapement_verge_probe::UsbDevice) {
    let bus = SimBus::default();
    let unit = bus.add_ours("identity");
    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
    let device = detect(&bus).unwrap().remove(0).device;
    (bus, device)
}

#[test]
fn stale_cleanup_leaves_a_replacement_running_on_both_interfaces() {
    let (bus, device) = pro();
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let a = VergeControl::new(&mut aux)
        .start_energy_stream_identified(1000, 0)
        .unwrap()
        .capture_id
        .unwrap();
    let Message::Header(header) = aux.message(Duration::from_secs(1)).unwrap().unwrap() else {
        panic!("missing header A")
    };
    assert_eq!(header.capture_id, Some(a));
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut second = VergeControl::new(&mut transport);
    let b = second
        .start_energy_capture_identified(2000)
        .unwrap()
        .capture_id
        .unwrap();
    assert_ne!(a, b);
    for stream in [false, true] {
        let mut first = VergeControl::new(&mut aux);
        let stale = if stream {
            first.end_energy_stream_if(a)
        } else {
            first.stop_energy_capture_if(a)
        };
        assert!(matches!(
            stale,
            Err(CommandError::Firmware(EnergyError::StaleCapture))
        ));
        let state = second.energy_capture_status().unwrap();
        assert!(state.running);
        assert_eq!(state.capture_id, Some(b));
        assert_eq!(state.actual_rate_pairs_per_second, 2000);
    }
    assert!(matches!(
        second.stop_energy_capture_if(a),
        Err(CommandError::Firmware(EnergyError::StaleCapture))
    ));
    second.stop_energy_capture_if(b).unwrap();
    second.stop_energy_capture_if(b).unwrap();
    let stopped = second.energy_capture_status().unwrap();
    assert!(!stopped.running);
    assert_eq!(stopped.capture_id, Some(b));
}

#[test]
fn saved_pre_reply_headers_are_discarded_when_attaching() {
    let (bus, device) = pro();
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let a = VergeControl::new(&mut aux)
        .start_energy_stream_identified(1000, 0)
        .unwrap()
        .capture_id
        .unwrap();
    // This status exchange saves A's pending header inside Aux, before B exists.
    VergeControl::new(&mut aux).energy_capture_status().unwrap();
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let b = VergeControl::new(&mut transport)
        .start_energy_capture_identified(2000)
        .unwrap()
        .capture_id
        .unwrap();
    VergeControl::new(&mut aux)
        .stream_energy_capture_identified(0)
        .unwrap();
    let Message::Header(header) = aux.message(Duration::from_secs(1)).unwrap().unwrap() else {
        panic!("missing B header")
    };
    assert_ne!(a, b);
    assert_eq!(header.capture_id, Some(b));
    assert_eq!(header.rate, 2000);
    assert!(!matches!(
        aux.message(Duration::from_millis(1)).unwrap(),
        Some(Message::Header(_))
    ));
}

#[test]
fn reboot_changes_the_boot_value_even_when_the_counter_repeats() {
    let (bus, device) = pro();
    let mut link = bus.open_dap(&device).unwrap();
    let a = VergeControl::new(&mut DapTransport(&mut link))
        .start_energy_capture_identified(1000)
        .unwrap()
        .capture_id
        .unwrap();
    link.control_out(dap::REBOOT_REQUEST_TYPE, dap::REBOOT_REQUEST, 0, 0)
        .unwrap();
    drop(link);
    bus.sleep(Duration::from_secs(1));
    let device = detect(&bus).unwrap().remove(0).device;
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut control = VergeControl::new(&mut transport);
    let b = control
        .start_energy_capture_identified(1000)
        .unwrap()
        .capture_id
        .unwrap();
    assert_eq!(a.as_bytes()[16..], b.as_bytes()[16..]);
    assert_ne!(a.as_bytes()[..16], b.as_bytes()[..16]);
    assert!(matches!(
        control.stop_energy_capture_if(a),
        Err(CommandError::Firmware(EnergyError::StaleCapture))
    ));
    assert!(control.energy_capture_status().unwrap().running);
}

struct LostStart {
    inner: SimAux,
    lose: bool,
}
impl AuxLink for LostStart {
    fn write(&mut self, request: &[u8]) -> Result<(), dap::LinkError> {
        self.lose = matches!(request.get(..2), Some([0x90, 9 | 10]));
        self.inner.write(request)
    }
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, dap::LinkError> {
        let reply = self.inner.read(timeout)?;
        if self.lose && reply.starts_with(&[0x90, 0, 0]) {
            self.lose = false;
            return Err(dap::LinkError::Timeout);
        }
        Ok(reply)
    }
}

#[test]
fn a_lost_start_reply_never_grants_cleanup_authority() {
    let (bus, device) = pro();
    let mut aux = Aux::open(LostStart {
        inner: bus.open_aux(&device).unwrap(),
        lose: false,
    })
    .unwrap();
    let mut first = VergeControl::new(&mut aux);
    assert!(first.start_energy_stream_identified(1000, 0).is_err());
    assert!(first.tracked_capture_id().is_none());
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut second = VergeControl::new(&mut transport);
    let b = second
        .start_energy_capture_identified(2000)
        .unwrap()
        .capture_id;
    assert!(matches!(
        first.stop_tracked_energy_capture(),
        Err(CommandError::InvalidRequest(_))
    ));
    assert!(matches!(
        first.end_tracked_energy_stream(),
        Err(CommandError::InvalidRequest(_))
    ));
    assert_eq!(second.energy_capture_status().unwrap().capture_id, b);
    assert!(second.energy_capture_status().unwrap().running);
    assert!(
        !bus.log()
            .iter()
            .any(|line| line.contains("AUX 900b") || line.contains("AUX 900d"))
    );
}

#[test]
fn older_firmware_is_readable_but_refused_before_identified_lifecycle_commands() {
    let bus = SimBus::scenario("ours-pro-old-aux").unwrap();
    let device = detect(&bus).unwrap().remove(0).device;
    let mut aux = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    let mut first = VergeControl::new(&mut aux);
    assert!(!first.info().unwrap().unwrap().supports_capture_identity());
    assert!(matches!(
        first.start_energy_capture_identified(1000),
        Err(CommandError::Unsupported(_))
    ));
    assert!(matches!(
        first.stream_energy_capture_identified(0),
        Err(CommandError::Unsupported(_))
    ));
    first.start_energy_capture(1000).unwrap();
    let mut link = bus.open_dap(&device).unwrap();
    assert_eq!(link.exchange(&[0x90, 12]).unwrap(), [0x90, 0xff]);
    assert_eq!(aux.exchange(&[0x90, 12]).unwrap(), [0x90, 0xff]);
    assert!(
        !bus.log()
            .iter()
            .any(|line| line.contains("AUX 9008") || line.contains("AUX 900a"))
    );
}

#[test]
fn a_lost_attach_reply_cannot_adopt_the_queued_header() {
    let (bus, device) = pro();
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut owner = VergeControl::new(&mut transport);
    let id = owner
        .start_energy_capture_identified(1000)
        .unwrap()
        .capture_id;
    let mut aux = Aux::open(LostStart {
        inner: bus.open_aux(&device).unwrap(),
        lose: false,
    })
    .unwrap();
    let mut attaching = VergeControl::new(&mut aux);
    assert!(attaching.stream_energy_capture_identified(0).is_err());
    assert!(!attaching.awaiting_energy_header());
    let Some(Message::Header(header)) = attaching
        .transport()
        .message(Duration::from_secs(1))
        .unwrap()
    else {
        panic!("queued header")
    };
    assert!(matches!(
        attaching.accept_energy_header(&header),
        Err(CommandError::InvalidRequest(_))
    ));
    assert!(attaching.tracked_capture_id().is_none());
    assert!(attaching.end_tracked_energy_stream().is_err());
    assert_eq!(owner.energy_capture_status().unwrap().capture_id, id);
    assert!(owner.energy_capture_status().unwrap().running);
}

#[test]
#[ignore = "uses USB; VERGE_BENCH_SERIAL must name an idle Pro"]
fn hardware_replacement_survives_stale_stop_on_aux_and_cmsis_dap() {
    let serial = std::env::var("VERGE_BENCH_SERIAL").expect("explicit Pro serial");
    let bus = escapement_verge_probe::usb::UsbBus::default();
    let unit = escapement_verge_probe::detect::detect_named(&bus, Some(&serial))
        .unwrap()
        .remove(0);
    let mut link = bus.open_dap(&unit.device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut second = VergeControl::new(&mut transport);
    assert_eq!(
        second.info().unwrap().unwrap().board,
        Some(escapement_verge_probe::BoardKind::Pro)
    );
    assert!(second.info().unwrap().unwrap().supports_capture_identity());
    let initial = second.energy_capture_status().unwrap();
    assert!(!initial.running, "leave an existing capture to its owner");
    assert!(
        second.read_target_power().unwrap(),
        "target supply must already be on"
    );
    let mut aux = Aux::open(bus.open_aux(&unit.device).unwrap()).unwrap();
    let mut first = VergeControl::new(&mut aux);
    for rate in [1000, 100_000] {
        let began = std::time::Instant::now();
        let a = first
            .start_energy_stream_identified(rate, 8)
            .unwrap()
            .capture_id
            .unwrap();
        let start_ms = began.elapsed().as_secs_f64() * 1000.0;
        let Some(Message::Header(header)) =
            first.transport().message(Duration::from_secs(2)).unwrap()
        else {
            panic!("A header")
        };
        assert_eq!(header.capture_id, Some(a));
        first.accept_energy_header(&header).unwrap();
        let mut pairs = 0;
        let mut next_sequence = header.first_sequence;
        let mut edges = 0;
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while std::time::Instant::now() < deadline {
            match first
                .transport()
                .message(Duration::from_millis(20))
                .unwrap()
            {
                Some(Message::Pairs(message)) => {
                    assert_eq!(message.first_sequence, next_sequence, "lost pairs");
                    pairs += message.pairs.len();
                    next_sequence = next_sequence.wrapping_add(message.pairs.len() as u32);
                }
                Some(Message::Edges(message)) => {
                    assert!(!message.lost());
                    edges += message.edges.len();
                }
                _ => {}
            }
        }
        assert!(
            pairs >= rate as usize / 2,
            "no sustained recording: {pairs}"
        );
        assert!(edges > 0, "no input edges recorded");
        let b = second
            .start_energy_capture_identified(2000)
            .unwrap()
            .capture_id
            .unwrap();
        assert_ne!(a, b);
        assert!(matches!(
            first.stop_energy_capture_if(a),
            Err(CommandError::Firmware(EnergyError::StaleCapture))
        ));
        assert!(matches!(
            first.end_energy_stream_if(a),
            Err(CommandError::Firmware(EnergyError::StaleCapture))
        ));
        assert!(matches!(
            second.stop_energy_capture_if(a),
            Err(CommandError::Firmware(EnergyError::StaleCapture))
        ));
        let current = second.energy_capture_status().unwrap();
        assert!(current.running);
        assert_eq!(current.capture_id, Some(b));
        assert_eq!(current.actual_rate_pairs_per_second, 2000);
        first.stream_energy_capture_identified(8).unwrap();
        let Some(Message::Header(header)) =
            first.transport().message(Duration::from_secs(2)).unwrap()
        else {
            panic!("B header")
        };
        assert_eq!(header.capture_id, Some(b));
        first.accept_energy_header(&header).unwrap();
        first.end_tracked_energy_stream().unwrap();
        assert!(second.energy_capture_status().unwrap().running);
        second.stop_energy_capture_if(b).unwrap();
        second.stop_energy_capture_if(b).unwrap();
        eprintln!(
            "{rate} pairs/s: {pairs} pairs, {edges} input edges; start {start_ms:.3} ms; stale stops left replacement running"
        );
    }
}
