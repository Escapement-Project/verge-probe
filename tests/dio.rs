//! A simulated Pro's DIO pins: they keep their state from one host to the next, and a restart
//! makes them inputs again.

use std::time::Duration;

use escapement_verge_probe::dap::{self, DapLink, Ours};
use escapement_verge_probe::sim::SimBus;
use escapement_verge_probe::{Bus, detect};

#[test]
fn aux_and_cmsis_dap_share_dio_state_and_refusals() {
    use escapement_verge_probe::{CommandError, DapTransport, VergeControl};
    let bus = SimBus::scenario("ours-pro-held").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let mut aux =
        escapement_verge_probe::verge_aux::Aux::open(bus.open_aux(&unit.device).unwrap()).unwrap();
    let mut control = VergeControl::new(&mut aux);
    let info = control.info().unwrap().unwrap();
    assert!(info.supports_dio_on_aux());
    assert!(info.flag_names().contains(&"DIO on Aux"));
    assert_eq!(control.access_dio(0, 0, 0).unwrap().read, 8);
    assert!(matches!(
        control.access_dio(1, 1, 1),
        Err(CommandError::Firmware(_))
    ));
    assert_eq!(control.access_dio(7, 7, 5).unwrap().read, 13);
    let mut dap = bus.open_dap(&unit.device).unwrap();
    let mut transport = DapTransport(&mut dap);
    assert_eq!(
        VergeControl::new(&mut transport)
            .access_dio(0, 0, 0)
            .unwrap()
            .outputs,
        7
    );
}

#[test]
fn dio_refusals_explain_old_aux_j15_and_a_board_without_them() {
    use escapement_verge_probe::{CommandError, VergeControl, pro};
    for (scenario, reason) in [
        ("ours-pro-old-aux", "predates DIO on Verge Aux"),
        ("ours-pro-j15", "take J15 off"),
    ] {
        let bus = SimBus::scenario(scenario).unwrap();
        let unit = detect(&bus).unwrap().remove(0);
        let mut aux =
            escapement_verge_probe::verge_aux::Aux::open(bus.open_aux(&unit.device).unwrap())
                .unwrap();
        let mut control = VergeControl::new(&mut aux);
        assert!(
            pro::check_offered("unit", &mut control, pro::Feature::Dio)
                .unwrap_err()
                .to_string()
                .contains(reason)
        );
        assert!(matches!(
            control.access_dio(0, 0, 0),
            Err(CommandError::Unsupported(_))
        ));
        assert!(!bus.log().iter().any(|line| line.contains("AUX 8e")));
    }
    // The standalone MCU-Link has them on J8; an on-board one has none.
    let bus = SimBus::default();
    let unit = bus.add_ours("26ea505");
    bus.set_board(unit, Ours::BOARD_ON_BOARD);
    assert!(
        pro::validate(&detect(&bus).unwrap()[0], pro::Feature::Dio)
            .unwrap_err()
            .to_string()
            .contains("has no DIO pins")
    );
    let bus = SimBus::scenario("ours-plain").unwrap();
    assert!(pro::validate(&detect(&bus).unwrap()[0], pro::Feature::Dio).is_ok());
}

#[test]
fn a_pro_keeps_its_dio_pins_until_it_restarts() {
    let bus = SimBus::default();
    let unit = bus.add_ours("26ea505");
    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
    let device = detect(&bus).unwrap().remove(0).device;

    let mut link = bus.open_dap(&device).unwrap();
    let set = dap::dio(&mut link, 0x0F, 0x08, 0x08).unwrap().unwrap();
    assert!(set.accepted());
    drop(link);

    // Another host, later, sees what the first left driving.
    let mut link = bus.open_dap(&device).unwrap();
    let left = dap::dio(&mut link, 0, 0, 0).unwrap().unwrap();
    assert_eq!((left.outputs, left.levels, left.read), (0x08, 0x08, 0x08));

    // A plain restart, not into the bootloader.
    link.control_out(dap::REBOOT_REQUEST_TYPE, dap::REBOOT_REQUEST, 0, 0)
        .unwrap();
    drop(link);
    bus.sleep(Duration::from_secs(1));
    let device = detect(&bus).unwrap().remove(0).device;
    let mut link = bus.open_dap(&device).unwrap();
    let after = dap::dio(&mut link, 0, 0, 0).unwrap().unwrap();
    assert_eq!((after.outputs, after.levels), (0, 0));
}

/// A read through CMSIS-DAP while a DIO stream runs shows what is attached, as the stream does.
#[test]
fn a_read_beside_a_dio_stream_shows_a_toggling_pin() {
    use escapement_verge_probe::{VergeControl, pro};
    let bus = SimBus::scenario("ours-plain-busy").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let mut aux = pro::open_aux(&bus, &unit, pro::Feature::Dio).unwrap();
    VergeControl::new(&mut aux).start_dio_stream(0x0F).unwrap();
    // DIO1 toggles high 50 ms in.
    bus.sleep(Duration::from_millis(60));
    let mut link = bus.open_dap(&unit.device).unwrap();
    assert_eq!(dap::dio(&mut link, 0, 0, 0).unwrap().unwrap().read, 0b0010);
}

/// A standalone MCU-Link streams its DIO pins alone (`0x90` mode 15): a format 5 header with the
/// pins asked for, an edge as a host drives one through CMSIS-DAP, a restart that drops the old
/// stream's messages, and an end that says when. A Pro, which records its pins beside energy,
/// refuses it.
#[test]
fn a_standalone_unit_streams_its_dio_pins_alone() {
    use escapement_verge_probe::stream::{EndReason, Message};
    use escapement_verge_probe::{CommandError, VergeControl, pro};
    let bus = SimBus::scenario("ours-plain").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let mut aux = pro::open_aux(&bus, &unit, pro::Feature::Dio).unwrap();
    let mut control = VergeControl::new(&mut aux);
    let info = control.info().unwrap().unwrap();
    assert!(info.supports_dio_stream() && info.flag_names().contains(&"DIO streams"));
    control.start_dio_stream(0b0011).unwrap();
    let Some(Message::DioHeader(header)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no DIO header");
    };
    assert_eq!((header.dio_mask, header.dio_levels), (0b0011, 0));

    bus.sleep(Duration::from_millis(5));
    let mut link = bus.open_dap(&unit.device).unwrap();
    assert!(
        dap::dio(&mut link, 0b0001, 0b0001, 0b0001)
            .unwrap()
            .unwrap()
            .accepted()
    );
    let Some(Message::Edges(edges)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no edge");
    };
    assert_eq!(edges.edges.len(), 1);
    assert_eq!(edges.edges[0].levels, 0b0001);
    assert!(edges.edges[0].time_ns >= 5_000_000);
    // DIO2 is not in the stream.
    assert!(
        dap::dio(&mut link, 0b0100, 0b0100, 0b0100)
            .unwrap()
            .unwrap()
            .accepted()
    );

    VergeControl::new(&mut aux).end_dio_stream().unwrap();
    let Some(Message::End(end)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no end");
    };
    assert_eq!((end.reason, end.last_sequence), (EndReason::Asked, 0));
    assert!(end.time_ns.is_some_and(|ns| ns >= edges.edges[0].time_ns));

    // A new start drops what the last stream sent before its reply, as the probe drops what it
    // queued: here an edge kept while another reply was awaited.
    VergeControl::new(&mut aux)
        .start_dio_stream(0b0011)
        .unwrap();
    let Some(Message::DioHeader(_)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("no DIO header");
    };
    assert!(
        dap::dio(&mut link, 0b0001, 0b0001, 0)
            .unwrap()
            .unwrap()
            .accepted()
    );
    VergeControl::new(&mut aux).access_dio(0, 0, 0).unwrap();
    VergeControl::new(&mut aux)
        .start_dio_stream(0b0011)
        .unwrap();
    let Some(Message::DioHeader(_)) = aux.message(Duration::from_secs(1)).unwrap() else {
        panic!("the new stream's header did not come first");
    };

    let bus = SimBus::scenario("ours-pro").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let mut aux = pro::open_aux(&bus, &unit, pro::Feature::Dio).unwrap();
    assert!(matches!(
        VergeControl::new(&mut aux).start_dio_stream(0b0001),
        Err(CommandError::Unsupported(_))
    ));
}
