//! A simulated Pro's DIO pins: they keep their state from one host to the next, and a restart
//! makes them inputs again.

use std::time::Duration;

use verge_probe::dap::{self, DapLink, Ours};
use verge_probe::sim::SimBus;
use verge_probe::{Bus, detect};

#[test]
fn aux_and_cmsis_dap_share_dio_state_and_refusals() {
    use verge_probe::{CommandError, DapTransport, VergeControl};
    let bus = SimBus::scenario("ours-pro-held").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let mut aux = verge_probe::aux::Aux::open(bus.open_aux(&unit.device).unwrap()).unwrap();
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
fn dio_refusals_explain_old_aux_j15_and_non_pro() {
    use verge_probe::{CommandError, VergeControl, pro};
    for (scenario, reason) in [
        ("ours-pro-old-aux", "predates DIO on Verge Aux"),
        ("ours-pro-j15", "remove J15"),
    ] {
        let bus = SimBus::scenario(scenario).unwrap();
        let unit = detect(&bus).unwrap().remove(0);
        let mut aux = verge_probe::aux::Aux::open(bus.open_aux(&unit.device).unwrap()).unwrap();
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
    let bus = SimBus::scenario("ours-plain").unwrap();
    assert!(
        pro::validate(&detect(&bus).unwrap()[0], pro::Feature::Dio)
            .unwrap_err()
            .to_string()
            .contains("not an MCU-Link Pro")
    );
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
