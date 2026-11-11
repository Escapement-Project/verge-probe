//! A simulated Pro's target supply and energy capture, reached through `DapTransport` as verge's
//! commands reach them: a capture runs on between hosts, and a restart turns both off.

use std::time::Duration;

use escapement_verge_probe::dap::{self, DapLink, Ours};
use escapement_verge_probe::sim::SimBus;
use escapement_verge_probe::{
    Bus, CalibrationRange, CommandError, DapTransport, EnergyCalibration, EnergyCaptureSample,
    EnergyError, EnergyTotal, VergeControl, detect,
};

#[test]
fn a_capture_runs_on_between_hosts_until_the_pro_restarts() {
    let bus = SimBus::default();
    let unit = bus.add_ours("26ea505");
    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
    let device = detect(&bus).unwrap().remove(0).device;

    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    assert!(verge.info().unwrap().unwrap().supports_energy_capture());
    assert!(!verge.read_target_power().unwrap());
    assert!(verge.set_target_power(true).unwrap());
    assert_eq!(
        verge.read_energy_accumulator().unwrap(),
        EnergyTotal::Stopped
    );
    let start = verge.start_energy_capture(1000).unwrap();
    assert_eq!(start.actual_rate_pairs_per_second, 1000);
    drop(verge);
    drop(link);

    // Another host, a second later, finds the capture still running and a total that grew.
    bus.sleep(Duration::from_secs(1));
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    let EnergyCaptureSample::Sample(sample) = verge.latest_energy_sample().unwrap() else {
        panic!("the capture stopped between hosts");
    };
    let calibrated = sample.calibrated.unwrap();
    assert_eq!(calibrated.voltage_millivolts, 3300);
    assert!(sample.raw.sequence >= 1000);
    let EnergyTotal::Total(total) = verge.read_energy_accumulator().unwrap() else {
        panic!("no total after a second of capture");
    };
    // 20 µA for 0.9 s and 5 mA for 0.1 s at 3.3 V: about 1.7 mJ.
    assert!(
        (1_600_000..1_800_000).contains(&total.energy_nanojoules),
        "{total:?}"
    );
    assert!(matches!(
        verge.read_energy_calibration(CalibrationRange::Low).unwrap(),
        EnergyCalibration::Calibrated(points) if points.len() == 3
    ));

    // A stop keeps the total.
    verge.stop_energy_capture().unwrap();
    let stopped = verge.read_energy_accumulator().unwrap();
    bus.sleep(Duration::from_secs(1));
    assert_eq!(
        verge.latest_energy_sample().unwrap(),
        EnergyCaptureSample::Stopped
    );
    assert_eq!(verge.read_energy_accumulator().unwrap(), stopped);
    drop(verge);

    // A plain restart turns the supply off and forgets the total.
    link.control_out(dap::REBOOT_REQUEST_TYPE, dap::REBOOT_REQUEST, 0, 0)
        .unwrap();
    drop(link);
    bus.sleep(Duration::from_secs(1));
    let device = detect(&bus).unwrap().remove(0).device;
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    assert!(!verge.read_target_power().unwrap());
    assert_eq!(
        verge.read_energy_accumulator().unwrap(),
        EnergyTotal::Stopped
    );
}

#[test]
fn a_plain_board_has_no_supply_or_capture() {
    let bus = SimBus::default();
    let unit = bus.add_ours("26ea505");
    bus.set_board(unit, Ours::BOARD_MCU_LINK);
    let device = detect(&bus).unwrap().remove(0).device;
    let mut link = bus.open_dap(&device).unwrap();
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    let info = verge.info().unwrap().unwrap();
    assert!(!info.supports_energy_capture());
    assert!(!info.supports_target_power());
    // Past the capability checks, the firmware refuses both itself: DAP_ERROR for the supply,
    // result 3 (unavailable) for a capture.
    assert_eq!(verge.send_vendor_command(0x8F, &[1]).unwrap(), [0xFF, 0]);
    assert_eq!(
        verge
            .send_vendor_command(0x90, &[0, 0xE8, 0x03, 0, 0])
            .unwrap(),
        [0, 3, 0, 0, 0, 0]
    );
}

fn open_pro(bus: &SimBus) -> impl DapLink + use<> {
    let device = detect(bus).unwrap().remove(0).device;
    bus.open_dap(&device).unwrap()
}

#[test]
fn the_total_keeps_what_was_drawn_while_the_supply_is_off() {
    let bus = SimBus::scenario("ours-pro-energy").unwrap();
    let mut link = open_pro(&bus);
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    let total = |verge: &mut VergeControl<'_, DapTransport<'_>>| match verge
        .read_energy_accumulator()
        .unwrap()
    {
        EnergyTotal::Total(total) => total.energy_nanojoules,
        other => panic!("{other:?}"),
    };
    let on = total(&mut verge);
    assert!(on > 17_000_000, "{on}");
    verge.set_target_power(false).unwrap();
    bus.sleep(Duration::from_secs(1));
    let off = total(&mut verge);
    // A command's 250 µs at 20 µA and 3.3 V is 16.5 nJ; nothing else is drawn while off.
    assert!((on..on + 1_000).contains(&off), "{on} then {off}");
    verge.set_target_power(true).unwrap();
    bus.sleep(Duration::from_secs(1));
    let again = total(&mut verge);
    assert!(
        (off + 1_600_000..off + 1_800_000).contains(&again),
        "{off} then {again}"
    );
}

#[test]
fn a_fault_answers_every_read_until_a_start_clears_it() {
    let bus = SimBus::scenario("ours-pro-energy-failed").unwrap();
    let mut link = open_pro(&bus);
    let mut transport = DapTransport(&mut link);
    let mut verge = VergeControl::new(&mut transport);
    fn fault<T>(result: Result<T, CommandError<EnergyError>>) -> bool {
        matches!(result, Err(CommandError::Firmware(EnergyError::DmaError)))
    }
    assert!(fault(verge.latest_energy_sample()));
    let EnergyTotal::Failed(kept) = verge.read_energy_accumulator().unwrap() else {
        panic!("a fault keeps the total");
    };
    assert_eq!(kept.integrated_pairs, 10_000);
    // A stop still stops, and still answers with the fault.
    assert!(fault(verge.stop_energy_capture()));
    assert!(fault(verge.latest_energy_sample()));
    assert!(verge.start_energy_capture(0).is_err());
    assert!(
        fault(verge.latest_energy_sample()),
        "a failed start keeps the fault"
    );
    verge.start_energy_capture(1000).unwrap();
    bus.sleep(Duration::from_millis(10));
    assert!(matches!(
        verge.latest_energy_sample().unwrap(),
        EnergyCaptureSample::Sample(_)
    ));
}
