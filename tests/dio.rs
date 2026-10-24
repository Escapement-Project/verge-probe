//! A simulated Pro's DIO pins: they keep their state from one host to the next, and a restart
//! makes them inputs again.

use std::time::Duration;

use verge_probe::dap::{self, DapLink, Ours};
use verge_probe::sim::SimBus;
use verge_probe::{Bus, detect};

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
