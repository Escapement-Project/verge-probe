//! A batch whose image is a file has no version to tell its image from another build of ours.
//! So a unit back on a port running ours is the one that left only where the file went on the
//! unit that left and that unit never ran ours; any other unit on that port is a unit of its own.

use std::time::Duration;

use escapement_verge_probe::batch::{
    Armed, Batch, Board, Filter, Observer, OursFilter, Record, Settled, StockFilter,
};
use escapement_verge_probe::progress::Progress;
use escapement_verge_probe::sim::{SimBus, SimFirmware, firmware_image};
use escapement_verge_probe::source::ImageVersion;
use escapement_verge_probe::update::Timing;
use escapement_verge_probe::version::StockVersion;
use escapement_verge_probe::{Bus, Cancel};

#[derive(Default)]
struct Seen {
    records: Vec<Record>,
}

impl Progress for Seen {}

impl Observer for Seen {
    fn record(&mut self, record: &Record) {
        self.records.push(record.clone());
    }
}

fn ours(build: &str) -> SimFirmware {
    SimFirmware::Ours {
        build: build.into(),
    }
}

#[test]
fn a_failure_stays_in_the_tally_when_another_unit_takes_its_port() {
    let bus = SimBus::default();
    let armed = Armed {
        image: firmware_image(&ours("v0.3.0")),
        name: "a file".into(),
        version: ImageVersion::Unknown,
        source: escapement_verge_probe::Source::File("mculink_cjtag.s19".into()),
    };
    let filter = Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Any,
        bootloaders: false,
        attached: false,
    };
    let mut batch = Batch::arm(
        &bus,
        armed,
        filter,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    // Unit A runs ours; its write fails.
    let unit = bus.plug_in_later(Duration::from_secs(1), ours("v0.2.0"));
    bus.set_fail_writes(unit, true);
    while seen.records.is_empty() {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[0].result, Settled::Failed);
    // A is set aside. Unit B, another serial, running ours, goes into the port.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.reflash(unit, &ours("v0.2.0"));
    bus.set_serials(unit, "CJBBBBBBBB", "BBBBBBBBBBBBB");
    bus.set_fail_writes(unit, false);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 2 {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[1].result, Settled::Updated);
    let tally = batch.tally();
    assert_eq!((tally.updated, tally.failed), (1, 1), "{tally}");
}

#[test]
fn a_failure_on_stock_stays_in_the_tally_when_a_unit_running_ours_takes_its_port() {
    let bus = SimBus::default();
    let armed = Armed {
        image: firmware_image(&ours("v0.3.0")),
        name: "a file".into(),
        version: ImageVersion::Unknown,
        source: escapement_verge_probe::Source::File("mculink_cjtag.s19".into()),
    };
    let filter = Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Any,
        bootloaders: false,
        attached: false,
    };
    let mut batch = Batch::arm(
        &bus,
        armed,
        filter,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    // Unit A runs stock; its write fails, so the file never went on it.
    let unit = bus.plug_in_later(
        Duration::from_secs(1),
        SimFirmware::Stock {
            version: StockVersion::parse("3.172").unwrap(),
        },
    );
    bus.set_fail_writes(unit, true);
    while seen.records.is_empty() {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[0].result, Settled::Failed);
    // A is set aside. Unit B, another serial, running a build of ours, goes into the port.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.reflash(unit, &ours("v0.2.0"));
    bus.set_serials(unit, "CJBBBBBBBB", "BBBBBBBBBBBBB");
    bus.set_fail_writes(unit, false);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 2 {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[1].result, Settled::Updated);
    let tally = batch.tally();
    assert_eq!((tally.updated, tally.failed), (1, 1), "{tally}");
}

#[test]
fn another_unit_running_ours_on_the_port_is_a_unit_of_its_own() {
    let bus = SimBus::default();
    let armed = Armed {
        image: firmware_image(&ours("v0.3.0")),
        name: "a file".into(),
        version: ImageVersion::Unknown,
        source: escapement_verge_probe::Source::File("mculink_cjtag.s19".into()),
    };
    let filter = Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Any,
        bootloaders: false,
        attached: false,
    };
    let mut batch = Batch::arm(
        &bus,
        armed,
        filter,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    // Unit A does not come back from its update.
    bus.set_boot_after_reset(Duration::from_secs(20));
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(
        Duration::from_secs(1),
        SimFirmware::Stock {
            version: StockVersion::parse("3.172").unwrap(),
        },
    );
    while seen.records.is_empty() {
        batch.step(&mut seen).unwrap();
    }
    assert!(
        seen.records[0]
            .reason
            .as_deref()
            .unwrap()
            .contains("did not come back")
    );
    bus.set_boot_after_reset(Duration::from_secs(1));
    bus.unplug(unit);
    // Past the restart A had scheduled, so that it cannot land in a later update.
    let until = bus.now() + Duration::from_secs(10);
    while bus.now() < until {
        batch.step(&mut seen).unwrap();
    }
    // Unit B, running an older build of ours, goes into the port; its write fails.
    bus.reflash(unit, &ours("v0.2.0"));
    bus.set_serials(unit, "CJBBBBBBBB", "BBBBBBBBBBBBB");
    bus.set_fail_writes(unit, true);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 2 {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[1].result, Settled::Failed);
    // Unit C, running ours, nothing wrong with it.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.reflash(unit, &ours("v0.2.0"));
    bus.set_serials(unit, "CJCCCCCCCC", "CCCCCCCCCCCCC");
    bus.set_fail_writes(unit, false);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 3 {
        batch.step(&mut seen).unwrap();
    }
    let c = &seen.records[2];
    assert_eq!(c.serial_before.as_deref(), Some("CJCCCCCCCC"));
    assert_eq!(
        c.result,
        Settled::Updated,
        "unit C: {:?}, tally {}",
        c.reason,
        batch.tally()
    );
}
