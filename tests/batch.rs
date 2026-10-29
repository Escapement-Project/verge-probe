//! Batches on the simulated bench, with units plugged in and out over simulated time. Every run
//! is bounded in the bus's time; none takes real time to time out.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;

use verge_probe::batch::{
    Armed, Batch, Board, Entry, Filter, Observer, OursFilter, Record, Settled, Status, StockFilter,
    Verdict, judge,
};
use verge_probe::bus::{BusError, UsbDevice};
use verge_probe::dap::{LinkError, Ours};
use verge_probe::progress::Progress;
use verge_probe::sim::{SimBus, SimDap, SimFirmware, SimRom, firmware_image};
use verge_probe::source::{ImageVersion, Source};
use verge_probe::update::Timing;
use verge_probe::version::StockVersion;
use verge_probe::{Bus, Cancel};

#[derive(Default)]
struct Seen {
    records: Vec<Record>,
    started: Vec<String>,
    pauses: Vec<Option<String>>,
    unplugs: Vec<(String, bool)>,
}

impl Progress for Seen {}

impl Observer for Seen {
    fn starting(&mut self, entry: &Entry) {
        self.started.push(entry.port.clone());
    }
    fn record(&mut self, record: &Record) {
        self.records.push(record.clone());
    }
    fn unplug(&mut self, port: &str, blinking: bool) {
        self.unplugs.push((port.to_owned(), blinking));
    }
    fn paused(&mut self, reason: Option<&str>) {
        self.pauses.push(reason.map(str::to_owned));
    }
}

/// The simulated bench with hooks: on every `open_dap`, and a say in whether `devices` fails.
/// A batch blinks on a thread of its own, so the hooks may be called from one.
struct Hooked<'a> {
    bus: &'a SimBus,
    /// When each `open_dap` came, in the bus's time, and on which instance.
    opened: Mutex<Vec<(Duration, String)>>,
    on_open: Box<dyn Fn(&UsbDevice) + Sync + 'a>,
    fail: Box<dyn Fn() -> bool + Sync + 'a>,
}

impl<'a> Hooked<'a> {
    fn new(bus: &'a SimBus) -> Hooked<'a> {
        Hooked {
            bus,
            opened: Mutex::new(Vec::new()),
            on_open: Box::new(|_| {}),
            fail: Box::new(|| false),
        }
    }
}

impl Bus for Hooked<'_> {
    type Aux = verge_probe::sim::SimAux;
    type Dap = SimDap;
    type Isp = SimRom;

    fn devices(&self) -> Result<Vec<UsbDevice>, BusError> {
        if (self.fail)() {
            return Err(BusError::Enumerate("the test's bus failed".into()));
        }
        self.bus.devices()
    }

    fn open_dap(&self, device: &UsbDevice) -> Result<SimDap, BusError> {
        self.opened
            .lock()
            .unwrap()
            .push((self.bus.now(), device.instance.clone()));
        (self.on_open)(device);
        self.bus.open_dap(device)
    }

    fn open_aux(&self, device: &UsbDevice) -> Result<verge_probe::sim::SimAux, BusError> {
        self.bus.open_aux(device)
    }

    fn device_control_out(
        &self,
        device: &UsbDevice,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<Result<(), LinkError>, BusError> {
        self.bus
            .device_control_out(device, request_type, request, value, index)
    }

    fn open_isp(&self, device: &UsbDevice) -> Result<SimRom, BusError> {
        self.bus.open_isp(device)
    }

    fn now(&self) -> Duration {
        self.bus.now()
    }

    fn sleep(&self, duration: Duration) {
        self.bus.sleep(duration);
    }

    fn keeps_instances(&self) -> bool {
        self.bus.keeps_instances()
    }
}

fn ours(build: &str) -> SimFirmware {
    SimFirmware::Ours {
        build: build.to_owned(),
    }
}

fn stock(version: &str) -> SimFirmware {
    SimFirmware::Stock {
        version: StockVersion::parse(version).unwrap(),
    }
}

fn release() -> Armed {
    Armed {
        image: firmware_image(&ours("v0.3.0")),
        name: "release v0.3.0".into(),
        version: ImageVersion::Ours("v0.3.0".into()),
        source: Source::Release(None),
    }
}

fn filter() -> Filter {
    Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Older,
        bootloaders: false,
        attached: false,
    }
}

fn arm<B: Bus + Sync>(bus: &B, filter: Filter, limit: Option<usize>) -> Batch<'_, B> {
    Batch::arm(
        bus,
        release(),
        filter,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        limit.map(|count| std::num::NonZeroUsize::new(count).unwrap()),
    )
    .unwrap()
}

/// Steps until the bus's clock reaches `until`, or the batch finishes.
fn drive<B: Bus + Sync>(batch: &mut Batch<'_, B>, bus: &SimBus, seen: &mut Seen, until: Duration) {
    while bus.now() < until && !batch.finished() {
        batch.step(seen).unwrap();
    }
}

fn status<B: Bus + Sync>(batch: &Batch<'_, B>, port: &str) -> Status {
    batch
        .entries()
        .iter()
        .find(|e| e.port == port)
        .unwrap_or_else(|| panic!("no entry for {port}: {:?}", batch.entries()))
        .status
        .clone()
}

fn index_of(log: &[String], what: &str) -> usize {
    log.iter()
        .position(|l| l == what)
        .unwrap_or_else(|| panic!("no {what:?} in {log:#?}"))
}

#[test]
fn two_units_arriving_together_go_through_the_bootloader_one_at_a_time() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    let b = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));

    assert_eq!(status(&batch, "1-1"), Status::Updated);
    assert_eq!(status(&batch, "1-2"), Status::Updated);
    assert_eq!(seen.started, ["1-1", "1-2"]);
    // Never two bootloaders on the bus: the second unit was asked only once the first was back.
    assert_eq!(bus.max_bootloaders(), 1);
    let log = bus.log();
    let first_back = index_of(
        &log,
        "1-1: enumerates as Firmware(Ours { build: \"v0.3.0\" })",
    );
    let second_asked = index_of(&log, "1-2: DAP 8101");
    assert!(first_back < second_asked, "{log:#?}");
    assert_eq!(batch.tally().updated, 2);
    let _ = (a, b);
}

#[test]
fn a_unit_back_with_its_jumper_fitted_is_not_taken_again() {
    let bus = SimBus::default();
    let mut batch = arm(
        &bus,
        Filter {
            bootloaders: true,
            ..filter()
        },
        None,
    );
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_bootloader_later(Duration::from_secs(1));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));

    assert_eq!(status(&batch, "1-1"), Status::JumperFitted);
    let flashes = bus
        .log()
        .iter()
        .filter(|l| l.contains("ROM command 0x04"))
        .count();
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(
        bus.log()
            .iter()
            .filter(|l| l.contains("ROM command 0x04"))
            .count(),
        flashes,
        "flashed again"
    );
    assert_eq!(seen.records.len(), 1);
    assert_eq!(seen.records[0].result, Settled::JumperFitted);
    assert!(
        batch.paused().unwrap().contains("jumper fitted"),
        "{:?}",
        batch.paused()
    );
    // Unplugged, the batch resumes.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    assert_eq!(batch.paused(), None);
}

#[test]
fn a_failure_does_not_stop_the_batch_and_its_bootloader_is_named() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_fail_writes(a, true);
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));

    assert!(
        matches!(status(&batch, "1-1"), Status::Failed(_)),
        "{:?}",
        status(&batch, "1-1")
    );
    assert_eq!(status(&batch, "1-2"), Status::Waiting);
    // In ISP mode nothing tells the board, so the pause names it and the batch's own image.
    let paused = batch.paused().unwrap();
    assert!(
        paused.contains("The unit at 1-1, a full MCU-Link, failed and is in ISP mode"),
        "{paused}"
    );
    assert!(
        paused.contains("`verge isp flash --release v0.3.0`"),
        "{paused}"
    );
    assert_eq!(seen.records[0].result, Settled::Failed);
    assert!(
        seen.records[0]
            .reason
            .as_deref()
            .unwrap()
            .contains("bootloader")
    );

    bus.unplug(a);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(status(&batch, "1-2"), Status::Updated);
    assert_eq!(bus.max_bootloaders(), 1);
    assert_eq!(batch.tally().updated, 1);
}

#[test]
fn a_unit_already_running_the_image_is_skipped_as_current() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), ours("v0.3.0"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(10));
    assert_eq!(status(&batch, "1-1"), Status::Current);
    assert!(!bus.log().iter().any(|l| l.contains("control")));
    assert_eq!(seen.records[0].result, Settled::Current);
    // It is as finished as an updated unit, so it blinks to be unplugged.
    assert!(bus.host_status_count(unit) > 0);
}

#[test]
fn a_mini_pauses_the_batch_until_it_is_unplugged() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let mini = bus.add_mini_bootloader();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(10));
    assert!(
        batch.paused().unwrap().contains("mini"),
        "{:?}",
        batch.paused()
    );
    assert_eq!(status(&batch, "1-2"), Status::Waiting);
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));

    bus.unplug(mini);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    assert_eq!(status(&batch, "1-2"), Status::Updated);
    assert_eq!(seen.pauses.last(), Some(&None));
}

#[test]
fn units_attached_when_armed_are_left_alone_unless_asked_for() {
    let bus = SimBus::default();
    bus.add_stock("3.172");
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(10));
    assert_eq!(status(&batch, "1-1"), Status::LeftAlone);
    assert!(seen.records.is_empty());

    let bus = SimBus::default();
    bus.add_stock("3.172");
    let mut batch = arm(
        &bus,
        Filter {
            attached: true,
            ..filter()
        },
        None,
    );
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(status(&batch, "1-1"), Status::Updated);
}

#[test]
fn a_count_stops_the_batch_after_that_many_units() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), Some(1));
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert!(batch.finished());
    assert_eq!(batch.tally().updated, 1);
    assert_eq!(status(&batch, "1-2"), Status::Waiting);
}

#[test]
fn stopping_takes_no_new_unit() {
    let bus = SimBus::default();
    let stop = Cancel::new();
    let mut batch = Batch::arm(
        &bus,
        release(),
        filter(),
        Timing::default(),
        stop.clone(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    stop.cancel();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    for _ in 0..40 {
        batch.step(&mut seen).unwrap();
    }
    assert!(batch.finished());
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
}

#[test]
fn the_filter_takes_only_what_it_names() {
    let bus = SimBus::default();
    let mut batch = arm(
        &bus,
        Filter {
            stock: StockFilter::Below(StockVersion::parse("3.172").unwrap()),
            ours: OursFilter::None,
            ..filter()
        },
        None,
    );
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.150"));
    bus.plug_in_later(Duration::from_secs(1), ours("14d05d4"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.108"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    let skipped = |port: &str| match status(&batch, port) {
        Status::Skipped(reason) => reason,
        other => panic!("{port}: {other:?}"),
    };
    assert!(skipped("1-1").contains("not below V3.172"));
    assert_eq!(status(&batch, "1-2"), Status::Updated);
    assert!(skipped("1-3").contains("does not take"));
    // Below the limit, but with no jumper-free update of its own.
    assert!(skipped("1-4").contains("needs the ISP jumper"));
}

#[test]
fn a_finished_unit_blinks_until_it_is_unplugged() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(status(&batch, "1-1"), Status::Updated);
    let blinks = bus.host_status_count(unit);
    assert!(blinks > 0);
    batch.step(&mut seen).unwrap();
    assert!(bus.host_status_count(unit) > blinks);
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    assert!(batch.entries().is_empty());
}

#[test]
fn each_unit_gets_a_record_line() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    let record = &seen.records[0];
    let json: serde_json::Value = serde_json::to_value(record).unwrap();
    assert_eq!(json["port"], "1-1");
    assert_eq!(json["result"], "updated");
    assert_eq!(json["serial_before"], "QK7RM2VTX8D41");
    assert_eq!(json["firmware_before"], "stock V3.172");
    assert_eq!(json["firmware_after"], "Verge, build v0.3.0");
    assert!(json["serial_after"].as_str().unwrap().starts_with("CJ"));
    assert!(json["time"].as_str().unwrap().ends_with('Z'));
    assert!(json["seconds"].as_f64().unwrap() > 2.4);
}

/// A unit that arrives in ISP mode blocks every other entry, so it is flashed first.
#[test]
fn a_waiting_bootloader_goes_before_a_waiting_probe() {
    let bus = SimBus::default();
    let mut batch = arm(
        &bus,
        Filter {
            bootloaders: true,
            ..filter()
        },
        None,
    );
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    let jumpered = bus.plug_in_bootloader_later(Duration::from_secs(1));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(seen.started, ["1-2"]);
    assert_eq!(status(&batch, "1-2"), Status::JumperFitted);
    assert_eq!(status(&batch, "1-1"), Status::Waiting);
    bus.unplug(jumpered);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(seen.started, ["1-2", "1-1"]);
    assert_eq!(status(&batch, "1-1"), Status::Updated);
    assert_eq!(bus.max_bootloaders(), 1);
}

/// Older than the image means nothing for stock or a file, so ours is taken only when the filter
/// takes any build. A release of ours is the case where the two filters disagree.
#[test]
fn ours_older_than_the_image_needs_an_image_of_ours() {
    let bus = SimBus::default();
    bus.add_running(ours("14d05d4"));
    bus.add_running(ours("v0.3.0"));
    let units = verge_probe::detect::detect(&bus).unwrap();
    let (old, current) = (&units[0], &units[1]);
    let older = filter();
    let any = Filter {
        ours: OursFilter::Any,
        ..filter()
    };
    let stock = ImageVersion::Stock(StockVersion::parse("3.172").unwrap());
    for version in [&stock, &ImageVersion::Unknown] {
        assert_eq!(
            judge(old, &older, version),
            Verdict::Skip("the image carries no Verge version to compare with".into())
        );
        assert_eq!(judge(old, &any, version), Verdict::Take);
        assert_eq!(judge(current, &any, version), Verdict::Take);
    }
    let release = ImageVersion::Ours("v0.3.0".into());
    assert_eq!(judge(old, &older, &release), Verdict::Take);
    assert_eq!(judge(current, &older, &release), Verdict::Current);
    assert_eq!(judge(current, &any, &release), Verdict::Current);
}

fn stock_probe(product: &str) -> verge_probe::Unit {
    use verge_probe::bus::UsbDevice;
    use verge_probe::dap::Identity;
    use verge_probe::{Firmware, State};
    verge_probe::Unit {
        device: UsbDevice {
            vid: 0x1fc9,
            pid: 0x0143,
            location: Some("1-4".into()),
            instance: "1".into(),
            serial: Some("PROSERIAL".into()),
            manufacturer: Some("NXP Semiconductors".into()),
            product: Some(product.into()),
            bcd_device: 0x0337,
            dap_v2: true,
            hid: true,
            aux: false,
        },
        state: State::Probe {
            identity: Identity::default(),
            firmware: Firmware::Stock {
                version: Some(StockVersion {
                    major: 3,
                    minor: 172,
                }),
            },
        },
    }
}

/// LinkServer's Readme: one stock image for every LPC55S69 MCU-Link, product string
/// `MCU-LINK [Pro|MR|on-board|<board>] (r<RRR>) CMSIS-DAP V<V>.<BBB>`. The batch's board is "the full
/// MCU-Link".
#[test]
fn a_batch_for_the_full_mculink_does_not_take_a_pro() {
    for product in [
        "MCU-LINK Pro (r1E2) CMSIS-DAP V3.172",
        "MCU-LINK FRDM-MCXN947 (r0E7) CMSIS-DAP V3.172",
    ] {
        let verdict = judge(
            &stock_probe(product),
            &filter(),
            &ImageVersion::Ours("v0.3.0".into()),
        );
        assert!(
            matches!(verdict, Verdict::Skip(_)),
            "{product}: {verdict:?}"
        );
    }
}

/// The update path waits `Timing::settle` after an enumeration before asking a unit what it runs.
/// A batch asks an arrival at once, and a unit it could not ask is skipped until it leaves the bus.
#[test]
fn a_unit_not_ready_for_a_moment_after_arriving_is_still_updated() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    // Not openable for its first moments on the bus.
    bus.set_busy(a, true);
    while bus.now() < Duration::from_millis(1300) {
        batch.step(&mut seen).unwrap();
    }
    bus.set_busy(a, false);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(status(&batch, "1-1"), Status::Updated, "{:?}", seen.records);
}

/// Ours reports its board from identify format 3, so a Pro running ours is not taken either.
#[test]
fn a_batch_does_not_take_ours_on_a_pro() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let pro = bus.plug_in_later(Duration::from_secs(1), ours("14d05d4"));
    bus.set_board(pro, Ours::BOARD_MCU_LINK_PRO);
    let plain = bus.plug_in_later(Duration::from_secs(1), ours("14d05d4"));
    bus.set_board(plain, Ours::BOARD_MCU_LINK);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(
        status(&batch, "1-1"),
        Status::Skipped("an MCU-Link Pro, not a full MCU-Link".into())
    );
    assert_eq!(status(&batch, "1-2"), Status::Updated);
    assert!(!bus.log().iter().any(|l| l.starts_with("1-1: control")));
}

/// The update path waits `Timing::settle` after an enumeration before it asks the unit anything,
/// and a batch's arrival gets the same.
#[test]
fn an_arrival_is_not_opened_before_it_has_settled() {
    let bus = SimBus::default();
    let hooked = Hooked::new(&bus);
    let mut batch = arm(&hooked, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(status(&batch, "1-1"), Status::Updated);
    let first_seen = batch.entries()[0].seen;
    let (first_open, _) = hooked.opened.lock().unwrap()[0].clone();
    assert!(
        first_open >= first_seen + Timing::default().settle,
        "opened at {first_open:?}, seen at {first_seen:?}"
    );
}

/// An open that fails soon after the unit arrives is not final until it has failed for the
/// settle time and a few polls more.
#[test]
fn an_arrival_that_cannot_be_opened_past_its_settle_time_is_still_updated() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_busy(a, true);
    // Busy two polls past the settle time.
    let timing = Timing::default();
    let until = Duration::from_secs(1) + timing.settle + timing.poll * 2;
    while bus.now() < until {
        batch.step(&mut seen).unwrap();
    }
    assert!(batch.entries().is_empty(), "{:?}", batch.entries());
    bus.set_busy(a, false);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(status(&batch, "1-1"), Status::Updated, "{:?}", seen.records);
}

#[test]
fn an_arrival_that_never_opens_is_skipped_once_its_grace_is_over() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_busy(a, true);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(
        status(&batch, "1-1"),
        Status::Skipped("the tool could not ask what it runs: another program has it open".into())
    );
    assert_eq!(seen.records.len(), 1);
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
}

/// A stop that lands while the batch asks a unit, between taking it off the queue and starting
/// its update, still keeps it out of its bootloader.
#[test]
fn a_stop_while_a_unit_is_asked_again_starts_no_update() {
    let bus = SimBus::default();
    let stop = Cancel::new();
    let opens = AtomicUsize::new(0);
    let mut hooked = Hooked::new(&bus);
    hooked.on_open = Box::new(|_| {
        let opened = opens.fetch_add(1, Ordering::Relaxed) + 1;
        // The first open asks the arrival what it runs; the second asks again before its update.
        if opened == 2 {
            stop.cancel();
        }
    });
    let mut batch = Batch::arm(
        &hooked,
        release(),
        filter(),
        Timing::default(),
        stop.clone(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert!(batch.finished());
    assert_eq!(opens.load(Ordering::Relaxed), 2);
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
    assert_eq!(status(&batch, "1-1"), Status::Waiting);
}

/// Units still waiting when the batch ends get a line in the record saying so.
#[test]
fn units_still_waiting_when_the_batch_ends_are_recorded() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), Some(1));
    let mut seen = Seen::default();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    batch.run_to_end(&mut seen).unwrap();
    let results: Vec<(&str, Settled)> = seen
        .records
        .iter()
        .map(|r| (r.port.as_str(), r.result))
        .collect();
    assert_eq!(
        results,
        [("1-1", Settled::Updated), ("1-2", Settled::Waiting)]
    );
    assert_eq!(
        seen.records[1].reason.as_deref(),
        Some("the batch stopped before its turn came")
    );
    // Closing again writes nothing more.
    batch.close(&mut seen);
    assert_eq!(seen.records.len(), 2);
}

/// The bus fails the look that follows an install. The install already ran, so it is recorded.
#[test]
fn a_bus_error_after_an_install_still_records_it() {
    let bus = SimBus::default();
    let back = AtomicBool::new(false);
    let mut hooked = Hooked::new(&bus);
    // The unit comes back running ours; the look after the update has asked it fails, once.
    hooked.on_open = Box::new(|device| {
        if device
            .product
            .as_deref()
            .is_some_and(|p| p.contains("Verge"))
        {
            back.store(true, Ordering::Relaxed);
        }
    });
    hooked.fail = Box::new(|| back.swap(false, Ordering::Relaxed));
    let mut batch = arm(&hooked, filter(), None);
    let mut seen = Seen::default();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    let error = batch.run_to_end(&mut seen).unwrap_err();
    assert_eq!(error, BusError::Enumerate("the test's bus failed".into()));
    assert_eq!(seen.records.len(), 1, "{:?}", seen.records);
    assert_eq!(seen.records[0].result, Settled::Updated);
    assert_eq!(
        seen.records[0].firmware_after.as_deref(),
        Some("Verge, build v0.3.0")
    );
}

/// The bus fails while a finished unit's LEDs are lit. The batch ends, and turns them off.
#[test]
fn a_bus_error_still_turns_the_leds_off() {
    let bus = SimBus::default();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    let mut hooked = Hooked::new(&bus);
    hooked.fail = Box::new(|| bus.leds_on(unit));
    let mut batch = arm(&hooked, filter(), None);
    let mut seen = Seen::default();
    assert!(batch.run_to_end(&mut seen).is_err());
    assert_eq!(seen.records[0].result, Settled::Updated);
    assert!(!bus.leds_on(unit), "the LEDs were left on");
}

/// A unit blinks only while the batch runs, so the last one under `--count` is not blinking.
#[test]
fn a_finished_unit_is_named_for_unplugging_and_blinks_only_while_the_batch_runs() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), Some(2));
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), ours("v0.3.0"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    batch.run_to_end(&mut seen).unwrap();
    assert_eq!(
        seen.unplugs,
        [
            ("1-1".to_owned(), true),
            ("1-2".to_owned(), true),
            ("1-3".to_owned(), false)
        ]
    );
}

/// The batch knows units by port, so a unit that reports none pauses it and is never taken.
#[test]
fn a_unit_with_no_port_pauses_the_batch() {
    let bus = SimBus::default();
    bus.set_locations(false);
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    let paused = batch.paused().unwrap();
    assert!(paused.contains("reports no USB port"), "{paused}");
    assert!(batch.entries().is_empty());
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    assert_eq!(batch.paused(), None);
}

/// On Windows a unit that comes back keeps its instance, so the batch sees it leave only by
/// looking while it is gone. One that is gone for a look is taken as new when it returns.
#[test]
fn where_a_unit_keeps_its_instance_it_is_new_once_it_was_seen_gone() {
    let bus = SimBus::default();
    bus.set_stable_instances(true);
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), ours("v0.3.0"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(5));
    assert_eq!(status(&batch, "1-1"), Status::Current);
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    assert!(batch.entries().is_empty());
    bus.plug_back_later(Duration::from_millis(100), unit);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(10));
    assert_eq!(seen.records.len(), 2);
    assert_eq!(status(&batch, "1-1"), Status::Current);
}

/// The jumper case where a bootloader that comes back keeps its instance, as on Windows.
#[test]
fn where_a_bootloader_keeps_its_instance_a_jumper_fitted_unit_is_still_seen() {
    let bus = SimBus::default();
    bus.set_stable_instances(true);
    let mut batch = arm(
        &bus,
        Filter {
            bootloaders: true,
            ..filter()
        },
        None,
    );
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_bootloader_later(Duration::from_secs(1));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    assert_eq!(status(&batch, "1-1"), Status::JumperFitted);
    assert!(batch.paused().unwrap().contains("jumper fitted"));
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    assert_eq!(batch.paused(), None);
}

/// A unit whose bootloader did not restart is flashed but not updated: it counts as failed, not
/// toward `--count`, and pauses the batch until it is plugged back in, when it runs the image.
#[test]
fn a_unit_that_did_not_restart_is_not_counted_as_updated() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), Some(1));
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_ignores_reset(unit, true);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    assert_eq!(
        status(&batch, "1-1"),
        Status::NotRestarted {
            maybe_jumper: false
        }
    );
    assert_eq!(seen.records[0].result, Settled::NotRestarted);
    assert!(
        !batch.finished(),
        "the count took a unit that did not restart"
    );
    assert_eq!(batch.tally().updated, 0);
    assert_eq!(batch.tally().failed, 1);
    let paused = batch.paused().unwrap();
    assert!(paused.contains("did not restart"), "{paused}");

    bus.unplug(unit);
    bus.plug_back_later(Duration::from_millis(500), unit);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    assert_eq!(status(&batch, "1-1"), Status::Current);
    assert_eq!(batch.paused(), None);
    // Plugged back in, it runs the image: one unit, counted as current.
    let tally = batch.tally();
    assert_eq!((tally.failed, tally.current), (0, 1), "{tally}");
}

/// The tally counts every unit the batch settled, including those unplugged since.
#[test]
fn the_tally_keeps_units_that_were_unplugged() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let a = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    let b = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_fail_writes(b, true);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    assert_eq!(batch.tally().updated, 1);
    assert_eq!(batch.tally().failed, 1);
    bus.unplug(a);
    bus.unplug(b);
    batch.step(&mut seen).unwrap();
    assert!(batch.entries().is_empty());
    let tally = batch.tally();
    assert_eq!((tally.updated, tally.failed), (1, 1), "{tally}");
}

/// A stop that lands while the update looks at the bus, before the unit is asked for its
/// bootloader, still keeps it out of the bootloader. The opens are: the arrival asked, asked
/// again, the update's look at the bus, the entry's look at the bus, and the entry's own.
#[test]
fn a_stop_before_the_bootloader_request_starts_no_update() {
    for stop_at in [3, 4, 5] {
        let bus = SimBus::default();
        let stop = Cancel::new();
        let opens = AtomicUsize::new(0);
        let mut hooked = Hooked::new(&bus);
        hooked.on_open = Box::new(|_| {
            let opened = opens.fetch_add(1, Ordering::Relaxed) + 1;
            if opened == stop_at {
                stop.cancel();
            }
        });
        let mut batch = Batch::arm(
            &hooked,
            release(),
            filter(),
            Timing::default(),
            stop.clone(),
            Cancel::new(),
            None,
        )
        .unwrap();
        let mut seen = Seen::default();
        bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
        batch.run_to_end(&mut seen).unwrap();
        assert!(
            !bus.log().iter().any(|l| l.contains("DAP 8101")),
            "stopped at open {stop_at}, and the unit was still asked for its bootloader"
        );
        assert_eq!(
            opens.load(Ordering::Relaxed),
            stop_at,
            "stopped at open {stop_at}"
        );
        let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
        assert_eq!(results, [Settled::Waiting], "stopped at open {stop_at}");
    }
}

/// A single update's cancel, arriving while the entry looks at the bus, sends no request.
#[test]
fn a_cancel_before_the_bootloader_request_sends_none() {
    let bus = SimBus::default();
    bus.add_stock("3.172");
    let unit = verge_probe::detect(&bus).unwrap().remove(0);
    let cancel = Cancel::new();
    let mut hooked = Hooked::new(&bus);
    hooked.on_open = Box::new(|_| cancel.cancel());
    let result = verge_probe::update::enter_bootloader(
        &hooked,
        &unit,
        false,
        &Timing::default(),
        &mut Seen::default(),
        &cancel,
    );
    assert!(
        matches!(result, Err(verge_probe::UpdateError::Stopped)),
        "{result:?}"
    );
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
}

/// Taking ours older than the image needs an image of ours to compare with: with stock's image
/// or a file the batch does not arm, rather than skip every unit running ours.
#[test]
fn a_batch_that_cannot_date_ours_does_not_arm() {
    use verge_probe::batch::ArmError;
    let bus = SimBus::default();
    let stock_image = Armed {
        image: firmware_image(&stock("3.172")),
        name: "stock V3.172".into(),
        version: ImageVersion::Stock(StockVersion::parse("3.172").unwrap()),
        source: Source::Stock(None),
    };
    let file = Armed {
        version: ImageVersion::Unknown,
        name: "a file".into(),
        ..release()
    };
    for (armed, ours, refused) in [
        (stock_image.clone(), OursFilter::Older, true),
        (file.clone(), OursFilter::Older, true),
        (stock_image, OursFilter::Any, false),
        (file, OursFilter::None, false),
        (release(), OursFilter::Older, false),
    ] {
        let name = armed.name.clone();
        let result = Batch::arm(
            &bus,
            armed,
            Filter { ours, ..filter() },
            Timing::default(),
            Cancel::new(),
            Cancel::new(),
            None,
        );
        assert_eq!(
            result.as_ref().err(),
            refused.then_some(&ArmError::OursUndated),
            "{name}, {ours:?}"
        );
    }
}

/// A batch for the Pro arms only with an image that declares the Pro, or stock's, and takes only
/// units that say they are one.
#[test]
fn a_batch_for_the_pro_takes_pros_with_an_image_that_declares_it() {
    use verge_probe::batch::ArmError;
    let bus = SimBus::default();
    let pro_filter = Filter {
        board: Board::McuLinkPro,
        ..filter()
    };
    let refused = Batch::arm(
        &bus,
        release(),
        pro_filter.clone(),
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    );
    assert_eq!(
        refused.err(),
        Some(ArmError::NotForBoard {
            board: Board::McuLinkPro,
            built_for: "the full MCU-Link only".into()
        })
    );

    let both = Armed {
        image: verge_probe::image::Image::parse(
            verge_probe::sim::firmware_srecord_declaring(&ours("v0.3.0"), 0b11).as_bytes(),
        )
        .unwrap(),
        ..release()
    };
    let mut batch = Batch::arm(
        &bus,
        both,
        pro_filter,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let pro = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_board(pro, Ours::BOARD_MCU_LINK_PRO);
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    assert_eq!(status(&batch, "1-1"), Status::Updated);
    assert_eq!(
        status(&batch, "1-2"),
        Status::Skipped("a full MCU-Link, not an MCU-Link Pro".into())
    );
    let back = verge_probe::detect(&bus).unwrap().remove(0);
    assert_eq!(
        back.hardware(),
        verge_probe::detect::Hardware::Board(Board::McuLinkPro)
    );
}

/// A unit that did not come back from its update and returns during the batch: running the
/// image, it is current; running anything else, it is taken once more, and if that fails too it
/// is left alone for the rest of the batch.
#[test]
fn a_unit_that_did_not_come_back_is_taken_once_more_and_no_more() {
    let entries = |bus: &SimBus| {
        bus.log()
            .iter()
            .filter(|l| l.as_str() == "1-1: DAP 8101")
            .count()
    };
    // Back late, running the image.
    let bus = SimBus::default();
    bus.set_boot_after_reset(Duration::from_secs(20));
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(results, [Settled::Failed, Settled::Current]);
    assert!(
        seen.records[0]
            .reason
            .as_deref()
            .unwrap()
            .contains("did not come back")
    );

    // Back late, each time running stock as if the flash had not taken.
    let bus = SimBus::default();
    bus.set_boot_after_reset(Duration::from_secs(20));
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    for failures in 1..=2 {
        while seen.records.len() < failures {
            batch.step(&mut seen).unwrap();
        }
        assert_eq!(seen.records[failures - 1].result, Settled::Failed);
        bus.reflash(unit, &stock("3.172"));
    }
    drive(
        &mut batch,
        &bus,
        &mut seen,
        bus.now() + Duration::from_secs(30),
    );
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(
        results,
        [Settled::Failed, Settled::Failed, Settled::Skipped],
        "{:?}",
        seen.records
    );
    assert_eq!(entries(&bus), 2);
    assert!(
        seen.records[2]
            .reason
            .as_deref()
            .unwrap()
            .contains("leaves it alone")
    );
}

/// A finished unit keeps blinking while another unit goes through its bootloader: a thread of
/// its own blinks it, with handles of its own. The blink period is real time, so this bench runs
/// paced, and a period of a few milliseconds fits many blinks into the second unit's update.
#[test]
fn a_finished_unit_keeps_blinking_while_another_updates() {
    let bus = SimBus::default().paced(0.02);
    let timing = Timing {
        poll: Duration::from_millis(10),
        ..Timing::default()
    };
    let mut batch = Batch::arm(
        &bus,
        release(),
        filter(),
        timing,
        Cancel::new(),
        Cancel::new(),
        std::num::NonZeroUsize::new(2),
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.plug_in_later(Duration::from_secs(10), stock("3.172"));
    batch.run_to_end(&mut seen).unwrap();
    assert_eq!(batch.tally().updated, 2);
    let log = bus.log();
    let entered = index_of(&log, "1-2: DAP 8101");
    let back = index_of(
        &log,
        "1-2: enumerates as Firmware(Ours { build: \"v0.3.0\" })",
    );
    // Each blink sets two LEDs, so two `DAP_HostStatus` commands.
    let commands = log[entered..back]
        .iter()
        .filter(|l| l.starts_with("1-1: DAP 01"))
        .count();
    assert!(
        commands >= 6,
        "1-1 answered {commands} DAP_HostStatus commands while 1-2 went through its bootloader"
    );
    assert!(!bus.leds_on(0), "the LEDs were left on");
}

/// On Windows a bootloader never seen to leave the bus may have restarted with its jumper fitted
/// between two looks. Either way it is not counted as updated, and the record says both causes.
#[test]
fn where_instances_are_kept_a_unit_never_seen_restarting_is_not_counted_as_updated() {
    let bus = SimBus::default();
    bus.set_stable_instances(true);
    let mut batch = arm(&bus, filter(), Some(1));
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_ignores_reset(unit, true);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    assert_eq!(
        status(&batch, "1-1"),
        Status::NotRestarted { maybe_jumper: true }
    );
    assert!(
        !batch.finished(),
        "the count took a unit that did not restart"
    );
    assert_eq!(batch.tally().updated, 0);
    let reason = seen.records[0].reason.as_deref().unwrap();
    assert!(
        reason.contains("take the jumper off if it is fitted"),
        "{reason}"
    );
}

/// Steps until the first unit's first record, which is its failure: it did not come back.
fn fail_once(batch: &mut Batch<'_, SimBus>, bus: &SimBus, seen: &mut Seen) -> usize {
    bus.set_boot_after_reset(Duration::from_secs(20));
    batch.step(seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    while seen.records.is_empty() {
        batch.step(seen).unwrap();
    }
    assert_eq!(seen.records[0].result, Settled::Failed);
    assert!(
        seen.records[0]
            .reason
            .as_deref()
            .unwrap()
            .contains("did not come back")
    );
    unit
}

fn entry_requests(bus: &SimBus) -> usize {
    bus.log().iter().filter(|l| *l == "1-1: DAP 8101").count()
}

/// The third review's proof: a unit that did not come back is taken once more, and that once is
/// counted when its update starts. One that came back and left again before its turn was never
/// taken once more.
#[test]
fn a_retry_counts_only_once_it_has_run() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    let unit = fail_once(&mut batch, &bus, &mut seen);
    // It comes back running stock, as if the flash had not taken, while a unit in ISP mode the
    // batch does not take pauses it.
    bus.reflash(unit, &stock("3.172"));
    let rom = bus.add_bootloader(true);
    let until = bus.now() + Duration::from_secs(10);
    while bus.now() < until {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(status(&batch, "1-1"), Status::Waiting);
    assert!(batch.paused().is_some());

    bus.unplug(unit);
    bus.unplug(rom);
    batch.step(&mut seen).unwrap();
    assert!(batch.entries().iter().all(|e| e.port != "1-1"));
    assert_eq!(entry_requests(&bus), 1, "one entry request so far");

    bus.set_boot_after_reset(Duration::from_secs(1));
    bus.plug_back_later(Duration::from_millis(500), unit);
    let until = bus.now() + Duration::from_secs(40);
    while bus.now() < until {
        batch.step(&mut seen).unwrap();
    }
    let last = seen.records.last().unwrap();
    assert_eq!(
        last.result,
        Settled::Updated,
        "after {} entry requests: {:?}",
        entry_requests(&bus),
        last.reason
    );
}

/// The third review's proof: after a unit on a port does not come back, another unit in that
/// port, known by its serial, is a unit of its own. Its failure leaves the port free for the next.
#[test]
fn another_unit_on_the_port_is_not_the_one_that_did_not_come_back() {
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    let unit = fail_once(&mut batch, &bus, &mut seen);
    bus.set_boot_after_reset(Duration::from_secs(1));
    // Unit A is put aside. Unit B goes into the port, and its flash fails.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.reflash(unit, &stock("3.172"));
    bus.set_serials(unit, "CJBBBBBBBB", "BBBBBBBBBBBBB");
    bus.set_fail_writes(unit, true);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 2 {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[1].result, Settled::Failed);
    assert_eq!(
        seen.records[1].serial_before.as_deref(),
        Some("BBBBBBBBBBBBB")
    );
    // B is unplugged; unit C, which nothing is wrong with, goes into the port.
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.reflash(unit, &stock("3.172"));
    bus.set_serials(unit, "CJCCCCCCCC", "CCCCCCCCCCCCC");
    bus.set_fail_writes(unit, false);
    bus.plug_back_later(Duration::from_millis(500), unit);
    while seen.records.len() < 3 {
        batch.step(&mut seen).unwrap();
    }
    let c = &seen.records[2];
    assert_eq!(c.serial_before.as_deref(), Some("CCCCCCCCCCCCC"));
    assert_eq!(
        c.result,
        Settled::Updated,
        "unit C: {:?} {:?}",
        c.result,
        c.reason
    );
    // Three units, each counted once: A and B failed, C updated.
    let tally = batch.tally();
    assert_eq!((tally.failed, tally.updated), (2, 1), "{tally}");
}

/// The tally counts units, each once, by how it ended; the record keeps a line per event. A unit
/// that failed and came back running the image counts as current. One updated, unplugged and
/// plugged back in counts as updated, once.
#[test]
fn the_tally_counts_each_unit_once_by_how_it_ended() {
    // Did not come back, then came back late running the image.
    let bus = SimBus::default();
    bus.set_boot_after_reset(Duration::from_secs(20));
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(60));
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(results, [Settled::Failed, Settled::Current]);
    let tally = batch.tally();
    assert_eq!((tally.failed, tally.current), (0, 1), "{tally}");

    // Updated, then unplugged and plugged back in.
    let bus = SimBus::default();
    let mut batch = arm(&bus, filter(), None);
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(30));
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.plug_back_later(Duration::from_millis(500), unit);
    drive(&mut batch, &bus, &mut seen, Duration::from_secs(40));
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(results, [Settled::Updated, Settled::Current]);
    let tally = batch.tally();
    assert_eq!((tally.updated, tally.current), (1, 0), "{tally}");
}

/// A file has no version, so a unit back on its port running ours cannot be told current by the
/// image. It is still the unit that left there: counted once, by how it ended. The second review
/// of the fixes proved it counted twice, as failed and updated.
#[test]
fn a_file_batch_counts_a_unit_that_came_back_once() {
    let file = || Armed {
        version: ImageVersion::Unknown,
        name: "a file".into(),
        source: Source::File("x.s19".into()),
        ..release()
    };
    let any = Filter {
        ours: OursFilter::Any,
        ..filter()
    };
    // The bootloader did not restart; plugged back in, the unit runs the file's image.
    let bus = SimBus::default();
    let mut batch = Batch::arm(
        &bus,
        file(),
        any.clone(),
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    let unit = bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    bus.set_ignores_reset(unit, true);
    while seen.records.is_empty() {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[0].result, Settled::NotRestarted);
    bus.set_ignores_reset(unit, false);
    bus.unplug(unit);
    batch.step(&mut seen).unwrap();
    bus.plug_back_later(Duration::from_millis(500), unit);
    drive(
        &mut batch,
        &bus,
        &mut seen,
        bus.now() + Duration::from_secs(40),
    );
    let tally = batch.tally();
    assert_eq!(
        (tally.updated, tally.failed),
        (1, 0),
        "{tally}: {:?}",
        seen.records
    );

    // It did not come back; later it does, running the file's image.
    let bus = SimBus::default();
    bus.set_boot_after_reset(Duration::from_secs(20));
    let mut batch = Batch::arm(
        &bus,
        file(),
        any,
        Timing::default(),
        Cancel::new(),
        Cancel::new(),
        None,
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
    while seen.records.is_empty() {
        batch.step(&mut seen).unwrap();
    }
    assert_eq!(seen.records[0].result, Settled::Failed);
    // Its retry restarts in time.
    bus.set_boot_after_reset(Duration::from_secs(1));
    drive(
        &mut batch,
        &bus,
        &mut seen,
        bus.now() + Duration::from_secs(40),
    );
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(results, [Settled::Failed, Settled::Updated]);
    let tally = batch.tally();
    assert_eq!(
        tally.failed + tally.updated + tally.current + tally.skipped,
        1,
        "{tally}: {:?}",
        seen.records
    );
    assert_eq!(tally.failed, 0, "{tally}");
}

/// A unit with its SWD jumper cannot say which build it runs, so a batch leaves it alone.
#[test]
fn a_batch_skips_a_unit_with_its_swd_jumper() {
    let bus = SimBus::scenario("swd-jumper").unwrap();
    let unit = verge_probe::detect::detect(&bus).unwrap().remove(0);
    let any = Filter {
        ours: OursFilter::Any,
        ..filter()
    };
    let Verdict::Skip(reason) = judge(&unit, &any, &ImageVersion::Ours("v0.3.0".into())) else {
        panic!("taken");
    };
    assert!(reason.contains("SWD jumper"), "{reason}");
}
