//! Whole updates on the simulated bench: both jumper-free entries, the bootloader flash sequence,
//! the jumper path, and the refusals.
//!
//! Every wait is bounded in the bus's time, which the simulator advances only when the tool
//! sleeps or a command costs time. No test here takes wall-clock time to time out.

use std::time::Duration;

#[cfg(not(windows))]
use verge_probe::detect::Jumpered;
use verge_probe::image::{Image, PAGE};
use verge_probe::progress::{Cancel, Progress, Step};
use verge_probe::sim::{Mode, SimBus, SimFirmware, firmware_image};
use verge_probe::source::{ImageVersion, Payload};
use verge_probe::update::{self, Outcome, Timing, UpdateError};
use verge_probe::{Firmware, State, Unit, detect};

/// What an image of ours from a file is.
static FILE: ImageVersion = ImageVersion::Unknown;

#[derive(Default)]
struct Steps {
    plan: Vec<Step>,
    started: Vec<(Step, u64)>,
    last_done: u64,
    notes: Vec<String>,
    cancel_after: Option<(Step, u64, Cancel)>,
}

impl Progress for Steps {
    fn plan(&mut self, steps: &[Step]) {
        self.plan = steps.to_vec();
    }
    fn start(&mut self, step: Step, total: u64) {
        self.started.push((step, total));
        self.last_done = 0;
    }
    fn advance(&mut self, done: u64) {
        assert!(done >= self.last_done, "progress went backwards");
        self.last_done = done;
        if let Some((step, at, cancel)) = &self.cancel_after
            && self.started.last().map(|s| s.0) == Some(*step)
            && done >= *at
        {
            cancel.cancel();
        }
    }
    fn note(&mut self, text: &str) {
        self.notes.push(text.to_owned());
    }
}

fn ours(build: &str) -> SimFirmware {
    SimFirmware::Ours {
        build: build.to_owned(),
    }
}

fn stock(version: &str) -> SimFirmware {
    SimFirmware::Stock {
        version: version.parse_stock(),
    }
}

trait ParseStock {
    fn parse_stock(&self) -> verge_probe::version::StockVersion;
}

impl ParseStock for str {
    fn parse_stock(&self) -> verge_probe::version::StockVersion {
        verge_probe::version::StockVersion::parse(self).unwrap()
    }
}

fn the_unit(bus: &SimBus, serial: &str) -> Unit {
    let units = detect(bus).unwrap();
    update::select(&units, Some(serial)).unwrap().clone()
}

fn running(outcome: &Outcome) -> &Firmware {
    match outcome {
        Outcome::Running(unit) => match &unit.state {
            State::Probe { firmware, .. } => firmware,
            other => panic!("came back as {other:?}"),
        },
        other => panic!("came back as {other:?}"),
    }
}

/// The pages a flashed image should leave, whole and padded.
fn expected_pages(image: &Image) -> Vec<(u32, Vec<u8>)> {
    image
        .page_runs()
        .into_iter()
        .flat_map(|run| {
            run.data
                .chunks(PAGE as usize)
                .enumerate()
                .map(|(i, page)| (run.address + i as u32 * PAGE, page.to_vec()))
                .collect::<Vec<_>>()
        })
        .collect()
}

#[test]
fn ours_updates_itself_without_the_jumper() {
    let bus = SimBus::scenario("bench").unwrap();
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let image = firmware_image(&ours("v0.3.0"));
    let mut steps = Steps::default();
    let outcome = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut steps,
        &Cancel::new(),
    )
    .unwrap();

    let Firmware::Ours(now) = running(&outcome) else {
        panic!("{outcome:?}");
    };
    assert_eq!(now.build, "v0.3.0");
    let started: Vec<Step> = steps.started.iter().map(|s| s.0).collect();
    assert_eq!(started, steps.plan);

    let log = bus.log();
    assert!(
        log.iter()
            .any(|l| l == "1-1: control 0x40 0x03 wValue 1 wIndex 0"),
        "{log:#?}"
    );
    // The stock unit beside it was left alone.
    assert!(!log.iter().any(|l| l.starts_with("1-2: DAP 81")));
    assert_eq!(bus.mode(1), Mode::Firmware(stock("3.172")));

    for (address, page) in expected_pages(&image) {
        assert_eq!(
            bus.flash(0).get(&address),
            Some(&page),
            "page 0x{address:x}"
        );
    }
}

#[test]
fn stock_takes_the_request_linkserver_sends() {
    let bus = SimBus::scenario("stock").unwrap();
    let unit = the_unit(&bus, "QK7RM2VTX8D41");
    let image = firmware_image(&ours("v0.3.0"));
    let outcome = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));
    // Its serial changes with the firmware, so the tool finds it by port, not by serial.
    let Outcome::Running(back) = &outcome else {
        unreachable!()
    };
    assert_eq!(back.serial(), Some(bus.serial(0).0.as_str()));
    assert_ne!(back.serial(), Some("QK7RM2VTX8D41"));

    // From the request until the unit leaves the bus, the tool sends DAP_Disconnect and nothing
    // else.
    let log = bus.log();
    let entry: Vec<&str> = log
        .iter()
        .map(String::as_str)
        .skip_while(|l| !l.ends_with("DAP 8101"))
        .take_while(|l| !l.ends_with("leaves the bus"))
        .filter(|l| l.contains(": DAP "))
        .collect();
    assert_eq!(entry, ["1-1: DAP 8101", "1-1: DAP 03"]);
}

/// Stock leaves the bus 2.1 s after the request and the bootloader enumerates 0.3 s later, so a
/// wait shorter than 2.4 s of bus time fails and a longer one does not.
#[test]
fn the_wait_for_stock_bootloader_is_counted_in_bus_time() {
    for (limit_ms, ok) in [(2300, false), (2500, true)] {
        let bus = SimBus::scenario("stock").unwrap();
        let unit = the_unit(&bus, "QK7RM2VTX8D41");
        let timing = Timing {
            bootloader_appears: Duration::from_millis(limit_ms),
            ..Timing::default()
        };
        let started = std::time::Instant::now();
        let result = update::enter_bootloader(
            &bus,
            &unit,
            false,
            &timing,
            &mut Steps::default(),
            &Cancel::new(),
        );
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "a simulated wait took real time"
        );
        match (result, ok) {
            (Ok(_), true) | (Err(UpdateError::NoBootloader(_)), false) => {}
            (other, _) => panic!("{limit_ms} ms: {other:?}"),
        }
    }
}

#[test]
fn old_stock_is_refused_before_anything_is_sent() {
    let bus = SimBus::scenario("stock-old").unwrap();
    let unit = the_unit(&bus, "QK7RM2VTX8D41");
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("V3.108"), "{reason}");
    assert!(!bus.log().iter().any(|l| l.contains("DAP 81")));
}

#[test]
fn a_unit_already_in_isp_mode_holds_back_another_units_entry() {
    let bus = SimBus::default();
    bus.add_ours("14d05d4");
    bus.add_bootloader(false);
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(matches!(result, Err(UpdateError::Refused(_))), "{result:?}");
    assert!(!bus.log().iter().any(|l| l.contains("control")));
}

#[test]
fn two_units_in_isp_mode_are_refused() {
    let bus = SimBus::scenario("two-isp").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.starts_with("2 units are in ISP mode"), "{reason}");
    assert!(!bus.log().iter().any(|l| l.contains("ROM command")));
}

#[test]
fn a_mini_is_refused() {
    let bus = SimBus::scenario("mini").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("mini"), "{reason}");
}

#[test]
fn the_jumper_path_flashes_and_says_the_jumper_is_still_on() {
    let bus = SimBus::scenario("isp-jumper").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let outcome = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert_eq!(outcome, Outcome::JumperStillFitted);
    // With the jumper off, the same flash boots.
    bus.set_jumper(0, false);
    let outcome = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));
}

#[test]
fn a_unit_back_between_two_looks_at_the_bus_is_seen() {
    // On the bench our firmware is back about 0.4 s after the reset, so its port can be empty
    // for less than one poll.
    for jumper in [false, true] {
        let bus = SimBus::scenario(if jumper { "isp-jumper" } else { "isp" }).unwrap();
        bus.set_boot_after_reset(Duration::from_millis(100));
        let image = firmware_image(&ours("v0.3.0"));
        let outcome = update::flash_now(
            &bus,
            &image,
            None,
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
        )
        .unwrap();
        if jumper {
            assert_eq!(outcome, Outcome::JumperStillFitted);
        } else {
            assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));
        }
    }
}

#[test]
fn back_to_stock_without_the_jumper() {
    let bus = SimBus::scenario("ours").unwrap();
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let image = firmware_image(&stock("3.172"));
    let version = ImageVersion::Stock("3.172".parse_stock());
    let outcome = update::update(
        &bus,
        &unit,
        Payload::new(&image, &version),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert_eq!(
        *running(&outcome),
        Firmware::Stock {
            version: Some("3.172".parse_stock())
        }
    );
}

/// The verify reads only pages the flash programmed: the simulated bootloader drops off the bus
/// on reading an erased page, as the real one does, so a clean update shows none was read.
#[test]
fn an_erased_page_read_drops_the_bootloader() {
    use mboot::McuBoot;
    use verge_probe::Bus;
    let bus = SimBus::scenario("isp").unwrap();
    let rom = bus.devices().unwrap().remove(0);
    let mut boot = McuBoot::new(bus.open_isp(&rom).unwrap());
    assert!(boot.read_memory(0, 512, 0).is_err());
    bus.sleep(Duration::from_millis(5));
    assert!(
        bus.devices().unwrap().is_empty(),
        "the bootloader is still on the bus"
    );
}

#[test]
fn an_image_that_does_not_boot_is_reported() {
    // A valid vector table and no simulator marker: flashed and verified, then nothing enumerates.
    let mut image = firmware_image(&ours("x"));
    let code = &mut image.chunks[0].data;
    let at = code.windows(6).position(|w| w == b"SIMFW:").unwrap();
    code[at..at + 6].fill(0);
    image.check_for_mculink().unwrap();
    let bus = SimBus::scenario("isp").unwrap();
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(
        matches!(result, Err(UpdateError::NoReturn(_))),
        "{result:?}"
    );
}

#[test]
fn a_cancelled_write_leaves_a_unit_the_next_flash_recovers() {
    let bus = SimBus::scenario("isp").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let cancel = Cancel::new();
    let mut steps = Steps {
        cancel_after: Some((Step::Write, 1, cancel.clone())),
        ..Steps::default()
    };
    let result = update::flash_now(&bus, &image, None, &Timing::default(), &mut steps, &cancel);
    assert!(
        matches!(
            result,
            Err(UpdateError::Isp(verge_probe::isp::IspError::Cancelled))
        ),
        "{result:?}"
    );
    assert_eq!(bus.mode(0), Mode::Bootloader);
    let outcome = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));
}

#[test]
fn an_image_that_cannot_be_firmware_is_refused_before_the_bootloader() {
    let bus = SimBus::scenario("ours").unwrap();
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let image = Image::parse(b"S1070100AABBCCDDE9\n").unwrap();
    let result = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(matches!(result, Err(UpdateError::Image(_))), "{result:?}");
    assert!(!bus.log().iter().any(|l| l.contains("control")));
}

#[test]
fn progress_counts_every_byte_of_every_page() {
    let bus = SimBus::scenario("isp").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let mut steps = Steps::default();
    update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut steps,
        &Cancel::new(),
    )
    .unwrap();
    let total: u64 = image.page_runs().iter().map(|r| r.data.len() as u64).sum();
    for step in [Step::Erase, Step::Write, Step::Verify] {
        assert!(
            steps.started.contains(&(step, total)),
            "{step:?} {:?}",
            steps.started
        );
    }
}

/// With a mini in ISP mode, nothing is flashed, whichever unit is asked for.
#[test]
fn a_mini_in_isp_mode_refuses_the_flash_of_the_other_bootloader() {
    let bus = SimBus::scenario("isp-mini").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("unplug the mini first"), "{reason}");
    assert!(!bus.log().iter().any(|l| l.contains("ROM command")));
}

#[test]
fn a_mini_in_isp_mode_holds_back_a_jumper_free_update() {
    let bus = SimBus::default();
    bus.add_ours("14d05d4");
    bus.add_mini_bootloader();
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(matches!(result, Err(UpdateError::Refused(_))), "{result:?}");
    assert!(
        !bus.log()
            .iter()
            .any(|l| l.contains("control") || l.contains("ROM command"))
    );
}

/// The bootloader is opened by retrying while it comes up. One that joins it meanwhile refuses
/// the flash before anything is erased.
#[test]
fn a_second_bootloader_that_arrives_while_connecting_refuses_the_flash() {
    let bus = SimBus::default();
    bus.add_bootloader(false);
    bus.plug_in_bootloader_later(Duration::from_millis(1));
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.starts_with("2 units are in ISP mode"), "{reason}");
    assert!(
        !bus.log()
            .iter()
            .any(|l| l.contains("ROM command 0x02") || l.contains("ROM command 0x04"))
    );
}

/// The bootloader checked leaves while the tool connects, and another takes its place on
/// another port: the one that answers is not the one checked.
#[test]
fn a_bootloader_replaced_while_connecting_refuses_the_flash() {
    let bus = SimBus::default();
    let first = bus.add_bootloader(false);
    bus.unplug_later(Duration::from_micros(1500), first);
    bus.plug_in_bootloader_later(Duration::from_micros(1500));
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(
        reason.contains("changed while the tool connected"),
        "{reason}"
    );
    assert!(!bus.log().iter().any(|l| l.contains("ROM command 0x02")));
}

/// Plugs `stranger` in as the flashed unit comes back, so both are new on the next look.
struct Stranger<'a> {
    bus: &'a SimBus,
    stranger: usize,
}

impl Progress for Stranger<'_> {
    fn start(&mut self, step: Step, _total: u64) {
        if step == Step::WaitForFirmware {
            // The unit enumerates 1 s after the reset command, which came just before this.
            self.bus
                .plug_back_later(Duration::from_millis(990), self.stranger);
        }
    }
}

/// Where no device reports its port, the unit that comes back is the one new device, and two
/// new devices are refused.
#[test]
fn without_ports_the_unit_back_is_taken_only_as_the_one_new_device() {
    let bus = SimBus::scenario("isp").unwrap();
    bus.set_locations(false);
    let image = firmware_image(&ours("v0.3.0"));
    let outcome = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));

    // The stranger is the first unit on the bench, so it is listed first.
    let bus = SimBus::default();
    let stranger = bus.add_stock("3.172");
    bus.unplug(stranger);
    bus.add_bootloader(false);
    bus.set_locations(false);
    let result = update::flash_now(
        &bus,
        &image,
        None,
        &Timing::default(),
        &mut Stranger {
            bus: &bus,
            stranger,
        },
        &Cancel::new(),
    );
    assert!(
        matches!(result, Err(UpdateError::Ambiguous { count: 2 })),
        "{result:?}"
    );
}

/// On Windows a bootloader that comes back to the same port keeps its instance. Seen gone, it is
/// the unit back with its jumper. Back too fast to be seen gone, it cannot be told from one that
/// never restarted, so it is not counted as flashed back: the outcome names both causes.
#[test]
fn where_a_bootloader_keeps_its_instance_the_jumper_is_still_seen() {
    for (boot_after_reset, expected) in [
        (1000, Outcome::JumperStillFitted),
        (100, Outcome::NotRestarted { maybe_jumper: true }),
    ] {
        let bus = SimBus::default();
        bus.set_stable_instances(true);
        bus.add_bootloader(true);
        bus.set_boot_after_reset(Duration::from_millis(boot_after_reset));
        let image = firmware_image(&ours("v0.3.0"));
        let outcome = update::flash_now(
            &bus,
            &image,
            None,
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
        );
        assert_eq!(
            outcome.as_ref().ok(),
            Some(&expected),
            "{boot_after_reset} ms: {outcome:?}"
        );
    }
    // Without the jumper it comes back as a firmware, under an instance of its own.
    let bus = SimBus::default();
    bus.set_stable_instances(true);
    bus.add_bootloader(false);
    let outcome = update::flash_now(
        &bus,
        &firmware_image(&ours("v0.3.0")),
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));
}

/// Where a device that comes back gets a new instance, a bootloader still there under its own
/// instance at the deadline never restarted. Where it keeps its instance, as on Windows, it may
/// equally have restarted too fast to see with the jumper fitted; it is not restarted either way,
/// and the outcome says both.
#[test]
fn a_bootloader_that_never_restarts_is_its_own_outcome() {
    for (keeps_instances, expected) in [
        (
            false,
            Outcome::NotRestarted {
                maybe_jumper: false,
            },
        ),
        (true, Outcome::NotRestarted { maybe_jumper: true }),
    ] {
        let bus = SimBus::default();
        bus.set_stable_instances(keeps_instances);
        let unit = bus.add_bootloader(false);
        bus.set_ignores_reset(unit, true);
        let outcome = update::flash_now(
            &bus,
            &firmware_image(&ours("v0.3.0")),
            None,
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
        );
        assert_eq!(
            outcome.as_ref().ok(),
            Some(&expected),
            "keeps instances {keeps_instances}: {outcome:?}"
        );
        assert_eq!(bus.mode(unit), Mode::Bootloader);
    }
}

/// The second review's proof: `enter_bootloader` on a Pro, then `flash_now` with ours, as
/// `verge isp enter` and the `verge isp flash` it advised did, put ours on the Pro. A unit in ISP
/// mode cannot say which board it is, so nothing after the entry can refuse it. The entry refuses
/// instead, unless stock's own image follows.
#[test]
fn a_pro_sent_to_its_bootloader_by_the_tool_is_not_then_flashed_with_ours() {
    let bus = SimBus::scenario("pro").unwrap();
    let pro = detect(&bus).unwrap().remove(0);
    let entered = update::enter_bootloader(
        &bus,
        &pro,
        false,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(
        matches!(&entered, Err(UpdateError::Refused(reason)) if reason.starts_with("the unit is an MCU-Link Pro")),
        "{entered:?}"
    );
    assert!(!bus.log().iter().any(|l| l.contains("DAP 8101")));
    let outcome = update::flash_now(
        &bus,
        &firmware_image(&ours("v0.3.0")),
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(
        !matches!(&outcome, Ok(Outcome::Running(_))),
        "ours went on the Pro: {outcome:?}"
    );
    // For stock it goes.
    update::enter_bootloader(
        &bus,
        &pro,
        true,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
}

#[test]
fn a_unit_known_to_be_another_board_goes_into_isp_mode_only_for_stock() {
    let pro = detect(&SimBus::scenario("pro").unwrap()).unwrap().remove(0);
    let refused = update::check_entry(&pro, false);
    assert!(
        matches!(&refused, Err(UpdateError::Refused(reason)) if reason.starts_with("the unit is an MCU-Link Pro")),
        "{refused:?}"
    );
    update::check_entry(&pro, true).unwrap();
    // The full MCU-Link, and a unit that does not name its board, go either way.
    for scenario in ["stock", "ours", "unnamed-board"] {
        let unit = detect(&SimBus::scenario(scenario).unwrap())
            .unwrap()
            .remove(0);
        update::check_entry(&unit, false).unwrap();
    }
}

/// The core itself refuses an image of ours that does not declare the unit's board, before
/// anything is sent to the unit, whichever front end calls it. Stock's own image goes on any
/// board, as its version says; an image of ours that declares the board goes too.
#[test]
fn the_update_itself_checks_the_board() {
    let bus = SimBus::scenario("pro").unwrap();
    let pro = detect(&bus).unwrap().remove(0);
    let plain = firmware_image(&ours("v0.3.0"));
    for result in [
        update::install(
            &bus,
            &pro,
            Payload::new(&plain, &FILE),
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
            &Cancel::new(),
        ),
        update::update(
            &bus,
            &pro,
            Payload::new(&plain, &ImageVersion::Ours("v0.3.0".into())),
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
        ),
    ] {
        assert!(
            matches!(&result, Err(UpdateError::Refused(reason))
                if reason.contains("the unit is an MCU-Link Pro, and this image is built for the full MCU-Link only")),
            "{result:?}"
        );
    }
    assert!(!bus.log().iter().any(|l| l.contains("DAP 81")));

    let declaring = Image::parse(
        verge_probe::sim::firmware_srecord_declaring(&ours("v0.3.0"), 0b10).as_bytes(),
    )
    .unwrap();
    let outcome = update::install(
        &bus,
        &pro,
        Payload::new(&declaring, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Ours(o) if o.build == "v0.3.0"));

    let back = detect(&bus).unwrap().remove(0);
    let stock_image = firmware_image(&stock("3.172"));
    let version = ImageVersion::Stock("3.172".parse_stock());
    let outcome = update::install(
        &bus,
        &back,
        Payload::new(&stock_image, &version),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(running(&outcome), Firmware::Stock { .. }));
}

/// With the SWD jumper fitted, ours has no interface to claim, so the reboot request goes to the
/// device itself. It comes back with the jumper still fitted.
#[cfg(not(windows))]
#[test]
fn ours_with_its_swd_jumper_updates_through_the_control_endpoint() {
    let bus = SimBus::scenario("swd-jumper").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let image = firmware_image(&ours("v0.3.0"));
    let outcome = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    let Outcome::Running(back) = &outcome else {
        panic!("{outcome:?}");
    };
    assert_eq!(
        back.state,
        State::SwdJumper {
            firmware: Jumpered::Ours
        }
    );
    let log = bus.log();
    assert!(
        log.iter()
            .any(|l| l == "1-1: control 0x40 0x03 wValue 1 wIndex 0"),
        "{log:#?}"
    );
    assert!(!log.iter().any(|l| l.contains(": DAP ")), "{log:#?}");
    assert_eq!(bus.mode(0), Mode::Firmware(ours("v0.3.0")));
    for (address, page) in expected_pages(&image) {
        assert_eq!(
            bus.flash(0).get(&address),
            Some(&page),
            "page 0x{address:x}"
        );
    }
}

/// The board comes from the product string, so a Pro with its SWD jumper refuses an image for
/// the plain board as a Pro without it does.
#[cfg(not(windows))]
#[test]
fn a_pro_with_its_swd_jumper_takes_only_an_image_that_declares_it() {
    let bus = SimBus::scenario("pro-swd-jumper").unwrap();
    let pro = detect(&bus).unwrap().remove(0);
    let plain = firmware_image(&ours("v0.3.0"));
    let refused = update::install(
        &bus,
        &pro,
        Payload::new(&plain, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
        &Cancel::new(),
    );
    assert!(
        matches!(&refused, Err(UpdateError::Refused(reason)) if reason.contains("the unit is an MCU-Link Pro")),
        "{refused:?}"
    );
    assert!(!bus.log().iter().any(|l| l.contains("control")));

    let declaring = Image::parse(
        verge_probe::sim::firmware_srecord_declaring(&ours("v0.3.0"), 0b10).as_bytes(),
    )
    .unwrap();
    let outcome = update::install(
        &bus,
        &pro,
        Payload::new(&declaring, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(outcome, Outcome::Running(_)), "{outcome:?}");
    assert_eq!(bus.mode(0), Mode::Firmware(ours("v0.3.0")));
}

/// Stock's update request goes over the CMSIS-DAP interface the SWD jumper took away.
#[test]
fn stock_with_its_swd_jumper_is_refused_before_anything_is_sent() {
    let bus = SimBus::scenario("stock-swd-jumper").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let image = firmware_image(&ours("v0.3.0"));
    let result = update::install(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("take the SWD jumper off"), "{reason}");
    let log = bus.log();
    assert!(
        !log.iter()
            .any(|l| l.contains(": DAP ") || l.contains("control")),
        "{log:#?}"
    );
}

/// A unit gone before the request is reported as gone, not as a firmware that refused it.
#[cfg(not(windows))]
#[test]
fn a_jumpered_unit_gone_before_the_request_is_named_gone() {
    let bus = SimBus::scenario("swd-jumper").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let image = firmware_image(&ours("v0.3.0"));
    bus.unplug(0);
    let result = update::update(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    assert!(
        matches!(
            result,
            Err(UpdateError::Bus(verge_probe::bus::BusError::Gone { .. }))
        ),
        "{result:?}"
    );
}

/// Stops the update as the given step starts.
#[cfg(not(windows))]
struct StopAt(Step, Cancel);

#[cfg(not(windows))]
impl Progress for StopAt {
    fn start(&mut self, step: Step, _total: u64) {
        if step == self.0 {
            self.1.cancel();
        }
    }
}

/// A stop that arrives as the entry starts, the last moment before the request, leaves a
/// jumpered unit alone.
#[cfg(not(windows))]
#[test]
fn a_stop_before_the_request_sends_nothing_to_a_jumpered_unit() {
    let bus = SimBus::scenario("swd-jumper").unwrap();
    let unit = detect(&bus).unwrap().remove(0);
    let image = firmware_image(&ours("v0.3.0"));
    let stop = Cancel::new();
    let result = update::install(
        &bus,
        &unit,
        Payload::new(&image, &FILE),
        &Timing::default(),
        &mut StopAt(Step::EnterBootloader, stop.clone()),
        &Cancel::new(),
        &stop,
    );
    assert!(matches!(result, Err(UpdateError::Stopped)), "{result:?}");
    assert!(!bus.log().iter().any(|l| l.contains("control")));
}

#[test]
fn a_reset_starts_the_image_a_unit_in_isp_mode_holds() {
    let bus = SimBus::scenario("isp-ours").unwrap();
    let held = bus.flash(0);
    let outcome = update::reset_now(
        &bus,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(
        matches!(running(&outcome), Firmware::Ours(_)),
        "{outcome:?}"
    );
    assert_eq!(bus.flash(0), held);
}

#[test]
fn a_reset_with_the_jumper_comes_back_in_isp_mode() {
    let bus = SimBus::scenario("isp-jumper").unwrap();
    let outcome = update::reset_now(
        &bus,
        None,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert_eq!(outcome, Outcome::JumperStillFitted);
}

/// With two units in ISP mode a port picks one, for a flash and for a reset; the other is left as
/// it was.
#[test]
fn a_port_picks_one_of_two_bootloaders() {
    let bus = SimBus::scenario("two-isp").unwrap();
    let image = firmware_image(&ours("v0.3.0"));
    let outcome = update::flash_now(
        &bus,
        &image,
        Some("1-2"),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert!(matches!(outcome, Outcome::Running(_)), "{outcome:?}");
    assert!(bus.flash(0).is_empty() && !bus.flash(1).is_empty());
    assert_eq!(bus.mode(0), Mode::Bootloader);

    let bus = SimBus::scenario("two-isp-ours").unwrap();
    let outcome = update::reset_now(
        &bus,
        Some("1-1"),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    let Outcome::Running(unit) = outcome else {
        panic!("{outcome:?}");
    };
    assert_eq!(unit.device.location.as_deref(), Some("1-1"));
    assert_eq!(bus.mode(1), Mode::Bootloader);
}

/// Where the bus opens the first bootloader whichever it is given, a port is no way to choose
/// among several, so nothing is sent; with one attached, its port is as good as none.
#[test]
fn a_port_among_bootloaders_that_look_the_same_is_refused() {
    let bus = SimBus::scenario("two-isp-ours").unwrap();
    bus.set_same_bootloaders(true);
    let result = update::reset_now(
        &bus,
        Some("1-2"),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("cannot tell which bootloader"), "{reason}");
    assert!(!bus.log().iter().any(|l| l.contains("ROM command")));

    let bus = SimBus::scenario("isp-ours").unwrap();
    bus.set_same_bootloaders(true);
    let (rom, info) =
        update::rom_info_now(&bus, Some("1-1"), &Timing::default(), &Cancel::new()).unwrap();
    assert_eq!(rom.location.as_deref(), Some("1-1"));
    assert!(!info.version.is_empty());
}

#[test]
fn a_port_with_no_bootloader_is_refused() {
    let bus = SimBus::scenario("isp-ours").unwrap();
    let result = update::rom_info_now(&bus, Some("1-9"), &Timing::default(), &Cancel::new());
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(
        reason.starts_with("no unit is in ISP mode at 1-9"),
        "{reason}"
    );
}

/// A mini in ISP mode refuses a flash even when a port names the other bootloader, as without.
#[test]
fn a_mini_refuses_a_flash_by_port_too() {
    let bus = SimBus::scenario("isp-mini").unwrap();
    let result = update::flash_now(
        &bus,
        &firmware_image(&ours("v0.3.0")),
        Some("1-1"),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("unplug the mini first"), "{reason}");
}

/// `isp enter` beside another bootloader goes ahead only where the bus tells bootloaders apart
/// and the unit has a port to find its own by; otherwise the old refusal stands.
#[test]
fn entering_beside_a_bootloader_needs_ports_that_tell_them_apart() {
    for (same, ports) in [(true, true), (false, false)] {
        let bus = SimBus::scenario("ours-beside-isp").unwrap();
        bus.set_same_bootloaders(same);
        bus.set_locations(ports);
        let unit = the_unit(&bus, "CJ3E7A91C2");
        let result = update::enter_bootloader(
            &bus,
            &unit,
            false,
            &Timing::default(),
            &mut Steps::default(),
            &Cancel::new(),
        );
        assert!(
            matches!(result, Err(UpdateError::Refused(_))),
            "same {same}, ports {ports}: {result:?}"
        );
    }
    let bus = SimBus::scenario("ours-beside-isp").unwrap();
    let unit = the_unit(&bus, "CJ3E7A91C2");
    let rom = update::enter_bootloader(
        &bus,
        &unit,
        false,
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    )
    .unwrap();
    assert_eq!(rom.location, unit.device.location);
}

/// A second bootloader that arrives while the tool connects, where the bus cannot tell them
/// apart, stops a flash by port before anything is written.
#[test]
fn a_bootloader_arriving_during_a_flash_by_port_stops_it() {
    let bus = SimBus::scenario("isp").unwrap();
    bus.set_same_bootloaders(true);
    let second = bus.add_bootloader(false);
    bus.unplug(second);
    bus.plug_back_later(Duration::from_millis(1), second);
    let result = update::flash_now(
        &bus,
        &firmware_image(&ours("v0.3.0")),
        Some("1-1"),
        &Timing::default(),
        &mut Steps::default(),
        &Cancel::new(),
    );
    let Err(UpdateError::Refused(reason)) = result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("cannot tell which bootloader"), "{reason}");
    assert!(bus.flash(0).is_empty());
}
