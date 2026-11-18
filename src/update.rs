//! An update from start to finish: into the bootloader, flash, and back to the firmware.

use std::time::Duration;

use mboot::McuBoot;

use crate::board::Board;
use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{self, DapLink, EntryError, LinkError};
use crate::daplink_rs;
use crate::detect::{self, Firmware, Hardware, State, Unit};
use crate::ids::Role;
use crate::image::{Image, ImageError};
use crate::isp::{self, IspError};
use crate::paths::{self, Path};
use crate::progress::{Cancel, Left, Progress, Step};
use crate::source::{self, Payload};

/// How long each wait lasts, in the bus's time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Timing {
    /// How often a wait looks at the bus.
    pub poll: Duration,
    /// From the entry request to the bootloader enumerating. Stock takes about 2.4 s.
    pub bootloader_appears: Duration,
    /// For the bootloader to answer once it has enumerated.
    pub rom_answers: Duration,
    /// From the reset to the firmware enumerating. UM11126 allows five seconds to start.
    pub firmware_returns: Duration,
    /// Between the firmware enumerating and the tool asking it what it runs.
    pub settle: Duration,
}

impl Default for Timing {
    fn default() -> Timing {
        Timing {
            poll: Duration::from_millis(250),
            bootloader_appears: Duration::from_secs(15),
            rom_answers: Duration::from_secs(10),
            firmware_returns: Duration::from_secs(15),
            settle: Duration::from_secs(1),
        }
    }
}

/// Why an update, a flash or a reset did not finish, and what it left behind.
#[derive(Debug, thiserror::Error)]
pub enum UpdateError {
    #[error(transparent)]
    Bus(#[from] BusError),
    #[error(transparent)]
    Image(#[from] ImageError),
    /// Stock or daplink-rs did not take its own request to restart into the bootloader.
    #[error(transparent)]
    Entry(#[from] EntryError),
    #[error("Verge did not take the bootloader request ({0}). Use the ISP jumper")]
    OursEntry(LinkError),
    #[error(transparent)]
    Isp(#[from] IspError),
    #[error("{0}")]
    Refused(String),
    /// A command for a unit in ISP mode found none. A running unit may enter it without the
    /// jumper, which the front end can say.
    #[error(
        "no unit is in ISP mode. Fit the ISP jumper, then plug the unit in: the bootloader samples the pin at reset, so a jumper fitted afterwards does nothing"
    )]
    NoneInIsp,
    #[error(
        "the unit did not come back as the bootloader within {} s. Use the ISP jumper",
        .0.as_secs()
    )]
    NoBootloader(Duration),
    #[error(
        "the bootloader enumerated but did not answer within {} s: {last}",
        .waited.as_secs()
    )]
    RomSilent { waited: Duration, last: String },
    #[error(
        "the unit did not come back within {} s of the reset. If it stays away, fit the ISP jumper and flash it again",
        .0.as_secs()
    )]
    NoReturn(Duration),
    #[error(
        "the unit came back as {id}{}, an ID the tool does not look for, so it cannot reach the unit. The image it was given uses that ID",
        product.as_ref().map(|p| format!(" ({p})")).unwrap_or_default()
    )]
    OtherId { id: String, product: Option<String> },
    #[error(
        "{count} units appeared while the tool waited for this one, and they report no USB port to tell them apart"
    )]
    Ambiguous { count: usize },
    /// A cancel after the unit was asked for its bootloader, saying what it left there.
    #[error("{0}")]
    Cancelled(Left),
    #[error("stopped before anything was sent to the unit, so nothing was changed")]
    Stopped,
}

/// How a flashed unit came back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Running a firmware, as this unit.
    Running(Box<Unit>),
    /// In ISP mode again: its ISP pin is still held low, by the jumper or by a probe on its debug
    /// connector that drives the target's ISP pin, or (after a reset alone) its flash holds no
    /// image the ROM starts.
    JumperStillFitted,
    /// Flashed and verified, but the bootloader was never seen to leave the bus after its reset,
    /// so the unit is still in ISP mode. `maybe_jumper` where a bootloader that restarts with its
    /// jumper fitted keeps its instance, as on Windows: there it may have restarted between two
    /// looks. [`not_restarted`] says what to do.
    NotRestarted { maybe_jumper: bool },
}

/// What happened to a unit whose outcome is [`Outcome::NotRestarted`], and what to do about it.
#[must_use]
pub fn not_restarted(maybe_jumper: bool) -> &'static str {
    if maybe_jumper {
        "its bootloader did not restart, or restarted too quickly to see with the ISP jumper fitted: unplug it, take the jumper off if it is fitted, and plug it back in"
    } else {
        "its bootloader did not restart: unplug it and plug it back in"
    }
}

/// Picks the unit to act on: the one with this serial, or the only one attached.
pub fn select<'a>(units: &'a [Unit], serial: Option<&str>) -> Result<&'a Unit, UpdateError> {
    let found: Vec<&Unit> = match serial {
        Some(serial) => units
            .iter()
            .filter(|u| u.serial() == Some(serial))
            .collect(),
        None => units.iter().collect(),
    };
    match (found.as_slice(), serial) {
        ([unit], _) => Ok(unit),
        ([], Some(serial)) => Err(UpdateError::Refused(format!(
            "no attached MCU-Link has serial {serial}"
        ))),
        ([], None) => Err(UpdateError::Refused("no MCU-Link is attached".into())),
        (several, _) => {
            let names: Vec<String> = several
                .iter()
                .map(|u| match (u.serial(), &u.device.location) {
                    (Some(serial), _) => serial.to_owned(),
                    (None, Some(port)) => format!("one at {port} with no serial"),
                    (None, None) => "one with no serial".to_owned(),
                })
                .collect();
            Err(UpdateError::Refused(format!(
                "{} MCU-Links are attached ({}); pick one by serial",
                several.len(),
                names.join(", ")
            )))
        }
    }
}

/// Looks at the bus until `look` finds something or `limit` runs out. A cancel meanwhile is
/// [`UpdateError::Cancelled`] with what it `left`.
fn wait_until<B: Bus, T>(
    bus: &B,
    timing: &Timing,
    limit: Duration,
    cancel: &Cancel,
    left: Left,
    mut look: impl FnMut(&[UsbDevice]) -> Option<T>,
) -> Result<Option<T>, UpdateError> {
    let deadline = bus.now() + limit;
    loop {
        if cancel.is_cancelled() {
            return Err(UpdateError::Cancelled(left));
        }
        if let Some(found) = look(&bus.devices()?) {
            return Ok(Some(found));
        }
        let now = bus.now();
        if now >= deadline {
            return Ok(None);
        }
        // The last look falls on the deadline, so nothing arriving after it is accepted.
        bus.sleep(timing.poll.min(deadline - now));
    }
}

/// Whether `device` is on the port `of` was on. A device that reports no port is on none.
fn on_port_of(device: &UsbDevice, of: &UsbDevice) -> bool {
    device.location.is_some() && device.location == of.location
}

/// The device among `new` that is the unit `of` once was: the one on its port, or where `of`
/// reported no port, the only one.
fn the_one(new: &[&UsbDevice], of: &UsbDevice) -> Option<Result<UsbDevice, UpdateError>> {
    match (&of.location, new) {
        (Some(_), _) => new
            .iter()
            .find(|d| on_port_of(d, of))
            .map(|d| Ok((*d).clone())),
        (None, []) => None,
        (None, [one]) => Some(Ok((*one).clone())),
        (None, several) => Some(Err(UpdateError::Ambiguous {
            count: several.len(),
        })),
    }
}

/// Whether `unit` may be sent into ISP mode to be flashed afterwards by [`flash_now`], which
/// cannot tell which board a unit in ISP mode is. Only a unit not known to be another board than
/// the full MCU-Link goes, unless `stock` says stock's own image follows, which goes on any board.
/// [`enter_bootloader`] checks it first.
pub fn check_entry(unit: &Unit, stock: bool) -> Result<(), UpdateError> {
    match unit.hardware() {
        Hardware::Board(Board::McuLink) | Hardware::Unknown => Ok(()),
        _ if stock => Ok(()),
        other => Err(UpdateError::Refused(format!(
            "the unit is {other}. In ISP mode nothing tells which board it is, so a flash there could not check that a Verge image is built for it"
        ))),
    }
}

/// Asks a running firmware to restart into its bootloader, and waits for the bootloader, for a
/// flash to follow by [`flash_now`]. `stock` says that flash is stock's own image; see
/// [`check_entry`].
///
/// A cancel that arrives before the request is sent stops it with [`UpdateError::Stopped`].
pub fn enter_bootloader<B: Bus>(
    bus: &B,
    unit: &Unit,
    stock: bool,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<UsbDevice, UpdateError> {
    check_entry(unit, stock)?;
    enter(bus, unit, true, timing, progress, cancel, cancel)
}

/// The jumper-free path `unit` takes into its bootloader, given every unit attached (`all`), or why
/// it has none.
///
/// Another unit in ISP mode closes the jumper-free path, because the flash that follows could not
/// tell the two bootloaders apart. With `alone`, nothing follows: other bootloaders are left out
/// where the bus tells them apart (`apart`, as [`Bus::tells_bootloaders_apart`] says) and this
/// unit has a port to find its own by.
pub fn entry_path(
    unit: &Unit,
    all: &[Unit],
    alone: bool,
    apart: bool,
) -> Result<Path, UpdateError> {
    let considered: Vec<Unit> = if alone && apart && unit.device.location.is_some() {
        all.iter()
            .filter(|u| u.device.role() != Some(Role::Bootloader))
            .cloned()
            .collect()
    } else {
        all.to_vec()
    };
    let offer = paths::offer(unit, &considered);
    let path = offer.paths.iter().copied().find(|path| {
        matches!(
            path,
            Path::OursJumperFree | Path::StockJumperFree | Path::DaplinkRsJumperFree
        )
    });
    path.ok_or_else(|| {
        let mut reason = "this unit has no jumper-free path into its bootloader".to_owned();
        for note in &offer.notes {
            reason.push_str(". ");
            reason.push_str(note.trim_end_matches('.'));
        }
        UpdateError::Refused(reason)
    })
}

/// What to do after [`UpdateError::RomSilent`]: the unit's bootloader enumerated and does not
/// answer, and power starts it afresh.
pub const ROM_SILENT_ADVICE: &str = "Unplug the unit and plug it back in, then try again. Without the ISP jumper it starts the image it holds; with the jumper fitted it comes back in ISP mode.";

/// [`enter_bootloader`], where `stop` keeps the unit out of its bootloader until the request is
/// sent and `cancel` stops the wait after it. `alone` is [`entry_path`]'s.
fn enter<B: Bus>(
    bus: &B,
    unit: &Unit,
    alone: bool,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
    stop: &Cancel,
) -> Result<UsbDevice, UpdateError> {
    let all = detect::detect(bus)?;
    if stop.is_cancelled() {
        return Err(UpdateError::Stopped);
    }
    let before: Vec<String> = all.iter().map(|u| u.device.instance.clone()).collect();
    let path = entry_path(unit, &all, alone, bus.tells_bootloaders_apart())?;

    progress.start(Step::EnterBootloader, 0);
    if let State::SwdJumper { .. } = unit.state {
        if stop.is_cancelled() {
            return Err(UpdateError::Stopped);
        }
        bus.device_control_out(
            &unit.device,
            dap::REBOOT_REQUEST_TYPE,
            dap::REBOOT_REQUEST,
            dap::REBOOT_TO_BOOTLOADER,
            0,
        )?
        .map_err(UpdateError::OursEntry)?;
    } else {
        let mut link = bus.open_dap(&unit.device)?;
        link.drain()
            .map_err(dap::DapError::from)
            .map_err(EntryError::from)?;
        if stop.is_cancelled() {
            return Err(UpdateError::Stopped);
        }
        match (path, &unit.state) {
            (Path::OursJumperFree, _) => {
                dap::request_ours_bootloader(&mut link).map_err(UpdateError::OursEntry)?;
            }
            (Path::DaplinkRsJumperFree, _) => daplink_rs::request_bootloader(&mut link)?,
            (
                _,
                State::Probe {
                    firmware: Firmware::Stock { version },
                    ..
                },
            ) => {
                if let Some(odd) = dap::request_stock_bootloader(&mut link, *version)? {
                    progress.note(&format!(
                        "NXP's firmware accepted the request, and DAP_Disconnect then answered {odd}. Waiting for the bootloader anyway."
                    ));
                }
            }
            _ => unreachable!("offer gives the stock path only to stock"),
        }
    }

    progress.start(Step::WaitForBootloader, 0);
    let found = wait_until(
        bus,
        timing,
        timing.bootloader_appears,
        cancel,
        Left::InIsp,
        |devices| {
            let new: Vec<&UsbDevice> = devices
                .iter()
                .filter(|d| d.role() == Some(Role::Bootloader) && !before.contains(&d.instance))
                .collect();
            the_one(&new, &unit.device)
        },
    )?;
    found.ok_or(UpdateError::NoBootloader(timing.bootloader_appears))?
}

/// The one unit in ISP mode, or why there is not exactly one.
///
/// A mini in ISP mode refuses the flash too, whichever unit was asked for, as the firmware
/// repository's `tools/flash.py` does.
pub fn only_bootloader(devices: &[UsbDevice]) -> Result<UsbDevice, UpdateError> {
    let roms: Vec<&UsbDevice> = devices
        .iter()
        .filter(|d| d.role() == Some(Role::Bootloader))
        .collect();
    let mini = devices
        .iter()
        .any(|d| d.role() == Some(Role::MiniBootloader));
    match roms.as_slice() {
        [] if mini => Err(UpdateError::Refused(
            "the unit in ISP mode is an MCU-Link mini, and verge does not work with the mini"
                .into(),
        )),
        [_] if mini => Err(UpdateError::Refused(MINI_TOO.into())),
        [rom] => Ok((*rom).clone()),
        [] => Err(UpdateError::NoneInIsp),
        several => Err(UpdateError::Refused(format!(
            "{} units are in ISP mode, at {}, and every LPC55S69 bootloader looks the same. Unplug all but one",
            several.len(),
            ports(several)
        ))),
    }
}

const MINI_TOO: &str = "an MCU-Link mini is in ISP mode too, and the tool works with a bootloader only while no other is attached: unplug the mini first";

/// The ports of `devices`, as a list.
fn ports(devices: &[&UsbDevice]) -> String {
    let ports: Vec<&str> = devices
        .iter()
        .map(|d| d.location.as_deref().unwrap_or("a port it does not report"))
        .collect();
    match ports.as_slice() {
        [] => String::new(),
        [one] => (*one).to_owned(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

/// The unit in ISP mode to act on: the one at `port`, as [`UsbDevice::location`] names it, or
/// else the only one ([`only_bootloader`]).
///
/// A port picks one bootloader among several only where the bus opens the one it is given
/// ([`Bus::tells_bootloaders_apart`]). Elsewhere every bootloader looks the same, so a port is
/// taken only while its bootloader is the only one.
pub fn select_bootloader<B: Bus>(
    bus: &B,
    devices: &[UsbDevice],
    port: Option<&str>,
) -> Result<UsbDevice, UpdateError> {
    let roms: Vec<&UsbDevice> = devices
        .iter()
        .filter(|d| d.role() == Some(Role::Bootloader))
        .collect();
    let Some(port) = port else {
        let mini = devices
            .iter()
            .any(|d| d.role() == Some(Role::MiniBootloader));
        if let [first, _, ..] = roms.as_slice()
            && !mini
            && bus.tells_bootloaders_apart()
            && let Some(at) = &first.location
        {
            return Err(UpdateError::Refused(format!(
                "{} units are in ISP mode, at {}. Pick one with --port, as --port {at}, or unplug all but one",
                roms.len(),
                ports(&roms)
            )));
        }
        return only_bootloader(devices);
    };
    let Some(rom) = roms.iter().find(|d| d.location.as_deref() == Some(port)) else {
        return Err(UpdateError::Refused(format!(
            "no unit is in ISP mode at {port}. `verge list` names each unit's port"
        )));
    };
    if roms.len() > 1 && !bus.tells_bootloaders_apart() {
        return Err(UpdateError::Refused(format!(
            "{} units are in ISP mode, and on this system the tool cannot tell which bootloader is the one at {port}. Unplug all but one",
            roms.len()
        )));
    }
    Ok((*rom).clone())
}

/// The unit [`flash_bootloader`] would flash, or why it refuses: [`select_bootloader`]'s choice,
/// and no flash at all while a mini is in ISP mode, with a port or without.
pub fn flash_target<B: Bus>(
    bus: &B,
    devices: &[UsbDevice],
    port: Option<&str>,
) -> Result<UsbDevice, UpdateError> {
    if port.is_some()
        && devices
            .iter()
            .any(|d| d.role() == Some(Role::MiniBootloader))
    {
        return Err(UpdateError::Refused(MINI_TOO.into()));
    }
    select_bootloader(bus, devices, port)
}

/// Opens `rom`'s bootloader and asks what it is, retrying while it enumerates. Then checks that
/// the bootloader [`select_bootloader`] picks is still `rom`: the bus can change during the
/// retries, and nothing has been sent to the unit yet.
fn connect<B: Bus>(
    bus: &B,
    rom: &UsbDevice,
    port: Option<&str>,
    timing: &Timing,
    cancel: &Cancel,
) -> Result<(McuBoot<B::Isp>, isp::RomInfo), UpdateError> {
    let deadline = bus.now() + timing.rom_answers;
    let (boot, info) = loop {
        if cancel.is_cancelled() {
            return Err(UpdateError::Cancelled(Left::InIsp));
        }
        let last = match bus.open_isp(rom) {
            Ok(protocol) => {
                let mut boot = McuBoot::new(protocol);
                match isp::rom_info(&mut boot) {
                    Ok(info) => break (boot, info),
                    Err(error) => error.to_string(),
                }
            }
            Err(error) => error.to_string(),
        };
        if bus.now() >= deadline {
            return Err(UpdateError::RomSilent {
                waited: timing.rom_answers,
                last,
            });
        }
        bus.sleep(timing.poll * 2);
    };
    tracing::info!("bootloader {} answers", info.version);
    let answering = select_bootloader(bus, &bus.devices()?, port)?;
    if answering.instance != rom.instance {
        return Err(UpdateError::Refused(format!(
            "the unit in ISP mode changed while the tool connected to it ({rom} then {answering}), so nothing was written. Check what is attached and try again"
        )));
    }
    Ok((boot, info))
}

/// A unit flashed and reset in ISP mode, for [`wait_for_return`].
#[derive(Clone, Debug)]
pub struct Flashed {
    pub rom: UsbDevice,
    /// Every instance on the bus just before the reset.
    before: Vec<String>,
    /// The unit's serial from before it entered ISP mode, where the tool knew the unit.
    serial: Option<String>,
    /// What a cancel while it restarts leaves: a flashed image, or the one it held.
    left: Left,
}

/// Flashes the unit in ISP mode, at `port` or the only one ([`flash_target`]), verifies it, and
/// resets it.
pub fn flash_bootloader<B: Bus>(
    bus: &B,
    image: &Image,
    port: Option<&str>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<Flashed, UpdateError> {
    image.check_for_mculink()?;
    let rom = flash_target(bus, &bus.devices()?, port)?;

    progress.start(Step::Connect, 0);
    let (mut boot, info) = connect(bus, &rom, port, timing, cancel)?;
    isp::program(&mut boot, image, &info, progress, cancel).map_err(|error| match error {
        IspError::Cancelled(left) => UpdateError::Cancelled(left),
        error => UpdateError::Isp(error),
    })?;
    progress.start(Step::Reset, 0);
    let before = bus.devices()?.into_iter().map(|d| d.instance).collect();
    isp::reset(&mut boot);
    Ok(Flashed {
        rom,
        before,
        serial: None,
        left: Left::Flashed,
    })
}

/// Waits for a reset unit to come back, and says what it came back as.
pub fn wait_for_return<B: Bus>(
    bus: &B,
    flashed: &Flashed,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<Outcome, UpdateError> {
    progress.start(Step::WaitForFirmware, 0);
    let rom = &flashed.rom;
    // A unit can leave and come back between two looks at the bus, so its leaving is not waited
    // for: any instance on the ROM's port that was not there before the reset is the unit. On
    // Windows a bootloader that comes back to the same port keeps its instance, so there it is
    // the unit only once it has been seen gone.
    let mut gone = false;
    let back = wait_until(
        bus,
        timing,
        timing.firmware_returns,
        cancel,
        flashed.left,
        |devices| {
            if !devices.iter().any(|d| d.instance == rom.instance) {
                gone = true;
            }
            let new: Vec<&UsbDevice> = devices
                .iter()
                .filter(|d| d.role().is_some())
                .filter(|d| {
                    !flashed.before.contains(&d.instance) || (gone && d.instance == rom.instance)
                })
                .collect();
            the_one(&new, rom)
        },
    )?;
    let device = match back {
        Some(found) => found?,
        None if !gone && bus.devices()?.iter().any(|d| d.instance == rom.instance) => {
            return Ok(Outcome::NotRestarted {
                maybe_jumper: bus.keeps_instances(),
            });
        }
        None => {
            // An image built for another ID boots where `devices` does not look. Its serial is
            // the unit's only if the unit ran ours before, so the port says more where there is one.
            let elsewhere = bus.every_device()?.into_iter().find(|d| {
                d.role().is_none()
                    && match (&rom.location, &flashed.serial) {
                        (Some(_), _) => on_port_of(d, rom),
                        (None, Some(serial)) => d.serial.as_ref() == Some(serial),
                        (None, None) => false,
                    }
            });
            return Err(match elsewhere {
                Some(device) => UpdateError::OtherId {
                    id: format!("{:04x}:{:04x}", device.vid, device.pid),
                    product: device.product,
                },
                None => UpdateError::NoReturn(timing.firmware_returns),
            });
        }
    };
    match device.role() {
        Some(Role::Bootloader) => Ok(Outcome::JumperStillFitted),
        _ => {
            bus.sleep(timing.settle);
            let units = detect::detect(bus)?;
            let unit = units
                .into_iter()
                .find(|u| u.device.instance == device.instance)
                .ok_or(UpdateError::NoReturn(timing.firmware_returns))?;
            Ok(Outcome::Running(Box::new(unit)))
        }
    }
}

/// The jumper-free update: into the bootloader, flash, reset, and back. Refuses an image of ours
/// that does not declare the unit's board, before anything is sent to the unit.
pub fn update<B: Bus>(
    bus: &B,
    unit: &Unit,
    payload: Payload<'_>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<Outcome, UpdateError> {
    update_unless(bus, unit, payload, timing, progress, cancel, cancel)
}

fn update_unless<B: Bus>(
    bus: &B,
    unit: &Unit,
    payload: Payload<'_>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
    stop: &Cancel,
) -> Result<Outcome, UpdateError> {
    let image = payload.image;
    image.check_for_mculink()?;
    source::check_board(Some(unit), image, payload.version)
        .map_err(|error| UpdateError::Refused(error.to_string()))?;
    progress.plan(&[
        Step::EnterBootloader,
        Step::WaitForBootloader,
        Step::Connect,
        Step::Erase,
        Step::Write,
        Step::Verify,
        Step::Reset,
        Step::WaitForFirmware,
    ]);
    enter(bus, unit, false, timing, progress, cancel, stop)?;
    let flashed = Flashed {
        serial: unit.device.serial.clone(),
        ..flash_bootloader(bus, image, None, timing, progress, cancel)?
    };
    wait_for_return(bus, &flashed, timing, progress, cancel)
}

/// Installs `image` on `unit` by the best path its state allows: the jumper-free update, or a
/// flash when it is in ISP mode already. Refuses, saying why, when neither is open to it.
///
/// `stop` asks it to start nothing: it is checked up to the bootloader request, or before a unit
/// already in ISP mode is connected to, and gives [`UpdateError::Stopped`]. After that only
/// `cancel` stops the update. A single update passes the same flag as both.
pub fn install<B: Bus>(
    bus: &B,
    unit: &Unit,
    payload: Payload<'_>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
    stop: &Cancel,
) -> Result<Outcome, UpdateError> {
    let units = detect::detect(bus)?;
    if stop.is_cancelled() {
        return Err(UpdateError::Stopped);
    }
    let offer = paths::offer(unit, &units);
    match offer.best() {
        Some(Path::FlashNow) => flash_now(bus, payload.image, None, timing, progress, cancel),
        Some(Path::OursJumperFree | Path::StockJumperFree | Path::DaplinkRsJumperFree) => {
            update_unless(bus, unit, payload, timing, progress, cancel, stop)
        }
        Some(Path::Jumper) => Err(UpdateError::Refused(format!(
            "this unit cannot be updated without the jumper now. {} To use the jumper: unplug the unit, fit the ISP jumper, plug it back in, and flash it with an image built for {}. In ISP mode the tool cannot check the board",
            offer.notes.join(" "),
            match unit.hardware() {
                Hardware::Unknown => "it".to_owned(),
                known => known.to_string(),
            }
        ))),
        None => Err(UpdateError::Refused(format!(
            "this unit cannot be updated as it is. {}",
            offer.notes.join(" ")
        ))),
    }
}

/// Flashes the unit that is in ISP mode already, as after fitting the jumper: the one at `port`,
/// or the only one.
pub fn flash_now<B: Bus>(
    bus: &B,
    image: &Image,
    port: Option<&str>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<Outcome, UpdateError> {
    progress.plan(&[
        Step::Connect,
        Step::Erase,
        Step::Write,
        Step::Verify,
        Step::Reset,
        Step::WaitForFirmware,
    ]);
    let flashed = flash_bootloader(bus, image, port, timing, progress, cancel)?;
    wait_for_return(bus, &flashed, timing, progress, cancel)
}

/// Resets a unit out of ISP mode without flashing it, so it starts the image it holds: the one at
/// `port`, or the only one. With its ISP pin still held low it comes back in ISP mode.
pub fn reset_now<B: Bus>(
    bus: &B,
    port: Option<&str>,
    timing: &Timing,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<Outcome, UpdateError> {
    progress.plan(&[Step::Connect, Step::Reset, Step::WaitForFirmware]);
    let rom = select_bootloader(bus, &bus.devices()?, port)?;
    progress.start(Step::Connect, 0);
    let (mut boot, _) = connect(bus, &rom, port, timing, cancel)?;
    progress.start(Step::Reset, 0);
    let before = bus.devices()?.into_iter().map(|d| d.instance).collect();
    isp::reset(&mut boot);
    let flashed = Flashed {
        rom,
        before,
        serial: None,
        left: Left::Reset,
    };
    wait_for_return(bus, &flashed, timing, progress, cancel)
}

/// How long a unit [`restart`] restarts has to come back on USB.
pub const RESTART_RETURNS: Duration = Duration::from_secs(5);

/// Why [`restart`] failed. Each front end words it.
#[derive(Debug, thiserror::Error)]
pub enum RestartError {
    /// The unit could not be opened to send the request.
    #[error(transparent)]
    Open(BusError),
    /// The unit did not take the request.
    #[error(transparent)]
    Request(LinkError),
    /// Looking at the bus for the unit's return failed.
    #[error(transparent)]
    Look(BusError),
    /// The unit did not come back within [`RESTART_RETURNS`].
    #[error("it did not come back within {} s", RESTART_RETURNS.as_secs())]
    NoReturn,
}

/// Restarts a unit running Verge, as unplugging it would, and waits for it to come back. The
/// request goes to the probe's USB core rather than through CMSIS-DAP, so it reaches a unit a
/// debugger holds.
pub fn restart<B: Bus>(bus: &B, unit: &Unit) -> Result<(), RestartError> {
    bus.device_control_out(
        &unit.device,
        dap::REBOOT_REQUEST_TYPE,
        dap::REBOOT_REQUEST,
        dap::REBOOT_RESTART,
        0,
    )
    .map_err(RestartError::Open)?
    .map_err(RestartError::Request)?;
    // It leaves USB, resets and enumerates again. Windows can give it back the same instance, so
    // there it counts as back only once it has been seen gone.
    let deadline = bus.now() + RESTART_RETURNS;
    let mut gone = false;
    while bus.now() < deadline {
        bus.sleep(Duration::from_millis(100));
        let devices = bus.devices().map_err(RestartError::Look)?;
        match devices.iter().find(|d| d.serial == unit.device.serial) {
            None => gone = true,
            Some(device) if gone || device.instance != unit.device.instance => {
                if device.role() == Some(Role::Probe) {
                    return Ok(());
                }
            }
            Some(_) => {}
        }
    }
    Err(RestartError::NoReturn)
}

/// What the bootloader of the unit in ISP mode says about itself, and the unit: the one at
/// `port`, or the only one. Changes nothing on it.
pub fn rom_info_now<B: Bus>(
    bus: &B,
    port: Option<&str>,
    timing: &Timing,
    cancel: &Cancel,
) -> Result<(UsbDevice, isp::RomInfo), UpdateError> {
    let rom = select_bootloader(bus, &bus.devices()?, port)?;
    let (_, info) = connect(bus, &rom, port, timing, cancel)?;
    Ok((rom, info))
}
