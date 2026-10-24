//! A batch: one image and one filter, armed once, applied to every newly attached unit the filter
//! takes, one unit through the bootloader at a time.
//!
//! Every LPC55S69 bootloader enumerates alike, so the batch starts a unit only while no bootloader
//! is on the bus: anything in ISP mode that is not the unit being flashed pauses it, and so does a
//! mini. Units are tracked by port, as a single update finds its unit again, and each is taken at
//! most once until it leaves the bus. A unit that reports no port cannot be followed, so it
//! pauses the batch too.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::fmt;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::time::{Duration, SystemTime};

use serde::Serialize;

use crate::board;
pub use crate::board::Board;
use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{self, LinkError};
use crate::detect::{self, Firmware, Hardware, Jumpered, State, Unit};
use crate::ids::Role;
use crate::image::{Image, ImageError};
use crate::paths::{self, Path};
use crate::progress::{Cancel, Progress};
use crate::source::{self, ImageVersion, Payload, Source};
use crate::update::{self, Outcome, Timing, UpdateError};
use crate::version::{Build, Relation, StockVersion};

/// Polls past [`Timing::settle`] that a new probe may fail to open before the batch skips it.
const OPEN_GRACE_POLLS: u32 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StockFilter {
    None,
    Any,
    /// Stock older than this version.
    Below(StockVersion),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OursFilter {
    None,
    Any,
    /// Ours older than the image, or a build that carries no version to compare.
    Older,
}

/// Which units a batch takes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Filter {
    pub board: Board,
    pub stock: StockFilter,
    pub ours: OursFilter,
    /// Units that arrive in ISP mode, as after fitting the jumper.
    pub bootloaders: bool,
    /// Units attached already when the batch is armed.
    pub attached: bool,
}

impl Filter {
    /// The filter in words, for the question that arms it.
    #[must_use]
    pub fn describe(&self) -> String {
        let mut firmware = Vec::new();
        match self.stock {
            StockFilter::None => {}
            StockFilter::Any => firmware.push("stock".to_owned()),
            StockFilter::Below(version) => firmware.push(format!("stock below V{version}")),
        }
        match self.ours {
            OursFilter::None => {}
            OursFilter::Any => firmware.push("ours".to_owned()),
            OursFilter::Older => firmware.push("ours older than the image".to_owned()),
        }
        let mut kinds = Vec::new();
        if !firmware.is_empty() {
            kinds.push(format!("running {}", firmware.join(" or ")));
        }
        if self.bootloaders {
            kinds.push("in ISP mode".to_owned());
        }
        if kinds.is_empty() {
            return "no unit".to_owned();
        }
        let when = if self.attached {
            "attached now or later"
        } else {
            "attached from now on"
        };
        format!("every {} {}, {when}", self.board, kinds.join(" or "))
    }

    /// Whether a batch with this filter can arm with an image of this kind. Taking ours older than
    /// the image needs an image of ours, which carries a version to compare.
    pub fn check_version(&self, version: &ImageVersion) -> Result<(), ArmError> {
        self.check_dated(matches!(version, ImageVersion::Ours(_)))
    }

    /// [`Filter::check_version`] before the image is loaded, from where it comes: only a release
    /// of ours carries a version of ours.
    pub fn check_source(&self, source: &Source) -> Result<(), ArmError> {
        self.check_dated(matches!(source, Source::Release(_)))
    }

    fn check_dated(&self, dated: bool) -> Result<(), ArmError> {
        if self.ours == OursFilter::Older && !dated {
            Err(ArmError::OursUndated)
        } else {
            Ok(())
        }
    }
}

/// Why a batch does not arm, rather than skip units without saying so.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ArmError {
    #[error(
        "the image carries no version of ours, so the batch cannot tell which units running ours are older than it"
    )]
    OursUndated,
    #[error("the batch takes the {board}, and the image is built for {built_for}")]
    NotForBoard { board: Board, built_for: String },
    #[error(transparent)]
    Image(#[from] ImageError),
}

/// The image a batch installs, loaded and checked once when it is armed.
#[derive(Clone, Debug)]
pub struct Armed {
    pub image: Image,
    pub name: String,
    pub version: ImageVersion,
    /// Where it came from, so that advice can name it.
    pub source: Source,
}

impl Armed {
    #[must_use]
    pub fn payload(&self) -> Payload<'_> {
        Payload::new(&self.image, &self.version)
    }

    /// Whether a batch with `filter` can arm with this image: [`Filter::check_version`], and an
    /// image of ours has to declare the board the batch takes. Stock's own image goes on any.
    pub fn check(&self, filter: &Filter) -> Result<(), ArmError> {
        check(filter, &self.image, &self.version)
    }
}

/// [`Armed::check`], for an image not armed yet.
pub fn check(filter: &Filter, image: &Image, version: &ImageVersion) -> Result<(), ArmError> {
    filter.check_version(version)?;
    if version.is_stock() {
        return Ok(());
    }
    let boards = image.boards()?;
    if boards.contains(&filter.board) {
        Ok(())
    } else {
        Err(ArmError::NotForBoard {
            board: filter.board,
            built_for: board::built_for(&boards),
        })
    }
}

/// What the batch makes of a unit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    Take,
    /// It runs the image already.
    Current,
    Skip(String),
}

/// Whether the batch takes `unit`.
#[must_use]
pub fn judge(unit: &Unit, filter: &Filter, version: &ImageVersion) -> Verdict {
    match &unit.state {
        State::MiniBootloader => {
            return Verdict::Skip("an MCU-Link mini, a different board".into());
        }
        State::Unreadable { reason, .. } => {
            return Verdict::Skip(format!("the tool could not ask what it runs: {reason}"));
        }
        State::SwdJumper {
            firmware: Jumpered::Ours,
        } => {
            return Verdict::Skip(
                "its SWD jumper is fitted, so the tool cannot ask which build it runs".into(),
            );
        }
        State::SwdJumper {
            firmware: Jumpered::Stock(_),
        } => {
            return Verdict::Skip(
                "its SWD jumper is fitted, which takes away the interface stock's update request goes through".into(),
            );
        }
        State::Bootloader | State::Probe { .. } => {}
    }
    if !filter.board.takes(unit) {
        return Verdict::Skip(match unit.hardware() {
            Hardware::Unknown => format!(
                "its firmware does not say which board it is, so it is not known to be {}",
                filter.board.a()
            ),
            other => format!("{other}, not {}", filter.board.a()),
        });
    }
    let taken = match &unit.state {
        State::Bootloader if filter.bootloaders => return Verdict::Take,
        State::Bootloader => {
            return Verdict::Skip("in ISP mode, which this batch does not take".into());
        }
        State::MiniBootloader | State::Unreadable { .. } | State::SwdJumper { .. } => {
            unreachable!("judged above")
        }
        State::Probe {
            firmware: Firmware::Ours(ours),
            ..
        } => {
            let build = Build::parse(&ours.build);
            if let ImageVersion::Ours(tag) = version
                && build.compare(tag) == Relation::UpToDate
            {
                return Verdict::Current;
            }
            match (filter.ours, version) {
                (OursFilter::None, _) => {
                    Verdict::Skip("runs ours, which this batch does not take".into())
                }
                (OursFilter::Any, _) => Verdict::Take,
                (OursFilter::Older, ImageVersion::Ours(tag)) => match build.compare(tag) {
                    Relation::Behind | Relation::Unknown => Verdict::Take,
                    Relation::Ahead => Verdict::Skip(format!("runs {build}, newer than {tag}")),
                    Relation::OtherVariant => {
                        Verdict::Skip(format!("runs {tag}, built as {build}"))
                    }
                    Relation::UpToDate => Verdict::Current,
                },
                (OursFilter::Older, _) => {
                    Verdict::Skip("the image carries no version of ours to compare with".into())
                }
            }
        }
        State::Probe {
            firmware: Firmware::Stock { version: running },
            ..
        } => {
            if let ImageVersion::Stock(image) = version
                && *running == Some(*image)
            {
                return Verdict::Current;
            }
            match (filter.stock, running) {
                (StockFilter::None, _) => {
                    Verdict::Skip("runs stock, which this batch does not take".into())
                }
                (StockFilter::Any, _) => Verdict::Take,
                (StockFilter::Below(limit), Some(running)) if *running < limit => Verdict::Take,
                (StockFilter::Below(limit), Some(running)) => {
                    Verdict::Skip(format!("runs stock V{running}, not below V{limit}"))
                }
                (StockFilter::Below(_), None) => {
                    Verdict::Skip("stock did not report its version".into())
                }
            }
        }
    };
    if taken != Verdict::Take {
        return taken;
    }
    // Whether it can enter its bootloader at all, judged alone: other units on the bus are the
    // batch's to wait for, not a reason to skip this one.
    let offer = paths::offer(unit, std::slice::from_ref(unit));
    match offer.best() {
        Some(Path::OursJumperFree | Path::StockJumperFree) => Verdict::Take,
        Some(Path::Jumper) => {
            Verdict::Skip(format!("needs the ISP jumper. {}", offer.notes.join(" ")))
        }
        _ => Verdict::Skip(offer.notes.join(" ")),
    }
}

/// Where a unit stands in the batch.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    /// Attached when the batch was armed, and left alone.
    LeftAlone,
    Waiting,
    Updated,
    /// Flashed, and back in ISP mode because its jumper is fitted.
    JumperFitted,
    /// Flashed, and still in ISP mode because its bootloader did not restart, as
    /// [`Outcome::NotRestarted`] says.
    NotRestarted {
        maybe_jumper: bool,
    },
    Current,
    Skipped(String),
    Failed(String),
}

/// One unit the batch has seen.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub port: String,
    pub device: UsbDevice,
    pub status: Status,
    pub serial_before: Option<String>,
    pub firmware_before: String,
    /// Its board, as it said when it arrived.
    pub hardware: Hardware,
    /// When the bus first showed it, in the bus's time.
    pub seen: Duration,
}

/// A unit the batch has met, however often it left the bus and came back.
#[derive(Clone, Debug)]
struct Met {
    /// Every serial it reported. A unit's serial changes with its firmware.
    serials: Vec<String>,
    /// How it ended, as far as it has: see [`ended`].
    ended: Option<Settled>,
    /// It did not come back from an update. `Some(true)` once it has been taken once more, after
    /// which it is left alone.
    lost: Option<bool>,
    /// It ran ours when the batch saw it, so its serial is ours' and finds it again.
    ran_ours: bool,
}

/// How a unit that ended as `old` stands after `new`. An update's result always stands; a unit
/// found running the image is current, unless it was updated in this batch; a skip or a wait
/// only stands for a unit with no result yet.
fn ended(old: Option<Settled>, new: Settled) -> Settled {
    match (old, new) {
        (None, new)
        | (
            _,
            new @ (Settled::Updated
            | Settled::JumperFitted
            | Settled::NotRestarted
            | Settled::Failed),
        ) => new,
        (Some(old @ (Settled::Updated | Settled::JumperFitted)), Settled::Current) => old,
        (Some(_), Settled::Current) => Settled::Current,
        (Some(old), Settled::Skipped | Settled::Waiting) => old,
    }
}

/// What became of a unit, as its record line says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Settled {
    Updated,
    /// Flashed, and back in ISP mode because its jumper is fitted.
    JumperFitted,
    /// Flashed and verified, and still in ISP mode because its bootloader did not restart. It
    /// starts the image once it is unplugged and plugged back in.
    NotRestarted,
    /// It runs the image already.
    Current,
    Skipped,
    Failed,
    /// Its turn had not come when the batch stopped.
    Waiting,
}

impl Settled {
    /// The name the record file gives it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Settled::Updated => "updated",
            Settled::JumperFitted => "jumper-fitted",
            Settled::NotRestarted => "not-restarted",
            Settled::Current => "current",
            Settled::Skipped => "skipped",
            Settled::Failed => "failed",
            Settled::Waiting => "waiting",
        }
    }
}

impl fmt::Display for Settled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One line of the batch's record.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Record {
    /// RFC 3339, UTC.
    pub time: String,
    pub port: String,
    pub serial_before: Option<String>,
    pub firmware_before: String,
    pub serial_after: Option<String>,
    pub firmware_after: Option<String>,
    pub result: Settled,
    pub reason: Option<String>,
    pub seconds: Option<f64>,
}

/// Opens a batch's record file to add lines to it, creating it if needed. Lines already there
/// stay.
pub fn open_record(path: &std::path::Path) -> std::io::Result<std::fs::File> {
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
}

/// How many units the batch has met ended each way, each counted once, and how many wait their
/// turn. A unit that did not restart counts as failed: it needs a person.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct Tally {
    pub updated: usize,
    pub current: usize,
    pub skipped: usize,
    pub failed: usize,
    pub waiting: usize,
}

impl fmt::Display for Tally {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} updated, {} current, {} skipped, {} failed, {} waiting",
            self.updated, self.current, self.skipped, self.failed, self.waiting
        )
    }
}

/// What a batch tells its front end, besides each update's progress.
pub trait Observer: Progress {
    /// A unit's update starts.
    fn starting(&mut self, _entry: &Entry) {}
    /// A unit is settled, as its record says.
    fn record(&mut self, _record: &Record) {}
    /// A settled unit can be unplugged. `blinking` says whether its LEDs blink to show which
    /// unit it is, which they do only while the batch runs.
    fn unplug(&mut self, _port: &str, _blinking: bool) {}
    /// The batch pauses, saying why, or resumes with `None`.
    fn paused(&mut self, _reason: Option<&str>) {}
}

/// Ignores everything.
pub struct Unobserved;

impl Progress for Unobserved {}
impl Observer for Unobserved {}

fn port(device: &UsbDevice) -> String {
    device
        .location
        .clone()
        .unwrap_or_else(|| format!("device {}", device.instance))
}

/// The units [`Batch::blink`] lights.
fn blinks(entry: &Entry) -> bool {
    matches!(entry.status, Status::Updated | Status::Current) && entry.device.dap_v2
}

/// Sets the LEDs of `units`, each by its port and device, opening each for the moment it takes.
fn light<B: Bus>(bus: &B, units: &[(String, UsbDevice)], on: bool) {
    for (port, device) in units {
        match bus.open_dap(device) {
            Ok(mut link) => {
                if let Err(error) = dap::set_leds(&mut link, on) {
                    tracing::debug!("{port}: no blink: {error}");
                }
            }
            Err(error) => tracing::debug!("{port}: no blink: {error}"),
        }
    }
}

/// Toggles the LEDs of `units` every `period`, starting from `lit`, until `done` closes, and
/// says whether it left them lit. It runs on a thread of its own while another unit is updated.
///
/// The period is real time, not the bus's: a person watches the LEDs, and a simulated bench's
/// clock belongs to the update under way.
fn keep_blinking<B: Bus>(
    bus: &B,
    units: &[(String, UsbDevice)],
    mut lit: bool,
    period: Duration,
    done: &Receiver<()>,
) -> bool {
    loop {
        lit = !lit;
        light(bus, units, lit);
        match done.recv_timeout(period) {
            Err(RecvTimeoutError::Timeout) => {}
            Ok(()) | Err(RecvTimeoutError::Disconnected) => return lit,
        }
    }
}

/// The bus, noting when each port first shows each device. An update polls the bus while it
/// waits, so units that arrive during one keep their order.
struct Watch<'b, B> {
    bus: &'b B,
    /// By port: the instance there, and when the bus first showed it.
    seen: RefCell<HashMap<String, (String, Duration)>>,
}

impl<B: Bus> Bus for Watch<'_, B> {
    type Dap = B::Dap;
    type Isp = B::Isp;

    fn devices(&self) -> Result<Vec<UsbDevice>, BusError> {
        let devices = self.bus.devices()?;
        let now = self.bus.now();
        let mut seen = self.seen.borrow_mut();
        for device in &devices {
            let first = seen
                .entry(port(device))
                .or_insert_with(|| (device.instance.clone(), now));
            if first.0 != device.instance {
                *first = (device.instance.clone(), now);
            }
        }
        Ok(devices)
    }

    fn open_dap(&self, device: &UsbDevice) -> Result<B::Dap, BusError> {
        self.bus.open_dap(device)
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

    fn open_isp(&self, device: &UsbDevice) -> Result<B::Isp, BusError> {
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

fn now_rfc3339() -> String {
    humantime::format_rfc3339_seconds(SystemTime::now()).to_string()
}

/// An armed batch. Drive it with [`Batch::step`] until [`Batch::finished`], then
/// [`Batch::close`] it.
pub struct Batch<'b, B: Bus> {
    bus: Watch<'b, B>,
    armed: Armed,
    filter: Filter,
    timing: Timing,
    entries: Vec<Entry>,
    queue: VecDeque<String>,
    paused: Option<String>,
    records: Vec<Record>,
    stop: Cancel,
    cancel: Cancel,
    limit: Option<usize>,
    updated: usize,
    /// Every unit the batch has met. An entry is forgotten when its unit leaves the bus; the unit
    /// stays here, so that it is counted once and known again if it comes back.
    met: Vec<Met>,
    /// By port, the unit last met there, as an index into `met`.
    met_at: HashMap<String, usize>,
    /// The instances on the bus at the first look, each kept while it stays on the bus.
    attached: Option<Vec<String>>,
    lit: bool,
    closed: bool,
}

impl<'b, B: Bus + Sync> Batch<'b, B> {
    /// Arms a batch, once [`Armed::check`] passes. `stop` stops it taking units; `cancel` stops
    /// the update under way, as a single update's cancel does. `limit` stops it after that many
    /// units are updated.
    pub fn arm(
        bus: &'b B,
        armed: Armed,
        filter: Filter,
        timing: Timing,
        stop: Cancel,
        cancel: Cancel,
        limit: Option<usize>,
    ) -> Result<Batch<'b, B>, ArmError> {
        armed.check(&filter)?;
        Ok(Batch {
            bus: Watch {
                bus,
                seen: RefCell::new(HashMap::new()),
            },
            armed,
            filter,
            timing,
            entries: Vec::new(),
            queue: VecDeque::new(),
            paused: None,
            records: Vec::new(),
            stop,
            cancel,
            limit,
            updated: 0,
            met: Vec::new(),
            met_at: HashMap::new(),
            attached: None,
            lit: false,
            closed: false,
        })
    }

    #[must_use]
    pub fn armed(&self) -> &Armed {
        &self.armed
    }

    #[must_use]
    pub fn filter(&self) -> &Filter {
        &self.filter
    }

    #[must_use]
    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    #[must_use]
    pub fn records(&self) -> &[Record] {
        &self.records
    }

    /// Why the batch is waiting, when it is.
    #[must_use]
    pub fn paused(&self) -> Option<&str> {
        self.paused.as_deref()
    }

    /// Every unit the batch has met, each once, by how it ended, including those unplugged since;
    /// and the units waiting their turn. A unit that left before it was settled is not counted.
    #[must_use]
    pub fn tally(&self) -> Tally {
        let waiting: Vec<usize> = self
            .entries
            .iter()
            .filter(|e| e.status == Status::Waiting && !self.closed)
            .filter_map(|e| self.met_at.get(&e.port).copied())
            .collect();
        let mut tally = Tally::default();
        for (index, met) in self.met.iter().enumerate() {
            if waiting.contains(&index) {
                tally.waiting += 1;
                continue;
            }
            match met.ended {
                None => {}
                Some(Settled::Updated | Settled::JumperFitted) => tally.updated += 1,
                Some(Settled::Current) => tally.current += 1,
                Some(Settled::Skipped) => tally.skipped += 1,
                Some(Settled::Failed | Settled::NotRestarted) => tally.failed += 1,
                Some(Settled::Waiting) => tally.waiting += 1,
            }
        }
        tally
    }

    /// The unit met before that `unit`, arriving at `at`, is: the one that reported its serial, or,
    /// where a serial cannot tell, the one last on that port if it left after a failed update and
    /// this one is in ISP mode or runs the image. A file carries no version to tell its image by,
    /// so with a file a build of ours stands for it, but only where the image went on the unit that
    /// left (it did not restart, or did not come back) and that unit never ran ours before.
    fn known(&self, at: &str, unit: &Unit, verdict: &Verdict) -> Option<usize> {
        if let Some(serial) = unit.serial()
            && let Some(index) = self
                .met
                .iter()
                .position(|met| met.serials.iter().any(|s| s == serial))
        {
            return Some(index);
        }
        let index = *self.met_at.get(at)?;
        let met = &self.met[index];
        let left_failed = matches!(met.ended, Some(Settled::Failed | Settled::NotRestarted));
        let nameless = unit.serial().is_none() || met.serials.is_empty();
        let runs_ours = matches!(
            unit.state,
            State::Probe {
                firmware: Firmware::Ours(_),
                ..
            }
        );
        let image_went_on = met.ended == Some(Settled::NotRestarted) || met.lost.is_some();
        let runs_image = *verdict == Verdict::Current
            || (self.armed.version == ImageVersion::Unknown
                && runs_ours
                && image_went_on
                && !met.ran_ours);
        (left_failed && (nameless || runs_image)).then_some(index)
    }

    /// Whether it takes no more units: stopped, or at its limit. No update is ever under way
    /// between steps.
    #[must_use]
    pub fn finished(&self) -> bool {
        self.stop.is_cancelled() || self.limit.is_some_and(|limit| self.updated >= limit)
    }

    fn entry(&mut self, port: &str) -> Option<&mut Entry> {
        self.entries.iter_mut().find(|e| e.port == port)
    }

    fn settle(
        &mut self,
        observer: &mut dyn Observer,
        port: &str,
        after: Option<&Unit>,
        seconds: Option<f64>,
    ) {
        let Some(entry) = self.entries.iter().find(|e| e.port == port) else {
            return;
        };
        let (result, reason) = match &entry.status {
            Status::Updated => (Settled::Updated, None),
            Status::JumperFitted => (Settled::JumperFitted, None),
            Status::NotRestarted { maybe_jumper } => (
                Settled::NotRestarted,
                Some(format!(
                    "it was flashed and verified, but {}",
                    update::not_restarted(*maybe_jumper)
                )),
            ),
            Status::Current => (Settled::Current, None),
            Status::Skipped(reason) => (Settled::Skipped, Some(reason.clone())),
            Status::Failed(reason) => (Settled::Failed, Some(reason.clone())),
            Status::Waiting if self.closed => (
                Settled::Waiting,
                Some("the batch stopped before its turn came".to_owned()),
            ),
            Status::Waiting | Status::LeftAlone => return,
        };
        let record = Record {
            time: now_rfc3339(),
            port: entry.port.clone(),
            serial_before: entry.serial_before.clone(),
            firmware_before: entry.firmware_before.clone(),
            serial_after: after.and_then(|u| u.serial().map(str::to_owned)),
            firmware_after: after.map(|u| u.state.to_string()),
            result,
            reason,
            seconds,
        };
        let unplug = blinks(entry);
        if let Some(met) = self.met_at.get(port).and_then(|&i| self.met.get_mut(i)) {
            met.ended = Some(ended(met.ended, result));
            met.serials.extend(record.serial_after.clone());
        }
        observer.record(&record);
        self.records.push(record);
        if unplug {
            observer.unplug(port, !self.finished());
        }
    }

    /// The batch's rule for starting a unit: nothing in ISP mode on the bus, unless it is a
    /// waiting unit the batch takes, and then that one goes first. Nothing starts while a unit
    /// that reports no port is attached either.
    fn guard(&self, devices: &[UsbDevice]) -> Option<String> {
        if let Some(lost) = devices.iter().find(|d| d.location.is_none()) {
            let serial = lost
                .serial
                .as_deref()
                .map(|s| format!(", serial {s}"))
                .unwrap_or_default();
            return Some(format!(
                "A unit ({lost}{serial}) reports no USB port. The batch knows units by their port, so it waits until that unit is unplugged."
            ));
        }
        if let Some(mini) = devices
            .iter()
            .find(|d| d.role() == Some(Role::MiniBootloader))
        {
            return Some(format!(
                "An MCU-Link mini is in ISP mode at {}. The batch waits until it is unplugged.",
                port(mini)
            ));
        }
        let roms: Vec<&UsbDevice> = devices
            .iter()
            .filter(|d| d.role() == Some(Role::Bootloader))
            .collect();
        match roms.as_slice() {
            [] => None,
            [rom] => {
                let at = port(rom);
                let entry = self.entries.iter().find(|e| e.port == at);
                match entry.map(|e| &e.status) {
                    Some(Status::Waiting) => None,
                    Some(Status::Failed(_)) => {
                        let again = source::flash_command(&self.armed.source, &self.armed.version);
                        Some(match entry.map(|e| &e.hardware) {
                            Some(Hardware::Board(board)) => format!(
                                "The unit at {at}, {}, failed and is in ISP mode, maybe with a partial image. The batch waits until it is unplugged. In ISP mode the tool cannot tell its board, so flash it again with the batch's image, which is built for it: `{again}`.",
                                board.a()
                            ),
                            _ => format!(
                                "The unit at {at} failed and is in ISP mode, maybe with a partial image. The batch waits until it is unplugged; `{again}` flashes it again with the batch's image."
                            ),
                        })
                    }
                    Some(Status::JumperFitted) => Some(format!(
                        "The unit at {at} is in ISP mode with its jumper fitted. The batch waits until it is unplugged; take the jumper off before plugging it in again."
                    )),
                    Some(Status::NotRestarted { maybe_jumper }) => Some(format!(
                        "The unit at {at} was flashed, but {}. The batch waits until it is unplugged; plugged back in, it starts the new image.",
                        update::not_restarted(*maybe_jumper)
                    )),
                    _ => Some(format!(
                        "A unit in ISP mode that this batch does not take is attached at {at}. The batch waits until it is unplugged or flashed with `verge isp flash`."
                    )),
                }
            }
            several => Some(format!(
                "{} units are in ISP mode, and every LPC55S69 bootloader looks the same. The batch waits until at most one is.",
                several.len()
            )),
        }
    }

    /// The next unit to start: a waiting bootloader first, since it blocks every other entry.
    fn next(&mut self) -> Option<String> {
        let rom = self.queue.iter().position(|p| {
            self.entries
                .iter()
                .any(|e| &e.port == p && e.device.role() == Some(Role::Bootloader))
        });
        match rom {
            Some(index) => self.queue.remove(index),
            None => self.queue.pop_front(),
        }
    }

    /// Looks at the bus once, and starts at most one unit's update. Without an update to run it
    /// waits one poll.
    pub fn step(&mut self, observer: &mut dyn Observer) -> Result<(), BusError> {
        let devices: Vec<UsbDevice> = self
            .bus
            .devices()?
            .into_iter()
            .filter(|d| d.role().is_some())
            .collect();
        let present: HashMap<String, &UsbDevice> = devices.iter().map(|d| (port(d), d)).collect();

        // A unit that left the bus, or came back as a new enumeration the batch did not cause, is
        // forgotten, and is new when it is next seen.
        self.entries.retain(|e| {
            present
                .get(&e.port)
                .is_some_and(|d| d.instance == e.device.instance)
        });
        let tracked: Vec<String> = self.entries.iter().map(|e| e.port.clone()).collect();
        self.queue.retain(|p| tracked.contains(p));
        self.bus
            .seen
            .borrow_mut()
            .retain(|p, _| present.contains_key(p));
        let attached = self
            .attached
            .get_or_insert_with(|| devices.iter().map(|d| d.instance.clone()).collect());
        attached.retain(|instance| devices.iter().any(|d| d.instance == *instance));
        let attached = attached.clone();

        let now = self.bus.now();
        let seen = self.bus.seen.borrow().clone();
        let first_seen = |d: &UsbDevice| seen.get(&port(d)).map_or(now, |(_, at)| *at);
        let mut arrivals: Vec<&UsbDevice> = devices
            .iter()
            .filter(|d| d.location.is_some() && !tracked.contains(&port(d)))
            .collect();
        arrivals.sort_by_key(|d| (first_seen(d), port(d)));
        for device in arrivals {
            let at = port(device);
            let was_attached = attached.contains(&device.instance);
            let leave_alone = was_attached && !self.filter.attached;
            let probe = device.role() == Some(Role::Probe);
            let since = now.saturating_sub(first_seen(device));
            // A firmware that has just enumerated is given the time the update path gives it.
            if probe && !was_attached && since < self.timing.settle {
                continue;
            }
            let Some(unit) = detect::query_device(&self.bus, device.clone()) else {
                continue;
            };
            // Opening a new device can fail for a moment while the system still sets it up.
            let grace = self.timing.settle + self.timing.poll * OPEN_GRACE_POLLS;
            if probe
                && !leave_alone
                && device.dap_v2
                && matches!(unit.state, State::Unreadable { .. })
                && since < grace
            {
                continue;
            }
            let status = if leave_alone {
                Status::LeftAlone
            } else {
                let verdict = judge(&unit, &self.filter, &self.armed.version);
                let index = match self.known(&at, &unit, &verdict) {
                    Some(index) => index,
                    None => {
                        self.met.push(Met {
                            serials: Vec::new(),
                            ended: None,
                            lost: None,
                            ran_ours: false,
                        });
                        self.met.len() - 1
                    }
                };
                self.met_at.insert(at.clone(), index);
                let met = &mut self.met[index];
                met.serials.extend(unit.serial().map(str::to_owned));
                met.ran_ours |= matches!(
                    unit.state,
                    State::Probe {
                        firmware: Firmware::Ours(_),
                        ..
                    }
                );
                match verdict {
                    Verdict::Take if met.lost == Some(true) => Status::Skipped(
                        "it did not come back from its update, and failed again when it was taken once more, so the batch leaves it alone".into(),
                    ),
                    Verdict::Take => {
                        self.queue.push_back(at.clone());
                        Status::Waiting
                    }
                    Verdict::Current => {
                        met.lost = None;
                        Status::Current
                    }
                    Verdict::Skip(reason) => Status::Skipped(reason),
                }
            };
            self.entries.push(Entry {
                port: at.clone(),
                device: unit.device.clone(),
                status,
                serial_before: unit.serial().map(str::to_owned),
                firmware_before: unit.state.to_string(),
                hardware: unit.hardware(),
                seen: first_seen(device),
            });
            self.settle(observer, &at, None, None);
        }

        let paused = self.guard(&devices);
        if paused != self.paused {
            observer.paused(paused.as_deref());
            self.paused = paused;
        }

        if self.paused.is_none()
            && !self.finished()
            && let Some(at) = self.next()
        {
            return self.run(&at, observer);
        }
        self.blink();
        self.bus.sleep(self.timing.poll);
        Ok(())
    }

    fn run(&mut self, at: &str, observer: &mut dyn Observer) -> Result<(), BusError> {
        let Some(device) = self.entry(at).map(|e| e.device.clone()) else {
            return Ok(());
        };
        // Asked again: it may have changed while it waited.
        let Some(unit) = detect::query_device(&self.bus, device) else {
            return Ok(());
        };
        let verdict = judge(&unit, &self.filter, &self.armed.version);
        if verdict != Verdict::Take {
            if let Some(entry) = self.entry(at) {
                entry.status = match verdict {
                    Verdict::Current => Status::Current,
                    Verdict::Skip(reason) => Status::Skipped(reason),
                    Verdict::Take => unreachable!(),
                };
            }
            self.settle(observer, at, None, None);
            return Ok(());
        }
        // Asking took time, and a stop that arrived meanwhile must still keep the unit out of its
        // bootloader.
        if self.finished() {
            self.queue.push_front(at.to_owned());
            return Ok(());
        }
        if let Some(entry) = self.entries.iter().find(|e| e.port == at) {
            observer.starting(entry);
        }
        let started = self.bus.now();
        // A unit that did not come back is taken once more, and this is that time.
        let index = self.met_at.get(at).copied();
        let was_lost = index.and_then(|i| self.met[i].lost);
        if let (Some(i), Some(false)) = (index, was_lost) {
            self.met[i].lost = Some(true);
        }
        let finished = self.finished_units();
        let (inner, lit, period) = (self.bus.bus, self.lit, self.timing.poll);
        let (result, lit) = std::thread::scope(|scope| {
            let (done, until) = mpsc::channel();
            let finished = &finished;
            let blinker = (!finished.is_empty())
                .then(|| scope.spawn(move || keep_blinking(inner, finished, lit, period, &until)));
            let result = update::install(
                &self.bus,
                &unit,
                self.armed.payload(),
                &self.timing,
                observer,
                &self.cancel,
                &self.stop,
            );
            drop(done);
            let lit = blinker.map_or(lit, |blinker| {
                blinker
                    .join()
                    .unwrap_or_else(|panic| std::panic::resume_unwind(panic))
            });
            (result, lit)
        });
        self.lit = lit;
        if matches!(result, Err(UpdateError::Stopped)) {
            if let Some(i) = index {
                self.met[i].lost = was_lost;
            }
            self.queue.push_front(at.to_owned());
            return Ok(());
        }
        let seconds = Some((self.bus.now() - started).as_secs_f64());
        // Whatever is on the port now is this unit, so the next step does not take it for a new
        // one. A failure to look is returned once the install is recorded.
        let looked = self.bus.devices();
        let now_at = looked
            .as_ref()
            .ok()
            .and_then(|devices| devices.iter().find(|d| port(d) == at).cloned());
        if let Some(met) = index.and_then(|i| self.met.get_mut(i)) {
            match &result {
                Ok(_) => met.lost = None,
                // The first time, it may be taken once more; after that, never.
                Err(UpdateError::NoReturn(_)) => met.lost = Some(met.lost.is_some()),
                Err(_) => {}
            }
        }
        let (status, after) = match result {
            Ok(Outcome::Running(back)) => {
                self.updated += 1;
                (Status::Updated, Some(*back))
            }
            Ok(Outcome::JumperStillFitted) => {
                self.updated += 1;
                (Status::JumperFitted, None)
            }
            Ok(Outcome::NotRestarted { maybe_jumper }) => {
                (Status::NotRestarted { maybe_jumper }, None)
            }
            Err(error) => (Status::Failed(error.to_string()), None),
        };
        if let Some(entry) = self.entry(at) {
            entry.status = status;
            if let Some(device) = after.as_ref().map(|u| u.device.clone()).or(now_at) {
                entry.device = device;
            }
        }
        self.settle(observer, at, after.as_ref(), seconds);
        looked.map(drop)
    }

    /// The finished units still attached, by port and device.
    fn finished_units(&self) -> Vec<(String, UsbDevice)> {
        self.entries
            .iter()
            .filter(|entry| blinks(entry))
            .map(|entry| (entry.port.clone(), entry.device.clone()))
            .collect()
    }

    /// Toggles the LEDs of every finished unit still attached, so a person can see which to
    /// unplug. While a unit is updated, a thread of its own does this instead.
    fn blink(&mut self) {
        self.lit = !self.lit;
        light(self.bus.bus, &self.finished_units(), self.lit);
    }

    /// Ends the batch: a record line for each unit still waiting, and the finished units' LEDs
    /// off. Call it once the batch has finished, and after a step fails.
    pub fn close(&mut self, observer: &mut dyn Observer) {
        if self.closed {
            return;
        }
        self.closed = true;
        let waiting: Vec<String> = self
            .entries
            .iter()
            .filter(|e| e.status == Status::Waiting)
            .map(|e| e.port.clone())
            .collect();
        for at in waiting {
            self.settle(observer, &at, None, None);
        }
        self.lit = true;
        self.blink();
    }

    /// Steps until the batch finishes or a step fails, then closes it.
    pub fn run_to_end(&mut self, observer: &mut dyn Observer) -> Result<(), BusError> {
        let mut result = Ok(());
        while !self.finished() {
            result = self.step(observer);
            if result.is_err() {
                break;
            }
        }
        self.close(observer);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The names the README gives for the record's `result`.
    #[test]
    fn a_result_is_written_as_the_readme_names_it() {
        for (result, name) in [
            (Settled::Updated, "updated"),
            (Settled::JumperFitted, "jumper-fitted"),
            (Settled::NotRestarted, "not-restarted"),
            (Settled::Current, "current"),
            (Settled::Skipped, "skipped"),
            (Settled::Failed, "failed"),
            (Settled::Waiting, "waiting"),
        ] {
            assert_eq!(serde_json::to_value(result).unwrap(), name);
            assert_eq!(result.as_str(), name);
        }
    }
}
