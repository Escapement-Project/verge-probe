//! A simulated bench: MCU-Links that answer like the recorded ones, in simulated time.
//!
//! What it rests on, and what it does not:
//!
//! - `DAP_Info` answers come from `recorded.json`, copied from the bench's throughput records.
//! - Stock's entry (`81 01` answered `81 00 01`, off the bus 2.1 s later, the bootloader 0.3 s
//!   after that) is as measured. Ours' entry and the bootloader's reset are not measured; the
//!   simulator takes 0.5 s and 1 s for them.
//! - The bootloader keeps flash's rules as UM11126 and the firmware's BENCH.md state them: erase
//!   before write, writes start on a page and fill it, 56-byte HID payloads, and a read of an
//!   erased page fails and drops the bootloader off the bus. Its property values are assumptions,
//!   listed in `sim/rom.rs`.
//! - A unit boots whatever its flash holds. Images from [`firmware_srecord`] carry a marker in
//!   page 0, after the vector table, that says which firmware they are. Every image rewrites
//!   page 0, so a smaller image does not boot as the larger one it replaced, and an image without
//!   the marker boots to nothing, as an unknown image would on the bench.

mod dap;
pub mod http;
mod rom;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use serde::Deserialize;

use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{self as cmsis_dap, LinkError, Ours};
use crate::ids::{self, Role};
use crate::image::{Image, PAGE};
use crate::version::StockVersion;

pub use dap::SimDap;
pub use rom::SimRom;

/// A firmware a simulated unit can run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SimFirmware {
    Ours {
        build: String,
    },
    Stock {
        version: StockVersion,
    },
    /// Stock V2.x, which speaks HID only.
    StockHid {
        version: StockVersion,
    },
}

impl SimFirmware {
    fn marker(&self) -> String {
        match self {
            SimFirmware::Ours { build } => format!("SIMFW:ours:{build};"),
            SimFirmware::Stock { version } => format!("SIMFW:stock:{version};"),
            SimFirmware::StockHid { version } => format!("SIMFW:stockhid:{version};"),
        }
    }

    fn from_marker(text: &str) -> Option<SimFirmware> {
        let (kind, value) = text.split_once(':')?;
        match kind {
            "ours" => Some(SimFirmware::Ours {
                build: value.to_owned(),
            }),
            "stock" => Some(SimFirmware::Stock {
                version: StockVersion::parse(value)?,
            }),
            "stockhid" => Some(SimFirmware::StockHid {
                version: StockVersion::parse(value)?,
            }),
            _ => None,
        }
    }
}

/// Where the marker sits in page 0.
const MARKER_AT: usize = 0x40;

/// Where an image that declares its boards carries the declaration: just after CPU 0's vector
/// table, as the firmware puts it.
const DECLARATION_AT: usize = 0x130;

/// An S-record for `firmware`: a vector table, the marker, and a second region at `0x40000` where
/// ours keeps CPU 1's image, so that a flash covers two page runs. It declares no boards.
#[must_use]
pub fn firmware_srecord(firmware: &SimFirmware) -> String {
    srecord(firmware, None)
}

/// [`firmware_srecord`], with a board declaration in format 1 whose boards field is `boards`.
#[must_use]
pub fn firmware_srecord_declaring(firmware: &SimFirmware, boards: u32) -> String {
    srecord(firmware, Some(boards))
}

fn srecord(firmware: &SimFirmware, boards: Option<u32>) -> String {
    let mut code = Vec::new();
    code.extend_from_slice(&0x2001_0000u32.to_le_bytes());
    code.extend_from_slice(&0x0000_0201u32.to_le_bytes());
    code.resize(MARKER_AT, 0);
    code.extend_from_slice(firmware.marker().as_bytes());
    if let Some(boards) = boards {
        code.resize(DECLARATION_AT, 0);
        code.extend_from_slice(crate::image::DECLARATION_MAGIC);
        code.extend_from_slice(&[1, 16, 0, 0]);
        code.extend_from_slice(&boards.to_le_bytes());
    }
    code.resize(0x200, 0);
    code.extend_from_slice(&[0x70, 0x47]); // bx lr, at the reset vector
    code.resize(0x1234, 0x5A);
    let core1 = vec![0xC1; 0x600];
    let image = Image {
        format: crate::image::Format::SRecord,
        chunks: vec![
            crate::image::Chunk {
                address: 0,
                data: code,
            },
            crate::image::Chunk {
                address: 0x40000,
                data: core1,
            },
        ],
        header: None,
        file_sha256: [0; 32],
    };
    image.to_srecord("simulated")
}

#[must_use]
pub fn firmware_image(firmware: &SimFirmware) -> Image {
    Image::parse(firmware_srecord(firmware).as_bytes()).expect("the simulator's own image parses")
}

/// What a simulated unit is doing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    Firmware(SimFirmware),
    Bootloader,
    /// Off the bus: between a reset and the next enumeration, or with nothing bootable in flash.
    Silent,
}

/// One simulated MCU-Link.
#[derive(Clone, Debug)]
pub struct SimUnit {
    pub location: String,
    /// The serial ours derives from this unit's unique id.
    pub ours_serial: String,
    /// The serial stock reports for this unit.
    pub stock_serial: String,
    /// Programmed pages by base address. A page that is absent is erased.
    pub flash: BTreeMap<u32, Vec<u8>>,
    pub mode: Mode,
    pub instance: Option<String>,
    /// Which enumeration this is, unique on the bench. A handle opened on an earlier one is
    /// dead, as a real one is, even where the instance comes back the same.
    pub generation: u64,
    /// The ISP jumper is fitted, so every reset comes back as the bootloader.
    pub jumper: bool,
    /// Another program holds the CMSIS-DAP interface.
    pub busy: bool,
    /// An MCU-Link mini, whose bootloader enumerates as `1fc9:0022`.
    pub mini: bool,
    /// Replies an earlier host left unread on the bulk IN endpoint.
    pub stale_replies: Vec<Vec<u8>>,
    /// In a USB port. An unplugged unit stays off the bus whatever is scheduled for it.
    pub plugged: bool,
    /// Its bootloader refuses every write, as a failing flash would.
    pub fail_writes: bool,
    /// Its bootloader answers the reset command and then stays on the bus, as one whose reset
    /// failed would.
    pub ignores_reset: bool,
    /// How many `DAP_HostStatus` commands it has answered: how often its LEDs were set.
    pub host_status: u32,
    /// Whether the last `DAP_HostStatus` turned its LEDs on.
    pub leds_on: bool,
    /// The board, which stock names in its product string and ours in a format 3 identify
    /// reply. `None` is the plain MCU-Link as recorded: stock's recorded string, and ours
    /// answering in format 2.
    pub board: Option<u8>,
    /// Stock's USB product string, where it is not the one for its board.
    pub stock_product: Option<String>,
    /// The SWD jumper is fitted, so a running firmware offers no CMSIS-DAP interface.
    pub swd_jumper: bool,
    /// The two-pin scan formats ours offers, bit n for SCNFMT n, which a restart sets back.
    pub scan_formats: u32,
    /// The DIO pins a Pro running ours drives, bit n for DIOn. A restart makes them inputs again.
    pub dio_outputs: u8,
    /// Each DIO pin's output level, which a restart clears.
    pub dio_levels: u8,
    /// What something attached holds the DIO pins at, which an input reads: low, the Pro's own
    /// pull-downs, unless set.
    pub dio_held: u8,
}

/// What ours offers after a restart: OScan1, OScan2 and OScan5.
pub(crate) const SIM_SCAN_FORMATS: u32 = (1 << crate::dap::SCNFMT_OSCAN1)
    | (1 << crate::dap::SCNFMT_OSCAN2)
    | (1 << crate::dap::SCNFMT_OSCAN5);

#[derive(Clone, Debug)]
pub(crate) enum Event {
    Leave,
    Appear(Mode),
    Boot,
    PlugIn,
    Unplug,
}

#[derive(Deserialize)]
struct Recorded {
    records: Vec<Record>,
}

#[derive(Deserialize, Clone)]
struct Record {
    label: String,
    dap_info: RecordedInfo,
    ours: Option<RecordedOurs>,
}

#[derive(Deserialize, Clone)]
pub(crate) struct RecordedInfo {
    pub vendor: String,
    pub product: String,
    #[serde(rename = "protocol version")]
    pub protocol: String,
    #[serde(rename = "max packet count")]
    pub packet_count: u8,
    #[serde(rename = "max packet size")]
    pub packet_size: u16,
}

#[derive(Deserialize, Clone)]
pub(crate) struct RecordedOurs {
    pub format: u8,
    pub flags: u8,
}

pub(crate) struct World {
    pub now: Duration,
    /// Real time per simulated second, so that a demonstration can be watched.
    pace: Option<f64>,
    next_generation: u64,
    pub units: Vec<SimUnit>,
    events: Vec<(Duration, usize, Event)>,
    pub log: Vec<String>,
    pub ours: (RecordedInfo, RecordedOurs),
    pub stock: RecordedInfo,
    /// The most LPC55S69 bootloaders that were ever on the bus at once.
    pub max_bootloaders: usize,
    pub boot_after_reset: Duration,
    /// Whether devices report the port they are on, as every platform the tool knows does.
    locations: bool,
    /// Whether a device that comes back keeps its instance, as a Windows device instance ID
    /// does: by serial number, or by port for a device without one.
    stable_instances: bool,
    /// Whether opening a bootloader opens the first one on the bus, whichever was asked for, as
    /// where nothing ties a HID device to its USB device.
    same_bootloaders: bool,
}

impl World {
    fn new() -> World {
        let recorded: Recorded = serde_json::from_str(include_str!("recorded.json"))
            .expect("recorded.json matches the records it was copied from");
        let find = |label: &str| {
            recorded
                .records
                .iter()
                .find(|r| r.label == label)
                .cloned()
                .expect("recorded.json has the gcc and stock records")
        };
        let gcc = find("gcc");
        let stock = find("stock");
        World {
            now: Duration::ZERO,
            pace: None,
            next_generation: 1,
            units: Vec::new(),
            events: Vec::new(),
            log: Vec::new(),
            ours: (gcc.dap_info, gcc.ours.expect("the gcc record ran ours")),
            stock: stock.dap_info,
            max_bootloaders: 0,
            boot_after_reset: Duration::from_millis(1000),
            locations: true,
            stable_instances: false,
            same_bootloaders: false,
        }
    }

    fn enumerate(&mut self, unit: usize) {
        let generation = self.next_generation;
        self.next_generation += 1;
        let u = &self.units[unit];
        let instance = match (&u.mode, self.stable_instances) {
            (_, false) => format!("sim-{generation}"),
            (Mode::Firmware(SimFirmware::Ours { .. }), true) => {
                format!("sim-0143-{}", u.ours_serial)
            }
            (Mode::Firmware(_), true) => format!("sim-0143-{}", u.stock_serial),
            (Mode::Bootloader | Mode::Silent, true) => format!("sim-rom-{}", u.location),
        };
        let u = &mut self.units[unit];
        u.instance = Some(instance);
        u.generation = generation;
        self.log
            .push(format!("{}: enumerates as {:?}", u.location, u.mode));
        let bootloaders = self
            .units
            .iter()
            .filter(|u| u.instance.is_some() && u.mode == Mode::Bootloader && !u.mini)
            .count();
        self.max_bootloaders = self.max_bootloaders.max(bootloaders);
    }

    pub fn schedule(&mut self, after: Duration, unit: usize, event: Event) {
        self.events.push((self.now + after, unit, event));
        self.events.sort_by_key(|(at, _, _)| *at);
    }

    /// Runs every event due by now.
    pub fn settle(&mut self) {
        while let Some((at, _, _)) = self.events.first() {
            if *at > self.now {
                break;
            }
            let (_, unit, event) = self.events.remove(0);
            self.apply(unit, event);
        }
    }

    fn apply(&mut self, unit: usize, event: Event) {
        if !self.units[unit].plugged && !matches!(event, Event::PlugIn) {
            return;
        }
        match event {
            Event::PlugIn => {
                let u = &mut self.units[unit];
                u.plugged = true;
                self.log.push(format!("{}: plugged in", u.location));
                self.apply(unit, Event::Boot);
            }
            Event::Unplug => {
                self.apply(unit, Event::Leave);
                let u = &mut self.units[unit];
                u.plugged = false;
                self.log.push(format!("{}: unplugged", u.location));
            }
            Event::Leave => {
                let u = &mut self.units[unit];
                u.instance = None;
                u.mode = Mode::Silent;
                self.log.push(format!("{}: leaves the bus", u.location));
            }
            Event::Appear(mode) => {
                self.units[unit].mode = mode;
                self.enumerate(unit);
            }
            Event::Boot => {
                self.units[unit].scan_formats = SIM_SCAN_FORMATS;
                self.units[unit].dio_outputs = 0;
                self.units[unit].dio_levels = 0;
                let u = &self.units[unit];
                // The ROM starts ISP mode when flash holds no image at all (UM11126, passive boot).
                let blank = u
                    .flash
                    .get(&0)
                    .is_none_or(|page| page.iter().all(|&b| b == 0xFF));
                let mode = if u.jumper || blank {
                    Mode::Bootloader
                } else {
                    match boot_from(&u.flash) {
                        Some(firmware) => Mode::Firmware(firmware),
                        None => Mode::Silent,
                    }
                };
                if mode == Mode::Silent {
                    let location = u.location.clone();
                    self.log.push(format!(
                        "{location}: nothing in flash boots; it stays off the bus"
                    ));
                    self.units[unit].mode = Mode::Silent;
                } else {
                    self.units[unit].mode = mode;
                    self.enumerate(unit);
                }
            }
        }
    }

    /// Moves time on, running events as they fall due. Returns how long to sleep for real.
    pub fn advance(&mut self, by: Duration) -> Duration {
        let target = self.now + by;
        while let Some((at, _, _)) = self.events.first() {
            if *at > target {
                break;
            }
            let (at, unit, event) = self.events.remove(0);
            self.now = self.now.max(at);
            self.apply(unit, event);
        }
        self.now = target;
        self.pace.map_or(Duration::ZERO, |pace| by.mul_f64(pace))
    }

    pub fn is_current(&self, unit: usize, generation: u64) -> bool {
        self.units[unit].instance.is_some() && self.units[unit].generation == generation
    }

    /// A control request to the unit, as the firmware's `tud_vendor_control_xfer_cb` takes it:
    /// ours restarts on the reboot request, and everything else stalls.
    pub(crate) fn control_out(
        &mut self,
        unit: usize,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<(), LinkError> {
        let location = self.units[unit].location.clone();
        self.log.push(format!(
            "{location}: control 0x{request_type:02x} 0x{request:02x} wValue {value} wIndex {index}"
        ));
        let ours = matches!(
            self.units[unit].mode,
            Mode::Firmware(SimFirmware::Ours { .. })
        );
        if !ours
            || request_type != cmsis_dap::REBOOT_REQUEST_TYPE
            || request != cmsis_dap::REBOOT_REQUEST
            || index != 0
            || value > 1
        {
            return Err(LinkError::Usb("the request stalled".into()));
        }
        // The firmware holds the disconnect for 20 ms before it resets (its src/reboot.c).
        self.schedule(Duration::from_millis(20), unit, Event::Leave);
        let after = Duration::from_millis(500);
        if value == cmsis_dap::REBOOT_TO_BOOTLOADER {
            self.schedule(after, unit, Event::Appear(Mode::Bootloader));
        } else {
            self.schedule(after, unit, Event::Boot);
        }
        Ok(())
    }

    fn device(&self, unit: usize) -> Option<UsbDevice> {
        let u = &self.units[unit];
        let instance = u.instance.clone()?;
        let (vid, pid) = match (&u.mode, u.mini) {
            (Mode::Firmware(_), _) => ids::id_for(Role::Probe),
            (Mode::Bootloader, false) => ids::id_for(Role::Bootloader),
            (Mode::Bootloader, true) => ids::id_for(Role::MiniBootloader),
            (Mode::Silent, _) => return None,
        };
        let debug = !u.swd_jumper;
        let (serial, manufacturer, product, dap_v2, hid) = match &u.mode {
            Mode::Firmware(SimFirmware::Ours { .. }) => (
                Some(u.ours_serial.clone()),
                Some(self.ours.0.vendor.clone()),
                Some(ours_product(&self.ours.0.product, u.board)),
                debug,
                debug,
            ),
            Mode::Firmware(SimFirmware::Stock { version }) => (
                Some(u.stock_serial.clone()),
                Some(self.stock.vendor.clone()),
                Some(
                    u.stock_product
                        .clone()
                        .unwrap_or_else(|| stock_product(u.board, *version, u.swd_jumper)),
                ),
                debug,
                // The power HID stays on the Pro, and the plain board keeps only its VCOM.
                debug || u.board == Some(Ours::BOARD_MCU_LINK_PRO),
            ),
            Mode::Firmware(SimFirmware::StockHid { version }) => (
                Some(u.stock_serial.clone()),
                Some(self.stock.vendor.clone()),
                Some(format!("MCU-LINK CMSIS-DAP V{version}")),
                false,
                true,
            ),
            Mode::Bootloader | Mode::Silent => (
                None,
                Some("NXP SEMICONDUCTOR INC".to_owned()),
                Some("USB COMPOSITE DEVICE".to_owned()),
                false,
                true,
            ),
        };
        Some(UsbDevice {
            vid,
            pid,
            location: self.locations.then(|| u.location.clone()),
            instance,
            serial,
            manufacturer,
            product,
            bcd_device: match &u.mode {
                Mode::Firmware(SimFirmware::Ours { .. }) => ours_bcd_device(u),
                // The Pro's with no jumper (mculink-stock-notes); nothing reads stock's.
                Mode::Firmware(_) => 0x0337,
                Mode::Bootloader | Mode::Silent => 0x0100,
            },
            dap_v2,
            hid,
        })
    }

    pub fn unit_for(&self, device: &UsbDevice) -> Option<usize> {
        self.units
            .iter()
            .position(|u| u.instance.as_deref() == Some(device.instance.as_str()))
    }
}

/// Stock's USB product string on `board`, in the form LinkServer's Readme gives. The straps are
/// the plain board's and the Pro's as stock V3.172 reported them; the on-board one's are made up.
/// The SWD jumper clears 0x080 of them.
fn stock_product(board: Option<u8>, version: StockVersion, swd_jumper: bool) -> String {
    let (name, straps) = match board {
        None | Some(Ours::BOARD_MCU_LINK) => ("", 0x0FB),
        Some(Ours::BOARD_MCU_LINK_PRO) => ("Pro ", 0x1CF),
        Some(_) => ("FRDM-MCXN947 ", 0x0E7),
    };
    let straps = if swd_jumper { straps & !0x080 } else { straps };
    format!("MCU-LINK {name}(r{straps:03X}) CMSIS-DAP V{version}")
}

/// Ours' product string, which names the Pro (`DAP_ProductName` in the firmware).
fn ours_product(recorded: &str, board: Option<u8>) -> String {
    if board == Some(Ours::BOARD_MCU_LINK_PRO) {
        "Verge Pro CMSIS-DAP".to_owned()
    } else {
        recorded.to_owned()
    }
}

/// Ours' bcdDevice: 0x01 and the functions present, debug in bit 0, VCOM in bit 1 and the Pro's
/// VCOM2 in bit 2 (the firmware's `src/usb_descriptors.c`).
fn ours_bcd_device(u: &SimUnit) -> u16 {
    let debug = if u.swd_jumper { 0 } else { 1 << 0 };
    let vcom2 = if u.board == Some(Ours::BOARD_MCU_LINK_PRO) {
        1 << 2
    } else {
        0
    };
    0x0100 | debug | 1 << 1 | vcom2
}

/// The firmware a unit's flash boots, if any: a vector table that points into programmed flash,
/// and the simulator's marker beside it in page 0.
fn boot_from(flash: &BTreeMap<u32, Vec<u8>>) -> Option<SimFirmware> {
    let page0 = flash.get(&0)?;
    let reset = u32::from_le_bytes(page0[4..8].try_into().ok()?);
    if reset & 1 == 0 || !flash.contains_key(&((reset & !1) / PAGE * PAGE)) {
        return None;
    }
    let rest = page0[MARKER_AT..].strip_prefix(b"SIMFW:")?;
    let end = rest.iter().position(|&b| b == b';')?;
    SimFirmware::from_marker(std::str::from_utf8(&rest[..end]).ok()?)
}

fn programmed(image: &Image) -> BTreeMap<u32, Vec<u8>> {
    let mut flash = BTreeMap::new();
    for run in image.page_runs() {
        for (index, page) in run.data.chunks(PAGE as usize).enumerate() {
            flash.insert(run.address + index as u32 * PAGE, page.to_vec());
        }
    }
    flash
}

/// The simulated bench. Cheap to clone; clones share one bench.
#[derive(Clone)]
pub struct SimBus {
    world: Arc<Mutex<World>>,
}

impl Default for SimBus {
    fn default() -> SimBus {
        SimBus {
            world: Arc::new(Mutex::new(World::new())),
        }
    }
}

/// The names [`SimBus::scenario`] knows, with what each sets up.
pub const SCENARIOS: &[(&str, &str)] = &[
    ("bench", "ours and stock V3.172, as on the bench"),
    ("ours", "one unit running ours"),
    ("stock", "one unit running stock V3.172"),
    (
        "stock-old",
        "one unit running stock V3.108, before the jumper-free update",
    ),
    ("stock-hid", "one unit running stock V2.245, HID only"),
    ("isp", "one unit in ISP mode with the jumper off"),
    (
        "isp-jumper",
        "one unit in ISP mode with the jumper still fitted",
    ),
    ("two-isp", "two units in ISP mode"),
    (
        "ours-beside-isp",
        "one unit running ours, build 14d05d4, and another in ISP mode",
    ),
    (
        "isp-ours",
        "one unit in ISP mode whose flash holds ours, build v0.3.0",
    ),
    (
        "two-isp-ours",
        "two units in ISP mode whose flash holds ours, build v0.3.0",
    ),
    ("mini", "an MCU-Link mini in ISP mode"),
    (
        "isp-mini",
        "one unit in ISP mode and an MCU-Link mini in ISP mode",
    ),
    ("busy", "ours, held open by another program"),
    (
        "arrivals",
        "an empty bench where five units are plugged in over ten seconds, for a batch",
    ),
    ("pro", "one MCU-Link Pro running stock V3.172"),
    (
        "ours-pro",
        "one MCU-Link Pro running a current image of ours",
    ),
    (
        "ours-pro-held",
        "one MCU-Link Pro running a current image of ours, with something holding DIO3 high",
    ),
    (
        "ours-plain",
        "one plain MCU-Link running a current image of ours",
    ),
    (
        "pro-old",
        "one MCU-Link Pro running stock V3.108, which needs the jumper",
    ),
    (
        "fails-writes",
        "one unit running stock V3.172 whose bootloader refuses every write",
    ),
    (
        "unnamed-board",
        "one unit running stock V3.172 whose product string names no board",
    ),
    (
        "swd-jumper",
        "one unit running ours with its SWD jumper fitted",
    ),
    (
        "pro-swd-jumper",
        "one MCU-Link Pro running ours with its SWD jumper fitted",
    ),
    (
        "stock-swd-jumper",
        "one unit running stock V3.172 with its SWD jumper fitted",
    ),
];

impl SimBus {
    pub(crate) fn world(&self) -> MutexGuard<'_, World> {
        self.world
            .lock()
            .expect("no simulated operation panics holding the bench")
    }

    /// Makes simulated time pass in real time too, `pace` real seconds per simulated second.
    #[must_use]
    pub fn paced(self, pace: f64) -> SimBus {
        self.world().pace = Some(pace);
        self
    }

    fn add(&self, mode: Mode, flash: BTreeMap<u32, Vec<u8>>) -> usize {
        let mut world = self.world();
        let index = world.units.len();
        let n = index + 1;
        world.units.push(SimUnit {
            location: format!("1-{n}"),
            ours_serial: if index == 0 {
                "CJ3E7A91C2".into()
            } else {
                format!("CJ{:08X}", 0x3E7A_91C2_u32.wrapping_add(n as u32 * 0x1111))
            },
            stock_serial: if index == 0 {
                "QK7RM2VTX8D41".into()
            } else {
                format!("QK7RM2VTX8D{:02}", 41 + n)
            },
            flash,
            mode: Mode::Silent,
            instance: None,
            generation: 0,
            jumper: false,
            busy: false,
            mini: false,
            stale_replies: Vec::new(),
            plugged: true,
            fail_writes: false,
            ignores_reset: false,
            host_status: 0,
            leds_on: false,
            board: None,
            stock_product: None,
            swd_jumper: false,
            scan_formats: SIM_SCAN_FORMATS,
            dio_outputs: 0,
            dio_levels: 0,
            dio_held: 0,
        });
        world.units[index].mode = mode;
        if world.units[index].mode != Mode::Silent {
            world.enumerate(index);
        }
        index
    }

    /// A unit running `firmware`, its flash holding the matching image.
    pub fn add_running(&self, firmware: SimFirmware) -> usize {
        let flash = programmed(&firmware_image(&firmware));
        self.add(Mode::Firmware(firmware), flash)
    }

    pub fn add_ours(&self, build: &str) -> usize {
        self.add_running(SimFirmware::Ours {
            build: build.to_owned(),
        })
    }

    pub fn add_stock(&self, version: &str) -> usize {
        self.add_running(SimFirmware::Stock {
            version: StockVersion::parse(version).expect("a version like 3.172"),
        })
    }

    /// A unit in ISP mode with erased flash.
    pub fn add_bootloader(&self, jumper: bool) -> usize {
        let index = self.add(Mode::Bootloader, BTreeMap::new());
        self.world().units[index].jumper = jumper;
        index
    }

    pub fn add_mini_bootloader(&self) -> usize {
        let index = self.add(Mode::Silent, BTreeMap::new());
        let mut world = self.world();
        world.units[index].mini = true;
        world.units[index].jumper = true;
        world.units[index].mode = Mode::Bootloader;
        world.enumerate(index);
        index
    }

    /// The serials a unit reports under ours and under stock. Units start with made-up ones, the
    /// first with the recorded pair.
    pub fn set_serials(&self, unit: usize, ours: &str, stock: &str) {
        let mut world = self.world();
        world.units[unit].ours_serial = ours.to_owned();
        world.units[unit].stock_serial = stock.to_owned();
    }

    pub fn set_busy(&self, unit: usize, busy: bool) {
        self.world().units[unit].busy = busy;
    }

    /// A unit running `firmware` that is plugged in `after` from now, on a port of its own.
    pub fn plug_in_later(&self, after: Duration, firmware: SimFirmware) -> usize {
        let unit = self.add_running(firmware);
        self.unplug(unit);
        self.plug_back_later(after, unit);
        unit
    }

    /// A unit with its ISP jumper fitted and erased flash, plugged in `after` from now.
    pub fn plug_in_bootloader_later(&self, after: Duration) -> usize {
        let unit = self.add_bootloader(true);
        self.unplug(unit);
        self.plug_back_later(after, unit);
        unit
    }

    /// Takes a unit out of its port now.
    pub fn unplug(&self, unit: usize) {
        let mut world = self.world();
        world.apply(unit, Event::Unplug);
    }

    pub fn unplug_later(&self, after: Duration, unit: usize) {
        self.world().schedule(after, unit, Event::Unplug);
    }

    /// Puts an unplugged unit back in its port `after` from now. It boots what its flash holds.
    pub fn plug_back_later(&self, after: Duration, unit: usize) {
        self.world().schedule(after, unit, Event::PlugIn);
    }

    /// Puts `firmware` in a unit's flash, as a flash the tool did not make would. It boots that
    /// at its next start.
    pub fn reflash(&self, unit: usize, firmware: &SimFirmware) {
        self.world().units[unit].flash = programmed(&firmware_image(firmware));
    }

    pub fn set_fail_writes(&self, unit: usize, fail: bool) {
        self.world().units[unit].fail_writes = fail;
    }

    /// Makes a unit's bootloader answer the reset command without restarting.
    pub fn set_ignores_reset(&self, unit: usize, ignores: bool) {
        self.world().units[unit].ignores_reset = ignores;
    }

    #[must_use]
    pub fn host_status_count(&self, unit: usize) -> u32 {
        self.world().units[unit].host_status
    }

    /// Whether the last `DAP_HostStatus` a unit answered turned its LEDs on.
    #[must_use]
    pub fn leds_on(&self, unit: usize) -> bool {
        self.world().units[unit].leds_on
    }

    /// A unit in ISP mode whose flash holds `firmware`, which it starts once reset without the
    /// jumper.
    pub fn add_bootloader_holding(&self, firmware: SimFirmware) -> usize {
        self.add(Mode::Bootloader, programmed(&firmware_image(&firmware)))
    }

    /// Makes a unit a board other than the plain MCU-Link, by the board byte ours reports.
    /// Something attached holds these DIO pins high, bit n for DIOn, which they read as inputs.
    pub fn hold_dio(&self, unit: usize, high: u8) {
        self.world.lock().unwrap().units[unit].dio_held = high;
    }

    pub fn set_board(&self, unit: usize, board: u8) {
        self.world().units[unit].board = Some(board);
    }

    /// Gives a unit's stock firmware this USB product string, whatever its board.
    pub fn set_stock_product(&self, unit: usize, product: &str) {
        self.world().units[unit].stock_product = Some(product.to_owned());
    }

    /// Fits or removes a unit's SWD jumper. Applies from its next enumeration.
    pub fn set_swd_jumper(&self, unit: usize, fitted: bool) {
        self.world().units[unit].swd_jumper = fitted;
    }

    /// Makes every device report no port, as on a platform that does not say.
    pub fn set_locations(&self, on: bool) {
        self.world().locations = on;
    }

    /// Makes the bus unable to open one bootloader among several, as on Windows and macOS.
    pub fn set_same_bootloaders(&self, on: bool) {
        self.world().same_bootloaders = on;
    }

    /// Makes a device that comes back keep its instance, as on Windows. Applies from the next
    /// enumeration.
    pub fn set_stable_instances(&self, on: bool) {
        self.world().stable_instances = on;
    }

    /// The most bootloaders that were ever on the bus at once.
    #[must_use]
    pub fn max_bootloaders(&self) -> usize {
        self.world().max_bootloaders
    }

    pub fn set_jumper(&self, unit: usize, jumper: bool) {
        self.world().units[unit].jumper = jumper;
    }

    /// How long after the bootloader's reset command a unit enumerates again. It leaves the bus
    /// 10 ms after the command, so this is more than that.
    pub fn set_boot_after_reset(&self, after: Duration) {
        self.world().boot_after_reset = after;
    }

    /// Leaves replies on a unit's bulk IN endpoint, as a host that stopped early would.
    pub fn leave_stale_replies(&self, unit: usize, replies: Vec<Vec<u8>>) {
        self.world().units[unit].stale_replies = replies;
    }

    /// A preset bench by name; see [`SCENARIOS`].
    #[must_use]
    pub fn scenario(name: &str) -> Option<SimBus> {
        let bus = SimBus::default();
        match name {
            "bench" => {
                bus.add_ours("14d05d4");
                let stock = bus.add_stock("3.172");
                // The recorded stock identity is this unit's; the recorded ours identity unit 0's.
                bus.set_serials(stock, "CJ3E7B02F9", "QK7RM2VTX8D41");
                bus.set_serials(0, "CJ3E7A91C2", "QK7RM2VTX8D40");
            }
            "ours" => {
                bus.add_ours("14d05d4");
            }
            "ours-pro" => {
                let unit = bus.add_ours("26ea505");
                bus.set_board(unit, crate::dap::Ours::BOARD_MCU_LINK_PRO);
            }
            "ours-pro-held" => {
                let unit = bus.add_ours("26ea505");
                bus.set_board(unit, crate::dap::Ours::BOARD_MCU_LINK_PRO);
                bus.hold_dio(unit, 1 << 3);
            }
            "ours-plain" => {
                let unit = bus.add_ours("26ea505");
                bus.set_board(unit, crate::dap::Ours::BOARD_MCU_LINK);
            }
            "stock" => {
                bus.add_stock("3.172");
            }
            "stock-old" => {
                bus.add_stock("3.108");
            }
            "stock-hid" => {
                bus.add_running(SimFirmware::StockHid {
                    version: StockVersion {
                        major: 2,
                        minor: 245,
                    },
                });
            }
            "isp" => {
                bus.add_bootloader(false);
            }
            "isp-jumper" => {
                bus.add_bootloader(true);
            }
            "two-isp" => {
                bus.add_bootloader(false);
                bus.add_bootloader(false);
            }
            "ours-beside-isp" => {
                bus.add_ours("14d05d4");
                bus.add_bootloader(false);
            }
            "isp-ours" | "two-isp-ours" => {
                let ours = || SimFirmware::Ours {
                    build: "v0.3.0".to_owned(),
                };
                bus.add_bootloader_holding(ours());
                if name == "two-isp-ours" {
                    bus.add_bootloader_holding(ours());
                }
            }
            "mini" => {
                bus.add_mini_bootloader();
            }
            "isp-mini" => {
                bus.add_bootloader(false);
                bus.add_mini_bootloader();
            }
            "busy" => {
                let unit = bus.add_ours("14d05d4");
                bus.set_busy(unit, true);
            }
            "arrivals" => {
                let stock = |v: &str| SimFirmware::Stock {
                    version: StockVersion::parse(v).expect("a version like 3.172"),
                };
                let ours = |build: &str| SimFirmware::Ours {
                    build: build.to_owned(),
                };
                bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
                bus.plug_in_later(Duration::from_secs(1), stock("3.172"));
                bus.plug_in_later(Duration::from_secs(3), ours("v0.3.0"));
                bus.plug_in_later(Duration::from_secs(5), stock("3.108"));
                bus.plug_in_later(Duration::from_secs(8), ours("14d05d4"));
            }
            "pro" => {
                let unit = bus.add_stock("3.172");
                bus.set_board(unit, crate::dap::Ours::BOARD_MCU_LINK_PRO);
            }
            "pro-old" => {
                let unit = bus.add_stock("3.108");
                bus.set_board(unit, crate::dap::Ours::BOARD_MCU_LINK_PRO);
            }
            "fails-writes" => {
                let unit = bus.add_stock("3.172");
                bus.set_fail_writes(unit, true);
            }
            "unnamed-board" => {
                let unit = bus.add_stock("3.172");
                bus.set_stock_product(unit, "MCU-LINK CMSIS-DAP V3.172");
            }
            "swd-jumper" | "pro-swd-jumper" | "stock-swd-jumper" => {
                let unit = if name == "stock-swd-jumper" {
                    bus.add_stock("3.172")
                } else {
                    bus.add_ours("14d05d4")
                };
                if name == "pro-swd-jumper" {
                    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
                }
                bus.set_swd_jumper(unit, true);
                bus.world().enumerate(unit);
            }
            _ => return None,
        }
        Some(bus)
    }

    /// Everything the bench did, in order.
    #[must_use]
    pub fn log(&self) -> Vec<String> {
        self.world().log.clone()
    }

    #[must_use]
    pub fn mode(&self, unit: usize) -> Mode {
        self.world().units[unit].mode.clone()
    }

    #[must_use]
    pub fn flash(&self, unit: usize) -> BTreeMap<u32, Vec<u8>> {
        self.world().units[unit].flash.clone()
    }

    #[must_use]
    pub fn serial(&self, unit: usize) -> (String, String) {
        let world = self.world();
        let u = &world.units[unit];
        (u.ours_serial.clone(), u.stock_serial.clone())
    }
}

impl Bus for SimBus {
    type Dap = SimDap;
    type Isp = SimRom;

    fn devices(&self) -> Result<Vec<UsbDevice>, BusError> {
        let mut world = self.world();
        world.settle();
        Ok((0..world.units.len())
            .filter_map(|unit| world.device(unit))
            .collect())
    }

    fn open_dap(&self, device: &UsbDevice) -> Result<SimDap, BusError> {
        let mut world = self.world();
        world.settle();
        let unit = world.unit_for(device).ok_or_else(|| BusError::Gone {
            device: device.to_string(),
        })?;
        let u = &world.units[unit];
        if u.busy {
            return Err(BusError::Busy {
                device: device.to_string(),
            });
        }
        if u.swd_jumper
            || !matches!(
                u.mode,
                Mode::Firmware(SimFirmware::Ours { .. } | SimFirmware::Stock { .. })
            )
        {
            return Err(BusError::Open {
                device: device.to_string(),
                reason: "it has no CMSIS-DAP v2 bulk interface".into(),
            });
        }
        let stale = std::mem::take(&mut world.units[unit].stale_replies);
        let generation = world.units[unit].generation;
        Ok(SimDap::new(self.world.clone(), unit, generation, stale))
    }

    fn device_control_out(
        &self,
        device: &UsbDevice,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<Result<(), LinkError>, BusError> {
        let mut world = self.world();
        world.settle();
        let unit = world.unit_for(device).ok_or_else(|| BusError::Gone {
            device: device.to_string(),
        })?;
        Ok(world.control_out(unit, request_type, request, value, index))
    }

    fn open_isp(&self, device: &UsbDevice) -> Result<SimRom, BusError> {
        let mut world = self.world();
        world.settle();
        let mut unit = world.unit_for(device).ok_or_else(|| BusError::Gone {
            device: device.to_string(),
        })?;
        if world.same_bootloaders {
            unit = (0..world.units.len())
                .find(|&u| {
                    world.units[u].mode == Mode::Bootloader
                        && !world.units[u].mini
                        && world.device(u).is_some()
                })
                .unwrap_or(unit);
        }
        if world.units[unit].mode != Mode::Bootloader {
            return Err(BusError::Open {
                device: device.to_string(),
                reason: "it is not in ISP mode".into(),
            });
        }
        let generation = world.units[unit].generation;
        let instance = world.units[unit]
            .instance
            .clone()
            .unwrap_or_else(|| device.instance.clone());
        Ok(SimRom::new(self.world.clone(), unit, instance, generation))
    }

    fn now(&self) -> Duration {
        self.world().now
    }

    fn sleep(&self, duration: Duration) {
        let real = self.world().advance(duration);
        if !real.is_zero() {
            std::thread::sleep(real);
        }
    }

    fn keeps_instances(&self) -> bool {
        self.world().stable_instances
    }

    fn tells_bootloaders_apart(&self) -> bool {
        !self.world().same_bootloaders
    }
}
