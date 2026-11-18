//! What every attached MCU-Link is running.

use std::fmt;

use crate::aux::Aux;
use crate::board::Board;
use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{self, DapError, Identity, Ours};
use crate::daplink_rs;
use crate::ids::Role;
use crate::version::{Build, StockVersion};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Firmware {
    Ours(Ours),
    Stock {
        version: Option<StockVersion>,
    },
    /// daplink-rs's MCU-Link build. `reboot` is what its reboot command says, `None` where it has
    /// none.
    DaplinkRs {
        reboot: Option<daplink_rs::Reboot>,
    },
}

/// What USB strings alone suggest about a probe the tool could not query.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Guess {
    Ours,
    Stock(Option<StockVersion>),
    DaplinkRs,
    Unknown,
}

impl Guess {
    #[must_use]
    pub fn from_strings(device: &UsbDevice) -> Guess {
        let product = device.product.as_deref().unwrap_or_default();
        let serial = device.serial.as_deref().unwrap_or_default();
        if daplink_rs::is_mculink_build(device) {
            Guess::DaplinkRs
        } else if serial.starts_with("CJ") || product.contains("Verge") {
            Guess::Ours
        } else if product.to_ascii_uppercase().contains("MCU-LINK") {
            Guess::Stock(StockVersion::from_product_string(product))
        } else {
            Guess::Unknown
        }
    }
}

/// Why the tool could not ask a unit what it runs, where the system does not let this user open
/// it ([`BusError::Denied`]).
pub const DENIED: &str = "this user may not open it";

/// Why the tool could not ask a unit what it runs, where another program holds it and Verge Aux
/// did not answer for it.
pub const HELD: &str = "another program has it open";

/// What a unit with its SWD jumper fitted runs, as its USB descriptors say.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Jumpered {
    Ours,
    Stock(Option<StockVersion>),
}

/// The bit of stock's straps, the `rXXX` in its product string, that the SWD jumper clears: J5 on
/// the plain board and J13 on the Pro, both seen on V3.172.
const STOCK_STRAP_SWD: u16 = 0x080;

/// bcdDevice's form in ours: 0x01 in the high byte, the functions present in the low byte.
const OURS_BCD_FORM: u16 = 0x0100;
const OURS_BCD_DEBUG: u16 = 1 << 0;

/// What a unit runs whose USB descriptors say its SWD jumper is fitted, or `None` where they do not
/// say so.
fn swd_jumper(device: &UsbDevice) -> Option<Jumpered> {
    match Guess::from_strings(device) {
        Guess::Ours => (device.bcd_device & 0xFF00 == OURS_BCD_FORM
            && device.bcd_device & OURS_BCD_DEBUG == 0)
            .then_some(Jumpered::Ours),
        Guess::Stock(version) => {
            let product = device.product.as_deref()?;
            // Only the boards where the bit was seen.
            let known = matches!(Hardware::from_stock_product(product), Hardware::Board(_));
            (known && stock_straps(product)? & STOCK_STRAP_SWD == 0)
                .then_some(Jumpered::Stock(version))
        }
        Guess::DaplinkRs | Guess::Unknown => None,
    }
}

/// The straps in stock's product string: `MCU-LINK (r0FB) CMSIS-DAP V3.172` gives 0x0FB.
fn stock_straps(product: &str) -> Option<u16> {
    let (_, rest) = product.split_once("(r")?;
    let (hex, _) = rest.split_once(')')?;
    u16::from_str_radix(hex, 16).ok()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum State {
    /// Running a CMSIS-DAP firmware the tool could query.
    Probe {
        identity: Identity,
        firmware: Firmware,
    },
    /// Running Verge with its CMSIS-DAP interface held by another program, a debugger most likely,
    /// and asked over Verge Aux instead. Nothing that goes through CMSIS-DAP reaches it, an update
    /// included.
    Held { ours: Ours },
    /// Running a firmware the tool could not query.
    Unreadable { reason: String, guess: Guess },
    /// Running a firmware with its SWD jumper fitted. The jumper takes the CMSIS-DAP interfaces
    /// away, so nothing can be asked over them.
    SwdJumper { firmware: Jumpered },
    /// In ISP mode: the LPC55S69 ROM bootloader.
    Bootloader,
    /// An MCU-Link mini in ISP mode.
    MiniBootloader,
}

/// Which board a unit is, as far as what it reports says.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Hardware {
    /// A board this tool knows, which an image of ours can declare.
    Board(Board),
    /// Another board with an LPC55S69 MCU-Link on it, named for a person to read. No image of
    /// ours goes on it.
    Other(String),
    /// Nothing says which: a unit in ISP mode, or a firmware that does not report it.
    Unknown,
}

impl Hardware {
    /// The board in stock's USB product string, which LinkServer's Readme gives as
    /// `MCU-LINK [Pro|MR|on-board|<board>] (r<RRR>) CMSIS-DAP V<V>.<BBB>`. The plain board has
    /// nothing between `MCU-LINK` and `(r`.
    #[must_use]
    pub fn from_stock_product(product: &str) -> Hardware {
        let Some((board, _)) = product
            .strip_prefix("MCU-LINK ")
            .and_then(|rest| rest.split_once("(r"))
        else {
            return Hardware::Unknown;
        };
        match board.trim() {
            "" => Hardware::Board(Board::McuLink),
            "Pro" => Hardware::Board(Board::McuLinkPro),
            "MR" => Hardware::Other("an MCU-Link MR".into()),
            "on-board" => Hardware::Other("an on-board MCU-Link".into()),
            name => Hardware::Other(format!("the MCU-Link on a {name}")),
        }
    }

    /// The board in ours' USB product string (`DAP_ProductName` in the firmware's
    /// `src/DAP_config.h`). Only the Pro has a string of its own: an on-board MCU-Link running
    /// ours gives the full board's, so that string names no board.
    #[must_use]
    pub fn from_ours_product(product: &str) -> Hardware {
        if product.starts_with("Verge Pro ") {
            Hardware::Board(Board::McuLinkPro)
        } else {
            Hardware::Unknown
        }
    }

    /// The board ours reports in its identify reply. Images before format 3 were built only for
    /// the plain board.
    #[must_use]
    pub fn from_ours(ours: &Ours) -> Hardware {
        match ours.board.map(crate::BoardKind::number) {
            None => Hardware::Board(Board::McuLink),
            Some(number) => match Board::from_number(number) {
                Some(board) => Hardware::Board(board),
                None if number == Ours::BOARD_ON_BOARD => {
                    Hardware::Other("an on-board MCU-Link".into())
                }
                None => Hardware::Other(format!("board {number}, which this tool does not know")),
            },
        }
    }
}

impl Hardware {
    /// Its name alone, for a label rather than a sentence: "MCU-Link Pro", not "an MCU-Link Pro".
    #[must_use]
    pub fn name(&self) -> String {
        match self {
            Hardware::Board(board) => board.name().to_owned(),
            Hardware::Other(name) => ["an ", "a ", "the "]
                .iter()
                .find_map(|article| name.strip_prefix(article))
                .unwrap_or(name)
                .to_owned(),
            Hardware::Unknown => "not reported".to_owned(),
        }
    }
}

impl fmt::Display for Hardware {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Hardware::Board(board) => f.write_str(board.a()),
            Hardware::Other(name) => f.write_str(name),
            Hardware::Unknown => f.write_str("a board it does not name"),
        }
    }
}

/// One attached MCU-Link.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unit {
    pub device: UsbDevice,
    pub state: State,
}

impl Unit {
    /// The serial a user would pick this unit by: what `DAP_Info` says, else the USB string.
    #[must_use]
    pub fn serial(&self) -> Option<&str> {
        match &self.state {
            State::Probe { identity, .. } => identity.serial.as_deref(),
            _ => None,
        }
        .or(self.device.serial.as_deref())
    }

    /// Whether the unit runs Verge as far as the tool can tell, so takes a restart request: it
    /// said so, its SWD jumper's descriptors say so, or a unit another program holds looks like
    /// Verge by its USB strings. One this user may not open cannot be sent the request either.
    #[must_use]
    pub fn takes_restart(&self) -> bool {
        self.verge().is_some()
            || matches!(
                self.state,
                State::SwdJumper {
                    firmware: Jumpered::Ours
                }
            )
            || matches!(
                &self.state,
                State::Unreadable { reason, guess: Guess::Ours } if reason != DENIED
            )
    }

    /// What Verge says it is, asked over CMSIS-DAP or Verge Aux.
    #[must_use]
    pub fn verge(&self) -> Option<&Ours> {
        match &self.state {
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            }
            | State::Held { ours } => Some(ours),
            _ => None,
        }
    }

    /// The build ours runs, parsed.
    #[must_use]
    pub fn build(&self) -> Option<Build> {
        self.verge().map(|ours| Build::parse(&ours.firmware))
    }

    #[must_use]
    pub fn is_bootloader(&self) -> bool {
        self.state == State::Bootloader
    }

    /// Which board this is. Every LPC55S69 bootloader looks alike, so a unit in ISP mode is
    /// [`Hardware::Unknown`].
    #[must_use]
    pub fn hardware(&self) -> Hardware {
        match &self.state {
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            }
            | State::Held { ours } => Hardware::from_ours(ours),
            State::SwdJumper {
                firmware: Jumpered::Ours,
            } => self
                .device
                .product
                .as_deref()
                .map_or(Hardware::Unknown, Hardware::from_ours_product),
            State::Probe {
                firmware: Firmware::DaplinkRs { .. },
                ..
            } => Hardware::Unknown,
            State::Probe { .. } | State::Unreadable { .. } | State::SwdJumper { .. } => self
                .device
                .product
                .as_deref()
                .map_or(Hardware::Unknown, Hardware::from_stock_product),
            State::Bootloader => Hardware::Unknown,
            State::MiniBootloader => Hardware::Other("an MCU-Link mini".into()),
        }
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            } if ours.firmware.is_empty() => write!(f, "Verge, identify format {}", ours.version),
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            } => write!(f, "Verge, build {}", ours.firmware),
            State::Probe {
                firmware: Firmware::Stock { version: Some(v) },
                ..
            } => write!(f, "NXP's firmware V{v}"),
            State::Probe {
                firmware: Firmware::Stock { version: None },
                ..
            } => write!(f, "NXP's firmware, version not reported"),
            State::Probe {
                firmware: Firmware::DaplinkRs { .. },
                identity,
            } => match identity.protocol.as_deref() {
                // Its DAP_Info protocol version carries its revision: 2.1.1.g507437947680.
                Some(protocol) if !protocol.is_empty() => write!(f, "daplink-rs {protocol}"),
                _ => write!(f, "daplink-rs"),
            },
            State::Held { ours } => write!(
                f,
                "Verge, build {}, its CMSIS-DAP held by another program",
                ours.firmware
            ),
            State::Unreadable { guess, .. } => match guess {
                Guess::Ours => write!(f, "probably Verge, could not ask"),
                Guess::Stock(Some(v)) => write!(f, "probably NXP's firmware V{v}, could not ask"),
                Guess::Stock(None) => write!(f, "probably NXP's firmware, could not ask"),
                Guess::DaplinkRs => write!(f, "probably daplink-rs, could not ask"),
                Guess::Unknown => write!(f, "a CMSIS-DAP firmware, could not ask"),
            },
            State::SwdJumper { firmware } => match firmware {
                Jumpered::Ours => write!(f, "Verge, SWD jumper fitted"),
                Jumpered::Stock(Some(v)) => write!(f, "NXP's firmware V{v}, SWD jumper fitted"),
                Jumpered::Stock(None) => write!(f, "NXP's firmware, SWD jumper fitted"),
            },
            State::Bootloader => write!(f, "in ISP mode, waiting for an image"),
            State::MiniBootloader => write!(f, "an MCU-Link mini in ISP mode"),
        }
    }
}

/// Every attached MCU-Link and what it runs.
///
/// A probe is opened, asked `DAP_Info` and vendor command 0x82, and released; daplink-rs is asked
/// its own discovery instead of 0x82. All are read-only. A Pro whose CMSIS-DAP another program
/// holds is asked 0x82 over Verge Aux. A bootloader is not opened, nor a probe without a CMSIS-DAP
/// v2 interface.
pub fn detect<B: Bus>(bus: &B) -> Result<Vec<Unit>, BusError> {
    Ok(bus
        .devices()?
        .into_iter()
        .filter_map(|device| {
            let role = device.role()?;
            Some(query(bus, device, role))
        })
        .collect())
}

/// [`detect`] for a command about the unit `serial` names. A probe whose USB serial is another is not
/// opened, so a unit another program is using between its own commands is left alone; bootloaders
/// are listed as ever, since they are never opened. When no probe's USB serial is `serial`, every
/// probe is asked, because the serial a user names is what the unit's `DAP_Info` reports.
pub fn detect_named<B: Bus>(bus: &B, serial: Option<&str>) -> Result<Vec<Unit>, BusError> {
    let Some(serial) = serial else {
        return detect(bus);
    };
    let devices = bus.devices()?;
    let named = |device: &UsbDevice| device.serial.as_deref() == Some(serial);
    let any_named = devices
        .iter()
        .any(|device| device.role().is_some_and(Role::is_firmware) && named(device));
    Ok(devices
        .into_iter()
        .filter_map(|device| {
            let role = device.role()?;
            if any_named && role.is_firmware() && !named(&device) {
                return None;
            }
            Some(query(bus, device, role))
        })
        .collect())
}

/// Why [`pick`] found no one unit.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum PickError {
    /// No unit is attached.
    NoneAttached,
    /// No attached unit has the serial named.
    NoSuchSerial(String),
    /// None of the attached units fits: all of them. A lone unit here is one a command can say it
    /// cannot act on.
    NoneFits(Vec<Unit>),
    /// Several units fit, or several have the serial named.
    Several(Vec<Unit>),
}

impl std::fmt::Display for PickError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PickError::NoneAttached => f.write_str("no MCU-Link is attached"),
            PickError::NoSuchSerial(serial) => {
                write!(f, "no attached MCU-Link has serial {serial}")
            }
            PickError::NoneFits(_) => f.write_str("no attached MCU-Link fits"),
            PickError::Several(units) => {
                write!(
                    f,
                    "{} attached MCU-Links fit: pick one by serial",
                    units.len()
                )
            }
        }
    }
}

impl std::error::Error for PickError {}

/// The unit a command acts on: the one `serial` names, or without a serial, the only one of
/// `units` that `fits`.
pub fn pick<'a>(
    units: &'a [Unit],
    serial: Option<&str>,
    fits: impl Fn(&Unit) -> bool,
) -> Result<&'a Unit, PickError> {
    let found: Vec<&Unit> = match serial {
        Some(serial) => units
            .iter()
            .filter(|unit| unit.serial() == Some(serial))
            .collect(),
        None => units.iter().filter(|unit| fits(unit)).collect(),
    };
    match (found.as_slice(), serial) {
        ([unit], _) => Ok(unit),
        ([_, _, ..], _) => Err(PickError::Several(found.into_iter().cloned().collect())),
        ([], Some(serial)) => Err(PickError::NoSuchSerial(serial.to_owned())),
        ([], None) if units.is_empty() => Err(PickError::NoneAttached),
        ([], None) => Err(PickError::NoneFits(units.to_vec())),
    }
}

/// What one device runs, asked as [`detect`] asks every device. `None` for a device this tool
/// ignores.
pub fn query_device<B: Bus>(bus: &B, device: UsbDevice) -> Option<Unit> {
    let role = device.role()?;
    Some(query(bus, device, role))
}

fn query<B: Bus>(bus: &B, device: UsbDevice, role: Role) -> Unit {
    let state = match role {
        Role::Bootloader => State::Bootloader,
        Role::MiniBootloader => State::MiniBootloader,
        Role::Probe | Role::DaplinkRs if !device.dap_v2 => match swd_jumper(&device) {
            Some(firmware) => State::SwdJumper { firmware },
            None => State::Unreadable {
                reason: "it has no CMSIS-DAP v2 interface (NXP's firmware V2.x speaks HID only)"
                    .into(),
                guess: Guess::from_strings(&device),
            },
        },
        Role::Probe | Role::DaplinkRs => match bus.open_dap(&device) {
            Ok(mut link) => match ask(&mut link, &device, role) {
                Ok(state) => state,
                Err(error) => State::Unreadable {
                    reason: error.to_string(),
                    guess: Guess::from_strings(&device),
                },
            },
            Err(BusError::Busy { .. }) => match (device.aux && role == Role::Probe)
                .then(|| ask_aux(bus, &device))
            {
                Some(Ok(ours)) => State::Held { ours },
                held => {
                    if let Some(Err(error)) = held {
                        tracing::debug!("{device}: Verge Aux did not say what it runs: {error}");
                    }
                    State::Unreadable {
                        reason: HELD.into(),
                        guess: Guess::from_strings(&device),
                    }
                }
            },
            Err(BusError::Denied { .. }) => State::Unreadable {
                reason: DENIED.into(),
                guess: Guess::from_strings(&device),
            },
            Err(error) => State::Unreadable {
                reason: error.to_string(),
                guess: Guess::from_strings(&device),
            },
        },
    };
    Unit { device, state }
}

/// Vendor command 0x82 over Verge Aux, which only Verge offers.
fn ask_aux<B: Bus>(bus: &B, device: &UsbDevice) -> Result<Ours, String> {
    let link = bus.open_aux(device).map_err(|error| error.to_string())?;
    let mut aux = Aux::open(link).map_err(|error| error.to_string())?;
    let reply = aux
        .exchange(&[dap::VENDOR_IDENTIFY])
        .map_err(|error| error.to_string())?;
    dap::parse_identify(&reply)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "it answered 0x82 as NXP's firmware would".to_owned())
}

/// `DAP_Info`, then what the firmware is: vendor command 0x82 for ours or stock, and only
/// daplink-rs's own discovery for daplink-rs, which gives 0x82 no meaning of ours.
fn ask(link: &mut dyn dap::DapLink, device: &UsbDevice, role: Role) -> Result<State, DapError> {
    let drained = link.drain()?;
    if drained > 0 {
        tracing::debug!("{device}: drained {drained} replies an earlier host left unread");
    }
    let identity = dap::read_identity(link)?;
    if role == Role::DaplinkRs {
        let reboot = daplink_rs::discover(link)?;
        return Ok(State::Probe {
            identity,
            firmware: Firmware::DaplinkRs { reboot },
        });
    }
    let firmware = match dap::identify(link)? {
        Some(ours) => Firmware::Ours(ours),
        None => Firmware::Stock {
            version: identity
                .firmware_version
                .as_deref()
                .and_then(StockVersion::parse)
                .or_else(|| {
                    device
                        .product
                        .as_deref()
                        .and_then(StockVersion::from_product_string)
                }),
        },
    };
    Ok(State::Probe { identity, firmware })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device(product: &str, bcd_device: u16) -> UsbDevice {
        UsbDevice {
            vid: 0x1fc9,
            pid: 0x0143,
            location: None,
            instance: "1".into(),
            serial: None,
            manufacturer: None,
            product: Some(product.into()),
            bcd_device,
            dap_v2: false,
            hid: false,
            aux: false,
        }
    }

    #[test]
    fn stock_says_its_swd_jumper_in_its_straps() {
        let v3172 = Some(StockVersion {
            major: 3,
            minor: 172,
        });
        // The product strings stock V3.172 gave with each jumper (mculink-stock-notes,
        // jumpers-2026-10-20).
        for (straps, fitted) in [
            ("MCU-LINK (r0FB)", false),
            ("MCU-LINK (r0BB)", false),
            ("MCU-LINK (r07B)", true),
            ("MCU-LINK Pro (r1CF)", false),
            ("MCU-LINK Pro (r18F)", false),
            ("MCU-LINK Pro (r14F)", true),
            ("MCU-LINK Pro (r1CB)", false),
        ] {
            let product = format!("{straps} CMSIS-DAP V3.172");
            assert_eq!(
                swd_jumper(&device(&product, 0x0337)),
                fitted.then_some(Jumpered::Stock(v3172)),
                "{product}"
            );
        }
    }

    #[test]
    fn stock_on_another_board_is_not_read_for_the_jumper() {
        let product = "MCU-LINK FRDM-MCXN947 (r067) CMSIS-DAP V3.172";
        assert_eq!(swd_jumper(&device(product, 0x0337)), None);
    }

    #[test]
    fn ours_says_its_swd_jumper_in_bcd_device() {
        for (bcd, fitted) in [
            (0x0102, true),
            (0x0100, true),
            (0x0103, false),
            (0x0002, false),
        ] {
            assert_eq!(
                swd_jumper(&device("Verge CMSIS-DAP", bcd)),
                fitted.then_some(Jumpered::Ours),
                "0x{bcd:04x}"
            );
        }
    }

    #[test]
    fn ours_names_only_the_pro_in_its_product_string() {
        assert_eq!(
            Hardware::from_ours_product("Verge CMSIS-DAP"),
            Hardware::Unknown
        );
        assert_eq!(
            Hardware::from_ours_product("Verge Pro CMSIS-DAP"),
            Hardware::Board(Board::McuLinkPro)
        );
        assert_eq!(
            Hardware::from_ours_product("MCU-LINK (r0FB) CMSIS-DAP V3.172"),
            Hardware::Unknown
        );
    }
}
