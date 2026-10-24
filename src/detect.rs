//! What every attached MCU-Link is running.

use std::fmt;

use crate::board::Board;
use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{self, DapError, Identity, Ours};
use crate::ids::Role;
use crate::version::{Build, StockVersion};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Firmware {
    Ours(Ours),
    Stock { version: Option<StockVersion> },
}

/// What USB strings alone suggest about a probe the tool could not query.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Guess {
    Ours,
    Stock(Option<StockVersion>),
    Unknown,
}

impl Guess {
    #[must_use]
    pub fn from_strings(device: &UsbDevice) -> Guess {
        let product = device.product.as_deref().unwrap_or_default();
        let serial = device.serial.as_deref().unwrap_or_default();
        if serial.starts_with("CJ") || product.contains("Verge") {
            Guess::Ours
        } else if product.to_ascii_uppercase().contains("MCU-LINK") {
            Guess::Stock(StockVersion::from_product_string(product))
        } else {
            Guess::Unknown
        }
    }
}

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
        Guess::Unknown => None,
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
        match ours.board {
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

    /// The build ours runs, parsed.
    #[must_use]
    pub fn build(&self) -> Option<Build> {
        match &self.state {
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            } => Some(Build::parse(&ours.build)),
            _ => None,
        }
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
            } => Hardware::from_ours(ours),
            State::SwdJumper {
                firmware: Jumpered::Ours,
            } => self
                .device
                .product
                .as_deref()
                .map_or(Hardware::Unknown, Hardware::from_ours_product),
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
            } if ours.build.is_empty() => write!(f, "ours, identify format {}", ours.format),
            State::Probe {
                firmware: Firmware::Ours(ours),
                ..
            } => write!(f, "ours, build {}", ours.build),
            State::Probe {
                firmware: Firmware::Stock { version: Some(v) },
                ..
            } => write!(f, "stock V{v}"),
            State::Probe {
                firmware: Firmware::Stock { version: None },
                ..
            } => write!(f, "stock, version not reported"),
            State::Unreadable { guess, .. } => match guess {
                Guess::Ours => write!(f, "probably ours, could not ask"),
                Guess::Stock(Some(v)) => write!(f, "probably stock V{v}, could not ask"),
                Guess::Stock(None) => write!(f, "probably stock, could not ask"),
                Guess::Unknown => write!(f, "a CMSIS-DAP firmware, could not ask"),
            },
            State::SwdJumper { firmware } => match firmware {
                Jumpered::Ours => write!(f, "ours, SWD jumper fitted"),
                Jumpered::Stock(Some(v)) => write!(f, "stock V{v}, SWD jumper fitted"),
                Jumpered::Stock(None) => write!(f, "stock, SWD jumper fitted"),
            },
            State::Bootloader => write!(f, "in ISP mode, waiting for an image"),
            State::MiniBootloader => write!(f, "an MCU-Link mini in ISP mode"),
        }
    }
}

/// Every attached MCU-Link and what it runs.
///
/// A probe is opened, asked `DAP_Info` and vendor command 0x82, and released. Both are read-only.
/// A bootloader is not opened, nor a probe without a CMSIS-DAP v2 interface.
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
        .any(|device| device.role() == Some(Role::Probe) && named(device));
    Ok(devices
        .into_iter()
        .filter_map(|device| {
            let role = device.role()?;
            if any_named && role == Role::Probe && !named(&device) {
                return None;
            }
            Some(query(bus, device, role))
        })
        .collect())
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
        Role::Probe if !device.dap_v2 => match swd_jumper(&device) {
            Some(firmware) => State::SwdJumper { firmware },
            None => State::Unreadable {
                reason: "it has no CMSIS-DAP v2 interface (stock V2.x speaks HID only)".into(),
                guess: Guess::from_strings(&device),
            },
        },
        Role::Probe => match bus.open_dap(&device) {
            Ok(mut link) => match ask(&mut link, &device) {
                Ok(state) => state,
                Err(error) => State::Unreadable {
                    reason: error.to_string(),
                    guess: Guess::from_strings(&device),
                },
            },
            Err(BusError::Busy { .. }) => State::Unreadable {
                reason: "another program has it open".into(),
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

fn ask(link: &mut dyn dap::DapLink, device: &UsbDevice) -> Result<State, DapError> {
    let drained = link.drain()?;
    if drained > 0 {
        tracing::debug!("{device}: drained {drained} replies an earlier host left unread");
    }
    let identity = dap::read_identity(link)?;
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
