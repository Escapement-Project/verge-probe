//! The CMSIS-DAP commands this tool sends, over any [`DapLink`].
//!
//! One command per packet; a reply's first byte echoes the command id, except that a firmware
//! answers a command it does not know with `ID_DAP_Invalid` (`0xFF`).

use crate::version::StockVersion;

pub const DAP_INFO: u8 = 0x00;
pub const DAP_HOST_STATUS: u8 = 0x01;
pub const DAP_DISCONNECT: u8 = 0x03;
pub const ID_DAP_INVALID: u8 = 0xFF;
pub const DAP_OK: u8 = 0x00;

/// Ours answers this with its identity; stock does not know it.
pub const VENDOR_IDENTIFY: u8 = 0x82;
/// Ours' two-pin (cJTAG) control. Mode 4 sets the scan formats the unit offers a host until it
/// restarts: a little-endian mask after the mode, bit n for SCNFMT n.
pub const VENDOR_CJTAG_CONTROL: u8 = 0x83;
pub const CJTAG_SET_FORMATS: u8 = 4;
/// The scan formats by their IEEE 1149.7 SCNFMT numbers.
pub const SCNFMT_OSCAN1: u32 = 9;
pub const SCNFMT_OSCAN2: u32 = 10;
pub const SCNFMT_OSCAN5: u32 = 13;
pub const IDENTIFY_MAGIC: &[u8; 5] = b"CJTAG";
/// Ours' DIO pins on the MCU-Link Pro's J19: `mask, outputs, levels` sets the direction and the
/// level of each pin in `mask`, bit n for DIOn; a mask of 0 only reads. Always three bytes after
/// the id, so it can sit inside `DAP_ExecuteCommands`.
pub const VENDOR_DIO: u8 = 0x8E;

/// Stock's jumper-free way into the bootloader, as LinkServer's `probe update` sends it.
pub const STOCK_BOOTLOADER_REQUEST: [u8; 2] = [0x81, 0x01];
pub const STOCK_BOOTLOADER_ACCEPTED: [u8; 3] = [0x81, 0x00, 0x01];

/// Ours reboots on a vendor request on endpoint 0, answered by CPU 0 even while a DAP command
/// runs. `wValue` 1 asks for the bootloader, 0 for a plain reboot (the firmware's
/// `src/reboot_usb.h`).
pub const REBOOT_REQUEST_TYPE: u8 = 0x40;
pub const REBOOT_REQUEST: u8 = 0x03;
pub const REBOOT_TO_BOOTLOADER: u16 = 1;

/// `DAP_Info` fields.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Info {
    Vendor = 0x01,
    Product = 0x02,
    Serial = 0x03,
    Protocol = 0x04,
    FirmwareVersion = 0x09,
    PacketCount = 0xFE,
    PacketSize = 0xFF,
}

/// A transport failure, below the level of any one command.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum LinkError {
    #[error("the probe did not answer in time")]
    Timeout,
    #[error("the probe left the bus")]
    Disconnected,
    #[error("USB error: {0}")]
    Usb(String),
}

/// A probe's CMSIS-DAP v2 bulk pair, plus endpoint 0 for vendor requests.
pub trait DapLink {
    /// Sends one command and reads its reply.
    fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError>;

    /// Reads and discards replies an earlier host left queued, returning how many.
    ///
    /// Nothing in CMSIS-DAP resynchronises a bulk endpoint: an unread reply is handed to whatever
    /// asks next, and every answer after it is off by one.
    fn drain(&mut self) -> Result<usize, LinkError>;

    /// A vendor request on endpoint 0 with no data stage.
    fn control_out(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<(), LinkError>;
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DapError {
    #[error(transparent)]
    Link(#[from] LinkError),
    #[error("asked for command 0x{asked:02x} and the reply was {got}: the probe is out of step")]
    OutOfStep { asked: u8, got: String },
    #[error("the reply to command 0x{command:02x} was {len} bytes, too short to parse")]
    Short { command: u8, len: usize },
    #[error("the probe refused command 0x{command:02x}")]
    Refused { command: u8 },
}

/// Sends a command and checks that the reply echoes it.
pub fn command(link: &mut dyn DapLink, request: &[u8]) -> Result<Vec<u8>, DapError> {
    let reply = link.exchange(request)?;
    match reply.first() {
        Some(&id) if id == request[0] => Ok(reply),
        other => Err(DapError::OutOfStep {
            asked: request[0],
            got: other.map_or_else(|| "empty".to_owned(), |id| format!("0x{id:02x}")),
        }),
    }
}

/// The value bytes of a `DAP_Info` field.
pub fn info(link: &mut dyn DapLink, field: Info) -> Result<Vec<u8>, DapError> {
    let reply = command(link, &[DAP_INFO, field as u8])?;
    let len = usize::from(*reply.get(1).ok_or(DapError::Short {
        command: DAP_INFO,
        len: reply.len(),
    })?);
    reply
        .get(2..2 + len)
        .map(<[u8]>::to_vec)
        .ok_or(DapError::Short {
            command: DAP_INFO,
            len: reply.len(),
        })
}

/// A string field, without its terminating NUL. An empty field is `None`.
pub fn info_string(link: &mut dyn DapLink, field: Info) -> Result<Option<String>, DapError> {
    let bytes = info(link, field)?;
    let text = bytes.split(|&b| b == 0).next().unwrap_or_default();
    Ok((!text.is_empty()).then(|| String::from_utf8_lossy(text).into_owned()))
}

/// What `DAP_Info` says about a probe.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Identity {
    pub vendor: Option<String>,
    pub product: Option<String>,
    pub serial: Option<String>,
    pub protocol: Option<String>,
    pub firmware_version: Option<String>,
    pub packet_count: Option<u8>,
    pub packet_size: Option<u16>,
}

pub fn read_identity(link: &mut dyn DapLink) -> Result<Identity, DapError> {
    let packet_count = info(link, Info::PacketCount)?.first().copied();
    let packet_size = info(link, Info::PacketSize)?
        .get(..2)
        .map(|b| u16::from_le_bytes([b[0], b[1]]));
    Ok(Identity {
        vendor: info_string(link, Info::Vendor)?,
        product: info_string(link, Info::Product)?,
        serial: info_string(link, Info::Serial)?,
        protocol: info_string(link, Info::Protocol)?,
        firmware_version: info_string(link, Info::FirmwareVersion)?,
        packet_count,
        packet_size,
    })
}

/// Ours, as its identify reply describes it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ours {
    /// The reply's format version: 2 carries flags and the build string, 3 the board as well.
    pub format: u8,
    pub flags: u8,
    /// `git describe` of the image; empty before format 2.
    pub build: String,
    /// The board byte, from format 3 on: [`Ours::BOARD_MCU_LINK`] or another board.
    pub board: Option<u8>,
    /// The two-pin scan formats the unit offers, from format 4 on: bit n for SCNFMT n.
    pub scan_formats: Option<u32>,
}

impl Ours {
    pub const FLAG_OSCAN1: u8 = 1 << 0;
    pub const FLAG_OSCAN1_ACTIVE: u8 = 1 << 1;
    pub const FLAG_TARGET_SENSE: u8 = 1 << 2;
    pub const FLAG_SWD_CODEGEN: u8 = 1 << 3;
    pub const FLAG_OSCAN1_ENTER: u8 = 1 << 4;
    pub const FLAG_JTAG_CODEGEN: u8 = 1 << 5;
    /// [`VENDOR_DIO`] acts: a Pro whose USB-SIO jumper, J15, was not fitted when it started.
    pub const FLAG_DIO: u8 = 1 << 6;

    /// The board byte's values (`BOARD_KIND_*` in the firmware's `src/board_straps.h`).
    pub const BOARD_MCU_LINK: u8 = 0;
    pub const BOARD_MCU_LINK_PRO: u8 = 1;
    pub const BOARD_ON_BOARD: u8 = 2;

    /// The capability flags by name, for display.
    #[must_use]
    pub fn flag_names(&self) -> Vec<&'static str> {
        [
            (Self::FLAG_OSCAN1, "OScan1"),
            (Self::FLAG_OSCAN1_ACTIVE, "OScan1 active"),
            (Self::FLAG_TARGET_SENSE, "target sense"),
            (Self::FLAG_SWD_CODEGEN, "generated SWD"),
            (Self::FLAG_OSCAN1_ENTER, "host-driven OScan1 entry"),
            (Self::FLAG_JTAG_CODEGEN, "generated JTAG"),
            (Self::FLAG_DIO, "DIO pins"),
        ]
        .into_iter()
        .filter(|(bit, _)| self.flags & bit != 0)
        .map(|(_, name)| name)
        .collect()
    }

    /// The two-pin scan formats offered, by name, or `None` before format 4.
    #[must_use]
    pub fn scan_format_names(&self) -> Option<Vec<String>> {
        self.scan_formats.map(|mask| {
            (0..32)
                .filter(|n| mask >> n & 1 != 0)
                .map(scan_format_name)
                .collect()
        })
    }
}

/// A scan format's name: OScan1 for SCNFMT 9, and so on.
#[must_use]
pub fn scan_format_name(scnfmt: u32) -> String {
    match scnfmt {
        0..=3 => format!("JScan{scnfmt}"),
        4..=7 => format!("SScan{}", scnfmt - 4),
        8..=15 => format!("OScan{}", scnfmt - 8),
        16 => "MScan".to_owned(),
        _ => format!("SCNFMT {scnfmt}"),
    }
}

/// Reads a reply to [`VENDOR_IDENTIFY`]: `Some` for ours, `None` for anything else.
///
/// Stock answers `ID_DAP_Invalid` or a bare echo; any reply without the magic counts as stock.
pub fn parse_identify(reply: &[u8]) -> Result<Option<Ours>, DapError> {
    match reply.first() {
        Some(&ID_DAP_INVALID) => return Ok(None),
        Some(&VENDOR_IDENTIFY) => {}
        other => {
            return Err(DapError::OutOfStep {
                asked: VENDOR_IDENTIFY,
                got: other.map_or_else(|| "empty".to_owned(), |id| format!("0x{id:02x}")),
            });
        }
    }
    let body = &reply[1..];
    if body.get(..IDENTIFY_MAGIC.len()) != Some(IDENTIFY_MAGIC.as_slice()) {
        return Ok(None);
    }
    let short = || DapError::Short {
        command: VENDOR_IDENTIFY,
        len: reply.len(),
    };
    let format = *body.get(5).ok_or_else(short)?;
    if format < 2 {
        return Ok(Some(Ours {
            format,
            flags: 0,
            build: String::new(),
            board: None,
            scan_formats: None,
        }));
    }
    let flags = *body.get(6).ok_or_else(short)?;
    let len = usize::from(*body.get(7).ok_or_else(short)?);
    let build = body.get(8..8 + len).ok_or_else(short)?;
    // A reply read over v1 HID is padded to the report, so a format 2 image has a byte here too.
    let board = if format >= 3 {
        Some(*body.get(8 + len).ok_or_else(short)?)
    } else {
        None
    };
    let scan_formats = if format >= 4 {
        let bytes = body.get(9 + len..13 + len).ok_or_else(short)?;
        Some(u32::from_le_bytes(bytes.try_into().expect("four bytes")))
    } else {
        None
    };
    Ok(Some(Ours {
        format,
        flags,
        build: String::from_utf8_lossy(build).into_owned(),
        board,
        scan_formats,
    }))
}

/// Limits the two-pin scan formats ours offers a host to `mask` (bit n for SCNFMT n) until the
/// unit restarts. The unit keeps OScan1 whatever the mask, and drops formats its image does not
/// frame; `u32::MAX` offers every one it does.
pub fn set_scan_formats(link: &mut dyn DapLink, mask: u32) -> Result<(), DapError> {
    let mut request = vec![VENDOR_CJTAG_CONTROL, CJTAG_SET_FORMATS];
    request.extend_from_slice(&mask.to_le_bytes());
    let reply = command(link, &request)?;
    match reply.get(1) {
        Some(&DAP_OK) => Ok(()),
        Some(_) => Err(DapError::Refused {
            command: VENDOR_CJTAG_CONTROL,
        }),
        None => Err(DapError::Short {
            command: VENDOR_CJTAG_CONTROL,
            len: reply.len(),
        }),
    }
}

/// The DIO pins as a [`VENDOR_DIO`] reply describes them, bit n for DIOn throughout.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Dio {
    /// Whether the unit made the change asked for. A refused one changes nothing.
    pub accepted: bool,
    /// The pins there are: none on a board without them, or on a Pro started with J15 fitted.
    pub pins: u8,
    /// The pins the unit drives.
    pub outputs: u8,
    /// Each pin's output level: what it drives, or will once it is an output.
    pub levels: u8,
    /// The levels on the unit's side of the pins' translators, so an output reads its own drive.
    pub read: u8,
    /// The pins that change direction together.
    pub groups: Vec<u8>,
}

impl Dio {
    /// The group that setting the pins in `mask` to the directions in `outputs` would leave
    /// partly driven, which the unit refuses.
    #[must_use]
    pub fn split(&self, mask: u8, outputs: u8) -> Option<u8> {
        let next = (self.outputs & !mask) | (outputs & mask);
        self.groups
            .iter()
            .copied()
            .find(|&group| next & group != 0 && next & group != group)
    }
}

/// Reads a reply to [`VENDOR_DIO`]: `None` from an image of ours that predates the command.
pub fn parse_dio(reply: &[u8]) -> Result<Option<Dio>, DapError> {
    match reply.first() {
        Some(&ID_DAP_INVALID) => return Ok(None),
        Some(&VENDOR_DIO) => {}
        other => {
            return Err(DapError::OutOfStep {
                asked: VENDOR_DIO,
                got: other.map_or_else(|| "empty".to_owned(), |id| format!("0x{id:02x}")),
            });
        }
    }
    let short = || DapError::Short {
        command: VENDOR_DIO,
        len: reply.len(),
    };
    let fixed = reply.get(1..7).ok_or_else(short)?;
    let groups = reply.get(7..7 + usize::from(fixed[5])).ok_or_else(short)?;
    Ok(Some(Dio {
        accepted: fixed[0] == DAP_OK,
        outputs: fixed[1],
        levels: fixed[2],
        read: fixed[3],
        pins: fixed[4],
        groups: groups.to_vec(),
    }))
}

/// Sets the directions (1 an output) and the levels of the DIO pins in `mask`, and returns the
/// pins as they are afterwards; a mask of 0 only reads. `None` from an image that predates them.
pub fn dio(
    link: &mut dyn DapLink,
    mask: u8,
    outputs: u8,
    levels: u8,
) -> Result<Option<Dio>, DapError> {
    parse_dio(&link.exchange(&[VENDOR_DIO, mask, outputs & mask, levels & mask])?)
}

pub fn identify(link: &mut dyn DapLink) -> Result<Option<Ours>, DapError> {
    parse_identify(&link.exchange(&[VENDOR_IDENTIFY])?)
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum EntryError {
    #[error("stock V{0} predates the jumper-free update (V3.122 and later); use the ISP jumper")]
    StockTooOld(StockVersion),
    #[error(
        "stock did not say which version it is, so the tool will not send it the update request"
    )]
    StockVersionUnknown,
    #[error("stock answered the update request with {0}, not 81 00 01; use the ISP jumper")]
    StockRefused(String),
    #[error(transparent)]
    Dap(#[from] DapError),
}

/// Asks stock to enter its bootloader: `81 01`, which must answer `81 00 01`, then
/// `DAP_Disconnect`. The unit leaves the bus about 2.1 s later.
///
/// Returns the reply to `DAP_Disconnect` when it was not `03 00`, so the caller can say so; by
/// then the request has been accepted and the unit may still leave the bus.
pub fn request_stock_bootloader(
    link: &mut dyn DapLink,
    version: Option<StockVersion>,
) -> Result<Option<String>, EntryError> {
    match version {
        None => return Err(EntryError::StockVersionUnknown),
        Some(version) if !version.has_jumper_free_update() => {
            return Err(EntryError::StockTooOld(version));
        }
        Some(_) => {}
    }
    let reply = link
        .exchange(&STOCK_BOOTLOADER_REQUEST)
        .map_err(DapError::from)?;
    if reply.get(..3) != Some(STOCK_BOOTLOADER_ACCEPTED.as_slice()) {
        return Err(EntryError::StockRefused(hex::encode(
            &reply[..reply.len().min(8)],
        )));
    }
    let disconnect = link.exchange(&[DAP_DISCONNECT]);
    Ok(match disconnect {
        Ok(reply) if reply.get(..2) == Some([DAP_DISCONNECT, DAP_OK].as_slice()) => None,
        Ok(reply) => Some(hex::encode(&reply[..reply.len().min(8)])),
        Err(error) => Some(error.to_string()),
    })
}

/// Sets both LEDs `DAP_HostStatus` drives, the connect and the running one. Every CMSIS-DAP
/// firmware answers it, stock included, so toggling it shows a person which unit is which.
pub fn set_leds(link: &mut dyn DapLink, on: bool) -> Result<(), DapError> {
    for led in [0, 1] {
        let reply = command(link, &[DAP_HOST_STATUS, led, u8::from(on)])?;
        if reply.get(1) != Some(&DAP_OK) {
            return Err(DapError::Short {
                command: DAP_HOST_STATUS,
                len: reply.len(),
            });
        }
    }
    Ok(())
}

/// Asks ours to enter the ROM bootloader through its endpoint 0 request.
pub fn request_ours_bootloader(link: &mut dyn DapLink) -> Result<(), LinkError> {
    link.control_out(REBOOT_REQUEST_TYPE, REBOOT_REQUEST, REBOOT_TO_BOOTLOADER, 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers from a fixed table, and records what it was asked.
    struct Table {
        replies: Vec<(Vec<u8>, Vec<u8>)>,
        asked: Vec<Vec<u8>>,
    }

    impl DapLink for Table {
        fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError> {
            self.asked.push(request.to_vec());
            self.replies
                .iter()
                .find(|(req, _)| req == request)
                .map(|(_, reply)| reply.clone())
                .ok_or(LinkError::Timeout)
        }
        fn drain(&mut self) -> Result<usize, LinkError> {
            Ok(0)
        }
        fn control_out(&mut self, _: u8, _: u8, _: u16, _: u16) -> Result<(), LinkError> {
            Ok(())
        }
    }

    fn table(replies: &[(&[u8], &[u8])]) -> Table {
        Table {
            replies: replies
                .iter()
                .map(|(a, b)| (a.to_vec(), b.to_vec()))
                .collect(),
            asked: Vec::new(),
        }
    }

    #[test]
    fn identify_ours() {
        let mut reply = vec![0x82];
        reply.extend_from_slice(b"CJTAG");
        reply.extend_from_slice(&[2, 61, 7]);
        reply.extend_from_slice(b"14d05d4");
        let ours = parse_identify(&reply).unwrap().unwrap();
        assert_eq!(ours.format, 2);
        assert_eq!(ours.flags, 61);
        assert_eq!(ours.build, "14d05d4");
        assert_eq!(
            ours.flag_names(),
            [
                "OScan1",
                "target sense",
                "generated SWD",
                "host-driven OScan1 entry",
                "generated JTAG"
            ]
        );
    }

    fn identify_reply(format: u8, build: &str, tail: &[u8]) -> Vec<u8> {
        let mut reply = vec![0x82];
        reply.extend_from_slice(b"CJTAG");
        reply.extend_from_slice(&[format, 61, build.len() as u8]);
        reply.extend_from_slice(build.as_bytes());
        reply.extend_from_slice(tail);
        reply
    }

    #[test]
    fn the_board_byte_is_read_from_format_3_on() {
        let pro = parse_identify(&identify_reply(3, "v0.3.0", &[Ours::BOARD_MCU_LINK_PRO]))
            .unwrap()
            .unwrap();
        assert_eq!(pro.board, Some(Ours::BOARD_MCU_LINK_PRO));
        // A format 2 reply read over v1 HID carries padding after the build string.
        let padded = parse_identify(&identify_reply(2, "14d05d4", &[1, 0, 0, 0]))
            .unwrap()
            .unwrap();
        assert_eq!(padded.board, None);
        assert!(matches!(
            parse_identify(&identify_reply(3, "v0.3.0", &[])),
            Err(DapError::Short { command: 0x82, .. })
        ));
    }

    #[test]
    fn the_scan_formats_are_read_from_format_4_on() {
        let ours = parse_identify(&identify_reply(4, "26ea505", &[0, 0x00, 0x06, 0, 0]))
            .unwrap()
            .unwrap();
        assert_eq!(ours.scan_formats, Some(0x600));
        assert_eq!(
            ours.scan_format_names(),
            Some(vec!["OScan1".to_owned(), "OScan2".to_owned()])
        );
        let older = parse_identify(&identify_reply(3, "v0.3.0", &[0, 0, 0, 0, 0]))
            .unwrap()
            .unwrap();
        assert_eq!(older.scan_formats, None);
        assert!(matches!(
            parse_identify(&identify_reply(4, "26ea505", &[0, 0x00, 0x06])),
            Err(DapError::Short { command: 0x82, .. })
        ));
    }

    #[test]
    fn a_dio_reply_carries_its_groups() {
        let dio = parse_dio(&[0x8E, 0x00, 0x07, 0x05, 0x0D, 0x0F, 2, 0x07, 0x08])
            .unwrap()
            .unwrap();
        assert_eq!(
            dio,
            Dio {
                accepted: true,
                pins: 0x0F,
                outputs: 0x07,
                levels: 0x05,
                read: 0x0D,
                groups: vec![0x07, 0x08],
            }
        );
        // Turning DIO0 back to an input alone would leave DIO1 and DIO2 driving.
        assert_eq!(dio.split(0x01, 0x00), Some(0x07));
        assert_eq!(dio.split(0x07, 0x00), None);
        assert_eq!(dio.split(0x08, 0x08), None);
        let refused = parse_dio(&[0x8E, 0xFF, 0, 0, 0, 0, 0]).unwrap().unwrap();
        assert!(!refused.accepted && refused.pins == 0 && refused.groups.is_empty());
        assert_eq!(parse_dio(&[0xFF]).unwrap(), None);
        assert!(matches!(
            parse_dio(&[0x8E, 0x00, 0x07, 0x05, 0x0D, 0x0F, 2, 0x07]),
            Err(DapError::Short { command: 0x8E, .. })
        ));
    }

    #[test]
    fn identify_stock() {
        assert_eq!(parse_identify(&[0xFF]).unwrap(), None);
        assert_eq!(parse_identify(&[0x82]).unwrap(), None);
        assert_eq!(
            parse_identify(&[0x82, b'C', b'J', b'T', b'A', b'X']).unwrap(),
            None
        );
        assert!(matches!(
            parse_identify(&[0x00, 0x01]),
            Err(DapError::OutOfStep { asked: 0x82, .. })
        ));
    }

    #[test]
    fn identify_truncated_build_is_an_error() {
        let mut reply = vec![0x82];
        reply.extend_from_slice(b"CJTAG");
        reply.extend_from_slice(&[2, 61, 9]);
        reply.extend_from_slice(b"14d05d4");
        assert!(matches!(
            parse_identify(&reply),
            Err(DapError::Short { command: 0x82, .. })
        ));
    }

    #[test]
    fn info_strings_drop_the_terminator() {
        let mut link = table(&[(&[0x00, 0x03], b"\x00\x0bCJ3E7A91C2\x00")]);
        assert_eq!(
            info_string(&mut link, Info::Serial).unwrap().as_deref(),
            Some("CJ3E7A91C2")
        );
        let mut link = table(&[(&[0x00, 0x04], &[0x00, 0x00])]);
        assert_eq!(info_string(&mut link, Info::Protocol).unwrap(), None);
    }

    #[test]
    fn stock_entry_refuses_old_and_unknown_versions_without_sending() {
        let mut link = table(&[]);
        let old = StockVersion {
            major: 3,
            minor: 108,
        };
        assert_eq!(
            request_stock_bootloader(&mut link, Some(old)),
            Err(EntryError::StockTooOld(old))
        );
        assert_eq!(
            request_stock_bootloader(&mut link, None),
            Err(EntryError::StockVersionUnknown)
        );
        assert!(link.asked.is_empty());
    }

    #[test]
    fn stock_entry_sends_the_request_then_disconnect() {
        let mut link = table(&[
            (&[0x81, 0x01], &[0x81, 0x00, 0x01]),
            (&[0x03], &[0x03, 0x00]),
        ]);
        let version = StockVersion::parse("3.172");
        assert_eq!(request_stock_bootloader(&mut link, version), Ok(None));
        assert_eq!(link.asked, [vec![0x81, 0x01], vec![0x03]]);
    }

    #[test]
    fn stock_entry_refuses_any_other_reply() {
        let version = StockVersion::parse("3.172");
        for reply in [&[0x81, 0x00, 0x00][..], &[0x81, 0x01], &[0xFF], &[0x81]] {
            let mut link = table(&[(&[0x81, 0x01], reply), (&[0x03], &[0x03, 0x00])]);
            assert!(
                matches!(
                    request_stock_bootloader(&mut link, version),
                    Err(EntryError::StockRefused(_))
                ),
                "accepted {reply:02x?}"
            );
            assert_eq!(
                link.asked,
                [vec![0x81, 0x01]],
                "sent more after {reply:02x?}"
            );
        }
    }
}
