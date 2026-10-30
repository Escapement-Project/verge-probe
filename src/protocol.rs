//! Shared Verge vendor-command payloads, independent of the transport.

use probe_rs::probe::ScanFormat;

pub(crate) const INFO_COMMAND: u8 = 0x82;
pub(crate) const CJTAG_CONTROL_COMMAND: u8 = 0x83;
pub(crate) const ENERGY_COMMAND: u8 = 0x90;
pub(crate) const TARGET_SENSE_COMMAND: u8 = 0x86;
pub(crate) const TARGET_POWER_COMMAND: u8 = 0x8F;
pub(crate) const DIO_COMMAND: u8 = 0x8E;
pub(crate) const REBOOT_COMMAND: u8 = 0x85;
pub(crate) const J8_COMMAND: u8 = 0x84;
pub(crate) const SAMPLE_COMMAND: u8 = 0x87;
pub(crate) const CODE_LAB_COMMAND: u8 = 0x88;
pub(crate) const SWD_CODEGEN_COMMAND: u8 = 0x89;
pub(crate) const JTAG_CODEGEN_COMMAND: u8 = 0x8A;
pub(crate) const JTAG_PACING_COMMAND: u8 = 0x8B;
pub(crate) const CPU_STAGING_COMMAND: u8 = 0x8C;
pub(crate) const JTAG_LOG_COMMAND: u8 = 0x8D;
pub(crate) const SPI_COMMAND: u8 = 0x91;
pub(crate) const I2C_COMMAND: u8 = 0x92;
pub(crate) const CAP_OSCAN1: u8 = 1 << 0;
pub(crate) const CAP_OSCAN1_ACTIVE: u8 = 1 << 1;
pub(crate) const CAP_OSCAN1_AFTER_ACTIVATION: u8 = 1 << 4;
pub(crate) const CAP_TARGET_SENSE: u8 = 1 << 2;
pub(crate) const CAP_SWD_CODEGEN: u8 = 1 << 3;
pub(crate) const CAP_JTAG_CODEGEN: u8 = 1 << 5;
pub(crate) const CAP_DIO: u8 = 1 << 6;
pub(crate) const CAP_TARGET_POWER: u8 = 1 << 7;
pub(crate) const CAP_ENERGY_CAPTURE: u32 = 1 << 0;
// Bit 1 advertised the same fields with the current in microamps. Reading it would show older
// firmware's current a thousand times too small, so those probes give raw samples only.
pub(crate) const CAP_CALIBRATED_ENERGY: u32 = 1 << 5;
pub(crate) const CAP_ENERGY_ACCUMULATOR: u32 = 1 << 2;
pub(crate) const CAP_DIO_AUX: u32 = 1 << 6;
pub(crate) const CAP_SPI: u32 = 1 << 3;
pub(crate) const CAP_I2C: u32 = 1 << 4;
pub(crate) const SPI_SOURCE_HZ: u32 = 96_000_000;
pub(crate) const SPI_MIN_CLOCK_HZ: u32 = SPI_SOURCE_HZ / 4096;
pub(crate) const SPI_MAX_CLOCK_HZ: u32 = SPI_SOURCE_HZ / 2;
pub(crate) const I2C_MIN_CLOCK_HZ: u32 = 10_000;
pub(crate) const I2C_MAX_CLOCK_HZ: u32 = 1_000_000;
pub(crate) const SPI_MAX_TRANSFER: usize = 504;
pub(crate) const I2C_MAX_WRITE: usize = 502;
pub(crate) const I2C_MAX_READ: usize = 508;
pub(crate) const TAP_RUN_TEST_IDLE: u8 = 1;
pub(crate) const SAMPLE_FILL_MAX: u8 = 32;

/// Firmware identity and capabilities returned by vendor command `0x82`.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct VergeInfo {
    /// Info format version.
    pub version: u8,
    /// Firmware capability byte.
    pub capabilities: u8,
    /// Firmware build string.
    pub firmware: String,
    /// Board kind reported by info format version 3 and later.
    pub board: Option<BoardKind>,
    /// Bit `n` is set when the probe frames scan format `n`.
    pub scan_formats: u32,
    /// Energy and other extended capability bits.
    pub extended_capabilities: u32,
}

/// Board kind reported by Verge firmware info.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BoardKind {
    /// Standalone MCU-Link probe.
    Plain,
    /// MCU-Link Pro.
    Pro,
    /// MCU-Link integrated on a development board.
    OnBoard,
    /// A board kind added by a newer firmware.
    Unknown(u8),
}

impl BoardKind {
    pub(crate) fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::Plain,
            1 => Self::Pro,
            2 => Self::OnBoard,
            other => Self::Unknown(other),
        }
    }
}

impl VergeInfo {
    /// Whether the firmware frames OScan1 after host activation.
    pub fn frames_after_host_activation(&self) -> bool {
        self.capabilities & CAP_OSCAN1_AFTER_ACTIVATION != 0
    }

    /// Whether the firmware frames `format` after host activation.
    pub fn frames(&self, format: ScanFormat) -> bool {
        if format == ScanFormat::OScan1 {
            self.frames_after_host_activation()
        } else {
            self.scan_formats & (1 << format.number()) != 0
        }
    }

    /// Whether the firmware implements OScan1 framing.
    pub fn supports_oscan1(&self) -> bool {
        self.capabilities & CAP_OSCAN1 != 0
    }

    /// Whether the firmware has OScan1 framing active.
    pub fn oscan1_active(&self) -> bool {
        self.capabilities & CAP_OSCAN1_ACTIVE != 0
    }

    /// Whether the firmware supports target sensing.
    pub fn supports_target_sense(&self) -> bool {
        self.capabilities & CAP_TARGET_SENSE != 0
    }

    /// Whether the firmware can generate SWD paths.
    pub fn supports_swd_codegen(&self) -> bool {
        self.capabilities & CAP_SWD_CODEGEN != 0
    }

    /// Whether the firmware can generate JTAG paths.
    pub fn supports_jtag_codegen(&self) -> bool {
        self.capabilities & CAP_JTAG_CODEGEN != 0
    }

    /// Whether the firmware exposes its DIO pins.
    pub fn supports_dio(&self) -> bool {
        self.capabilities & CAP_DIO != 0
    }

    /// Whether Verge Aux carries DIO commands.
    pub fn supports_dio_on_aux(&self) -> bool {
        self.extended_capabilities & CAP_DIO_AUX != 0
    }

    /// Whether the firmware can switch the target supply.
    pub fn supports_target_power(&self) -> bool {
        self.capabilities & CAP_TARGET_POWER != 0
    }

    /// Whether the firmware supports voltage/current capture.
    pub fn supports_energy_capture(&self) -> bool {
        self.extended_capabilities & CAP_ENERGY_CAPTURE != 0
    }

    /// Whether latest-sample replies contain calibrated fields.
    pub fn supports_calibrated_energy(&self) -> bool {
        self.extended_capabilities & CAP_CALIBRATED_ENERGY != 0
    }

    /// Whether the firmware supports reading its accumulated energy counter.
    pub fn supports_energy_accumulator(&self) -> bool {
        self.extended_capabilities & CAP_ENERGY_ACCUMULATOR != 0
    }

    /// Whether the firmware exposes the Pro SPI bridge.
    pub fn supports_spi(&self) -> bool {
        self.extended_capabilities & CAP_SPI != 0
    }

    /// Whether the firmware exposes the Pro I2C bridge.
    pub fn supports_i2c(&self) -> bool {
        self.extended_capabilities & CAP_I2C != 0
    }
}

/// State of the Verge Pro's DIO pins.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct DioState {
    /// Whether the requested pin update was accepted.
    pub status: VendorStatus,
    /// Pins currently configured as outputs.
    pub outputs: u8,
    /// Output latch values.
    pub levels: u8,
    /// Sampled pin levels.
    pub read: u8,
    /// Pins present on this board.
    pub pins: u8,
    /// Pin masks that must change direction together.
    pub groups: Vec<u8>,
}

/// Result status reported by a Verge vendor command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum VendorStatus {
    /// The firmware accepted the request.
    Ok,
    /// The firmware refused the request.
    Error,
    /// An unrecognized status byte.
    Unknown(u8),
}

impl VendorStatus {
    pub(crate) fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::Ok,
            0xFF => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

impl VergeInfo {
    pub const FLAG_OSCAN1: u8 = CAP_OSCAN1;
    pub const FLAG_OSCAN1_ACTIVE: u8 = CAP_OSCAN1_ACTIVE;
    pub const FLAG_TARGET_SENSE: u8 = CAP_TARGET_SENSE;
    pub const FLAG_SWD_CODEGEN: u8 = CAP_SWD_CODEGEN;
    pub const FLAG_OSCAN1_ENTER: u8 = CAP_OSCAN1_AFTER_ACTIVATION;
    pub const FLAG_JTAG_CODEGEN: u8 = CAP_JTAG_CODEGEN;
    /// [`crate::dap::VENDOR_DIO`] acts: a Pro whose USB-SIO jumper, J15, was not fitted when it started.
    pub const FLAG_DIO: u8 = CAP_DIO;
    pub const FLAG_TARGET_POWER: u8 = CAP_TARGET_POWER;

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
            (Self::FLAG_TARGET_POWER, "target power"),
        ]
        .into_iter()
        .filter(|(bit, _)| self.capabilities & bit != 0)
        .map(|(_, name)| name)
        .chain(
            [
                (CAP_ENERGY_CAPTURE, "energy capture"),
                (CAP_CALIBRATED_ENERGY, "calibrated energy"),
                (CAP_ENERGY_ACCUMULATOR, "energy accumulator"),
                (CAP_SPI, "SPI bridge"),
                (CAP_I2C, "I2C bridge"),
                (CAP_DIO_AUX, "DIO on Aux"),
            ]
            .into_iter()
            .filter(|(bit, _)| self.extended_capabilities & bit != 0)
            .map(|(_, name)| name),
        )
        .collect()
    }

    /// The two-pin scan formats offered, by name, or `None` before format 4.
    #[must_use]
    pub fn scan_format_names(&self) -> Option<Vec<String>> {
        (self.version >= 4).then(|| {
            let mask = self.scan_formats;
            (0..32)
                .filter(|n| mask >> n & 1 != 0)
                .map(crate::dap::scan_format_name)
                .collect()
        })
    }
}

impl BoardKind {
    /// The board number reported by the firmware.
    pub const fn number(self) -> u8 {
        match self {
            Self::Plain => 0,
            Self::Pro => 1,
            Self::OnBoard => 2,
            Self::Unknown(number) => number,
        }
    }
}

impl DioState {
    pub fn accepted(&self) -> bool {
        self.status == VendorStatus::Ok
    }

    /// The direction group a requested update would split.
    pub fn split(&self, mask: u8, outputs: u8) -> Option<u8> {
        let next = (self.outputs & !mask) | (outputs & mask);
        self.groups
            .iter()
            .copied()
            .find(|&group| next & group != 0 && next & group != group)
    }
}

pub(crate) fn encode_dio(mask: u8, outputs: u8, levels: u8) -> [u8; 3] {
    [mask, outputs & mask, levels & mask]
}

pub(crate) fn encode_scan_formats(mask: u32) -> [u8; 5] {
    let mut request = [4, 0, 0, 0, 0];
    request[1..].copy_from_slice(&mask.to_le_bytes());
    request
}

pub(crate) fn decode_info(reply: &[u8]) -> Result<Option<VergeInfo>, &'static str> {
    let Some(rest) = reply.strip_prefix(b"CJTAG") else {
        return Ok(None);
    };
    let version = *rest.first().ok_or("short identity response")?;
    let mut info = VergeInfo {
        version,
        capabilities: 0,
        firmware: String::new(),
        board: None,
        scan_formats: 0,
        extended_capabilities: 0,
    };
    if version < 2 {
        return Ok(Some(info));
    }
    let short = "short identity response";
    info.capabilities = *rest.get(1).ok_or(short)?;
    let len = usize::from(*rest.get(2).ok_or(short)?);
    info.firmware = String::from_utf8_lossy(rest.get(3..3 + len).ok_or(short)?).into_owned();
    let mut tail = rest.get(3 + len..).ok_or(short)?;
    if version >= 3 {
        info.board = Some(BoardKind::from_byte(*tail.first().ok_or(short)?));
        tail = &tail[1..];
    }
    if version >= 4 {
        info.scan_formats =
            u32::from_le_bytes(tail.get(..4).ok_or(short)?.try_into().map_err(|_| short)?);
    }
    if version >= 6 {
        info.extended_capabilities =
            u32::from_le_bytes(tail.get(4..8).ok_or(short)?.try_into().map_err(|_| short)?);
    }
    Ok(Some(info))
}

pub(crate) fn decode_dio(reply: &[u8]) -> Result<DioState, &'static str> {
    let fixed = reply.get(..6).ok_or("short DIO response")?;
    let groups = reply
        .get(6..6 + usize::from(fixed[5]))
        .ok_or("short DIO groups response")?;
    Ok(DioState {
        status: VendorStatus::from_byte(fixed[0]),
        outputs: fixed[1],
        levels: fixed[2],
        read: fixed[3],
        pins: fixed[4],
        groups: groups.to_vec(),
    })
}
