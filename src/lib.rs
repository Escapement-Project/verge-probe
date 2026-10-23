//! Host operations for MCU-Link probes running the Verge firmware.

use probe_rs::probe::{
    DebugProbeError, Probe, ScanFormat,
    cmsisdap::{CmsisDap, CmsisDapCjtagExtension},
};

const INFO_COMMAND: u8 = 0x82;
const CJTAG_CONTROL_COMMAND: u8 = 0x83;
const ENERGY_COMMAND: u8 = 0x90;
const CAP_OSCAN1_AFTER_ACTIVATION: u8 = 1 << 4;
const CAP_ENERGY_CAPTURE: u32 = 1 << 0;
const CAP_CALIBRATED_ENERGY: u32 = 1 << 1;
const CAP_ENERGY_ACCUMULATOR: u32 = 1 << 2;
const TAP_RUN_TEST_IDLE: u8 = 1;

/// Firmware identity and capabilities returned by vendor command `0x82`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VergeInfo {
    /// Info format version.
    pub version: u8,
    /// Firmware capability byte.
    pub capabilities: u8,
    /// Firmware build string.
    pub firmware: String,
    /// Bit `n` is set when the probe frames scan format `n`.
    pub scan_formats: u32,
    /// Energy and other extended capability bits.
    pub extended_capabilities: u32,
}

impl VergeInfo {
    /// Whether the firmware frames OScan1 after host activation.
    pub fn frames_after_host_activation(&self) -> bool {
        self.capabilities & CAP_OSCAN1_AFTER_ACTIVATION != 0
    }

    /// Whether the firmware frames `format` after host activation.
    pub fn frames(&self, format: ScanFormat) -> bool {
        self.frames_after_host_activation()
            && (format == ScanFormat::OScan1 || self.scan_formats & (1 << format.number()) != 0)
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
}

/// A result code returned by an energy-capture command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EnergyStatus {
    /// The operation succeeded.
    Ok,
    /// Capture has not started or is stopped.
    Stopped,
    /// Capture has started but has no valid pair yet.
    NoSample,
    /// The operation is unavailable.
    Unavailable,
    /// The requested rate is unsupported.
    BadRate,
    /// The request has an invalid argument.
    BadArgument,
    /// ADC calibration failed.
    AdcError,
    /// DMA failed.
    DmaError,
    /// An unrecognized result code.
    Unknown(u8),
}

impl EnergyStatus {
    fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::Ok,
            1 => Self::Stopped,
            2 => Self::NoSample,
            3 => Self::Unavailable,
            4 => Self::BadRate,
            5 => Self::BadArgument,
            6 => Self::AdcError,
            7 => Self::DmaError,
            other => Self::Unknown(other),
        }
    }
}

/// One raw voltage/current pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawEnergySample {
    /// Target-voltage ADC code.
    pub voltage_code: u16,
    /// Target-current ADC code.
    pub current_code: u16,
    /// Firmware pair sequence number.
    pub sequence: u32,
}

/// Calibrated values attached to a sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibratedEnergySample {
    /// Target voltage in millivolts.
    pub voltage_millivolts: u32,
    /// Target current in microamps, positive into the target.
    pub current_microamps: i32,
    /// Per-sample current-sense path, 0 through 4.
    pub range_id: u8,
}

/// Result of starting an energy capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnergyCaptureStart {
    /// Firmware result code.
    pub status: EnergyStatus,
    /// Effective voltage/current pairs per second.
    pub actual_rate_pairs_per_second: u32,
}

/// Result of requesting the latest complete voltage/current pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnergyCaptureSample {
    /// Firmware result code.
    pub status: EnergyStatus,
    /// Raw pair when the result is `Ok`.
    pub sample: Option<RawEnergySample>,
    /// Calibrated values when advertised and valid.
    pub calibrated: Option<CalibratedEnergySample>,
}

/// Accumulated energy since the last successful capture start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnergyAccumulator {
    /// Firmware result code.
    pub status: EnergyStatus,
    /// Accumulated energy in nanojoules.
    pub energy_nanojoules: u64,
    /// Number of valid calibrated pairs included in the total.
    pub integrated_pairs: u64,
}

/// Operations provided by the Verge firmware over CMSIS-DAP vendor commands.
pub struct VergeProbe<'a> {
    probe: &'a mut Probe,
    info: Option<VergeInfo>,
}

impl<'a> VergeProbe<'a> {
    /// Creates a Verge command client for an already-open probe.
    pub fn new(probe: &'a mut Probe) -> Self {
        Self { probe, info: None }
    }

    /// Reads the firmware identity and capabilities.
    pub fn info(&mut self) -> Result<Option<VergeInfo>, DebugProbeError> {
        if self.info.is_none() {
            self.info = read_info(&mut |command_id, payload| {
                self.probe.send_vendor_command(command_id, payload)
            })?;
        }
        Ok(self.info.clone())
    }

    /// Starts voltage/current capture at the requested pair rate.
    pub fn start_energy_capture(
        &mut self,
        rate_pairs_per_second: u32,
    ) -> Result<EnergyCaptureStart, DebugProbeError> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let mut request = [0; 5];
        request[0] = 0;
        request[1..].copy_from_slice(&rate_pairs_per_second.to_le_bytes());
        let response = self.send(ENERGY_COMMAND, &request)?;
        check_dap_status(&response)?;
        Ok(EnergyCaptureStart {
            status: EnergyStatus::from_byte(response_byte(&response, 1)?),
            actual_rate_pairs_per_second: u32::from_le_bytes(response_array(&response, 2..6)?),
        })
    }

    /// Stops voltage/current capture.
    pub fn stop_energy_capture(&mut self) -> Result<EnergyStatus, DebugProbeError> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let response = self.send(ENERGY_COMMAND, &[2])?;
        check_dap_status(&response)?;
        Ok(EnergyStatus::from_byte(response_byte(&response, 1)?))
    }

    /// Reads the latest complete voltage/current pair.
    pub fn latest_energy_sample(&mut self) -> Result<EnergyCaptureSample, DebugProbeError> {
        let info = self
            .require_capability(CAP_ENERGY_CAPTURE, "energy capture")?
            .clone();
        let response = self.send(ENERGY_COMMAND, &[1])?;
        check_dap_status(&response)?;
        let status = EnergyStatus::from_byte(response_byte(&response, 1)?);
        let sample = if status == EnergyStatus::Ok {
            Some(RawEnergySample {
                voltage_code: u16::from_le_bytes(response_array(&response, 2..4)?),
                current_code: u16::from_le_bytes(response_array(&response, 4..6)?),
                sequence: u32::from_le_bytes(response_array(&response, 6..10)?),
            })
        } else {
            None
        };
        let calibrated = if info.supports_calibrated_energy() {
            let valid = response_byte(&response, 19)?;
            if valid > 1 {
                return Err(protocol_error("invalid calibrated-valid byte"));
            }
            if status == EnergyStatus::Ok && valid == 1 {
                Some(CalibratedEnergySample {
                    voltage_millivolts: u32::from_le_bytes(response_array(&response, 10..14)?),
                    current_microamps: i32::from_le_bytes(response_array(&response, 14..18)?),
                    range_id: response_byte(&response, 18)?,
                })
            } else {
                None
            }
        } else {
            None
        };
        Ok(EnergyCaptureSample {
            status,
            sample,
            calibrated,
        })
    }

    /// Reads the energy total without resetting it.
    pub fn read_energy_accumulator(&mut self) -> Result<EnergyAccumulator, DebugProbeError> {
        self.require_capability(CAP_ENERGY_ACCUMULATOR, "energy accumulator")?;
        let response = self.send(ENERGY_COMMAND, &[3])?;
        check_dap_status(&response)?;
        Ok(EnergyAccumulator {
            status: EnergyStatus::from_byte(response_byte(&response, 1)?),
            energy_nanojoules: u64::from_le_bytes(response_array(&response, 2..10)?),
            integrated_pairs: u64::from_le_bytes(response_array(&response, 10..18)?),
        })
    }

    fn require_capability(
        &mut self,
        capability: u32,
        command_name: &'static str,
    ) -> Result<&VergeInfo, DebugProbeError> {
        let info = self
            .info()?
            .ok_or(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "Verge firmware",
            })?;
        if info.extended_capabilities & capability == 0 {
            return Err(DebugProbeError::CommandNotSupportedByProbe { command_name });
        }
        self.info
            .as_ref()
            .ok_or(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "Verge firmware",
            })
    }

    fn send(&mut self, command_id: u8, payload: &[u8]) -> Result<Vec<u8>, DebugProbeError> {
        self.probe.send_vendor_command(command_id, payload)
    }
}

/// Installs Verge's cJTAG vendor-command adapter on an opened CMSIS-DAP probe.
pub fn install_cjtag_support(probe: &mut Probe) -> Result<(), DebugProbeError> {
    let cmsisdap =
        probe
            .try_into::<CmsisDap>()
            .ok_or(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "CMSIS-DAP cJTAG extension",
            })?;
    cmsisdap.set_cjtag_extension(Box::new(VergeCjtagExtension));
    Ok(())
}

struct VergeCjtagExtension;

impl CmsisDapCjtagExtension for VergeCjtagExtension {
    fn scan_formats(
        &mut self,
        send_vendor_command: &mut dyn FnMut(u8, &[u8]) -> Result<Vec<u8>, DebugProbeError>,
    ) -> Result<Vec<ScanFormat>, DebugProbeError> {
        let Some(info) = read_info(send_vendor_command)? else {
            return Ok(Vec::new());
        };
        Ok([
            ScanFormat::OScan1,
            ScanFormat::OScan2,
            ScanFormat::OScan5,
            ScanFormat::OScan6,
        ]
        .into_iter()
        .filter(|format| info.frames(*format))
        .collect())
    }

    fn set_framing(
        &mut self,
        format: Option<ScanFormat>,
        send_vendor_command: &mut dyn FnMut(u8, &[u8]) -> Result<Vec<u8>, DebugProbeError>,
    ) -> Result<(), DebugProbeError> {
        let request = match format {
            None => vec![0],
            Some(ScanFormat::OScan1) => vec![2],
            Some(format) => vec![3, format.number(), TAP_RUN_TEST_IDLE],
        };
        let response = send_vendor_command(CJTAG_CONTROL_COMMAND, &request)?;
        check_dap_status(&response)
    }
}

fn read_info(
    send_vendor_command: &mut dyn FnMut(u8, &[u8]) -> Result<Vec<u8>, DebugProbeError>,
) -> Result<Option<VergeInfo>, DebugProbeError> {
    let reply = send_vendor_command(INFO_COMMAND, &[])?;
    let Some(rest) = reply.strip_prefix(b"CJTAG") else {
        return Ok(None);
    };
    let version = response_byte(rest, 0)?;
    let capabilities = response_byte(rest, 1)?;
    let build_len = usize::from(response_byte(rest, 2)?);
    let build = rest
        .get(3..3 + build_len)
        .ok_or_else(|| protocol_error("short firmware build field"))?;
    let after_board = rest
        .get(3 + build_len + 1..)
        .ok_or_else(|| protocol_error("short cJTAG info reply"))?;
    let scan_formats = if version >= 4 {
        u32::from_le_bytes(response_array(after_board, 0..4)?)
    } else {
        0
    };
    let extended_capabilities = if version >= 6 {
        u32::from_le_bytes(response_array(after_board, 4..8)?)
    } else {
        0
    };
    Ok(Some(VergeInfo {
        version,
        capabilities,
        firmware: String::from_utf8_lossy(build).into_owned(),
        scan_formats,
        extended_capabilities,
    }))
}

fn check_dap_status(response: &[u8]) -> Result<(), DebugProbeError> {
    match response_byte(response, 0)? {
        0 => Ok(()),
        0xFF => Err(protocol_error("vendor command returned DAP_ERROR")),
        _ => Err(protocol_error("invalid CMSIS-DAP status byte")),
    }
}

fn response_byte(response: &[u8], index: usize) -> Result<u8, DebugProbeError> {
    response
        .get(index)
        .copied()
        .ok_or_else(|| protocol_error("short vendor command response"))
}

fn response_array<const N: usize>(
    response: &[u8],
    range: std::ops::Range<usize>,
) -> Result<[u8; N], DebugProbeError> {
    response
        .get(range)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or_else(|| protocol_error("short vendor command response"))
}

fn protocol_error(message: &str) -> DebugProbeError {
    DebugProbeError::Other(message.to_owned())
}
