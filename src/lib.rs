//! Host operations for MCU-Link probes running the Verge firmware.

use probe_rs::{
    Session,
    probe::{
        DebugProbeError, Probe, ScanFormat,
        cmsisdap::{CmsisDap, CmsisDapCjtagExtension},
    },
};

const INFO_COMMAND: u8 = 0x82;
const CJTAG_CONTROL_COMMAND: u8 = 0x83;
const ENERGY_COMMAND: u8 = 0x90;
const TARGET_SENSE_COMMAND: u8 = 0x86;
const TARGET_POWER_COMMAND: u8 = 0x8F;
const DIO_COMMAND: u8 = 0x8E;
const REBOOT_COMMAND: u8 = 0x85;
const J8_COMMAND: u8 = 0x84;
const SAMPLE_COMMAND: u8 = 0x87;
const CODE_LAB_COMMAND: u8 = 0x88;
const SWD_CODEGEN_COMMAND: u8 = 0x89;
const JTAG_CODEGEN_COMMAND: u8 = 0x8A;
const JTAG_PACING_COMMAND: u8 = 0x8B;
const CPU_STAGING_COMMAND: u8 = 0x8C;
const JTAG_LOG_COMMAND: u8 = 0x8D;
const CAP_OSCAN1: u8 = 1 << 0;
const CAP_OSCAN1_ACTIVE: u8 = 1 << 1;
const CAP_OSCAN1_AFTER_ACTIVATION: u8 = 1 << 4;
const CAP_TARGET_SENSE: u8 = 1 << 2;
const CAP_SWD_CODEGEN: u8 = 1 << 3;
const CAP_JTAG_CODEGEN: u8 = 1 << 5;
const CAP_DIO: u8 = 1 << 6;
const CAP_TARGET_POWER: u8 = 1 << 7;
const CAP_ENERGY_CAPTURE: u32 = 1 << 0;
const CAP_CALIBRATED_ENERGY: u32 = 1 << 1;
const CAP_ENERGY_ACCUMULATOR: u32 = 1 << 2;
const TAP_RUN_TEST_IDLE: u8 = 1;
const SAMPLE_FILL_MAX: u8 = 32;

/// Firmware identity and capabilities returned by vendor command `0x82`.
#[derive(Clone, Debug, PartialEq, Eq)]
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
    fn from_byte(value: u8) -> Self {
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

/// One target-sense conversion returned by vendor command `0x86`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TargetSenseReading {
    /// Whether the firmware accepted the conversion.
    pub status: VendorStatus,
    /// Raw ADC code.
    pub code: u16,
    /// Millivolts at the selected pin.
    pub millivolts: u16,
    /// Whether the target-detect line is grounded.
    pub detected: bool,
}

/// State of the Verge Pro's DIO pins.
#[derive(Clone, Debug, PartialEq, Eq)]
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

/// Verge firmware reboot destination.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RebootMode {
    /// Restart the Verge firmware.
    Firmware,
    /// Enter the probe's ROM bootloader for reflashing.
    Bootloader,
}

/// Code-generation behavior used by vendor commands `0x89` and `0x8A`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum CodegenMode {
    /// Report current state without changing measurement behavior.
    #[default]
    Keep = 0,
    /// Measure latency on the active wire now.
    Measure = 1,
    /// Stop automatic measurement and keep the current latency.
    Pin = 2,
    /// Measure latency automatically when the firmware decides it is needed.
    Automatic = 3,
}

/// Parsed state from the SWD code generator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SwdCodegenState {
    /// Whether the request was accepted.
    pub status: VendorStatus,
    /// Latency used by the generated paths.
    pub latency: u8,
    /// Whether the generated path uses masked timing.
    pub masked: bool,
    /// Whether code generation succeeded.
    pub generated: bool,
    /// Clock requested through CMSIS-DAP, in hertz.
    pub clock_hz: u32,
    /// Generated write period, in core cycles.
    pub write_period: u32,
    /// Generated read period, in core cycles.
    pub read_period: u32,
    /// Generated code size, in bytes.
    pub code_bytes: u32,
    /// Successful generation count.
    pub builds: u32,
    /// Failed generation count.
    pub failures: u32,
    /// Latest latency measurement, when returned by the firmware.
    pub measurement: Option<CodegenMeasurement>,
}

/// Parsed state from the JTAG code generator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JtagCodegenState {
    /// Whether the request was accepted.
    pub status: VendorStatus,
    /// Two-pin latency used by generated paths.
    pub two_pin_latency: u8,
    /// Four-wire latency used by generated paths.
    pub four_wire_latency: u8,
    /// Whether the generated path uses masked timing.
    pub masked: bool,
    /// Bit 0 means generated; bit 1 means transfers were generated; bit 2 means data bits loop.
    pub flags: u8,
    /// Clock requested through CMSIS-DAP, in hertz.
    pub clock_hz: u32,
    /// Generated period, in core cycles.
    pub period: u32,
    /// Generated JTAG clock period, in core cycles.
    pub frame: u32,
    /// Generated code size, in bytes.
    pub code_bytes: u32,
    /// Successful generation count.
    pub builds: u32,
    /// Failed generation count.
    pub failures: u32,
    /// Latest latency measurement, when returned by the firmware.
    pub measurement: Option<CodegenMeasurement>,
    /// Number of configurations restored from a retained build, when reported.
    pub restores: Option<u32>,
}

impl JtagCodegenState {
    /// Whether code generation succeeded.
    pub fn generated(&self) -> bool {
        self.flags & 1 != 0
    }

    /// Whether complete transfers were generated instead of running as scans.
    pub fn transfers_generated(&self) -> bool {
        self.flags & 2 != 0
    }

    /// Whether generated transfer data bits use a loop.
    pub fn data_bits_looped(&self) -> bool {
        self.flags & 4 != 0
    }
}

/// JTAG wire selected by the Verge code generator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum JtagWire {
    /// Four-wire JTAG.
    FourWire,
    /// Two-pin OScan1.
    OScan1,
    /// Two-pin OScan2.
    OScan2,
    /// Two-pin OScan5.
    OScan5,
    /// Two-pin OScan6.
    OScan6,
    /// A wire added by newer firmware.
    Unknown(u8),
}

impl JtagWire {
    fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::FourWire,
            1 => Self::OScan1,
            2 => Self::OScan2,
            3 => Self::OScan5,
            4 => Self::OScan6,
            other => Self::Unknown(other),
        }
    }
}

/// Outcome reported by a latency measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum LatencyOutcome {
    /// No measurement has run.
    NeverMeasured,
    /// One latency agrees with every read.
    Measured,
    /// Two neighboring latencies agree; the later one was selected.
    Ambiguous,
    /// Several latencies agree; the latest one was selected.
    Unresolved,
    /// Nothing read correctly at the host or safe period.
    NoBand,
    /// No latency agrees with every read.
    Inconsistent,
    /// An outcome added by newer firmware.
    Unknown(u8),
}

impl LatencyOutcome {
    fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::NeverMeasured,
            1 => Self::Measured,
            2 => Self::Ambiguous,
            3 => Self::Unresolved,
            4 => Self::NoBand,
            5 => Self::Inconsistent,
            other => Self::Unknown(other),
        }
    }
}

/// Result of the latest latency sweep reported by a code generator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CodegenMeasurement {
    /// Whether future configurations trigger automatic measurements.
    pub automatic: bool,
    /// Outcome reported by the latency search.
    pub outcome: LatencyOutcome,
    /// Wire measured by JTAG codegen. SWD measurements do not set this field.
    pub wire: Option<JtagWire>,
    /// Latency selected by the measurement.
    pub latency: u8,
    /// Number of generations attempted.
    pub attempts: u8,
    /// Number of periods measured.
    pub periods: u8,
    /// DPIDR or IDCODE value that read correctly.
    pub id_code: u32,
    /// Core cycles spent in the sweep.
    pub cycles: u32,
    /// Number of reads performed.
    pub runs: u32,
    /// Bit mask of latencies every read agreed with.
    pub consistent_latencies: u64,
    /// Failed connections recovered by the sweep.
    pub rescues: u32,
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
    fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::Ok,
            0xFF => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

/// Position at which the SWD sample instrument samples SWDIO.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SamplePosition {
    /// Sample this many cycles after the rising-edge store.
    AfterEdge(u8),
    /// Use the production-shaped lead path with this many high-phase filler cycles.
    Lead(u8),
    /// Use the production-shaped lead path with the full filler phase.
    BeforeEdge,
}

/// Inputs to the SWD sample-position instrument.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleRequest {
    /// Delay-loop iterations in the low phase, less one.
    pub low: u8,
    /// Filler cycles in each clock, at most 32.
    pub fill: u8,
    /// Sample location within the clock.
    pub position: SamplePosition,
    /// Number of DPIDR reads to make.
    pub count: u16,
    /// DPIDR value expected from a correct read.
    pub expected_dpidr: u32,
}

/// Results from the SWD sample-position instrument.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleResult {
    /// Whether the firmware accepted the request.
    pub status: VendorStatus,
    /// Reads with a good acknowledgement, parity and DPIDR.
    pub good: u16,
    /// Reads with a bad acknowledgement.
    pub bad_acknowledgements: u16,
    /// Reads with a bad DPIDR or parity.
    pub bad_words: u16,
    /// Acknowledgement from the last bad read.
    pub last_bad_acknowledgement: u8,
    /// DPIDR from the last bad read.
    pub last_bad_dpidr: u32,
    /// Dry-run cycles for a 32-clock read.
    pub cycles: u32,
}

/// One executable arena exposed by the optional code-lab firmware command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CodeLabArena {
    /// Arena base address in probe memory.
    pub base_address: u32,
    /// Arena size in bytes.
    pub size: u32,
}

/// State of JTAG pacing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JtagPacingState {
    /// Whether the request was accepted.
    pub status: VendorStatus,
    /// Whether learned pacing is enabled.
    pub enabled: bool,
    /// Learned idle clocks currently added by the generator.
    pub paced_clocks: u8,
}

/// State of command staging between the firmware's USB and probe cores.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuStagingState {
    /// Whether the request was accepted.
    pub status: VendorStatus,
    /// Whether CPU 0 stages DAP commands for CPU 1.
    pub enabled: bool,
}

/// State reported after a target-power request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TargetPowerState {
    /// Whether the firmware accepted the request.
    pub status: VendorStatus,
    /// State last set since the probe booted.
    pub enabled: bool,
}

/// State reported after a J8 pin ownership request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct J8State {
    /// Whether the firmware accepted the request.
    pub status: VendorStatus,
    /// Current levels read from the J8 pins.
    pub levels: u8,
}

/// One JTAG code-generation log entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JtagGenerationLogEntry {
    /// Total generation count when the entry was read.
    pub count: u32,
    /// CMSIS-DAP command that caused the generation.
    pub command: u8,
    /// Generated wire.
    pub wire: JtagWire,
    /// Bit mask of fields changed since the previous generation.
    pub changed: u8,
    /// Generation flags.
    pub flags: u8,
    /// Latency used.
    pub latency: u8,
    /// Bypass bits before the selected TAP.
    pub chain_before: u8,
    /// Bypass bits after the selected TAP.
    pub chain_after: u8,
    /// Idle clocks added.
    pub idle_clocks: u8,
    /// Learned idle clocks included in `idle_clocks`.
    pub paced_clocks: u8,
    /// Generated period, in cycles.
    pub period: u16,
    /// Core-cycle timestamp when generation started.
    pub at: u32,
    /// Generation duration, in core cycles.
    pub cycles: u32,
}

/// Verge firmware operations on the probe already owned by a probe-rs session.
pub struct VergeSession<'a> {
    session: &'a mut Session,
    info_loaded: bool,
    info: Option<VergeInfo>,
}

impl<'a> VergeSession<'a> {
    /// Creates a Verge command client for an attached probe-rs session.
    pub fn new(session: &'a mut Session) -> Self {
        Self {
            session,
            info_loaded: false,
            info: None,
        }
    }

    /// Returns the underlying probe-rs session for ordinary target operations.
    pub fn session(&mut self) -> &mut Session {
        self.session
    }

    /// Reads the firmware identity and capabilities.
    pub fn info(&mut self) -> Result<Option<&VergeInfo>, DebugProbeError> {
        if !self.info_loaded {
            self.refresh_info()?;
        }
        Ok(self.info.as_ref())
    }

    /// Reads firmware identity and capabilities again, replacing the cached values.
    pub fn refresh_info(&mut self) -> Result<Option<&VergeInfo>, DebugProbeError> {
        self.info = read_info(&mut |command_id, payload| {
            self.session.send_vendor_command(command_id, payload)
        })?;
        self.info_loaded = true;
        Ok(self.info.as_ref())
    }

    /// Sends a Verge vendor command directly for firmware features without a typed wrapper.
    pub fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        self.session.send_vendor_command(command_id, payload)
    }

    /// Reads the target supply voltage and target-detect line.
    pub fn read_target_sense(&mut self) -> Result<TargetSenseReading, DebugProbeError> {
        self.require_flag(CAP_TARGET_SENSE, "target sensing")?;
        self.read_target_sense_mode(0, 0)
    }

    /// Converts an ADC0 channel. Channel uses bits 0 through 4; bit 7 selects side B.
    pub fn convert_target_sense_channel(
        &mut self,
        channel: u8,
    ) -> Result<TargetSenseReading, DebugProbeError> {
        self.require_flag(CAP_TARGET_SENSE, "target sensing")?;
        if channel & 0x60 != 0 {
            return Err(protocol_error("invalid target-sense channel"));
        }
        self.read_target_sense_mode(1, channel)
    }

    /// Sets the target supply state and returns the state reported by the probe.
    pub fn set_target_power(&mut self, enabled: bool) -> Result<TargetPowerState, DebugProbeError> {
        self.require_flag(CAP_TARGET_POWER, "target power control")?;
        self.target_power_command(if enabled { 1 } else { 0 })
    }

    /// Reads the target supply state last set since the probe booted.
    pub fn read_target_power(&mut self) -> Result<TargetPowerState, DebugProbeError> {
        self.require_flag(CAP_TARGET_POWER, "target power control")?;
        self.target_power_command(2)
    }

    /// Reads DIO state, or updates the direction and levels for the selected pins.
    ///
    /// A set bit in `mask` selects a pin. Matching bits in `outputs` set output direction,
    /// and matching bits in `levels` set the output latch. A zero mask only reads state.
    pub fn access_dio(
        &mut self,
        mask: u8,
        outputs: u8,
        levels: u8,
    ) -> Result<DioState, DebugProbeError> {
        self.require_flag(CAP_DIO, "DIO pins")?;
        let response = self.send(DIO_COMMAND, &[mask, outputs & mask, levels & mask])?;
        if response.len() < 6 {
            return Err(protocol_error("short DIO response"));
        }
        let status = VendorStatus::from_byte(response_byte(&response, 0)?);
        let groups_len = usize::from(response[5]);
        let groups = response
            .get(6..6 + groups_len)
            .ok_or_else(|| protocol_error("short DIO groups response"))?
            .to_vec();
        Ok(DioState {
            status,
            outputs: response[1],
            levels: response[2],
            read: response[3],
            pins: response[4],
            groups,
        })
    }

    /// Reboots the probe into its firmware or ROM bootloader.
    pub fn reboot(&mut self, mode: RebootMode) -> Result<VendorStatus, DebugProbeError> {
        let response = self.send(
            REBOOT_COMMAND,
            &[match mode {
                RebootMode::Firmware => 0,
                RebootMode::Bootloader => 1,
            }],
        )?;
        Ok(VendorStatus::from_byte(response_byte(&response, 0)?))
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
        let mut reader = ResponseReader::new(&response);
        check_vendor_status(reader.u8()?)?;
        Ok(EnergyCaptureStart {
            status: EnergyStatus::from_byte(reader.u8()?),
            actual_rate_pairs_per_second: reader.u32()?,
        })
    }

    /// Stops voltage/current capture.
    pub fn stop_energy_capture(&mut self) -> Result<EnergyStatus, DebugProbeError> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let response = self.send(ENERGY_COMMAND, &[2])?;
        let mut reader = ResponseReader::new(&response);
        check_vendor_status(reader.u8()?)?;
        Ok(EnergyStatus::from_byte(reader.u8()?))
    }

    /// Reads the latest complete voltage/current pair.
    pub fn latest_energy_sample(&mut self) -> Result<EnergyCaptureSample, DebugProbeError> {
        let calibrated_supported = self
            .require_capability(CAP_ENERGY_CAPTURE, "energy capture")?
            .supports_calibrated_energy();
        let response = self.send(ENERGY_COMMAND, &[1])?;
        let mut reader = ResponseReader::new(&response);
        check_vendor_status(reader.u8()?)?;
        let status = EnergyStatus::from_byte(reader.u8()?);
        let raw_sample = RawEnergySample {
            voltage_code: reader.u16()?,
            current_code: reader.u16()?,
            sequence: reader.u32()?,
        };
        let sample = if status == EnergyStatus::Ok {
            Some(raw_sample)
        } else {
            None
        };
        let calibrated = if calibrated_supported {
            let voltage_millivolts = reader.u32()?;
            let current_microamps = reader.i32()?;
            let range_id = reader.u8()?;
            let valid = read_bool(&mut reader, "calibrated-valid flag")?;
            if status == EnergyStatus::Ok && valid {
                Some(CalibratedEnergySample {
                    voltage_millivolts,
                    current_microamps,
                    range_id,
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
        let mut reader = ResponseReader::new(&response);
        check_vendor_status(reader.u8()?)?;
        Ok(EnergyAccumulator {
            status: EnergyStatus::from_byte(reader.u8()?),
            energy_nanojoules: reader.u64()?,
            integrated_pairs: reader.u64()?,
        })
    }

    /// Hands selected J8 pins from the firmware marker to the host.
    ///
    /// Set `mask` to zero to return all four pins to the marker.
    pub fn set_j8_pins(&mut self, mask: u8, levels: u8) -> Result<J8State, DebugProbeError> {
        let response = self.send(J8_COMMAND, &[mask & 0x0F, levels & mask & 0x0F])?;
        Ok(J8State {
            status: VendorStatus::from_byte(response_byte(&response, 0)?),
            levels: response_byte(&response, 1)?,
        })
    }

    /// Runs the SWD sample-position instrument.
    pub fn sample_swd(&mut self, request: SampleRequest) -> Result<SampleResult, DebugProbeError> {
        if request.fill > SAMPLE_FILL_MAX {
            return Err(protocol_error("sample fill exceeds firmware limit"));
        }
        let at = match request.position {
            SamplePosition::AfterEdge(cycles) if cycles <= request.fill => cycles,
            SamplePosition::Lead(cycles) if cycles <= request.fill => 0x80 | cycles,
            SamplePosition::BeforeEdge => 0xFF,
            _ => return Err(protocol_error("sample position exceeds filler phase")),
        };
        let mut payload = vec![request.low, request.fill, at];
        payload.extend_from_slice(&request.count.to_le_bytes());
        payload.extend_from_slice(&request.expected_dpidr.to_le_bytes());
        let response = self.send(SAMPLE_COMMAND, &payload)?;
        let mut reader = ResponseReader::new(&response);
        Ok(SampleResult {
            status: VendorStatus::from_byte(reader.u8()?),
            good: reader.u16()?,
            bad_acknowledgements: reader.u16()?,
            bad_words: reader.u16()?,
            last_bad_acknowledgement: reader.u8()?,
            last_bad_dpidr: reader.u32()?,
            cycles: reader.u32()?,
        })
    }

    /// Returns the executable arenas exposed by a code-lab firmware build.
    pub fn code_lab_arenas(&mut self) -> Result<Vec<CodeLabArena>, DebugProbeError> {
        let response = self.send(CODE_LAB_COMMAND, &[0])?;
        let mut reader = ResponseReader::new(&response);
        check_vendor_status(reader.u8()?)?;
        let count = usize::from(reader.u8()?);
        (0..count)
            .map(|_| {
                Ok(CodeLabArena {
                    base_address: reader.u32()?,
                    size: reader.u32()?,
                })
            })
            .collect()
    }

    /// Writes caller-provided code or data into a code-lab arena.
    pub fn write_code_lab(
        &mut self,
        arena: u8,
        offset: u16,
        bytes: &[u8],
    ) -> Result<(), DebugProbeError> {
        let length = u16::try_from(bytes.len())
            .map_err(|_| protocol_error("code-lab write is longer than u16::MAX"))?;
        let mut payload = Vec::with_capacity(6 + bytes.len());
        payload.extend_from_slice(&[1, arena]);
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&length.to_le_bytes());
        payload.extend_from_slice(bytes);
        check_dap_status(&self.send(CODE_LAB_COMMAND, &payload)?)
    }

    /// Reads bytes from a code-lab arena.
    pub fn read_code_lab(
        &mut self,
        arena: u8,
        offset: u16,
        length: u16,
    ) -> Result<Vec<u8>, DebugProbeError> {
        let mut payload = vec![2, arena];
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&length.to_le_bytes());
        let response = self.send(CODE_LAB_COMMAND, &payload)?;
        check_dap_status(&response)?;
        response
            .get(1..1 + usize::from(length))
            .map(<[u8]>::to_vec)
            .ok_or_else(|| protocol_error("short code-lab read response"))
    }

    /// Runs a Thumb entry point from a code-lab arena with four 32-bit arguments.
    ///
    /// The firmware masks interrupts while the function runs. A fault resets the probe, and code
    /// that does not return can leave it hung until it is reset.
    pub fn run_code_lab(
        &mut self,
        arena: u8,
        entry_offset: u16,
        arguments: [u32; 4],
    ) -> Result<[u32; 2], DebugProbeError> {
        let mut payload = vec![3, arena];
        payload.extend_from_slice(&entry_offset.to_le_bytes());
        for argument in arguments {
            payload.extend_from_slice(&argument.to_le_bytes());
        }
        let response = self.send(CODE_LAB_COMMAND, &payload)?;
        check_dap_status(&response)?;
        Ok([
            u32::from_le_bytes(response_array(&response, 1..5)?),
            u32::from_le_bytes(response_array(&response, 5..9)?),
        ])
    }

    /// Reads or changes the SWD code generator's latency policy.
    pub fn configure_swd_codegen(
        &mut self,
        latency: u8,
        mode: CodegenMode,
    ) -> Result<SwdCodegenState, DebugProbeError> {
        self.require_flag(CAP_SWD_CODEGEN, "SWD code generation")?;
        let response = self.send(SWD_CODEGEN_COMMAND, &[latency, mode as u8])?;
        let mut reader = ResponseReader::new(&response);
        let status = VendorStatus::from_byte(reader.u8()?);
        let latency = reader.u8()?;
        let masked = read_bool(&mut reader, "SWD masked flag")?;
        let generated = read_bool(&mut reader, "SWD generated flag")?;
        let clock_hz = reader.u32()?;
        let write_period = reader.u32()?;
        let read_period = reader.u32()?;
        let code_bytes = reader.u32()?;
        let builds = reader.u32()?;
        let failures = reader.u32()?;
        let measurement = if reader.remaining() >= 29 {
            Some(read_codegen_measurement(&mut reader, false)?)
        } else {
            None
        };
        Ok(SwdCodegenState {
            status,
            latency,
            masked,
            generated,
            clock_hz,
            write_period,
            read_period,
            code_bytes,
            builds,
            failures,
            measurement,
        })
    }

    /// Reads or changes the JTAG code generator's latency policy.
    pub fn configure_jtag_codegen(
        &mut self,
        two_pin_latency: u8,
        four_wire_latency: u8,
        mode: CodegenMode,
    ) -> Result<JtagCodegenState, DebugProbeError> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let response = self.send(
            JTAG_CODEGEN_COMMAND,
            &[two_pin_latency, four_wire_latency, mode as u8],
        )?;
        let mut reader = ResponseReader::new(&response);
        let status = VendorStatus::from_byte(reader.u8()?);
        let two_pin_latency = reader.u8()?;
        let four_wire_latency = reader.u8()?;
        let masked = read_bool(&mut reader, "JTAG masked flag")?;
        let flags = reader.u8()?;
        let clock_hz = reader.u32()?;
        let period = reader.u32()?;
        let frame = reader.u32()?;
        let code_bytes = reader.u32()?;
        let builds = reader.u32()?;
        let failures = reader.u32()?;
        let measurement = if reader.remaining() >= 30 {
            Some(read_codegen_measurement(&mut reader, true)?)
        } else {
            None
        };
        let restores = if reader.remaining() >= 4 {
            Some(reader.u32()?)
        } else {
            None
        };
        Ok(JtagCodegenState {
            status,
            two_pin_latency,
            four_wire_latency,
            masked,
            flags,
            clock_hz,
            period,
            frame,
            code_bytes,
            builds,
            failures,
            measurement,
            restores,
        })
    }

    /// Reads the current JTAG pacing state or enables/disables learned idle clocks.
    pub fn set_jtag_pacing(
        &mut self,
        enabled: Option<bool>,
    ) -> Result<JtagPacingState, DebugProbeError> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let mode = enabled.map_or(2, u8::from);
        let response = self.send(JTAG_PACING_COMMAND, &[mode])?;
        Ok(JtagPacingState {
            status: VendorStatus::from_byte(response_byte(&response, 0)?),
            enabled: read_bool_at(&response, 1, "JTAG pacing state")?,
            paced_clocks: response_byte(&response, 2)?,
        })
    }

    /// Reads command staging state or selects which firmware core stages DAP requests.
    pub fn set_cpu_staging(
        &mut self,
        enabled: Option<bool>,
    ) -> Result<CpuStagingState, DebugProbeError> {
        let mode = enabled.map_or(2, u8::from);
        let response = self.send(CPU_STAGING_COMMAND, &[mode])?;
        Ok(CpuStagingState {
            status: VendorStatus::from_byte(response_byte(&response, 0)?),
            enabled: read_bool_at(&response, 1, "CPU staging state")?,
        })
    }

    /// Reads one JTAG code-generation log entry, with zero selecting the newest entry.
    pub fn read_jtag_generation_log(
        &mut self,
        generations_back: u8,
    ) -> Result<Option<JtagGenerationLogEntry>, DebugProbeError> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let response = self.send(JTAG_LOG_COMMAND, &[generations_back])?;
        let mut reader = ResponseReader::new(&response);
        match VendorStatus::from_byte(reader.u8()?) {
            VendorStatus::Ok => {}
            VendorStatus::Error => return Ok(None),
            VendorStatus::Unknown(_) => return Err(protocol_error("invalid JTAG log status")),
        }
        Ok(Some(JtagGenerationLogEntry {
            count: reader.u32()?,
            command: reader.u8()?,
            wire: JtagWire::from_byte(reader.u8()?),
            changed: reader.u8()?,
            flags: reader.u8()?,
            latency: reader.u8()?,
            chain_before: reader.u8()?,
            chain_after: reader.u8()?,
            idle_clocks: reader.u8()?,
            paced_clocks: reader.u8()?,
            period: reader.u16()?,
            at: reader.u32()?,
            cycles: reader.u32()?,
        }))
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
        Ok(info)
    }

    fn require_flag(
        &mut self,
        capability: u8,
        command_name: &'static str,
    ) -> Result<(), DebugProbeError> {
        let info = self
            .info()?
            .ok_or(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "Verge firmware",
            })?;
        if info.capabilities & capability == 0 {
            return Err(DebugProbeError::CommandNotSupportedByProbe { command_name });
        }
        Ok(())
    }

    fn read_target_sense_mode(
        &mut self,
        mode: u8,
        channel: u8,
    ) -> Result<TargetSenseReading, DebugProbeError> {
        let response = self.send(TARGET_SENSE_COMMAND, &[mode, channel])?;
        let detected = response_byte(&response, 5)?;
        if detected > 1 {
            return Err(protocol_error("invalid target-detect byte"));
        }
        Ok(TargetSenseReading {
            status: VendorStatus::from_byte(response_byte(&response, 0)?),
            code: u16::from_le_bytes(response_array(&response, 1..3)?),
            millivolts: u16::from_le_bytes(response_array(&response, 3..5)?),
            detected: detected == 1,
        })
    }

    fn target_power_command(&mut self, mode: u8) -> Result<TargetPowerState, DebugProbeError> {
        let response = self.send(TARGET_POWER_COMMAND, &[mode])?;
        let enabled = match response_byte(&response, 1)? {
            0 => false,
            1 => true,
            _ => return Err(protocol_error("invalid target-power state byte")),
        };
        Ok(TargetPowerState {
            status: VendorStatus::from_byte(response_byte(&response, 0)?),
            enabled,
        })
    }

    fn send(&mut self, command_id: u8, payload: &[u8]) -> Result<Vec<u8>, DebugProbeError> {
        self.session.send_vendor_command(command_id, payload)
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

/// Sets the scan formats the Verge firmware offers during future cJTAG discovery.
///
/// Call this on the opened probe before selecting cJTAG and attaching. The mask uses bit `n` for
/// scan format `n`.
pub fn set_cjtag_scan_formats(
    probe: &mut Probe,
    formats: u32,
) -> Result<VendorStatus, DebugProbeError> {
    let mut request = vec![4];
    request.extend_from_slice(&formats.to_le_bytes());
    let response = probe.send_vendor_command(CJTAG_CONTROL_COMMAND, &request)?;
    Ok(VendorStatus::from_byte(response_byte(&response, 0)?))
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

type VendorCommandSender<'a> =
    dyn for<'b> FnMut(u8, &'b [u8]) -> Result<Vec<u8>, DebugProbeError> + 'a;

fn read_info(
    send_vendor_command: &mut VendorCommandSender<'_>,
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
    let mut fields = 3 + build_len;
    let board = if version >= 3 {
        let board = response_byte(rest, fields)?;
        fields += 1;
        Some(BoardKind::from_byte(board))
    } else {
        None
    };
    let after_board = rest
        .get(fields..)
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
        board,
        scan_formats,
        extended_capabilities,
    }))
}

fn check_dap_status(response: &[u8]) -> Result<(), DebugProbeError> {
    check_vendor_status(response_byte(response, 0)?)
}

struct ResponseReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> ResponseReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.offset)
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], DebugProbeError> {
        let value = response_array(self.bytes, self.offset..self.offset + N)?;
        self.offset += N;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, DebugProbeError> {
        Ok(self.take::<1>()?[0])
    }

    fn u16(&mut self) -> Result<u16, DebugProbeError> {
        Ok(u16::from_le_bytes(self.take()?))
    }

    fn u32(&mut self) -> Result<u32, DebugProbeError> {
        Ok(u32::from_le_bytes(self.take()?))
    }

    fn i32(&mut self) -> Result<i32, DebugProbeError> {
        Ok(i32::from_le_bytes(self.take()?))
    }

    fn u64(&mut self) -> Result<u64, DebugProbeError> {
        Ok(u64::from_le_bytes(self.take()?))
    }
}

fn read_bool(reader: &mut ResponseReader<'_>, field: &str) -> Result<bool, DebugProbeError> {
    match reader.u8()? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(protocol_error(&format!("invalid {field}"))),
    }
}

fn read_bool_at(response: &[u8], index: usize, field: &str) -> Result<bool, DebugProbeError> {
    match response_byte(response, index)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(protocol_error(&format!("invalid {field}"))),
    }
}

fn read_codegen_measurement(
    reader: &mut ResponseReader<'_>,
    has_wire: bool,
) -> Result<CodegenMeasurement, DebugProbeError> {
    let automatic = read_bool(reader, "automatic codegen flag")?;
    let outcome = LatencyOutcome::from_byte(reader.u8()?);
    let wire = if has_wire { Some(reader.u8()?) } else { None };
    let latency = reader.u8()?;
    let attempts = reader.u8()?;
    let periods = reader.u8()?;
    let id_code = reader.u32()?;
    let cycles = reader.u32()?;
    let runs = reader.u32()?;
    let consistent_latencies = reader.u64()?;
    let rescues = reader.u32()?;
    Ok(CodegenMeasurement {
        automatic,
        outcome,
        wire: wire.map(JtagWire::from_byte),
        latency,
        attempts,
        periods,
        id_code,
        cycles,
        runs,
        consistent_latencies,
        rescues,
    })
}

fn check_vendor_status(status: u8) -> Result<(), DebugProbeError> {
    match VendorStatus::from_byte(status) {
        VendorStatus::Ok => Ok(()),
        VendorStatus::Error => Err(protocol_error("vendor command refused the request")),
        VendorStatus::Unknown(_) => Err(protocol_error("invalid vendor command status")),
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
