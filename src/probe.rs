//! Host operations for MCU-Link probes running the Verge firmware.

use probe_rs::{
    Permissions, Session,
    probe::{
        DebugProbe, DebugProbeError, DebugProbeInfo, Probe, ProbeTransport, UsbProbeInfo,
        WireProtocol,
        cmsisdap::{CmsisDap, ScanFormat, VendorCommandId},
        list::Lister,
    },
};
use std::any::Any;
use std::convert::Infallible;
use std::fmt::Debug;
use std::time::{Duration, Instant};
use thiserror::Error;

use crate::protocol::{
    self, BUS_REPLY_HEADER, CAP_CAPTURE_ID, CAP_DIO, CAP_DIO_STREAM, CAP_ENERGY_ACCUMULATOR,
    CAP_ENERGY_CAPTURE, CAP_I2C, CAP_JTAG_CODEGEN, CAP_MARKS, CAP_SPI, CAP_SWD_CODEGEN,
    CAP_TARGET_POWER, CAP_TARGET_SENSE, CODE_LAB_COMMAND, CPU_STAGING_COMMAND, DIO_COMMAND,
    ENERGY_COMMAND, I2C_COMMAND, I2C_MAX_CLOCK_HZ, I2C_MIN_CLOCK_HZ, I2C_REQUEST_HEADER,
    INFO_COMMAND, J8_COMMAND, JTAG_CODEGEN_COMMAND, JTAG_LOG_COMMAND, JTAG_PACING_COMMAND,
    LARGEST_PACKET, MARK_COMMAND, SAMPLE_COMMAND, SAMPLE_FILL_MAX, SPI_COMMAND, SPI_MAX_CLOCK_HZ,
    SPI_MIN_CLOCK_HZ, SPI_REQUEST_HEADER, SWD_CODEGEN_COMMAND, TARGET_POWER_COMMAND,
    TARGET_SENSE_COMMAND,
};
pub use crate::protocol::{BoardKind, DioState, EnergyRanges, VendorStatus, VergeInfo};

use crate::dap::{DapLink, ID_DAP_INVALID};

/// A failure from a typed Verge command.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum CommandError<E: std::error::Error + 'static> {
    /// The probe or its USB connection failed.
    #[error(transparent)]
    Probe(#[from] DebugProbeError),
    /// The probe's firmware does not offer the command.
    #[error("this probe's firmware does not support {0}")]
    Unsupported(&'static str),
    /// The request is outside what the command supports. The command itself was not sent.
    #[error("invalid request: {0}")]
    InvalidRequest(&'static str),
    /// The reply does not follow the command's protocol.
    #[error("malformed reply: {0}")]
    Malformed(String),
    /// The firmware refused or failed the command.
    #[error(transparent)]
    Firmware(E),
}

impl<E: std::error::Error + 'static> CommandError<E> {
    pub(crate) fn widen(error: CommandError<Infallible>) -> Self {
        match error {
            CommandError::Probe(error) => Self::Probe(error),
            CommandError::Unsupported(command) => Self::Unsupported(command),
            CommandError::InvalidRequest(reason) => Self::InvalidRequest(reason),
            CommandError::Malformed(reason) => Self::Malformed(reason),
            CommandError::Firmware(never) => match never {},
        }
    }
}

/// The firmware refused a request. It carries the state the firmware reported with the refusal.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[error("the firmware refused the request")]
pub struct Refused<S: Debug = ()>(pub S);

/// Where a marker went (mculink-verge docs/host-interface.md, "Markers: 0x94").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Marked {
    /// It is in the energy stream.
    Sent,
    /// No capture runs.
    NoCapture,
    /// A capture runs, but no host streams it.
    NoStream,
    /// The probe had no room for it. The stream's next markers message says one was lost.
    Full,
}

/// Why a target-sense conversion failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum TargetSenseError {
    /// The ADC's offset calibration failed.
    #[error("ADC offset calibration failed")]
    OffsetCalibration,
    /// The ADC's gain calibration failed.
    #[error("ADC gain calibration failed")]
    GainCalibration,
    /// The conversion produced no result.
    #[error("ADC conversion produced no result")]
    NoResult,
    /// A failure code added by newer firmware.
    #[error("unknown target-sense failure {0}")]
    Unknown(u16),
}

impl TargetSenseError {
    fn from_code(code: u16) -> Self {
        match code {
            1 => Self::OffsetCalibration,
            2 => Self::GainCalibration,
            3 => Self::NoResult,
            other => Self::Unknown(other),
        }
    }
}

/// A firmware failure from an energy-capture command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum EnergyError {
    #[error("the capture identity no longer matches; the current capture was left unchanged")]
    StaleCapture,
    #[error("the probe could not establish a capture identity")]
    IdentityUnavailable,
    /// Capture has not started or is stopped.
    #[error("capture is stopped")]
    Stopped,
    /// Capture has started but has no valid pair yet.
    #[error("capture has no sample yet")]
    NoSample,
    /// The operation is unavailable.
    #[error("operation is unavailable")]
    Unavailable,
    /// The requested rate is unsupported.
    #[error("unsupported capture rate")]
    BadRate,
    /// The request has an invalid argument.
    #[error("invalid capture argument")]
    BadArgument,
    /// ADC calibration failed.
    #[error("ADC calibration failed")]
    AdcError,
    /// Acquisition failed: a DMA error, an ADC FIFO overflow, or a segment of the ring the probe
    /// took too late.
    #[error("energy acquisition failed")]
    DmaError,
    /// An unrecognized result code.
    #[error("unknown firmware status {0:#04x}")]
    Unknown(u8),
}

impl EnergyError {
    fn check(value: u8) -> Result<(), Self> {
        Err(match value {
            0 => return Ok(()),
            1 => Self::Stopped,
            2 => Self::NoSample,
            3 => Self::Unavailable,
            4 => Self::BadRate,
            5 => Self::BadArgument,
            6 => Self::AdcError,
            7 => Self::DmaError,
            8 => Self::StaleCapture,
            9 => Self::IdentityUnavailable,
            other => Self::Unknown(other),
        })
    }
}

/// A firmware failure from a Pro SPI or I2C transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum ProBusError {
    /// The bridge is unavailable on this board or with the current jumper setting.
    #[error("operation is unavailable")]
    Unavailable,
    /// The request arguments are not supported.
    #[error("invalid bridge arguments")]
    Invalid,
    /// Another firmware feature currently owns the bus pins.
    #[error("bridge pins are busy")]
    Busy,
    /// The peripheral transfer failed.
    #[error("peripheral transfer failed")]
    TransferError,
    /// The I2C target did not acknowledge.
    #[error("I2C target did not acknowledge")]
    Nack,
    /// An unrecognized result code.
    #[error("unknown firmware status {0:#04x}")]
    Unknown(u8),
}

impl ProBusError {
    fn check(value: u8) -> Result<(), Self> {
        Err(match value {
            0 => return Ok(()),
            1 => Self::Unavailable,
            2 => Self::Invalid,
            3 => Self::Busy,
            4 => Self::TransferError,
            5 => Self::Nack,
            other => Self::Unknown(other),
        })
    }
}

/// SPI clock polarity and phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0.
    Mode0,
    /// CPOL=0, CPHA=1.
    Mode1,
    /// CPOL=1, CPHA=0.
    Mode2,
    /// CPOL=1, CPHA=1.
    Mode3,
}

impl SpiMode {
    const fn bits(self) -> u8 {
        match self {
            Self::Mode0 => 0,
            Self::Mode1 => 1 << 1,
            Self::Mode2 => 1,
            Self::Mode3 => (1 << 1) | 1,
        }
    }
}

/// Why an energy monitor stopped.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum EnergyMonitorError {
    #[error("energy monitor poll interval must be greater than zero")]
    InvalidPollInterval,
    #[error("starting energy capture failed: {0}")]
    Start(#[source] CommandError<EnergyError>),
    #[error("starting energy capture failed: {start}; stopping it failed: {stop}")]
    StartAndStop {
        start: CommandError<EnergyError>,
        stop: CommandError<EnergyError>,
    },
    #[error("energy sample poll failed: {0}")]
    Poll(#[source] CommandError<EnergyError>),
    #[error("energy poll failed: {poll}; stopping capture failed: {stop}")]
    PollAndStop {
        poll: CommandError<EnergyError>,
        stop: CommandError<EnergyError>,
    },
    #[error("stopping energy capture failed: {0}")]
    Stop(#[source] CommandError<EnergyError>),
}

/// One raw voltage/current pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
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
#[non_exhaustive]
pub struct CalibratedEnergySample {
    /// Target voltage in millivolts.
    pub voltage_millivolts: u32,
    /// Target current in nanoamps, positive into the target.
    pub current_nanoamps: i32,
    /// Per-sample current-sense path, 0 through 4: [`CalibratedEnergySample::sense`] names it.
    pub range_id: u8,
}

impl CalibratedEnergySample {
    /// The current range the pair was read on, and the range jumper's setting: `None` for a
    /// range ID this library does not know.
    #[must_use]
    pub fn sense(&self) -> Option<SensePath> {
        SensePath::from_id(self.range_id)
    }
}

/// Where a pair's current was read: on the low or the high range, in the 350 mA or the 50 mA
/// setting of the range jumper (host-interface.md, "The range is 1 for the low range in the
/// 350 mA setting").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct SensePath {
    pub range: crate::stream::CurrentRange,
    /// The range jumper's setting, 350 or 50 mA.
    pub setting_ma: u16,
}

impl SensePath {
    /// The path a firmware range ID names: 1 to 4.
    #[must_use]
    pub fn from_id(id: u8) -> Option<SensePath> {
        use crate::stream::CurrentRange::{High, Low};
        let (range, setting_ma) = match id {
            1 => (Low, 350),
            2 => (High, 350),
            3 => (Low, 50),
            4 => (High, 50),
            _ => return None,
        };
        Some(SensePath { range, setting_ma })
    }
}

impl std::fmt::Display for SensePath {
    /// As "low range, 350 mA setting".
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let range = match self.range {
            crate::stream::CurrentRange::Low => "low",
            crate::stream::CurrentRange::High => "high",
        };
        write!(f, "{range} range, {} mA setting", self.setting_ma)
    }
}

/// What [`VergeControl::stop_running_energy_capture`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EnergyStop {
    /// A capture ran, and stopped.
    Stopped,
    /// A fault had stopped the capture already.
    AlreadyFailed,
    /// No capture ran.
    NotRunning,
}

/// Result of starting an energy capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct EnergyCaptureStart {
    /// Effective voltage/current pairs per second.
    pub actual_rate_pairs_per_second: u32,
    pub capture_id: Option<crate::CaptureId>,
}

/// What [`VergeControl::energy_capture_status`] says of the probe's capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct EnergyCaptureStatus {
    pub running: bool,
    /// The running capture's rate; 0 with none.
    pub actual_rate_pairs_per_second: u32,
    /// The running capture's identity, or the last one's.
    pub capture_id: Option<crate::CaptureId>,
    /// Why no capture runs: [`EnergyError::Stopped`], or the fault that stopped the last one.
    /// `None` while one runs.
    pub result: Option<EnergyError>,
}

/// One complete voltage/current pair with optional calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct EnergySample {
    pub raw: RawEnergySample,
    pub calibrated: Option<CalibratedEnergySample>,
}

/// The latest energy reading, including normal states without a sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EnergyCaptureSample {
    Sample(EnergySample),
    NoSample,
    Stopped,
}

/// Accumulated energy since the last successful capture start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct EnergyAccumulator {
    /// Accumulated energy in nanojoules.
    pub energy_nanojoules: u64,
    /// Number of valid calibrated pairs included in the total.
    pub integrated_pairs: u64,
}

/// The energy accumulator's reading, including the states without a total.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EnergyTotal {
    /// The total since the last successful capture start.
    Total(EnergyAccumulator),
    /// The firmware's acquisition fault is set: a DMA error, an ADC FIFO overflow or a ring
    /// segment taken too late during a capture, or a start that could not take over the ADC. The total is what was integrated
    /// before the fault, which after a fault mid-capture leaves out the samples lost to it, and is
    /// zero when no capture ever started. A successful start clears the fault and the total.
    Failed(EnergyAccumulator),
    /// Capture has started but has not integrated a calibrated pair yet.
    NoSample,
    /// No capture has started since the probe booted.
    Stopped,
}

/// A current range the energy circuit calibrates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationRange {
    /// The sense resistor alone, for small currents.
    Low = 0,
    /// A second sense resistor in parallel, for large currents.
    High = 1,
}

/// One calibration point: an ADC code and the current through the sense resistor at that code.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct CalibrationPoint {
    pub code: u16,
    pub current_nanoamps: u32,
}

/// A range's calibration points, lowest first.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum EnergyCalibration {
    /// The latest calibration succeeded.
    Calibrated(Vec<CalibrationPoint>),
    /// The latest calibration failed, with the points it measured before failing, or none ran
    /// because the jumper setting could not be read.
    Failed(Vec<CalibrationPoint>),
}

/// One target-sense conversion returned by vendor command `0x86`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct TargetSenseReading {
    /// Raw ADC code.
    pub code: u16,
    /// Millivolts at the selected pin.
    pub millivolts: u16,
    /// Whether the target-detect line is grounded.
    pub detected: bool,
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
#[non_exhaustive]
pub struct SwdCodegenState {
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
#[non_exhaustive]
pub struct JtagCodegenState {
    /// cJTAG latency used by generated paths.
    pub two_pin_latency: u8,
    /// JTAG latency used by generated paths.
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
    /// JTAG.
    FourWire,
    /// cJTAG OScan1.
    OScan1,
    /// cJTAG OScan2.
    OScan2,
    /// cJTAG OScan5.
    OScan5,
    /// cJTAG OScan6.
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
    /// No latency agrees with every read, even with each claimed range a cycle uncertain.
    Inconsistent,
    /// No latency agrees with every read, but some do with each claimed range a cycle uncertain, as
    /// when the latency sits on a cycle boundary or one read was disturbed. A latency with a cycle
    /// to spare past those was selected.
    Boundary,
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
            6 => Self::Boundary,
            other => Self::Unknown(other),
        }
    }
}

/// Result of the latest latency sweep reported by a code generator.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct CodegenMeasurement {
    /// Whether future configurations trigger automatic measurements.
    pub automatic: bool,
    /// Outcome reported by the latency search.
    pub outcome: LatencyOutcome,
    /// Wire measured by JTAG codegen. SWD measurements do not set this field.
    pub wire: Option<JtagWire>,
    /// Latency the measurement found, or 0 when it kept the one in use. cJTAG builds sample
    /// one cycle later than this.
    pub latency: u8,
    /// Generations that ran reads, over every search: the firmware searches again, up to three
    /// times, while a search ends [`LatencyOutcome::Inconsistent`].
    pub attempts: u8,
    /// Read periods measured at, over every search.
    pub periods: u8,
    /// DPIDR or IDCODE value that read correctly.
    pub id_code: u32,
    /// Core cycles spent in the sweep.
    pub cycles: u32,
    /// Measurements since the probe started.
    pub runs: u32,
    /// Bit mask of latencies every read agreed with.
    pub consistent_latencies: u64,
    /// Failed connections recovered by the sweep.
    pub rescues: u32,
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
#[non_exhaustive]
pub struct SampleResult {
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
#[non_exhaustive]
pub struct CodeLabArena {
    /// Arena base address in probe memory.
    pub base_address: u32,
    /// Arena size in bytes.
    pub size: u32,
}

/// State of JTAG pacing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct JtagPacingState {
    /// Whether learned pacing is enabled.
    pub enabled: bool,
    /// Learned idle clocks currently added by the generator.
    pub paced_clocks: u8,
}

/// One JTAG code-generation log entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
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

/// A CMSIS-DAP target that accepts Verge vendor commands.
pub trait VergeTransport {
    /// Whether commands use Verge Aux rather than CMSIS-DAP.
    fn is_aux(&self) -> bool {
        false
    }

    /// The CMSIS-DAP packet size, which bounds both a command and its reply, when the transport
    /// can tell.
    fn packet_size(&mut self) -> Result<Option<usize>, DebugProbeError> {
        Ok(None)
    }

    /// Sends one CMSIS-DAP vendor command and returns its reply.
    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError>;
}

impl VergeTransport for Probe {
    fn packet_size(&mut self) -> Result<Option<usize>, DebugProbeError> {
        Ok(cmsis_dap(Some(self.driver_mut())).map(|dap| dap.packet_size()))
    }

    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        cmsis_dap(Some(self.driver_mut()))
            .ok_or(NOT_CMSIS_DAP)?
            .send_vendor_command(vendor_id(command_id)?, payload)
    }
}

/// Commands reach the probe after everything the session did before them.
impl VergeTransport for Session {
    fn packet_size(&mut self) -> Result<Option<usize>, DebugProbeError> {
        Ok(cmsis_dap(self.driver_mut()).map(|dap| dap.packet_size()))
    }

    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        cmsis_dap(self.driver_mut())
            .ok_or(NOT_CMSIS_DAP)?
            .send_vendor_command(vendor_id(command_id)?, payload)
    }
}

/// The probe's CMSIS-DAP driver, where it has one: no other driver carries vendor commands.
fn cmsis_dap(driver: Option<&mut dyn DebugProbe>) -> Option<&mut CmsisDap> {
    (driver? as &mut dyn Any).downcast_mut::<CmsisDap>()
}

const NOT_CMSIS_DAP: DebugProbeError = DebugProbeError::CommandNotSupportedByProbe {
    command_name: "CMSIS-DAP vendor commands",
};

/// probe-rs's form of a vendor command ID. Every one of Verge's is in the vendor range.
fn vendor_id(command_id: u8) -> Result<VendorCommandId, DebugProbeError> {
    VendorCommandId::new(command_id).ok_or_else(|| {
        DebugProbeError::Other(format!(
            "{command_id:#04x} is not a CMSIS-DAP vendor command"
        ))
    })
}

/// Vendor commands over a [`DapLink`], the link verge's own commands open. Replies come back as
/// probe-rs gives them: without the command byte, and a lone `ID_DAP_Invalid` as a command the
/// firmware does not know.
pub struct DapTransport<'a>(pub &'a mut dyn DapLink);

impl VergeTransport for DapTransport<'_> {
    fn packet_size(&mut self) -> Result<Option<usize>, DebugProbeError> {
        match crate::dap::info(self.0, crate::dap::Info::PacketSize) {
            Ok(size) => Ok(size
                .get(..2)
                .map(|size| usize::from(u16::from_le_bytes([size[0], size[1]])))),
            Err(error) => {
                self.resynchronise();
                Err(DebugProbeError::Other(error.to_string()))
            }
        }
    }

    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        if !(0x80..=0x9F).contains(&command_id) {
            return Err(DebugProbeError::Other(format!(
                "0x{command_id:02x} is not a vendor command"
            )));
        }
        let request = [&[command_id][..], payload].concat();
        let reply = match self.0.exchange(&request) {
            Ok(reply) => reply,
            Err(error) => {
                self.resynchronise();
                return Err(DebugProbeError::Other(error.to_string()));
            }
        };
        match reply.split_first() {
            Some((&id, rest)) if id == command_id => Ok(rest.to_vec()),
            Some((&ID_DAP_INVALID, _)) => Err(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "vendor command",
            }),
            other => {
                self.resynchronise();
                Err(DebugProbeError::Other(format!(
                    "asked for command 0x{command_id:02x} and the reply was {}: the probe is out of step",
                    other.map_or_else(|| "empty".to_owned(), |(id, _)| format!("0x{id:02x}"))
                )))
            }
        }
    }
}

impl DapTransport<'_> {
    /// Takes off the link any reply still owed after a failure. Left there, it would answer the
    /// next command in its place, and as every 0x90 mode shares one command byte, nothing would
    /// notice. A drain takes the replies already on their way. The probe answers in order, so a
    /// reply later still comes before the answer to a `DAP_Info` sent after it: one that is not
    /// that answer is stale, and the answer follows it.
    fn resynchronise(&mut self) {
        let _ = self.0.drain();
        if let Ok(reply) = self.0.exchange(&[crate::dap::DAP_INFO, 0xFE])
            && reply.first() != Some(&crate::dap::DAP_INFO)
        {
            let _ = self.0.drain();
        }
    }
}

enum InfoCache {
    Unread,
    NotVerge,
    Verge(VergeInfo),
}

impl InfoCache {
    fn info(&self) -> Option<&VergeInfo> {
        match self {
            Self::Verge(info) => Some(info),
            _ => None,
        }
    }
}

/// Verge firmware operations on an opened probe or an attached probe-rs session.
pub struct VergeControl<'a, T: VergeTransport + ?Sized> {
    transport: &'a mut T,
    info: InfoCache,
    packet_size: Option<usize>,
    capture_id: Option<crate::CaptureId>,
    awaiting_header: bool,
}

/// Verge commands sent through an opened probe, before attaching to its target.
///
/// Hand it only a probe that [`is_mcu_link`] accepts. Other firmware reads Verge's vendor commands
/// as its own: DAPLink's `0x82` sets its UART.
pub type VergeProbe<'a> = VergeControl<'a, Probe>;

/// Verge commands sent through an attached probe-rs session.
///
/// Hand it only a probe that [`is_mcu_link`] accepts. Other firmware reads Verge's vendor commands
/// as its own: DAPLink's `0x82` sets its UART.
pub type VergeSession<'a> = VergeControl<'a, Session>;

impl<'a, T: VergeTransport + ?Sized> VergeControl<'a, T> {
    /// Creates a Verge command client for an opened probe or attached session.
    pub fn new(transport: &'a mut T) -> Self {
        Self {
            transport,
            info: InfoCache::Unread,
            packet_size: None,
            capture_id: None,
            awaiting_header: false,
        }
    }

    /// The transport the commands go through: an [`crate::verge_aux::Aux`] for its stream messages.
    pub fn transport(&mut self) -> &mut T {
        self.transport
    }

    /// Reads the firmware identity and capabilities, once.
    ///
    /// The result is cached. Its cJTAG framing flag changes when a cJTAG session starts or ends;
    /// [`Self::refresh_info`] reads it again.
    pub fn info(&mut self) -> Result<Option<&VergeInfo>, CommandError<Infallible>> {
        if matches!(self.info, InfoCache::Unread) {
            self.refresh_info()?;
        }
        Ok(self.info.info())
    }

    /// Reads firmware identity and capabilities again, replacing the cached values.
    pub fn refresh_info(&mut self) -> Result<Option<&VergeInfo>, CommandError<Infallible>> {
        let info = read_info(&mut |command_id, payload| {
            self.transport.send_vendor_command(command_id, payload)
        })?;
        self.info = match info {
            Some(info) => InfoCache::Verge(info),
            None => InfoCache::NotVerge,
        };
        Ok(self.info.info())
    }

    /// Sends a Verge vendor command directly for firmware features without a typed wrapper.
    pub fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        self.transport.send_vendor_command(command_id, payload)
    }

    /// Reads the target supply voltage and target-detect line.
    pub fn read_target_sense(
        &mut self,
    ) -> Result<TargetSenseReading, CommandError<TargetSenseError>> {
        self.require_flag(CAP_TARGET_SENSE, "target sensing")?;
        self.read_target_sense_mode(0, 0)
    }

    /// Converts an ADC0 channel. Channel uses bits 0 through 4; bit 7 selects side B.
    pub fn convert_target_sense_channel(
        &mut self,
        channel: u8,
    ) -> Result<TargetSenseReading, CommandError<TargetSenseError>> {
        self.require_flag(CAP_TARGET_SENSE, "target sensing")?;
        if channel & 0x60 != 0 {
            return Err(CommandError::InvalidRequest(
                "a target-sense channel uses bits 0 to 4 and 7",
            ));
        }
        self.read_target_sense_mode(1, channel)
    }

    /// Switches the target supply and returns whether it is on.
    pub fn set_target_power(&mut self, enabled: bool) -> Result<bool, CommandError<Refused<bool>>> {
        self.require_flag(CAP_TARGET_POWER, "target power control")?;
        self.target_power_command(u8::from(enabled))
    }

    /// Reads whether the target supply is on, as last set since the probe booted.
    pub fn read_target_power(&mut self) -> Result<bool, CommandError<Refused<bool>>> {
        self.require_flag(CAP_TARGET_POWER, "target power control")?;
        self.target_power_command(2)
    }

    /// Reads DIO state, or updates the direction and levels for the selected pins.
    ///
    /// A set bit in `mask` selects a pin. Matching bits in `outputs` set output direction,
    /// and matching bits in `levels` set the output latch. A zero mask only reads state. A
    /// refused update, such as one that would split a direction group, leaves the pins as the
    /// refusal's state reports them.
    pub fn access_dio(
        &mut self,
        mask: u8,
        outputs: u8,
        levels: u8,
    ) -> Result<DioState, CommandError<Refused<DioState>>> {
        self.require_flag(CAP_DIO, "the DIO pins")?;
        if self.transport.is_aux() {
            self.require_capability(
                protocol::CAP_DIO_AUX,
                "DIO on Verge Aux: update the firmware first",
            )?;
        }
        let response = self.send(DIO_COMMAND, &protocol::encode_dio(mask, outputs, levels))?;
        let state = protocol::decode_dio(&response).map_err(malformed)?;
        match state.status {
            VendorStatus::Ok => Ok(state),
            VendorStatus::Error => Err(CommandError::Firmware(Refused(state))),
            VendorStatus::Unknown(_) => Err(malformed("invalid DIO status").into()),
        }
    }

    /// Runs one full-duplex transaction on the Pro SPI bridge.
    ///
    /// The request must fit the probe's packet: 504 bytes on bulk at high speed, 56 on HID and at
    /// full speed. A transport that cannot tell the packet is held to the high-speed limit.
    pub fn spi_transfer(
        &mut self,
        clock_hz: u32,
        mode: SpiMode,
        lsb_first: bool,
        tx: &[u8],
    ) -> Result<Vec<u8>, CommandError<ProBusError>> {
        self.require_capability(CAP_SPI, "the SPI bridge")?;
        let packet = self.packet_size()?;
        let request = encode_spi_request(clock_hz, mode, lsb_first, tx, packet)
            .map_err(CommandError::InvalidRequest)?;
        parse_bus_response(&self.send(SPI_COMMAND, &request)?, tx.len())
    }

    /// Runs one Pro I2C write, read, or combined write/read transaction.
    ///
    /// The request and the reply must fit the probe's packet: 502 bytes written or 508 read on bulk
    /// at high speed, 54 or 60 on HID and at full speed. A transport that cannot tell the packet is
    /// held to the high-speed limits.
    pub fn i2c_transfer(
        &mut self,
        address: u8,
        clock_hz: u32,
        write: &[u8],
        read_len: usize,
    ) -> Result<Vec<u8>, CommandError<ProBusError>> {
        self.require_capability(CAP_I2C, "the I2C bridge")?;
        let packet = self.packet_size()?;
        let request = encode_i2c_request(address, clock_hz, write, read_len, packet)
            .map_err(CommandError::InvalidRequest)?;
        parse_bus_response(&self.send(I2C_COMMAND, &request)?, read_len)
    }

    /// Sets the current ranges the next capture's start uses (`0x90` mode 14). That start takes
    /// the setting and later ones switch automatically again, so a capture that should hold the
    /// high range sets it before its own start. A running capture keeps its own. Firmware without
    /// the setting switches automatically, so asking it for automatic succeeds and asking it for
    /// the high range does not.
    pub fn set_energy_ranges(
        &mut self,
        ranges: EnergyRanges,
    ) -> Result<(), CommandError<EnergyError>> {
        let info = self
            .info()
            .map_err(CommandError::widen)?
            .ok_or(CommandError::Unsupported("Verge commands"))?;
        if !info.supports_energy_ranges() {
            return match ranges {
                EnergyRanges::Auto => Ok(()),
                EnergyRanges::High => Err(CommandError::Unsupported(
                    "holding the high range; update the firmware first",
                )),
            };
        }
        parse_energy_status(&self.send(ENERGY_COMMAND, &[14, ranges.wire()])?)
    }

    /// Starts voltage/current capture at the requested pair rate.
    pub fn start_energy_capture(
        &mut self,
        rate_pairs_per_second: u32,
    ) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let request = energy_capture_start_request(rate_pairs_per_second);
        parse_energy_capture_start(&self.send(ENERGY_COMMAND, &request)?)
    }

    /// Starts a capture at `rate` pairs a second, replacing any that runs, and keeps the identity
    /// the probe gives it (`0x90` mode 8), so the stop can name it
    /// ([`Self::stop_tracked_energy_capture`]).
    pub fn start_energy_capture_identified(
        &mut self,
        rate: u32,
    ) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
        self.start_identified(rate, None)
    }

    /// [`Self::start_energy_capture_identified`], streaming the capture through Verge Aux from its
    /// first pair with the edges of the DIO pins in `dio_mask` (mode 9). Its header comes after the
    /// reply. [`crate::stream::Reader::start_energy`] runs the whole stream.
    pub fn start_energy_stream_identified(
        &mut self,
        rate: u32,
        dio_mask: u8,
    ) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
        self.start_identified(rate, Some(dio_mask))
    }

    fn start_identified(
        &mut self,
        rate: u32,
        dio_mask: Option<u8>,
    ) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
        self.require_capability(
            CAP_CAPTURE_ID,
            "conditional capture stop; update the firmware first",
        )?;
        self.capture_id = None;
        self.awaiting_header = false;
        let mut request = vec![if dio_mask.is_some() { 9 } else { 8 }];
        request.extend_from_slice(&rate.to_le_bytes());
        if let Some(mask) = dio_mask {
            request.push(mask);
        }
        let reply = self.send(ENERGY_COMMAND, &request)?;
        let mut start = parse_energy_capture_start(&reply)?;
        start.capture_id = Some(parse_capture_id(
            reply
                .get(6..30)
                .ok_or_else(|| malformed("capture identity is missing"))?,
        )?);
        self.capture_id = start.capture_id;
        self.awaiting_header = dio_mask.is_some();
        Ok(start)
    }

    /// Streams the capture that runs from its next pair, with the edges of the DIO pins in
    /// `dio_mask`, and leaves it running (mode 10, Verge Aux only). The capture's identity comes in
    /// the stream's header: pass the header to [`Self::accept_energy_header`].
    pub fn stream_energy_capture_identified(
        &mut self,
        dio_mask: u8,
    ) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(
            CAP_CAPTURE_ID,
            "conditional capture stop; update the firmware first",
        )?;
        self.capture_id = None;
        self.awaiting_header = false;
        parse_energy_status(&self.send(ENERGY_COMMAND, &[10, dio_mask])?)?;
        self.awaiting_header = true;
        Ok(())
    }

    /// Takes the capture's identity from the header of a stream this side started or joined.
    /// Refuses a header that names another capture than the one started.
    pub fn accept_energy_header(
        &mut self,
        header: &crate::stream::Header,
    ) -> Result<(), CommandError<EnergyError>> {
        if !self.awaiting_header {
            return Err(CommandError::InvalidRequest(
                "no acknowledged stream is awaiting a header",
            ));
        }
        let id = header
            .capture_id
            .ok_or_else(|| malformed("the identified stream has no capture identity"))?;
        if self.capture_id.is_some_and(|expected| expected != id) {
            return Err(malformed("the stream header names a different capture").into());
        }
        self.capture_id = Some(id);
        self.awaiting_header = false;
        Ok(())
    }

    /// Whether a stream this side started or joined has yet to send its header.
    pub fn awaiting_energy_header(&self) -> bool {
        self.awaiting_header
    }

    /// The identity of the capture this side started or joined, once known.
    pub fn tracked_capture_id(&self) -> Option<crate::CaptureId> {
        self.capture_id
    }

    /// Stops the capture this side started or joined, if it still runs: a capture another host
    /// started meanwhile runs on.
    pub fn stop_tracked_energy_capture(&mut self) -> Result<(), CommandError<EnergyError>> {
        let id = self.capture_id.ok_or(CommandError::InvalidRequest(
            "capture identity is not known",
        ))?;
        self.stop_energy_capture_if(id)
    }

    /// Ends the stream of the capture this side joined, leaving the capture running.
    pub fn end_tracked_energy_stream(&mut self) -> Result<(), CommandError<EnergyError>> {
        let id = self.capture_id.ok_or(CommandError::InvalidRequest(
            "capture identity is not known",
        ))?;
        self.end_energy_stream_if(id)
    }

    /// Stops the capture `id` names, if it still runs (mode 11). Another capture is left alone,
    /// with [`EnergyError::StaleCapture`].
    pub fn stop_energy_capture_if(
        &mut self,
        id: crate::CaptureId,
    ) -> Result<(), CommandError<EnergyError>> {
        self.conditional_energy(11, id)
    }

    /// Ends the stream of the capture `id` names, leaving the capture running (mode 13). Another
    /// capture's stream is left alone, with [`EnergyError::StaleCapture`].
    pub fn end_energy_stream_if(
        &mut self,
        id: crate::CaptureId,
    ) -> Result<(), CommandError<EnergyError>> {
        self.conditional_energy(13, id)
    }

    fn conditional_energy(
        &mut self,
        mode: u8,
        id: crate::CaptureId,
    ) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(
            CAP_CAPTURE_ID,
            "conditional capture stop; update the firmware first",
        )?;
        let mut request = vec![mode];
        request.extend_from_slice(id.as_bytes());
        parse_energy_status(&self.send(ENERGY_COMMAND, &request)?)
    }

    /// Whether a capture runs, at what rate, under which identity, and how the last one ended
    /// (mode 12).
    pub fn energy_capture_status(
        &mut self,
    ) -> Result<EnergyCaptureStatus, CommandError<EnergyError>> {
        self.require_capability(
            CAP_CAPTURE_ID,
            "conditional capture stop; update the firmware first",
        )?;
        let reply = self.send(ENERGY_COMMAND, &[12])?;
        let mut reader = ResponseReader::new(&reply);
        energy_mode_known(reader.u8()?)?;
        let result = EnergyError::check(reader.u8()?).err();
        let running = match reader.u8()? {
            0 => false,
            1 => true,
            _ => return Err(malformed("capture running flag is not 0 or 1").into()),
        };
        let rate = reader.u32()?;
        if running != result.is_none() || (!running && rate != 0) {
            return Err(malformed("inconsistent capture state").into());
        }
        let bytes = reply
            .get(7..31)
            .ok_or_else(|| malformed("capture identity is missing"))?;
        let capture_id = if bytes.iter().all(|&b| b == 0) {
            None
        } else {
            Some(parse_capture_id(bytes)?)
        };
        if running && capture_id.is_none() {
            return Err(malformed("a running capture has no identity").into());
        }
        Ok(EnergyCaptureStatus {
            running,
            actual_rate_pairs_per_second: rate,
            capture_id,
            result,
        })
    }

    /// Polls distinct energy samples until `should_stop` returns true, then stops capture.
    /// Returns the capture's effective rate.
    ///
    /// Capture continues in probe firmware between polls. The poll interval controls host traffic;
    /// samples between polls remain included in the probe's energy accumulator. Between polls,
    /// `should_stop` is checked at least every 10 ms; `Duration::MAX` never polls. Capture is
    /// conditionally stopped after a successful identified start, including a panic in either
    /// callback. An uncertain start reply grants no identity and cannot be cleaned up automatically.
    pub fn monitor_energy<Stop, Sample>(
        &mut self,
        rate_pairs_per_second: u32,
        poll_interval: Duration,
        mut should_stop: Stop,
        mut on_sample: Sample,
    ) -> Result<EnergyCaptureStart, EnergyMonitorError>
    where
        Stop: FnMut() -> bool,
        Sample: FnMut(EnergySample),
    {
        if poll_interval.is_zero() {
            return Err(EnergyMonitorError::InvalidPollInterval);
        }
        self.require_capability::<EnergyError>(CAP_ENERGY_CAPTURE, "energy capture")
            .map_err(EnergyMonitorError::Start)?;
        let start = match self.start_energy_capture_identified(rate_pairs_per_second) {
            Ok(start) => start,
            Err(start) => {
                return Err(EnergyMonitorError::Start(start));
            }
        };

        let mut capture = StopOnDrop {
            control: self,
            armed: true,
        };
        let mut last_sequence = None;
        let poll_result = loop {
            if wait_or_stop(&mut should_stop, poll_interval) {
                break Ok(());
            }
            match capture.control.latest_energy_sample() {
                Ok(EnergyCaptureSample::Sample(sample)) => {
                    if last_sequence != Some(sample.raw.sequence) {
                        last_sequence = Some(sample.raw.sequence);
                        on_sample(sample);
                    }
                }
                Ok(EnergyCaptureSample::NoSample) => {}
                Ok(EnergyCaptureSample::Stopped) => {
                    break Err(CommandError::Firmware(EnergyError::Stopped));
                }
                Err(error) => break Err(error),
            }
        };

        capture.armed = false;
        let stop_result = capture.control.stop_tracked_energy_capture();
        match (poll_result, stop_result) {
            (Err(poll), Ok(())) => Err(EnergyMonitorError::Poll(poll)),
            (Err(poll), Err(stop)) => Err(EnergyMonitorError::PollAndStop { poll, stop }),
            (Ok(()), Err(error)) => Err(EnergyMonitorError::Stop(error)),
            (Ok(()), Ok(())) => Ok(start),
        }
    }

    /// Starts a capture as [`Self::start_energy_capture`] does, and streams it from its first pair
    /// with the edges of the DIO pins in `dio_mask`: `0x90` mode 5, which only Verge Aux carries.
    /// The reply comes before the stream's header, which says which pins report.
    pub fn start_energy_stream(
        &mut self,
        rate_pairs_per_second: u32,
        dio_mask: u8,
    ) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let mut request = vec![5];
        request.extend_from_slice(&rate_pairs_per_second.to_le_bytes());
        request.push(dio_mask);
        parse_energy_capture_start(&self.send(ENERGY_COMMAND, &request)?)
    }

    /// Streams the running capture from its next pair, with the edges of the DIO pins in
    /// `dio_mask`: mode 6, on Verge Aux only.
    /// [`EnergyError::Stopped`] when nothing runs, and [`EnergyError::DmaError`] when the capture
    /// failed or the probe had no room for the header.
    pub fn stream_energy_capture(&mut self, dio_mask: u8) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        parse_energy_status(&self.send(ENERGY_COMMAND, &[6, dio_mask])?)
    }

    /// Starts a DIO stream on a board without energy capture: the edges of the DIO pins in
    /// `dio_mask`, with no pairs, on Verge Aux only (`0x90` mode 15). Its first message is a
    /// [`crate::stream::DioHeader`], and its times count from it. A stream already running ends
    /// first.
    pub fn start_dio_stream(&mut self, dio_mask: u8) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(CAP_DIO_STREAM, "DIO streams")?;
        parse_energy_status(&self.send(ENERGY_COMMAND, &[15, dio_mask])?)
    }

    /// Ends a DIO stream: mode 7, on Verge Aux only. Its end message says when.
    pub fn end_dio_stream(&mut self) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(CAP_DIO_STREAM, "DIO streams")?;
        parse_energy_status(&self.send(ENERGY_COMMAND, &[7])?)
    }

    /// Ends the stream, leaving the capture running: mode 7, on Verge Aux only.
    pub fn end_energy_stream(&mut self) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        parse_energy_status(&self.send(ENERGY_COMMAND, &[7])?)
    }

    /// Stops whatever capture runs, by its identity, so a capture another host starts meanwhile
    /// runs on. Says what it found: a capture to stop, one a fault had stopped already, or none.
    pub fn stop_running_energy_capture(&mut self) -> Result<EnergyStop, CommandError<EnergyError>> {
        let status = self.energy_capture_status()?;
        if let Some(id) = status.capture_id {
            match self.stop_energy_capture_if(id) {
                // A fault stops a capture all the same, and the stop answers with it.
                Ok(()) | Err(CommandError::Firmware(EnergyError::DmaError)) => {}
                Err(error) => return Err(error),
            }
        }
        Ok(match (status.running, status.result) {
            (true, _) => EnergyStop::Stopped,
            (false, Some(EnergyError::DmaError)) => EnergyStop::AlreadyFailed,
            (false, _) => EnergyStop::NotRunning,
        })
    }

    /// Stops voltage/current capture.
    pub fn stop_energy_capture(&mut self) -> Result<(), CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let response = self.send(ENERGY_COMMAND, &[2])?;
        parse_energy_status(&response)
    }

    /// Marks this moment in the energy stream a host is recording, with `number` and a `label` of
    /// at most 32 bytes. The probe stamps it as it runs the command, so in a batch it falls
    /// between the commands around it. Only [`Marked::Sent`] puts it anywhere.
    pub fn mark(&mut self, number: u32, label: &[u8]) -> Result<Marked, CommandError<Refused>> {
        if label.len() > crate::stream::LABEL_MAX {
            return Err(CommandError::InvalidRequest(
                "a marker's label is at most 32 bytes",
            ));
        }
        self.require_capability(CAP_MARKS, "markers")?;
        let mut payload = number.to_le_bytes().to_vec();
        payload.push(label.len() as u8);
        payload.extend_from_slice(label);
        let response = self.send(MARK_COMMAND, &payload)?;
        if !vendor_status(response_byte(&response, 0)?)? {
            return Err(CommandError::Firmware(Refused(())));
        }
        match response_byte(&response, 1)? {
            0 => Ok(Marked::Sent),
            1 => Ok(Marked::NoCapture),
            2 => Ok(Marked::NoStream),
            3 => Ok(Marked::Full),
            other => Err(malformed(format!("marker result {other}")).into()),
        }
    }

    /// Reads the latest complete voltage/current pair.
    pub fn latest_energy_sample(
        &mut self,
    ) -> Result<EnergyCaptureSample, CommandError<EnergyError>> {
        let calibrated_supported = self
            .require_capability(CAP_ENERGY_CAPTURE, "energy capture")?
            .supports_calibrated_energy();
        let response = self.send(ENERGY_COMMAND, &[1])?;
        parse_energy_capture_sample(&response, calibrated_supported)
    }

    /// Reads the energy total without resetting it.
    pub fn read_energy_accumulator(&mut self) -> Result<EnergyTotal, CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_ACCUMULATOR, "the energy accumulator")?;
        let response = self.send(ENERGY_COMMAND, &[3])?;
        parse_energy_accumulator(&response)
    }

    /// Reads the calibration points the probe's latest calibration left for `range`.
    pub fn read_energy_calibration(
        &mut self,
        range: CalibrationRange,
    ) -> Result<EnergyCalibration, CommandError<EnergyError>> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let response = self.send(ENERGY_COMMAND, &[4, range as u8])?;
        parse_energy_calibration(&response)
    }

    /// Gives the standalone MCU-Link's J8 pins in `pins`, bit n DIOn, to the firmware's bench
    /// marker, which drives them high while each command runs; the rest go back to DIO as inputs.
    /// Zero, as at boot, gives all four to DIO. Returns the pins the marker holds. A bench command:
    /// a pin the marker holds is an output, whatever is wired to it.
    pub fn set_marker_pins(&mut self, pins: u8) -> Result<u8, CommandError<Refused>> {
        self.require_verge()?;
        let response = self.send(J8_COMMAND, &[pins & 0x0F])?;
        expect_accepted(&response)?;
        Ok(response_byte(&response, 1)?)
    }

    /// Runs the SWD sample-position instrument.
    pub fn sample_swd(
        &mut self,
        request: SampleRequest,
    ) -> Result<SampleResult, CommandError<Refused>> {
        let payload = encode_sample_request(request).map_err(CommandError::InvalidRequest)?;
        self.require_verge()?;
        let response = self.send(SAMPLE_COMMAND, &payload)?;
        expect_accepted(&response)?;
        let mut reader = ResponseReader::new(&response[1..]);
        Ok(SampleResult {
            good: reader.u16()?,
            bad_acknowledgements: reader.u16()?,
            bad_words: reader.u16()?,
            last_bad_acknowledgement: reader.u8()?,
            last_bad_dpidr: reader.u32()?,
            cycles: reader.u32()?,
        })
    }

    /// Returns the executable arenas exposed by a code-lab firmware build.
    pub fn code_lab_arenas(&mut self) -> Result<Vec<CodeLabArena>, CommandError<Refused>> {
        self.require_verge()?;
        let response = self.send(CODE_LAB_COMMAND, &[0])?;
        expect_accepted(&response)?;
        let mut reader = ResponseReader::new(&response[1..]);
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
    ) -> Result<(), CommandError<Refused>> {
        let length = u16::try_from(bytes.len())
            .map_err(|_| CommandError::InvalidRequest("a code-lab write is at most 65535 bytes"))?;
        self.require_verge()?;
        let mut payload = Vec::with_capacity(6 + bytes.len());
        payload.extend_from_slice(&[1, arena]);
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&length.to_le_bytes());
        payload.extend_from_slice(bytes);
        expect_accepted(&self.send(CODE_LAB_COMMAND, &payload)?)
    }

    /// Reads bytes from a code-lab arena.
    pub fn read_code_lab(
        &mut self,
        arena: u8,
        offset: u16,
        length: u16,
    ) -> Result<Vec<u8>, CommandError<Refused>> {
        self.require_verge()?;
        let mut payload = vec![2, arena];
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&length.to_le_bytes());
        let response = self.send(CODE_LAB_COMMAND, &payload)?;
        expect_accepted(&response)?;
        Ok(response
            .get(1..1 + usize::from(length))
            .map(<[u8]>::to_vec)
            .ok_or_else(|| malformed("short code-lab read response"))?)
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
    ) -> Result<[u32; 2], CommandError<Refused>> {
        self.require_verge()?;
        let mut payload = vec![3, arena];
        payload.extend_from_slice(&entry_offset.to_le_bytes());
        for argument in arguments {
            payload.extend_from_slice(&argument.to_le_bytes());
        }
        let response = self.send(CODE_LAB_COMMAND, &payload)?;
        expect_accepted(&response)?;
        Ok([
            u32::from_le_bytes(response_array(&response, 1..5)?),
            u32::from_le_bytes(response_array(&response, 5..9)?),
        ])
    }

    /// Reads or changes the SWD code generator's latency policy.
    ///
    /// A refusal can follow a change the firmware applied, such as a latency it set and pinned
    /// before refusing the measurement. The refusal's state is the generator after the request.
    pub fn configure_swd_codegen(
        &mut self,
        latency: u8,
        mode: CodegenMode,
    ) -> Result<SwdCodegenState, CommandError<Refused<SwdCodegenState>>> {
        self.require_flag(CAP_SWD_CODEGEN, "SWD code generation")?;
        let response = self.send(SWD_CODEGEN_COMMAND, &[latency, mode as u8])?;
        let (accepted, state) = parse_swd_codegen_state(&response)?;
        refused(accepted, state)
    }

    /// Reads or changes the JTAG code generator's latency policy.
    ///
    /// As with [`Self::configure_swd_codegen`], a refusal can follow a change the firmware applied.
    pub fn configure_jtag_codegen(
        &mut self,
        two_pin_latency: u8,
        four_wire_latency: u8,
        mode: CodegenMode,
    ) -> Result<JtagCodegenState, CommandError<Refused<JtagCodegenState>>> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let response = self.send(
            JTAG_CODEGEN_COMMAND,
            &[two_pin_latency, four_wire_latency, mode as u8],
        )?;
        let (accepted, state) = parse_jtag_codegen_state(&response)?;
        refused(accepted, state)
    }

    /// Reads the current JTAG pacing state or enables/disables learned idle clocks.
    pub fn set_jtag_pacing(
        &mut self,
        enabled: Option<bool>,
    ) -> Result<JtagPacingState, CommandError<Refused<JtagPacingState>>> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let mode = enabled.map_or(2, u8::from);
        let response = self.send(JTAG_PACING_COMMAND, &[mode])?;
        let accepted = vendor_status(response_byte(&response, 0)?)?;
        let state = JtagPacingState {
            enabled: read_bool_at(&response, 1, "JTAG pacing state")?,
            paced_clocks: response_byte(&response, 2)?,
        };
        refused(accepted, state)
    }

    /// Reads or sets whether CPU 0 stages DAP commands for CPU 1, and returns whether it does.
    pub fn set_cpu_staging(
        &mut self,
        enabled: Option<bool>,
    ) -> Result<bool, CommandError<Refused<bool>>> {
        self.require_verge()?;
        let mode = enabled.map_or(2, u8::from);
        let response = self.send(CPU_STAGING_COMMAND, &[mode])?;
        let accepted = vendor_status(response_byte(&response, 0)?)?;
        refused(accepted, read_bool_at(&response, 1, "CPU staging state")?)
    }

    /// Reads one JTAG code-generation log entry, with zero selecting the newest entry.
    pub fn read_jtag_generation_log(
        &mut self,
        generations_back: u8,
    ) -> Result<Option<JtagGenerationLogEntry>, CommandError<Infallible>> {
        self.require_flag(CAP_JTAG_CODEGEN, "JTAG code generation")?;
        let response = self.send(JTAG_LOG_COMMAND, &[generations_back])?;
        let mut reader = ResponseReader::new(&response);
        if !vendor_status(reader.u8()?)? {
            return Ok(None);
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

    fn require_verge<E: std::error::Error + 'static>(&mut self) -> Result<(), CommandError<E>> {
        self.info()
            .map_err(CommandError::widen)?
            .ok_or(CommandError::Unsupported("Verge commands"))?;
        Ok(())
    }

    fn require_capability<E: std::error::Error + 'static>(
        &mut self,
        capability: u32,
        command_name: &'static str,
    ) -> Result<&VergeInfo, CommandError<E>> {
        let info = self
            .info()
            .map_err(CommandError::widen)?
            .ok_or(CommandError::Unsupported("Verge commands"))?;
        if info.extended_capabilities & capability == 0 {
            return Err(CommandError::Unsupported(command_name));
        }
        Ok(info)
    }

    fn require_flag<E: std::error::Error + 'static>(
        &mut self,
        capability: u8,
        command_name: &'static str,
    ) -> Result<(), CommandError<E>> {
        let info = self
            .info()
            .map_err(CommandError::widen)?
            .ok_or(CommandError::Unsupported("Verge commands"))?;
        if info.capabilities & capability == 0 {
            return Err(CommandError::Unsupported(command_name));
        }
        Ok(())
    }

    fn read_target_sense_mode(
        &mut self,
        mode: u8,
        channel: u8,
    ) -> Result<TargetSenseReading, CommandError<TargetSenseError>> {
        let response = self.send(TARGET_SENSE_COMMAND, &[mode, channel])?;
        let accepted = vendor_status(response_byte(&response, 0)?)?;
        let code = u16::from_le_bytes(response_array(&response, 1..3)?);
        let millivolts = u16::from_le_bytes(response_array(&response, 3..5)?);
        let detected = read_bool_at(&response, 5, "target-detect byte")?;
        if !accepted {
            // The firmware reports the failing step in the code field.
            return Err(CommandError::Firmware(TargetSenseError::from_code(code)));
        }
        Ok(TargetSenseReading {
            code,
            millivolts,
            detected,
        })
    }

    fn target_power_command(&mut self, mode: u8) -> Result<bool, CommandError<Refused<bool>>> {
        let response = self.send(TARGET_POWER_COMMAND, &[mode])?;
        let accepted = vendor_status(response_byte(&response, 0)?)?;
        refused(
            accepted,
            read_bool_at(&response, 1, "target-power state byte")?,
        )
    }

    /// The transport's packet, read once it has been read without error. A transport that cannot
    /// tell gets the largest: probe-rs then refuses a request longer than its packet, and the
    /// firmware a reply.
    fn packet_size<E: std::error::Error + 'static>(&mut self) -> Result<usize, CommandError<E>> {
        if let Some(size) = self.packet_size {
            return Ok(size);
        }
        let size = self.transport.packet_size()?.unwrap_or(LARGEST_PACKET);
        self.packet_size = Some(size);
        Ok(size)
    }

    fn send(&mut self, command_id: u8, payload: &[u8]) -> Result<Vec<u8>, SendError> {
        self.transport
            .send_vendor_command(command_id, payload)
            .map_err(|error| match error {
                // A build without the command answers it as unknown.
                DebugProbeError::CommandNotSupportedByProbe { .. } => {
                    SendError::Unsupported(command_name(command_id))
                }
                error => SendError::Probe(error),
            })
    }
}

/// Stops energy capture if dropped while armed, so an unwinding callback does not leave it running.
struct StopOnDrop<'c, 'a, T: VergeTransport + ?Sized> {
    control: &'c mut VergeControl<'a, T>,
    armed: bool,
}

impl<T: VergeTransport + ?Sized> Drop for StopOnDrop<'_, '_, T> {
    fn drop(&mut self) {
        if self.armed {
            // A transport that panics again here, on a poisoned lock for one, would abort the
            // process mid-unwind. The caller's panic is the one to report.
            let control = &mut *self.control;
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                control.stop_tracked_energy_capture()
            }));
        }
    }
}

/// Sleeps for `interval`, checking `should_stop` at least every 10 ms. Returns whether it asked to stop.
fn wait_or_stop(should_stop: &mut impl FnMut() -> bool, interval: Duration) -> bool {
    const CHECK: Duration = Duration::from_millis(10);
    let deadline = Instant::now().checked_add(interval);
    loop {
        if should_stop() {
            return true;
        }
        let left = deadline.map_or(CHECK, |deadline| {
            deadline.saturating_duration_since(Instant::now())
        });
        if left.is_zero() {
            return false;
        }
        std::thread::sleep(left.min(CHECK));
    }
}

impl VergeControl<'_, Session> {
    /// Returns the underlying probe-rs session for ordinary target operations.
    pub fn session(&mut self) -> &mut Session {
        self.transport
    }
}

/// Why [`attach`] could not open a session.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum AttachError {
    #[error("no probe with serial number {0} is attached")]
    NotFound(String),
    #[error("no probe is attached")]
    NoneAttached,
    #[error("{0} probes are attached; name one by its serial number")]
    Several(usize),
    #[error("{0} is not an MCU-Link, and cJTAG needs one running Verge")]
    NotMcuLink(String),
    #[error("the MCU-Link does not run Verge, and cJTAG needs it")]
    NotVerge,
    #[error(
        "the MCU-Link runs Verge build {0}, older than the cJTAG commands this version sends. Update it"
    )]
    VergeTooOld(String),
    #[error("an on-board MCU-Link has no cJTAG")]
    NoCjtag,
    #[error("opening the probe failed")]
    Open(#[source] DebugProbeError),
    #[error("setting up the probe failed")]
    Configure(#[source] DebugProbeError),
    #[error("attaching to {chip} failed")]
    Target {
        chip: String,
        #[source]
        source: probe_rs::Error,
    },
}

/// Whether `info` carries an MCU-Link's USB ID, so its firmware is NXP's or Verge. Both answer
/// `0x82` without acting on it, so Verge's vendor commands are safe to send it.
#[must_use]
pub fn is_mcu_link(info: &DebugProbeInfo) -> bool {
    let usb = usb(info);
    mcu_link_id(usb.vendor_id, usb.product_id)
}

fn usb(info: &DebugProbeInfo) -> &UsbProbeInfo {
    let ProbeTransport::Usb(usb) = &info.transport;
    usb
}

fn mcu_link_id(vid: u16, pid: u16) -> bool {
    crate::ids::role(vid, pid) == Some(crate::ids::Role::Probe)
}

/// Readies an opened probe for probe-rs to select `protocol`, as [`attach`] does. For cJTAG it
/// refuses a probe that is not an MCU-Link running Verge with cJTAG, and selects the scan format
/// the connection comes up in: `cjtag_format`, or OScan1 without one. Nothing checks that the
/// target has that format. For SWD and JTAG it sends nothing.
///
/// `usb_id` is the probe's VID and PID, from its listing or the selector that opened it.
pub fn prepare_probe(
    probe: &mut Probe,
    usb_id: (u16, u16),
    protocol: WireProtocol,
    cjtag_format: Option<ScanFormat>,
) -> Result<(), AttachError> {
    if protocol != WireProtocol::CJtag {
        return Ok(());
    }
    let (vid, pid) = usb_id;
    if !mcu_link_id(vid, pid) {
        return Err(AttachError::NotMcuLink(format!(
            "the probe with USB ID {vid:04x}:{pid:04x}"
        )));
    }
    require_cjtag(probe)?;
    let cmsisdap =
        cmsis_dap(Some(probe.driver_mut())).ok_or(AttachError::Configure(NOT_CMSIS_DAP))?;
    cmsisdap.set_cjtag_scan_format(cjtag_format.unwrap_or_default());
    Ok(())
}

/// On an MCU-Link, before probe-rs selects cJTAG: refuses one whose firmware has none, or whose
/// `0x95` does not select the next connect's format yet (info format 8).
fn require_cjtag<T: VergeTransport + ?Sized>(transport: &mut T) -> Result<(), AttachError> {
    let info = VergeControl::new(transport)
        .info()
        .map_err(|error| AttachError::Configure(into_probe_error(error)))?
        .cloned();
    match info {
        None => Err(AttachError::NotVerge),
        Some(info) if info.version < 8 => Err(AttachError::VergeTooOld(info.firmware)),
        Some(info) if info.scan_formats == 0 => Err(AttachError::NoCjtag),
        Some(_) => Ok(()),
    }
}

/// Opens the probe with `serial`, or the only one attached, and attaches to `chip` over
/// `protocol`.
///
/// [`prepare_probe`] readies it first, selecting `cjtag_format` for cJTAG, or OScan1. Without
/// `speed_khz` the probe keeps its default clock.
pub fn attach(
    serial: Option<&str>,
    chip: &str,
    protocol: WireProtocol,
    speed_khz: Option<u32>,
    cjtag_format: Option<ScanFormat>,
) -> Result<Session, AttachError> {
    let mut probes = Lister::new().list_all();
    let info = match serial {
        Some(serial) => probes
            .into_iter()
            .find(|probe| usb(probe).serial_number.as_deref() == Some(serial))
            .ok_or_else(|| AttachError::NotFound(serial.to_owned()))?,
        None => match probes.len() {
            0 => return Err(AttachError::NoneAttached),
            1 => probes.remove(0),
            several => return Err(AttachError::Several(several)),
        },
    };
    if protocol == WireProtocol::CJtag && !is_mcu_link(&info) {
        return Err(AttachError::NotMcuLink(info.identifier));
    }
    let mut probe = info.open().map_err(AttachError::Open)?;
    prepare_probe(
        &mut probe,
        (usb(&info).vendor_id, usb(&info).product_id),
        protocol,
        cjtag_format,
    )?;
    probe
        .select_protocol(protocol)
        .map_err(AttachError::Configure)?;
    if let Some(speed_khz) = speed_khz {
        probe.set_speed(speed_khz).map_err(AttachError::Configure)?;
    }
    probe
        .attach(chip, Permissions::default())
        .map_err(|source| AttachError::Target {
            chip: chip.to_owned(),
            source,
        })
}

type VendorCommandSender<'a> =
    dyn for<'b> FnMut(u8, &'b [u8]) -> Result<Vec<u8>, DebugProbeError> + 'a;

fn read_info(
    send_vendor_command: &mut VendorCommandSender<'_>,
) -> Result<Option<VergeInfo>, CommandError<Infallible>> {
    let reply = match send_vendor_command(INFO_COMMAND, &[]) {
        Ok(reply) => reply,
        // A probe without vendor commands, or firmware that answers 0x82 as unknown, is not Verge.
        Err(
            DebugProbeError::NotImplemented { .. }
            | DebugProbeError::CommandNotSupportedByProbe { .. },
        ) => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    Ok(protocol::decode_info(&reply).map_err(malformed)?)
}

fn into_probe_error(error: CommandError<Infallible>) -> DebugProbeError {
    match error {
        CommandError::Probe(error) => error,
        CommandError::Unsupported(command_name) => {
            DebugProbeError::CommandNotSupportedByProbe { command_name }
        }
        CommandError::InvalidRequest(reason) => DebugProbeError::Other(reason.to_owned()),
        CommandError::Malformed(reason) => DebugProbeError::Other(reason),
        CommandError::Firmware(never) => match never {},
    }
}

/// Whether a vendor reply's status byte accepts the request.
fn vendor_status(status: u8) -> Result<bool, MalformedReply> {
    match VendorStatus::from_byte(status) {
        VendorStatus::Ok => Ok(true),
        VendorStatus::Error => Ok(false),
        VendorStatus::Unknown(_) => Err(malformed("invalid vendor command status")),
    }
}

fn refused<S: Debug + 'static>(accepted: bool, state: S) -> Result<S, CommandError<Refused<S>>> {
    if accepted {
        Ok(state)
    } else {
        Err(CommandError::Firmware(Refused(state)))
    }
}

fn expect_accepted(response: &[u8]) -> Result<(), CommandError<Refused>> {
    refused(vendor_status(response_byte(response, 0)?)?, ())
}

/// The energy command answers DAP_ERROR only to a mode it does not know.
fn energy_mode_known(status: u8) -> Result<(), MalformedReply> {
    if vendor_status(status)? {
        Ok(())
    } else {
        Err(malformed("the firmware did not recognise the energy mode"))
    }
}

fn energy_capture_start_request(rate_pairs_per_second: u32) -> [u8; 5] {
    let mut request = [0; 5];
    request[1..].copy_from_slice(&rate_pairs_per_second.to_le_bytes());
    request
}

fn parse_energy_capture_start(
    response: &[u8],
) -> Result<EnergyCaptureStart, CommandError<EnergyError>> {
    let mut reader = ResponseReader::new(response);
    energy_mode_known(reader.u8()?)?;
    let status = reader.u8()?;
    let actual_rate_pairs_per_second = reader.u32()?;
    EnergyError::check(status).map_err(CommandError::Firmware)?;
    Ok(EnergyCaptureStart {
        actual_rate_pairs_per_second,
        capture_id: None,
    })
}

fn parse_capture_id(bytes: &[u8]) -> Result<crate::CaptureId, CommandError<EnergyError>> {
    crate::CaptureId::from_bytes(
        bytes
            .try_into()
            .map_err(|_| malformed("capture identity has the wrong length"))?,
    )
    .ok_or_else(|| malformed("capture identity has a zero counter").into())
}

fn parse_energy_status(response: &[u8]) -> Result<(), CommandError<EnergyError>> {
    let mut reader = ResponseReader::new(response);
    energy_mode_known(reader.u8()?)?;
    EnergyError::check(reader.u8()?).map_err(CommandError::Firmware)
}

fn parse_energy_capture_sample(
    response: &[u8],
    calibrated_supported: bool,
) -> Result<EnergyCaptureSample, CommandError<EnergyError>> {
    let mut reader = ResponseReader::new(response);
    energy_mode_known(reader.u8()?)?;
    let status = reader.u8()?;
    let raw_sample = RawEnergySample {
        voltage_code: reader.u16()?,
        current_code: reader.u16()?,
        sequence: reader.u32()?,
    };
    let calibrated = if calibrated_supported {
        let voltage_millivolts = reader.u32()?;
        let current_nanoamps = reader.i32()?;
        let range_id = reader.u8()?;
        let valid = read_bool(&mut reader, "calibrated-valid flag")?;
        if valid {
            Some(CalibratedEnergySample {
                voltage_millivolts,
                current_nanoamps,
                range_id,
            })
        } else {
            None
        }
    } else {
        None
    };
    match EnergyError::check(status) {
        Ok(()) => Ok(EnergyCaptureSample::Sample(EnergySample {
            raw: raw_sample,
            calibrated,
        })),
        Err(EnergyError::NoSample) => Ok(EnergyCaptureSample::NoSample),
        Err(EnergyError::Stopped) => Ok(EnergyCaptureSample::Stopped),
        Err(error) => Err(CommandError::Firmware(error)),
    }
}

fn parse_energy_accumulator(response: &[u8]) -> Result<EnergyTotal, CommandError<EnergyError>> {
    let mut reader = ResponseReader::new(response);
    energy_mode_known(reader.u8()?)?;
    let status = reader.u8()?;
    let accumulator = EnergyAccumulator {
        energy_nanojoules: reader.u64()?,
        integrated_pairs: reader.u64()?,
    };
    match EnergyError::check(status) {
        Ok(()) => Ok(EnergyTotal::Total(accumulator)),
        Err(EnergyError::DmaError) => Ok(EnergyTotal::Failed(accumulator)),
        Err(EnergyError::NoSample) => Ok(EnergyTotal::NoSample),
        Err(EnergyError::Stopped) => Ok(EnergyTotal::Stopped),
        Err(error) => Err(CommandError::Firmware(error)),
    }
}

fn parse_energy_calibration(
    response: &[u8],
) -> Result<EnergyCalibration, CommandError<EnergyError>> {
    let mut reader = ResponseReader::new(response);
    // Firmware from before mode 4 answers it DAP_ERROR.
    if !vendor_status(reader.u8()?)? {
        return Err(CommandError::Unsupported("the energy calibration points"));
    }
    let status = reader.u8()?;
    let count = usize::from(reader.u8()?);
    let mut points = Vec::with_capacity(4);
    for _ in 0..4 {
        points.push(CalibrationPoint {
            code: reader.u16()?,
            current_nanoamps: reader.u32()?,
        });
    }
    if count > points.len() {
        return Err(malformed(format!(
            "{count} calibration points in a reply that holds four"
        ))
        .into());
    }
    points.truncate(count);
    match EnergyError::check(status) {
        Ok(()) => Ok(EnergyCalibration::Calibrated(points)),
        Err(EnergyError::AdcError) => Ok(EnergyCalibration::Failed(points)),
        Err(error) => Err(CommandError::Firmware(error)),
    }
}

fn encode_spi_request(
    clock_hz: u32,
    mode: SpiMode,
    lsb_first: bool,
    tx: &[u8],
    packet: usize,
) -> Result<Vec<u8>, &'static str> {
    if !(SPI_MIN_CLOCK_HZ..=SPI_MAX_CLOCK_HZ).contains(&clock_hz) {
        return Err("SPI clock is outside the supported range");
    }
    if tx.is_empty() {
        return Err("SPI transfer is empty");
    }
    if tx.len() > packet.saturating_sub(SPI_REQUEST_HEADER) {
        return Err("SPI transfer is longer than this probe's packet holds");
    }
    let count = u16::try_from(tx.len()).map_err(|_| "SPI transfer is too long")?;
    let mut request = Vec::with_capacity(7 + tx.len());
    request.push(mode.bits() | if lsb_first { 1 << 2 } else { 0 });
    request.extend_from_slice(&clock_hz.to_le_bytes());
    request.extend_from_slice(&count.to_le_bytes());
    request.extend_from_slice(tx);
    Ok(request)
}

fn parse_bus_response(
    response: &[u8],
    expected_count: usize,
) -> Result<Vec<u8>, CommandError<ProBusError>> {
    let mut reader = ResponseReader::new(response);
    let status = reader.u8()?;
    let count = usize::from(reader.u16()?);
    let data = response
        .get(3..3 + count)
        .ok_or_else(|| malformed("short bridge data response"))?;
    if status == 0 && count != expected_count {
        return Err(malformed("bridge response length does not match the request").into());
    }
    if status != 0 && count != 0 {
        return Err(malformed("failed bridge response contains data").into());
    }
    ProBusError::check(status).map_err(CommandError::Firmware)?;
    Ok(data.to_vec())
}

fn encode_i2c_request(
    address: u8,
    clock_hz: u32,
    write: &[u8],
    read_len: usize,
    packet: usize,
) -> Result<Vec<u8>, &'static str> {
    if address > 0x7f {
        return Err("I2C address is not 7-bit");
    }
    if !(I2C_MIN_CLOCK_HZ..=I2C_MAX_CLOCK_HZ).contains(&clock_hz) {
        return Err("I2C clock is outside the supported range");
    }
    if write.is_empty() && read_len == 0 {
        return Err("I2C transfer is empty");
    }
    if write.len() > packet.saturating_sub(I2C_REQUEST_HEADER) {
        return Err("I2C write is longer than this probe's packet holds");
    }
    if read_len > packet.saturating_sub(BUS_REPLY_HEADER) {
        return Err("I2C read is longer than this probe's packet holds");
    }
    let write_count = u16::try_from(write.len()).map_err(|_| "I2C write is too long")?;
    let read_count = u16::try_from(read_len).map_err(|_| "I2C read is too long")?;
    let mut request = Vec::with_capacity(9 + write.len());
    request.push(address);
    request.extend_from_slice(&clock_hz.to_le_bytes());
    request.extend_from_slice(&write_count.to_le_bytes());
    request.extend_from_slice(&read_count.to_le_bytes());
    request.extend_from_slice(write);
    Ok(request)
}

fn parse_swd_codegen_state(response: &[u8]) -> Result<(bool, SwdCodegenState), MalformedReply> {
    let mut reader = ResponseReader::new(response);
    let accepted = vendor_status(reader.u8()?)?;
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
    let state = SwdCodegenState {
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
    };
    Ok((accepted, state))
}

fn parse_jtag_codegen_state(response: &[u8]) -> Result<(bool, JtagCodegenState), MalformedReply> {
    let mut reader = ResponseReader::new(response);
    let accepted = vendor_status(reader.u8()?)?;
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
    let state = JtagCodegenState {
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
    };
    Ok((accepted, state))
}

fn encode_sample_request(request: SampleRequest) -> Result<Vec<u8>, &'static str> {
    if request.fill > SAMPLE_FILL_MAX {
        return Err("sample fill exceeds the firmware limit");
    }
    let at = match request.position {
        SamplePosition::AfterEdge(cycles) if cycles <= request.fill => cycles,
        SamplePosition::Lead(cycles) if cycles <= request.fill => 0x80 | cycles,
        SamplePosition::BeforeEdge => 0xFF,
        _ => return Err("sample position exceeds the filler phase"),
    };
    let mut payload = vec![request.low, request.fill, at];
    payload.extend_from_slice(&request.count.to_le_bytes());
    payload.extend_from_slice(&request.expected_dpidr.to_le_bytes());
    Ok(payload)
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

    fn take<const N: usize>(&mut self) -> Result<[u8; N], MalformedReply> {
        let value = response_array(self.bytes, self.offset..self.offset + N)?;
        self.offset += N;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, MalformedReply> {
        Ok(self.take::<1>()?[0])
    }

    fn u16(&mut self) -> Result<u16, MalformedReply> {
        Ok(u16::from_le_bytes(self.take()?))
    }

    fn u32(&mut self) -> Result<u32, MalformedReply> {
        Ok(u32::from_le_bytes(self.take()?))
    }

    fn i32(&mut self) -> Result<i32, MalformedReply> {
        Ok(i32::from_le_bytes(self.take()?))
    }

    fn u64(&mut self) -> Result<u64, MalformedReply> {
        Ok(u64::from_le_bytes(self.take()?))
    }
}

fn read_bool(reader: &mut ResponseReader<'_>, field: &str) -> Result<bool, MalformedReply> {
    match reader.u8()? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(malformed(format!("invalid {field}"))),
    }
}

fn read_bool_at(response: &[u8], index: usize, field: &str) -> Result<bool, MalformedReply> {
    match response_byte(response, index)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(malformed(format!("invalid {field}"))),
    }
}

fn read_codegen_measurement(
    reader: &mut ResponseReader<'_>,
    has_wire: bool,
) -> Result<CodegenMeasurement, MalformedReply> {
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

fn response_byte(response: &[u8], index: usize) -> Result<u8, MalformedReply> {
    response
        .get(index)
        .copied()
        .ok_or_else(|| malformed("short vendor command response"))
}

fn response_array<const N: usize>(
    response: &[u8],
    range: std::ops::Range<usize>,
) -> Result<[u8; N], MalformedReply> {
    response
        .get(range)
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or_else(|| malformed("short vendor command response"))
}

/// Why a vendor command was not answered.
enum SendError {
    Probe(DebugProbeError),
    Unsupported(&'static str),
}

impl<E: std::error::Error + 'static> From<SendError> for CommandError<E> {
    fn from(error: SendError) -> Self {
        match error {
            SendError::Probe(error) => Self::Probe(error),
            SendError::Unsupported(command) => Self::Unsupported(command),
        }
    }
}

fn command_name(command_id: u8) -> &'static str {
    match command_id {
        J8_COMMAND => "the J8 marker pins",
        TARGET_SENSE_COMMAND => "target sensing",
        SAMPLE_COMMAND => "the SWD sample instrument",
        CODE_LAB_COMMAND => "the code lab",
        SWD_CODEGEN_COMMAND => "SWD code generation",
        JTAG_CODEGEN_COMMAND | JTAG_PACING_COMMAND | JTAG_LOG_COMMAND => "JTAG code generation",
        CPU_STAGING_COMMAND => "command staging",
        DIO_COMMAND => "the DIO pins",
        TARGET_POWER_COMMAND => "target power control",
        ENERGY_COMMAND => "energy capture",
        SPI_COMMAND => "the SPI bridge",
        I2C_COMMAND => "the I2C bridge",
        MARK_COMMAND => "markers",
        _ => "this vendor command",
    }
}

/// A reply that does not follow its command's protocol.
#[derive(Debug)]
struct MalformedReply(String);

fn malformed(reason: impl Into<String>) -> MalformedReply {
    MalformedReply(reason.into())
}

impl<E: std::error::Error + 'static> From<MalformedReply> for CommandError<E> {
    fn from(reply: MalformedReply) -> Self {
        Self::Malformed(reply.0)
    }
}

impl From<MalformedReply> for DebugProbeError {
    fn from(reply: MalformedReply) -> Self {
        Self::Other(reply.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::CAP_CALIBRATED_ENERGY;
    use std::collections::VecDeque;

    struct FakeTransport {
        replies: VecDeque<Vec<u8>>,
        commands: Vec<(u8, Vec<u8>)>,
    }

    impl VergeTransport for FakeTransport {
        fn send_vendor_command(
            &mut self,
            command_id: u8,
            payload: &[u8],
        ) -> Result<Vec<u8>, DebugProbeError> {
            self.commands.push((command_id, payload.to_vec()));
            Ok(self.replies.pop_front().unwrap())
        }
    }

    /// `formats` fills the field formats 4 to 6 used for 0x83's scan formats, 0 from format 7.
    fn info_reply(
        version: u8,
        capabilities: u8,
        board: u8,
        formats: u32,
        extended_capabilities: u32,
    ) -> Vec<u8> {
        let build = b"test-build";
        let mut reply = b"CJTAG".to_vec();
        reply.extend_from_slice(&[version, capabilities, build.len() as u8]);
        reply.extend_from_slice(build);
        if version >= 3 {
            reply.push(board);
        }
        if version >= 4 {
            reply.extend_from_slice(&formats.to_le_bytes());
        }
        if version >= 6 {
            reply.extend_from_slice(&extended_capabilities.to_le_bytes());
        }
        reply
    }

    #[test]
    fn target_power_is_sent_through_the_selected_probe_transport() {
        let mut transport = FakeTransport {
            replies: [info_reply(6, CAP_TARGET_POWER, 1, 0, 0), vec![0, 1]].into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);

        assert!(verge.set_target_power(true).unwrap());
        assert_eq!(
            transport.commands,
            [(INFO_COMMAND, vec![]), (TARGET_POWER_COMMAND, vec![1])]
        );
    }

    #[test]
    fn identity_cache_remembers_absence_and_refreshes_it() {
        let mut transport = FakeTransport {
            replies: [Vec::new(), info_reply(6, CAP_DIO, 1, 0, 0)].into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);
        assert!(verge.info().unwrap().is_none());
        assert!(verge.info().unwrap().is_none());
        assert!(verge.refresh_info().unwrap().unwrap().supports_dio());
        assert!(verge.info().unwrap().unwrap().supports_dio());
        assert_eq!(transport.commands.len(), 2);
    }

    #[test]
    fn a_probe_that_cannot_answer_the_identity_command_is_not_verge() {
        let unsupported: [fn() -> DebugProbeError; 2] = [
            || DebugProbeError::CommandNotSupportedByProbe {
                command_name: "vendor command",
            },
            || DebugProbeError::NotImplemented {
                function_name: "vendor commands",
            },
        ];
        for error in unsupported {
            let mut send = |_, _: &[u8]| Err(error());
            assert_eq!(read_info(&mut send).unwrap(), None);
        }
        let mut send = |_, _: &[u8]| Err(DebugProbeError::Timeout);
        assert!(matches!(
            read_info(&mut send),
            Err(CommandError::Probe(DebugProbeError::Timeout))
        ));
    }

    #[test]
    fn a_failed_identity_read_can_be_retried() {
        struct Flaky(u8);
        impl VergeTransport for Flaky {
            fn send_vendor_command(&mut self, _: u8, _: &[u8]) -> Result<Vec<u8>, DebugProbeError> {
                self.0 += 1;
                if self.0 == 1 {
                    Err(DebugProbeError::Other("read failed".to_owned()))
                } else {
                    Ok(info_reply(6, CAP_DIO, 1, 0, 0))
                }
            }
        }
        let mut transport = Flaky(0);
        let mut verge = VergeControl::new(&mut transport);
        assert!(verge.info().is_err());
        assert!(verge.info().unwrap().unwrap().supports_dio());
        assert!(verge.info().unwrap().unwrap().supports_dio());
        assert_eq!(transport.0, 2);
    }

    #[test]
    fn a_refusal_is_an_error_that_carries_the_reported_state() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, CAP_TARGET_POWER | CAP_DIO, 1, 0, 0),
                vec![0xFF, 0],
                vec![0xFF, 0, 0, 0, 15, 2, 7, 8],
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);

        assert!(matches!(
            verge.set_target_power(true),
            Err(CommandError::Firmware(Refused(false)))
        ));
        let Err(CommandError::Firmware(Refused(state))) = verge.access_dio(0x07, 0x01, 0x01) else {
            panic!("a refused DIO update must be an error");
        };
        assert_eq!((state.outputs, state.groups), (0, vec![7, 8]));
    }

    #[test]
    fn a_failed_target_sense_names_the_failed_step() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, CAP_TARGET_SENSE, 1, 0, 0),
                vec![0xFF, 2, 0, 0, 0, 1],
                vec![0, 0x34, 0x12, 0xE4, 0x0C, 1],
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);

        assert!(matches!(
            verge.read_target_sense(),
            Err(CommandError::Firmware(TargetSenseError::GainCalibration))
        ));
        assert_eq!(
            verge.read_target_sense().unwrap(),
            TargetSenseReading {
                code: 0x1234,
                millivolts: 3300,
                detected: true,
            }
        );
    }

    /// Firmware before the range setting always switches automatically: asking it for that is
    /// no request at all, and asking it to hold the high range is refused before anything is sent.
    #[test]
    fn the_range_setting_asks_only_firmware_that_has_it() {
        struct Ranges(Vec<Vec<u8>>, u32);
        impl VergeTransport for Ranges {
            fn send_vendor_command(
                &mut self,
                command_id: u8,
                payload: &[u8],
            ) -> Result<Vec<u8>, DebugProbeError> {
                self.0.push([&[command_id][..], payload].concat());
                match command_id {
                    INFO_COMMAND => Ok(info_reply(6, 0, 1, 0, self.1)),
                    ENERGY_COMMAND => Ok(vec![0, 0]),
                    _ => Err(DebugProbeError::CommandNotSupportedByProbe {
                        command_name: "vendor command",
                    }),
                }
            }
        }
        let mut old = Ranges(Vec::new(), 1);
        VergeControl::new(&mut old)
            .set_energy_ranges(EnergyRanges::Auto)
            .unwrap();
        assert!(matches!(
            VergeControl::new(&mut old).set_energy_ranges(EnergyRanges::High),
            Err(CommandError::Unsupported(_))
        ));
        assert!(old.0.iter().all(|sent| sent[0] == INFO_COMMAND));
        let mut new = Ranges(Vec::new(), 1 | 1 << 9);
        for (ranges, wire) in [(EnergyRanges::High, 1), (EnergyRanges::Auto, 0)] {
            VergeControl::new(&mut new)
                .set_energy_ranges(ranges)
                .unwrap();
            assert_eq!(new.0.last().unwrap(), &vec![ENERGY_COMMAND, 14, wire]);
        }
    }

    #[test]
    fn a_command_the_build_lacks_is_unsupported_and_verge_only_commands_ask_first() {
        struct NoCodeLab(Vec<u8>);
        impl VergeTransport for NoCodeLab {
            fn send_vendor_command(
                &mut self,
                command_id: u8,
                _: &[u8],
            ) -> Result<Vec<u8>, DebugProbeError> {
                self.0.push(command_id);
                match command_id {
                    INFO_COMMAND => Ok(info_reply(6, 0, 1, 0, 0)),
                    _ => Err(DebugProbeError::CommandNotSupportedByProbe {
                        command_name: "vendor command",
                    }),
                }
            }
        }
        let mut transport = NoCodeLab(Vec::new());
        assert!(matches!(
            VergeControl::new(&mut transport).code_lab_arenas(),
            Err(CommandError::Unsupported("the code lab"))
        ));
        assert_eq!(transport.0, [INFO_COMMAND, CODE_LAB_COMMAND]);

        let mut transport = FakeTransport {
            replies: [b"DAPLink".to_vec()].into(),
            commands: Vec::new(),
        };
        assert!(matches!(
            VergeControl::new(&mut transport).set_marker_pins(0),
            Err(CommandError::Unsupported("Verge commands"))
        ));
        assert_eq!(transport.commands, [(INFO_COMMAND, vec![])]);
    }

    #[test]
    fn unsupported_and_invalid_requests_send_nothing() {
        let mut transport = FakeTransport {
            replies: [info_reply(6, 0, 1, 0, CAP_SPI)].into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);

        assert!(matches!(
            verge.i2c_transfer(0x50, 100_000, &[], 1),
            Err(CommandError::Unsupported("the I2C bridge"))
        ));
        assert!(matches!(
            verge.spi_transfer(1, SpiMode::Mode0, false, &[0]),
            Err(CommandError::InvalidRequest(_))
        ));
        assert_eq!(transport.commands, [(INFO_COMMAND, vec![])]);

        let mut transport = FakeTransport {
            replies: [Vec::new()].into(),
            commands: Vec::new(),
        };
        assert!(matches!(
            VergeControl::new(&mut transport).read_target_power(),
            Err(CommandError::Unsupported("Verge commands"))
        ));
    }

    #[test]
    fn spi_request_and_reply_enforce_encoding_and_counts() {
        let request =
            encode_spi_request(100_000, SpiMode::Mode3, true, &[0x12, 0x34], 512).unwrap();
        assert_eq!(&request[..7], &[7, 0xA0, 0x86, 1, 0, 2, 0]);
        assert_eq!(&request[7..], &[0x12, 0x34]);
        assert!(encode_spi_request(1, SpiMode::Mode0, false, &[0], 512).is_err());
        assert!(encode_spi_request(100_000, SpiMode::Mode0, false, &[], 512).is_err());
        // docs/host-interface.md: 504 bytes on bulk at high speed, 56 on HID and at full speed.
        for (packet, longest) in [(512, 504), (64, 56)] {
            let spi =
                |len| encode_spi_request(100_000, SpiMode::Mode0, false, &vec![0; len], packet);
            assert!(spi(longest).is_ok());
            assert!(spi(longest + 1).is_err());
        }

        let parsed = parse_bus_response(&[0, 2, 0, 0xA5, 0x5A], 2).unwrap();
        assert_eq!(parsed, [0xA5, 0x5A]);
        assert!(parse_bus_response(&[0, 1, 0, 0xA5], 2).is_err());
        assert_eq!(
            parse_bus_response(&[3, 0, 0], 2).unwrap_err().to_string(),
            ProBusError::Busy.to_string()
        );
    }

    /// A Pro on bulk at full speed, answering the bridges with zeros. `DAP_Info` times out while
    /// `info_fails` counts down.
    struct FullSpeedLink {
        sent: Vec<Vec<u8>>,
        info_fails: usize,
    }

    impl DapLink for FullSpeedLink {
        fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, crate::dap::LinkError> {
            self.sent.push(request.to_vec());
            if request.first() == Some(&crate::dap::DAP_INFO) && self.info_fails > 0 {
                self.info_fails -= 1;
                return Err(crate::dap::LinkError::Timeout);
            }
            Ok(match request {
                [crate::dap::DAP_INFO, 0xFF] => vec![crate::dap::DAP_INFO, 2, 64, 0],
                [INFO_COMMAND] => {
                    let info = info_reply(6, 0, 2, 0, CAP_SPI | CAP_I2C);
                    [&[INFO_COMMAND][..], &info].concat()
                }
                [SPI_COMMAND, _, _, _, _, _, lo, hi, ..] | [I2C_COMMAND, .., lo, hi] => {
                    let count = u16::from_le_bytes([*lo, *hi]);
                    let mut reply = vec![request[0], 0, *lo, *hi];
                    reply.resize(BUS_REPLY_HEADER + usize::from(count), 0);
                    reply
                }
                _ => vec![ID_DAP_INVALID],
            })
        }

        fn drain(&mut self) -> Result<usize, crate::dap::LinkError> {
            Ok(0)
        }

        fn control_out(
            &mut self,
            _: u8,
            _: u8,
            _: u16,
            _: u16,
        ) -> Result<(), crate::dap::LinkError> {
            Ok(())
        }
    }

    #[test]
    fn bridges_are_sized_from_the_links_packet() {
        let mut link = FullSpeedLink {
            sent: Vec::new(),
            info_fails: 0,
        };
        let mut transport = DapTransport(&mut link);
        let mut verge = VergeControl::new(&mut transport);
        assert!(matches!(
            verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0; 57]),
            Err(CommandError::InvalidRequest(_))
        ));
        assert!(matches!(
            verge.i2c_transfer(0x50, 100_000, &[], 61),
            Err(CommandError::InvalidRequest(_))
        ));
        assert_eq!(
            verge
                .spi_transfer(100_000, SpiMode::Mode0, false, &[0; 56])
                .unwrap(),
            [0; 56]
        );
        assert_eq!(verge.i2c_transfer(0x50, 100_000, &[], 60).unwrap(), [0; 60]);
        let commands: Vec<Vec<u8>> = link
            .sent
            .iter()
            .map(|request| request.iter().take(2).copied().collect())
            .collect();
        assert_eq!(
            commands,
            [
                vec![INFO_COMMAND],
                vec![crate::dap::DAP_INFO, 0xFF],
                vec![SPI_COMMAND, 0],
                vec![I2C_COMMAND, 0x50],
            ]
        );
    }

    #[test]
    fn a_packet_size_that_could_not_be_read_is_asked_again() {
        let mut link = FullSpeedLink {
            sent: Vec::new(),
            info_fails: 1,
        };
        let mut transport = DapTransport(&mut link);
        let mut verge = VergeControl::new(&mut transport);
        assert!(verge.info().unwrap().is_some());
        assert!(matches!(
            verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0; 100]),
            Err(CommandError::Probe(_))
        ));
        assert!(matches!(
            verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0; 100]),
            Err(CommandError::InvalidRequest(_))
        ));
        assert!(!link.sent.iter().any(|request| request[0] == SPI_COMMAND));
    }

    #[test]
    fn a_transport_that_cannot_tell_gets_the_largest_packet() {
        let mut transport = FakeTransport {
            replies: [info_reply(6, 0, 2, 0, CAP_SPI)].into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);
        assert!(matches!(
            verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0; 505]),
            Err(CommandError::InvalidRequest(_))
        ));
        verge
            .transport
            .replies
            .push_back([&[0, 0xF8, 1][..], &[0; 504]].concat());
        assert!(
            verge
                .spi_transfer(100_000, SpiMode::Mode0, false, &[0; 504])
                .is_ok()
        );
    }

    #[test]
    fn i2c_request_and_reply_enforce_ranges_and_counts() {
        let request = encode_i2c_request(0x50, 400_000, &[0x10], 2, 512).unwrap();
        assert_eq!(request, [0x50, 0x80, 0x1A, 0x06, 0, 1, 0, 2, 0, 0x10]);
        assert!(encode_i2c_request(0x80, 100_000, &[], 1, 512).is_err());
        assert!(encode_i2c_request(0x50, 1_000, &[], 1, 512).is_err());
        assert!(encode_i2c_request(0x50, 100_000, &[], 0, 512).is_err());
        // docs/host-interface.md: 502 written or 508 read at high speed, 54 or 60 on HID and at
        // full speed.
        for (packet, write, read) in [(512, 502, 508), (64, 54, 60)] {
            let i2c =
                |write, read| encode_i2c_request(0x50, 100_000, &vec![0; write], read, packet);
            assert!(i2c(write, read).is_ok());
            assert!(i2c(write + 1, 0).is_err());
            assert!(i2c(0, read + 1).is_err());
        }
        assert!(encode_i2c_request(0x50, 100_000, &[], usize::MAX, 512).is_err());

        let parsed = parse_bus_response(&[0, 2, 0, 0xAA, 0xBB], 2).unwrap();
        assert_eq!(parsed, [0xAA, 0xBB]);
        assert_eq!(
            parse_bus_response(&[5, 0, 0], 1).unwrap_err().to_string(),
            ProBusError::Nack.to_string()
        );
        assert!(parse_bus_response(&[0, 1, 0, 0xAA], 2).is_err());
    }

    #[test]
    fn extended_capabilities_advertise_the_pro_bridges() {
        let reply = info_reply(6, 0, 1, 0, CAP_SPI | CAP_I2C);
        let mut send = |command_id, _: &[u8]| {
            assert_eq!(command_id, INFO_COMMAND);
            Ok(reply.clone())
        };
        let info = read_info(&mut send).unwrap().unwrap();
        assert!(info.supports_spi());
        assert!(info.supports_i2c());
        assert!(!info.supports_energy_capture());
    }

    fn test_capture_id() -> crate::CaptureId {
        let mut bytes = [0x42; 24];
        bytes[16..].copy_from_slice(&7u64.to_le_bytes());
        crate::CaptureId::from_bytes(bytes).unwrap()
    }

    fn identified_start_reply(rate: u32) -> Vec<u8> {
        [
            vec![0, 0],
            rate.to_le_bytes().to_vec(),
            test_capture_id().as_bytes().to_vec(),
        ]
        .concat()
    }

    fn identified_stop_request() -> Vec<u8> {
        [vec![11], test_capture_id().as_bytes().to_vec()].concat()
    }

    #[test]
    fn energy_monitor_delivers_new_samples_and_stops_capture() {
        let mut latest = vec![0, 0];
        latest.extend_from_slice(&1u16.to_le_bytes());
        latest.extend_from_slice(&2u16.to_le_bytes());
        latest.extend_from_slice(&7u32.to_le_bytes());
        latest.extend_from_slice(&3300u32.to_le_bytes());
        latest.extend_from_slice(&100i32.to_le_bytes());
        latest.extend_from_slice(&[0, 1]);
        let mut next = latest.clone();
        next[6..10].copy_from_slice(&8u32.to_le_bytes());
        let mut transport = FakeTransport {
            replies: [
                info_reply(
                    6,
                    CAP_TARGET_POWER,
                    1,
                    0,
                    CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID | CAP_CALIBRATED_ENERGY,
                ),
                identified_start_reply(1000),
                latest.clone(),
                latest,
                next,
                vec![0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let running = std::rc::Rc::new(std::cell::Cell::new(true));
        let stop = running.clone();
        let sample_running = running.clone();
        let mut samples = Vec::new();
        let mut verge = VergeControl::new(&mut transport);

        verge
            .monitor_energy(
                1_000,
                Duration::from_millis(1),
                || !stop.get(),
                |sample| {
                    samples.push(sample);
                    sample_running.set(samples.len() < 2);
                },
            )
            .unwrap();

        assert_eq!(samples.len(), 2);
        assert_eq!(samples[1].raw.sequence, 8);
        assert_eq!(samples[0].raw.sequence, 7);
        assert_eq!(
            transport.commands[1],
            (ENERGY_COMMAND, vec![8, 0xE8, 3, 0, 0])
        );
        assert_eq!(transport.commands[2], (ENERGY_COMMAND, vec![1]));
        assert_eq!(
            transport.commands[5],
            (ENERGY_COMMAND, identified_stop_request())
        );
    }

    #[test]
    fn energy_monitor_stops_capture_after_a_malformed_poll_reply() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                identified_start_reply(1000),
                vec![0],
                vec![0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);

        let error = verge
            .monitor_energy(
                1_000,
                Duration::from_millis(1),
                || false,
                |_| panic!("malformed sample must not be delivered"),
            )
            .unwrap_err();

        assert!(matches!(error, EnergyMonitorError::Poll(_)));
        assert_eq!(
            transport.commands.last(),
            Some(&(ENERGY_COMMAND, identified_stop_request()))
        );
    }

    #[test]
    fn energy_monitor_stops_capture_when_a_callback_panics() {
        let mut sample = vec![0, 0];
        sample.extend_from_slice(&[0; 4]);
        sample.extend_from_slice(&1u32.to_le_bytes());
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                identified_start_reply(1000),
                sample,
                vec![0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = verge.monitor_energy(
                1_000,
                Duration::from_millis(1),
                || false,
                |_| panic!("the caller's sample handler failed"),
            );
        }));

        assert!(panicked.is_err());
        assert_eq!(
            transport.commands.last(),
            Some(&(ENERGY_COMMAND, identified_stop_request()))
        );
    }

    #[test]
    fn energy_monitor_does_not_stop_after_a_garbled_start_reply() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                vec![0],
                vec![0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let error = VergeControl::new(&mut transport)
            .monitor_energy(1_000, Duration::from_millis(1), || true, |_| {})
            .unwrap_err();

        assert!(matches!(
            error,
            EnergyMonitorError::Start(CommandError::Malformed(_))
        ));
        assert_eq!(transport.commands.len(), 2);
        assert_eq!(
            transport.commands.last(),
            Some(&(ENERGY_COMMAND, vec![8, 0xe8, 3, 0, 0]))
        );
    }

    #[test]
    fn energy_monitor_survives_a_transport_that_panics_while_stopping() {
        let mut sample = vec![0, 0];
        sample.extend_from_slice(&[0; 4]);
        sample.extend_from_slice(&1u32.to_le_bytes());
        // No stop reply: the fake panics on the stop, during the unwind.
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                identified_start_reply(1000),
                sample,
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut verge = VergeControl::new(&mut transport);
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = verge.monitor_energy(
                1_000,
                Duration::from_millis(1),
                || false,
                |_| panic!("the caller's sample handler failed"),
            );
        }));

        assert!(panicked.is_err());
        assert_eq!(
            transport.commands.last(),
            Some(&(ENERGY_COMMAND, identified_stop_request()))
        );
    }

    #[test]
    fn energy_monitor_returns_the_effective_rate_and_checks_stop_between_polls() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                identified_start_reply(999),
                vec![0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let mut checks = 0;
        let started = Instant::now();
        let start = VergeControl::new(&mut transport)
            .monitor_energy(
                1_000,
                Duration::MAX,
                || {
                    checks += 1;
                    checks == 3
                },
                |_| panic!("no poll is due"),
            )
            .unwrap();

        assert_eq!(start.actual_rate_pairs_per_second, 999);
        assert!(started.elapsed() < Duration::from_secs(1));
        assert_eq!(transport.commands.len(), 3);
    }

    #[test]
    fn energy_monitor_does_not_stop_after_a_refused_start() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                vec![0, 4, 0, 0, 0, 0],
            ]
            .into(),
            commands: Vec::new(),
        };
        let error = VergeControl::new(&mut transport)
            .monitor_energy(0, Duration::from_millis(1), || true, |_| {})
            .unwrap_err();
        assert!(matches!(
            error,
            EnergyMonitorError::Start(CommandError::Firmware(EnergyError::BadRate))
        ));
        assert_eq!(transport.commands.len(), 2);
    }

    #[test]
    fn energy_monitor_preserves_firmware_and_cleanup_errors() {
        for status in [1, 3, 4, 5, 6, 7, 0x7f] {
            for stop_reply in [vec![0, 0], vec![0, 7], vec![0]] {
                let mut sample = vec![0; 10];
                sample[1] = status;
                let mut transport = FakeTransport {
                    replies: [
                        info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE | CAP_CAPTURE_ID),
                        identified_start_reply(1000),
                        sample,
                        stop_reply.clone(),
                    ]
                    .into(),
                    commands: Vec::new(),
                };
                let error = VergeControl::new(&mut transport)
                    .monitor_energy(
                        1000,
                        Duration::from_millis(1),
                        || false,
                        |_| panic!("failed capture produced a sample"),
                    )
                    .unwrap_err();
                let expected = EnergyError::check(status).unwrap_err();
                match error {
                    EnergyMonitorError::Poll(CommandError::Firmware(poll)) => {
                        assert_eq!(poll, expected);
                        assert_eq!(stop_reply, [0, 0]);
                    }
                    EnergyMonitorError::PollAndStop {
                        poll: CommandError::Firmware(poll),
                        stop,
                    } => {
                        assert_eq!(poll, expected);
                        match stop {
                            CommandError::Firmware(EnergyError::DmaError) => {
                                assert_eq!(stop_reply, [0, 7])
                            }
                            CommandError::Malformed(_) => assert_eq!(stop_reply, [0]),
                            error => panic!("unexpected cleanup error: {error}"),
                        }
                    }
                    error => panic!("unexpected monitor error: {error}"),
                }
                assert_eq!(
                    transport.commands.last(),
                    Some(&(ENERGY_COMMAND, identified_stop_request()))
                );
            }
        }
    }

    #[test]
    fn bridge_errors_preserve_unknown_codes_and_reject_data() {
        assert!(matches!(
            parse_bus_response(&[0x7f, 0, 0], 1),
            Err(CommandError::Firmware(ProBusError::Unknown(0x7f)))
        ));
        assert!(matches!(
            parse_bus_response(&[5, 1, 0, 42], 1),
            Err(CommandError::Malformed(_))
        ));
        assert!(matches!(
            parse_bus_response(&[0, 2, 0, 42], 2),
            Err(CommandError::Malformed(_))
        ));
    }

    #[test]
    fn info_parser_reads_only_fields_supported_by_the_format() {
        for (version, board, scan_formats, extended_capabilities) in [
            (1, None, 0, 0),
            (3, Some(BoardKind::Pro), 0, 0),
            (4, Some(BoardKind::Pro), 0, 0),
            (6, Some(BoardKind::Pro), 0, 0x7),
            (7, Some(BoardKind::Pro), protocol::CJTAG_FORMATS, 0x7),
            (8, Some(BoardKind::Pro), protocol::CJTAG_FORMATS, 0x7),
        ] {
            let reply = info_reply(version, 0xA5, 1, 0x204, 0x7);
            let mut send = |command_id, payload: &[u8]| {
                assert_eq!(command_id, INFO_COMMAND);
                assert!(payload.is_empty());
                Ok(reply.clone())
            };
            let info = read_info(&mut send).unwrap().unwrap();
            assert_eq!(info.version, version);
            assert_eq!(info.capabilities, if version < 2 { 0 } else { 0xA5 });
            assert_eq!(info.firmware, if version < 2 { "" } else { "test-build" });
            assert_eq!(info.board, board);
            assert_eq!(info.scan_formats, scan_formats);
            assert_eq!(info.extended_capabilities, extended_capabilities);
        }
    }

    #[test]
    fn both_transports_decode_the_same_identity_and_gpio_payloads() {
        for payload in [
            b"CJTAG\x01".to_vec(),
            info_reply(3, CAP_DIO, 1, 0, 0),
            info_reply(6, CAP_TARGET_POWER, 1, 0x600, 0x1f),
            b"CJTAG\x06\x00\x05x".to_vec(),
            Vec::new(),
        ] {
            let mut packet = vec![INFO_COMMAND];
            packet.extend_from_slice(&payload);
            let raw = crate::dap::parse_identify(&packet);
            let mut send = |_, _: &[u8]| Ok(payload.clone());
            let probe = read_info(&mut send);
            match (raw, probe) {
                (Ok(a), Ok(b)) => assert_eq!(a, b),
                (Err(_), Err(_)) => {}
                replies => panic!("decoders disagree: {replies:?}"),
            }
        }
        for payload in [
            vec![0, 7, 5, 13, 15, 2, 7, 8],
            vec![0x42, 0, 0, 0, 0, 0],
            vec![0],
        ] {
            let mut packet = vec![DIO_COMMAND];
            packet.extend_from_slice(&payload);
            let mut transport = FakeTransport {
                replies: [info_reply(6, CAP_DIO, 1, 0, 0), payload].into(),
                commands: Vec::new(),
            };
            let probe = VergeControl::new(&mut transport).access_dio(0, 0, 0);
            match (crate::dap::parse_dio(&packet), probe) {
                (Ok(Some(a)), Ok(b)) => assert_eq!(a, b),
                // A status that is neither accepted nor refused is the protocol's to reject here.
                (Ok(Some(a)), Err(CommandError::Malformed(_))) => {
                    assert!(matches!(a.status, VendorStatus::Unknown(_)))
                }
                (Err(_), Err(CommandError::Malformed(_))) => {}
                replies => panic!("decoders disagree: {replies:?}"),
            }
        }
    }

    #[test]
    fn only_an_mcu_links_usb_id_is_an_mcu_link() {
        let info = |vid, pid| {
            DebugProbeInfo::new(
                "probe",
                vid,
                pid,
                None,
                &probe_rs::probe::cmsisdap::CmsisDapFactory,
                None,
                false,
            )
        };
        assert!(is_mcu_link(&info(0x1fc9, 0x0143)));
        // Arm's DAPLink ID, which daplink-rs on an MCU-Link shares with every DAPLink probe, and
        // the LPC55S69 ROM bootloader.
        assert!(!is_mcu_link(&info(0x0d28, 0x0204)));
        assert!(!is_mcu_link(&info(0x1fc9, 0x0021)));
    }

    #[test]
    fn cjtag_needs_verge_with_cjtag_and_nothing_is_cleared() {
        let prepare = |reply: Vec<u8>| {
            let mut transport = FakeTransport {
                replies: [reply].into(),
                commands: Vec::new(),
            };
            let result = require_cjtag(&mut transport);
            (result, transport.commands)
        };
        let identify = (INFO_COMMAND, vec![]);
        let (result, sent) = prepare(info_reply(8, 0, BoardKind::Pro.number(), 0, 0));
        assert!(result.is_ok());
        assert_eq!(sent, std::slice::from_ref(&identify));
        let (result, sent) = prepare(info_reply(8, 0, BoardKind::OnBoard.number(), 0, 0));
        assert!(matches!(result, Err(AttachError::NoCjtag)));
        assert_eq!(sent, std::slice::from_ref(&identify));
        // Format 7 has 0x95, but on it a Format sent while disconnected selects nothing.
        let (result, sent) = prepare(info_reply(7, 0, BoardKind::Pro.number(), 0, 0));
        assert!(matches!(result, Err(AttachError::VergeTooOld(build)) if build == "test-build"));
        assert_eq!(sent, std::slice::from_ref(&identify));
        // NXP's firmware: 0x82 without CJTAG.
        let (result, sent) = prepare(vec![]);
        assert!(matches!(result, Err(AttachError::NotVerge)));
        assert_eq!(sent, std::slice::from_ref(&identify));
    }

    #[test]
    fn info_parser_accepts_non_verge_firmware_and_rejects_truncated_fields() {
        let mut stock_reply = |_, _: &[u8]| Ok(vec![0x82]);
        assert_eq!(read_info(&mut stock_reply).unwrap(), None);

        let mut truncated_reply = |_, _: &[u8]| Ok(b"CJTAG\x06\x00\x05x".to_vec());
        assert!(read_info(&mut truncated_reply).is_err());
    }

    #[test]
    fn energy_capture_requests_and_replies_use_little_endian_fields() {
        assert_eq!(
            energy_capture_start_request(0x1234_5678),
            [0, 0x78, 0x56, 0x34, 0x12]
        );

        let start = parse_energy_capture_start(&[0, 0, 0x78, 0x56, 0x34, 0x12]).unwrap();
        assert_eq!(start.actual_rate_pairs_per_second, 0x1234_5678);

        let mut accumulator_reply = vec![0, 0];
        accumulator_reply.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
        accumulator_reply.extend_from_slice(&0x1112_1314_1516_1718u64.to_le_bytes());
        let total = EnergyAccumulator {
            energy_nanojoules: 0x0102_0304_0506_0708,
            integrated_pairs: 0x1112_1314_1516_1718,
        };
        assert_eq!(
            parse_energy_accumulator(&accumulator_reply).unwrap(),
            EnergyTotal::Total(total)
        );
    }

    #[test]
    fn an_acquisition_failure_keeps_the_retained_total() {
        let reply = |status: u8, energy: u64, pairs: u64| {
            let mut reply = vec![0, status];
            reply.extend_from_slice(&energy.to_le_bytes());
            reply.extend_from_slice(&pairs.to_le_bytes());
            reply
        };
        let total = EnergyAccumulator {
            energy_nanojoules: 77_189_068,
            integrated_pairs: 50_046,
        };
        assert_eq!(
            parse_energy_accumulator(&reply(7, 77_189_068, 50_046)).unwrap(),
            EnergyTotal::Failed(total)
        );
        assert_eq!(
            parse_energy_accumulator(&reply(1, 0, 0)).unwrap(),
            EnergyTotal::Stopped
        );
        assert_eq!(
            parse_energy_accumulator(&reply(2, 0, 0)).unwrap(),
            EnergyTotal::NoSample
        );
        assert!(matches!(
            parse_energy_accumulator(&reply(6, 0, 0)),
            Err(CommandError::Firmware(EnergyError::AdcError))
        ));
    }

    #[test]
    fn energy_capture_sample_parses_raw_and_calibrated_values() {
        let mut reply = vec![0, 0];
        reply.extend_from_slice(&0x1234u16.to_le_bytes());
        reply.extend_from_slice(&0x5678u16.to_le_bytes());
        reply.extend_from_slice(&0x1122_3344u32.to_le_bytes());
        reply.extend_from_slice(&3300u32.to_le_bytes());
        reply.extend_from_slice(&(-1_250_250i32).to_le_bytes());
        reply.extend_from_slice(&[3, 1]);

        let EnergyCaptureSample::Sample(sample) =
            parse_energy_capture_sample(&reply, true).unwrap()
        else {
            panic!("expected sample")
        };
        assert_eq!(
            sample.raw,
            RawEnergySample {
                voltage_code: 0x1234,
                current_code: 0x5678,
                sequence: 0x1122_3344,
            }
        );
        assert_eq!(
            sample.calibrated,
            Some(CalibratedEnergySample {
                voltage_millivolts: 3300,
                current_nanoamps: -1_250_250,
                range_id: 3,
            })
        );

        let EnergyCaptureSample::Sample(raw_only) =
            parse_energy_capture_sample(&reply[..10], false).unwrap()
        else {
            panic!("expected raw sample")
        };
        assert_eq!(raw_only.raw, sample.raw);
        assert_eq!(raw_only.calibrated, None);

        let mut invalid_calibrated_flag = reply.clone();
        invalid_calibrated_flag[19] = 2;
        assert!(parse_energy_capture_sample(&invalid_calibrated_flag, true).is_err());

        let mut no_sample_reply = reply;
        no_sample_reply[1] = 2;
        no_sample_reply[19] = 0;
        let no_sample = parse_energy_capture_sample(&no_sample_reply, true).unwrap();
        assert_eq!(no_sample, EnergyCaptureSample::NoSample);
    }

    #[test]
    fn calibration_points_stop_at_the_count_and_a_failed_calibration_keeps_them() {
        let reply = |status: u8, count: u8| {
            let mut reply = vec![0, status, count];
            for (code, nanoamps) in [(26u16, 0u32), (1_590, 496_000), (3_150, 1_000_000), (0, 0)] {
                reply.extend_from_slice(&code.to_le_bytes());
                reply.extend_from_slice(&nanoamps.to_le_bytes());
            }
            reply
        };
        let point = |code, current_nanoamps| CalibrationPoint {
            code,
            current_nanoamps,
        };
        assert_eq!(
            parse_energy_calibration(&reply(0, 3)).unwrap(),
            EnergyCalibration::Calibrated(vec![
                point(26, 0),
                point(1_590, 496_000),
                point(3_150, 1_000_000)
            ])
        );
        assert_eq!(
            parse_energy_calibration(&reply(6, 1)).unwrap(),
            EnergyCalibration::Failed(vec![point(26, 0)])
        );
        assert!(matches!(
            parse_energy_calibration(&reply(0, 5)),
            Err(CommandError::Malformed(_))
        ));
        assert!(matches!(
            parse_energy_calibration(&reply(3, 0)),
            Err(CommandError::Firmware(EnergyError::Unavailable))
        ));
        assert!(matches!(
            parse_energy_calibration(&[0xFF, 0]),
            Err(CommandError::Unsupported(_))
        ));
        assert!(parse_energy_calibration(&reply(0, 3)[..26]).is_err());
    }

    /// Replies in order, `None` for an exchange that times out; counts drains.
    struct ScriptedLink(VecDeque<Option<Vec<u8>>>, usize);

    impl DapLink for ScriptedLink {
        fn exchange(&mut self, _request: &[u8]) -> Result<Vec<u8>, crate::dap::LinkError> {
            self.0
                .pop_front()
                .unwrap()
                .ok_or(crate::dap::LinkError::Timeout)
        }

        fn drain(&mut self) -> Result<usize, crate::dap::LinkError> {
            self.1 += 1;
            Ok(0)
        }

        fn control_out(
            &mut self,
            _request_type: u8,
            _request: u8,
            _value: u16,
            _index: u16,
        ) -> Result<(), crate::dap::LinkError> {
            Ok(())
        }
    }

    #[test]
    fn a_dap_link_answers_vendor_commands_as_probe_rs_does() {
        let info = Some(vec![0x00, 1, 8]);
        let replies = [
            Some(vec![0x8F, 0, 1]),
            Some(vec![0xFF]),
            // Another command's reply: a drain, then a DAP_Info that answers in step.
            Some(vec![0x90, 0]),
            info.clone(),
            // A timeout: a drain, then a DAP_Info whose reply is the late one, so a second drain.
            None,
            Some(vec![0x90, 0, 1]),
        ];
        let mut link = ScriptedLink(replies.into(), 0);
        let send = |link: &mut ScriptedLink, command_id| {
            DapTransport(link).send_vendor_command(command_id, &[2])
        };
        assert_eq!(send(&mut link, 0x8F).unwrap(), [0, 1]);
        assert!(matches!(
            send(&mut link, 0x8F),
            Err(DebugProbeError::CommandNotSupportedByProbe { .. })
        ));
        assert_eq!(link.1, 0, "an unknown command leaves nothing owed");
        assert!(matches!(
            send(&mut link, 0x8F),
            Err(DebugProbeError::Other(message)) if message.contains("out of step")
        ));
        assert_eq!(link.1, 1, "in step again after one drain");
        assert!(send(&mut link, 0x90).is_err());
        assert_eq!(
            link.1, 3,
            "a late reply in place of DAP_Info's answer takes a second drain"
        );
        assert!(link.0.is_empty());
        assert!(matches!(
            send(&mut link, 0x00),
            Err(DebugProbeError::Other(message)) if message.contains("not a vendor command")
        ));
    }

    #[test]
    fn energy_parsers_reject_failed_transport_status_and_short_replies() {
        assert!(parse_energy_status(&[0xFF, 1]).is_err());
        assert!(parse_energy_capture_start(&[0, 0]).is_err());
        assert!(parse_energy_capture_sample(&[0, 0, 1], false).is_err());
        assert!(parse_energy_accumulator(&[0, 0]).is_err());
    }

    #[test]
    fn sample_request_encodes_each_sample_position() {
        let request = |position| SampleRequest {
            low: 2,
            fill: 8,
            position,
            count: 0x1234,
            expected_dpidr: 0x1122_3344,
        };

        assert_eq!(
            encode_sample_request(request(SamplePosition::AfterEdge(3))).unwrap(),
            [2, 8, 3, 0x34, 0x12, 0x44, 0x33, 0x22, 0x11]
        );
        assert_eq!(
            encode_sample_request(request(SamplePosition::Lead(3))).unwrap()[2],
            0x83
        );
        assert_eq!(
            encode_sample_request(request(SamplePosition::BeforeEdge)).unwrap()[2],
            0xFF
        );
        assert!(encode_sample_request(request(SamplePosition::AfterEdge(9))).is_err());
        assert!(
            encode_sample_request(SampleRequest {
                fill: 33,
                ..request(SamplePosition::BeforeEdge)
            })
            .is_err()
        );
    }

    #[test]
    fn codegen_measurement_parser_handles_wire_and_unknown_values() {
        let mut swd = vec![1, 0xFE, 4, 5, 6];
        swd.extend_from_slice(&0x1122_3344u32.to_le_bytes());
        swd.extend_from_slice(&10u32.to_le_bytes());
        swd.extend_from_slice(&20u32.to_le_bytes());
        swd.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
        swd.extend_from_slice(&30u32.to_le_bytes());
        let mut reader = ResponseReader::new(&swd);
        let measurement = read_codegen_measurement(&mut reader, false).unwrap();
        assert!(measurement.automatic);
        assert_eq!(measurement.outcome, LatencyOutcome::Unknown(0xFE));
        assert_eq!(LatencyOutcome::from_byte(6), LatencyOutcome::Boundary);
        assert_eq!(measurement.wire, None);
        assert_eq!(measurement.id_code, 0x1122_3344);
        assert_eq!(measurement.consistent_latencies, 0x0102_0304_0506_0708);
        assert_eq!(measurement.rescues, 30);
        assert_eq!(reader.remaining(), 0);

        let mut jtag = vec![0, 1, 3, 5, 6, 2];
        jtag.extend_from_slice(&[0; 24]);
        let mut reader = ResponseReader::new(&jtag);
        let measurement = read_codegen_measurement(&mut reader, true).unwrap();
        assert_eq!(measurement.wire, Some(JtagWire::OScan5));
        assert_eq!(reader.remaining(), 0);
    }

    #[test]
    fn codegen_states_parse_legacy_and_extended_replies() {
        let mut swd_legacy = vec![0, 3, 1, 1];
        swd_legacy.extend_from_slice(&[0; 24]);
        assert_eq!(
            parse_swd_codegen_state(&swd_legacy).unwrap().1.measurement,
            None
        );

        let mut swd_extended = swd_legacy;
        swd_extended.extend_from_slice(&[0; 29]);
        assert!(
            parse_swd_codegen_state(&swd_extended)
                .unwrap()
                .1
                .measurement
                .is_some()
        );

        let mut jtag_legacy = vec![0, 3, 4, 1, 1];
        jtag_legacy.extend_from_slice(&[0; 24]);
        let (_, legacy) = parse_jtag_codegen_state(&jtag_legacy).unwrap();
        assert_eq!(legacy.measurement, None);
        assert_eq!(legacy.restores, None);

        let mut jtag_extended = jtag_legacy;
        jtag_extended.extend_from_slice(&[0; 30]);
        jtag_extended.extend_from_slice(&7u32.to_le_bytes());
        let (_, extended) = parse_jtag_codegen_state(&jtag_extended).unwrap();
        assert!(extended.measurement.is_some());
        assert_eq!(extended.restores, Some(7));
    }

    #[test]
    fn unknown_status_codes_are_preserved() {
        assert_eq!(VendorStatus::from_byte(0x7F), VendorStatus::Unknown(0x7F));
        assert_eq!(EnergyError::check(0x7F), Err(EnergyError::Unknown(0x7F)));
    }
}
