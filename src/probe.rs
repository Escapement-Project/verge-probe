//! Host operations for MCU-Link probes running the Verge firmware.

use probe_rs::{
    Session,
    probe::{
        DebugProbeError, Probe, ScanFormat,
        cmsisdap::{CmsisDap, CmsisDapCjtagExtension},
    },
};
use std::time::Duration;
use thiserror::Error;

use crate::protocol::{
    self, CAP_DIO, CAP_ENERGY_ACCUMULATOR, CAP_ENERGY_CAPTURE, CAP_I2C, CAP_JTAG_CODEGEN, CAP_SPI,
    CAP_SWD_CODEGEN, CAP_TARGET_POWER, CAP_TARGET_SENSE, CJTAG_CONTROL_COMMAND, CODE_LAB_COMMAND,
    CPU_STAGING_COMMAND, DIO_COMMAND, ENERGY_COMMAND, I2C_COMMAND, I2C_MAX_CLOCK_HZ, I2C_MAX_READ,
    I2C_MAX_WRITE, I2C_MIN_CLOCK_HZ, INFO_COMMAND, J8_COMMAND, JTAG_CODEGEN_COMMAND,
    JTAG_LOG_COMMAND, JTAG_PACING_COMMAND, REBOOT_COMMAND, SAMPLE_COMMAND, SAMPLE_FILL_MAX,
    SPI_COMMAND, SPI_MAX_CLOCK_HZ, SPI_MAX_TRANSFER, SPI_MIN_CLOCK_HZ, SWD_CODEGEN_COMMAND,
    TAP_RUN_TEST_IDLE, TARGET_POWER_COMMAND, TARGET_SENSE_COMMAND,
};
pub use crate::protocol::{BoardKind, DioState, VendorStatus, VergeInfo};

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

/// Result code returned by the Pro SPI and I2C bridges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ProBusStatus {
    /// The transfer completed.
    Ok,
    /// The bridge is unavailable on this board or with the current jumper setting.
    Unavailable,
    /// The request arguments are not supported.
    Invalid,
    /// Another firmware feature currently owns the bus pins.
    Busy,
    /// The peripheral transfer failed.
    TransferError,
    /// The I2C target did not acknowledge.
    Nack,
    /// An unrecognized result code.
    Unknown(u8),
}

impl ProBusStatus {
    fn from_byte(value: u8) -> Self {
        match value {
            0 => Self::Ok,
            1 => Self::Unavailable,
            2 => Self::Invalid,
            3 => Self::Busy,
            4 => Self::TransferError,
            5 => Self::Nack,
            other => Self::Unknown(other),
        }
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

/// Result of one full-duplex Pro SPI transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpiTransferResult {
    /// Firmware result code.
    pub status: ProBusStatus,
    /// Bytes received during the transfer.
    pub rx: Vec<u8>,
}

/// Result of one Pro I2C transaction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I2cTransferResult {
    /// Firmware result code.
    pub status: ProBusStatus,
    /// Bytes read during the transaction.
    pub rx: Vec<u8>,
}

/// Why a background energy monitor stopped.
#[derive(Debug, Error)]
pub enum EnergyMonitorError {
    /// The polling interval must be greater than zero.
    #[error("energy monitor poll interval must be greater than zero")]
    InvalidPollInterval,
    /// Sending the start request failed.
    #[error("starting energy capture failed: {0}")]
    StartProbe(#[source] DebugProbeError),
    /// The firmware refused to start capture.
    #[error("energy capture start returned {0:?}")]
    Start(EnergyStatus),
    /// A poll failed. The monitor still attempted to stop capture.
    #[error("energy sample poll failed: {0}")]
    Poll(#[source] DebugProbeError),
    /// Polling failed and stopping capture also failed.
    #[error("energy poll failed: {poll}; stopping capture returned {stop:?}")]
    PollAndStop {
        #[source]
        poll: DebugProbeError,
        stop: EnergyStatus,
    },
    /// Polling and stopping both failed at the probe transport.
    #[error("energy poll failed: {poll}; stopping capture failed: {stop}")]
    PollAndStopProbe {
        poll: DebugProbeError,
        stop: DebugProbeError,
    },
    /// Stopping the probe failed.
    #[error("stopping energy capture failed: {0}")]
    StopProbe(#[source] DebugProbeError),
    /// The firmware refused to stop capture.
    #[error("energy capture stop returned {0:?}")]
    Stop(EnergyStatus),
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

/// A CMSIS-DAP target that accepts Verge vendor commands.
pub trait VergeTransport {
    /// Sends one CMSIS-DAP vendor command and returns its reply.
    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError>;
}

impl VergeTransport for Probe {
    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        Probe::send_vendor_command(self, command_id, payload)
    }
}

impl VergeTransport for Session {
    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        Session::send_vendor_command(self, command_id, payload)
    }
}

/// Verge firmware operations on an opened probe or an attached probe-rs session.
pub struct VergeControl<'a, T: VergeTransport> {
    transport: &'a mut T,
    info_loaded: bool,
    info: Option<VergeInfo>,
}

/// Verge commands sent through an opened probe, before attaching to its target.
pub type VergeProbe<'a> = VergeControl<'a, Probe>;

/// Verge commands sent through an attached probe-rs session.
pub type VergeSession<'a> = VergeControl<'a, Session>;

impl<'a, T: VergeTransport> VergeControl<'a, T> {
    /// Creates a Verge command client for an opened probe or attached session.
    pub fn new(transport: &'a mut T) -> Self {
        Self {
            transport,
            info_loaded: false,
            info: None,
        }
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
            self.transport.send_vendor_command(command_id, payload)
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
        self.transport.send_vendor_command(command_id, payload)
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
        let response = self.send(DIO_COMMAND, &protocol::encode_dio(mask, outputs, levels))?;
        protocol::decode_dio(&response).map_err(protocol_error)
    }

    /// Runs one full-duplex transaction on the Pro SPI bridge.
    pub fn spi_transfer(
        &mut self,
        clock_hz: u32,
        mode: SpiMode,
        lsb_first: bool,
        tx: &[u8],
    ) -> Result<SpiTransferResult, DebugProbeError> {
        self.require_capability(CAP_SPI, "SPI bridge")?;
        let request = encode_spi_request(clock_hz, mode, lsb_first, tx)?;
        parse_spi_response(&self.send(SPI_COMMAND, &request)?, tx.len())
    }

    /// Runs one Pro I2C write, read, or combined write/read transaction.
    pub fn i2c_transfer(
        &mut self,
        address: u8,
        clock_hz: u32,
        write: &[u8],
        read_len: usize,
    ) -> Result<I2cTransferResult, DebugProbeError> {
        self.require_capability(CAP_I2C, "I2C bridge")?;
        let request = encode_i2c_request(address, clock_hz, write, read_len)?;
        parse_i2c_response(&self.send(I2C_COMMAND, &request)?, read_len)
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
        let request = energy_capture_start_request(rate_pairs_per_second);
        parse_energy_capture_start(&self.send(ENERGY_COMMAND, &request)?)
    }

    /// Polls distinct energy samples until `should_stop` returns true, then stops capture.
    ///
    /// Capture continues in probe firmware between polls. The poll interval controls host traffic;
    /// samples between polls remain included in the probe's energy accumulator.
    pub fn monitor_energy<Stop, Sample>(
        &mut self,
        rate_pairs_per_second: u32,
        poll_interval: Duration,
        mut should_stop: Stop,
        mut on_sample: Sample,
    ) -> Result<(), EnergyMonitorError>
    where
        Stop: FnMut() -> bool,
        Sample: FnMut(EnergyCaptureSample),
    {
        if poll_interval.is_zero() {
            return Err(EnergyMonitorError::InvalidPollInterval);
        }
        let start = self
            .start_energy_capture(rate_pairs_per_second)
            .map_err(EnergyMonitorError::StartProbe)?;
        if start.status != EnergyStatus::Ok {
            return Err(EnergyMonitorError::Start(start.status));
        }

        let mut last_sequence = None;
        let poll_result = loop {
            if should_stop() {
                break Ok(());
            }
            std::thread::sleep(poll_interval);
            if should_stop() {
                break Ok(());
            }
            match self.latest_energy_sample() {
                Ok(sample) => {
                    if sample.status == EnergyStatus::Ok
                        && let Some(raw) = sample.sample
                        && last_sequence != Some(raw.sequence)
                    {
                        last_sequence = Some(raw.sequence);
                        on_sample(sample);
                    } else if !matches!(sample.status, EnergyStatus::Ok | EnergyStatus::NoSample) {
                        break Err(protocol_error("energy sample acquisition failed"));
                    }
                }
                Err(error) => break Err(error),
            }
        };

        let stop_result = self.stop_energy_capture();
        match (poll_result, stop_result) {
            (Err(poll), Ok(EnergyStatus::Ok)) => Err(EnergyMonitorError::Poll(poll)),
            (Err(poll), Ok(status)) => Err(EnergyMonitorError::PollAndStop { poll, stop: status }),
            (Err(poll), Err(stop)) => Err(EnergyMonitorError::PollAndStopProbe { poll, stop }),
            (Ok(()), Err(error)) => Err(EnergyMonitorError::StopProbe(error)),
            (Ok(()), Ok(EnergyStatus::Ok)) => Ok(()),
            (Ok(()), Ok(status)) => Err(EnergyMonitorError::Stop(status)),
        }
    }

    /// Stops voltage/current capture.
    pub fn stop_energy_capture(&mut self) -> Result<EnergyStatus, DebugProbeError> {
        self.require_capability(CAP_ENERGY_CAPTURE, "energy capture")?;
        let response = self.send(ENERGY_COMMAND, &[2])?;
        parse_energy_status(&response)
    }

    /// Reads the latest complete voltage/current pair.
    pub fn latest_energy_sample(&mut self) -> Result<EnergyCaptureSample, DebugProbeError> {
        let calibrated_supported = self
            .require_capability(CAP_ENERGY_CAPTURE, "energy capture")?
            .supports_calibrated_energy();
        let response = self.send(ENERGY_COMMAND, &[1])?;
        parse_energy_capture_sample(&response, calibrated_supported)
    }

    /// Reads the energy total without resetting it.
    pub fn read_energy_accumulator(&mut self) -> Result<EnergyAccumulator, DebugProbeError> {
        self.require_capability(CAP_ENERGY_ACCUMULATOR, "energy accumulator")?;
        let response = self.send(ENERGY_COMMAND, &[3])?;
        parse_energy_accumulator(&response)
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
        let payload = encode_sample_request(request)?;
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
        parse_swd_codegen_state(&response)
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
        parse_jtag_codegen_state(&response)
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
        self.transport.send_vendor_command(command_id, payload)
    }
}

impl VergeControl<'_, Session> {
    /// Returns the underlying probe-rs session for ordinary target operations.
    pub fn session(&mut self) -> &mut Session {
        self.transport
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
    let request = protocol::encode_scan_formats(formats);
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
    protocol::decode_info(&reply).map_err(protocol_error)
}

fn check_dap_status(response: &[u8]) -> Result<(), DebugProbeError> {
    check_vendor_status(response_byte(response, 0)?)
}

fn energy_capture_start_request(rate_pairs_per_second: u32) -> [u8; 5] {
    let mut request = [0; 5];
    request[1..].copy_from_slice(&rate_pairs_per_second.to_le_bytes());
    request
}

fn parse_energy_capture_start(response: &[u8]) -> Result<EnergyCaptureStart, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
    check_vendor_status(reader.u8()?)?;
    Ok(EnergyCaptureStart {
        status: EnergyStatus::from_byte(reader.u8()?),
        actual_rate_pairs_per_second: reader.u32()?,
    })
}

fn parse_energy_status(response: &[u8]) -> Result<EnergyStatus, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
    check_vendor_status(reader.u8()?)?;
    Ok(EnergyStatus::from_byte(reader.u8()?))
}

fn parse_energy_capture_sample(
    response: &[u8],
    calibrated_supported: bool,
) -> Result<EnergyCaptureSample, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
    check_vendor_status(reader.u8()?)?;
    let status = EnergyStatus::from_byte(reader.u8()?);
    let raw_sample = RawEnergySample {
        voltage_code: reader.u16()?,
        current_code: reader.u16()?,
        sequence: reader.u32()?,
    };
    let sample = (status == EnergyStatus::Ok).then_some(raw_sample);
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

fn parse_energy_accumulator(response: &[u8]) -> Result<EnergyAccumulator, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
    check_vendor_status(reader.u8()?)?;
    Ok(EnergyAccumulator {
        status: EnergyStatus::from_byte(reader.u8()?),
        energy_nanojoules: reader.u64()?,
        integrated_pairs: reader.u64()?,
    })
}

fn encode_spi_request(
    clock_hz: u32,
    mode: SpiMode,
    lsb_first: bool,
    tx: &[u8],
) -> Result<Vec<u8>, DebugProbeError> {
    if !(SPI_MIN_CLOCK_HZ..=SPI_MAX_CLOCK_HZ).contains(&clock_hz) {
        return Err(protocol_error("SPI clock is outside the supported range"));
    }
    if tx.is_empty() || tx.len() > SPI_MAX_TRANSFER {
        return Err(protocol_error(
            "SPI transfer length is outside the supported range",
        ));
    }
    let count = u16::try_from(tx.len()).map_err(|_| protocol_error("SPI transfer is too long"))?;
    let mut request = Vec::with_capacity(7 + tx.len());
    request.push(mode.bits() | if lsb_first { 1 << 2 } else { 0 });
    request.extend_from_slice(&clock_hz.to_le_bytes());
    request.extend_from_slice(&count.to_le_bytes());
    request.extend_from_slice(tx);
    Ok(request)
}

fn parse_spi_response(
    response: &[u8],
    expected_count: usize,
) -> Result<SpiTransferResult, DebugProbeError> {
    let status = ProBusStatus::from_byte(response_byte(response, 0)?);
    let count = usize::from(u16::from_le_bytes(response_array(response, 1..3)?));
    let data = response
        .get(3..3 + count)
        .ok_or_else(|| protocol_error("short SPI data response"))?;
    if status == ProBusStatus::Ok && count != expected_count {
        return Err(protocol_error(
            "SPI response length does not match the request",
        ));
    }
    if status != ProBusStatus::Ok && count != 0 {
        return Err(protocol_error("failed SPI response contains data"));
    }
    Ok(SpiTransferResult {
        status,
        rx: data.to_vec(),
    })
}

fn encode_i2c_request(
    address: u8,
    clock_hz: u32,
    write: &[u8],
    read_len: usize,
) -> Result<Vec<u8>, DebugProbeError> {
    if address > 0x7f {
        return Err(protocol_error("I2C address is not 7-bit"));
    }
    if !(I2C_MIN_CLOCK_HZ..=I2C_MAX_CLOCK_HZ).contains(&clock_hz) {
        return Err(protocol_error("I2C clock is outside the supported range"));
    }
    if write.is_empty() && read_len == 0 {
        return Err(protocol_error("I2C transfer is empty"));
    }
    if write.len() > I2C_MAX_WRITE || read_len > I2C_MAX_READ {
        return Err(protocol_error(
            "I2C transfer is longer than the supported packet size",
        ));
    }
    let write_count =
        u16::try_from(write.len()).map_err(|_| protocol_error("I2C write is too long"))?;
    let read_count = u16::try_from(read_len).map_err(|_| protocol_error("I2C read is too long"))?;
    let mut request = Vec::with_capacity(9 + write.len());
    request.push(address);
    request.extend_from_slice(&clock_hz.to_le_bytes());
    request.extend_from_slice(&write_count.to_le_bytes());
    request.extend_from_slice(&read_count.to_le_bytes());
    request.extend_from_slice(write);
    Ok(request)
}

fn parse_i2c_response(
    response: &[u8],
    expected_count: usize,
) -> Result<I2cTransferResult, DebugProbeError> {
    let status = ProBusStatus::from_byte(response_byte(response, 0)?);
    let count = usize::from(u16::from_le_bytes(response_array(response, 1..3)?));
    let data = response
        .get(3..3 + count)
        .ok_or_else(|| protocol_error("short I2C data response"))?;
    if status == ProBusStatus::Ok && count != expected_count {
        return Err(protocol_error(
            "I2C response length does not match the request",
        ));
    }
    if status != ProBusStatus::Ok && count != 0 {
        return Err(protocol_error("failed I2C response contains data"));
    }
    Ok(I2cTransferResult {
        status,
        rx: data.to_vec(),
    })
}

fn parse_swd_codegen_state(response: &[u8]) -> Result<SwdCodegenState, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
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

fn parse_jtag_codegen_state(response: &[u8]) -> Result<JtagCodegenState, DebugProbeError> {
    let mut reader = ResponseReader::new(response);
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

fn encode_sample_request(request: SampleRequest) -> Result<Vec<u8>, DebugProbeError> {
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

    fn info_reply(
        version: u8,
        capabilities: u8,
        board: u8,
        scan_formats: u32,
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
            reply.extend_from_slice(&scan_formats.to_le_bytes());
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

        let result = verge.set_target_power(true).unwrap();

        assert!(result.enabled);
        assert_eq!(
            transport.commands,
            [(INFO_COMMAND, vec![]), (TARGET_POWER_COMMAND, vec![1])]
        );
    }

    #[test]
    fn spi_request_and_reply_enforce_encoding_and_counts() {
        let request = encode_spi_request(100_000, SpiMode::Mode3, true, &[0x12, 0x34]).unwrap();
        assert_eq!(&request[..7], &[7, 0xA0, 0x86, 1, 0, 2, 0]);
        assert_eq!(&request[7..], &[0x12, 0x34]);
        assert!(encode_spi_request(1, SpiMode::Mode0, false, &[0]).is_err());
        assert!(encode_spi_request(100_000, SpiMode::Mode0, false, &[]).is_err());
        assert!(
            encode_spi_request(100_000, SpiMode::Mode0, false, &vec![0; SPI_MAX_TRANSFER]).is_ok()
        );
        assert!(
            encode_spi_request(
                100_000,
                SpiMode::Mode0,
                false,
                &vec![0; SPI_MAX_TRANSFER + 1]
            )
            .is_err()
        );

        let parsed = parse_spi_response(&[0, 2, 0, 0xA5, 0x5A], 2).unwrap();
        assert_eq!(parsed.status, ProBusStatus::Ok);
        assert_eq!(parsed.rx, [0xA5, 0x5A]);
        assert!(parse_spi_response(&[0, 1, 0, 0xA5], 2).is_err());
        assert_eq!(
            parse_spi_response(&[3, 0, 0], 2).unwrap().status,
            ProBusStatus::Busy
        );
    }

    #[test]
    fn i2c_request_and_reply_enforce_ranges_and_counts() {
        let request = encode_i2c_request(0x50, 400_000, &[0x10], 2).unwrap();
        assert_eq!(request, [0x50, 0x80, 0x1A, 0x06, 0, 1, 0, 2, 0, 0x10]);
        assert!(encode_i2c_request(0x80, 100_000, &[], 1).is_err());
        assert!(encode_i2c_request(0x50, 1_000, &[], 1).is_err());
        assert!(encode_i2c_request(0x50, 100_000, &[], 0).is_err());
        assert!(encode_i2c_request(0x50, 100_000, &vec![0; I2C_MAX_WRITE + 1], 0).is_err());
        assert!(encode_i2c_request(0x50, 100_000, &[], I2C_MAX_READ + 1).is_err());

        let parsed = parse_i2c_response(&[0, 2, 0, 0xAA, 0xBB], 2).unwrap();
        assert_eq!(parsed.status, ProBusStatus::Ok);
        assert_eq!(parsed.rx, [0xAA, 0xBB]);
        assert_eq!(
            parse_i2c_response(&[5, 0, 0], 1).unwrap().status,
            ProBusStatus::Nack
        );
        assert!(parse_i2c_response(&[0, 1, 0, 0xAA], 2).is_err());
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

    #[test]
    fn energy_monitor_delivers_new_samples_and_stops_capture() {
        let mut latest = vec![0, 0];
        latest.extend_from_slice(&1u16.to_le_bytes());
        latest.extend_from_slice(&2u16.to_le_bytes());
        latest.extend_from_slice(&7u32.to_le_bytes());
        latest.extend_from_slice(&3300u32.to_le_bytes());
        latest.extend_from_slice(&100i32.to_le_bytes());
        latest.extend_from_slice(&[0, 1]);
        let mut transport = FakeTransport {
            replies: [
                info_reply(
                    6,
                    CAP_TARGET_POWER,
                    1,
                    0,
                    CAP_ENERGY_CAPTURE | CAP_CALIBRATED_ENERGY,
                ),
                [vec![0, 0], 1000u32.to_le_bytes().to_vec()].concat(),
                latest,
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
                    sample_running.set(false);
                },
            )
            .unwrap();

        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].sample.unwrap().sequence, 7);
        assert_eq!(
            transport.commands[1],
            (ENERGY_COMMAND, vec![0, 0xE8, 3, 0, 0])
        );
        assert_eq!(transport.commands[2], (ENERGY_COMMAND, vec![1]));
        assert_eq!(transport.commands[3], (ENERGY_COMMAND, vec![2]));
    }

    #[test]
    fn energy_monitor_stops_capture_after_a_malformed_poll_reply() {
        let mut transport = FakeTransport {
            replies: [
                info_reply(6, 0, 1, 0, CAP_ENERGY_CAPTURE),
                [vec![0, 0], 1000u32.to_le_bytes().to_vec()].concat(),
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
        assert_eq!(transport.commands.last(), Some(&(ENERGY_COMMAND, vec![2])));
    }

    #[test]
    fn info_parser_reads_only_fields_supported_by_the_format() {
        for (version, board, scan_formats, extended_capabilities) in [
            (1, None, 0, 0),
            (3, Some(BoardKind::Pro), 0, 0),
            (4, Some(BoardKind::Pro), 0x204, 0),
            (6, Some(BoardKind::Pro), 0x204, 0x7),
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
                (Err(_), Err(_)) => {}
                replies => panic!("decoders disagree: {replies:?}"),
            }
        }
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
        assert_eq!(start.status, EnergyStatus::Ok);
        assert_eq!(start.actual_rate_pairs_per_second, 0x1234_5678);

        let mut accumulator_reply = vec![0, 0];
        accumulator_reply.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
        accumulator_reply.extend_from_slice(&0x1112_1314_1516_1718u64.to_le_bytes());
        let accumulator = parse_energy_accumulator(&accumulator_reply).unwrap();
        assert_eq!(accumulator.energy_nanojoules, 0x0102_0304_0506_0708);
        assert_eq!(accumulator.integrated_pairs, 0x1112_1314_1516_1718);
    }

    #[test]
    fn energy_capture_sample_parses_raw_and_calibrated_values() {
        let mut reply = vec![0, 0];
        reply.extend_from_slice(&0x1234u16.to_le_bytes());
        reply.extend_from_slice(&0x5678u16.to_le_bytes());
        reply.extend_from_slice(&0x1122_3344u32.to_le_bytes());
        reply.extend_from_slice(&3300u32.to_le_bytes());
        reply.extend_from_slice(&(-1250i32).to_le_bytes());
        reply.extend_from_slice(&[3, 1]);

        let sample = parse_energy_capture_sample(&reply, true).unwrap();
        assert_eq!(sample.status, EnergyStatus::Ok);
        assert_eq!(
            sample.sample,
            Some(RawEnergySample {
                voltage_code: 0x1234,
                current_code: 0x5678,
                sequence: 0x1122_3344,
            })
        );
        assert_eq!(
            sample.calibrated,
            Some(CalibratedEnergySample {
                voltage_millivolts: 3300,
                current_microamps: -1250,
                range_id: 3,
            })
        );

        let raw_only = parse_energy_capture_sample(&reply[..10], false).unwrap();
        assert_eq!(raw_only.sample, sample.sample);
        assert_eq!(raw_only.calibrated, None);

        let mut invalid_calibrated_flag = reply.clone();
        invalid_calibrated_flag[19] = 2;
        assert!(parse_energy_capture_sample(&invalid_calibrated_flag, true).is_err());

        let mut no_sample_reply = reply;
        no_sample_reply[1] = 2;
        no_sample_reply[19] = 0;
        let no_sample = parse_energy_capture_sample(&no_sample_reply, true).unwrap();
        assert_eq!(no_sample.status, EnergyStatus::NoSample);
        assert_eq!(no_sample.sample, None);
        assert_eq!(no_sample.calibrated, None);
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
            parse_swd_codegen_state(&swd_legacy).unwrap().measurement,
            None
        );

        let mut swd_extended = swd_legacy;
        swd_extended.extend_from_slice(&[0; 29]);
        assert!(
            parse_swd_codegen_state(&swd_extended)
                .unwrap()
                .measurement
                .is_some()
        );

        let mut jtag_legacy = vec![0, 3, 4, 1, 1];
        jtag_legacy.extend_from_slice(&[0; 24]);
        let legacy = parse_jtag_codegen_state(&jtag_legacy).unwrap();
        assert_eq!(legacy.measurement, None);
        assert_eq!(legacy.restores, None);

        let mut jtag_extended = jtag_legacy;
        jtag_extended.extend_from_slice(&[0; 30]);
        jtag_extended.extend_from_slice(&7u32.to_le_bytes());
        let extended = parse_jtag_codegen_state(&jtag_extended).unwrap();
        assert!(extended.measurement.is_some());
        assert_eq!(extended.restores, Some(7));
    }

    #[test]
    fn unknown_status_codes_are_preserved() {
        assert_eq!(VendorStatus::from_byte(0x7F), VendorStatus::Unknown(0x7F));
        assert_eq!(EnergyStatus::from_byte(0x7F), EnergyStatus::Unknown(0x7F));
    }
}
