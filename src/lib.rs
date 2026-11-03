//! Library API for discovering, debugging and updating MCU-Link probes running Verge or NXP
//! firmware. It combines the shared update workflows with probe-rs sessions and typed Verge
//! commands. It contains no CLI or GUI.
//!
//! USB update and bootloader operations use [`bus::Bus`]. Target debugging uses probe-rs. The
//! `sim` module provides an in-memory bench for tests and development with the `simulate` feature.

pub mod aux;
pub mod batch;
pub mod board;
pub mod bus;
pub mod cache;
mod capture_id;
pub use capture_id::CaptureId;
#[cfg(feature = "capture")]
pub mod capture;
pub mod dap;
pub mod daplink_rs;
pub mod detect;
pub mod hid;
pub mod ids;
pub mod image;
pub mod isp;
pub mod paths;
pub mod pro;
pub mod progress;
pub mod record;
pub mod release;
#[cfg(feature = "simulate")]
pub mod sim;
pub mod source;
pub mod stock;
pub mod stream;
pub mod update;
pub mod usb;
pub mod version;

pub use bus::{Bus, UsbDevice};
pub use detect::{Firmware, State, Unit, detect};
pub use image::Image;
pub use paths::{Offer, Path, offer};
pub use probe_rs;
pub use progress::{Cancel, Progress, Step};
pub use source::{Loaded, Source};
pub use update::{Outcome, Timing, UpdateError};

mod protocol;

mod probe;
pub use probe::{
    AttachError, BoardKind, CalibratedEnergySample, CalibrationPoint, CalibrationRange,
    CodeLabArena, CodegenMeasurement, CodegenMode, CommandError, DapTransport, DioState,
    EnergyAccumulator, EnergyCalibration, EnergyCaptureSample, EnergyCaptureStart,
    EnergyCaptureStatus, EnergyError, EnergyMonitorError, EnergySample, EnergyTotal,
    JtagCodegenState, JtagGenerationLogEntry, JtagPacingState, JtagWire, LatencyOutcome, Marked,
    ProBusError, RawEnergySample, Refused, SamplePosition, SampleRequest, SampleResult, SpiMode,
    SwdCodegenState, TargetSenseError, TargetSenseReading, VendorStatus, VergeControl, VergeInfo,
    VergeProbe, VergeSession, VergeTransport, attach, install_cjtag_support,
    set_cjtag_scan_formats,
};
