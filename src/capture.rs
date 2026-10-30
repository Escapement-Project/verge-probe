//! Disk-backed energy captures. Intervals are half-open and samples cover one nominal period.
//! Gaps never contribute to charge, energy or time-weighted means.

mod recorder;
mod store;
pub use recorder::Recorder;
pub use store::{Archive, CaptureWriter, Envelope, View, recovered_sessions, remove};

use crate::record::RecordError;
use crate::stream::Sample;
use serde::{Deserialize, Serialize};
use std::io;
use std::path::PathBuf;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CaptureError {
    #[error("not a Verge capture")]
    NotCapture,
    #[error("a Verge capture in format {0}, which this version cannot read")]
    Version(i64),
    #[error("{} already exists", .0.display())]
    Exists(PathBuf),
    #[error("{} is not a UTF-8 path", .0.display())]
    Path(PathBuf),
    #[error("the capture restarted before its stream ended")]
    Restarted,
    #[error("a stream of {0} pairs a second, which a capture cannot hold")]
    Rate(u32),
    #[error("the stream's times went back")]
    Timestamps,
    #[error("a time past what a capture holds")]
    Time,
    #[error("cancelled")]
    Cancelled,
    #[error("the recording is closed")]
    Closed,
    /// The disk queue filled: the disk cannot keep up with the stream.
    #[error("the capture's disk writer fell behind the stream")]
    Behind,
    #[error("the capture's disk writer stopped")]
    Stopped,
    #[error("the capture's disk writer panicked")]
    Panicked,
    #[error(transparent)]
    Sqlite(#[from] rusqlite::Error),
    #[error("a capture record: {0}")]
    Encoding(#[from] postcard::Error),
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error(transparent)]
    Record(#[from] RecordError),
}

type Result<T> = std::result::Result<T, CaptureError>;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Interval {
    pub start_ns: u64,
    pub end_ns: u64,
}

impl Interval {
    pub fn new(start_ns: u64, end_ns: u64) -> Self {
        Self {
            start_ns,
            end_ns: end_ns.max(start_ns),
        }
    }
    pub fn duration_ns(self) -> u64 {
        self.end_ns.saturating_sub(self.start_ns)
    }
}

/// Integer weighted sums are kept separately from rendering summaries.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Statistics {
    pub samples: u64,
    pub measured_ns: u64,
    pub current_na_ns: u128,
    pub voltage_mv_ns: u128,
    pub power_mv_na_ns: u128,
    pub min_na: Option<u32>,
    pub max_na: Option<u32>,
    pub min_mv: Option<u32>,
    pub max_mv: Option<u32>,
    pub min_mv_na: Option<u64>,
    pub max_mv_na: Option<u64>,
}

impl Statistics {
    pub fn merge(&mut self, other: &Self) {
        self.samples += other.samples;
        self.measured_ns += other.measured_ns;
        self.current_na_ns += other.current_na_ns;
        self.voltage_mv_ns += other.voltage_mv_ns;
        self.power_mv_na_ns += other.power_mv_na_ns;
        macro_rules! extrema {
            ($name:ident, $method:ident) => {
                self.$name = match (self.$name, other.$name) {
                    (Some(a), Some(b)) => Some(a.$method(b)),
                    (a, b) => a.or(b),
                };
            };
        }
        extrema!(min_na, min);
        extrema!(max_na, max);
        extrema!(min_mv, min);
        extrema!(max_mv, max);
        extrema!(min_mv_na, min);
        extrema!(max_mv_na, max);
    }
    pub fn add(&mut self, sample: Sample, end_ns: u64, interval: Interval) {
        let duration = end_ns
            .min(interval.end_ns)
            .saturating_sub(sample.time_ns.max(interval.start_ns));
        if duration == 0 {
            return;
        }
        let power = u64::from(sample.millivolts) * u64::from(sample.nanoamps);
        self.merge(&Self {
            samples: 1,
            measured_ns: duration,
            current_na_ns: u128::from(sample.nanoamps) * u128::from(duration),
            voltage_mv_ns: u128::from(sample.millivolts) * u128::from(duration),
            power_mv_na_ns: u128::from(power) * u128::from(duration),
            min_na: Some(sample.nanoamps),
            max_na: Some(sample.nanoamps),
            min_mv: Some(sample.millivolts),
            max_mv: Some(sample.millivolts),
            min_mv_na: Some(power),
            max_mv_na: Some(power),
        });
    }
    pub fn mean_current_a(&self) -> Option<f64> {
        (self.measured_ns > 0).then(|| self.current_na_ns as f64 / self.measured_ns as f64 * 1e-9)
    }
    pub fn mean_voltage_v(&self) -> Option<f64> {
        (self.measured_ns > 0).then(|| self.voltage_mv_ns as f64 / self.measured_ns as f64 * 1e-3)
    }
    pub fn mean_power_w(&self) -> Option<f64> {
        (self.measured_ns > 0).then(|| self.power_mv_na_ns as f64 / self.measured_ns as f64 * 1e-12)
    }
    pub fn charge_c(&self) -> f64 {
        self.current_na_ns as f64 * 1e-18
    }
    pub fn energy_j(&self) -> f64 {
        self.power_mv_na_ns as f64 * 1e-21
    }
}
