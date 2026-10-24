//! How long operations report what they are doing, and how they are stopped.

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// One stage of an update, in the order an update runs them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Step {
    Download,
    EnterBootloader,
    WaitForBootloader,
    Connect,
    Erase,
    Write,
    Verify,
    Reset,
    WaitForFirmware,
}

impl Step {
    /// What the step counts its progress in.
    #[must_use]
    pub fn unit(self) -> Unit {
        match self {
            Step::Download | Step::Erase | Step::Write | Step::Verify => Unit::Bytes,
            Step::EnterBootloader
            | Step::WaitForBootloader
            | Step::Connect
            | Step::Reset
            | Step::WaitForFirmware => Unit::None,
        }
    }
}

impl fmt::Display for Step {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Step::Download => "download the release",
            Step::EnterBootloader => "ask the firmware for its bootloader",
            Step::WaitForBootloader => "wait for the bootloader",
            Step::Connect => "connect to the bootloader",
            Step::Erase => "erase",
            Step::Write => "write",
            Step::Verify => "verify",
            Step::Reset => "reset",
            Step::WaitForFirmware => "wait for the firmware",
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Bytes,
    None,
}

/// Receives an operation's progress. Every method has a default that ignores it.
pub trait Progress {
    /// The steps about to run, before the first one starts.
    fn plan(&mut self, _steps: &[Step]) {}
    /// A step starts; `total` is in the step's [`Unit`], 0 when it has none.
    fn start(&mut self, _step: Step, _total: u64) {}
    /// How much of the current step is done, in its unit.
    fn advance(&mut self, _done: u64) {}
    /// Something the user should read, said once.
    fn note(&mut self, _text: &str) {}
}

/// Ignores everything.
pub struct Quiet;

impl Progress for Quiet {}

/// A request to stop, checked between pages, polls and steps.
///
/// Cancelling a write leaves the unit in its bootloader with a partial image, from which the next
/// flash recovers it.
#[derive(Clone, Debug, Default)]
pub struct Cancel(Arc<AtomicBool>);

impl Cancel {
    #[must_use]
    pub fn new() -> Cancel {
        Cancel::default()
    }

    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}
