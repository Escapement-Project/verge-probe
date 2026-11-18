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
            Step::EnterBootloader => "ask for the bootloader",
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

/// What a step's progress counts.
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

/// What a cancel left on the unit, by the part of the update it stopped in. A cancel before the
/// unit is asked for its bootloader is [`crate::UpdateError::Stopped`] instead: that changes
/// nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Left {
    /// Before anything was written: the unit is in ISP mode, with the image it held.
    InIsp,
    /// During the erase or the write: the unit is in ISP mode, with a partial image.
    Partial,
    /// During the verify: the unit is in ISP mode, with the image written but not checked.
    Unverified,
    /// After the verified flash and its reset: the unit restarts into the new image.
    Flashed,
    /// After a reset without a flash: the unit restarts into the image it holds.
    Reset,
}

impl Left {
    /// Whether the unit stays in ISP mode.
    #[must_use]
    pub fn in_isp(self) -> bool {
        matches!(self, Left::InIsp | Left::Partial | Left::Unverified)
    }

    /// The command that recovers the unit, in a sentence, where it needs one. `again` is the
    /// command that flashes the same image ([`crate::source::flash_command`]), and `board` the
    /// unit's board where it is known.
    #[must_use]
    pub fn recovery(
        self,
        again: Option<&str>,
        board: Option<crate::board::Board>,
    ) -> Option<String> {
        match (self, again) {
            (Left::InIsp, Some(again)) => Some(format!(
                "`verge isp reset` starts that image, and `{again}` installs the new one."
            )),
            (Left::InIsp, None) => Some("`verge isp reset` starts that image.".to_owned()),
            (Left::Partial | Left::Unverified, Some(again)) => Some(match board {
                Some(board) => format!(
                    "In ISP mode the tool cannot tell its board, so flash it again with the same image, which is built for {}: `{again}`.",
                    board.a()
                ),
                None => format!("`{again}` flashes it again."),
            }),
            (Left::Partial | Left::Unverified | Left::Flashed | Left::Reset, _) => None,
        }
    }
}

impl fmt::Display for Left {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Left::InIsp => {
                "stopped before anything was written: the unit is in ISP mode, with the image it held"
            }
            Left::Partial => {
                "stopped during the flash: the unit is in ISP mode, with a partial image"
            }
            Left::Unverified => {
                "stopped during the verify: the unit is in ISP mode, with the image written but not checked"
            }
            Left::Flashed => {
                "stopped after the image was written and verified: the unit is restarting into it"
            }
            Left::Reset => {
                "stopped after the reset: the unit is restarting into the image it holds"
            }
        })
    }
}

/// A request to stop, checked between pages, polls and steps. What a cancel leaves on the unit
/// depends on where it lands: see [`Left`].
#[derive(Clone, Debug, Default)]
pub struct Cancel(Arc<AtomicBool>);

impl Cancel {
    /// A request not made yet.
    #[must_use]
    pub fn new() -> Cancel {
        Cancel::default()
    }

    /// Makes the request, for every clone of this one.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}
