//! The DIO pins: their names, a change to them, and the checks the probe would make of it.
//!
//! The Pro has them on J19, the standalone MCU-Link on J8 ([The
//! Pro](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/pro.md#dio-pins), [Probes
//! and jumpers](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/probes.md#the-mcu-links-dio-pins)).
//! A pin set is a mask, bit `k` for DIO`k`.

use std::fmt;

use crate::{BoardKind, CommandError, DioState, Refused, VergeControl, VergeTransport};

/// All four pins.
pub const ALL: u8 = 0x0F;

/// Pins by name, as "DIO0, DIO1".
#[must_use]
pub fn names(pins: u8) -> String {
    (0..8)
        .filter(|n| pins >> n & 1 != 0)
        .map(|n| format!("DIO{n}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// A name that is no pin.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinError(String);

impl fmt::Display for PinError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0.is_empty() {
            f.write_str("an empty pin name: name pins as 0 to 3, dio0 to dio3, or all")
        } else {
            write!(
                f,
                "{} is not a DIO pin: name one as 0 to 3, dio0 to dio3, or all",
                self.0
            )
        }
    }
}

impl std::error::Error for PinError {}

/// One pin, or all of them: `2`, `dio2` in any case, or `all`.
pub fn parse_pin(text: &str) -> Result<u8, PinError> {
    if text.eq_ignore_ascii_case("all") {
        return Ok(ALL);
    }
    let digits = text
        .get(..3)
        .filter(|prefix| prefix.eq_ignore_ascii_case("dio"))
        .map_or(text, |_| &text[3..]);
    match digits.parse::<u8>() {
        Ok(n) if n < 4 && digits.bytes().all(|b| b.is_ascii_digit()) => Ok(1 << n),
        _ => Err(PinError(text.to_owned())),
    }
}

/// Pins named in a list, as `0,dio3` or `all`.
pub fn parse_pins(list: &str) -> Result<u8, PinError> {
    list.split(',')
        .map(|name| parse_pin(name.trim()))
        .try_fold(0, |pins, pin| Ok(pins | pin?))
}

/// A change to the pins: those to drive high, those to drive low, those to make inputs again.
/// The rest stay as they are.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct DioChange {
    pub high: u8,
    pub low: u8,
    pub input: u8,
}

impl DioChange {
    /// Drives `pins` high, as outputs.
    #[must_use]
    pub fn high(mut self, pins: u8) -> Self {
        self.high |= pins;
        self
    }

    /// Drives `pins` low, as outputs.
    #[must_use]
    pub fn low(mut self, pins: u8) -> Self {
        self.low |= pins;
        self
    }

    /// Makes `pins` inputs.
    #[must_use]
    pub fn input(mut self, pins: u8) -> Self {
        self.input |= pins;
        self
    }

    /// The pins the change touches.
    #[must_use]
    pub fn pins(&self) -> u8 {
        self.high | self.low | self.input
    }

    /// The pins the change makes outputs.
    #[must_use]
    pub fn outputs(&self) -> u8 {
        self.high | self.low
    }

    /// Pins the change names more than once.
    #[must_use]
    pub fn overlap(&self) -> u8 {
        (self.high & self.low) | (self.high & self.input) | (self.low & self.input)
    }
}

/// Why [`VergeControl::change_dio`] left the pins as they were. All but `Command` are found from
/// the pins' state, and the change was not sent.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum DioError {
    /// Pins the change names more than one way.
    #[error("{} named to go more than one way", names(*.0))]
    Overlap(u8),
    #[error("the DIO pins are off: the Pro started with its USB-SIO jumper, J15, fitted")]
    Off,
    /// A bench tool gave a standalone MCU-Link's pins to its command marker, which keeps them
    /// until the unit restarts.
    #[error("a bench tool's command marker holds {}", names(*.0))]
    Held(u8),
    /// The pins of a direction group, which the change would split.
    #[error("{} change direction together", names(*.0))]
    Split(u8),
    /// Reading or changing the pins failed. Where the probe refused the change,
    /// [`CommandError::Firmware`] holds the pins as they stayed.
    #[error(transparent)]
    Command(#[from] CommandError<Refused<DioState>>),
}

impl<T: VergeTransport + ?Sized> VergeControl<'_, T> {
    /// Changes the pins as `change` says, and returns their state after it. A change the probe
    /// would refuse is refused before it is sent, saying why. An empty change only reads, and
    /// still says when the marker holds every pin or J15 turned them off.
    pub fn change_dio(&mut self, change: &DioChange) -> Result<DioState, DioError> {
        let overlap = change.overlap();
        if overlap != 0 {
            return Err(DioError::Overlap(overlap));
        }
        // A build from before the board byte runs on a Pro.
        let pro = self
            .info()
            .map_err(CommandError::widen)?
            .and_then(|info| info.board)
            .is_none_or(|board| board == BoardKind::Pro);
        let before = self.access_dio(0, 0, 0)?;
        if before.pins == 0 && pro {
            return Err(DioError::Off);
        }
        // On the standalone MCU-Link a pin missing from `pins` is one the marker holds.
        let held = if before.pins == 0 {
            ALL
        } else {
            change.pins() & !before.pins
        };
        if held != 0 {
            return Err(DioError::Held(held));
        }
        if change.pins() == 0 {
            return Ok(before);
        }
        if let Some(group) = before.split(change.pins(), change.outputs()) {
            return Err(DioError::Split(group));
        }
        Ok(self.access_dio(change.pins(), change.outputs(), change.high)?)
    }
}
