//! The boards an MCU-Link firmware of ours runs on, numbered as vendor command 0x82's board byte
//! and an image's declaration number them.

use std::fmt;

use serde::Serialize;

use crate::dap::Ours;
use crate::detect::{Hardware, State, Unit};

/// A board this tool knows by number.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
pub enum Board {
    /// The plain MCU-Link.
    McuLink,
    McuLinkPro,
}

impl Board {
    pub const ALL: [Board; 2] = [Board::McuLink, Board::McuLinkPro];

    /// Its number: the board byte ours reports, and its bit in an image's declaration.
    #[must_use]
    pub fn number(self) -> u8 {
        match self {
            Board::McuLink => Ours::BOARD_MCU_LINK,
            Board::McuLinkPro => Ours::BOARD_MCU_LINK_PRO,
        }
    }

    /// The board with this number, where this tool knows it. Number 2, an on-board MCU-Link, is
    /// one design per evaluation board, so no single number tells which.
    #[must_use]
    pub fn from_number(number: u8) -> Option<Board> {
        Board::ALL
            .into_iter()
            .find(|board| board.number() == number)
    }

    /// Its name with its article, for a sentence.
    #[must_use]
    pub fn a(self) -> &'static str {
        match self {
            Board::McuLink => "a full MCU-Link",
            Board::McuLinkPro => "an MCU-Link Pro",
        }
    }

    /// Whether `unit` is this board. A unit in ISP mode cannot say which board it is, so it
    /// passes; the question that arms a batch taking such units says so.
    #[must_use]
    pub fn takes(self, unit: &Unit) -> bool {
        unit.state == State::Bootloader || unit.hardware() == Hardware::Board(self)
    }
}

impl fmt::Display for Board {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Board::McuLink => "full MCU-Link",
            Board::McuLinkPro => "MCU-Link Pro",
        })
    }
}

/// The boards an image runs on, to follow "built for" in a sentence: "the full MCU-Link only",
/// or "the full MCU-Link and the MCU-Link Pro".
#[must_use]
pub fn built_for(boards: &[Board]) -> String {
    let names: Vec<String> = boards.iter().map(|board| format!("the {board}")).collect();
    match names.as_slice() {
        [] => "no board this tool knows".to_owned(),
        [one] => format!("{one} only"),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}
