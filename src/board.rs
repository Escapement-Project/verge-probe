//! The boards an MCU-Link firmware of ours runs on, numbered as vendor command 0x82's board byte
//! and an image's declaration number them.

use std::fmt;

use serde::Serialize;

use crate::dap::Ours;
use crate::detect::{Hardware, State, Unit};
use crate::image::{Image, ImageError};

/// A board this tool knows by number.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
pub enum Board {
    /// The standalone MCU-Link.
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

    /// Its name alone, for a label.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Board::McuLink => "standalone MCU-Link",
            Board::McuLinkPro => "MCU-Link Pro",
        }
    }

    /// Its name with its article, for a sentence.
    #[must_use]
    pub fn a(self) -> &'static str {
        match self {
            Board::McuLink => "a standalone MCU-Link",
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
        f.write_str(self.name())
    }
}

/// The boards an image runs on, to follow "built for" in a sentence: "the standalone MCU-Link
/// only", or "the standalone MCU-Link and the MCU-Link Pro".
#[must_use]
pub fn built_for(boards: &[Board]) -> String {
    let names: Vec<String> = boards.iter().map(|board| format!("the {board}")).collect();
    match names.as_slice() {
        [] => "no board this tool knows".to_owned(),
        [one] => format!("{one} only"),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

/// What `image` says about its boards, to follow "the image" in a sentence: "is built for the
/// MCU-Link Pro only". An image with no declaration runs on the standalone MCU-Link only, as a
/// Verge image; the clause says that is what verge takes it for.
pub fn image_is_for(image: &Image) -> Result<String, ImageError> {
    Ok(match image.declaration()? {
        Some(_) => format!("is built for {}", built_for(&image.boards()?)),
        None => format!(
            "declares no boards, so verge takes it as a Verge image for the {} only",
            Board::McuLink
        ),
    })
}
