//! The flash sequence on the ROM bootloader, through NXP's own McuBoot client (the `rblhost`
//! crate's `mboot` library, the same code LinkServer ships as its `rblhost` tool).

use mboot::McuBoot;
use mboot::protocols::{CommunicationError, Protocol};
use mboot::tags::property::{PropertyTag, PropertyTagDiscriminants};

use crate::image::{Image, PAGE};
use crate::progress::{Cancel, Progress, Step};

/// The bytes moved per bootloader command. A multiple of the page, so each write starts on a page
/// as flash requires; also how often progress is reported and a cancel is noticed.
pub const CHUNK: usize = 8 * PAGE as usize;

/// Internal flash, in the McuBoot memory id space.
const INTERNAL: u32 = 0;

#[derive(Debug, thiserror::Error)]
pub enum IspError {
    #[error("the bootloader: {0}")]
    Rom(#[from] CommunicationError),
    #[error("flash at 0x{address:08x} reads back 0x{got:02x} where the image has 0x{expected:02x}")]
    Verify { address: u32, expected: u8, got: u8 },
    #[error("read {got} bytes back at 0x{address:08x} and asked for {asked}")]
    ShortRead {
        address: u32,
        asked: usize,
        got: usize,
    },
    #[error(
        "the bootloader reports flash from 0x{start:08x} to 0x{end:08x}, and the image reaches 0x{needed:08x}"
    )]
    DoesNotFit { start: u32, end: u64, needed: u64 },
    #[error(
        "cancelled; the unit stays in ISP mode with a partial image, and flashing it again recovers it"
    )]
    Cancelled,
}

/// What the bootloader says about itself.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RomInfo {
    /// Such as `K3.0.0`.
    pub version: String,
    /// Start and size of internal flash, when the bootloader answers both.
    pub flash: Option<(u32, u32)>,
}

/// Asks for the bootloader's version, which is also how the tool knows it answers at all.
pub fn rom_version<P: Protocol>(boot: &mut McuBoot<P>) -> Result<String, CommunicationError> {
    let response = boot.get_property(PropertyTagDiscriminants::CurrentVersion, INTERNAL)?;
    Ok(match response.property {
        PropertyTag::CurrentVersion(v) => {
            format!("{}{}.{}.{}", v.mark, v.major, v.minor, v.fixation)
        }
        other => format!("{other}"),
    })
}

fn property_u32<P: Protocol>(boot: &mut McuBoot<P>, tag: PropertyTagDiscriminants) -> Option<u32> {
    match boot.get_property(tag, INTERNAL) {
        Ok(response) => response.response_words.first().copied(),
        Err(error) => {
            tracing::debug!("the bootloader did not answer property {tag:?}: {error}");
            None
        }
    }
}

pub fn rom_info<P: Protocol>(boot: &mut McuBoot<P>) -> Result<RomInfo, CommunicationError> {
    let version = rom_version(boot)?;
    let start = property_u32(boot, PropertyTagDiscriminants::FlashStartAddress);
    let size = property_u32(boot, PropertyTagDiscriminants::FlashSize);
    Ok(RomInfo {
        version,
        flash: start.zip(size),
    })
}

/// Erases, writes and verifies `image`, whole pages at a time.
///
/// Only the pages the image covers are erased, as `rblhost flash-image` does. It erases in 1 KB
/// units and so can erase a page past the image; this erases exactly the pages it then programs,
/// which is also every page the verify reads.
pub fn program<P: Protocol>(
    boot: &mut McuBoot<P>,
    image: &Image,
    rom: &RomInfo,
    progress: &mut dyn Progress,
    cancel: &Cancel,
) -> Result<(), IspError> {
    let runs = image.page_runs();
    let total: u64 = runs.iter().map(|run| run.data.len() as u64).sum();
    if let Some((start, size)) = rom.flash {
        let end = u64::from(start) + u64::from(size);
        for run in &runs {
            let needed = u64::from(run.address) + run.data.len() as u64;
            if run.address < start || needed > end {
                return Err(IspError::DoesNotFit { start, end, needed });
            }
        }
    }

    let pieces = || {
        runs.iter().flat_map(|run| {
            run.data
                .chunks(CHUNK)
                .enumerate()
                .map(|(index, data)| (run.address + (index * CHUNK) as u32, data))
        })
    };

    progress.start(Step::Erase, total);
    let mut done = 0;
    for (address, data) in pieces() {
        if cancel.is_cancelled() {
            return Err(IspError::Cancelled);
        }
        boot.flash_erase_region(address, data.len() as u32, INTERNAL)?;
        done += data.len() as u64;
        progress.advance(done);
    }

    progress.start(Step::Write, total);
    let mut done = 0;
    for (address, data) in pieces() {
        if cancel.is_cancelled() {
            return Err(IspError::Cancelled);
        }
        boot.write_memory(address, INTERNAL, data)?;
        done += data.len() as u64;
        progress.advance(done);
    }

    progress.start(Step::Verify, total);
    let mut done = 0;
    for (address, data) in pieces() {
        if cancel.is_cancelled() {
            return Err(IspError::Cancelled);
        }
        let read = boot.read_memory(address, data.len() as u32, INTERNAL)?;
        if read.bytes.len() != data.len() {
            return Err(IspError::ShortRead {
                address,
                asked: data.len(),
                got: read.bytes.len(),
            });
        }
        if let Some(offset) = data.iter().zip(read.bytes.iter()).position(|(a, b)| a != b) {
            return Err(IspError::Verify {
                address: address + offset as u32,
                expected: data[offset],
                got: read.bytes[offset],
            });
        }
        done += data.len() as u64;
        progress.advance(done);
    }
    Ok(())
}

/// Resets out of the bootloader. The ROM clears the boot mode on entry, so without the jumper the
/// unit comes back running its firmware.
///
/// The bootloader answers before it resets. A reply lost to the reset itself is not an error: the
/// caller waits for the unit to leave the bus either way.
pub fn reset<P: Protocol>(boot: &mut McuBoot<P>) {
    if let Err(error) = boot.reset() {
        tracing::debug!("the reset command's reply did not arrive: {error}");
    }
}
