//! Firmware images: an S-record or an ELF, reduced to the bytes that go into flash.

use std::collections::BTreeMap;
use std::fmt;
use std::path::Path;

use object::elf;
use object::read::elf::{ElfFile32, FileHeader, ProgramHeader, SectionHeader};
use sha2::{Digest, Sha256};

use crate::board::Board;

/// The LPC55S69's flash page: the unit of erase and of programming.
pub const PAGE: u32 = 512;

/// The end of the flash an image may use: 630 KB, as UM11126 table 4 gives it. The table says
/// the last "16 pages (10 KB)" of the 640 KB are reserved, which does not add up (16 pages are
/// 8 KB), and the protected flash region starts at `0x9DE00`, 1.5 KB above this. The tool takes
/// the lowest of the three.
pub const APP_FLASH_END: u32 = 0x9_D800;

/// The ASCII bytes that start an image's board declaration, with no terminator.
pub const DECLARATION_MAGIC: &[u8; 8] = b"VERGEIMG";

/// A declaration's length at format 1, the format that added the boards.
const DECLARATION_LEN: usize = 16;

/// The boards an image says it runs on: a record in its flash, little-endian, found by its magic.
///
/// | offset | size | field |
/// | --- | --- | --- |
/// | 0 | 8 | [`DECLARATION_MAGIC`] |
/// | 8 | 1 | format, 1 |
/// | 9 | 1 | the record's length in bytes, 16 at format 1 |
/// | 10 | 2 | zero |
/// | 12 | 4 | boards: bit n set means the image runs on board n, numbered as [`Board::number`] |
///
/// Fields are only ever appended, and each is read only from the format that added it on. An image
/// with no declaration runs on the plain MCU-Link only, as every image built before it did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Declaration {
    /// Where the record starts.
    pub address: u32,
    pub format: u8,
    pub boards: u32,
}

impl Declaration {
    /// Whether the record names `board`. A bit for a board this tool does not know names nothing.
    #[must_use]
    pub fn names(&self, board: Board) -> bool {
        self.boards & (1 << board.number()) != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    SRecord,
    Elf,
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Format::SRecord => "S-record",
            Format::Elf => "ELF",
        })
    }
}

/// Contiguous bytes at an address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chunk {
    pub address: u32,
    pub data: Vec<u8>,
}

impl Chunk {
    #[must_use]
    pub fn end(&self) -> u64 {
        u64::from(self.address) + self.data.len() as u64
    }
}

/// Whole pages to erase and program, padded with `0xFF` where the image has no bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PageRun {
    pub address: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ImageError {
    #[error("this is neither an ELF nor an S-record file")]
    UnknownFormat,
    #[error("S-record line {line}: {reason}")]
    SRecord { line: usize, reason: String },
    #[error("the S-record count record says {stated} data records and the file has {found}")]
    SRecordCount { stated: u32, found: u32 },
    #[error("ELF: {0}")]
    Elf(String),
    #[error("the image has no bytes to flash")]
    Empty,
    #[error("two parts of the image disagree about the byte at 0x{0:08x}")]
    Overlap(u32),
    #[error(
        "the image has bytes from 0x{start:08x} to 0x{end:08x}, outside the flash an MCU-Link image may use (0x00000000 to 0x{limit:08x})"
    )]
    OutsideFlash { start: u32, end: u64, limit: u32 },
    #[error("the image has no vector table at address 0")]
    NoVectorTable,
    #[error("the reset vector 0x{0:08x} does not point into the image's code")]
    BadResetVector(u32),
    #[error("the initial stack pointer 0x{0:08x} is not a plausible RAM address")]
    BadStackPointer(u32),
    #[error("reading {path}: {reason}")]
    Read { path: String, reason: String },
    #[error(
        "the image declares its boards twice, at 0x{first:08x} and 0x{second:08x}, so it is malformed"
    )]
    TwoDeclarations { first: u32, second: u32 },
    #[error("the image's board declaration at 0x{address:08x} is malformed: {reason}")]
    BadDeclaration { address: u32, reason: String },
}

/// A firmware image ready to flash.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Image {
    pub format: Format,
    /// Sorted by address, merged where they touch, none overlapping.
    pub chunks: Vec<Chunk>,
    /// The S0 header of an S-record.
    pub header: Option<String>,
    /// SHA-256 of the file as given, which is what a release's checksum covers.
    pub file_sha256: [u8; 32],
}

impl Image {
    pub fn from_path(path: &Path) -> Result<Image, ImageError> {
        let bytes = std::fs::read(path).map_err(|error| ImageError::Read {
            path: path.display().to_string(),
            reason: error.to_string(),
        })?;
        Image::parse(&bytes)
    }

    /// Reads an ELF or an S-record, telling them apart by content.
    pub fn parse(bytes: &[u8]) -> Result<Image, ImageError> {
        let file_sha256 = Sha256::digest(bytes).into();
        let (format, pieces, header) = if bytes.starts_with(b"\x7fELF") {
            (Format::Elf, elf_pieces(bytes)?, None)
        } else if bytes.first() == Some(&b'S') {
            let (pieces, header) = srecord_pieces(bytes)?;
            (Format::SRecord, pieces, header)
        } else {
            return Err(ImageError::UnknownFormat);
        };
        let chunks = merge(pieces)?;
        if chunks.is_empty() {
            return Err(ImageError::Empty);
        }
        Ok(Image {
            format,
            chunks,
            header,
            file_sha256,
        })
    }

    /// The number of bytes the image puts into flash, without padding.
    #[must_use]
    pub fn data_len(&self) -> usize {
        self.chunks.iter().map(|chunk| chunk.data.len()).sum()
    }

    #[must_use]
    pub fn sha256_hex(&self) -> String {
        hex::encode(self.file_sha256)
    }

    /// The little-endian word at `address`, if the image has all four bytes.
    #[must_use]
    pub fn word_at(&self, address: u32) -> Option<u32> {
        let chunk = self
            .chunks
            .iter()
            .find(|chunk| chunk.address <= address && u64::from(address) + 4 <= chunk.end())?;
        let offset = (address - chunk.address) as usize;
        Some(u32::from_le_bytes(
            chunk.data[offset..offset + 4].try_into().ok()?,
        ))
    }

    fn contains(&self, address: u32) -> bool {
        self.chunks
            .iter()
            .any(|chunk| chunk.address <= address && u64::from(address) < chunk.end())
    }

    /// Refuses an image that cannot be an MCU-Link firmware: bytes outside the flash it may use,
    /// or a vector table that would not start.
    ///
    /// This catches the wrong file, not a wrong firmware: any Cortex-M image for this part passes.
    pub fn check_for_mculink(&self) -> Result<(), ImageError> {
        let start = self.chunks.first().map_or(0, |chunk| chunk.address);
        let end = self.chunks.last().map_or(0, Chunk::end);
        if end > u64::from(APP_FLASH_END) {
            return Err(ImageError::OutsideFlash {
                start,
                end,
                limit: APP_FLASH_END,
            });
        }
        let (Some(stack), Some(reset)) = (self.word_at(0), self.word_at(4)) else {
            return Err(ImageError::NoVectorTable);
        };
        if reset & 1 == 0 || !self.contains(reset & !1) {
            return Err(ImageError::BadResetVector(reset));
        }
        if stack == 0 || stack == u32::MAX || stack % 4 != 0 {
            return Err(ImageError::BadStackPointer(stack));
        }
        self.declaration()?;
        Ok(())
    }

    /// The image's board declaration, looked for at every 4-byte-aligned address of every chunk.
    /// `None` where it has none. Two of them, or one that does not hold together, make the image
    /// malformed.
    pub fn declaration(&self) -> Result<Option<Declaration>, ImageError> {
        let mut found: Vec<(&Chunk, usize)> = Vec::new();
        for chunk in &self.chunks {
            let mut offset = ((4 - chunk.address % 4) % 4) as usize;
            while offset + DECLARATION_MAGIC.len() <= chunk.data.len() {
                if chunk.data[offset..].starts_with(DECLARATION_MAGIC) {
                    found.push((chunk, offset));
                }
                offset += 4;
            }
        }
        let address = |(chunk, offset): (&Chunk, usize)| chunk.address + offset as u32;
        let (chunk, offset) = match found.as_slice() {
            [] => return Ok(None),
            [one] => *one,
            [first, second, ..] => {
                return Err(ImageError::TwoDeclarations {
                    first: address(*first),
                    second: address(*second),
                });
            }
        };
        let record = &chunk.data[offset..];
        let bad = |reason: String| ImageError::BadDeclaration {
            address: address((chunk, offset)),
            reason,
        };
        let past = || bad("it runs past the end of its part of the image".into());
        let (&format, &len) = record.get(8).zip(record.get(9)).ok_or_else(past)?;
        if format == 0 {
            return Err(bad("its format is 0".into()));
        }
        let len = usize::from(len);
        if len < DECLARATION_LEN {
            return Err(bad(format!(
                "its length is {len}, under the {DECLARATION_LEN} bytes of format 1"
            )));
        }
        let fields = record.get(..len).ok_or_else(past)?;
        Ok(Some(Declaration {
            address: address((chunk, offset)),
            format,
            boards: u32::from_le_bytes([fields[12], fields[13], fields[14], fields[15]]),
        }))
    }

    /// The boards the image runs on: those its declaration names, or the plain MCU-Link where it
    /// declares nothing.
    pub fn boards(&self) -> Result<Vec<Board>, ImageError> {
        Ok(match self.declaration()? {
            None => vec![Board::McuLink],
            Some(declaration) => Board::ALL
                .into_iter()
                .filter(|board| declaration.names(*board))
                .collect(),
        })
    }

    /// The image as whole pages, grouped into runs of consecutive pages.
    ///
    /// Programming whole pages means no page is written twice, which flash refuses, and a verify
    /// read covers only pages this tool programmed. A read of an erased page on this part fails
    /// its ECC check and drops the bootloader off the bus.
    #[must_use]
    pub fn page_runs(&self) -> Vec<PageRun> {
        let page = PAGE as usize;
        let mut pages: BTreeMap<u32, Vec<u8>> = BTreeMap::new();
        for chunk in &self.chunks {
            let mut address = chunk.address;
            let mut data = chunk.data.as_slice();
            while !data.is_empty() {
                let base = address - address % PAGE;
                let offset = (address - base) as usize;
                let take = data.len().min(page - offset);
                let buffer = pages.entry(base).or_insert_with(|| vec![0xFF; page]);
                buffer[offset..offset + take].copy_from_slice(&data[..take]);
                data = &data[take..];
                address = base.wrapping_add(PAGE);
            }
        }
        let mut runs: Vec<PageRun> = Vec::new();
        for (base, data) in pages {
            match runs.last_mut() {
                Some(run) if u64::from(run.address) + run.data.len() as u64 == u64::from(base) => {
                    run.data.extend_from_slice(&data);
                }
                _ => runs.push(PageRun {
                    address: base,
                    data,
                }),
            }
        }
        runs
    }

    /// The image as an S-record, 32-bit addresses, 32 bytes a line.
    #[must_use]
    pub fn to_srecord(&self, header: &str) -> String {
        let mut records = vec![srec::Record::S0(header.to_owned())];
        for chunk in &self.chunks {
            for (index, line) in chunk.data.chunks(32).enumerate() {
                records.push(srec::Record::S3(srec::Data {
                    address: srec::Address32(chunk.address + (index * 32) as u32),
                    data: line.to_vec(),
                }));
            }
        }
        records.push(srec::Record::S7(srec::Address32(
            self.word_at(4).unwrap_or(0) & !1,
        )));
        srec::generate_srec_file(&records)
    }
}

fn srecord_pieces(bytes: &[u8]) -> Result<(Vec<Chunk>, Option<String>), ImageError> {
    let text = std::str::from_utf8(bytes).map_err(|error| ImageError::SRecord {
        line: 0,
        reason: format!("not text ({error})"),
    })?;
    let mut pieces = Vec::new();
    let mut header = None;
    let mut stated = None;
    let mut found = 0u32;
    let lines = text
        .lines()
        .enumerate()
        .map(|(index, line)| (index + 1, line.trim()))
        .filter(|(_, line)| !line.is_empty());
    for (line_number, line) in lines {
        let record: srec::Record =
            line.parse()
                .map_err(|error: srec::ReaderError| ImageError::SRecord {
                    line: line_number,
                    reason: error.to_string(),
                })?;
        let (address, data) = match record {
            srec::Record::S0(text) => {
                header = Some(text);
                continue;
            }
            srec::Record::S1(data) => (u32::from(data.address), data.data),
            srec::Record::S2(data) => (u32::from(data.address), data.data),
            srec::Record::S3(data) => (u32::from(data.address), data.data),
            srec::Record::S5(count) => {
                stated = Some(u32::from(count));
                continue;
            }
            srec::Record::S6(count) => {
                stated = Some(u32::from(count));
                continue;
            }
            srec::Record::S7(_) | srec::Record::S8(_) | srec::Record::S9(_) => continue,
        };
        found += 1;
        pieces.push(Chunk { address, data });
    }
    if let Some(stated) = stated
        && stated != found
    {
        return Err(ImageError::SRecordCount { stated, found });
    }
    Ok((pieces, header))
}

/// The bytes of every section with contents that is loaded, at its load address.
///
/// This is what `objcopy -O srec` writes. Reading program headers instead is not the same: a
/// linker may put two sections into one segment, and the segment then carries the padding between
/// them, which objcopy leaves out. A section's load address comes from the segment that holds it,
/// which is how `.data` lands in flash after the code.
fn elf_pieces(bytes: &[u8]) -> Result<Vec<Chunk>, ImageError> {
    let file = ElfFile32::<object::Endianness>::parse(bytes)
        .map_err(|error| ImageError::Elf(format!("not a 32-bit ELF: {error}")))?;
    let endian = file.endian();
    let machine = file.elf_header().e_machine(endian);
    if machine != elf::EM_ARM {
        return Err(ImageError::Elf(format!(
            "built for machine {machine}, not Arm"
        )));
    }
    let loads: Vec<_> = file
        .elf_program_headers()
        .iter()
        .filter(|segment| segment.p_type(endian) == elf::PT_LOAD)
        .collect();
    let mut pieces = Vec::new();
    for section in file.elf_section_table().iter() {
        let loaded = section.sh_flags(endian).contains(elf::SHF_ALLOC)
            && section.sh_type(endian) != elf::SHT_NOBITS
            && section.sh_size(endian) > 0;
        if !loaded {
            continue;
        }
        let data = section
            .data(endian, bytes)
            .map_err(|error| ImageError::Elf(format!("a section: {error}")))?;
        let offset = section.sh_offset(endian);
        let end = u64::from(offset) + u64::from(section.sh_size(endian));
        let address = loads
            .iter()
            .find(|segment| {
                let start = segment.p_offset(endian);
                start <= offset && end <= u64::from(start) + u64::from(segment.p_filesz(endian))
            })
            .map_or(section.sh_addr(endian), |segment| {
                segment.p_paddr(endian) + (offset - segment.p_offset(endian))
            });
        pieces.push(Chunk {
            address,
            data: data.to_vec(),
        });
    }
    Ok(pieces)
}

fn merge(mut pieces: Vec<Chunk>) -> Result<Vec<Chunk>, ImageError> {
    pieces.sort_by_key(|piece| piece.address);
    let mut chunks: Vec<Chunk> = Vec::new();
    for piece in pieces.into_iter().filter(|piece| !piece.data.is_empty()) {
        if piece.end() > 1 << 32 {
            return Err(ImageError::OutsideFlash {
                start: piece.address,
                end: piece.end(),
                limit: APP_FLASH_END,
            });
        }
        let Some(last) = chunks.last_mut() else {
            chunks.push(piece);
            continue;
        };
        if u64::from(piece.address) > last.end() {
            chunks.push(piece);
            continue;
        }
        let overlap = usize::try_from(last.end() - u64::from(piece.address))
            .expect("an overlap within one 32-bit address space fits usize");
        let shared = overlap.min(piece.data.len());
        let tail_start = last.data.len() - overlap;
        for i in 0..shared {
            if last.data[tail_start + i] != piece.data[i] {
                return Err(ImageError::Overlap(piece.address + i as u32));
            }
        }
        last.data.extend_from_slice(&piece.data[shared..]);
    }
    Ok(chunks)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn srec_line(record: srec::Record) -> String {
        srec::generate_srec_file(&[record])
    }

    fn s3(address: u32, data: &[u8]) -> String {
        srec_line(srec::Record::S3(srec::Data {
            address: srec::Address32(address),
            data: data.to_vec(),
        }))
    }

    fn vectors(stack: u32, reset: u32) -> Vec<u8> {
        let mut bytes = stack.to_le_bytes().to_vec();
        bytes.extend_from_slice(&reset.to_le_bytes());
        bytes
    }

    #[test]
    fn srecord_records_merge_into_chunks() {
        let text = [
            s3(0x10, &[3, 4]),
            s3(0x0, &vectors(0x2001_0000, 0x11)),
            s3(0x8, &[0; 8]),
            s3(0x200, &[9]),
        ]
        .concat();
        let image = Image::parse(text.as_bytes()).unwrap();
        assert_eq!(image.format, Format::SRecord);
        assert_eq!(image.chunks.len(), 2);
        assert_eq!(image.chunks[0].address, 0);
        assert_eq!(image.chunks[0].data.len(), 0x12);
        assert_eq!(
            image.chunks[1],
            Chunk {
                address: 0x200,
                data: vec![9]
            }
        );
        image.check_for_mculink().unwrap();
    }

    #[test]
    fn overlaps_must_agree() {
        let agree = [s3(0, &[1, 2, 3, 4]), s3(2, &[3, 4, 5])].concat();
        assert_eq!(
            Image::parse(agree.as_bytes()).unwrap().chunks[0].data,
            [1, 2, 3, 4, 5]
        );
        let disagree = [s3(0, &[1, 2, 3, 4]), s3(2, &[3, 9])].concat();
        assert_eq!(
            Image::parse(disagree.as_bytes()),
            Err(ImageError::Overlap(3))
        );
        // Wholly inside an earlier record, and agreeing.
        let inside = [s3(0, &[1, 2, 3, 4]), s3(1, &[2])].concat();
        assert_eq!(
            Image::parse(inside.as_bytes()).unwrap().chunks[0].data,
            [1, 2, 3, 4]
        );
    }

    #[test]
    fn a_bad_checksum_names_its_line() {
        let mut text = [s3(0, &[1, 2, 3, 4]), s3(4, &[5])].concat();
        // Flip the last hex digit of the second line's checksum.
        let end = text.trim_end().len() - 1;
        let digit = if text.as_bytes()[end] == b'0' {
            "1"
        } else {
            "0"
        };
        text.replace_range(end..=end, digit);
        assert!(matches!(
            Image::parse(text.as_bytes()),
            Err(ImageError::SRecord { line: 2, .. })
        ));
    }

    #[test]
    fn the_count_record_is_checked() {
        let count = srec_line(srec::Record::S5(srec::Count16(3)));
        let text = [s3(0, &[1]), s3(1, &[2]), count].concat();
        assert_eq!(
            Image::parse(text.as_bytes()),
            Err(ImageError::SRecordCount {
                stated: 3,
                found: 2
            })
        );
    }

    #[test]
    fn page_runs_pad_and_group() {
        let text = [
            s3(0, &vectors(0x2001_0000, 0x201)),
            s3(0x200, &[0xAA; 4]),
            s3(0x5FE, &[1, 2, 3, 4]),
            s3(0x40000, &[7]),
        ]
        .concat();
        let image = Image::parse(text.as_bytes()).unwrap();
        let runs = image.page_runs();
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0].address, 0);
        // Pages 0x000 to 0x600, the last two sharing the four bytes at 0x5FE.
        assert_eq!(runs[0].data.len(), 4 * 512);
        assert_eq!(runs[0].data[8], 0xFF);
        assert_eq!(&runs[0].data[0x200..0x204], &[0xAA; 4]);
        assert_eq!(&runs[0].data[0x5FE..0x602], &[1, 2, 3, 4]);
        assert_eq!(runs[0].data[0x602], 0xFF);
        assert_eq!(runs[1].address, 0x40000);
        assert_eq!(runs[1].data.len(), 512);
        for run in &runs {
            assert_eq!(run.address % PAGE, 0);
            assert_eq!(run.data.len() % PAGE as usize, 0);
        }
    }

    #[test]
    fn checks_refuse_what_cannot_be_firmware() {
        let parse = |text: String| Image::parse(text.as_bytes()).unwrap().check_for_mculink();
        assert_eq!(parse(s3(0x100, &[1; 8])), Err(ImageError::NoVectorTable));
        assert_eq!(
            parse(s3(0, &vectors(0x2001_0000, 0x200))),
            Err(ImageError::BadResetVector(0x200))
        );
        assert_eq!(
            parse(s3(0, &vectors(0x2001_0000, 0x1001))),
            Err(ImageError::BadResetVector(0x1001))
        );
        assert_eq!(
            parse(s3(0, &vectors(0xFFFF_FFFF, 0x5))),
            Err(ImageError::BadStackPointer(0xFFFF_FFFF))
        );
        let past_the_end = [
            s3(0, &vectors(0x2001_0000, 0x5)),
            s3(APP_FLASH_END - 2, &[0; 4]),
        ]
        .concat();
        assert!(matches!(
            parse(past_the_end),
            Err(ImageError::OutsideFlash { .. })
        ));
        let up_to_the_end = [
            s3(0, &vectors(0x2001_0000, 0x5)),
            s3(APP_FLASH_END - 4, &[0; 4]),
        ]
        .concat();
        assert_eq!(parse(up_to_the_end), Ok(()));
    }

    /// A declaration as the firmware lays it out, written out by hand: magic, format, length, two
    /// zero bytes, boards.
    fn record(format: u8, len: u8, boards: u32) -> Vec<u8> {
        let mut bytes = b"VERGEIMG".to_vec();
        bytes.extend_from_slice(&[format, len, 0, 0]);
        bytes.extend_from_slice(&boards.to_le_bytes());
        bytes
    }

    fn declared(parts: &[(u32, Vec<u8>)]) -> Result<Option<Declaration>, ImageError> {
        let mut text = s3(0, &vectors(0x2001_0000, 0x9));
        for (address, data) in parts {
            text.push_str(&s3(*address, data));
        }
        Image::parse(text.as_bytes()).unwrap().declaration()
    }

    #[test]
    fn a_declaration_is_found_by_its_magic_at_an_aligned_address() {
        let found = declared(&[(0x130, record(1, 16, 0b11))]).unwrap().unwrap();
        assert_eq!(
            found,
            Declaration {
                address: 0x130,
                format: 1,
                boards: 0b11
            }
        );
        // A chunk that starts off a 4-byte boundary: the record sits at offset 2 of it, and at
        // the aligned address 0x204.
        let mut shifted = vec![0xEE, 0xEE];
        shifted.extend(record(1, 16, 0b10));
        let found = declared(&[(0x202, shifted)]).unwrap().unwrap();
        assert_eq!((found.address, found.boards), (0x204, 0b10));
        // The same bytes one address off: not aligned, so not a declaration.
        let mut unaligned = vec![0xEE];
        unaligned.extend(record(1, 16, 0b10));
        assert_eq!(declared(&[(0x200, unaligned)]), Ok(None));
        assert_eq!(declared(&[]), Ok(None));
        // A later format keeps the boards where format 1 put them, and may be longer.
        let mut later = record(2, 20, 0b10);
        later.extend_from_slice(&[0xAB; 4]);
        assert_eq!(declared(&[(0x130, later)]).unwrap().unwrap().boards, 0b10);
    }

    #[test]
    fn a_malformed_declaration_is_refused() {
        let bad = |parts: &[(u32, Vec<u8>)]| match declared(parts) {
            Err(ImageError::BadDeclaration { address, reason }) => (address, reason),
            other => panic!("{other:?}"),
        };
        assert_eq!(
            declared(&[(0x130, record(1, 16, 1)), (0x40000, record(1, 16, 2))]),
            Err(ImageError::TwoDeclarations {
                first: 0x130,
                second: 0x40000
            })
        );
        assert_eq!(bad(&[(0x130, record(0, 16, 1))]).1, "its format is 0");
        assert!(bad(&[(0x130, record(1, 15, 1))]).1.contains("length is 15"));
        // Its length says 20, and its part of the image ends after 16.
        assert!(bad(&[(0x130, record(1, 20, 1))]).1.contains("runs past"));
        // The magic ends its part of the image.
        assert!(
            bad(&[(0x130, b"VERGEIMG\x01".to_vec())])
                .1
                .contains("runs past")
        );
        let image = Image::parse(
            [
                s3(0, &vectors(0x2001_0000, 0x5)),
                s3(0x130, &record(1, 15, 1)),
            ]
            .concat()
            .as_bytes(),
        )
        .unwrap();
        assert!(matches!(
            image.check_for_mculink(),
            Err(ImageError::BadDeclaration { address: 0x130, .. })
        ));
    }

    #[test]
    fn the_boards_an_image_runs_on() {
        let boards = |parts: &[(u32, Vec<u8>)]| {
            let mut text = s3(0, &vectors(0x2001_0000, 0x9));
            for (address, data) in parts {
                text.push_str(&s3(*address, data));
            }
            Image::parse(text.as_bytes()).unwrap().boards().unwrap()
        };
        assert_eq!(boards(&[]), [Board::McuLink]);
        assert_eq!(boards(&[(0x130, record(1, 16, 0b01))]), [Board::McuLink]);
        assert_eq!(boards(&[(0x130, record(1, 16, 0b10))]), [Board::McuLinkPro]);
        assert_eq!(
            boards(&[(0x130, record(1, 16, 0b11))]),
            [Board::McuLink, Board::McuLinkPro]
        );
        // Board 2, on-board, and board 31 are not boards this tool knows.
        assert_eq!(boards(&[(0x130, record(1, 16, 1 << 2 | 1 << 31))]), []);
    }

    #[test]
    fn neither_format_is_refused() {
        assert_eq!(Image::parse(b"\x00\x01"), Err(ImageError::UnknownFormat));
        assert_eq!(Image::parse(b""), Err(ImageError::UnknownFormat));
    }

    #[test]
    fn srecord_round_trip() {
        let text = [s3(0, &vectors(0x2001_0000, 0x9)), s3(0x40, &[5; 70])].concat();
        let image = Image::parse(text.as_bytes()).unwrap();
        let again = Image::parse(image.to_srecord("x").as_bytes()).unwrap();
        assert_eq!(again.chunks, image.chunks);
        assert_eq!(again.header.as_deref(), Some("x"));
    }
}
