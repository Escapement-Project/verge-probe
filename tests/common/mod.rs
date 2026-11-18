//! Stream messages as a probe sends them, for the tests to play back.
#![allow(dead_code)]

/// An identified stream header at 1,000 pairs a second, reporting the edges of `pins`.
pub fn header(pins: u8, levels: u8) -> Vec<u8> {
    let mut header = vec![0xA1, 4];
    header.extend_from_slice(&96_000_000u32.to_le_bytes());
    header.extend_from_slice(&96_000u32.to_le_bytes());
    header.push(1);
    header.extend_from_slice(&3300u16.to_le_bytes());
    header.extend_from_slice(&1u32.to_le_bytes());
    header.push(1);
    // Both ranges: two points, then two empty slots.
    let mut table = vec![2];
    for (code, nanoamps) in [(0u16, 0u32), (60_000, 60_000), (0, 0), (0, 0)] {
        table.extend_from_slice(&code.to_le_bytes());
        table.extend_from_slice(&nanoamps.to_le_bytes());
    }
    header.extend_from_slice(&table);
    header.extend_from_slice(&table);
    header.extend_from_slice(&[0; 4]);
    header.extend_from_slice(&[0; 16]);
    header.extend_from_slice(&1u64.to_le_bytes());
    // Step 5's fields, zero.
    header.extend_from_slice(&[0; 7]);
    header[68] = pins;
    header[69] = levels;
    header[70] = 11;
    header
}

pub fn pairs(first_sequence: u32, count: u16) -> Vec<u8> {
    let mut message = vec![0xA0, 0];
    message.extend_from_slice(&first_sequence.to_le_bytes());
    message.extend_from_slice(&(u64::from(first_sequence - 1) * 1_000_000).to_le_bytes());
    message.extend_from_slice(&count.to_le_bytes());
    for _ in 0..count {
        message.extend_from_slice(&[0x00, 0x80, 0x10, 0x00, 1]);
    }
    message
}

pub fn end(reason: u8, last_sequence: u32) -> Vec<u8> {
    let mut end = vec![0xA2, reason];
    end.extend_from_slice(&last_sequence.to_le_bytes());
    end
}
