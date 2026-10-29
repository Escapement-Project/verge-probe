//! Verge Aux's energy stream (mculink-verge HOST-INTERFACE.md, "The energy stream"): its messages,
//! each pair's conversion to millivolts and nanoamps, and a stream's total as the probe keeps it.
//!
//! The conversion is the firmware's, step for step and on integers, so a stream started with the
//! capture sums to the total the probe reports. The firmware writes vectors for it
//! (`tools/energy_vectors.json`); `tests/stream.rs` holds this converter to a copy of them.

use thiserror::Error;

use crate::probe::CalibrationPoint;

pub const PAIRS: u8 = 0xA0;
pub const HEADER: u8 = 0xA1;
pub const END: u8 = 0xA2;
pub const EDGES: u8 = 0xA3;

const HEADER_BYTES: usize = 68;
const PAIRS_FIXED: usize = 16;
const PAIR_BYTES: usize = 5;
const END_BYTES: usize = 6;

/// Whether a message from the auxiliary interface is a stream message rather than a reply: its first
/// byte is outside the vendor command range.
#[must_use]
pub fn is_stream_message(bytes: &[u8]) -> bool {
    bytes
        .first()
        .is_some_and(|&first| (PAIRS..=EDGES).contains(&first))
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum StreamError {
    #[error("a stream message of {0} bytes is too short for its kind")]
    Short(usize),
    #[error("0x{0:02x} starts no stream message")]
    Unknown(u8),
    #[error("stream format {0} is newer than this tool reads")]
    Format(u8),
}

/// The first message of a stream: what converting its pairs takes.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Header {
    /// Pairs a second.
    pub rate: u32,
    /// The current range jumper: 350 or 50 mA, `None` when the probe could not read it.
    pub setting_ma: Option<u16>,
    pub full_scale_mv: u16,
    pub first_sequence: u32,
    /// Whether the pair before the first one sent was on the low range.
    pub previous_low: bool,
    pub low: Vec<CalibrationPoint>,
    pub high: Vec<CalibrationPoint>,
    /// The DIO pins whose edges the stream reports, their levels as it started, and the edges'
    /// time resolution.
    pub dio_mask: u8,
    pub dio_levels: u8,
    pub edge_resolution_ns: u16,
}

/// One voltage and current reading, as the converter left them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pair {
    pub voltage_code: u16,
    pub current_code: u16,
    pub low: bool,
}

/// Up to 99 evenly spaced pairs.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Pairs {
    pub flags: u8,
    pub first_sequence: u32,
    /// The first pair's time, in ns since the capture started.
    pub time_ns: u64,
    pub pairs: Vec<Pair>,
}

impl Pairs {
    pub const PAUSED: u8 = 1;
    pub const HELD: u8 = 2;

    /// A message as the probe would send it, for a simulator or a test.
    #[must_use]
    pub fn new(flags: u8, first_sequence: u32, time_ns: u64, pairs: Vec<Pair>) -> Pairs {
        Pairs {
            flags,
            first_sequence,
            time_ns,
            pairs,
        }
    }

    /// The capture paused before the first pair: a pair waiting for the next one's current kept
    /// its own, and the range before is low again.
    #[must_use]
    pub fn paused(&self) -> bool {
        self.flags & Self::PAUSED != 0
    }

    /// The conversions were held before the first pair, as a debugger connected: time jumped, and
    /// nothing else.
    #[must_use]
    pub fn held(&self) -> bool {
        self.flags & Self::HELD != 0
    }

    /// Pair `index`'s time, in ns since the capture started, at the header's `rate`.
    #[must_use]
    pub fn time_of(&self, index: usize, rate: u32) -> u64 {
        self.time_ns + index as u64 * 1_000_000_000 / u64::from(rate.max(1))
    }
}

/// Why a stream ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EndReason {
    /// A host sent `0x90` mode 7.
    Asked,
    Stopped,
    /// A new start, on either interface, restarted the capture.
    Restarted,
    /// The capture failed: `0x90` result 7.
    Failed,
    Other(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct End {
    pub reason: EndReason,
    /// The last pair the capture took.
    pub last_sequence: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Message {
    Header(Header),
    Pairs(Pairs),
    End(End),
    /// DIO edges, which this firmware does not send yet.
    Edges(Vec<u8>),
}

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().expect("four bytes"))
}

fn points(bytes: &[u8], at: usize) -> Vec<CalibrationPoint> {
    let count = usize::from(bytes[at]).min(4);
    (0..count)
        .map(|i| CalibrationPoint {
            code: u16_at(bytes, at + 1 + 6 * i),
            current_nanoamps: u32_at(bytes, at + 3 + 6 * i),
        })
        .collect()
}

/// One message from the auxiliary interface's IN endpoint.
pub fn parse(bytes: &[u8]) -> Result<Message, StreamError> {
    let short = || StreamError::Short(bytes.len());
    match bytes.first() {
        Some(&HEADER) => {
            if bytes.len() < HEADER_BYTES {
                return Err(short());
            }
            if bytes[1] != 1 {
                return Err(StreamError::Format(bytes[1]));
            }
            Ok(Message::Header(Header {
                rate: u32_at(bytes, 2),
                setting_ma: match bytes[6] {
                    1 => Some(350),
                    2 => Some(50),
                    _ => None,
                },
                full_scale_mv: u16_at(bytes, 7),
                first_sequence: u32_at(bytes, 9),
                previous_low: bytes[13] & 1 != 0,
                low: points(bytes, 14),
                high: points(bytes, 39),
                dio_mask: bytes[64],
                dio_levels: bytes[65],
                edge_resolution_ns: u16_at(bytes, 66),
            }))
        }
        Some(&PAIRS) => {
            if bytes.len() < PAIRS_FIXED {
                return Err(short());
            }
            let count = usize::from(u16_at(bytes, 14));
            if bytes.len() < PAIRS_FIXED + count * PAIR_BYTES {
                return Err(short());
            }
            Ok(Message::Pairs(Pairs {
                flags: bytes[1],
                first_sequence: u32_at(bytes, 2),
                time_ns: u64::from_le_bytes(bytes[6..14].try_into().expect("eight bytes")),
                pairs: bytes[PAIRS_FIXED..PAIRS_FIXED + count * PAIR_BYTES]
                    .as_chunks::<PAIR_BYTES>()
                    .0
                    .iter()
                    .map(|pair| Pair {
                        voltage_code: u16_at(pair, 0),
                        current_code: u16_at(pair, 2),
                        low: pair[4] & 1 != 0,
                    })
                    .collect(),
            }))
        }
        Some(&END) => {
            if bytes.len() < END_BYTES {
                return Err(short());
            }
            Ok(Message::End(End {
                reason: match bytes[1] {
                    1 => EndReason::Asked,
                    2 => EndReason::Stopped,
                    3 => EndReason::Restarted,
                    4 => EndReason::Failed,
                    other => EndReason::Other(other),
                },
                last_sequence: u32_at(bytes, 2),
            }))
        }
        Some(&EDGES) => Ok(Message::Edges(bytes.to_vec())),
        Some(&other) => Err(StreamError::Unknown(other)),
        None => Err(short()),
    }
}

/// Step 1: the voltage channel halves the target side's voltage on its way to the converter.
#[must_use]
pub fn millivolts(voltage_code: u16, full_scale_mv: u16) -> u32 {
    (u32::from(voltage_code) * 2 * u32::from(full_scale_mv)) >> 16
}

/// Step 4: the voltage divider's own current through the sense resistor, taken off.
#[must_use]
pub fn target_nanoamps(shunt_nanoamps: u32, millivolts: u32) -> u32 {
    let divider = ((u64::from(millivolts) * 99_297) >> 16) as u32;
    shunt_nanoamps.saturating_sub(divider)
}

/// A range's points as a piecewise-linear table, with the slopes the firmware computes from them.
#[derive(Clone, Debug)]
struct Table {
    points: Vec<CalibrationPoint>,
    slopes: Vec<u64>,
}

impl Table {
    fn new(points: &[CalibrationPoint]) -> Table {
        let points: Vec<CalibrationPoint> = points.iter().take(4).copied().collect();
        let slopes = points
            .windows(2)
            .map(|pair| {
                let current = u64::from(
                    pair[1]
                        .current_nanoamps
                        .wrapping_sub(pair[0].current_nanoamps),
                );
                let code = u64::from(pair[1].code.wrapping_sub(pair[0].code)).max(1);
                (current << 16) / code
            })
            .collect();
        Table { points, slopes }
    }

    /// Step 3: the current through the sense resistor. The segment is the last point at or below
    /// the code among the first n - 1; a code below the first point uses the first segment.
    fn shunt_nanoamps(&self, code: u16) -> u32 {
        let Some(first) = self.points.first() else {
            return 0;
        };
        // One point, as an uncalibrated range can report, is a flat line through it.
        if self.points.len() < 2 {
            return first.current_nanoamps;
        }
        let mut segment = 0;
        while segment + 2 < self.points.len() && code >= self.points[segment + 1].code {
            segment += 1;
        }
        let point = self.points[segment];
        let delta = i64::from(code) - i64::from(point.code);
        // Rust's division truncates toward zero, as C's does.
        let correction = i128::from(delta) * i128::from(self.slopes[segment]) / 65_536;
        let current = i128::from(point.current_nanoamps) + correction;
        current.clamp(0, i128::from(u32::MAX)) as u32
    }
}

/// One pair in physical units, steps 1 to 4: the latest reply's calibrated fields for its codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
    pub millivolts: u32,
    pub shunt_nanoamps: u32,
    pub nanoamps: u32,
}

/// A stream's total, as `0x90` mode 3 counts it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Total {
    pub picojoules: u64,
    pub fraction_q32: u32,
    pub pairs: u64,
}

impl Total {
    #[must_use]
    pub fn nanojoules(&self) -> u64 {
        self.picojoules / 1000
    }
}

/// Converts a stream's pairs, and sums them as the probe does (step 5 and the total).
#[derive(Clone, Debug)]
pub struct Converter {
    full_scale_mv: u16,
    low: Table,
    high: Table,
    scale_q32: u64,
    previous_low: bool,
    /// A pair whose range differs from the one before, waiting for the next pair's current.
    held: Option<(u32, u32)>,
    total: Total,
}

impl Converter {
    #[must_use]
    pub fn new(header: &Header) -> Converter {
        Converter {
            full_scale_mv: header.full_scale_mv,
            low: Table::new(&header.low),
            high: Table::new(&header.high),
            scale_q32: (1u64 << 32) / u64::from(header.rate.max(1)),
            previous_low: header.previous_low,
            held: None,
            total: Total::default(),
        }
    }

    /// Steps 1 to 4 for one pair, alone.
    #[must_use]
    pub fn reading(&self, pair: &Pair) -> Reading {
        let millivolts = millivolts(pair.voltage_code, self.full_scale_mv);
        let table = if pair.low { &self.low } else { &self.high };
        let shunt_nanoamps = table.shunt_nanoamps(pair.current_code);
        Reading {
            millivolts,
            shunt_nanoamps,
            nanoamps: target_nanoamps(shunt_nanoamps, millivolts),
        }
    }

    fn add_pair(&mut self, millivolts: u32, shunt_nanoamps: u32) {
        let power = u64::from(millivolts) * u64::from(target_nanoamps(shunt_nanoamps, millivolts));
        let low = (power & 0xFFFF_FFFF) * self.scale_q32;
        let mut increment = (power >> 32) * self.scale_q32 + (low >> 32);
        let previous = self.total.fraction_q32;
        self.total.fraction_q32 = previous.wrapping_add(low as u32);
        increment += u64::from(self.total.fraction_q32 < previous);
        self.total.picojoules = self.total.picojoules.saturating_add(increment);
        self.total.pairs = self.total.pairs.saturating_add(1);
    }

    /// A pause: a waiting pair keeps its own current, and the range before is low again.
    pub fn pause(&mut self) {
        if let Some((millivolts, shunt)) = self.held.take() {
            self.add_pair(millivolts, shunt);
        }
        self.previous_low = true;
    }

    /// One pair into the total, step 5 included.
    pub fn add(&mut self, pair: &Pair) {
        let reading = self.reading(pair);
        let switched = pair.low != self.previous_low;
        if !switched && self.held.is_none() {
            self.add_pair(reading.millivolts, reading.shunt_nanoamps);
            return;
        }
        self.previous_low = pair.low;
        if let Some((millivolts, own)) = self.held.take() {
            self.add_pair(
                millivolts,
                if switched {
                    own
                } else {
                    reading.shunt_nanoamps
                },
            );
        }
        if switched {
            self.held = Some((reading.millivolts, reading.shunt_nanoamps));
            return;
        }
        self.add_pair(reading.millivolts, reading.shunt_nanoamps);
    }

    /// A pairs message into the total.
    pub fn add_message(&mut self, pairs: &Pairs) {
        if pairs.paused() {
            self.pause();
        }
        for pair in &pairs.pairs {
            self.add(pair);
        }
    }

    /// The total once the stream has ended: a pair still waiting keeps its own current.
    #[must_use]
    pub fn finish(mut self) -> Total {
        if let Some((millivolts, shunt)) = self.held.take() {
            self.add_pair(millivolts, shunt);
        }
        self.total
    }

    /// The total so far, leaving a pair still waiting out of it.
    #[must_use]
    pub fn total(&self) -> Total {
        self.total
    }
}
