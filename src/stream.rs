//! Verge Aux's energy stream (mculink-verge HOST-INTERFACE.md, "The energy stream"): its messages,
//! each pair's conversion to millivolts and nanoamps, and a stream's total as the probe keeps it.
//!
//! The conversion is the firmware's, step for step and on integers, so a stream started with the
//! capture sums to the total the probe reports. The firmware writes vectors for it
//! (`tools/energy_vectors.json`); `tests/stream.rs` holds this converter to a copy of them.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::probe::CalibrationPoint;

pub const PAIRS: u8 = 0xA0;
pub const HEADER: u8 = 0xA1;
pub const END: u8 = 0xA2;
pub const EDGES: u8 = 0xA3;
pub const MARKS: u8 = 0xA4;
/// The longest label a marker carries.
pub const LABEL_MAX: usize = 32;

const HEADER_BYTES: usize = 68;
const PAIRS_FIXED: usize = 16;
const PAIR_BYTES: usize = 5;
const END_BYTES: usize = 6;
const EDGES_FIXED: usize = 12;
const EDGE_BYTES: usize = 5;
const MARKS_FIXED: usize = 3;
/// A marker's time, number and label length, before its label.
const MARK_BYTES: usize = 13;

/// Whether a message from the auxiliary interface is a stream message rather than a reply: its first
/// byte is outside the vendor command range.
#[must_use]
pub fn is_stream_message(bytes: &[u8]) -> bool {
    bytes
        .first()
        .is_some_and(|&first| (PAIRS..=MARKS).contains(&first))
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum StreamError {
    #[error("the stream header has no valid capture identity")]
    Identity,
    #[error("a stream message of {0} bytes is too short for its kind")]
    Short(usize),
    #[error("0x{0:02x} starts no stream message")]
    Unknown(u8),
    #[error("stream format {0} is newer than this tool reads")]
    Format(u8),
}

/// The first message of a stream: what converting its pairs takes.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Header {
    #[serde(skip)]
    pub capture_id: Option<crate::CaptureId>,
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
    /// Step 5: the previous pair's share, in 1/65536, of the current a pair that switched up takes.
    /// 0 from firmware older than the field.
    #[serde(default)]
    pub switch_weight_q16: u16,
    /// Step 5: the probe re-arms the low range between pairs, so a pair that switched down keeps
    /// its own current.
    #[serde(default)]
    pub rearm_between_pairs: bool,
    /// The current through the sense resistor of the pair before the first one sent, where the
    /// probe had one: a stream attached to a running capture.
    #[serde(default)]
    pub previous_shunt_nanoamps: Option<u32>,
}

/// One voltage and current reading, as the converter left them.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pair {
    pub voltage_code: u16,
    pub current_code: u16,
    pub low: bool,
}

/// Up to 99 evenly spaced pairs.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
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
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
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

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct End {
    pub reason: EndReason,
    /// The last pair the capture took.
    pub last_sequence: u32,
}

/// One DIO edge.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Edge {
    /// In ns since the capture started.
    pub time_ns: u64,
    /// Every reporting pin's level after the edge, bit `k` for DIO`k`; other pins read 0.
    pub levels: u8,
}

/// Up to 99 DIO edges, in time order. Every edge up to a pairs message's last pair arrives before
/// the next pairs message or the end.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Edges {
    pub flags: u8,
    pub edges: Vec<Edge>,
}

impl Edges {
    pub const LOST: u8 = 1;

    /// A message as the probe would send it, for a simulator or a test.
    #[must_use]
    pub fn new(flags: u8, edges: Vec<Edge>) -> Edges {
        Edges { flags, edges }
    }

    /// The probe lost edges before the first one, so the levels before it are not known.
    #[must_use]
    pub fn lost(&self) -> bool {
        self.flags & Self::LOST != 0
    }

    /// The message's bytes, as the probe sends them. Empty without edges, which the probe never
    /// sends.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let Some(first) = self.edges.first() else {
            return Vec::new();
        };
        let mut bytes = vec![EDGES, self.flags];
        bytes.extend_from_slice(&first.time_ns.to_le_bytes());
        bytes.extend_from_slice(&(self.edges.len() as u16).to_le_bytes());
        for edge in &self.edges {
            let offset =
                u32::try_from(edge.time_ns - first.time_ns).expect("a message spans under 4 s");
            bytes.extend_from_slice(&offset.to_le_bytes());
            bytes.push(edge.levels);
        }
        bytes
    }
}

/// One moment a debugging host marked with `0x94`.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub struct Marker {
    /// In ns since the capture started, as the probe ran the command.
    pub time_ns: u64,
    pub number: u32,
    /// Up to 32 bytes, UTF-8 by convention, as the host sent them.
    pub label: Vec<u8>,
}

/// Up to 11 markers, in the order the probe ran them. They can arrive before or after the pairs
/// around them.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Markers {
    pub flags: u8,
    pub markers: Vec<Marker>,
}

impl Markers {
    pub const LOST: u8 = 1;

    /// A message as the probe would send it, for a simulator or a test.
    #[must_use]
    pub fn new(flags: u8, markers: Vec<Marker>) -> Markers {
        Markers { flags, markers }
    }

    /// The probe lost markers before the first one: the record was full, or so was the host's.
    #[must_use]
    pub fn lost(&self) -> bool {
        self.flags & Self::LOST != 0
    }

    /// The message's bytes, as the probe sends them.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = vec![MARKS, self.flags, self.markers.len() as u8];
        for marker in &self.markers {
            bytes.extend_from_slice(&marker.time_ns.to_le_bytes());
            bytes.extend_from_slice(&marker.number.to_le_bytes());
            bytes.push(marker.label.len() as u8);
            bytes.extend_from_slice(&marker.label);
        }
        bytes
    }
}

/// Recorded captures store these by variant index: add new ones at the end.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub enum Message {
    Header(Header),
    Pairs(Pairs),
    End(End),
    Edges(Edges),
    Markers(Markers),
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
            if !matches!(bytes[1], 1 | 2) {
                return Err(StreamError::Format(bytes[1]));
            }
            let capture_id = if bytes[1] == 2 {
                let bytes = bytes.get(68..92).ok_or_else(short)?;
                Some(
                    crate::CaptureId::from_bytes(bytes.try_into().expect("24 bytes"))
                        .ok_or(StreamError::Identity)?,
                )
            } else {
                None
            };
            // Step 5's fields follow the identity in format 2; firmware before them ends there.
            let at = if bytes[1] == 2 { 92 } else { HEADER_BYTES };
            let (switch_weight_q16, flags, previous) = match bytes.get(at..at + 7) {
                Some(fields) => (u16_at(fields, 0), fields[2], u32_at(fields, 3)),
                None => (0, 0, 0),
            };
            Ok(Message::Header(Header {
                capture_id,
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
                switch_weight_q16: switch_weight_q16.min(32768),
                rearm_between_pairs: flags & 1 != 0,
                previous_shunt_nanoamps: (flags & 2 != 0).then_some(previous),
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
        Some(&EDGES) => {
            if bytes.len() < EDGES_FIXED {
                return Err(short());
            }
            let count = usize::from(u16_at(bytes, 10));
            if bytes.len() < EDGES_FIXED + count * EDGE_BYTES {
                return Err(short());
            }
            let first = u64::from_le_bytes(bytes[2..10].try_into().expect("eight bytes"));
            Ok(Message::Edges(Edges {
                flags: bytes[1],
                edges: bytes[EDGES_FIXED..EDGES_FIXED + count * EDGE_BYTES]
                    .as_chunks::<EDGE_BYTES>()
                    .0
                    .iter()
                    .map(|edge| Edge {
                        time_ns: first + u64::from(u32_at(edge, 0)),
                        levels: edge[4],
                    })
                    .collect(),
            }))
        }
        Some(&MARKS) => {
            if bytes.len() < MARKS_FIXED {
                return Err(short());
            }
            let mut at = MARKS_FIXED;
            let mut markers = Vec::with_capacity(usize::from(bytes[2]));
            for _ in 0..bytes[2] {
                let label = at + MARK_BYTES;
                if bytes.len() < label || bytes.len() < label + usize::from(bytes[label - 1]) {
                    return Err(short());
                }
                let end = label + usize::from(bytes[label - 1]);
                markers.push(Marker {
                    time_ns: u64::from_le_bytes(bytes[at..at + 8].try_into().expect("eight bytes")),
                    number: u32_at(bytes, at + 8),
                    label: bytes[label..end].to_vec(),
                });
                at = end;
            }
            Ok(Message::Markers(Markers {
                flags: bytes[1],
                markers,
            }))
        }
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

/// The current range a pair was converted on.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurrentRange {
    Low,
    High,
}

impl CurrentRange {
    #[must_use]
    pub fn new(low: bool) -> CurrentRange {
        if low {
            CurrentRange::Low
        } else {
            CurrentRange::High
        }
    }
}

/// One pair as the total counts it: its voltage, and the current step 5 gives it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub sequence: u32,
    /// In ns since the capture started.
    pub time_ns: u64,
    pub millivolts: u32,
    pub nanoamps: u32,
    /// The pair's own range. Step 5 can give it a current from the pairs either side, which then
    /// share it. None only for a sample from a capture archive older than this field.
    pub range: Option<CurrentRange>,
}

/// A pair whose range differs from the one before, waiting for the next pair's current.
#[derive(Clone, Copy, Debug)]
struct Waiting {
    sequence: u32,
    time_ns: u64,
    millivolts: u32,
    shunt_nanoamps: u32,
    low: bool,
    /// The previous pair's share of the stand-in, and its current: 0 but for a switch up after a pair.
    weight_q16: u16,
    before_nanoamps: u32,
}

/// Step 5's stand-in: the next pair's current less the previous pair's share.
fn stand_in(next_nanoamps: u32, before_nanoamps: u32, weight_q16: u16) -> u32 {
    let weight = u64::from(weight_q16);
    ((u64::from(next_nanoamps) * (65536 - weight) + u64::from(before_nanoamps) * weight + 32768)
        >> 16) as u32
}

/// Converts a stream's pairs, and sums them as the probe does (step 5 and the total).
#[derive(Clone, Debug)]
pub struct Converter {
    rate: u32,
    full_scale_mv: u16,
    low: Table,
    high: Table,
    scale_q32: u64,
    previous_low: bool,
    /// The last pair's own current through the sense resistor; none since a pause or a gap.
    before: Option<u32>,
    switch_weight_q16: u16,
    rearm_between_pairs: bool,
    waiting: Option<Waiting>,
    total: Total,
    samples: Vec<Sample>,
}

impl Converter {
    #[must_use]
    pub fn new(header: &Header) -> Converter {
        Converter {
            rate: header.rate,
            full_scale_mv: header.full_scale_mv,
            low: Table::new(&header.low),
            high: Table::new(&header.high),
            scale_q32: (1u64 << 32) / u64::from(header.rate.max(1)),
            previous_low: header.previous_low,
            before: header.previous_shunt_nanoamps,
            switch_weight_q16: header.switch_weight_q16.min(32768),
            rearm_between_pairs: header.rearm_between_pairs,
            waiting: None,
            total: Total::default(),
            samples: Vec::new(),
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

    fn count(&mut self, pair: Waiting, shunt_nanoamps: u32) {
        let nanoamps = target_nanoamps(shunt_nanoamps, pair.millivolts);
        let power = u64::from(pair.millivolts) * u64::from(nanoamps);
        let low = (power & 0xFFFF_FFFF) * self.scale_q32;
        let mut increment = (power >> 32) * self.scale_q32 + (low >> 32);
        let previous = self.total.fraction_q32;
        self.total.fraction_q32 = previous.wrapping_add(low as u32);
        increment += u64::from(self.total.fraction_q32 < previous);
        self.total.picojoules = self.total.picojoules.saturating_add(increment);
        self.total.pairs = self.total.pairs.saturating_add(1);
        self.samples.push(Sample {
            sequence: pair.sequence,
            time_ns: pair.time_ns,
            millivolts: pair.millivolts,
            nanoamps,
            range: Some(CurrentRange::new(pair.low)),
        });
    }

    /// A pause: a waiting pair keeps its own current, the range before is low again, and there is
    /// no pair before for the stand-in.
    fn pause(&mut self) {
        if let Some(waiting) = self.waiting.take() {
            self.count(waiting, waiting.shunt_nanoamps);
        }
        self.previous_low = true;
        self.before = None;
    }

    /// One pair into the total, step 5 included.
    fn add(&mut self, pair: &Pair, sequence: u32, time_ns: u64) {
        let reading = self.reading(pair);
        let before = self.before.replace(reading.shunt_nanoamps);
        let up_after_a_pair = !pair.low && before.is_some();
        let this = Waiting {
            sequence,
            time_ns,
            millivolts: reading.millivolts,
            shunt_nanoamps: reading.shunt_nanoamps,
            low: pair.low,
            weight_q16: if up_after_a_pair {
                self.switch_weight_q16
            } else {
                0
            },
            before_nanoamps: before.unwrap_or(0),
        };
        let switched = pair.low != self.previous_low;
        self.previous_low = pair.low;
        if let Some(waiting) = self.waiting.take() {
            let shunt = if switched {
                waiting.shunt_nanoamps
            } else {
                stand_in(
                    reading.shunt_nanoamps,
                    waiting.before_nanoamps,
                    waiting.weight_q16,
                )
            };
            self.count(waiting, shunt);
        }
        if switched && !(pair.low && self.rearm_between_pairs) {
            self.waiting = Some(this);
        } else {
            self.count(this, reading.shunt_nanoamps);
        }
    }

    /// A pairs message into the total.
    pub fn add_message(&mut self, pairs: &Pairs) {
        if pairs.paused() {
            self.pause();
        }
        for (index, pair) in pairs.pairs.iter().enumerate() {
            let sequence = pairs.first_sequence.wrapping_add(index as u32);
            self.add(pair, sequence, pairs.time_of(index, self.rate));
        }
    }

    /// A missing sequence leaves a pending range-switch pair without its correcting current.
    /// Keep its own reading and start the next received range without inferring a lost switch.
    pub fn discontinuity(&mut self, next_low: bool) {
        self.end();
        self.previous_low = next_low;
        self.before = None;
    }

    /// The stream ended: a pair still waiting keeps its own current.
    pub fn end(&mut self) {
        if let Some(waiting) = self.waiting.take() {
            self.count(waiting, waiting.shunt_nanoamps);
        }
    }

    /// The pairs the total has counted since the last call, in order. A pair waiting at a range
    /// switch comes with the pair after it, or at [`Converter::end`].
    pub fn take_samples(&mut self) -> Vec<Sample> {
        std::mem::take(&mut self.samples)
    }

    /// The total so far, leaving a pair still waiting out of it.
    #[must_use]
    pub fn total(&self) -> Total {
        self.total
    }
}
