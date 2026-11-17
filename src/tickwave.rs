//! Energy captures as tickwave recordings (tickwave's FORMAT.md): a stream's messages in, a
//! `.tickwave` file's bytes out. Times are ticks of the probe's clock since the capture started.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use escapement_tickwave_format::decode::{DecodeError, Footer};
use escapement_tickwave_format::header::{
    AnalogCoding, Declaration, DeclaredChannel, DeclaredKind, Derived, Exact, IntCoding, JSON_SAFE,
    SourceFormat, SourceHeader,
};
use escapement_tickwave_format::mapping::{Spacing, finest_shift};
use escapement_tickwave_format::model::{Gap, Ratio};
use escapement_tickwave_format::recut::{Recutter, shifts_for};
use escapement_tickwave_format::snapshot::Snapshot;
use escapement_tickwave_format::writer::{
    Bundle, Compression, Finish, Finished, WriteOptions, Writer,
};

use crate::probe::CalibrationPoint;
use crate::record::temp_beside;
use crate::stream::skipped;
use crate::stream::{
    Converter, CurrentRange, DioHeader, End, EndReason, Header, Markers, Message, Pairs, Sample,
};

/// tickwave's format crate, for reading what this writes.
pub use escapement_tickwave_format as format;
/// tickwave's view crate, whose `Recording` [`export`] reads.
pub use escapement_tickwave_view as view;

pub const V: usize = 0;
pub const I: usize = 1;
pub const LOW_RANGE: usize = 2;
pub const UNDER_RANGE: usize = 3;
pub const FIRST_DIO: usize = 4;
/// The generator name markers carry as annotations.
pub const GENERATOR: &str = "verge";
pub const FORMAT: &str = "verge-energy";
/// A DIO stream's recording (`0x90` mode 15): the pins alone, channel `k` the `k`th pin reported.
pub const DIO_FORMAT: &str = "verge-dio";
/// A DIO recording's overview, 2^23 ticks a bucket, about 56 ms at 150 MHz: no pair period
/// gives the pins a rate while the recording runs. [`compact`] gives each pin its own.
const DIO_OVERVIEW_SHIFT: u8 = 23;

mod export;
pub use export::export;

#[cfg(all(test, feature = "capture"))]
mod tests;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    #[error("the capture restarted before its stream ended")]
    Restarted,
    #[error("a stream of {0} pairs a second, which a recording cannot hold")]
    Rate(f64),
    #[error("the stream's times went back")]
    Timestamps,
    #[error("the stream's end reports {0} pairs lost after the last, more than a recording holds")]
    LostAfter(u64),
    #[error("{} already exists", .0.display())]
    Exists(PathBuf),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("writing the recording: {0}")]
    Write(#[from] escapement_tickwave_format::writer::WriteError),
    #[error("compacting the recording: {0}")]
    Recut(#[from] escapement_tickwave_format::recut::RecutError),
    #[error("reading the recording: {0}")]
    View(#[from] escapement_tickwave_view::Error),
    #[error(transparent)]
    Record(#[from] crate::record::RecordError),
    #[error("the recording isn't an energy capture Verge recorded")]
    NotVerge,
    #[error("the DIO stream reports no pins")]
    NoPins,
    #[error("a DIO stream sent pairs")]
    DioPairs,
    #[error("cancelled")]
    Cancelled,
}

type Result<T> = std::result::Result<T, Error>;

/// The probe's time `ns` as ticks of `clock_hz`. The probe rounds counts of its clock down to ns,
/// and a count is longer than 2 ns, so rounding up gives the count back exactly, for one rounding
/// and for an edge's two (its message's first time plus its offset).
#[must_use]
pub fn tick(ns: u64, clock_hz: u32) -> i64 {
    ((u128::from(ns) * u128::from(clock_hz)).div_ceil(1_000_000_000)) as i64
}

/// Tick `tick` of `clock_hz` in ns, rounded down as the probe rounds it.
#[must_use]
pub fn nanos(tick: i64, clock_hz: u32) -> u64 {
    (u128::from(tick.max(0) as u64) * 1_000_000_000 / u128::from(clock_hz.max(1))) as u64
}

/// What the recording says about where it came from, beyond the stream.
#[derive(Clone, Debug, Default)]
pub struct Context {
    /// The probe's model, as "MCU-Link Pro".
    pub instrument: Option<String>,
    /// The firmware's version.
    pub software: Option<String>,
    /// The host's clock as the recording started, RFC 3339 in UTC.
    pub started_utc: Option<String>,
}

pub fn write_options() -> WriteOptions {
    WriteOptions {
        level: 16,
        block_bytes: 1 << 20,
        promise_with_items: false,
        compression: Compression::Spread,
    }
}

/// The finest overview level's shift for a pair period, by tickwave's rule, which every channel
/// takes while the capture runs: the flags' and pins' rates are known only at its end, and
/// [`compact`] gives each its own. The source's ticks are the recording's, so a pair's spacing on
/// the axis is its period.
fn overview_shift(period: Ratio) -> u8 {
    finest_shift(Some(Spacing::of_samples(period, Ratio::ONE)))
}

fn int(scale: i64) -> AnalogCoding {
    let mut coding = IntCoding::new(32, false);
    coding.scale = Exact::new(1, scale);
    AnalogCoding::Int(coding)
}

fn analog(name: &str, unit: &str, period: Ratio, coding: AnalogCoding) -> DeclaredChannel {
    DeclaredChannel {
        name: name.into(),
        source_name: name.into(),
        source: 0,
        data: DeclaredKind::Analog {
            unit: unit.into(),
            period,
            coding,
            significant: Some(4),
            phase: None,
        },
        shaping: Default::default(),
        overview_shift: None,
        requires: Vec::new(),
        codec: None,
    }
}

fn digital(name: &str, initial: bool) -> DeclaredChannel {
    DeclaredChannel {
        name: name.into(),
        source_name: name.into(),
        source: 0,
        data: DeclaredKind::Digital { initial },
        shaping: Default::default(),
        overview_shift: None,
        requires: Vec::new(),
        codec: None,
    }
}

fn points(points: &[CalibrationPoint]) -> String {
    let pairs: Vec<[u64; 2]> = points
        .iter()
        .map(|p| [u64::from(p.code), u64::from(p.current_nanoamps)])
        .collect();
    serde_json::to_string(&pairs).expect("numbers serialise")
}

/// The stream header as a source's `extra`, field by field.
fn header_extra(header: &Header, context: &Context) -> BTreeMap<String, String> {
    let mut extra = BTreeMap::new();
    let mut put = |key: &str, value: String| {
        extra.insert(key.to_owned(), value);
    };
    put(
        "stream_format",
        if header.capture_id.is_some() {
            "4"
        } else {
            "3"
        }
        .into(),
    );
    put("clock_hz", header.clock_hz.to_string());
    put("period", header.period.to_string());
    put(
        "setting_ma",
        header
            .setting_ma
            .map_or("unknown".into(), |ma| ma.to_string()),
    );
    put("full_scale_mv", header.full_scale_mv.to_string());
    put("first_sequence", header.first_sequence.to_string());
    put("previous_low", header.previous_low.to_string());
    put("calibration_low", points(&header.low));
    put("calibration_high", points(&header.high));
    put("dio_mask", header.dio_mask.to_string());
    put("dio_levels", header.dio_levels.to_string());
    put("edge_resolution_ns", header.edge_resolution_ns.to_string());
    put("switch_weight_q16", header.switch_weight_q16.to_string());
    put(
        "rearm_between_pairs",
        header.rearm_between_pairs.to_string(),
    );
    if let Some(na) = header.previous_shunt_nanoamps {
        put("previous_shunt_nanoamps", na.to_string());
    }
    put("high_range_only", header.high_range_only.to_string());
    if let Some(id) = header.capture_id {
        put("capture_id", hex::encode(id.as_bytes()));
    }
    if let Some(started) = &context.started_utc {
        put("started_utc", started.clone());
    }
    extra
}

/// The stream header a recording's source states, as [`header_extra`] wrote it.
pub fn header_from(extra: &BTreeMap<String, String>) -> Option<Header> {
    fn get<T: std::str::FromStr>(extra: &BTreeMap<String, String>, key: &str) -> Option<T> {
        extra.get(key)?.parse().ok()
    }
    let calibration = |key: &str| -> Option<Vec<CalibrationPoint>> {
        let pairs: Vec<[u64; 2]> = serde_json::from_str(extra.get(key)?).ok()?;
        pairs
            .into_iter()
            .map(|[code, na]| {
                Some(CalibrationPoint {
                    code: code.try_into().ok()?,
                    current_nanoamps: na.try_into().ok()?,
                })
            })
            .collect()
    };
    let capture_id = match extra.get("capture_id") {
        Some(id) => Some(crate::CaptureId::from_bytes(
            hex::decode(id).ok()?.try_into().ok()?,
        )?),
        None => None,
    };
    Some(Header {
        capture_id,
        clock_hz: get(extra, "clock_hz")?,
        period: get(extra, "period")?,
        setting_ma: match extra.get("setting_ma")?.as_str() {
            "unknown" => None,
            ma => Some(ma.parse().ok()?),
        },
        full_scale_mv: get(extra, "full_scale_mv")?,
        first_sequence: get(extra, "first_sequence")?,
        previous_low: get(extra, "previous_low")?,
        low: calibration("calibration_low")?,
        high: calibration("calibration_high")?,
        dio_mask: get(extra, "dio_mask")?,
        dio_levels: get(extra, "dio_levels")?,
        edge_resolution_ns: get(extra, "edge_resolution_ns")?,
        switch_weight_q16: get(extra, "switch_weight_q16")?,
        rearm_between_pairs: get(extra, "rearm_between_pairs")?,
        previous_shunt_nanoamps: match extra.get("previous_shunt_nanoamps") {
            Some(na) => Some(na.parse().ok()?),
            None => None,
        },
        high_range_only: get(extra, "high_range_only")?,
    })
}

fn end_reason(reason: Option<EndReason>) -> String {
    match reason {
        None => "none".into(),
        Some(EndReason::Asked) => "asked".into(),
        Some(EndReason::Stopped) => "stopped".into(),
        Some(EndReason::Restarted) => "restarted".into(),
        Some(EndReason::Failed) => "failed".into(),
        Some(EndReason::Other(code)) => format!("other {code}"),
    }
}

#[derive(Clone, Copy, Debug)]
enum Dio {
    Edge { tick: i64, levels: u8 },
    Lost { from: i64, to: i64, levels: u8 },
}

impl Dio {
    fn tick(self) -> i64 {
        match self {
            Dio::Edge { tick, .. } => tick,
            Dio::Lost { from, .. } => from,
        }
    }
}

/// A run the sample with `sequence` starts.
struct RunStart {
    sequence: u32,
    tick: i64,
    lost: u64,
}

/// Every tick held here is the capture's, as the source's ticks are. The axis starts at `base`,
/// so an annotation, a promise, a flush and the length go to the writer `base` earlier.
struct Open {
    writer: Writer,
    header: Header,
    clock_hz: u32,
    /// The capture's tick at the recording's tick 0: where an attached stream can first stamp
    /// anything, 0 for a stream from the capture's start.
    base: i64,
    period: i64,
    converter: Converter,
    /// Each DIO channel's pin, in channel order.
    pins: Vec<u8>,
    /// The sequence number the next pair should carry.
    next: u32,
    /// The first pairs message's first pair: its tick.
    first_pair: Option<i64>,
    /// The last pair taken in, by its message: its tick.
    last_pair: Option<i64>,
    /// The runs whose first sample the converter has yet to give.
    runs: VecDeque<RunStart>,
    /// An attached stream's DIO items from before its first pairs message: its pins' first gap
    /// ends at that message's first pair, so they wait for it.
    early: Vec<Dio>,
    /// The latest DIO tick taken in: an edge's, or a loss report's `to`.
    dio_arrived: Option<i64>,
    last_sample: Option<i64>,
    /// low_range's and under_range's levels, once the first sample set them.
    range: [Option<bool>; 2],
    /// The levels the DIO channels hold; none until a stream attached to a running capture
    /// places its first gap.
    dio_levels: Option<u8>,
    /// The latest DIO tick placed: one past an edge, or a gap's `to`.
    dio_end: i64,
    /// Where an attached stream's first DIO gap ends: the pins are known there, so a loss report
    /// from before it starts past it.
    dio_start: Option<i64>,
    markers: u64,
    marker_losses: u64,
    /// Loss reports that came with no marker, which no time places.
    untimed_marker_losses: u64,
    marker_ticks: HashMap<i64, u32>,
    last_marker: Option<i64>,
    lost_after: u64,
    reason: Option<EndReason>,
    /// The latest promise: every item before it is in.
    promised: i64,
    flushed: i64,
}

/// A stream's messages in, a tickwave recording's bytes out. Sans-io: the caller writes
/// [`Encoder::take_output`] to the file.
pub struct Encoder {
    context: Context,
    open: Option<Open>,
    /// A DIO stream's recording, in place of `open`.
    dio: Option<DioOpen>,
    ended: bool,
}

impl Encoder {
    #[must_use]
    pub fn new(context: Context) -> Self {
        Self {
            context,
            open: None,
            dio: None,
            ended: false,
        }
    }

    /// One message. True once the stream has ended.
    pub fn ingest(&mut self, message: &Message) -> Result<bool> {
        if self.ended {
            return Ok(true);
        }
        if let Message::Header(_) | Message::DioHeader(_) = message
            && (self.open.is_some() || self.dio.is_some())
        {
            return Err(Error::Restarted);
        }
        if let Some(dio) = self.dio.as_mut() {
            self.ended = dio.ingest(message)?;
            return Ok(self.ended);
        }
        match message {
            Message::DioHeader(header) => self.dio = Some(DioOpen::new(header, &self.context)?),
            Message::Header(header) => self.open = Some(Open::new(header, &self.context)?),
            // Nothing before the header can be converted or placed.
            _ if self.open.is_none() => {}
            // A DIO stream's alone: a capture's pairs say how far it has got.
            Message::DioTime(_) => {}
            Message::Pairs(pairs) => self.open.as_mut().unwrap().pairs(pairs)?,
            Message::Edges(edges) => {
                let open = self.open.as_mut().unwrap();
                for edge in &edges.edges {
                    let tick = open.stamp(edge.time_ns)?;
                    open.arrive(Dio::Edge {
                        tick,
                        levels: edge.levels,
                    })?;
                }
            }
            Message::DioLost(lost) => {
                let open = self.open.as_mut().unwrap();
                let from = open.stamp(lost.from_ns)?;
                let to = open.stamp(lost.to_ns)?;
                // The stream's rule: a report up to a pairs message's last pair comes before the
                // next pairs message, so none ends before what that message promised.
                if to < open.promised {
                    return Err(Error::Timestamps);
                }
                // A report that spans nothing lost nothing: the levels stand from then on.
                let item = match to.cmp(&from) {
                    std::cmp::Ordering::Less => return Err(Error::Timestamps),
                    std::cmp::Ordering::Equal => Dio::Edge {
                        tick: to,
                        levels: lost.levels,
                    },
                    std::cmp::Ordering::Greater => Dio::Lost {
                        from,
                        to,
                        levels: lost.levels,
                    },
                };
                open.arrive(item)?;
            }
            Message::Markers(markers) => self.open.as_mut().unwrap().markers(markers)?,
            Message::End(end) => {
                self.open.as_mut().unwrap().end(end)?;
                self.ended = true;
            }
        }
        Ok(self.ended)
    }

    /// The recording's bytes since the last call. Each call also compresses a slice of the
    /// blocks closed so far, which reach the output over the calls after their close.
    pub fn take_output(&mut self) -> Vec<u8> {
        if let Some(dio) = self.dio.as_mut() {
            return dio.writer.take_output();
        }
        self.open
            .as_mut()
            .map_or_else(Vec::new, |open| open.writer.take_output())
    }

    /// Whether closed blocks still wait to be compressed, so that [`Encoder::take_output`]
    /// has more to give.
    #[must_use]
    pub fn pending(&self) -> bool {
        self.open.as_ref().is_some_and(|open| open.writer.pending())
            || self.dio.as_ref().is_some_and(|dio| dio.writer.pending())
    }

    /// Closes every channel's open block at the latest promise, so a crash loses only what came
    /// after once the output is in the file. Nothing when nothing was promised since the last
    /// flush.
    pub fn flush(&mut self) -> Result<()> {
        if let Some(dio) = self.dio.as_mut()
            && dio.promised > dio.flushed
        {
            dio.writer.flush(dio.promised)?;
            dio.flushed = dio.promised;
        }
        if let Some(open) = self.open.as_mut()
            && open.promised > open.flushed
        {
            open.writer.flush(open.promised - open.base)?;
            open.flushed = open.promised;
        }
        Ok(())
    }

    /// What is written so far, for a view in this process. Publish it only once
    /// [`Encoder::take_output`]'s bytes are in the file.
    #[must_use]
    pub fn snapshot(&self) -> Option<Snapshot> {
        if let Some(dio) = self.dio.as_ref() {
            return Some(dio.writer.snapshot());
        }
        self.open.as_ref().map(|open| open.writer.snapshot())
    }

    /// Whether the header has come, so that there is a recording to finish.
    #[must_use]
    pub fn started(&self) -> bool {
        self.open.is_some() || self.dio.is_some()
    }

    /// Ends the recording, after the end message or without one, as when the stream stopped.
    /// `stem` names the file `<stem>.tickwave`, for the bundled markers. `None` without a header.
    pub fn finish(mut self, stem: &str) -> Result<Option<Finished>> {
        if let Some(dio) = self.dio.take() {
            return dio.finish(stem).map(Some);
        }
        let Some(mut open) = self.open.take() else {
            return Ok(None);
        };
        if !self.ended {
            open.close()?;
        }
        open.finish(stem).map(Some)
    }
}

/// A DIO stream's recording: a digital channel for each pin it reports, from the levels its header
/// gives. Ticks count from the header, as the stream's times do.
struct DioOpen {
    writer: Writer,
    clock_hz: u32,
    /// Each channel's pin, in channel order.
    pins: Vec<u8>,
    mask: u8,
    levels: u8,
    /// The latest tick placed: one past an edge or a gap's `to`, or a DIO time.
    end: i64,
    /// The end message's time, which reaches past the last edge.
    ended_at: Option<i64>,
    reason: Option<EndReason>,
    promised: i64,
    flushed: i64,
}

impl DioOpen {
    fn new(header: &DioHeader, context: &Context) -> Result<Self> {
        // The tick conversion needs a count longer than 2 ns.
        if header.clock_hz == 0 || header.clock_hz >= 500_000_000 {
            return Err(Error::Timestamps);
        }
        let pins: Vec<u8> = (0..4).filter(|k| header.dio_mask & (1 << k) != 0).collect();
        if pins.is_empty() {
            return Err(Error::NoPins);
        }
        let timebase = Ratio::integer(u64::from(header.clock_hz)).expect("a non-zero clock");
        let channels = pins
            .iter()
            .map(|&k| digital(&format!("DIO{k}"), header.dio_levels & (1 << k) != 0))
            .collect();
        let mut source = SourceHeader::new(SourceFormat::Other(DIO_FORMAT.into()), timebase);
        source.begin_s = Some(0.0);
        source.instrument = context.instrument.clone();
        source.software = context.software.clone();
        source.converter =
            Some(concat!("escapement-verge-probe ", env!("CARGO_PKG_VERSION")).into());
        let mut extra = BTreeMap::new();
        extra.insert("stream_format".into(), "5".into());
        extra.insert("clock_hz".into(), header.clock_hz.to_string());
        extra.insert("dio_mask".into(), header.dio_mask.to_string());
        extra.insert("dio_levels".into(), header.dio_levels.to_string());
        extra.insert(
            "edge_resolution_ns".into(),
            header.edge_resolution_ns.to_string(),
        );
        if let Some(started) = &context.started_utc {
            extra.insert("started_utc".into(), started.clone());
        }
        source.extra = extra;
        let declaration = Declaration {
            timebase,
            channels,
            derived: Vec::new(),
            sources: vec![source],
            title: None,
            meta: BTreeMap::new(),
            overview_shift: DIO_OVERVIEW_SHIFT,
            requires: Vec::new(),
        };
        Ok(Self {
            writer: Writer::new(declaration, write_options())?,
            clock_hz: header.clock_hz,
            pins,
            mask: header.dio_mask,
            levels: header.dio_levels & header.dio_mask,
            end: 0,
            ended_at: None,
            reason: None,
            promised: 0,
            flushed: 0,
        })
    }

    /// One message after the header. True at the end.
    fn ingest(&mut self, message: &Message) -> Result<bool> {
        match message {
            Message::Header(_) | Message::DioHeader(_) => return Err(Error::Restarted),
            Message::Pairs(_) => return Err(Error::DioPairs),
            // 0x94 records markers only beside energy capture, which a DIO stream's board lacks.
            Message::Markers(_) => {}
            Message::Edges(edges) => {
                let mut last = None;
                for edge in &edges.edges {
                    let tick = self.at(edge.time_ns)?;
                    self.edge(tick, edge.levels)?;
                    last = Some(tick);
                }
                if let Some(last) = last {
                    self.promise(last + 1)?;
                }
            }
            Message::DioLost(lost) => {
                let from = self.at(lost.from_ns)?;
                let to = self.at(lost.to_ns)?;
                match to.cmp(&from) {
                    std::cmp::Ordering::Less => return Err(Error::Timestamps),
                    // A report that spans nothing lost nothing: the levels stand from then on.
                    std::cmp::Ordering::Equal => self.edge(to, lost.levels)?,
                    std::cmp::Ordering::Greater => self.lost(from, to, lost.levels)?,
                }
                self.promise(to + 1)?;
            }
            // Every edge before it is in, so the pins held their levels until then.
            Message::DioTime(time) => {
                let tick = self.at(time.time_ns)?;
                if tick < self.promised {
                    return Err(Error::Timestamps);
                }
                self.end = self.end.max(tick);
                self.promise(tick)?;
            }
            Message::End(end) => {
                self.reason = Some(end.reason);
                self.ended_at = end.time_ns.map(|ns| self.at(ns)).transpose()?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `ns` in ticks, up to 2^53 − 2, so that one past it is still a length the writer takes.
    fn at(&self, ns: u64) -> Result<i64> {
        let ticks = (u128::from(ns) * u128::from(self.clock_hz)).div_ceil(1_000_000_000);
        i64::try_from(ticks)
            .ok()
            .filter(|&tick| tick <= (1 << 53) - 2)
            .ok_or(Error::Timestamps)
    }

    /// Every item before `tick` is in. Within a message the next edge can share this one's tick.
    fn promise(&mut self, tick: i64) -> Result<()> {
        if tick > self.promised {
            self.writer.promise(tick)?;
            self.promised = tick;
        }
        Ok(())
    }

    // The stream's rule: times never go back, edges stamped together go in one message, and what
    // comes after a message or a loss report comes after its last time. So each message promises
    // one tick past it, and a flush closes the block holding it however long the pins stay quiet.
    fn edge(&mut self, tick: i64, levels: u8) -> Result<()> {
        if tick < self.promised {
            return Err(Error::Timestamps);
        }
        let levels = levels & self.mask;
        for (i, &k) in self.pins.iter().enumerate() {
            if (levels ^ self.levels) & (1 << k) != 0 {
                self.writer.edge(i, tick)?;
            }
        }
        self.levels = levels;
        self.end = self.end.max(tick + 1);
        self.promise(tick)
    }

    fn lost(&mut self, from: i64, to: i64, levels: u8) -> Result<()> {
        if from < self.promised {
            return Err(Error::Timestamps);
        }
        let levels = levels & self.mask;
        for (i, &k) in self.pins.iter().enumerate() {
            let level = levels & (1 << k) != 0;
            self.writer.gap(i, Gap { from, to, level })?;
        }
        self.levels = levels;
        // The levels are known at `to`: a gap that reached the length would read as unknown to
        // the end instead.
        self.end = self.end.max(to + 1);
        Ok(())
    }

    fn finish(self, stem: &str) -> Result<Finished> {
        let end = [self.ended_at, Some(self.end), Some(1)]
            .into_iter()
            .flatten()
            .max()
            .expect("one candidate");
        let mut extra = BTreeMap::new();
        extra.insert("end_reason".into(), end_reason(self.reason));
        let finish = Finish {
            extra: vec![extra],
            bundle: Some(Bundle::Generated { stem: stem.into() }),
            drop_gaps_past_end: false,
            lost_after: BTreeMap::new(),
        };
        let ends = vec![end; self.pins.len()];
        Ok(self.writer.finish(end, &ends, finish)?)
    }
}

impl Open {
    fn new(header: &Header, context: &Context) -> Result<Self> {
        let rate_ok = header.period != 0
            && header.period <= header.clock_hz
            && u64::from(header.clock_hz) <= 100_000 * u64::from(header.period)
            // The tick conversion needs a count longer than 2 ns.
            && header.clock_hz < 500_000_000;
        if !rate_ok {
            return Err(Error::Rate(header.rate()));
        }
        let timebase = Ratio::integer(u64::from(header.clock_hz)).expect("a non-zero clock");
        let period = Ratio::integer(u64::from(header.period)).expect("a non-zero period");
        let pins: Vec<u8> = (0..4).filter(|k| header.dio_mask & (1 << k) != 0).collect();
        // A stream from the capture's first pair knows the pins from its start; one attached to a
        // running capture knows them only from when it started, so it places a gap first.
        let from_start = header.first_sequence <= 1;
        // A stream attached to a running capture starts at its request, after pair
        // first_sequence − 1, which sits at least that many periods minus one in: pauses and holds
        // only add time. Starting the recording there keeps the hours before out of it.
        let base = if from_start {
            0
        } else {
            i64::from(header.first_sequence - 2) * i64::from(header.period)
        };
        let mut channels = vec![
            analog("V", "V", period, int(1000)),
            analog("I", "A", period, int(1_000_000_000)),
            digital("low_range", false),
            digital("under_range", false),
        ];
        for &k in &pins {
            let initial = from_start && header.dio_levels & (1 << k) != 0;
            channels.push(digital(&format!("DIO{k}"), initial));
        }
        let mut source = SourceHeader::new(SourceFormat::Other(FORMAT.into()), timebase);
        source.begin_s = Some(0.0);
        source.origin = -base;
        source.instrument = context.instrument.clone();
        source.software = context.software.clone();
        source.converter =
            Some(concat!("escapement-verge-probe ", env!("CARGO_PKG_VERSION")).into());
        source.extra = header_extra(header, context);
        let declaration = Declaration {
            timebase,
            channels,
            derived: vec![Derived {
                significant: Some(4),
                ..Derived::multiply("P", "W", "I", "V")
            }],
            sources: vec![source],
            title: None,
            meta: BTreeMap::new(),
            overview_shift: overview_shift(period),
            requires: Vec::new(),
        };
        Ok(Self {
            writer: Writer::new(declaration, write_options())?,
            header: header.clone(),
            clock_hz: header.clock_hz,
            base,
            period: i64::from(header.period),
            converter: Converter::new(header),
            pins,
            next: header.first_sequence,
            first_pair: None,
            last_pair: None,
            runs: VecDeque::new(),
            early: Vec::new(),
            dio_arrived: None,
            last_sample: None,
            range: [None, None],
            dio_levels: from_start.then_some(header.dio_levels),
            dio_end: base,
            dio_start: None,
            markers: 0,
            marker_losses: 0,
            untimed_marker_losses: 0,
            marker_ticks: HashMap::new(),
            last_marker: None,
            lost_after: 0,
            reason: None,
            promised: base,
            flushed: base,
        })
    }

    fn pairs(&mut self, pairs: &Pairs) -> Result<()> {
        let Some(first) = pairs.pairs.first() else {
            return Ok(());
        };
        let start = self.at(pairs.time_ns)?;
        let missing = skipped(self.next, pairs.first_sequence);
        // Lost pairs take their periods, and pauses and holds add moments: a message can come no
        // earlier than its sequence puts it, and no more than `ahead` later.
        let due = self
            .last_pair
            .map_or(self.base, |last| last.saturating_add(self.period))
            .saturating_add(
                i64::try_from(missing)
                    .unwrap_or(i64::MAX)
                    .saturating_mul(self.period),
            );
        if start < due || start > due.saturating_add(self.ahead()) {
            return Err(Error::Timestamps);
        }
        let lost = match self.last_pair {
            // The pairs lost before the first lie between the start and it, as `due` holds.
            None => Some(missing),
            Some(_) if missing > 0 => Some(missing),
            Some(_) if pairs.paused() || pairs.held() => Some(0),
            Some(last) if start != last + self.period => Some(0),
            Some(_) => None,
        };
        if let Some(lost) = lost {
            self.runs.push_back(RunStart {
                sequence: pairs.first_sequence,
                tick: start,
                lost,
            });
        }
        if missing > 0 {
            // The store's rule: a pair waiting at a switch keeps its own current.
            self.converter.discontinuity(first.low);
        }
        // Every edge and loss report up to the last message's last pair has come by now, and a
        // pair the converter still holds lies at that pair.
        if let Some(previous) = self.last_pair {
            self.promise(previous)?;
        }
        self.converter.add_message(pairs);
        self.next = pairs.first_sequence.wrapping_add(pairs.pairs.len() as u32);
        self.last_pair = Some(start + (pairs.pairs.len() as i64 - 1) * self.period);
        if self.first_pair.is_none() {
            self.first_pair = Some(start);
            if !self.early.is_empty() {
                self.start_dio(None)?;
            }
        }
        self.take_samples()
    }

    /// Every item before the capture's tick `before` is in.
    fn promise(&mut self, before: i64) -> Result<()> {
        // With no DIO item yet, an attached stream's first gap goes in before the first promise
        // at or past its first pair, and ends before the promise: every later edge lies at or
        // after it, and every later loss report ends there or later.
        if self.dio_levels.is_none() && self.first_pair.is_some_and(|first| before >= first) {
            self.start_dio(Some(before))?;
        }
        if before > self.promised {
            self.writer.promise(before - self.base)?;
            self.promised = before;
        }
        Ok(())
    }

    /// The probe's time `ns` as the capture's tick. Nothing the stream stamps comes before the
    /// recording's start, so a time that would is refused rather than moved.
    fn at(&self, ns: u64) -> Result<i64> {
        let ticks = (u128::from(ns) * u128::from(self.clock_hz)).div_ceil(1_000_000_000);
        let at = i64::try_from(ticks).map_err(|_| Error::Timestamps)?;
        if at < self.base {
            return Err(Error::Timestamps);
        }
        Ok(at)
    }

    /// How far past what its pairs account for a stream may stamp anything: its pauses and holds
    /// are moments, so further is a corrupt stamp, and placing one would stretch the recording to
    /// it, with every bucket of overview in between.
    fn ahead(&self) -> i64 {
        i64::from(AHEAD_S) * i64::from(self.clock_hz)
    }

    /// An edge's, loss report's or marker's stamp as `at` gives it, refused past `ahead` of the
    /// newest pair, or of the start before any.
    fn stamp(&self, ns: u64) -> Result<i64> {
        let at = self.at(ns)?;
        if at
            > self
                .last_pair
                .unwrap_or(self.base)
                .saturating_add(self.ahead())
        {
            return Err(Error::Timestamps);
        }
        Ok(at)
    }

    fn take_samples(&mut self) -> Result<()> {
        for sample in self.converter.take_samples() {
            let at = self.at(sample.time_ns)?;
            self.place_sample(at, &sample)?;
        }
        Ok(())
    }

    fn arrive(&mut self, item: Dio) -> Result<()> {
        // The stream's rule: each report starts past what came before it, and every edge after a
        // report comes after its end.
        let after = match item {
            Dio::Edge { tick, .. } => self.dio_arrived.is_none_or(|last| tick >= last),
            Dio::Lost { from, .. } => self.dio_arrived.is_none_or(|last| from > last),
        };
        if !after {
            return Err(Error::Timestamps);
        }
        self.dio_arrived = Some(match item {
            Dio::Edge { tick, .. } => tick,
            Dio::Lost { to, .. } => to,
        });
        let mask = self.header.dio_mask;
        let item = match item {
            Dio::Edge { tick, levels } => Dio::Edge {
                tick,
                levels: levels & mask,
            },
            Dio::Lost { from, to, levels } => Dio::Lost {
                from,
                to,
                levels: levels & mask,
            },
        };
        if self.dio_levels.is_none() {
            if self.first_pair.is_none() {
                self.early.push(item);
                return Ok(());
            }
            self.start_dio(Some(item.tick()))?;
        }
        self.place_dio(item)
    }

    fn markers(&mut self, markers: &Markers) -> Result<()> {
        if markers.lost() {
            self.marker_losses += 1;
            if markers.markers.is_empty() {
                self.untimed_marker_losses += 1;
            }
        }
        for (index, marker) in markers.markers.iter().enumerate() {
            let tick = self.stamp(marker.time_ns)?;
            let at = tick - self.base;
            let seen = self.marker_ticks.entry(at).or_default();
            let id = match *seen {
                0 => format!("marker/{at}"),
                n => format!("marker/{at}.{n}"),
            };
            *seen += 1;
            let kind = if index == 0 && markers.lost() {
                "marker-after-loss"
            } else {
                "marker"
            };
            let mut annotation = serde_json::Map::new();
            annotation.insert("id".into(), id.into());
            annotation.insert("kind".into(), kind.into());
            annotation.insert("start".into(), at.into());
            annotation.insert(
                "label".into(),
                crate::record::marker_label(&marker.label).into(),
            );
            let mut values = serde_json::Map::new();
            values.insert("number".into(), marker.number.to_string().into());
            annotation.insert("values".into(), values.into());
            self.writer.annotation(GENERATOR, annotation)?;
            self.markers += 1;
            self.last_marker = Some(self.last_marker.map_or(tick, |last| last.max(tick)));
        }
        Ok(())
    }

    /// The stream's end message: the pairs the capture took after the last that came, which
    /// lie in the periods after it.
    fn end(&mut self, end: &End) -> Result<()> {
        let lost = skipped(self.next, end.last_sequence.wrapping_add(1));
        if let Some(last) = self.last_pair {
            let reach =
                i128::from(last - self.base) + i128::from(self.period) * (1 + i128::from(lost));
            if reach >= i128::from(JSON_SAFE) {
                return Err(Error::LostAfter(lost));
            }
        }
        self.lost_after = lost;
        self.reason = Some(end.reason);
        self.close()
    }

    /// The stream ended or stopped: a pair the converter still holds goes in, and an attached
    /// stream's pins their first gap, if anything came.
    fn close(&mut self) -> Result<()> {
        self.converter.end();
        self.take_samples()?;
        if self.dio_levels.is_none() && (self.first_pair.is_some() || !self.early.is_empty()) {
            self.start_dio(None)?;
        }
        Ok(())
    }

    /// An attached stream's pins are unknown from the recording's start until the stream started:
    /// before its first pair, and before its first edge or loss report, the first early one's or
    /// `next`'s. The early ones go in after.
    fn start_dio(&mut self, next: Option<i64>) -> Result<()> {
        let levels = self.header.dio_levels;
        self.dio_levels = Some(levels);
        let first_dio = self.early.first().map(|d| d.tick()).or(next);
        let to = [self.first_pair, first_dio.map(|t| t - 1)]
            .into_iter()
            .flatten()
            .min();
        for (i, &k) in self.pins.iter().enumerate() {
            let level = levels & (1 << k) != 0;
            let from = self.base;
            match to {
                Some(to) if to > from => self.writer.gap(FIRST_DIO + i, Gap { from, to, level })?,
                // With no room for a gap (the first item at the start or a tick after it), the
                // pins read as known from the start, at most a tick early.
                _ if level => self.writer.edge(FIRST_DIO + i, from)?,
                _ => {}
            }
        }
        if let Some(to) = to {
            self.dio_end = self.dio_end.max(to);
        }
        self.dio_start = Some(to.map_or(self.base, |to| to.max(self.base)));
        for item in std::mem::take(&mut self.early) {
            self.place_dio(item)?;
        }
        Ok(())
    }

    fn place_sample(&mut self, tick: i64, sample: &Sample) -> Result<()> {
        let mut lost_from = None;
        if let Some(run) = self
            .runs
            .pop_front_if(|run| run.sequence == sample.sequence)
        {
            debug_assert_eq!(run.tick, tick);
            if run.lost > 0
                && let Some(last) = self.last_sample
                && tick > last + self.period
            {
                lost_from = Some(last + self.period);
            }
            self.writer.run(V, tick, run.lost)?;
            self.writer.run(I, tick, run.lost)?;
        }
        self.writer.sample(V, u64::from(sample.millivolts))?;
        self.writer.sample(I, u64::from(sample.nanoamps))?;
        let flags = [sample.range == Some(CurrentRange::Low), sample.under_range];
        for (k, flag) in flags.into_iter().enumerate() {
            let c = LOW_RANGE + k;
            match self.range[k] {
                None if tick == self.base => {
                    if flag {
                        self.writer.edge(c, tick)?;
                    }
                }
                None => self.writer.gap(
                    c,
                    Gap {
                        from: self.base,
                        to: tick,
                        level: flag,
                    },
                )?,
                Some(level) => match lost_from {
                    // The range wasn't measured while pairs were lost.
                    Some(from) => self.writer.gap(
                        c,
                        Gap {
                            from,
                            to: tick,
                            level: flag,
                        },
                    )?,
                    None if level != flag => self.writer.edge(c, tick)?,
                    None => {}
                },
            }
            self.range[k] = Some(flag);
        }
        self.last_sample = Some(tick);
        Ok(())
    }

    fn place_dio(&mut self, item: Dio) -> Result<()> {
        let before = self.dio_levels.expect("the first gap is placed first");
        match item {
            Dio::Edge { tick, levels } => {
                for (i, &k) in self.pins.iter().enumerate() {
                    if (levels ^ before) & (1 << k) != 0 {
                        self.writer.edge(FIRST_DIO + i, tick)?;
                    }
                }
                self.dio_levels = Some(levels);
                self.dio_end = self.dio_end.max(tick + 1);
            }
            Dio::Lost { from, to, levels } => {
                let from = self.dio_start.map_or(from, |known| from.max(known + 1));
                if to <= from {
                    return self.place_dio(Dio::Edge { tick: from, levels });
                }
                for (i, &k) in self.pins.iter().enumerate() {
                    let gap = Gap {
                        from,
                        to,
                        level: levels & (1 << k) != 0,
                    };
                    self.writer.gap(FIRST_DIO + i, gap)?;
                }
                self.dio_levels = Some(levels);
                // The levels are known at `to`: a gap that reached the length would read as
                // unknown to the end instead.
                self.dio_end = self.dio_end.max(to + 1);
            }
        }
        Ok(())
    }

    fn finish(mut self, stem: &str) -> Result<Finished> {
        // The pairs lost after the last take the periods after its own.
        let lost_after = i64::try_from(self.lost_after).expect("below 2^31, as skipped gives");
        let end = [
            self.last_sample.map(|t| t + self.period * (1 + lost_after)),
            self.last_marker.map(|t| t + 1),
            Some(self.dio_end),
            Some(self.base + 1),
        ]
        .into_iter()
        .flatten()
        .max()
        .expect("one candidate");
        // A channel never known reads as unknown throughout: a gap from its start to the end.
        let never = Gap {
            from: self.base,
            to: end,
            level: false,
        };
        for k in 0..2 {
            if self.range[k].is_none() {
                self.writer.gap(LOW_RANGE + k, never)?;
            }
        }
        if self.dio_levels.is_none() {
            for i in 0..self.pins.len() {
                self.writer.gap(FIRST_DIO + i, never)?;
            }
        }
        // The range is measured with each pair, so it is known to the last one's end, before the
        // pairs lost after it.
        let range_end = self.last_sample.map_or(end, |t| t + self.period);
        let mut ends = vec![range_end; 2];
        ends.resize(2 + self.pins.len(), end);
        let mut extra = BTreeMap::new();
        extra.insert("end_reason".into(), end_reason(self.reason));
        extra.insert("markers".into(), self.markers.to_string());
        extra.insert("marker_losses".into(), self.marker_losses.to_string());
        extra.insert(
            "untimed_marker_losses".into(),
            self.untimed_marker_losses.to_string(),
        );
        // A stream that brought no pair has no sample for its losses to lie beside.
        let lost_after = if self.last_sample.is_some() && self.lost_after > 0 {
            BTreeMap::from([(V, self.lost_after), (I, self.lost_after)])
        } else {
            BTreeMap::new()
        };
        let finish = Finish {
            extra: vec![extra],
            bundle: Some(Bundle::Generated { stem: stem.into() }),
            drop_gaps_past_end: false,
            lost_after,
        };
        Ok(self.writer.finish(end - self.base, &ends, finish)?)
    }
}

/// How much of a file compaction and recovery read at a time.
const PIECE: usize = 1 << 20;

fn pieces(mut input: impl Read, mut feed: impl FnMut(&[u8]) -> Result<()>) -> Result<()> {
    let mut buf = vec![0; PIECE];
    loop {
        let n = input.read(&mut buf)?;
        if n == 0 {
            return Ok(());
        }
        feed(&buf[..n])?;
    }
}

/// How much of a file's end [`finished_header`] reads first: a footer is a few kilobytes a
/// channel.
const TAIL: u64 = 1 << 18;
/// The most it reads, for a footer that says it starts further back.
const TAIL_MAX: u64 = 1 << 24;

/// The header of the finished recording at `path`, from its footer; none for one a crash cut
/// off, or one whose footer doesn't read.
fn finished_header(path: &Path) -> Result<Option<escapement_tickwave_format::header::Header>> {
    let mut file = File::open(path)?;
    let len = file.metadata()?.len();
    let mut from = len.saturating_sub(TAIL);
    loop {
        file.seek(SeekFrom::Start(from))?;
        let mut tail = Vec::new();
        Read::take(&mut file, len - from).read_to_end(&mut tail)?;
        match Footer::parse(&tail, len) {
            Ok(footer) => return Ok(Some(footer.header)),
            Err(DecodeError::NeedFrom(at)) if at < from && len - at <= TAIL_MAX => from = at,
            Err(_) => return Ok(None),
        }
    }
}

fn stem(path: &Path) -> String {
    path.file_stem()
        .map_or_else(|| "capture".into(), |s| s.to_string_lossy().into_owned())
}

/// Writes the recording at `path` again with full-size blocks, as a recorder's one-second blocks
/// are not, into a copy beside it, then renames the copy over it. `path` stays readable until
/// then. A finished recording keeps its content id, which the Recutter checks, and each channel
/// gets the overview its own rate calls for. One a crash cut off is finished by repair's rules,
/// with its markers bundled and the overview it declared.
pub fn compact(path: &Path) -> Result<()> {
    let header = finished_header(path)?;
    let temp = temp_beside(path)?;
    let mut out = BufWriter::new(temp.as_file());
    let mut recutter = Recutter::new(write_options());
    if let Some(header) = &header {
        recutter.shifts(shifts_for(header));
    }
    recutter.bundle(Bundle::Generated { stem: stem(path) });
    pieces(File::open(path)?, |bytes| {
        recutter.feed(bytes)?;
        out.write_all(recutter.output())?;
        let n = recutter.output().len();
        recutter.consume(n);
        Ok(())
    })?;
    let finished = recutter.finish()?;
    out.write_all(&finished.tail)?;
    out.flush()?;
    drop(out);
    temp.as_file().sync_all()?;
    temp.persist(path).map_err(|error| error.error)?;
    Ok(())
}

/// A recording written to disk as its stream arrives: blocks closed at least every second, a
/// snapshot after each batch for a view in this process, compacted once it ends.
pub struct FileWriter {
    path: PathBuf,
    file: File,
    syncer: Syncer,
    encoder: Encoder,
    flushed: Instant,
    flush_every: Duration,
    /// A flush whose bytes are not all in the file yet, to sync once they are.
    unsynced: bool,
    published: Option<Instant>,
    latest: Arc<Mutex<Option<Snapshot>>>,
}

/// Syncs the file to the disk on a thread of its own, so a slow disk holds up no message.
struct Syncer {
    wake: Option<SyncSender<()>>,
    thread: Option<JoinHandle<std::io::Result<()>>>,
}

impl Syncer {
    fn new(file: &File) -> Result<Self> {
        let file = file.try_clone()?;
        let (wake, woken) = mpsc::sync_channel(1);
        let thread = thread::Builder::new()
            .name("capture-sync".into())
            .spawn(move || {
                while woken.recv().is_ok() {
                    file.sync_data()?;
                }
                Ok(())
            })?;
        Ok(Self {
            wake: Some(wake),
            thread: Some(thread),
        })
    }

    /// Asks for what is written so far to reach the disk. A sync already asked for and not yet
    /// begun covers it. A sync that failed is this call's error.
    fn ask(&mut self) -> Result<()> {
        match self.wake.as_ref().map(|wake| wake.try_send(())) {
            Some(Err(TrySendError::Disconnected(()))) => self.stop(),
            _ => Ok(()),
        }
    }

    /// Ends the thread after the sync it is in, if any, closing its handle on the file.
    fn stop(&mut self) -> Result<()> {
        self.wake.take();
        match self.thread.take() {
            Some(thread) => Ok(thread
                .join()
                .map_err(|_| std::io::Error::other("the sync thread panicked"))??),
            None => Ok(()),
        }
    }
}

/// How far a stream may stamp past what its pairs account for, in seconds (`Open::ahead`).
const AHEAD_S: u32 = 60;

/// How often a crash can lose what came in, at most.
const FLUSH_EVERY: Duration = Duration::from_secs(1);
/// How often a view gets a new snapshot, at most.
const PUBLISH_EVERY: Duration = Duration::from_millis(200);

impl FileWriter {
    pub fn create(path: &Path, context: Context) -> Result<Self> {
        let file = File::options()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|error| match error.kind() {
                std::io::ErrorKind::AlreadyExists => Error::Exists(path.to_owned()),
                _ => error.into(),
            })?;
        Ok(Self {
            path: path.to_owned(),
            syncer: Syncer::new(&file)?,
            file,
            encoder: Encoder::new(context),
            flushed: Instant::now(),
            flush_every: FLUSH_EVERY,
            unsynced: false,
            published: None,
            latest: Arc::default(),
        })
    }

    /// Blocks closed after every message, for tests of many-block files.
    #[cfg(all(test, feature = "capture"))]
    fn flushing_always(mut self) -> Self {
        self.flush_every = Duration::ZERO;
        self
    }

    /// The latest snapshot of what is in the file, for a view in this process.
    #[must_use]
    pub fn snapshots(&self) -> Arc<Mutex<Option<Snapshot>>> {
        Arc::clone(&self.latest)
    }

    /// One message. True once the stream has ended.
    pub fn ingest(&mut self, message: &Message) -> Result<bool> {
        let ended = self.encoder.ingest(message)?;
        self.write()?;
        Ok(ended)
    }

    /// The recorder's clock, when no message came for a while. No message comes to compress a
    /// stalled stream's last blocks a slice at a time, so this compresses all of them.
    pub fn flush(&mut self) -> Result<()> {
        self.write()?;
        while self.encoder.pending() {
            self.take_output()?;
        }
        Ok(())
    }

    fn write(&mut self) -> Result<()> {
        if self.flushed.elapsed() >= self.flush_every {
            self.encoder.flush()?;
            self.flushed = Instant::now();
            self.unsynced = true;
        }
        self.take_output()?;
        if self
            .published
            .is_none_or(|at| at.elapsed() >= PUBLISH_EVERY)
            && let Some(snapshot) = self.encoder.snapshot()
        {
            *self.latest.lock().unwrap_or_else(|e| e.into_inner()) = Some(snapshot);
            self.published = Some(Instant::now());
        }
        Ok(())
    }

    fn take_output(&mut self) -> Result<()> {
        let bytes = self.encoder.take_output();
        if !bytes.is_empty() {
            self.file.write_all(&bytes)?;
        }
        // A power cut then loses no more than a crash: what came after the last flush.
        if self.unsynced && !self.encoder.pending() {
            self.syncer.ask()?;
            self.unsynced = false;
        }
        Ok(())
    }

    /// Ends the recording and compacts it. A stream that never sent its header leaves no file.
    pub fn finish(mut self) -> Result<()> {
        // Compacting renames a copy over the file, which Windows refuses while a handle is open.
        // A sync that failed is reported once the recording is finished as far as it can be.
        let synced = self.syncer.stop();
        let finished = self.finish_file();
        synced.and(finished)
    }

    fn finish_file(mut self) -> Result<()> {
        let bytes = self.encoder.take_output();
        self.file.write_all(&bytes)?;
        let encoder = std::mem::replace(&mut self.encoder, Encoder::new(Context::default()));
        match encoder.finish(&stem(&self.path))? {
            Some(finished) => {
                self.file.write_all(&finished.tail)?;
                self.file.sync_all()?;
                drop(self.file);
                compact(&self.path)
            }
            None => {
                drop(self.file);
                Ok(std::fs::remove_file(&self.path)?)
            }
        }
    }
}
