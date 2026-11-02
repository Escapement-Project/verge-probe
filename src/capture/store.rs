use super::{CaptureError, Interval, Result, Statistics};
use crate::record::{Sink, temp_beside};
use crate::stream::{Converter, Header, Message, Sample};
use rusqlite::{Connection, OpenFlags, OptionalExtension, params};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

const BLOCK: usize = 1024;
/// The most bins a view holds, however many pixels ask.
pub(super) const BINS: usize = 8192;
pub(super) const FAN: u64 = 16;
const APPLICATION: i64 = 0x56475245;
// Bump user_version when changing the postcard layouts stored in this archive.

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Node {
    interval: Interval,
    stats: Statistics,
}

#[derive(Clone, Debug)]
pub struct Envelope {
    pub interval: Interval,
    pub statistics: Statistics,
    /// Some part of this bin was not measured. Render it without joining its neighbours.
    pub gap: bool,
}

#[derive(Clone, Debug)]
pub struct View {
    pub extent: Interval,
    pub interval: Interval,
    pub statistics: Statistics,
    pub envelopes: Vec<Envelope>,
    pub lost_pairs: u64,
    pub ended: bool,
    /// Every sample the interval overlaps, when there are no more of them than bins.
    pub samples: Option<Vec<Supported>>,
}

/// A sample and the end of the time it covers: the next sample's time, or one period on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Supported {
    pub sample: Sample,
    pub end_ns: u64,
}

struct Bins<'a> {
    interval: Interval,
    values: Vec<Statistics>,
    cancel: &'a AtomicBool,
}
impl Bins<'_> {
    fn bin(&self, time: u64) -> usize {
        let offset = u128::from(time.saturating_sub(self.interval.start_ns));
        let scaled = (offset + 1) * self.values.len() as u128 - 1;
        (scaled / u128::from(self.interval.duration_ns().max(1)))
            .min((self.values.len() - 1) as u128) as usize
    }
}
pub(super) fn bin_interval(interval: Interval, pixels: usize, index: usize) -> Interval {
    let at = |n| {
        interval.start_ns + (u128::from(interval.duration_ns()) * n as u128 / pixels as u128) as u64
    };
    Interval::new(at(index), at(index + 1))
}

pub struct Archive {
    pub(super) db: Connection,
    pub(super) version: i64,
}

/// Fails with `error` unless `condition` holds.
fn ensure(condition: bool, error: CaptureError) -> Result<()> {
    if condition { Ok(()) } else { Err(error) }
}

pub(super) fn cancelled(cancel: &AtomicBool) -> Result<()> {
    ensure(!cancel.load(Ordering::Relaxed), CaptureError::Cancelled)
}

/// Persists `temp` at `path`, never over a file already there.
fn persist(temp: tempfile::NamedTempFile, path: &Path) -> Result<()> {
    temp.persist_noclobber(path).map_err(|error| error.error)?;
    Ok(())
}

fn configure(db: &Connection) -> Result<()> {
    db.busy_timeout(std::time::Duration::from_secs(2))?;
    db.execute_batch("PRAGMA cache_size=-4096; PRAGMA mmap_size=0;")?;
    Ok(())
}

pub(super) fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>> {
    Ok(postcard::to_stdvec(value)?)
}
pub(super) fn decode<T: for<'a> Deserialize<'a>>(bytes: &[u8]) -> Result<T> {
    Ok(postcard::from_bytes(bytes)?)
}

/// The stream header as archives before version 5 stored it, without step 5's fields.
#[derive(Serialize, Deserialize)]
struct HeaderV4 {
    rate: u32,
    setting_ma: Option<u16>,
    full_scale_mv: u16,
    first_sequence: u32,
    previous_low: bool,
    low: Vec<crate::CalibrationPoint>,
    high: Vec<crate::CalibrationPoint>,
    dio_mask: u8,
    dio_levels: u8,
    edge_resolution_ns: u16,
}

/// A block's sample as archives before version 4 stored it, without its range.
#[derive(Serialize, Deserialize)]
struct SupportedV3 {
    sample: SampleV3,
    end_ns: u64,
}
#[derive(Serialize, Deserialize)]
struct SampleV3 {
    sequence: u32,
    time_ns: u64,
    millivolts: u32,
    nanoamps: u32,
}

impl Archive {
    pub fn open(path: &Path) -> Result<Self> {
        let db = Connection::open_with_flags(path, OpenFlags::SQLITE_OPEN_READ_ONLY)?;
        configure(&db)?;
        ensure(
            db.query_row("PRAGMA application_id", [], |r| r.get::<_, i64>(0))? == APPLICATION,
            CaptureError::NotCapture,
        )?;
        let version = db.query_row("PRAGMA user_version", [], |r| r.get::<_, i64>(0))?;
        ensure((1..=5).contains(&version), CaptureError::Version(version))?;
        Ok(Self { db, version })
    }
    pub(super) fn meta<T: for<'a> Deserialize<'a>>(&self, key: &str) -> Result<T> {
        let bytes: Vec<u8> =
            self.db
                .query_row("SELECT value FROM metadata WHERE key=?", [key], |r| {
                    r.get(0)
                })?;
        decode(&bytes)
    }
    pub fn header(&self) -> Result<Header> {
        let mut header: Header = if self.version >= 5 {
            self.meta("header")?
        } else {
            let old: HeaderV4 = self.meta("header")?;
            Header {
                capture_id: None,
                rate: old.rate,
                setting_ma: old.setting_ma,
                full_scale_mv: old.full_scale_mv,
                first_sequence: old.first_sequence,
                previous_low: old.previous_low,
                low: old.low,
                high: old.high,
                dio_mask: old.dio_mask,
                dio_levels: old.dio_levels,
                edge_resolution_ns: old.edge_resolution_ns,
                switch_weight_q16: 0,
                rearm_between_pairs: false,
                previous_shunt_nanoamps: None,
            }
        };
        let id: Option<Vec<u8>> = self
            .db
            .query_row(
                "SELECT value FROM metadata WHERE key='capture_id'",
                [],
                |row| row.get(0),
            )
            .optional()?;
        header.capture_id = id.map(|bytes| decode(&bytes)).transpose()?;
        Ok(header)
    }
    pub fn extent(&self) -> Result<Interval> {
        let mut extent: Interval = self.meta("extent")?;
        if self.version >= 3 {
            let first: Option<i64> = self.db.query_row("SELECT time_ns FROM markers WHERE time_ns IS NOT NULL ORDER BY time_ns,n LIMIT 1", [], |r| r.get(0)).optional()?;
            let last: Option<i64> = self.db.query_row("SELECT time_ns FROM markers WHERE time_ns IS NOT NULL ORDER BY time_ns DESC,n DESC LIMIT 1", [], |r| r.get(0)).optional()?;
            if let (Some(first), Some(last)) = (first, last) {
                extent.start_ns = if extent.duration_ns() == 0 {
                    first as u64
                } else {
                    extent.start_ns.min(first as u64)
                };
                extent.end_ns = extent.end_ns.max((last as u64).saturating_add(1));
            }
        }
        Ok(extent)
    }
    fn node(&self, level: u32, index: u64) -> Result<Option<Node>> {
        let bytes: Option<Vec<u8>> = self
            .db
            .query_row(
                "SELECT data FROM nodes WHERE level=? AND n=?",
                params![level, index as i64],
                |r| r.get(0),
            )
            .optional()?;
        bytes.map(|b| decode(&b)).transpose()
    }
    fn block(&self, index: u64) -> Result<Vec<Supported>> {
        let bytes: Vec<u8> =
            self.db
                .query_row("SELECT data FROM blocks WHERE n=?", [index as i64], |r| {
                    r.get(0)
                })?;
        self.decode_block(&bytes)
    }
    pub(super) fn decode_block(&self, bytes: &[u8]) -> Result<Vec<Supported>> {
        if self.version >= 4 {
            return decode(bytes);
        }
        let old: Vec<SupportedV3> = decode(bytes)?;
        Ok(old
            .into_iter()
            .map(|s| Supported {
                sample: Sample {
                    sequence: s.sample.sequence,
                    time_ns: s.sample.time_ns,
                    millivolts: s.sample.millivolts,
                    nanoamps: s.sample.nanoamps,
                    range: None,
                },
                end_ns: s.end_ns,
            })
            .collect())
    }
    fn visit(
        &self,
        level: u32,
        index: u64,
        interval: Interval,
        stats: &mut Statistics,
        cancel: &AtomicBool,
    ) -> Result<()> {
        cancelled(cancel)?;
        let Some(node) = self.node(level, index)? else {
            return Ok(());
        };
        if node.interval.end_ns <= interval.start_ns || node.interval.start_ns >= interval.end_ns {
            return Ok(());
        }
        if interval.start_ns <= node.interval.start_ns && interval.end_ns >= node.interval.end_ns {
            stats.merge(&node.stats);
        } else if level > 0 {
            for child in index * FAN..(index + 1) * FAN {
                self.visit(level - 1, child, interval, stats, cancel)?;
            }
        } else {
            let samples = self.block(index)?;
            for sample in &samples {
                stats.add(sample.sample, sample.end_ns, interval);
            }
        }
        Ok(())
    }
    fn statistics_inner(&self, interval: Interval, cancel: &AtomicBool) -> Result<Statistics> {
        let blocks: u64 = self.meta("blocks")?;
        let mut level = 0;
        let mut covered = 1u64;
        while covered < blocks {
            covered = covered.saturating_mul(FAN);
            level += 1;
        }
        let mut stats = Statistics::default();
        self.visit(level, 0, interval, &mut stats, cancel)?;
        Ok(stats)
    }
    pub fn statistics(&self, interval: Interval, cancel: &AtomicBool) -> Result<Statistics> {
        self.db.execute_batch("BEGIN")?;
        let result = self.statistics_inner(interval, cancel);
        self.db.execute_batch("ROLLBACK")?;
        result
    }
    /// A stable snapshot, with at most `pixels` extrema bins regardless of capture length, and
    /// the samples themselves once they are no more than the bins.
    pub fn view(&self, interval: Interval, pixels: usize, cancel: &AtomicBool) -> Result<View> {
        self.db.execute_batch("BEGIN")?;
        let result = (|| {
            let extent = self.extent()?;
            let statistics = self.statistics_inner(interval, cancel)?;
            let pixels = pixels
                .clamp(1, BINS)
                .min(interval.duration_ns().clamp(1, BINS as u64) as usize);
            let blocks: u64 = self.meta("blocks")?;
            let mut level = 0;
            let mut covered = 1u64;
            while covered < blocks {
                covered = covered.saturating_mul(FAN);
                level += 1;
            }
            let mut query = Bins {
                interval,
                values: vec![Statistics::default(); pixels],
                cancel,
            };
            self.envelopes(level, 0, &mut query)?;
            let envelopes = query
                .values
                .into_iter()
                .enumerate()
                .filter_map(|(i, statistics)| {
                    if statistics.samples == 0 {
                        return None;
                    }
                    let bin = bin_interval(interval, pixels, i);
                    Some(Envelope {
                        interval: bin,
                        gap: statistics.measured_ns < bin.duration_ns(),
                        statistics,
                    })
                })
                .collect();
            let samples = if statistics.samples <= pixels as u64 {
                let mut samples = Vec::new();
                self.overlapping(interval, cancel, |block| {
                    samples.extend(block);
                    Ok(())
                })?;
                Some(samples)
            } else {
                None
            };
            Ok(View {
                extent,
                interval,
                statistics,
                envelopes,
                lost_pairs: self.meta("lost")?,
                ended: self.meta("ended")?,
                samples,
            })
        })();
        self.db.execute_batch("ROLLBACK")?;
        result
    }
    fn envelopes(&self, level: u32, index: u64, query: &mut Bins<'_>) -> Result<()> {
        cancelled(query.cancel)?;
        let Some(node) = self.node(level, index)? else {
            return Ok(());
        };
        let interval = query.interval;
        if interval.duration_ns() == 0
            || node.interval.end_ns <= interval.start_ns
            || node.interval.start_ns >= interval.end_ns
        {
            return Ok(());
        }
        let start = query.bin(node.interval.start_ns.max(interval.start_ns));
        let end = query.bin(node.interval.end_ns.min(interval.end_ns).saturating_sub(1));
        if start == end
            && node.interval.start_ns >= interval.start_ns
            && node.interval.end_ns <= interval.end_ns
        {
            query.values[start].merge(&node.stats);
        } else if level > 0 {
            for child in index * FAN..(index + 1) * FAN {
                self.envelopes(level - 1, child, query)?;
            }
        } else {
            let samples = self.block(index)?;
            for supported in samples {
                let sample = supported.sample;
                let stop = supported.end_ns;
                if stop <= interval.start_ns || sample.time_ns >= interval.end_ns {
                    continue;
                }
                let first = query.bin(sample.time_ns.max(interval.start_ns));
                let last = query.bin(stop.min(interval.end_ns).saturating_sub(1));
                let pixels = query.values.len();
                for bin in first..=last {
                    query.values[bin].add(sample, stop, bin_interval(interval, pixels, bin));
                }
            }
        }
        Ok(())
    }
    /// Each stored block's samples whose support overlaps `interval`, in order.
    fn overlapping(
        &self,
        interval: Interval,
        cancel: &AtomicBool,
        mut each: impl FnMut(Vec<Supported>) -> Result<()>,
    ) -> Result<()> {
        let mut statement = self
            .db
            .prepare("SELECT data FROM blocks WHERE end_ns>? AND start_ns<? ORDER BY n")?;
        let mut rows = statement.query(params![
            interval.start_ns.min(i64::MAX as u64) as i64,
            interval.end_ns.min(i64::MAX as u64) as i64
        ])?;
        while let Some(row) = rows.next()? {
            cancelled(cancel)?;
            let samples = self.decode_block(&row.get::<_, Vec<u8>>(0)?)?;
            each(
                samples
                    .into_iter()
                    .filter(|s| s.end_ns > interval.start_ns && s.sample.time_ns < interval.end_ns)
                    .collect(),
            )?;
        }
        Ok(())
    }
    /// SQLite copies a consistent snapshot, including committed WAL records, to a new file.
    pub fn save(&self, path: &Path) -> Result<()> {
        ensure(!path.exists(), CaptureError::Exists(path.to_owned()))?;
        let temp = temp_beside(path)?;
        let into = temp
            .path()
            .to_str()
            .ok_or_else(|| CaptureError::Path(temp.path().to_owned()))?;
        self.db.execute("VACUUM INTO ?", [into])?;
        persist(temp, path)
    }
    /// Export whole samples whose measured support overlaps the interval. Statistics clip
    /// boundary support; exports retain original timestamps and complete boundary samples.
    pub fn export(
        &self,
        path: &Path,
        csv: bool,
        interval: Interval,
        cancel: &AtomicBool,
    ) -> Result<()> {
        ensure(!path.exists(), CaptureError::Exists(path.to_owned()))?;
        self.db.execute_batch("BEGIN")?;
        let result = (|| {
            let header = self.header()?;
            // Only the sink holds the file open, so a CSV marker merge can rename over it.
            let (file, temp) = temp_beside(path)?.into_parts();
            let mut sink = Sink::from_file(file, &temp, csv)?;
            let pins = if self.version >= 2 {
                header.dio_mask
            } else {
                0
            };
            if self.version >= 3
                && self.meta::<u64>("marker_count")? + self.meta::<u64>("marker_losses")? > 0
            {
                sink.include_markers()?;
            }
            sink.begin_with_dio_gaps(header.rate, pins, header.dio_levels)?;
            let extent = self.extent()?;
            self.export_markers(
                &mut sink,
                Interval::new(
                    interval.start_ns.max(extent.start_ns),
                    interval.end_ns.min(extent.end_ns),
                ),
                cancel,
            )?;
            if pins == 0 {
                self.overlapping(interval, cancel, |block| {
                    let samples: Vec<_> = block.into_iter().map(|s| s.sample).collect();
                    Ok(sink.samples(&samples)?)
                })?;
            } else {
                self.export_digital(&mut sink, interval, cancel)?;
            }
            sink.finish_with_cancel(cancel)?;
            cancelled(cancel)?;
            temp.persist_noclobber(path).map_err(|error| error.error)?;
            Ok(())
        })();
        self.db.execute_batch("ROLLBACK")?;
        result
    }
}

/// One writer per archive. Buffers hold at most one block and 128 original stream messages.
pub struct CaptureWriter {
    db: Connection,
    header: Option<Header>,
    /// The header's period in ns, rounded up: the most a sample covers.
    period: u64,
    converter: Option<Converter>,
    samples: Vec<Supported>,
    pending: Option<Sample>,
    messages: Vec<Message>,
    blocks: u64,
    lost: u64,
    next: Option<u32>,
    extent: Interval,
    ended: bool,
    message_n: u64,
    digital: super::digital::DigitalWriter,
    markers: super::markers::MarkerWriter,
}

impl CaptureWriter {
    pub fn create(path: &Path) -> Result<Self> {
        // create_new prevents a mistaken path from replacing another capture.
        fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)?;
        let db = Connection::open(path)?;
        configure(&db)?;
        db.execute_batch(&format!("PRAGMA application_id={APPLICATION}; PRAGMA user_version=5;
            PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL;
            CREATE TABLE metadata(key TEXT PRIMARY KEY,value BLOB NOT NULL);
            CREATE TABLE blocks(n INTEGER PRIMARY KEY,start_ns INTEGER,end_ns INTEGER,data BLOB NOT NULL);
            CREATE INDEX blocks_time ON blocks(start_ns,end_ns);
            CREATE TABLE nodes(level INTEGER,n INTEGER,data BLOB NOT NULL,PRIMARY KEY(level,n)) WITHOUT ROWID;
            CREATE TABLE messages(n INTEGER PRIMARY KEY,data BLOB NOT NULL);
            CREATE TABLE markers(n INTEGER PRIMARY KEY,time_ns INTEGER,data BLOB NOT NULL);
            CREATE INDEX markers_time ON markers(time_ns,n);
            CREATE TABLE dio_blocks(n INTEGER PRIMARY KEY,start_ns INTEGER,end_ns INTEGER,data BLOB NOT NULL);
            CREATE INDEX dio_time ON dio_blocks(end_ns,start_ns);
            CREATE TABLE dio_nodes(level INTEGER,n INTEGER,data BLOB NOT NULL,PRIMARY KEY(level,n)) WITHOUT ROWID;"))?;
        let writer = Self {
            db,
            header: None,
            period: 0,
            converter: None,
            samples: Vec::with_capacity(BLOCK + 100),
            pending: None,
            messages: Vec::new(),
            blocks: 0,
            lost: 0,
            next: None,
            extent: Interval::default(),
            ended: false,
            message_n: 0,
            digital: super::digital::DigitalWriter::default(),
            markers: super::markers::MarkerWriter::default(),
        };
        writer.digital.flush(&writer.db, 0)?;
        writer.markers.flush(&writer.db)?;
        writer.metadata()?;
        Ok(writer)
    }
    fn set<T: Serialize>(&self, key: &str, value: &T) -> Result<()> {
        self.db.execute(
            "INSERT OR REPLACE INTO metadata VALUES(?,?)",
            params![key, encode(value)?],
        )?;
        Ok(())
    }
    fn metadata(&self) -> Result<()> {
        self.set("blocks", &self.blocks)?;
        self.set("extent", &self.extent)?;
        self.set("lost", &self.lost)?;
        self.set("ended", &self.ended)?;
        Ok(())
    }
    pub fn ingest(&mut self, message: Message) -> Result<bool> {
        match &message {
            Message::Header(header) => {
                ensure(self.header.is_none(), CaptureError::Restarted)?;
                ensure(
                    (1..=100_000).contains(&header.rate),
                    CaptureError::Rate(header.rate),
                )?;
                self.set("header", header)?;
                if let Some(id) = header.capture_id {
                    self.set("capture_id", &id)?;
                }
                self.period = 1_000_000_000u64.div_ceil(u64::from(header.rate));
                self.header = Some(header.clone());
                self.converter = Some(Converter::new(header));
                self.next = Some(header.first_sequence);
                self.digital.tail = Some((0, Some(header.dio_levels)));
            }
            Message::Pairs(pairs) => {
                let Some(converter) = self.converter.as_mut() else {
                    return Ok(false);
                };
                if let Some(next) = self.next {
                    let missing = skipped(next, pairs.first_sequence);
                    self.lost += missing;
                    if missing > 0
                        && let Some(pair) = pairs.pairs.first()
                    {
                        converter.discontinuity(pair.low);
                    }
                }
                self.next = Some(pairs.first_sequence.wrapping_add(pairs.pairs.len() as u32));
                converter.add_message(pairs);
                let samples = converter.take_samples();
                self.append(samples)?;
            }
            Message::End(end) => {
                let Some(converter) = self.converter.as_mut() else {
                    return Ok(false);
                };
                if let Some(next) = self.next {
                    self.lost += skipped(next, end.last_sequence.wrapping_add(1));
                }
                converter.end();
                let samples = converter.take_samples();
                self.append(samples)?;
                self.final_sample();
                self.ended = true;
            }
            Message::Edges(_) if self.header.is_none() => return Ok(false),
            Message::Edges(edges) => {
                if edges.lost() {
                    self.digital.losses += 1;
                }
                for (index, &edge) in edges.edges.iter().enumerate() {
                    self.digital.edge(edge, index == 0 && edges.lost())?;
                }
                if edges.lost() && edges.edges.is_empty() {
                    self.digital.lost();
                }
            }
            Message::Markers(_) if self.header.is_none() => return Ok(false),
            Message::Markers(markers) => self.markers.ingest(markers),
        }
        self.messages.push(message);
        if self.samples.len() >= BLOCK || self.messages.len() >= 128 || self.ended {
            self.flush()?;
        }
        Ok(self.ended)
    }
    fn append(&mut self, samples: impl IntoIterator<Item = Sample>) -> Result<()> {
        for sample in samples {
            if let Some(previous) = self.pending.replace(sample) {
                ensure(sample.time_ns > previous.time_ns, CaptureError::Timestamps)?;
                self.samples.push(Supported {
                    sample: previous,
                    end_ns: sample
                        .time_ns
                        .min(previous.time_ns.saturating_add(self.period)),
                });
            }
        }
        Ok(())
    }
    fn final_sample(&mut self) {
        if let Some(sample) = self.pending.take() {
            self.samples.push(Supported {
                sample,
                end_ns: sample.time_ns.saturating_add(self.period),
            });
        }
    }
    pub fn flush(&mut self) -> Result<()> {
        self.db.execute_batch("BEGIN IMMEDIATE")?;
        let result = self.flush_inner();
        match result {
            Ok(()) => {
                if let Err(error) = self.db.execute_batch("COMMIT") {
                    let _ = self.db.execute_batch("ROLLBACK");
                    return Err(error.into());
                }
                self.message_n += self.messages.len() as u64;
                self.messages.clear();
                if !self.samples.is_empty() {
                    if self.blocks == 0 {
                        self.extent.start_ns = self.samples[0].sample.time_ns;
                    }
                    self.extent.end_ns = self.samples.last().unwrap().end_ns;
                    self.blocks += 1;
                    self.samples.clear();
                }
                self.digital.committed();
                self.markers.committed();
                Ok(())
            }
            Err(error) => {
                let _ = self.db.execute_batch("ROLLBACK");
                Err(error)
            }
        }
    }
    fn flush_inner(&mut self) -> Result<()> {
        self.markers.flush(&self.db)?;
        self.digital
            .flush(&self.db, self.header.as_ref().map_or(0, |h| h.dio_mask))?;
        for (offset, message) in self.messages.iter().enumerate() {
            self.db.execute(
                "INSERT INTO messages VALUES(?,?)",
                params![(self.message_n + offset as u64) as i64, encode(message)?],
            )?;
        }
        if !self.samples.is_empty() {
            let start = self.samples[0].sample.time_ns;
            let end = self.samples.last().expect("samples").end_ns;
            let interval = Interval::new(start, end);
            let mut stats = Statistics::default();
            for sample in &self.samples {
                stats.add(sample.sample, sample.end_ns, interval);
            }
            self.db.execute(
                "INSERT INTO blocks VALUES(?,?,?,?)",
                params![
                    self.blocks as i64,
                    i64::try_from(start).map_err(|_| CaptureError::Time)?,
                    i64::try_from(end).map_err(|_| CaptureError::Time)?,
                    encode(&self.samples)?
                ],
            )?;
            self.put_node(0, self.blocks, &Node { interval, stats })?;
            let mut index = self.blocks;
            let mut level = 0;
            while index > 0 || level == 0 {
                index /= FAN;
                level += 1;
                let mut parent = Node {
                    interval: Interval::new(u64::MAX, u64::MAX),
                    stats: Statistics::default(),
                };
                let mut statement = self
                    .db
                    .prepare("SELECT data FROM nodes WHERE level=? AND n>=? AND n<? ORDER BY n")?;
                let rows = statement.query_map(
                    params![level - 1, (index * FAN) as i64, ((index + 1) * FAN) as i64],
                    |r| r.get::<_, Vec<u8>>(0),
                )?;
                for row in rows {
                    let node: Node = decode(&row?)?;
                    if parent.stats.samples == 0 {
                        parent.interval = node.interval;
                    } else {
                        parent.interval.end_ns = node.interval.end_ns;
                    }
                    parent.stats.merge(&node.stats);
                }
                drop(statement);
                self.put_node(level, index, &parent)?;
                if index == 0 {
                    break;
                }
            }
            let extent = Interval::new(
                if self.blocks == 0 {
                    start
                } else {
                    self.extent.start_ns
                },
                end,
            );
            self.set("blocks", &(self.blocks + 1))?;
            self.set("extent", &extent)?;
        } else {
            self.set("blocks", &self.blocks)?;
            self.set("extent", &self.extent)?;
        }
        self.set("lost", &self.lost)?;
        self.set("ended", &self.ended)
    }
    fn put_node(&self, level: u32, index: u64, node: &Node) -> Result<()> {
        self.db.execute(
            "INSERT OR REPLACE INTO nodes VALUES(?,?,?)",
            params![level, index as i64, encode(node)?],
        )?;
        Ok(())
    }
    pub fn finish(mut self) -> Result<()> {
        if let Some(converter) = self.converter.as_mut() {
            converter.end();
            let samples = converter.take_samples();
            self.append(samples)?;
        }
        self.final_sample();
        self.flush()?;
        // Out of WAL mode, a finished capture is one file. Both steps give way at once to a reader
        // in a query, which would otherwise hold the caller for the busy timeout; the capture then
        // stays in WAL mode, which reads the same.
        self.db.busy_timeout(std::time::Duration::ZERO)?;
        let _ = self
            .db
            .query_row("PRAGMA wal_checkpoint(TRUNCATE)", [], |r| {
                r.get::<_, i64>(0)
            });
        let _ = self
            .db
            .query_row("PRAGMA journal_mode=DELETE", [], |r| r.get::<_, String>(0));
        Ok(())
    }
}

pub fn skipped(next: u32, first: u32) -> u64 {
    let difference = first.wrapping_sub(next);
    if difference < 0x8000_0000 {
        u64::from(difference)
    } else {
        0
    }
}

/// Deletes the capture at `path`, with the WAL and its index a capture still in WAL mode keeps.
pub fn remove(path: &Path) -> Result<()> {
    fs::remove_file(path)?;
    for suffix in ["-wal", "-shm"] {
        let mut sibling = path.as_os_str().to_owned();
        sibling.push(suffix);
        match fs::remove_file(PathBuf::from(sibling)) {
            Err(error) if error.kind() != std::io::ErrorKind::NotFound => return Err(error.into()),
            _ => {}
        }
    }
    Ok(())
}

pub fn recovered_sessions(directory: &Path) -> Result<Vec<PathBuf>> {
    if !directory.exists() {
        return Ok(Vec::new());
    }
    let mut paths = Vec::new();
    for entry in fs::read_dir(directory)? {
        let path = entry?.path();
        if path.extension().is_some_and(|ext| ext == "vergecapture") {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CalibrationPoint,
        stream::{CurrentRange, End, EndReason, Pair, Pairs},
    };
    use std::sync::atomic::AtomicBool;

    #[test]
    fn pre_identity_headers_and_archive_read_in_the_layout_before_version_5() {
        let bytes = include_bytes!("../../tests/fixtures/identity/header-f45a4ed.postcard");
        let message_bytes =
            include_bytes!("../../tests/fixtures/identity/message-header-f45a4ed.postcard");
        // The capture identity left the layout as it was; step 5's fields changed it.
        let old: HeaderV4 = decode(bytes).unwrap();
        assert_eq!((old.rate, old.first_sequence, old.dio_mask), (1000, 1, 8));
        assert_eq!(encode(&old).unwrap(), bytes);
        assert_eq!((message_bytes[0], &message_bytes[1..]), (0, &bytes[..]));
        assert!(decode::<Header>(bytes).is_err());
        let archive = Archive::open(
            &Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("tests/fixtures/identity/capture-f45a4ed.vergecapture"),
        )
        .unwrap();
        let header = archive.header().unwrap();
        assert!(header.capture_id.is_none());
        assert_eq!(
            (
                header.rate,
                header.switch_weight_q16,
                header.rearm_between_pairs
            ),
            (old.rate, 0, false)
        );
        let raw: Vec<u8> = archive
            .db
            .query_row("SELECT data FROM messages WHERE n=0", [], |row| row.get(0))
            .unwrap();
        assert_eq!(&raw, message_bytes);
        assert_eq!(
            archive
                .statistics(archive.extent().unwrap(), &AtomicBool::new(false))
                .unwrap()
                .samples,
            3
        );
    }

    #[test]
    fn archive_identity_is_separate_from_message_header_layout() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("identified.vergecapture");
        let mut h = header(1000);
        let mut token = [0; 24];
        token[16] = 9;
        h.capture_id = crate::CaptureId::from_bytes(token);
        let mut writer = CaptureWriter::create(&path).unwrap();
        writer.ingest(Message::Header(h.clone())).unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        assert_eq!(archive.header().unwrap(), h);
        let bytes: Vec<u8> = archive
            .db
            .query_row("SELECT data FROM messages WHERE n=0", [], |row| row.get(0))
            .unwrap();
        let Message::Header(stored) = decode::<Message>(&bytes).unwrap() else {
            panic!("header")
        };
        assert!(stored.capture_id.is_none());
    }

    fn header(rate: u32) -> Header {
        Header {
            capture_id: None,
            rate,
            setting_ma: Some(350),
            full_scale_mv: 3300,
            first_sequence: u32::MAX - 2,
            previous_low: true,
            low: vec![
                CalibrationPoint {
                    code: 0,
                    current_nanoamps: 0,
                },
                CalibrationPoint {
                    code: 65535,
                    current_nanoamps: 100_000_000,
                },
            ],
            high: vec![],
            dio_mask: 0,
            dio_levels: 0,
            edge_resolution_ns: 0,
            switch_weight_q16: 0,
            rearm_between_pairs: false,
            previous_shunt_nanoamps: None,
        }
    }
    fn raw_writer(path: &Path, rate: u32) -> CaptureWriter {
        let mut writer = CaptureWriter::create(path).unwrap();
        writer.ingest(Message::Header(header(rate))).unwrap();
        writer
    }
    fn sample(i: u32) -> Sample {
        Sample {
            sequence: i,
            time_ns: u64::from(i) * 1_000_000,
            millivolts: 3000,
            nanoamps: if i.is_multiple_of(100) {
                100_000_000
            } else {
                1_000_000
            },
            range: Some(CurrentRange::new(!i.is_multiple_of(100))),
        }
    }
    #[test]
    fn markers_are_indexed_paged_and_exported_at_their_own_times() {
        use crate::stream::{Marker, Markers};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("markers.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..10).map(sample)).unwrap();
        writer.flush().unwrap();
        let markers: Vec<_> = (0..1100)
            .map(|number| Marker {
                time_ns: 2_000_000,
                number,
                label: format!("step,{number}").into_bytes(),
            })
            .collect();
        for chunk in markers.chunks(11) {
            writer
                .ingest(Message::Markers(Markers::new(
                    if chunk[0].number == 0 {
                        Markers::LOST
                    } else {
                        0
                    },
                    chunk.to_vec(),
                )))
                .unwrap();
        }
        writer.flush().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let interval = Interval::new(2_000_000, 3_000_000);
        let mut after = None;
        let mut numbers = Vec::new();
        loop {
            let page = archive
                .markers(interval, after, usize::MAX, &cancel)
                .unwrap();
            assert!(page.entries.len() <= 512);
            assert_eq!((page.total, page.loss_events), (1100, 1));
            numbers.extend(page.entries.iter().map(|m| m.marker.number));
            after = page.next;
            if after.is_none() {
                break;
            }
        }
        assert_eq!(numbers, (0..1100).collect::<Vec<_>>());
        assert!(
            archive
                .markers(Interval::new(0, 2_000_000), None, 10, &cancel)
                .unwrap()
                .entries
                .is_empty()
        );
        assert!(
            archive
                .markers(interval, None, 10, &AtomicBool::new(true))
                .is_err()
        );
        let out = dir.path().join("selected.csv");
        archive.export(&out, true, interval, &cancel).unwrap();
        let mut csv = csv::Reader::from_path(&out).unwrap();
        let fields = csv.headers().unwrap().clone();
        let number = fields.iter().position(|f| f == "marker_number").unwrap();
        let rows: Vec<_> = csv.records().map(|r| r.unwrap()).collect();
        assert_eq!(rows.iter().filter(|r| !r[number].is_empty()).count(), 1100);
        assert!(
            rows.windows(2).all(
                |rows| rows[0][1].parse::<u64>().unwrap() <= rows[1][1].parse::<u64>().unwrap()
            )
        );
        assert_eq!(&rows[0][number + 1], "step,0");
        let saved = dir.path().join("saved.vergecapture");
        archive.save(&saved).unwrap();
        assert_eq!(
            Archive::open(&saved)
                .unwrap()
                .markers(interval, None, 1, &cancel)
                .unwrap()
                .total,
            1100
        );
        drop(archive);
        writer.finish().unwrap();
        let db = Connection::open(&saved).unwrap();
        old_blocks(&db);
        db.execute_batch("PRAGMA user_version=2; DROP TABLE markers;")
            .unwrap();
        drop(db);
        assert_eq!(
            Archive::open(&saved)
                .unwrap()
                .markers(interval, None, 10, &cancel)
                .unwrap()
                .total,
            0
        );
    }

    #[test]
    fn marker_extent_includes_before_after_and_marker_only_captures() {
        use crate::stream::{Marker, Markers};
        let dir = tempfile::tempdir().unwrap();
        for analog in [true, false] {
            let path = dir.path().join(format!("extent-{analog}.vergecapture"));
            let mut writer = raw_writer(&path, 1000);
            if analog {
                writer.append([sample(3)]).unwrap();
            }
            writer
                .ingest(Message::Markers(Markers::new(
                    0,
                    vec![
                        Marker {
                            time_ns: 2_000_000,
                            number: 1,
                            label: b"before".to_vec(),
                        },
                        Marker {
                            time_ns: 8_000_000,
                            number: 2,
                            label: b"after".to_vec(),
                        },
                    ],
                )))
                .unwrap();
            writer.finish().unwrap();
            let archive = Archive::open(&path).unwrap();
            let extent = archive.extent().unwrap();
            assert_eq!(extent, Interval::new(2_000_000, 8_000_001));
            let out = dir.path().join(format!("extent-{analog}.csv"));
            archive
                .export(&out, true, extent, &AtomicBool::new(false))
                .unwrap();
            let text = fs::read_to_string(out).unwrap();
            assert!(text.contains("before") && text.contains("after"), "{text}");
        }
    }

    #[test]
    fn marker_flush_rolls_back_and_retries_with_analog() {
        use crate::stream::{Marker, Markers};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("retry-markers.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..8).map(sample)).unwrap();
        writer
            .ingest(Message::Markers(Markers::new(
                Markers::LOST,
                vec![Marker {
                    time_ns: 2_000_000,
                    number: 5,
                    label: vec![0xff],
                }],
            )))
            .unwrap();
        writer.db.execute_batch("CREATE TRIGGER fail_blocks BEFORE INSERT ON blocks BEGIN SELECT RAISE(FAIL, 'injected write failure'); END;").unwrap();
        assert!(writer.flush().is_err());
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let interval = Interval::new(0, 8_000_000);
        assert_eq!(
            archive.markers(interval, None, 10, &cancel).unwrap().total,
            0
        );
        writer.db.execute_batch("DROP TRIGGER fail_blocks").unwrap();
        writer.finish().unwrap();
        let page = archive.markers(interval, None, 10, &cancel).unwrap();
        assert_eq!((page.total, page.loss_events), (1, 1));
        assert_eq!(page.entries[0].marker.label, [0xff]);
        assert!(page.entries[0].lost_before);
    }

    #[test]
    fn review_the_edge_before_a_loss_keeps_its_level() {
        use crate::stream::{Edge, Edges};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("loss.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        let mut h = header(1000);
        h.dio_mask = 8;
        writer.header = Some(h.clone());
        writer.set("header", &h).unwrap();
        writer.append((1..8).map(sample)).unwrap();
        for (flags, time_ns, levels) in [(0, 2_000_000, 8), (Edges::LOST, 5_000_000, 0)] {
            writer
                .ingest(Message::Edges(Edges::new(
                    flags,
                    vec![Edge { time_ns, levels }],
                )))
                .unwrap();
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let extent = archive.extent().unwrap();
        let view = archive.digital(extent, 1000, &cancel).unwrap();
        assert!(view.bins.iter().any(|b| b.high & 8 != 0));
        let out = dir.path().join("loss.csv");
        archive.export(&out, true, extent, &cancel).unwrap();
        let csv = fs::read_to_string(out).unwrap();
        assert!(csv.contains(",2000000,,,,1,0\n,2000000,,,,,1\n"), "{csv}");
    }

    #[test]
    fn review_pulse_at_block_end_lands_in_its_own_bin() {
        use crate::stream::{Edge, Edges};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("pulse.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let mut h = header(1000);
        h.dio_mask = 8;
        writer.ingest(Message::Header(h)).unwrap();
        writer.append((0..2).map(sample)).unwrap();
        writer
            .ingest(Message::Edges(Edges::new(
                0,
                vec![
                    Edge {
                        time_ns: 1_000_000,
                        levels: 8,
                    },
                    Edge {
                        time_ns: 1_000_000,
                        levels: 0,
                    },
                ],
            )))
            .unwrap();
        writer.flush().unwrap();
        writer.append((2..5).map(sample)).unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        for pixels in [2, 4] {
            let view = archive
                .digital(Interval::new(0, 2_000_000), pixels, &cancel)
                .unwrap();
            let high: Vec<_> = view.bins.iter().filter(|b| b.high & 8 != 0).collect();
            assert_eq!(high.len(), 1);
            assert_eq!(high[0].interval.start_ns, 1_000_000);
        }
        let out = dir.path().join("pulse.csv");
        archive
            .export(&out, true, archive.extent().unwrap(), &cancel)
            .unwrap();
        let csv = fs::read_to_string(out).unwrap();
        assert!(!csv.lines().any(|row| row.starts_with(",0,")), "{csv}");
        assert_eq!(csv.lines().filter(|row| row.starts_with(',')).count(), 2);
    }

    #[test]
    fn origin_pulse_exports_both_real_edges_without_a_header_edge() {
        use crate::stream::{Edge, Edges};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("origin.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let mut h = header(1000);
        h.dio_mask = 8;
        writer.ingest(Message::Header(h)).unwrap();
        writer
            .ingest(Message::Edges(Edges::new(
                0,
                vec![
                    Edge {
                        time_ns: 0,
                        levels: 8,
                    },
                    Edge {
                        time_ns: 0,
                        levels: 0,
                    },
                ],
            )))
            .unwrap();
        writer.append((0..4).map(sample)).unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let out = dir.path().join("origin.csv");
        archive
            .export(
                &out,
                true,
                archive.extent().unwrap(),
                &AtomicBool::new(false),
            )
            .unwrap();
        let csv = fs::read_to_string(out).unwrap();
        let edges: Vec<_> = csv.lines().filter(|row| row.starts_with(',')).collect();
        assert_eq!(edges, [",0,,,,1,0", ",0,,,,0,0"]);
        assert!(csv.contains("1,1000000,3000,1000000,low,0,0"), "{csv}");
    }

    #[test]
    fn failed_flush_can_retry_without_losing_digital_or_analog_blocks() {
        use crate::stream::{Edge, Edges};
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("retry.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..8).map(sample)).unwrap();
        writer
            .ingest(Message::Edges(Edges::new(
                0,
                vec![Edge {
                    time_ns: 2_000_000,
                    levels: 8,
                }],
            )))
            .unwrap();
        writer.db.execute_batch("CREATE TRIGGER fail_blocks BEFORE INSERT ON blocks BEGIN SELECT RAISE(FAIL, 'injected write failure'); END;").unwrap();
        assert!(writer.flush().is_err());
        assert_eq!(writer.blocks, 0);
        assert_eq!(writer.message_n, 0);
        let blocks: i64 = writer
            .db
            .query_row("SELECT count(*) FROM dio_blocks", [], |r| r.get(0))
            .unwrap();
        assert_eq!(blocks, 0);
        writer.db.execute_batch("DROP TRIGGER fail_blocks").unwrap();
        writer.flush().unwrap();
        assert_eq!(writer.message_n, 2);
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        assert_eq!(
            archive
                .statistics(archive.extent().unwrap(), &AtomicBool::new(false))
                .unwrap()
                .samples,
            8
        );
        assert_eq!(archive.meta::<u64>("dio_blocks").unwrap(), 1);
    }

    #[test]
    fn digital_queries_and_exports_keep_pulses_and_unknown_intervals() {
        use crate::stream::{Edge, Edges};
        use std::io::Read;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("dio.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let empty = Archive::open(&path).unwrap();
        assert!(
            empty
                .digital(Interval::new(0, 1), 20, &AtomicBool::new(false))
                .unwrap()
                .bins
                .is_empty()
        );
        drop(empty);
        let mut h = header(1000);
        h.dio_mask = 0x0f;
        h.dio_levels = 0;
        writer.ingest(Message::Header(h)).unwrap();
        writer.digital.tail = Some((0, Some(0)));
        writer.append((0..10).map(sample)).unwrap();
        for (flags, edges) in [
            (0, vec![(1_200_000, 8), (1_400_000, 0), (3_000_000, 8)]),
            (Edges::LOST, vec![(6_000_000, 0), (8_000_000, 8)]),
        ] {
            writer
                .ingest(Message::Edges(Edges::new(
                    flags,
                    edges
                        .into_iter()
                        .map(|(time_ns, levels)| Edge { time_ns, levels })
                        .collect(),
                )))
                .unwrap();
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let digital = archive
            .digital(Interval::new(0, 10_000_000), 100, &cancel)
            .unwrap();
        assert_eq!((digital.pins, digital.loss_events), (15, 1));
        assert!(
            digital
                .bins
                .iter()
                .filter(|b| b.interval.start_ns >= 3_000_000 && b.interval.end_ns <= 6_000_000)
                .all(|b| b.gap)
        );
        assert!(
            digital
                .bins
                .iter()
                .any(|b| b.interval.start_ns == 1_200_000 && b.high == 8)
        );
        let coarse = archive
            .digital(Interval::new(0, 10_000_000), 1, &cancel)
            .unwrap();
        assert_eq!(coarse.bins.len(), 1);
        assert_eq!(
            (coarse.bins[0].high, coarse.bins[0].low, coarse.bins[0].gap),
            (8, 15, true)
        );
        for csv in [true, false] {
            let out = dir.path().join(if csv { "dio.csv" } else { "dio.sr" });
            archive
                .export(&out, csv, Interval::new(1_000_000, 9_000_000), &cancel)
                .unwrap();
            if csv {
                let text = fs::read_to_string(&out).unwrap();
                assert!(text.contains(",1200000,,,,0,0,0,1,0\n"), "{text}");
                assert!(
                    text.contains(",3000000,,,,0,0,0,1,0\n,3000000,,,,,,,,1\n"),
                    "{text}"
                );
                assert!(
                    text.contains("4,4000000,3000,1000000,low,,,,,1\n"),
                    "{text}"
                );
                assert!(!text.contains("9,9000000"));
            } else {
                let mut zip = zip::ZipArchive::new(fs::File::open(&out).unwrap()).unwrap();
                let mut logic = Vec::new();
                zip.by_name("logic-1-1")
                    .unwrap()
                    .read_to_end(&mut logic)
                    .unwrap();
                // DIO3's pulse is stretched at 2 ms. Bit 5 marks unknown DIO at 3..6 ms.
                assert_eq!(logic, [0, 16, 48, 48, 48, 32, 0, 16]);
            }
        }
        let out = dir.path().join("inside-loss.csv");
        archive
            .export(&out, true, Interval::new(4_500_000, 5_500_000), &cancel)
            .unwrap();
        let text = fs::read_to_string(out).unwrap();
        assert!(text.contains("4,4000000,3000,1000000,low,,,,,1"), "{text}");
        assert!(
            archive
                .digital(Interval::new(0, 10), 3, &AtomicBool::new(true))
                .is_err()
        );
    }

    #[test]
    fn digital_history_is_bounded_and_old_analog_archives_still_open() {
        use crate::stream::Edge;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("many.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let mut h = header(1000);
        h.dio_mask = 1;
        writer.ingest(Message::Header(h)).unwrap();
        writer.digital.tail = Some((0, Some(0)));
        for block in 0..300u32 {
            for n in block * 100..(block + 1) * 100 {
                writer
                    .digital
                    .edge(
                        Edge {
                            time_ns: u64::from(n + 1) * 10_000,
                            levels: (n % 2) as u8,
                        },
                        false,
                    )
                    .unwrap();
            }
            writer.append([sample(block)]).unwrap();
            writer.flush().unwrap();
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let view = archive
            .digital(Interval::new(0, 300_000_000), 13, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(view.bins.len(), 13);
        assert!(view.bins.iter().all(|bin| bin.high == 1 && bin.low == 1));
        drop(archive);
        let db = Connection::open(&path).unwrap();
        old_blocks(&db);
        db.execute_batch("PRAGMA user_version=1; DROP TABLE dio_nodes; DROP TABLE dio_blocks;")
            .unwrap();
        drop(db);
        let archive = Archive::open(&path).unwrap();
        assert_eq!(
            archive
                .digital(archive.extent().unwrap(), 13, &AtomicBool::new(false))
                .unwrap()
                .pins,
            0
        );
        assert_eq!(
            archive
                .statistics(archive.extent().unwrap(), &AtomicBool::new(false))
                .unwrap()
                .samples,
            300
        );
    }

    #[test]
    fn attached_stream_accepts_edges_before_its_first_pair_and_same_time_pulses() {
        use crate::stream::{Edge, Edges};
        use std::io::Read;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("attached.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let mut h = header(1000);
        h.dio_mask = 8;
        writer.ingest(Message::Header(h)).unwrap();
        writer
            .ingest(Message::Pairs(Pairs::new(
                0,
                u32::MAX - 2,
                1_000_000,
                vec![
                    Pair {
                        voltage_code: 32768,
                        current_code: 1000,
                        low: true
                    };
                    4
                ],
            )))
            .unwrap();
        writer
            .ingest(Message::Edges(Edges::new(
                0,
                vec![
                    Edge {
                        time_ns: 500_000,
                        levels: 8,
                    },
                    Edge {
                        time_ns: 1_500_000,
                        levels: 0,
                    },
                    Edge {
                        time_ns: 2_700_000,
                        levels: 8,
                    },
                    Edge {
                        time_ns: 2_700_000,
                        levels: 0,
                    },
                ],
            )))
            .unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let view = archive
            .digital(Interval::new(0, 4_000_000), 40, &cancel)
            .unwrap();
        assert!(view.bins.iter().all(|b| b.interval.start_ns >= 1_000_000));
        assert!(
            view.bins
                .iter()
                .any(|b| b.interval.start_ns == 2_700_000 && b.high == 8 && b.low == 8)
        );
        let csv = dir.path().join("attached.csv");
        archive
            .export(&csv, true, archive.extent().unwrap(), &cancel)
            .unwrap();
        let text = fs::read_to_string(csv).unwrap();
        assert!(
            text.contains(",2700000,,,,1,0\n,2700000,,,,0,0\n"),
            "{text}"
        );
        let sr = dir.path().join("attached.sr");
        archive
            .export(&sr, false, archive.extent().unwrap(), &cancel)
            .unwrap();
        let mut zip = zip::ZipArchive::new(fs::File::open(sr).unwrap()).unwrap();
        let mut logic = Vec::new();
        zip.by_name("logic-1-1")
            .unwrap()
            .read_to_end(&mut logic)
            .unwrap();
        assert_eq!(
            logic,
            [2, 0, 2, 0],
            "the sub-sample pulse must be stretched"
        );
    }

    #[test]
    fn sigrok_selection_in_an_analog_gap_keeps_digital_data() {
        use crate::stream::{Edge, Edges};
        use std::io::Read;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("gap.vergecapture");
        let mut writer = CaptureWriter::create(&path).unwrap();
        let mut h = header(1000);
        h.dio_mask = 8;
        writer.ingest(Message::Header(h)).unwrap();
        writer.append([sample(0), sample(10)]).unwrap();
        writer
            .ingest(Message::Edges(Edges::new(
                0,
                vec![
                    Edge {
                        time_ns: 3_000_000,
                        levels: 8,
                    },
                    Edge {
                        time_ns: 4_000_000,
                        levels: 0,
                    },
                ],
            )))
            .unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let csv = dir.path().join("static-gap.csv");
        archive
            .export(
                &csv,
                true,
                Interval::new(4_500_000, 6_000_000),
                &AtomicBool::new(false),
            )
            .unwrap();
        assert!(
            fs::read_to_string(csv)
                .unwrap()
                .contains(",4500000,,,,0,0\n")
        );
        for (name, interval, expected) in [
            (
                "edge-start.sr",
                Interval::new(4_000_000, 6_000_000),
                vec![1, 1],
            ),
            (
                "high-gap.sr",
                Interval::new(3_500_000, 6_000_000),
                vec![3, 1, 1],
            ),
            (
                "gap.sr",
                Interval::new(2_000_000, 6_000_000),
                vec![1, 3, 1, 1],
            ),
            (
                "leading.sr",
                Interval::new(2_000_000, 11_000_000),
                vec![1, 3, 1, 1, 1, 1, 1, 1, 0],
            ),
        ] {
            let sr = dir.path().join(name);
            archive
                .export(&sr, false, interval, &AtomicBool::new(false))
                .unwrap();
            let mut zip = zip::ZipArchive::new(fs::File::open(sr).unwrap()).unwrap();
            let mut logic = Vec::new();
            zip.by_name("logic-1-1")
                .unwrap()
                .read_to_end(&mut logic)
                .unwrap();
            assert_eq!(logic, expected, "{name}");
        }
    }

    #[test]
    fn selected_digital_state_is_seeded_without_creating_an_edge() {
        use crate::stream::{Edge, Edges};
        use std::io::Read;
        let dir = tempfile::tempdir().unwrap();
        for pulse in [false, true] {
            let path = dir.path().join(format!("seed-{pulse}.vergecapture"));
            let mut writer = CaptureWriter::create(&path).unwrap();
            let mut h = header(1000);
            h.dio_mask = 8;
            h.dio_levels = 8;
            writer.ingest(Message::Header(h)).unwrap();
            writer.append((1..5).map(sample)).unwrap();
            let edges = if pulse {
                vec![(1_000_000, 0), (2_000_000, 8), (2_000_000, 0)]
            } else {
                vec![(2_000_000, 0)]
            };
            writer
                .ingest(Message::Edges(Edges::new(
                    0,
                    edges
                        .into_iter()
                        .map(|(time_ns, levels)| Edge { time_ns, levels })
                        .collect(),
                )))
                .unwrap();
            writer.finish().unwrap();
            let archive = Archive::open(&path).unwrap();
            for (suffix, interval, expected) in [
                (
                    "selected",
                    Interval::new(2_000_000, 4_000_000),
                    if pulse { vec![2, 0] } else { vec![0, 0] },
                ),
                (
                    "before",
                    Interval::new(0, 2_000_000),
                    if pulse { vec![0] } else { vec![2] },
                ),
                ("after", Interval::new(30_000_000, 31_000_000), vec![]),
            ] {
                let out = dir.path().join(format!("{pulse}-{suffix}.sr"));
                archive
                    .export(&out, false, interval, &AtomicBool::new(false))
                    .unwrap();
                let mut zip = zip::ZipArchive::new(fs::File::open(out).unwrap()).unwrap();
                let mut logic = Vec::new();
                if let Ok(mut file) = zip.by_name("logic-1-1") {
                    file.read_to_end(&mut logic).unwrap();
                }
                assert_eq!(logic, expected, "{pulse} {suffix}");
            }
            let view = archive
                .digital(
                    Interval::new(2_000_000, 4_000_000),
                    20,
                    &AtomicBool::new(false),
                )
                .unwrap();
            assert_eq!(view.bins[0].high & 8 != 0, pulse);
        }
    }
    #[test]
    fn selections_merge_exactly_across_tree_and_block_boundaries() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        let count = 35000;
        for block in (0..count).step_by(BLOCK) {
            writer
                .append(
                    (block..(block + BLOCK as u32).min(count))
                        .filter(|i| !(*i >= 1200 && *i < 1250))
                        .map(sample),
                )
                .unwrap();
            writer.flush().unwrap();
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        for interval in [
            Interval::new(0, 35_000_000_000),
            Interval::new(1_199_500_000, 1_250_500_000),
            Interval::new(1, 1),
            Interval::new(0, 1),
            Interval::new(1_024_000_001, 33_017_123_456),
        ] {
            let mut expected = Statistics::default();
            for sample in (0..count)
                .filter(|i| !(*i >= 1200 && *i < 1250))
                .map(sample)
            {
                expected.add(sample, sample.time_ns + 1_000_000, interval);
            }
            let actual = archive
                .statistics(interval, &AtomicBool::new(false))
                .unwrap();
            assert_eq!(actual, expected, "{interval:?}");
            for pixels in [1, 37, 500] {
                assert_eq!(
                    archive
                        .view(interval, pixels, &AtomicBool::new(false))
                        .unwrap()
                        .statistics,
                    expected
                );
            }
        }
        let interval = Interval::new(0, 1_000_000);
        let s = archive
            .statistics(interval, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(s.charge_c(), 0.0001);
        assert!((s.energy_j() - 0.0003).abs() < 1e-15);
        let cancel = AtomicBool::new(true);
        assert!(archive.statistics(interval, &cancel).is_err());
    }
    #[test]
    fn a_view_holds_its_samples_once_they_are_no_more_than_its_bins() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        for block in (0..3000).step_by(BLOCK) {
            writer
                .append((block..(block + BLOCK as u32).min(3000)).map(sample))
                .unwrap();
            writer.flush().unwrap();
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        // Sample 1000 starts before the interval, and its support reaches into it.
        let interval = Interval::new(1_000_500_000, 1_100_000_000);
        let samples = archive
            .view(interval, 100, &cancel)
            .unwrap()
            .samples
            .unwrap();
        assert_eq!(samples.len(), 100);
        assert_eq!(samples[0].sample, sample(1000));
        assert_eq!(samples[99].end_ns, 1_100_000_000);
        assert!(
            archive
                .view(interval, 99, &cancel)
                .unwrap()
                .samples
                .is_none()
        );
        // Across a block boundary, in order.
        let interval = Interval::new(1_000_000_000, 1_048_000_000);
        let samples = archive
            .view(interval, 64, &cancel)
            .unwrap()
            .samples
            .unwrap();
        let sequences: Vec<_> = samples.iter().map(|s| s.sample.sequence).collect();
        assert_eq!(sequences, (1000..1048).collect::<Vec<_>>());
    }
    #[test]
    fn raw_metadata_wrap_loss_end_and_recovery_survive_reopen() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer
            .ingest(Message::Pairs(Pairs::new(
                0,
                u32::MAX - 2,
                0,
                vec![
                    Pair {
                        voltage_code: 32768,
                        current_code: 1000,
                        low: true
                    };
                    3
                ],
            )))
            .unwrap();
        writer
            .ingest(Message::Pairs(Pairs::new(
                Pairs::HELD,
                2,
                5_000_000,
                vec![
                    Pair {
                        voltage_code: 32768,
                        current_code: 2000,
                        low: true
                    };
                    2
                ],
            )))
            .unwrap();
        writer
            .ingest(Message::End(End {
                reason: EndReason::Stopped,
                last_sequence: 5,
            }))
            .unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let view = archive
            .view(archive.extent().unwrap(), 10, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(view.lost_pairs, 4);
        assert!(view.ended);
        assert_eq!(view.statistics.samples, 5);
        assert_eq!(view.statistics.measured_ns, 5_000_000);
        assert_eq!(archive.header().unwrap(), header(1000));
        let copy = dir.path().join("saved.vergecapture");
        archive.save(&copy).unwrap();
        assert_eq!(Archive::open(&copy).unwrap().extent().unwrap(), view.extent);
        assert!(archive.save(&copy).is_err());
        assert_eq!(recovered_sessions(dir.path()).unwrap().len(), 2);
        let count: i64 = archive
            .db
            .query_row("SELECT count(*) FROM messages", [], |r| r.get(0))
            .unwrap();
        assert_eq!(count, 4);
    }
    #[test]
    fn committed_capture_can_be_read_while_writer_is_alive() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..1024).map(sample)).unwrap();
        writer.flush().unwrap();
        let archive = Archive::open(&path).unwrap();
        let old = Interval::new(0, 512_000_000);
        let first = archive.statistics(old, &AtomicBool::new(false)).unwrap();
        writer.append((1024..2048).map(sample)).unwrap();
        writer.flush().unwrap();
        assert_eq!(
            archive.statistics(old, &AtomicBool::new(false)).unwrap(),
            first
        );
        assert_eq!(archive.extent().unwrap().end_ns, 2_047_000_000);
        drop(writer);
        assert_eq!(
            Archive::open(&path)
                .unwrap()
                .statistics(old, &AtomicBool::new(false))
                .unwrap(),
            first
        );
    }
    #[test]
    fn a_finished_capture_is_one_file_and_remove_takes_all_of_it() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("done.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..10).map(sample)).unwrap();
        writer.finish().unwrap();
        let names = || {
            let mut names: Vec<_> = fs::read_dir(dir.path())
                .unwrap()
                .map(|e| e.unwrap().file_name().into_string().unwrap())
                .collect();
            names.sort();
            names
        };
        assert_eq!(names(), ["done.vergecapture"]);
        assert_eq!(
            Archive::open(&path)
                .unwrap()
                .statistics(Interval::new(0, u64::MAX), &AtomicBool::new(false))
                .unwrap()
                .samples,
            10
        );
        // Reading it leaves nothing beside it either.
        assert_eq!(names(), ["done.vergecapture"]);
        // A crash leaves the WAL beside it.
        let live = dir.path().join("live.vergecapture");
        let mut writer = raw_writer(&live, 1000);
        writer.append((0..10).map(sample)).unwrap();
        writer.flush().unwrap();
        std::mem::forget(writer);
        assert!(names().len() > 2);
        remove(&live).unwrap();
        assert_eq!(names(), ["done.vergecapture"]);
    }
    #[test]
    fn finishing_does_not_wait_for_a_reader_in_a_query() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("busy.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..2048).map(sample)).unwrap();
        writer.flush().unwrap();
        let archive = Archive::open(&path).unwrap();
        archive.db.execute_batch("BEGIN").unwrap();
        archive.extent().unwrap();
        let started = std::time::Instant::now();
        writer.finish().unwrap();
        assert!(
            started.elapsed() < std::time::Duration::from_millis(500),
            "{:?}",
            started.elapsed()
        );
        archive.db.execute_batch("ROLLBACK").unwrap();
        assert_eq!(
            Archive::open(&path)
                .unwrap()
                .statistics(Interval::new(0, u64::MAX), &AtomicBool::new(false))
                .unwrap()
                .samples,
            2048
        );
    }
    #[test]
    fn a_writer_error_keeps_what_came_before_it() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("restart.vergecapture");
        let recorder = crate::capture::Recorder::create(&path).unwrap();
        let mut recorder = recorder;
        recorder.send(Message::Header(header(1000))).unwrap();
        let pairs = Pairs::new(
            0,
            header(1000).first_sequence,
            0,
            vec![
                Pair {
                    voltage_code: 32768,
                    current_code: 1000,
                    low: true
                };
                50
            ],
        );
        recorder.send(Message::Pairs(pairs)).unwrap();
        recorder.send(Message::Header(header(1000))).unwrap();
        assert!(matches!(recorder.finish(), Err(CaptureError::Restarted)));
        let stats = Archive::open(&path)
            .unwrap()
            .statistics(Interval::new(0, u64::MAX), &AtomicBool::new(false))
            .unwrap();
        assert_eq!(stats.samples, 50);
    }
    #[cfg(unix)]
    #[test]
    fn exports_get_the_permissions_of_a_new_file() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("p.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..10).map(sample)).unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let out = dir.path().join("p.csv");
        archive
            .export(
                &out,
                true,
                archive.extent().unwrap(),
                &AtomicBool::new(false),
            )
            .unwrap();
        let plain = dir.path().join("plain");
        fs::write(&plain, b"").unwrap();
        let mode = |p: &Path| fs::metadata(p).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode(&out), mode(&plain));
    }
    #[test]
    fn a_capture_left_by_a_crash_opens_read_only() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("live.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..2048).map(sample)).unwrap();
        writer.flush().unwrap();
        // The files as a killed process leaves them: committed pages still in the WAL, no locks.
        let crashed = dir.path().join("crashed");
        fs::create_dir(&crashed).unwrap();
        for suffix in ["", "-wal", "-shm"] {
            let name = format!("live.vergecapture{suffix}");
            fs::copy(dir.path().join(&name), crashed.join(&name)).unwrap();
        }
        assert!(
            fs::metadata(crashed.join("live.vergecapture-wal"))
                .unwrap()
                .len()
                > 0
        );
        let archive = Archive::open(&crashed.join("live.vergecapture")).unwrap();
        let stats = archive
            .statistics(archive.extent().unwrap(), &AtomicBool::new(false))
            .unwrap();
        assert_eq!(stats.samples, 2047);
        assert!(
            !archive
                .view(archive.extent().unwrap(), 8, &AtomicBool::new(false))
                .unwrap()
                .ended
        );
        let saved = dir.path().join("saved.vergecapture");
        archive.save(&saved).unwrap();
        assert_eq!(
            Archive::open(&saved)
                .unwrap()
                .statistics(archive.extent().unwrap(), &AtomicBool::new(false))
                .unwrap(),
            stats
        );
        drop(writer);
    }
    #[test]
    fn csv_export_keeps_full_resolution_and_selection_bounds() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..5).map(sample)).unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let out = dir.path().join("out.csv");
        archive
            .export(
                &out,
                true,
                Interval::new(1_000_000, 3_000_000),
                &AtomicBool::new(false),
            )
            .unwrap();
        let csv = fs::read_to_string(out).unwrap();
        assert_eq!(csv.lines().count(), 3);
        assert!(csv.contains("1,1000000,3000,1000000,low\n"));
        assert!(!csv.contains("3,3000000"));
    }
    /// Rewrites the header in the layout archives before version 5 stored.
    fn old_header(db: &Connection) {
        let data: Vec<u8> = db
            .query_row("SELECT value FROM metadata WHERE key='header'", [], |r| {
                r.get(0)
            })
            .unwrap();
        let header: Header = decode(&data).unwrap();
        let old = HeaderV4 {
            rate: header.rate,
            setting_ma: header.setting_ma,
            full_scale_mv: header.full_scale_mv,
            first_sequence: header.first_sequence,
            previous_low: header.previous_low,
            low: header.low,
            high: header.high,
            dio_mask: header.dio_mask,
            dio_levels: header.dio_levels,
            edge_resolution_ns: header.edge_resolution_ns,
        };
        db.execute(
            "UPDATE metadata SET value=? WHERE key='header'",
            params![encode(&old).unwrap()],
        )
        .unwrap();
    }
    /// Rewrites every sample block in the layout archives before version 4 stored, and the header
    /// in the one before version 5.
    fn old_blocks(db: &Connection) {
        old_header(db);
        let blocks: Vec<(i64, Vec<u8>)> = db
            .prepare("SELECT n, data FROM blocks")
            .unwrap()
            .query_map([], |r| Ok((r.get(0)?, r.get(1)?)))
            .unwrap()
            .map(|row| row.unwrap())
            .collect();
        for (n, data) in blocks {
            let old: Vec<SupportedV3> = decode::<Vec<Supported>>(&data)
                .unwrap()
                .into_iter()
                .map(|s| SupportedV3 {
                    sample: SampleV3 {
                        sequence: s.sample.sequence,
                        time_ns: s.sample.time_ns,
                        millivolts: s.sample.millivolts,
                        nanoamps: s.sample.nanoamps,
                    },
                    end_ns: s.end_ns,
                })
                .collect();
            db.execute(
                "UPDATE blocks SET data=? WHERE n=?",
                params![encode(&old).unwrap(), n],
            )
            .unwrap();
        }
    }
    #[test]
    fn version_4_headers_read_without_step_5() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("v4.vergecapture");
        let mut h = header(1000);
        h.switch_weight_q16 = 9786;
        h.rearm_between_pairs = true;
        h.previous_shunt_nanoamps = Some(1_000_000);
        let mut writer = CaptureWriter::create(&path).unwrap();
        writer.ingest(Message::Header(h.clone())).unwrap();
        writer.append((0..5).map(sample)).unwrap();
        writer.finish().unwrap();
        assert_eq!(Archive::open(&path).unwrap().header().unwrap(), h);
        let db = Connection::open(&path).unwrap();
        old_header(&db);
        db.execute_batch("PRAGMA user_version=4;").unwrap();
        drop(db);
        let archive = Archive::open(&path).unwrap();
        let read = archive.header().unwrap();
        assert_eq!((read.rate, read.low.len(), read.dio_mask), (1000, 2, 0));
        assert_eq!(
            (
                read.switch_weight_q16,
                read.rearm_between_pairs,
                read.previous_shunt_nanoamps
            ),
            (0, false, None)
        );
        let extent = archive.extent().unwrap();
        let samples = archive
            .view(extent, 64, &AtomicBool::new(false))
            .unwrap()
            .samples
            .unwrap();
        assert_eq!(samples.len(), 5);
    }
    #[test]
    fn version_3_blocks_read_without_a_range() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("v3.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer.append((0..5).map(sample)).unwrap();
        writer.finish().unwrap();
        let db = Connection::open(&path).unwrap();
        old_blocks(&db);
        db.execute_batch("PRAGMA user_version=3;").unwrap();
        drop(db);
        let archive = Archive::open(&path).unwrap();
        let cancel = AtomicBool::new(false);
        let extent = archive.extent().unwrap();
        let samples = archive.view(extent, 64, &cancel).unwrap().samples.unwrap();
        assert_eq!(samples.len(), 5);
        assert!(
            samples
                .iter()
                .all(|s| s.sample.range.is_none() && s.sample.millivolts == 3000)
        );
        let out = dir.path().join("v3.csv");
        archive.export(&out, true, extent, &cancel).unwrap();
        let csv = fs::read_to_string(out).unwrap();
        assert!(csv.contains("1,1000000,3000,1000000,\n"), "{csv}");
        assert_eq!(csv.lines().count(), 6, "{csv}");
    }
    #[test]
    fn partial_pulse_integrals_have_independent_integer_expectations() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("pulse.vergecapture");
        let mut writer = raw_writer(&path, 1000);
        writer
            .append((0..1026).map(|i| Sample {
                sequence: i,
                time_ns: u64::from(i) * 1_000_000,
                millivolts: 3000,
                nanoamps: if i == 1023 { 10_000_000 } else { 2_000_000 },
                range: Some(CurrentRange::High),
            }))
            .unwrap();
        writer.flush().unwrap();
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let stats = archive
            .statistics(
                Interval::new(1_022_500_000, 1_024_500_000),
                &AtomicBool::new(false),
            )
            .unwrap();
        assert_eq!(stats.samples, 3);
        assert_eq!(stats.measured_ns, 2_000_000);
        assert_eq!(stats.current_na_ns, 12_000_000_000_000);
        assert_eq!(stats.voltage_mv_ns, 6_000_000_000);
        assert_eq!(stats.power_mv_na_ns, 36_000_000_000_000_000);
        assert_eq!(
            (stats.min_na, stats.max_na),
            (Some(2_000_000), Some(10_000_000))
        );
        let csv = dir.path().join("half.csv");
        archive
            .export(
                &csv,
                true,
                Interval::new(1_023_500_000, 1_024_000_000),
                &AtomicBool::new(false),
            )
            .unwrap();
        let text = fs::read_to_string(&csv).unwrap();
        assert_eq!(text.lines().count(), 2);
        assert!(text.contains("1023,1023000000"));
        assert!(
            archive
                .export(
                    &csv,
                    true,
                    archive.extent().unwrap(),
                    &AtomicBool::new(false)
                )
                .is_err()
        );
        assert_eq!(fs::read_to_string(csv).unwrap(), text);
    }
    #[test]
    fn non_divisor_rates_have_continuous_coverage_and_binned_extrema() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("rate.vergecapture");
        let mut writer = raw_writer(&path, 1500);
        for first in (0..6000u32).step_by(99) {
            writer
                .append((first..(first + 99).min(6000)).map(|i| Sample {
                    sequence: i,
                    time_ns: u64::from(i) * 1_000_000_000 / 1500,
                    millivolts: 3000,
                    nanoamps: if i == 3000 { 5000 } else { 1000 },
                    range: Some(CurrentRange::Low),
                }))
                .unwrap();
            if writer.samples.len() >= BLOCK {
                writer.flush().unwrap();
            }
        }
        writer.finish().unwrap();
        let archive = Archive::open(&path).unwrap();
        let extent = archive.extent().unwrap();
        assert_eq!(extent, Interval::new(0, 4_000_000_000));
        let view = archive.view(extent, 3, &AtomicBool::new(false)).unwrap();
        assert_eq!(view.statistics.measured_ns, 4_000_000_000);
        assert_eq!(view.statistics.samples, 6000);
        assert_eq!(view.envelopes.len(), 3);
        assert!(view.envelopes.iter().all(|bin| !bin.gap));
        assert_eq!(view.envelopes[0].statistics.max_na, Some(1000));
        assert_eq!(view.envelopes[1].statistics.max_na, Some(5000));
        assert_eq!(view.envelopes[2].statistics.max_na, Some(1000));
        let bins = Bins {
            interval: Interval::new(100, 110),
            values: vec![Statistics::default(); 3],
            cancel: &AtomicBool::new(false),
        };
        assert_eq!(
            (
                bins.bin(100),
                bins.bin(102),
                bins.bin(103),
                bins.bin(106),
                bins.bin(109)
            ),
            (0, 0, 1, 2, 2)
        );
    }

    #[test]
    #[ignore = "generated long-capture performance check"]
    fn long_capture_queries_keep_a_bounded_working_set() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("long.vergecapture");
        let mut writer = raw_writer(&path, 100_000);
        let start = std::time::Instant::now();
        for first in (0..3_000_000u32).step_by(1024) {
            writer
                .append((first..(first + 1024).min(3_000_000)).map(|i| Sample {
                    sequence: i,
                    time_ns: u64::from(i) * 10_000,
                    millivolts: 3300,
                    nanoamps: if i.is_multiple_of(100_000) {
                        200_000_000
                    } else {
                        1_000_000
                    },
                    range: Some(CurrentRange::High),
                }))
                .unwrap();
            writer.flush().unwrap();
        }
        writer.finish().unwrap();
        let written = start.elapsed();
        let archive = Archive::open(&path).unwrap();
        let start = std::time::Instant::now();
        let view = archive
            .view(archive.extent().unwrap(), 1000, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(view.statistics.samples, 3_000_000);
        assert!(view.envelopes.len() <= 1000);
        let plotted = start.elapsed();
        let start = std::time::Instant::now();
        let old = archive
            .statistics(
                Interval::new(1_000_000_000, 2_000_000_000),
                &AtomicBool::new(false),
            )
            .unwrap();
        assert_eq!(old.samples, 100_000);
        eprintln!(
            "3M pairs write {written:?}; full envelope {plotted:?}; old selection {:?}; archive {} bytes",
            start.elapsed(),
            fs::metadata(&path).unwrap().len()
        );
    }
}
