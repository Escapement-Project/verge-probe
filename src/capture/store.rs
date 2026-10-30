use super::{CaptureError, Interval, Result, Statistics};
use crate::record::Sink;
use crate::stream::{Converter, Header, Message, Sample};
use rusqlite::{Connection, OpenFlags, OptionalExtension, params};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

const BLOCK: usize = 1024;
const FAN: u64 = 16;
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
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct Supported {
    sample: Sample,
    end_ns: u64,
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
fn bin_interval(interval: Interval, pixels: usize, index: usize) -> Interval {
    let at = |n| {
        interval.start_ns + (u128::from(interval.duration_ns()) * n as u128 / pixels as u128) as u64
    };
    Interval::new(at(index), at(index + 1))
}

pub struct Archive {
    db: Connection,
}

/// Fails with `error` unless `condition` holds.
fn ensure(condition: bool, error: CaptureError) -> Result<()> {
    if condition { Ok(()) } else { Err(error) }
}

fn cancelled(cancel: &AtomicBool) -> Result<()> {
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

/// A temporary file beside `path`, with the permissions a new file there gets rather than
/// tempfile's owner-only ones.
fn temp_beside(path: &Path) -> std::io::Result<tempfile::NamedTempFile> {
    let mut builder = tempfile::Builder::new();
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        builder.permissions(fs::Permissions::from_mode(0o666));
    }
    builder.tempfile_in(destination_dir(path))
}

fn destination_dir(path: &Path) -> &Path {
    path.parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."))
}

fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>> {
    Ok(postcard::to_stdvec(value)?)
}
fn decode<T: for<'a> Deserialize<'a>>(bytes: &[u8]) -> Result<T> {
    Ok(postcard::from_bytes(bytes)?)
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
        ensure(version == 1, CaptureError::Version(version))?;
        Ok(Self { db })
    }
    fn meta<T: for<'a> Deserialize<'a>>(&self, key: &str) -> Result<T> {
        let bytes: Vec<u8> =
            self.db
                .query_row("SELECT value FROM metadata WHERE key=?", [key], |r| {
                    r.get(0)
                })?;
        decode(&bytes)
    }
    pub fn header(&self) -> Result<Header> {
        self.meta("header")
    }
    pub fn extent(&self) -> Result<Interval> {
        self.meta("extent")
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
        decode(&bytes)
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
    /// A stable snapshot, with at most `pixels` extrema bins regardless of capture length.
    pub fn view(&self, interval: Interval, pixels: usize, cancel: &AtomicBool) -> Result<View> {
        self.db.execute_batch("BEGIN")?;
        let result = (|| {
            let extent = self.extent()?;
            let statistics = self.statistics_inner(interval, cancel)?;
            let pixels = pixels
                .clamp(1, 2048)
                .min(interval.duration_ns().clamp(1, 2048) as usize);
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
            Ok(View {
                extent,
                interval,
                statistics,
                envelopes,
                lost_pairs: self.meta("lost")?,
                ended: self.meta("ended")?,
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
    /// This export contains analog readings and GAP; DIO export remains in `record::Sink`.
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
            let temp = temp_beside(path)?;
            let mut sink = Sink::from_file(temp.reopen()?, csv)?;
            sink.begin(header.rate, 0, 0)?;
            let mut statement = self
                .db
                .prepare("SELECT data FROM blocks WHERE end_ns>? AND start_ns<? ORDER BY n")?;
            let mut rows = statement.query(params![
                interval.start_ns.min(i64::MAX as u64) as i64,
                interval.end_ns.min(i64::MAX as u64) as i64
            ])?;
            while let Some(row) = rows.next()? {
                cancelled(cancel)?;
                let samples: Vec<Supported> = decode(&row.get::<_, Vec<u8>>(0)?)?;
                let selected: Vec<_> = samples
                    .into_iter()
                    .filter(|s| s.end_ns > interval.start_ns && s.sample.time_ns < interval.end_ns)
                    .map(|s| s.sample)
                    .collect();
                sink.samples(&selected)?;
            }
            sink.finish()?;
            persist(temp, path)
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
        db.execute_batch(&format!("PRAGMA application_id={APPLICATION}; PRAGMA user_version=1;
            PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL;
            CREATE TABLE metadata(key TEXT PRIMARY KEY,value BLOB NOT NULL);
            CREATE TABLE blocks(n INTEGER PRIMARY KEY,start_ns INTEGER,end_ns INTEGER,data BLOB NOT NULL);
            CREATE INDEX blocks_time ON blocks(start_ns,end_ns);
            CREATE TABLE nodes(level INTEGER,n INTEGER,data BLOB NOT NULL,PRIMARY KEY(level,n)) WITHOUT ROWID;
            CREATE TABLE messages(n INTEGER PRIMARY KEY,data BLOB NOT NULL);"))?;
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
        };
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
                self.period = 1_000_000_000u64.div_ceil(u64::from(header.rate));
                self.header = Some(header.clone());
                self.converter = Some(Converter::new(header));
                self.next = Some(header.first_sequence);
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
            Message::Edges(_) => {}
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
                self.db.execute_batch("COMMIT")?;
                Ok(())
            }
            Err(error) => {
                let _ = self.db.execute_batch("ROLLBACK");
                Err(error)
            }
        }
    }
    fn flush_inner(&mut self) -> Result<()> {
        for message in &self.messages {
            self.db.execute(
                "INSERT INTO messages VALUES(?,?)",
                params![self.message_n as i64, encode(message)?],
            )?;
            self.message_n += 1;
        }
        self.messages.clear();
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
            if self.blocks == 0 {
                self.extent.start_ns = start;
            }
            self.extent.end_ns = end;
            self.blocks += 1;
            self.samples.clear();
        }
        self.metadata()
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
        stream::{End, EndReason, Pair, Pairs},
    };
    use std::sync::atomic::AtomicBool;

    fn header(rate: u32) -> Header {
        Header {
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
        assert!(csv.contains("1,1000000,3000,1000000\n"));
        assert!(!csv.contains("3,3000000"));
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
