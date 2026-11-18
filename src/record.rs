//! Recording files: CSV and sigrok sessions from converted pairs, and [`Writer`], which writes a
//! stream's messages as either, or as a tickwave recording.

mod markers;
mod writer;
use crate::stream::{CurrentRange, DioLost, Edge, Markers, Sample};
pub use markers::label as marker_label;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
pub use writer::{Format, SinkWriter, Writer};
use zip::ZipWriter;
use zip::result::ZipError;
use zip::write::SimpleFileOptions;

/// What a recording says about where it came from, beyond the stream.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Context {
    /// The probe's model, as "MCU-Link Pro".
    pub instrument: Option<String>,
    /// The firmware's version.
    pub software: Option<String>,
    /// The host's clock as the recording started, RFC 3339 in UTC.
    pub started_utc: Option<String>,
}

/// A temporary file beside `path`, named after it (`<name>.<random>.partial`), with the
/// permissions a new file there gets rather than tempfile's owner-only ones. Only a drop removes
/// it, so a process that exits while it is open leaves it behind.
pub(crate) fn temp_beside(path: &Path) -> io::Result<tempfile::NamedTempFile> {
    let mut prefix = path.file_name().unwrap_or_default().to_owned();
    prefix.push(".");
    let mut builder = tempfile::Builder::new();
    builder.prefix(&prefix).suffix(".partial");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        builder.permissions(std::fs::Permissions::from_mode(0o666));
    }
    let dir = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    builder.tempfile_in(dir)
}

/// Why a recording file could not be written.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RecordError {
    #[error("creating {}", path.display())]
    Create {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error(transparent)]
    Zip(#[from] ZipError),
    #[error(transparent)]
    Csv(#[from] csv::Error),
    #[error("marker times went back")]
    MarkerTime,
    #[error("cancelled")]
    Cancelled,
    #[error("merging the markers into {}; the file keeps its pairs, without markers", path.display())]
    Merge {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("{} is no longer the file recorded to; that file keeps its pairs, without markers", path.display())]
    Replaced { path: PathBuf },
    /// A tickwave recording's error, boxed: an export's can hold a [`RecordError`] in turn.
    #[cfg(feature = "tickwave")]
    #[error(transparent)]
    Tickwave(Box<crate::tickwave::Error>),
    #[error("writing a tickwave recording needs verge-probe's tickwave feature")]
    NoTickwave,
    #[error("the file's writer has stopped; finishing the file gives the reason")]
    Stopped,
    #[error("the file's writer panicked")]
    Panicked,
    #[error("the stream began again before it ended")]
    SecondStream,
    #[error("a sigrok session or CSV records an energy stream, and this is a DIO stream alone")]
    DioOnly,
}

#[cfg(feature = "tickwave")]
impl From<crate::tickwave::Error> for RecordError {
    fn from(error: crate::tickwave::Error) -> Self {
        RecordError::Tickwave(Box::new(error))
    }
}

impl RecordError {
    /// Whether writing failed because the reader of a pipe the recording went to has gone.
    #[must_use]
    pub fn is_broken_pipe(&self) -> bool {
        let io = match self {
            RecordError::Io(io) | RecordError::Create { source: io, .. } => Some(io),
            RecordError::Csv(error) => match error.kind() {
                csv::ErrorKind::Io(io) => Some(io),
                _ => None,
            },
            _ => None,
        };
        io.is_some_and(|io| io.kind() == io::ErrorKind::BrokenPipe)
    }
}

type Result<T> = std::result::Result<T, RecordError>;

/// The levels of the DIO pins a stream reports, as its edges move them.
#[derive(Default)]
struct Dio {
    pins: u8,
    levels: u8,
    edges: VecDeque<DioEvent>,
    unknown: bool,
    gaps: bool,
}

#[derive(Clone, Copy)]
struct DioEvent {
    time_ns: u64,
    levels: Option<u8>,
}

impl Dio {
    /// The levels at `time_ns`, taking the edges up to it. A pin that moved and came back since
    /// the last call reads the other way, so a pulse between two samples shows in one.
    fn at(&mut self, time_ns: u64) -> (u8, bool) {
        let before = self.levels;
        let was_unknown = self.unknown;
        let mut uncertain = self.unknown;
        let mut moved = 0;
        while let Some(edge) = self.next_until(time_ns) {
            if let Some(levels) = edge.levels {
                if !self.unknown {
                    moved |= self.levels ^ levels;
                }
                self.levels = levels;
            }
            self.unknown = edge.levels.is_none();
            uncertain |= self.unknown;
        }
        let back = if was_unknown || self.unknown {
            0
        } else {
            moved & !(before ^ self.levels)
        };
        ((self.levels ^ back) & self.pins, uncertain)
    }

    /// The next edge, if it comes by `time_ns`.
    fn next_until(&mut self, time_ns: u64) -> Option<DioEvent> {
        self.edges.pop_front_if(|edge| edge.time_ns <= time_ns)
    }

    /// The pins, lowest first.
    fn names(&self) -> impl Iterator<Item = u8> + '_ {
        (0..4).filter(|pin| self.pins >> pin & 1 != 0)
    }

    /// One CSV field per pin: its level in `levels`.
    fn fields(&self, levels: u8) -> String {
        self.names()
            .map(|pin| format!(",{}", levels >> pin & 1))
            .collect()
    }
}

/// Where the pairs go.
pub enum Sink {
    Csv(Box<Csv>),
    Sigrok(Box<Sigrok>),
}

impl Sink {
    /// Writes to `path`, emptying a file that is there: CSV where `csv` says so, else a sigrok
    /// session.
    pub fn create(path: &Path, csv: bool) -> Result<Sink> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)
            .map_err(|source| RecordError::Create {
                path: path.to_owned(),
                source,
            })?;
        Self::from_file(file, path, csv)
    }

    /// [`Sink::create`], refusing a file that is there already.
    pub fn create_new(path: &Path, csv: bool) -> Result<Sink> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|source| RecordError::Create {
                path: path.to_owned(),
                source,
            })?;
        Self::from_file(file, path, csv)
    }

    /// `file` is open for reading and writing at `path`. A CSV whose markers are merged at finish
    /// is replaced there by a merged copy.
    pub fn from_file(file: File, path: &Path, csv: bool) -> Result<Sink> {
        let file = BufWriter::new(file);
        Ok(if csv {
            Sink::Csv(Box::new(Csv {
                file,
                path: path.to_owned(),
                dio: Dio::default(),
                markers: None,
            }))
        } else {
            Sink::Sigrok(Box::new(Sigrok::new(file)?))
        })
    }

    /// Records markers: an embedded markers.csv in sigrok sessions, and in CSV, marker columns and
    /// rows merged in by time at finish. A CSV that is not a regular file, such as a pipe, cannot
    /// be merged, and [`Sink::markers`] drops its markers. Call before begin.
    pub fn include_markers(&mut self) -> Result<()> {
        match self {
            Self::Csv(csv) if csv.markers.is_none() => {
                if csv.file.get_ref().metadata()?.is_file() {
                    csv.markers = Some(markers::CsvMarkers {
                        spool: markers::Spool::new()?,
                    });
                }
            }
            Self::Sigrok(sigrok) if sigrok.markers.is_none() => {
                sigrok.markers = Some(markers::Spool::new()?);
            }
            _ => {}
        }
        Ok(())
    }
    /// Whether the markers were recorded: false without [`Sink::include_markers`], or for a CSV
    /// that cannot take them.
    pub fn markers(&mut self, message: &Markers) -> Result<bool> {
        let spool = match self {
            Self::Csv(csv) => csv.markers.as_mut().map(|markers| &mut markers.spool),
            Self::Sigrok(sigrok) => sigrok.markers.as_mut(),
        };
        let Some(spool) = spool else {
            return Ok(false);
        };
        spool.append(message)?;
        Ok(true)
    }

    /// A marker read back from a recording, its label the text [`marker_label`] made of it.
    /// Whether it was recorded, as [`Sink::markers`].
    #[cfg(feature = "tickwave")]
    pub(crate) fn marker_text(
        &mut self,
        time_ns: u64,
        number: u32,
        label: &str,
        after_loss: bool,
    ) -> Result<bool> {
        let Some(spool) = self.spool() else {
            return Ok(false);
        };
        spool.append_text(time_ns, number, label, after_loss)?;
        Ok(true)
    }
    /// A marker loss report that came with no marker, read back from a recording.
    #[cfg(feature = "tickwave")]
    pub(crate) fn marker_loss(&mut self) -> bool {
        let Some(spool) = self.spool() else {
            return false;
        };
        spool.untimed_loss();
        true
    }
    #[cfg(feature = "tickwave")]
    fn spool(&mut self) -> Option<&mut markers::Spool> {
        match self {
            Self::Csv(csv) => csv.markers.as_mut().map(|markers| &mut markers.spool),
            Self::Sigrok(sigrok) => sigrok.markers.as_mut(),
        }
    }

    /// The stream began at `rate` pairs a second, reporting the edges of `pins`, at `levels`.
    pub fn begin(&mut self, rate: u32, pins: u8, levels: u8) -> Result<()> {
        self.begin_mode(rate, pins, levels, false)
    }

    /// Includes a DIO_GAP channel (dio_gap in CSV) for intervals of unknown levels.
    pub fn begin_with_dio_gaps(&mut self, rate: u32, pins: u8, levels: u8) -> Result<()> {
        self.begin_mode(rate, pins, levels, true)
    }

    fn begin_mode(&mut self, rate: u32, pins: u8, levels: u8, gaps: bool) -> Result<()> {
        let dio = Dio {
            pins,
            levels: levels & pins,
            edges: VecDeque::new(),
            unknown: false,
            gaps,
        };
        match self {
            Sink::Csv(csv) => {
                let mut columns: String = dio.names().map(|pin| format!(",dio{pin}")).collect();
                if pins != 0 && gaps {
                    columns.push_str(",dio_gap");
                }
                csv.row(format_args!(
                    "sequence,time_ns,millivolts,nanoamps,range{columns}"
                ))?;
                csv.dio = dio;
            }
            Sink::Sigrok(sigrok) => {
                sigrok.rate = rate;
                sigrok.dio = dio;
            }
        }
        Ok(())
    }

    /// DIO edges, in time order. Each lands among the samples by its time, so send the edges up to
    /// a batch of samples before it.
    pub fn edges(&mut self, edges: Vec<Edge>) {
        for edge in edges {
            self.dio_state(edge.time_ns, Some(edge.levels));
        }
    }

    /// Sets the state preceding the exported events without creating a transition.
    pub fn seed_dio(&mut self, levels: Option<u8>) {
        let dio = match self {
            Sink::Csv(csv) => &mut csv.dio,
            Sink::Sigrok(sigrok) => &mut sigrok.dio,
        };
        dio.unknown = levels.is_none();
        dio.levels = levels.unwrap_or(0) & dio.pins;
    }

    /// A known level, or the start of an interval whose edges were lost.
    pub fn dio_state(&mut self, time_ns: u64, levels: Option<u8>) {
        let dio = match self {
            Sink::Csv(csv) => &mut csv.dio,
            Sink::Sigrok(sigrok) => &mut sigrok.dio,
        };
        dio.edges.push_back(DioEvent { time_ns, levels });
    }

    /// The probe lost edges: the levels are unknown from `lost.from_ns` and known again at
    /// `lost.to_ns`.
    pub fn dio_lost(&mut self, lost: &DioLost) {
        self.dio_state(lost.from_ns, None);
        self.dio_state(lost.to_ns, Some(lost.levels));
    }

    /// Flush events and missing pairs before `time_ns` without buffering the interval.
    pub fn advance_to(&mut self, time_ns: u64) -> Result<()> {
        match self {
            Sink::Csv(csv) => csv.before(time_ns.saturating_sub(1)),
            Sink::Sigrok(sigrok) => sigrok.before(time_ns),
        }
    }

    /// Starts a missing analog interval without treating its placeholder as a measurement.
    pub fn gap_sample(&mut self, sample: Sample) -> Result<()> {
        match self {
            Sink::Sigrok(sigrok) => {
                sigrok.put(&sample, true, sample.time_ns)?;
                sigrok.last = Some(sample);
                sigrok.advanced = 1;
            }
            Sink::Csv(csv) => {
                csv.before(sample.time_ns)?;
                csv.edge(DioEvent {
                    time_ns: sample.time_ns,
                    levels: (!csv.dio.unknown).then_some(csv.dio.levels),
                })?;
            }
        }
        Ok(())
    }

    /// Converted pairs, in sequence order. A gap in their times is written as a gap.
    pub fn samples(&mut self, samples: &[Sample]) -> Result<()> {
        match self {
            Sink::Csv(csv) => {
                for sample in samples {
                    csv.sample(sample)?;
                }
            }
            Sink::Sigrok(sigrok) => {
                for sample in samples {
                    sigrok.push(sample)?;
                }
            }
        }
        Ok(())
    }

    /// Writes what is left, and the markers, and closes the file.
    pub fn finish(self) -> Result<()> {
        self.finish_with_cancel(&AtomicBool::new(false))
    }

    /// [`Sink::finish`], stopping with [`RecordError::Cancelled`] once `cancel` is set.
    pub fn finish_with_cancel(self, cancel: &AtomicBool) -> Result<()> {
        check_cancel(cancel)?;
        match self {
            Sink::Csv(csv) => csv.finish(cancel)?,
            Sink::Sigrok(sigrok) => sigrok.finish(cancel)?,
        }
        Ok(())
    }
}

fn check_cancel(cancel: &AtomicBool) -> Result<()> {
    if cancel.load(Ordering::Relaxed) {
        return Err(RecordError::Cancelled);
    }
    Ok(())
}

/// One line per pair, and one per edge between them, in time order: an edge's line has no
/// sequence, millivolts, nanoamps or range, and gives the levels after it.
pub struct Csv {
    file: BufWriter<File>,
    path: PathBuf,
    dio: Dio,
    markers: Option<markers::CsvMarkers>,
}

impl Csv {
    fn row(&mut self, row: std::fmt::Arguments<'_>) -> Result<()> {
        writeln!(self.file, "{row}")?;
        Ok(())
    }
    fn fields(&self) -> String {
        if self.dio.pins == 0 {
            return String::new();
        }
        if !self.dio.gaps {
            return self.dio.fields(self.dio.levels);
        }
        if self.dio.unknown {
            format!("{},1", ",".repeat(self.dio.names().count()))
        } else {
            format!("{},0", self.dio.fields(self.dio.levels))
        }
    }
    fn edge(&mut self, edge: DioEvent) -> Result<()> {
        self.dio.unknown = edge.levels.is_none();
        if let Some(levels) = edge.levels {
            self.dio.levels = levels;
        }
        let fields = self.fields();
        self.row(format_args!(",{},,,{fields}", edge.time_ns))?;
        Ok(())
    }

    fn before(&mut self, time_ns: u64) -> Result<()> {
        while let Some(edge) = self.dio.next_until(time_ns) {
            self.edge(edge)?;
        }
        Ok(())
    }
    fn sample(&mut self, sample: &Sample) -> Result<()> {
        self.before(sample.time_ns)?;
        let fields = self.fields();
        let range = match sample.range {
            CurrentRange::Low => "low",
            CurrentRange::High if sample.under_range => "under",
            CurrentRange::High => "high",
        };
        self.row(format_args!(
            "{},{},{},{},{range}{fields}",
            sample.sequence, sample.time_ns, sample.millivolts, sample.nanoamps
        ))?;
        Ok(())
    }

    fn finish(mut self, cancel: &AtomicBool) -> Result<()> {
        while let Some(edge) = self.dio.edges.pop_front() {
            self.edge(edge)?;
        }
        if let Some(markers) = self.markers {
            markers.finish(self.file, &self.path, &|| check_cancel(cancel))?;
        } else {
            self.file.flush()?;
        }
        Ok(())
    }
}

/// A sigrok session file (srzip, format 2), as libsigrok's own srzip output writes it: the
/// voltage in V and the current in mA as analog channels; a logic channel GAP that is high where
/// the probe measured nothing, across which the last pair is repeated; a logic channel for each
/// DIO pin reported, its level at each sample, with a pulse between two samples stretched to one;
/// and last, UNDER, high where the pair was read on the high range too near its floor to trust.
pub struct Sigrok {
    zip: ZipWriter<BufWriter<File>>,
    rate: u32,
    dio: Dio,
    last: Option<Sample>,
    volts: Vec<f32>,
    milliamps: Vec<f32>,
    logic: Vec<u8>,
    chunks: u32,
    advanced: u64,
    markers: Option<markers::Spool>,
    origin: Option<u64>,
}

/// Samples a chunk file holds: 4 MiB of each analog channel.
const CHUNK: usize = 1 << 20;
const GAP: u8 = 1;

impl Sigrok {
    fn new(file: BufWriter<File>) -> Result<Sigrok> {
        let mut zip = ZipWriter::new(file);
        zip.start_file("version", options())?;
        zip.write_all(b"2")?;
        Ok(Sigrok {
            zip,
            rate: 0,
            dio: Dio::default(),
            last: None,
            volts: Vec::new(),
            milliamps: Vec::new(),
            logic: Vec::new(),
            chunks: 0,
            advanced: 0,
            markers: None,
            origin: None,
        })
    }

    /// `sample` at `time_ns`: itself, or the last pair repeated across a gap.
    fn put(&mut self, sample: &Sample, gap: bool, time_ns: u64) -> Result<()> {
        let first = self.origin.is_none();
        self.origin.get_or_insert(time_ns);
        self.volts.push(sample.millivolts as f32 / 1e3);
        self.milliamps.push(sample.nanoamps as f32 / 1e6);
        // GAP is bit 0, then the pins in order.
        let (levels, unknown) = self.dio.at(time_ns);
        // The first sample has no sample before it in the file, so it reads the pins at its own
        // instant, as the CSV's first row does, not the export's seed before it.
        let unknown = if first { self.dio.unknown } else { unknown };
        let dio = self
            .dio
            .names()
            .enumerate()
            .fold(0, |logic, (channel, pin)| {
                logic | (levels >> pin & 1) << (channel + 1)
            });
        let dio_gap = if unknown && self.dio.pins != 0 && self.dio.gaps {
            1 << (self.logic_channels() - 2)
        } else {
            0
        };
        let under = u8::from(sample.under_range) << (self.logic_channels() - 1);
        self.logic
            .push(if gap { GAP } else { 0 } | dio | dio_gap | under);
        if self.logic.len() == CHUNK {
            self.flush()?;
        }
        Ok(())
    }

    fn before(&mut self, time_ns: u64) -> Result<()> {
        if let Some(last) = self.last {
            let rate = u128::from(self.rate.max(1));
            let elapsed = u128::from(time_ns.saturating_sub(last.time_ns));
            let periods = (elapsed * rate + 500_000_000) / 1_000_000_000;
            for period in u128::from(self.advanced.max(1))..periods {
                let at = last.time_ns + (period * 1_000_000_000 / rate) as u64;
                self.put(&last, true, at)?;
            }
            self.advanced = self.advanced.max(periods as u64);
        }
        Ok(())
    }
    fn push(&mut self, sample: &Sample) -> Result<()> {
        self.before(sample.time_ns)?;
        self.put(sample, false, sample.time_ns)?;
        self.last = Some(*sample);
        self.advanced = 1;
        Ok(())
    }

    fn flush(&mut self) -> Result<()> {
        if self.logic.is_empty() {
            return Ok(());
        }
        self.chunks += 1;
        let chunk = self.chunks;
        self.zip.start_file(format!("logic-1-{chunk}"), options())?;
        self.zip.write_all(&self.logic)?;
        // Analog channels number on from the logic ones, as libsigrok numbers them.
        let logic = self.logic_channels();
        for (index, values) in [(logic + 1, &self.volts), (logic + 2, &self.milliamps)] {
            self.zip
                .start_file(format!("analog-1-{index}-{chunk}"), options())?;
            let bytes: Vec<u8> = values
                .iter()
                .flat_map(|value| value.to_le_bytes())
                .collect();
            self.zip.write_all(&bytes)?;
        }
        self.volts.clear();
        self.milliamps.clear();
        self.logic.clear();
        Ok(())
    }

    /// GAP, the pins, DIO_GAP where there is one, and UNDER.
    fn logic_channels(&self) -> usize {
        1 + self.dio.names().count() + usize::from(self.dio.pins != 0 && self.dio.gaps) + 1
    }

    fn finish(mut self, cancel: &AtomicBool) -> Result<()> {
        self.flush()?;
        // libsigrok reads the keys in order: the totals make the channels the names then rename.
        let logic = self.logic_channels();
        let mut metadata = format!(
            "[global]\nsigrok version=0.5.2\n\n[device 1]\ncapturefile=logic-1\ntotal probes={logic}\n\
             samplerate={}\ntotal analog=2\n",
            samplerate(self.rate)
        );
        let names = std::iter::once("GAP".to_owned())
            .chain(self.dio.names().map(|pin| format!("DIO{pin}")))
            .chain((self.dio.pins != 0 && self.dio.gaps).then(|| "DIO_GAP".to_owned()))
            .chain(std::iter::once("UNDER".to_owned()));
        for (index, name) in names.enumerate() {
            metadata.push_str(&format!("probe{}={name}\n", index + 1));
        }
        for (index, name) in [(logic + 1, "V"), (logic + 2, "mA")] {
            metadata.push_str(&format!("analog{index}={name}\n"));
        }
        metadata.push_str("unitsize=1\n");
        if let Some(markers) = self.markers.take() {
            let origin = self.origin.or(markers.first).unwrap_or(0);
            metadata.push_str(&format!("verge capture origin ns={origin}\n"));
            self.zip.start_file("markers.csv", options())?;
            let mut output = csv::Writer::from_writer(&mut self.zip);
            output.write_record([
                "time_ns",
                "marker_number",
                "marker_label",
                "marker_lost",
                "session_time_ns",
            ])?;
            for row in markers.reader()?.records() {
                check_cancel(cancel)?;
                let mut row = row?;
                let relative = if row[0].is_empty() {
                    String::new()
                } else {
                    (i128::from(row[0].parse::<u64>().map_err(|_| RecordError::MarkerTime)?)
                        - i128::from(origin))
                    .to_string()
                };
                row.push_field(&relative);
                output.write_record(&row)?;
            }
            output.flush()?;
        }
        check_cancel(cancel)?;
        self.zip.start_file("metadata", options())?;
        self.zip.write_all(metadata.as_bytes())?;
        self.zip.finish()?.flush()?;
        Ok(())
    }
}

fn options() -> SimpleFileOptions {
    SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated)
}

/// A rate as libsigrok's `sr_samplerate_string` writes it, which its parser reads back.
pub fn samplerate(rate: u32) -> String {
    match rate {
        0 => "0 Hz".to_owned(),
        r if r % 1_000_000 == 0 => format!("{} MHz", r / 1_000_000),
        r if r % 1_000 == 0 => format!("{} kHz", r / 1_000),
        r => format!("{r} Hz"),
    }
}
