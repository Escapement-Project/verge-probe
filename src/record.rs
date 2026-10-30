//! CSV and sigrok session writers shared by frontends.

use crate::stream::{Edge, Sample};
use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use zip::ZipWriter;
use zip::result::ZipError;
use zip::write::SimpleFileOptions;

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
}

type Result<T> = std::result::Result<T, RecordError>;

/// The levels of the DIO pins a stream reports, as its edges move them.
#[derive(Default)]
struct Dio {
    pins: u8,
    levels: u8,
    edges: VecDeque<Edge>,
}

impl Dio {
    /// The levels at `time_ns`, taking the edges up to it. A pin that moved and came back since
    /// the last call reads the other way, so a pulse between two samples shows in one.
    fn at(&mut self, time_ns: u64) -> u8 {
        let before = self.levels;
        let mut moved = 0;
        while let Some(edge) = self.next_until(time_ns) {
            moved |= self.levels ^ edge.levels;
            self.levels = edge.levels;
        }
        let back = moved & !(before ^ self.levels);
        (self.levels ^ back) & self.pins
    }

    /// The next edge, if it comes by `time_ns`.
    fn next_until(&mut self, time_ns: u64) -> Option<Edge> {
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
    pub fn create(path: &Path, csv: bool) -> Result<Sink> {
        let file = File::create(path).map_err(|source| RecordError::Create {
            path: path.to_owned(),
            source,
        })?;
        Self::from_file(file, csv)
    }

    pub fn create_new(path: &Path, csv: bool) -> Result<Sink> {
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|source| RecordError::Create {
                path: path.to_owned(),
                source,
            })?;
        Self::from_file(file, csv)
    }

    pub fn from_file(file: File, csv: bool) -> Result<Sink> {
        let file = BufWriter::new(file);
        Ok(if csv {
            Sink::Csv(Box::new(Csv {
                file,
                dio: Dio::default(),
            }))
        } else {
            Sink::Sigrok(Box::new(Sigrok::new(file)?))
        })
    }

    /// The stream began at `rate` pairs a second, reporting the edges of `pins`, at `levels`.
    pub fn begin(&mut self, rate: u32, pins: u8, levels: u8) -> Result<()> {
        let dio = Dio {
            pins,
            levels: levels & pins,
            edges: VecDeque::new(),
        };
        match self {
            Sink::Csv(csv) => {
                let columns: String = dio.names().map(|pin| format!(",dio{pin}")).collect();
                writeln!(csv.file, "sequence,time_ns,millivolts,nanoamps{columns}")?;
                csv.dio = dio;
            }
            Sink::Sigrok(sigrok) => {
                sigrok.rate = rate;
                sigrok.dio = dio;
            }
        }
        Ok(())
    }

    pub fn edges(&mut self, edges: Vec<Edge>) {
        match self {
            Sink::Csv(csv) => csv.dio.edges.extend(edges),
            Sink::Sigrok(sigrok) => sigrok.dio.edges.extend(edges),
        }
    }

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

    pub fn finish(self) -> Result<()> {
        match self {
            Sink::Csv(csv) => csv.finish()?,
            Sink::Sigrok(sigrok) => sigrok.finish()?,
        }
        Ok(())
    }
}

/// One line per pair, and one per edge between them, in time order: an edge's line has no
/// sequence, millivolts or nanoamps, and gives the levels after it.
pub struct Csv {
    file: BufWriter<File>,
    dio: Dio,
}

impl Csv {
    fn edge(&mut self, edge: Edge) -> Result<()> {
        let fields = self.dio.fields(edge.levels);
        writeln!(self.file, ",{},,{fields}", edge.time_ns)?;
        Ok(())
    }

    fn sample(&mut self, sample: &Sample) -> Result<()> {
        while let Some(edge) = self.dio.next_until(sample.time_ns) {
            self.dio.levels = edge.levels;
            self.edge(edge)?;
        }
        let fields = self.dio.fields(self.dio.levels);
        writeln!(
            self.file,
            "{},{},{},{}{fields}",
            sample.sequence, sample.time_ns, sample.millivolts, sample.nanoamps
        )?;
        Ok(())
    }

    fn finish(mut self) -> Result<()> {
        while let Some(edge) = self.dio.edges.pop_front() {
            self.edge(edge)?;
        }
        self.file.flush()?;
        Ok(())
    }
}

/// A sigrok session file (srzip, format 2), as libsigrok's own srzip output writes it: the
/// voltage in V and the current in mA as analog channels; a logic channel GAP that is high where
/// the probe measured nothing, across which the last pair is repeated; and a logic channel for each
/// DIO pin reported, its level at each sample, with a pulse between two samples stretched to one.
pub struct Sigrok {
    zip: ZipWriter<BufWriter<File>>,
    rate: u32,
    dio: Dio,
    last: Option<Sample>,
    volts: Vec<f32>,
    milliamps: Vec<f32>,
    logic: Vec<u8>,
    chunks: u32,
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
        })
    }

    /// `sample` at `time_ns`: itself, or the last pair repeated across a gap.
    fn put(&mut self, sample: &Sample, gap: bool, time_ns: u64) -> Result<()> {
        self.volts.push(sample.millivolts as f32 / 1e3);
        self.milliamps.push(sample.nanoamps as f32 / 1e6);
        // GAP is bit 0, then the pins in order.
        let levels = self.dio.at(time_ns);
        let dio = self
            .dio
            .names()
            .enumerate()
            .fold(0, |logic, (channel, pin)| {
                logic | (levels >> pin & 1) << (channel + 1)
            });
        self.logic.push(if gap { GAP } else { 0 } | dio);
        if self.logic.len() == CHUNK {
            self.flush()?;
        }
        Ok(())
    }

    fn push(&mut self, sample: &Sample) -> Result<()> {
        if let Some(last) = self.last {
            // Periods since the last pair, to the nearest: more than one is a pause, a hold or
            // pairs lost.
            let rate = u128::from(self.rate.max(1));
            let elapsed = u128::from(sample.time_ns.saturating_sub(last.time_ns));
            let periods = (elapsed * rate + 500_000_000) / 1_000_000_000;
            for period in 1..periods {
                let time_ns = last.time_ns + (period * 1_000_000_000 / rate) as u64;
                self.put(&last, true, time_ns)?;
            }
        }
        self.put(sample, false, sample.time_ns)?;
        self.last = Some(*sample);
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

    /// GAP and the pins.
    fn logic_channels(&self) -> usize {
        1 + self.dio.names().count()
    }

    fn finish(mut self) -> Result<()> {
        self.flush()?;
        // libsigrok reads the keys in order: the totals make the channels the names then rename.
        let logic = self.logic_channels();
        let mut metadata = format!(
            "[global]\nsigrok version=0.5.2\n\n[device 1]\ncapturefile=logic-1\ntotal probes={logic}\n\
             samplerate={}\ntotal analog=2\n",
            samplerate(self.rate)
        );
        let names = std::iter::once("GAP".to_owned())
            .chain(self.dio.names().map(|pin| format!("DIO{pin}")));
        for (index, name) in names.enumerate() {
            metadata.push_str(&format!("probe{}={name}\n", index + 1));
        }
        for (index, name) in [(logic + 1, "V"), (logic + 2, "mA")] {
            metadata.push_str(&format!("analog{index}={name}\n"));
        }
        metadata.push_str("unitsize=1\n");
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
