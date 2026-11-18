//! A recording file written from a stream's messages, on a thread of its own: a tickwave
//! recording, a sigrok session or CSV.

use std::fs::File;
use std::path::Path;
use std::sync::mpsc;
use std::thread::{self, JoinHandle};

use super::{Context, RecordError, Result, Sink};
use crate::stream::{Converter, Message, Sample, skipped};

/// What a recording writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Format {
    /// A tickwave recording, which tickwave opens. Needs the `tickwave` feature.
    Tickwave,
    /// A sigrok session, which PulseView opens.
    Sigrok,
    Csv,
}

impl Format {
    /// The format as a sentence names it: "a tickwave recording", "a sigrok session", "CSV".
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Format::Tickwave => "a tickwave recording",
            Format::Sigrok => "a sigrok session",
            Format::Csv => "CSV",
        }
    }

    /// The file extension that names the format, without its dot.
    #[must_use]
    pub fn extension(self) -> &'static str {
        match self {
            Format::Tickwave => "tickwave",
            Format::Sigrok => "sr",
            Format::Csv => "csv",
        }
    }

    /// The format `path`'s extension names, in any case, where it names one.
    #[must_use]
    pub fn named_by(path: &Path) -> Option<Format> {
        let extension = path.extension()?.to_str()?.to_ascii_lowercase();
        [Format::Tickwave, Format::Sigrok, Format::Csv]
            .into_iter()
            .find(|format| format.extension() == extension)
    }
}

/// A recording file, written from a stream's messages in the order they came, on a thread of its
/// own, so that reading the stream never waits on the disk. Its queue has no bound: a disk that
/// falls behind costs memory rather than the recording.
///
/// It takes every message of one stream, from its header to its end, as [`crate::stream::Reader`]
/// gives them. A sigrok session or CSV records an energy stream; a tickwave recording takes a DIO
/// stream too.
pub struct Writer {
    inner: Inner,
}

enum Inner {
    Sink(SinkWriter),
    #[cfg(feature = "tickwave")]
    Tickwave(crate::tickwave::Recorder),
}

impl Writer {
    /// Starts a recording at `path` in `format`. A tickwave recording names where it came from with
    /// `context`, and is written as it goes, so a crash loses little; [`Writer::finish`] rewrites it
    /// with full-size blocks. A sigrok session or CSV records markers too.
    pub fn create(path: &Path, format: Format, context: Context) -> Result<Writer> {
        let inner = match format {
            #[cfg(feature = "tickwave")]
            Format::Tickwave => {
                Inner::Tickwave(crate::tickwave::Recorder::create_unbounded(path, context)?)
            }
            #[cfg(not(feature = "tickwave"))]
            Format::Tickwave => {
                let _ = context;
                return Err(RecordError::NoTickwave);
            }
            Format::Sigrok | Format::Csv => Inner::Sink(SinkWriter::new(
                Sink::create(path, format == Format::Csv)?,
                true,
            )?),
        };
        Ok(Writer { inner })
    }

    /// CSV to `file`, which need not be a regular file: stdout, say. It records no markers, which
    /// are merged into a file by rewriting it. `path` names it in errors.
    pub fn csv(file: File, path: &Path) -> Result<Writer> {
        Ok(Writer {
            inner: Inner::Sink(SinkWriter::new(Sink::from_file(file, path, true)?, false)?),
        })
    }

    /// The stream's next message. Fails once the file has stopped on an error, which
    /// [`Writer::finish`] gives.
    pub fn send(&mut self, message: Message) -> Result<()> {
        match &mut self.inner {
            Inner::Sink(sink) => sink.send(message),
            #[cfg(feature = "tickwave")]
            Inner::Tickwave(recorder) => Ok(recorder.send(message)?),
        }
    }

    /// Waits for everything sent to be written, and the file finished.
    pub fn finish(self) -> Result<()> {
        match self.inner {
            Inner::Sink(sink) => sink.finish(),
            #[cfg(feature = "tickwave")]
            Inner::Tickwave(recorder) => Ok(recorder.finish()?),
        }
    }
}

/// A sigrok session or CSV, written from an energy stream's messages on a thread of its own.
/// Writing a sigrok chunk takes long enough that the probe's messages would fill meanwhile, so its
/// queue has no bound.
pub struct SinkWriter {
    send: Option<mpsc::Sender<Message>>,
    thread: Option<JoinHandle<Result<()>>>,
}

impl SinkWriter {
    /// Writes into `sink`, recording markers where `markers` says to and the sink can take them.
    pub fn new(sink: Sink, markers: bool) -> Result<SinkWriter> {
        let (send, receive) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("sink writer".into())
            .spawn(move || write(sink, markers, &receive))?;
        Ok(SinkWriter {
            send: Some(send),
            thread: Some(thread),
        })
    }

    /// The stream's next message. Fails once the writer has stopped on an error, which
    /// [`SinkWriter::finish`] gives.
    pub fn send(&mut self, message: Message) -> Result<()> {
        self.send
            .as_ref()
            .and_then(|send| send.send(message).ok())
            .ok_or(RecordError::Stopped)
    }

    /// Waits for everything sent to be written, and the file finished.
    pub fn finish(mut self) -> Result<()> {
        self.join()
    }

    fn join(&mut self) -> Result<()> {
        drop(self.send.take());
        match self.thread.take() {
            Some(thread) => thread.join().map_err(|_| RecordError::Panicked)?,
            None => Ok(()),
        }
    }
}

impl Drop for SinkWriter {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            tracing::warn!("finishing the recording: {error}");
        }
    }
}

/// The writer's thread: converts the pairs and writes them, each message's held back until the
/// next comes, as every edge up to them has by then.
fn write(mut sink: Sink, markers: bool, receive: &mpsc::Receiver<Message>) -> Result<()> {
    let mut converter: Option<Converter> = None;
    let mut next_sequence = None;
    let mut held: Vec<Sample> = Vec::new();
    let mut dropped_markers = false;
    for message in receive {
        match message {
            Message::Header(header) => {
                if converter.is_some() {
                    return Err(RecordError::SecondStream);
                }
                if markers {
                    sink.include_markers()?;
                }
                sink.begin_with_dio_gaps(header.whole_rate(), header.dio_mask, header.dio_levels)?;
                next_sequence = Some(header.first_sequence);
                converter = Some(Converter::new(&header));
            }
            Message::DioHeader(_) => return Err(RecordError::DioOnly),
            Message::Pairs(pairs) => {
                let Some(converter) = converter.as_mut() else {
                    continue;
                };
                if let Some(next) = next_sequence
                    && skipped(next, pairs.first_sequence) > 0
                    && let Some(pair) = pairs.pairs.first()
                {
                    converter.discontinuity(pair.low);
                }
                next_sequence = Some(pairs.first_sequence.wrapping_add(pairs.pairs.len() as u32));
                converter.add_message(&pairs);
                sink.samples(&std::mem::replace(&mut held, converter.take_samples()))?;
            }
            _ if converter.is_none() => {}
            Message::Edges(edges) => sink.edges(edges.edges),
            Message::DioLost(lost) => sink.dio_lost(&lost),
            Message::Markers(message) => {
                if !sink.markers(&message)? && !dropped_markers {
                    dropped_markers = true;
                    tracing::warn!("markers are not recorded: the output is not a regular file");
                }
            }
            Message::End(_) => break,
            Message::DioTime(_) => {}
        }
    }
    // At the end, or wherever the stream stopped: a pair still waiting keeps its own current.
    if let Some(converter) = converter.as_mut() {
        converter.end();
        held.extend(converter.take_samples());
    }
    sink.samples(&held)?;
    sink.finish()
}
