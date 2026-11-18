use super::{Context, Error, FileWriter, Result, Snapshots, compact};
use crate::stream::Message;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// A [`FileWriter`] on a thread of its own, so reading the stream never waits on the disk. A full
/// queue fails the recording rather than drop a message.
pub struct Recorder {
    path: PathBuf,
    snapshots: Snapshots,
    sender: Option<SyncSender<Message>>,
    thread: Option<JoinHandle<Result<bool>>>,
    /// Whether the writer left a file, once its thread has been joined.
    written: bool,
}

impl Recorder {
    /// Starts a recording at `path`, as [`FileWriter::create`] does.
    pub fn create(path: &Path, context: Context) -> Result<Self> {
        let mut writer = FileWriter::create(path, context)?;
        let snapshots = writer.snapshots();
        let (sender, receiver) = mpsc::sync_channel(256);
        let thread = thread::Builder::new()
            .name("recorder".into())
            .spawn(move || {
                let received = loop {
                    let step = match receiver.recv_timeout(Duration::from_millis(200)) {
                        Ok(message) => writer.ingest(&message),
                        Err(RecvTimeoutError::Timeout) => writer.flush().map(|()| false),
                        Err(RecvTimeoutError::Disconnected) => Ok(true),
                    };
                    match step {
                        Ok(false) => {}
                        Ok(true) => break Ok(()),
                        Err(error) => break Err(error),
                    }
                };
                // What arrived before an error is kept; the first error is the one reported.
                let closed = writer.close();
                received.and(closed)
            })?;
        Ok(Self {
            path: path.to_owned(),
            snapshots,
            sender: Some(sender),
            thread: Some(thread),
            written: false,
        })
    }

    /// The latest snapshot of what is in the file, for a view in this process to take.
    #[must_use]
    pub fn snapshots(&self) -> Snapshots {
        Arc::clone(&self.snapshots)
    }

    pub fn send(&mut self, message: Message) -> Result<()> {
        let sender = self.sender.as_ref().ok_or(Error::Closed)?;
        match sender.try_send(message) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => Err(Error::Behind),
            // The writer ended: its own error says why.
            Err(TrySendError::Disconnected(_)) => self.check().and(Err(Error::Stopped)),
        }
    }

    /// The writer's error once it has stopped early; `Stopped` if it stopped without one.
    pub fn check(&mut self) -> Result<()> {
        if self
            .thread
            .as_ref()
            .is_some_and(|thread| thread.is_finished())
        {
            self.join()?;
            return Err(Error::Stopped);
        }
        Ok(())
    }

    /// Ends the recording and compacts it, as [`FileWriter::finish`] does.
    pub fn finish(mut self) -> Result<()> {
        self.join()?;
        if self.written {
            compact(&self.path)?;
        }
        Ok(())
    }

    /// Ends the recording without compacting it, as [`FileWriter::close`] does, for a caller that
    /// compacts it with [`super::compacted`] around a view that reads it.
    pub fn close(mut self) -> Result<()> {
        self.join()
    }

    fn join(&mut self) -> Result<()> {
        self.sender.take();
        if let Some(thread) = self.thread.take() {
            self.written = thread.join().map_err(|_| Error::Panicked)??;
        }
        Ok(())
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            tracing::error!("closing the recording: {error}");
        }
    }
}
