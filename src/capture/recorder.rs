use super::{CaptureError, CaptureWriter, Result};
use crate::stream::Message;
use crate::tickwave::{Context, FileWriter, Snapshots};
use std::path::Path;
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// USB submits without waiting for disk. A full queue is a recording failure, never silent loss.
pub struct Recorder {
    sender: Option<SyncSender<Message>>,
    thread: Option<JoinHandle<Result<()>>>,
}

/// Where a recorder's messages go.
trait Disk: Send + 'static {
    fn ingest(&mut self, message: Message) -> Result<bool>;
    fn flush(&mut self) -> Result<()>;
    fn finish(self) -> Result<()>;
}

impl Disk for CaptureWriter {
    fn ingest(&mut self, message: Message) -> Result<bool> {
        CaptureWriter::ingest(self, message)
    }
    fn flush(&mut self) -> Result<()> {
        CaptureWriter::flush(self)
    }
    fn finish(self) -> Result<()> {
        CaptureWriter::finish(self)
    }
}

impl Disk for FileWriter {
    fn ingest(&mut self, message: Message) -> Result<bool> {
        Ok(FileWriter::ingest(self, &message)?)
    }
    fn flush(&mut self) -> Result<()> {
        Ok(FileWriter::flush(self)?)
    }
    fn finish(self) -> Result<()> {
        FileWriter::close(self)?;
        Ok(())
    }
}

impl Recorder {
    /// A `.vergecapture` archive.
    pub fn create(path: &Path) -> Result<Self> {
        Self::spawn(CaptureWriter::create(path)?)
    }

    /// A `.tickwave` recording, and the latest snapshot of it for a view in this process. Finishing
    /// leaves it uncompacted: its owner compacts it with [`crate::tickwave::compacted`] around the
    /// view that reads it.
    pub fn tickwave(path: &Path, context: Context) -> Result<(Self, Snapshots)> {
        let writer = FileWriter::create(path, context)?;
        let snapshots = writer.snapshots();
        Ok((Self::spawn(writer)?, snapshots))
    }

    fn spawn(mut writer: impl Disk) -> Result<Self> {
        let (sender, receiver) = mpsc::sync_channel(256);
        let thread = thread::Builder::new()
            .name("capture-disk".into())
            .spawn(move || {
                let received = loop {
                    let step = match receiver.recv_timeout(Duration::from_millis(200)) {
                        Ok(message) => writer.ingest(message),
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
                let finished = writer.finish();
                received.and(finished)
            })?;
        Ok(Self {
            sender: Some(sender),
            thread: Some(thread),
        })
    }

    pub fn send(&mut self, message: Message) -> Result<()> {
        let sender = self.sender.as_ref().ok_or(CaptureError::Closed)?;
        match sender.try_send(message) {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => Err(CaptureError::Behind),
            // The writer ended: its own error says why.
            Err(TrySendError::Disconnected(_)) => self.check().and(Err(CaptureError::Stopped)),
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
            return Err(CaptureError::Stopped);
        }
        Ok(())
    }

    pub fn finish(mut self) -> Result<()> {
        self.join()
    }

    fn join(&mut self) -> Result<()> {
        self.sender.take();
        match self.thread.take() {
            Some(thread) => thread.join().map_err(|_| CaptureError::Panicked)?,
            None => Ok(()),
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            tracing::error!("closing capture: {error}");
        }
    }
}
