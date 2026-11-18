use super::{Context, Error, FileWriter, Result, Snapshots, compact};
use crate::stream::Message;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender, TrySendError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// A [`FileWriter`] on a thread of its own, so reading the stream never waits on the disk. A full
/// queue fails the recording rather than drop a message.
pub struct Recorder {
    path: PathBuf,
    snapshots: Snapshots,
    sender: Option<Queue>,
    thread: Option<JoinHandle<Result<bool>>>,
    /// Whether the writer left a file, once its thread has been joined.
    written: bool,
}

/// The way to the writer's thread: a queue of 256 messages, or one without a bound.
enum Queue {
    Bounded(SyncSender<Message>),
    Unbounded(Sender<Message>),
}

impl Recorder {
    /// Starts a recording at `path`, as [`FileWriter::create`] does.
    pub fn create(path: &Path, context: Context) -> Result<Self> {
        let (sender, receiver) = mpsc::sync_channel(256);
        Self::spawn(path, context, Queue::Bounded(sender), receiver)
    }

    /// [`Recorder::create`] with a queue that has no bound: a disk that falls behind costs memory
    /// rather than the recording.
    pub(crate) fn create_unbounded(path: &Path, context: Context) -> Result<Self> {
        let (sender, receiver) = mpsc::channel();
        Self::spawn(path, context, Queue::Unbounded(sender), receiver)
    }

    fn spawn(
        path: &Path,
        context: Context,
        sender: Queue,
        receiver: Receiver<Message>,
    ) -> Result<Self> {
        let mut writer = FileWriter::create(path, context)?;
        let snapshots = writer.snapshots();
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

    /// The stream's next message. Fails when the writer has fallen 256 messages behind, and once it
    /// has stopped on an error, which [`Recorder::finish`] gives.
    pub fn send(&mut self, message: Message) -> Result<()> {
        let sent = match self.sender.as_ref().ok_or(Error::Closed)? {
            Queue::Bounded(sender) => sender.try_send(message),
            Queue::Unbounded(sender) => sender
                .send(message)
                .map_err(|error| TrySendError::Disconnected(error.0)),
        };
        match sent {
            Ok(()) => Ok(()),
            Err(TrySendError::Full(_)) => Err(Error::Behind),
            // The writer ended: its own error says why.
            Err(TrySendError::Disconnected(_)) => self.check().and(Err(Error::Stopped)),
        }
    }

    /// The writer's error once it has stopped early; `Stopped` if it stopped without one, as it
    /// does once the stream has ended.
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

    /// Tells the writer nothing more comes. It writes the recording's end and syncs the file on its
    /// own thread, which can take seconds on a busy disk; [`Recorder::is_finished`] says when
    /// [`Recorder::close`] no longer waits for it.
    pub fn end(&mut self) {
        self.sender.take();
    }

    /// Whether the writer's thread has stopped: after the stream's End message, after
    /// [`Recorder::end`], or on an error.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(JoinHandle::is_finished)
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
