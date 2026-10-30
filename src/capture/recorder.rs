use super::{CaptureError, CaptureWriter, Result};
use crate::stream::Message;
use std::path::Path;
use std::sync::mpsc::{self, RecvTimeoutError, SyncSender, TrySendError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// USB submits without waiting for disk. A full queue is a recording failure, never silent loss.
pub struct Recorder {
    sender: Option<SyncSender<Message>>,
    thread: Option<JoinHandle<Result<()>>>,
}

impl Recorder {
    pub fn create(path: &Path) -> Result<Self> {
        let mut writer = CaptureWriter::create(path)?;
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
