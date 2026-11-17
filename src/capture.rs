//! Energy captures written to disk without holding up the stream.

mod recorder;
pub use recorder::Recorder;

use std::io;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CaptureError {
    #[error("the recording is closed")]
    Closed,
    /// The disk queue filled: the disk cannot keep up with the stream.
    #[error("the capture's disk writer fell behind the stream")]
    Behind,
    #[error("the capture's disk writer stopped")]
    Stopped,
    #[error("the capture's disk writer panicked")]
    Panicked,
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error(transparent)]
    Tickwave(#[from] crate::tickwave::Error),
}

type Result<T> = std::result::Result<T, CaptureError>;
