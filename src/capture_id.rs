use serde::{Deserialize, Serialize};

/// A probe's boot value and capture counter, compared as one opaque token.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CaptureId([u8; 24]);

impl CaptureId {
    pub const BYTES: usize = 24;

    #[must_use]
    pub fn from_bytes(bytes: [u8; 24]) -> Option<Self> {
        bytes[16..]
            .iter()
            .any(|&byte| byte != 0)
            .then_some(Self(bytes))
    }

    #[must_use]
    pub fn as_bytes(&self) -> &[u8; 24] {
        &self.0
    }
}
