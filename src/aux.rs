//! Verge Aux, the second vendor interface of an MCU-Link running Verge (mculink-verge
//! docs/host-interface.md, "The auxiliary interface: Verge Aux"): the DIO, supply and energy
//! commands beside a debugger that holds CMSIS-DAP, and the energy and DIO streams.
//!
//! One request at a time, each answered by one reply in its command's CMSIS-DAP format. Stream
//! messages come between replies, told apart by their first byte, and wait here for
//! [`Aux::message`].

use std::collections::VecDeque;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use probe_rs::probe::DebugProbeError;

use crate::dap::LinkError;
use crate::probe::VergeTransport;
use crate::stream::{self, Message, StreamError};

/// The interface string the firmware gives it, which no debugger looks for.
pub const INTERFACE_NAME: &str = "Verge Aux";

const REPLY_TIMEOUT: Duration = Duration::from_secs(2);
const DRAIN_TIMEOUT: Duration = Duration::from_millis(50);
/// What a probe can have waiting for a host: the 16 messages it keeps, the transfer already armed,
/// and a reply to a request the last program never read.
const WAITING: usize = 18;
/// Stream messages kept while a caller reads only replies: about a second at the top rate.
const KEPT_MESSAGES: usize = 1024;
const ID_DAP_INVALID: u8 = 0xFF;

/// The interface's bulk pair, as the bus opens it.
pub trait AuxLink {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError>;
    /// One transfer from the IN endpoint, or [`LinkError::Timeout`] if none came.
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError>;
    /// Time since an arbitrary start, which a stream's waits are timed by. The simulator's link
    /// gives its bench's time, as [`crate::Bus::now`] does, so a link that wraps another forwards
    /// it.
    fn now(&self) -> Duration {
        static START: OnceLock<Instant> = OnceLock::new();
        START.get_or_init(Instant::now).elapsed()
    }
}

impl<L: AuxLink + ?Sized> AuxLink for Box<L> {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError> {
        (**self).write(request)
    }

    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError> {
        (**self).read(timeout)
    }

    fn now(&self) -> Duration {
        (**self).now()
    }
}

/// Why reading Verge Aux's stream failed: the link, or a message that does not parse.
#[derive(Debug, thiserror::Error)]
pub enum AuxError {
    #[error(transparent)]
    Link(#[from] LinkError),
    #[error(transparent)]
    Stream(#[from] StreamError),
}

/// A claimed auxiliary interface.
pub struct Aux<L: AuxLink> {
    link: L,
    /// Stream messages that arrived while a reply was awaited, oldest first.
    messages: VecDeque<Vec<u8>>,
    /// Messages dropped because nobody read them.
    dropped: u64,
}

impl<L: AuxLink> Aux<L> {
    /// Takes the interface, dropping what the program before this one left waiting on it: a reply
    /// or a stream message meant for that program. A stream it left running is ended (`0x90` mode
    /// 7): no other program can hold the interface, so nothing else would read it, and the probe
    /// keeps a stream nobody reads running. Its capture carries on. Mode 7 goes whether or not a
    /// stream shows: a slow one can stay quiet past the drain, and with none it changes nothing.
    pub fn open(link: L) -> Result<Aux<L>, LinkError> {
        let mut aux = Aux {
            link,
            messages: VecDeque::new(),
            dropped: 0,
        };
        // Read before writing. A reply the last program never read waits behind the message ahead
        // of it, and the probe takes no request until that reply is on its way.
        aux.drain()?;
        // A reply the last program left can still come first and be taken for this one's; the
        // drain after drops the real one with the stream's last pairs and its end.
        aux.exchange(&[0x90, 7])?;
        if aux.drain()? {
            return Err(LinkError::Usb(
                "the probe kept streaming after it was asked to stop".into(),
            ));
        }
        aux.messages.clear();
        Ok(aux)
    }

    /// Reads and drops what is waiting, until the probe goes quiet or [`WAITING`] transfers have
    /// come. True when a stream may still be running: pairs came after anything that ended it.
    fn drain(&mut self) -> Result<bool, LinkError> {
        let mut streaming = false;
        for _ in 0..WAITING {
            match self.link.read(DRAIN_TIMEOUT) {
                Ok(data) => match data.first() {
                    Some(&stream::PAIRS | &stream::HEADER | &stream::EDGES) => streaming = true,
                    Some(&stream::END) => streaming = false,
                    _ => {}
                },
                Err(LinkError::Timeout) => return Ok(streaming),
                Err(error) => return Err(error),
            }
        }
        Ok(true)
    }

    fn keep(&mut self, message: Vec<u8>) {
        if self.messages.len() == KEPT_MESSAGES {
            self.messages.pop_front();
            self.dropped += 1;
        }
        self.messages.push_back(message);
    }

    /// Sends one request and returns its reply, command byte first. Stream messages that come
    /// before it are kept for [`Aux::message`].
    pub fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError> {
        self.link.write(request)?;
        loop {
            let data = self.link.read(REPLY_TIMEOUT)?;
            if stream::is_stream_message(&data) {
                self.keep(data);
            } else {
                if request.first() == Some(&0x90)
                    && matches!(request.get(1), Some(9 | 10 | 15))
                    && data.get(..3) == Some(&[0x90, 0, 0])
                {
                    self.messages.clear();
                }
                return Ok(data);
            }
        }
    }

    /// The next stream message, waiting up to `timeout`: `None` if none came. A reply that arrives
    /// here belongs to no request this side sent, and is dropped.
    pub fn message(&mut self, timeout: Duration) -> Result<Option<Message>, AuxError> {
        loop {
            let data = match self.messages.pop_front() {
                Some(data) => data,
                None => match self.link.read(timeout) {
                    Ok(data) => data,
                    Err(LinkError::Timeout) => return Ok(None),
                    Err(error) => return Err(error.into()),
                },
            };
            if stream::is_stream_message(&data) {
                return Ok(Some(stream::parse(&data)?));
            }
        }
    }

    /// Stream messages this side dropped, keeping the newest while nobody read them.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// The link's time ([`AuxLink::now`]).
    #[must_use]
    pub fn now(&self) -> Duration {
        self.link.now()
    }
}

/// The interface carries `0x82`, `0x8E`, `0x8F` and `0x90`, in their CMSIS-DAP formats. Replies come back
/// as [`crate::DapTransport`] gives them: without the command byte, and a lone `0xFF` as a command
/// the interface does not carry.
impl<L: AuxLink> VergeTransport for Aux<L> {
    fn is_aux(&self) -> bool {
        true
    }

    fn send_vendor_command(
        &mut self,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, DebugProbeError> {
        let request = [&[command_id][..], payload].concat();
        let reply = self
            .exchange(&request)
            .map_err(|error| DebugProbeError::Other(error.to_string()))?;
        match reply.split_first() {
            Some((&id, rest)) if id == command_id => Ok(rest.to_vec()),
            Some((&ID_DAP_INVALID, _)) => Err(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "vendor command on Verge Aux",
            }),
            other => Err(DebugProbeError::Other(format!(
                "asked Verge Aux for command 0x{command_id:02x} and the reply was {}",
                other.map_or_else(|| "empty".to_owned(), |(id, _)| format!("0x{id:02x}"))
            ))),
        }
    }
}
