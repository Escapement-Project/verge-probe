//! A unit's stream through Verge Aux, from its start to its end: the protocol work every program
//! that records one does, in one place.

use std::time::{Duration, SystemTime};

use crate::aux::{Aux, AuxError, AuxLink};
use crate::record::Context;
use crate::stream::{Converter, DioHeader, End, Header, Message, Total, skipped};
use crate::{
    BoardKind, CaptureId, CommandError, EnergyCaptureSample, EnergyCaptureStart, EnergyError,
    EnergyRanges, EnergyTotal, VergeControl, VergeInfo,
};

/// How long the probe has to send a stream's header once its start is answered, and its end once
/// it is asked for.
pub const WAIT: Duration = Duration::from_secs(2);

/// How long [`Reader::messages`] waits on each read before it asks `stop` again.
const SLICE: Duration = Duration::from_millis(50);

/// How an energy stream begins: [`Reader::start_energy`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct EnergyStream {
    /// Pairs a second for a capture the start begins, 1 to 100,000.
    pub rate: u32,
    /// The current ranges for a capture the start begins.
    pub ranges: EnergyRanges,
    /// The DIO pins whose edges the stream reports, bit `k` for DIO`k`.
    pub dio: u8,
    /// Whether to follow a capture that runs already rather than start a new one.
    pub join: bool,
}

impl EnergyStream {
    /// A stream at `rate` pairs a second, switching ranges automatically, with no DIO pins, that
    /// follows a capture already running.
    #[must_use]
    pub fn new(rate: u32) -> Self {
        Self {
            rate,
            ranges: EnergyRanges::Auto,
            dio: 0,
            join: true,
        }
    }

    /// The current ranges for a capture the start begins.
    #[must_use]
    pub fn ranges(mut self, ranges: EnergyRanges) -> Self {
        self.ranges = ranges;
        self
    }

    /// The DIO pins whose edges the stream reports, bit `k` for DIO`k`.
    #[must_use]
    pub fn dio(mut self, pins: u8) -> Self {
        self.dio = pins;
        self
    }

    /// Whether to follow a capture that runs already. Off, the start replaces it, and clears its
    /// total.
    #[must_use]
    pub fn join(mut self, join: bool) -> Self {
        self.join = join;
        self
    }
}

/// How [`Reader::start_energy`] began the stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Began {
    /// A capture ran already. The stream follows it from its next pair, at its own rate and
    /// ranges, and leaves it running.
    Joined,
    /// The reader started a capture, which it stops when the stream ends. `replaced` is the total
    /// of the capture the start replaced, where one had run or failed and the firmware keeps a
    /// total.
    #[non_exhaustive]
    Started {
        start: EnergyCaptureStart,
        replaced: Option<EnergyTotal>,
    },
}

/// What the stream under way, or the last one, carried.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct Counts {
    /// The pairs and their energy, as the probe's total counts them. Zero in a DIO stream.
    pub total: Total,
    /// Pairs the probe dropped because nobody read them in time, from the gaps in their sequence.
    pub lost: u64,
    /// Pairs read under range: too near the high range's floor to trust.
    pub under_range: u64,
    /// The DIO pins the stream reports, bit `k` for DIO`k`.
    pub pins: u8,
    /// Their levels after the latest edge, or where edges were lost, the levels known again.
    pub levels: u8,
    /// Each reported pin's edges, DIO0 first.
    pub edges: [u64; 4],
    /// How often the probe lost edges. The levels between are unknown.
    pub edge_losses: u64,
    /// Marker messages that came after markers the probe had no room for.
    pub marker_losses: u64,
    /// The stream's end, once it has come.
    pub end: Option<End>,
}

/// Why a stream could not be started, read or ended.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ReaderError {
    #[error("a stream is under way already")]
    Busy,
    #[error("asking what the probe runs")]
    Info(#[source] CommandError<std::convert::Infallible>),
    #[error("the probe no longer answers as Verge")]
    NotVerge,
    #[error("asking whether a capture runs")]
    Status(#[source] CommandError<EnergyError>),
    #[error("setting the current ranges")]
    Ranges(#[source] CommandError<EnergyError>),
    #[error("starting the stream")]
    Start(#[source] CommandError<EnergyError>),
    #[error("following the running capture")]
    Join(#[source] CommandError<EnergyError>),
    #[error("ending the stream")]
    End(#[source] CommandError<EnergyError>),
    #[error("reading the stream")]
    Read(#[source] AuxError),
    #[error("the probe sent no stream header within {} s of the start", WAIT.as_secs())]
    NoHeader,
    #[error("the probe did not end the stream within {} s of being asked", WAIT.as_secs())]
    NotEnded,
    #[error("the probe began a second stream before ending the first")]
    SecondStream,
    #[error("the probe began a DIO stream, which carries no energy")]
    DioInEnergy,
    #[error("the probe streamed energy samples, not its DIO pins alone")]
    EnergyInDio,
    #[error("the stream header carries no capture identity")]
    NoIdentity,
    #[error("the stream header names another capture than the one started")]
    OtherCapture,
}

/// A unit's Verge Aux, with at most one stream on it: an energy stream on a Pro, or a DIO stream on
/// a board without energy capture.
///
/// A start answers before anything is read, so a refused one leaves nothing to clean up. Then
/// [`Reader::read`] or [`Reader::messages`] gives the stream's messages from its header to its end,
/// dropping what a stream it replaced left. The probe's other commands go through
/// [`Reader::control`] meanwhile.
///
/// Dropping the reader, or [`Reader::abandon`], ends the stream under way: a capture the reader
/// started stops, and the stream of a capture it joined ends, the capture running on.
pub struct Reader<L: AuxLink> {
    aux: Aux<L>,
    live: Option<Live>,
    counts: Counts,
    header: Option<Header>,
    dio_header: Option<DioHeader>,
    info: Option<VergeInfo>,
    started_utc: Option<SystemTime>,
}

/// The stream under way.
struct Live {
    kind: Kind,
    /// When the start was answered.
    started: Duration,
    /// When the end was asked for.
    asked: Option<Duration>,
    /// An end asked for before a joined stream's header, sent once the header names the capture.
    end_pending: bool,
    begun: bool,
    capture_id: Option<CaptureId>,
    converter: Option<Converter>,
    next_sequence: Option<u32>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// `owned`: the reader started the capture.
    Energy {
        owned: bool,
    },
    Dio,
}

impl<L: AuxLink> Reader<L> {
    /// A reader on `aux`, with no stream under way.
    pub fn new(aux: Aux<L>) -> Self {
        Self {
            aux,
            live: None,
            counts: Counts::default(),
            header: None,
            dio_header: None,
            info: None,
            started_utc: None,
        }
    }

    /// The probe's commands, through the same interface. A stream's messages that arrive while
    /// one awaits its reply wait for [`Reader::read`]. Start and end streams with the reader's own
    /// calls: one sent through here changes the stream behind the reader's back.
    pub fn control(&mut self) -> VergeControl<'_, Aux<L>> {
        VergeControl::new(&mut self.aux)
    }

    /// Starts an energy stream: follows a capture that runs, where `stream.join` allows, or starts
    /// one. A capture that failed counts as none running.
    pub fn start_energy(&mut self, stream: &EnergyStream) -> Result<Began, ReaderError> {
        if self.live.is_some() {
            return Err(ReaderError::Busy);
        }
        let info = self.read_info()?;
        let mut control = VergeControl::new(&mut self.aux);
        let running = match control.latest_energy_sample() {
            Ok(EnergyCaptureSample::Stopped) => None,
            Err(CommandError::Firmware(EnergyError::DmaError)) => Some(false),
            Ok(_) => Some(true),
            Err(error) => return Err(ReaderError::Status(error)),
        };
        if running == Some(true) && stream.join {
            control
                .stream_energy_capture_identified(stream.dio)
                .map_err(ReaderError::Join)?;
            self.begin(Kind::Energy { owned: false }, None);
            return Ok(Began::Joined);
        }
        let replaced = match running {
            Some(_) if info.supports_energy_accumulator() => control.read_energy_accumulator().ok(),
            _ => None,
        };
        control
            .set_energy_ranges(stream.ranges)
            .map_err(ReaderError::Ranges)?;
        let start = control
            .start_energy_stream_identified(stream.rate, stream.dio)
            .map_err(ReaderError::Start)?;
        self.begin(Kind::Energy { owned: true }, start.capture_id);
        Ok(Began::Started { start, replaced })
    }

    /// Starts a DIO stream of the pins in `pins`, bit `k` for DIO`k`, on a board without energy
    /// capture ([`VergeInfo::supports_dio_stream`]).
    pub fn start_dio(&mut self, pins: u8) -> Result<(), ReaderError> {
        if self.live.is_some() {
            return Err(ReaderError::Busy);
        }
        self.read_info()?;
        VergeControl::new(&mut self.aux)
            .start_dio_stream(pins)
            .map_err(ReaderError::Start)?;
        self.begin(Kind::Dio, None);
        Ok(())
    }

    fn read_info(&mut self) -> Result<VergeInfo, ReaderError> {
        let info = VergeControl::new(&mut self.aux)
            .info()
            .map_err(ReaderError::Info)?
            .ok_or(ReaderError::NotVerge)?
            .clone();
        self.info = Some(info.clone());
        Ok(info)
    }

    fn begin(&mut self, kind: Kind, capture_id: Option<CaptureId>) {
        self.counts = Counts::default();
        self.header = None;
        self.dio_header = None;
        self.started_utc = Some(SystemTime::now());
        self.live = Some(Live {
            kind,
            started: self.aux.now(),
            asked: None,
            end_pending: false,
            begun: false,
            capture_id,
            converter: None,
            next_sequence: None,
        });
    }

    /// The stream's next message, waiting up to `timeout` for it: `None` if none came, or no stream
    /// is under way. Its header comes first; once its end has come, the reader is idle again.
    ///
    /// Fails when the header is [`WAIT`] late, or the end is that late once asked for, and on a
    /// message that does not belong in the stream. A header or an end that has come is taken however
    /// late it is read. The stream stays under way after an error, for [`Reader::abandon`] or a drop
    /// to clean up.
    pub fn read(&mut self, timeout: Duration) -> Result<Option<Message>, ReaderError> {
        loop {
            if self.live.is_none() {
                return Ok(None);
            }
            let message = self.aux.message(timeout).map_err(ReaderError::Read)?;
            let empty = message.is_none();
            if let Some(message) = message
                && let Some(message) = self.take(message)?
            {
                return Ok(Some(message));
            }
            // Nothing came, or only what a replaced stream left: the probe is late if anything.
            let Some(live) = &self.live else {
                return Ok(None);
            };
            let now = self.aux.now();
            if !live.begun && now.saturating_sub(live.started) >= WAIT {
                return Err(ReaderError::NoHeader);
            }
            if live
                .asked
                .is_some_and(|asked| now.saturating_sub(asked) > WAIT)
            {
                return Err(ReaderError::NotEnded);
            }
            if empty {
                return Ok(None);
            }
        }
    }

    /// Counts `message` and passes it on, or drops what a replaced stream left before the header.
    fn take(&mut self, message: Message) -> Result<Option<Message>, ReaderError> {
        let Some(live) = self.live.as_mut() else {
            return Ok(None);
        };
        let counts = &mut self.counts;
        match (live.kind, &message) {
            (Kind::Energy { .. }, Message::DioHeader(_)) => return Err(ReaderError::DioInEnergy),
            (Kind::Energy { .. }, Message::Header(header)) => {
                if live.begun {
                    return Err(ReaderError::SecondStream);
                }
                let id = header.capture_id.ok_or(ReaderError::NoIdentity)?;
                if live.capture_id.is_some_and(|started| started != id) {
                    return Err(ReaderError::OtherCapture);
                }
                live.capture_id = Some(id);
                live.begun = true;
                live.converter = Some(Converter::new(header));
                live.next_sequence = Some(header.first_sequence);
                counts.pins = header.dio_mask;
                counts.levels = header.dio_levels;
                let send = std::mem::take(&mut live.end_pending);
                self.header = Some(header.clone());
                if send {
                    self.send_end()?;
                }
                return Ok(Some(message));
            }
            (Kind::Dio, Message::DioHeader(header)) => {
                if live.begun {
                    return Err(ReaderError::SecondStream);
                }
                live.begun = true;
                counts.pins = header.dio_mask;
                counts.levels = header.dio_levels;
                self.dio_header = Some(*header);
                return Ok(Some(message));
            }
            // A DIO stream's heartbeat: in an energy stream, one this stream replaced.
            (Kind::Energy { .. }, Message::DioTime(_)) => return Ok(None),
            _ if !live.begun => return Ok(None),
            (Kind::Dio, Message::Header(_) | Message::Pairs(_)) => {
                return Err(ReaderError::EnergyInDio);
            }
            _ => {}
        }
        match &message {
            Message::Pairs(pairs) => {
                if let Some(converter) = live.converter.as_mut() {
                    if let Some(next) = live.next_sequence {
                        let missing = skipped(next, pairs.first_sequence);
                        counts.lost += missing;
                        if missing > 0
                            && let Some(pair) = pairs.pairs.first()
                        {
                            converter.discontinuity(pair.low);
                        }
                    }
                    live.next_sequence =
                        Some(pairs.first_sequence.wrapping_add(pairs.pairs.len() as u32));
                    converter.add_message(pairs);
                    counts.under_range += under_range(converter);
                    counts.total = converter.total();
                }
            }
            Message::Edges(message) => {
                for edge in &message.edges {
                    let moved = (counts.levels ^ edge.levels) & counts.pins;
                    for (pin, count) in counts.edges.iter_mut().enumerate() {
                        *count += u64::from(moved >> pin & 1);
                    }
                    counts.levels = edge.levels;
                }
            }
            Message::DioLost(lost) => {
                counts.edge_losses += 1;
                counts.levels = lost.levels;
            }
            Message::Markers(markers) if markers.lost() => counts.marker_losses += 1,
            Message::End(end) => {
                // The probe keeps room for the end when it drops pairs, so pairs lost just before
                // it show only in its last sequence.
                if let (Some(next), Some(converter)) = (live.next_sequence, live.converter.as_mut())
                {
                    counts.lost += skipped(next, end.last_sequence.wrapping_add(1));
                    converter.end();
                    counts.under_range += under_range(converter);
                    counts.total = converter.total();
                }
                counts.end = Some(*end);
                self.live = None;
            }
            _ => {}
        }
        Ok(Some(message))
    }

    /// Asks the probe to end the stream: a capture the reader started stops, the stream of one it
    /// joined ends, and a DIO stream ends. Its end message follows the last of its messages. Does
    /// nothing once asked, or with no stream under way.
    pub fn end(&mut self) -> Result<(), ReaderError> {
        let Some(live) = &mut self.live else {
            return Ok(());
        };
        if live.asked.is_some() || live.end_pending {
            return Ok(());
        }
        if live.kind == (Kind::Energy { owned: false }) && live.capture_id.is_none() {
            live.end_pending = true;
            return Ok(());
        }
        self.send_end()
    }

    fn send_end(&mut self) -> Result<(), ReaderError> {
        let now = self.aux.now();
        let live = self.live.as_mut().expect("a stream is under way");
        let mut control = VergeControl::new(&mut self.aux);
        let sent = match (live.kind, live.capture_id) {
            (Kind::Dio, _) => control.end_dio_stream(),
            (Kind::Energy { owned: true }, Some(id)) => control.stop_energy_capture_if(id),
            (Kind::Energy { owned: false }, Some(id)) => control.end_energy_stream_if(id),
            // `end` waits for a joined stream's header, and a start's reply names its capture.
            (Kind::Energy { .. }, None) => return Err(ReaderError::NoIdentity),
        };
        match sent {
            // A capture that failed stops all the same, and one another host restarted ends this
            // stream by itself.
            Ok(())
            | Err(CommandError::Firmware(EnergyError::DmaError | EnergyError::StaleCapture)) => {
                live.asked = Some(now);
                Ok(())
            }
            Err(error) => Err(ReaderError::End(error)),
        }
    }

    /// Ends the stream without waiting for its end, and makes the reader idle: a capture the
    /// reader started stops, and the stream of one it joined ends, the capture running on. The end
    /// goes again if it was asked for already. A joined stream whose header has not come is given
    /// [`WAIT`] for it, as only the header names the capture; without it, the stream is ended
    /// whatever capture it follows.
    pub fn abandon(&mut self) -> Result<(), ReaderError> {
        let Some(live) = &self.live else {
            return Ok(());
        };
        let mut waited = Ok(());
        if live.kind == (Kind::Energy { owned: false }) && live.capture_id.is_none() {
            waited = self.wait_for_identity();
        }
        let ended = match self.live.take() {
            Some(live) if live.capture_id.is_some() || live.kind == Kind::Dio => {
                self.live = Some(live);
                let ended = self.send_end();
                self.live = None;
                ended
            }
            // A joined stream whose capture is not known: mode 7 ends this interface's stream.
            Some(_) => VergeControl::new(&mut self.aux)
                .end_energy_stream()
                .map_err(ReaderError::End),
            None => Ok(()),
        };
        waited.and(ended)
    }

    /// Reads until a joined stream's header names its capture, its end comes, or [`WAIT`] passes.
    /// Messages that do not parse are skipped.
    fn wait_for_identity(&mut self) -> Result<(), ReaderError> {
        let until = self.aux.now() + WAIT;
        while self
            .live
            .as_ref()
            .is_some_and(|live| live.capture_id.is_none())
        {
            if self.aux.now() >= until {
                return Err(ReaderError::NoHeader);
            }
            match self.aux.message(SLICE) {
                // The header sends an end asked for before it; an end makes the reader idle.
                Ok(Some(message)) => {
                    self.take(message)?;
                }
                Ok(None) | Err(AuxError::Stream(_)) => {}
                Err(error) => return Err(ReaderError::Read(error)),
            }
        }
        Ok(())
    }

    /// An iterator over the stream's messages that asks for the end once `stop` returns true, and
    /// ends after the stream's end. It checks `stop` at least every 50 ms. After an error it
    /// abandons the stream ([`Reader::abandon`]) and ends.
    pub fn messages<F: FnMut() -> bool>(&mut self, stop: F) -> Messages<'_, L, F> {
        Messages {
            reader: self,
            stop,
            done: false,
        }
    }

    /// Whether a stream is under way: from its start until its end has been read.
    #[must_use]
    pub fn active(&self) -> bool {
        self.live.is_some()
    }

    /// Whether the end of the stream under way has been asked for.
    #[must_use]
    pub fn ending(&self) -> bool {
        self.live
            .as_ref()
            .is_some_and(|live| live.asked.is_some() || live.end_pending)
    }

    /// Whether the reader started the capture of the stream under way, and so stops it.
    #[must_use]
    pub fn owns_capture(&self) -> bool {
        self.live
            .as_ref()
            .is_some_and(|live| live.kind == Kind::Energy { owned: true })
    }

    /// The identity of the capture under way: from the start's reply, or a joined stream's header.
    #[must_use]
    pub fn capture_id(&self) -> Option<CaptureId> {
        self.live.as_ref().and_then(|live| live.capture_id)
    }

    /// What the stream under way, or the last one, carried.
    #[must_use]
    pub fn counts(&self) -> &Counts {
        &self.counts
    }

    /// The header of the energy stream under way, or the last one, once it has come.
    #[must_use]
    pub fn header(&self) -> Option<&Header> {
        self.header.as_ref()
    }

    /// The header of the DIO stream under way, or the last one, once it has come.
    #[must_use]
    pub fn dio_header(&self) -> Option<&DioHeader> {
        self.dio_header.as_ref()
    }

    /// What the probe says it is, read once, and again at each start.
    pub fn info(&mut self) -> Result<&VergeInfo, ReaderError> {
        if self.info.is_none() {
            self.read_info()?;
        }
        Ok(self.info.as_ref().expect("read above"))
    }

    /// What a recording of the stream says about where it came from: the probe's model and
    /// firmware, and when the stream under way started, or before a start, now. Empty until the
    /// probe has said what it is ([`Reader::info`], or a start).
    #[must_use]
    pub fn context(&self) -> Context {
        let Some(info) = &self.info else {
            return Context::default();
        };
        let started = match self.live {
            Some(_) => self.started_utc,
            None => None,
        };
        let instrument = match info.board {
            Some(BoardKind::Pro) => "MCU-Link Pro",
            _ => "MCU-Link",
        };
        Context {
            instrument: Some(instrument.into()),
            software: Some(info.firmware.clone()),
            started_utc: Some(
                humantime::format_rfc3339_millis(started.unwrap_or_else(SystemTime::now))
                    .to_string(),
            ),
        }
    }
}

/// Pairs the converter has counted since the last call that were read under range.
fn under_range(converter: &mut Converter) -> u64 {
    converter
        .drain_samples()
        .filter(|sample| sample.under_range)
        .count() as u64
}

impl<L: AuxLink> Drop for Reader<L> {
    fn drop(&mut self) {
        if let Err(error) = self.abandon() {
            tracing::warn!("could not end the stream: {error}");
        }
    }
}

/// [`Reader::messages`].
pub struct Messages<'a, L: AuxLink, F> {
    reader: &'a mut Reader<L>,
    stop: F,
    done: bool,
}

impl<L: AuxLink, F> Messages<'_, L, F> {
    /// The reader, for what the stream has carried so far ([`Reader::counts`]).
    #[must_use]
    pub fn reader(&self) -> &Reader<L> {
        self.reader
    }
}

impl<L: AuxLink, F: FnMut() -> bool> Iterator for Messages<'_, L, F> {
    type Item = Result<Message, ReaderError>;

    fn next(&mut self) -> Option<Self::Item> {
        while !self.done && self.reader.active() {
            let step = if (self.stop)() {
                self.reader.end()
            } else {
                Ok(())
            };
            match step.and_then(|()| self.reader.read(SLICE)) {
                Ok(Some(message)) => return Some(Ok(message)),
                Ok(None) => {}
                Err(error) => {
                    self.done = true;
                    if let Err(left) = self.reader.abandon() {
                        tracing::warn!("could not end the stream: {left}");
                    }
                    return Some(Err(error));
                }
            }
        }
        None
    }
}
