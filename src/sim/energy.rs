//! A simulated Pro's target supply (`0x8F`) and energy capture (`0x90`), as HOST-INTERFACE.md
//! describes them.
//!
//! The target is a sleeping microcontroller that wakes for 10 ms every 100 ms: 20 µA asleep and
//! 5 mA awake, at 3.3 V while the Pro's supply is on and nothing while it is off. It holds DIO3 high
//! while awake, a marker a stream can report. Its values are made up; only their shape matters to a
//! host.

use std::time::Duration;

use crate::protocol::{ENERGY_COMMAND, TARGET_POWER_COMMAND};
use crate::stream::{Edge, Edges};

const DAP_OK: u8 = 0x00;
const DAP_ERROR: u8 = 0xFF;
const OK: u8 = 0;
const STOPPED: u8 = 1;
const NO_SAMPLE: u8 = 2;
const UNAVAILABLE: u8 = 3;
const BAD_RATE: u8 = 4;
const BAD_ARGUMENT: u8 = 5;
const DMA_ERROR: u8 = 7;
const MAX_RATE: u32 = 100_000;

const SUPPLY_MILLIVOLTS: u32 = 3_300;
const ASLEEP_NANOAMPS: u32 = 20_000;
const AWAKE_NANOAMPS: u32 = 5_000_000;
const PERIOD_MICROS: u64 = 100_000;
const AWAKE_MICROS: u64 = 10_000;
/// The 350 mA setting's low range ends at 2 mA (100 mV across 50 Ω).
const LOW_RANGE_NANOAMPS: u32 = 2_000_000;

/// The points a calibration of each range leaves, lowest first.
const CALIBRATION: [&[(u16, u32)]; 2] = [
    &[(410, 0), (31_200, 960_000), (62_100, 1_930_000)],
    &[(390, 0), (24_800, 98_000_000), (49_600, 197_000_000)],
];

#[derive(Clone, Debug, Default)]
pub struct SimEnergy {
    pub supply_on: bool,
    /// The capture since the last start, `None` before the first.
    pub capture: Option<Capture>,
    /// An acquisition fault, which stops integrating and lasts until a start succeeds.
    pub fault: bool,
    /// Verge Aux's stream of the capture.
    pub stream: Option<Stream>,
    /// Stream messages an event produced ahead of the next read: the last pairs of a capture a
    /// start replaced.
    outbox: Vec<Vec<u8>>,
}

/// A stream of the capture that started at `since`, its next pair counted from 0, reporting the
/// edges of `pins` after `edges_after` µs into the capture.
#[derive(Clone, Copy, Debug)]
pub struct Stream {
    pub since: Duration,
    pub next: u64,
    pub pins: u8,
    pub edges_after: u64,
}

/// The pin the target marks its waking time on.
const MARKER: u8 = 1 << 3;
/// The resolution a stream header gives for edges, as the Pro's 96 MHz stamping clock's.
const EDGE_RESOLUTION_NS: u16 = 11;

const FULL_SCALE_MV: u32 = 3_000;
const PAIRS_A_MESSAGE: u64 = 99;
const ASKED: u8 = 1;
const STOPPED_END: u8 = 2;
const RESTARTED: u8 = 3;
const FAILED: u8 = 4;

#[derive(Clone, Copy, Debug)]
pub struct Capture {
    pub since: Duration,
    pub rate: u32,
    pub stopped_at: Option<Duration>,
    /// When the supply last switched, in µs into the capture, and the energy integrated by then.
    pub banked: (u64, u64),
}

impl Capture {
    fn new(since: Duration, rate: u32) -> Self {
        Self {
            since,
            rate,
            stopped_at: None,
            banked: (0, 0),
        }
    }

    /// Complete pairs by `now`, or by the stop if it came first.
    fn pairs(&self, now: Duration) -> u64 {
        let end = self.stopped_at.unwrap_or(now).max(self.since);
        ((end - self.since).as_micros() * u128::from(self.rate) / 1_000_000) as u64
    }

    /// When pair `n` was taken, in microseconds from the start.
    fn micros(&self, n: u64) -> u64 {
        n * 1_000_000 / u64::from(self.rate)
    }
}

impl SimEnergy {
    /// What a restart leaves: supply off, no capture, no total.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Supply on and a capture running since `now` at `rate`.
    pub fn running(now: Duration, rate: u32) -> Self {
        Self {
            supply_on: true,
            capture: Some(Capture::new(now, rate)),
            fault: false,
            stream: None,
            outbox: Vec::new(),
        }
    }

    /// An acquisition fault at `now`, as a DMA error or an ADC overrun mid-capture leaves.
    pub fn fail(&mut self, now: Duration) {
        if let Some(capture) = &mut self.capture {
            capture.stopped_at.get_or_insert(now);
        }
        self.fault = true;
    }

    fn millivolts(&self) -> u32 {
        if self.supply_on { SUPPLY_MILLIVOLTS } else { 0 }
    }

    fn nanoamps_at(&self, micros: u64) -> u32 {
        match (self.supply_on, micros % PERIOD_MICROS < AWAKE_MICROS) {
            (false, _) => 0,
            (true, true) => AWAKE_NANOAMPS,
            (true, false) => ASLEEP_NANOAMPS,
        }
    }

    /// The total `micros` into the capture: what was banked at the last supply switch, and
    /// what the target has drawn since if the supply is on.
    fn nanojoules(&self, capture: &Capture, micros: u64) -> u64 {
        let (switched, banked) = capture.banked;
        if !self.supply_on {
            return banked;
        }
        banked + Self::drawn(micros) - Self::drawn(switched.min(micros))
    }

    /// The energy the target draws over the first `micros` of a capture with the supply on, in nJ.
    fn drawn(micros: u64) -> u64 {
        let awake =
            micros / PERIOD_MICROS * AWAKE_MICROS + (micros % PERIOD_MICROS).min(AWAKE_MICROS);
        // nA times µs is fC; times mV, 1e-18 J.
        let charge = u128::from(ASLEEP_NANOAMPS) * u128::from(micros)
            + u128::from(AWAKE_NANOAMPS - ASLEEP_NANOAMPS) * u128::from(awake);
        (charge * u128::from(SUPPLY_MILLIVOLTS) / 1_000_000_000) as u64
    }

    pub fn target_power(&mut self, pro: bool, mode: u8, now: Duration) -> Vec<u8> {
        let status = match (pro, mode) {
            (true, 0 | 1) => {
                if let Some(capture) = self.capture {
                    let micros = capture.micros(capture.pairs(now));
                    let banked = self.nanojoules(&capture, micros);
                    self.capture = Some(Capture {
                        banked: (micros, banked),
                        ..capture
                    });
                }
                self.supply_on = mode == 1;
                DAP_OK
            }
            (true, 2) => DAP_OK,
            _ => DAP_ERROR,
        };
        vec![TARGET_POWER_COMMAND, status, u8::from(self.supply_on)]
    }

    pub fn energy(&mut self, pro: bool, now: Duration, request: &[u8]) -> Vec<u8> {
        let mut reply = vec![ENERGY_COMMAND, DAP_OK];
        match request {
            [0, rate @ ..] if rate.len() == 4 => {
                let rate = u32::from_le_bytes(rate.try_into().expect("four bytes"));
                let result = if !pro {
                    UNAVAILABLE
                } else if rate == 0 || rate > MAX_RATE {
                    BAD_RATE
                } else {
                    // The probe streams the replaced capture's last pairs before the end that
                    // says it restarted.
                    let tail = self.stream_messages(now);
                    self.outbox.extend(tail);
                    self.capture = Some(Capture::new(now, rate));
                    self.fault = false;
                    OK
                };
                reply.push(result);
                let effective = if result == OK { rate } else { 0 };
                reply.extend_from_slice(&effective.to_le_bytes());
            }
            [1] => {
                let pairs = self
                    .capture
                    .filter(|c| c.stopped_at.is_none())
                    .map(|c| (c, c.pairs(now)));
                let (result, sample) = match pairs {
                    _ if !pro => (UNAVAILABLE, None),
                    _ if self.fault => (DMA_ERROR, None),
                    None => (STOPPED, None),
                    Some((_, 0)) => (NO_SAMPLE, None),
                    Some((capture, pairs)) => (OK, Some((pairs, capture.micros(pairs)))),
                };
                reply.push(result);
                let (sequence, millivolts, nanoamps) = sample
                    .map_or((0, 0, 0), |(pairs, micros)| {
                        (pairs as u32, self.millivolts(), self.nanoamps_at(micros))
                    });
                let range = match (sample, nanoamps > LOW_RANGE_NANOAMPS) {
                    (None, _) => 0,
                    (Some(_), false) => 1,
                    (Some(_), true) => 2,
                };
                // Codes as a 16-bit ADC against 3.0 V would read them: half the voltage, and the
                // current as 30 times its drop across the range's sense resistor.
                let voltage_code = (u64::from(millivolts) * 65_535 / 6_000) as u16;
                let current_code = (u64::from(nanoamps) * 65_535 / 2_000_000).min(65_535) as u16;
                reply.extend_from_slice(&voltage_code.to_le_bytes());
                reply.extend_from_slice(&current_code.to_le_bytes());
                reply.extend_from_slice(&sequence.to_le_bytes());
                reply.extend_from_slice(&millivolts.to_le_bytes());
                reply.extend_from_slice(&(nanoamps as i32).to_le_bytes());
                reply.extend_from_slice(&[range, u8::from(sample.is_some())]);
            }
            [2] => {
                if let Some(capture) = &mut self.capture {
                    capture.stopped_at.get_or_insert(now);
                }
                // A stop still stops, but answers with a fault until a start clears it.
                reply.push(match (pro, self.fault) {
                    (false, _) => UNAVAILABLE,
                    (true, true) => DMA_ERROR,
                    (true, false) => OK,
                });
            }
            [3] => {
                let (result, nanojoules, pairs) = match self.capture {
                    _ if !pro => (UNAVAILABLE, 0, 0),
                    None => (STOPPED, 0, 0),
                    Some(capture) => match capture.pairs(now) {
                        0 if !self.fault => (NO_SAMPLE, 0, 0),
                        pairs => (
                            if self.fault { DMA_ERROR } else { OK },
                            self.nanojoules(&capture, capture.micros(pairs)),
                            pairs,
                        ),
                    },
                };
                reply.push(result);
                reply.extend_from_slice(&nanojoules.to_le_bytes());
                reply.extend_from_slice(&pairs.to_le_bytes());
            }
            [4, range] => {
                let points = CALIBRATION.get(usize::from(*range)).filter(|_| pro);
                reply.push(match points {
                    Some(_) => OK,
                    None if pro => BAD_ARGUMENT,
                    None => UNAVAILABLE,
                });
                let points = points.copied().unwrap_or(&[]);
                reply.push(points.len() as u8);
                for slot in 0..4 {
                    let (code, nanoamps) = points.get(slot).copied().unwrap_or((0, 0));
                    reply.extend_from_slice(&code.to_le_bytes());
                    reply.extend_from_slice(&nanoamps.to_le_bytes());
                }
            }
            _ => reply[1] = DAP_ERROR,
        }
        reply
    }

    /// Pair `n`'s codes and range, as the probe would read the target's current through its sense
    /// resistor, the voltage divider's current included.
    fn pair(&self, capture: &Capture, n: u64) -> ([u8; 5], bool) {
        let millivolts = self.millivolts();
        let shunt = self.nanoamps_at(capture.micros(n)) + ((millivolts * 99_297) >> 16);
        let low = shunt <= LOW_RANGE_NANOAMPS;
        let table = CALIBRATION[usize::from(!low)];
        let segment = (0..table.len() - 1)
            .rev()
            .find(|&j| j == 0 || shunt >= table[j].1)
            .unwrap_or(0);
        let ((c0, i0), (c1, i1)) = (table[segment], table[segment + 1]);
        let code = i64::from(c0)
            + (i64::from(shunt) - i64::from(i0)) * (i64::from(c1) - i64::from(c0))
                / (i64::from(i1) - i64::from(i0));
        let current_code = code.clamp(0, 65_535) as u16;
        let voltage_code = ((u64::from(millivolts) << 16) / u64::from(2 * FULL_SCALE_MV)) as u16;
        let [v0, v1] = voltage_code.to_le_bytes();
        let [c0, c1] = current_code.to_le_bytes();
        ([v0, v1, c0, c1, u8::from(low)], low)
    }

    /// The marker's level `micros` into the capture.
    fn marker(&self, micros: u64) -> u8 {
        if self.supply_on && micros % PERIOD_MICROS < AWAKE_MICROS {
            MARKER
        } else {
            0
        }
    }

    /// The edges of `pins` after `after` µs into the capture and up to `until`, as one message
    /// each 99, flagged as the probe flags them.
    fn edges(&self, pins: u8, after: u64, until: u64) -> Vec<Vec<u8>> {
        if pins & MARKER == 0 || !self.supply_on || until <= after {
            return Vec::new();
        }
        let mut edges = Vec::new();
        let mut period = after / PERIOD_MICROS * PERIOD_MICROS;
        while period <= until {
            for at in [period, period + AWAKE_MICROS] {
                if at > after && at <= until {
                    edges.push(Edge {
                        time_ns: at * 1_000,
                        levels: self.marker(at),
                    });
                }
            }
            period += PERIOD_MICROS;
        }
        // 99 to a message, and none past what its offsets can hold, as the probe sends them.
        let mut messages: Vec<Vec<Edge>> = Vec::new();
        for edge in edges {
            match messages.last_mut() {
                Some(message)
                    if message.len() < 99
                        && edge.time_ns - message[0].time_ns <= u64::from(u32::MAX) =>
                {
                    message.push(edge);
                }
                _ => messages.push(vec![edge]),
            }
        }
        messages
            .into_iter()
            .map(|edges| Edges::new(0, edges).to_bytes())
            .collect()
    }

    fn header(
        capture: &Capture,
        first_sequence: u32,
        previous_low: bool,
        pins: u8,
        levels: u8,
    ) -> Vec<u8> {
        let mut header = vec![0xA1, 1];
        header.extend_from_slice(&capture.rate.to_le_bytes());
        header.push(1);
        header.extend_from_slice(&(FULL_SCALE_MV as u16).to_le_bytes());
        header.extend_from_slice(&first_sequence.to_le_bytes());
        header.push(u8::from(previous_low));
        for points in CALIBRATION {
            header.push(points.len() as u8);
            for slot in 0..4 {
                let (code, nanoamps) = points.get(slot).copied().unwrap_or((0, 0));
                header.extend_from_slice(&code.to_le_bytes());
                header.extend_from_slice(&nanoamps.to_le_bytes());
            }
        }
        header.extend_from_slice(&[pins, levels & pins]);
        let resolution = if pins == 0 { 0 } else { EDGE_RESOLUTION_NS };
        header.extend_from_slice(&resolution.to_le_bytes());
        header
    }

    fn end(reason: u8, last_sequence: u64) -> Vec<u8> {
        let mut end = vec![0xA2, reason];
        end.extend_from_slice(&(last_sequence as u32).to_le_bytes());
        end
    }

    /// The stream's messages due by `now`: pairs up to it, and the end once its capture has
    /// stopped, failed or been restarted.
    pub fn stream_messages(&mut self, now: Duration) -> Vec<Vec<u8>> {
        let mut messages = std::mem::take(&mut self.outbox);
        let Some(mut stream) = self.stream else {
            return messages;
        };
        let Some(capture) = self.capture.filter(|capture| capture.since == stream.since) else {
            self.stream = None;
            messages.push(Self::end(RESTARTED, stream.next));
            return messages;
        };
        // Each pairs message, then the edges up to its last pair, as the probe sends them.
        let available = capture.pairs(now);
        while stream.next < available {
            let count = (available - stream.next).min(PAIRS_A_MESSAGE);
            let mut message = vec![0xA0, 0];
            message.extend_from_slice(&((stream.next + 1) as u32).to_le_bytes());
            message.extend_from_slice(&(capture.micros(stream.next) * 1_000).to_le_bytes());
            message.extend_from_slice(&(count as u16).to_le_bytes());
            for n in stream.next..stream.next + count {
                message.extend_from_slice(&self.pair(&capture, n).0);
            }
            messages.push(message);
            stream.next += count;
            let last = capture.micros(stream.next - 1);
            messages.extend(self.edges(stream.pins, stream.edges_after, last));
            stream.edges_after = stream.edges_after.max(last);
        }
        if capture.stopped_at.is_some() {
            self.stream = None;
            messages.push(Self::end(
                if self.fault { FAILED } else { STOPPED_END },
                stream.next,
            ));
        } else {
            self.stream = Some(stream);
        }
        messages
    }

    /// `0x90` mode 5, which only Verge Aux has: a start, as mode 0, streamed from its first pair.
    /// The firmware holds stream messages back while the request is out, so its reply comes
    /// first, then the end of any stream the start restarted, then the header.
    pub fn start_stream(
        &mut self,
        pro: bool,
        now: Duration,
        rate: [u8; 4],
        pins: u8,
    ) -> Vec<Vec<u8>> {
        let mut messages = self.stream_messages(now);
        let mut request = vec![0];
        request.extend_from_slice(&rate);
        let reply = self.energy(pro, now, &request);
        let started = reply[2] == OK;
        messages.push(reply);
        messages.extend(self.stream_messages(now));
        if let Some(capture) = self.capture.filter(|_| started) {
            let pins = pins & 0x0F;
            self.stream = Some(Stream {
                since: capture.since,
                next: 0,
                pins,
                edges_after: 0,
            });
            messages.push(Self::header(&capture, 1, true, pins, self.marker(0)));
        }
        messages
    }

    /// `0x90` mode 6: the running capture, streamed from its next pair. Its reply, then the end of
    /// any stream it replaces, then the header.
    pub fn stream_running(&mut self, pro: bool, now: Duration, pins: u8) -> Vec<Vec<u8>> {
        let mut messages = self.stream_messages(now);
        let running = self.capture.filter(|capture| capture.stopped_at.is_none());
        let result = match running {
            _ if !pro => UNAVAILABLE,
            _ if self.fault => DMA_ERROR,
            None => STOPPED,
            Some(_) => OK,
        };
        messages.push(vec![ENERGY_COMMAND, DAP_OK, result]);
        if result == OK
            && let Some(stream) = self.stream.take()
        {
            messages.push(Self::end(ASKED, stream.next));
        }
        if let Some(capture) = running.filter(|_| result == OK) {
            let next = capture.pairs(now);
            let previous_low = next == 0 || self.pair(&capture, next - 1).1;
            let pins = pins & 0x0F;
            let started = (now.saturating_sub(capture.since)).as_micros() as u64;
            self.stream = Some(Stream {
                since: capture.since,
                next,
                pins,
                edges_after: started,
            });
            messages.push(Self::header(
                &capture,
                (next + 1) as u32,
                previous_low,
                pins,
                self.marker(started),
            ));
        }
        messages
    }

    /// `0x90` mode 7: the stream stops, the capture carries on. The reply, then the end.
    pub fn stop_stream(&mut self, pro: bool, now: Duration) -> Vec<Vec<u8>> {
        let mut messages = self.stream_messages(now);
        messages.push(vec![
            ENERGY_COMMAND,
            DAP_OK,
            if pro { OK } else { UNAVAILABLE },
        ]);
        if let Some(stream) = self.stream.take() {
            messages.push(Self::end(ASKED, stream.next));
        }
        messages
    }
}
