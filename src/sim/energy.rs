//! A simulated Pro's target supply (`0x8F`) and energy capture (`0x90`), as docs/host-interface.md
//! describes them.
//!
//! The target is a sleeping microcontroller that wakes for 10 ms every 100 ms: 20 µA asleep and
//! 5 mA awake, at 3.3 V while the Pro's supply is on and nothing while it is off. It holds DIO3 high
//! while awake, a marker a stream can report. Its values are made up; only their shape matters to a
//! host.

use std::time::Duration;

use crate::protocol::{ENERGY_COMMAND, MARK_COMMAND, TARGET_POWER_COMMAND};
use crate::stream::{Edge, Edges, LABEL_MAX, Marker, Markers};

const DAP_OK: u8 = 0x00;
const DAP_ERROR: u8 = 0xFF;
const OK: u8 = 0;
const STOPPED: u8 = 1;
const NO_SAMPLE: u8 = 2;
const UNAVAILABLE: u8 = 3;
const BAD_RATE: u8 = 4;
const BAD_ARGUMENT: u8 = 5;
const DMA_ERROR: u8 = 7;
const MARK_SENT: u8 = 0;
const MARK_NO_CAPTURE: u8 = 1;
const MARK_NO_STREAM: u8 = 2;
const MAX_RATE: u32 = 100_000;

const SUPPLY_MILLIVOLTS: u32 = 3_300;
const ASLEEP_NANOAMPS: u32 = 20_000;
const AWAKE_NANOAMPS: u32 = 5_000_000;
const PERIOD_MICROS: u64 = 100_000;
const AWAKE_MICROS: u64 = 10_000;
/// The 350 mA setting's low range ends at 2 mA (100 mV across 50 Ω).
const LOW_RANGE_NANOAMPS: u32 = 2_000_000;
/// A high-range code under this is marked under range: the table's zero-current code, the
/// amplifier's floor here, plus calibration's 64-code margin.
const UNDER_RANGE_CODE: u16 = 390 + 64;

/// The points a calibration of each range leaves, lowest first.
const CALIBRATION: [&[(u16, u32)]; 2] = [
    &[(410, 0), (31_200, 960_000), (62_100, 1_930_000)],
    &[(390, 0), (24_800, 98_000_000), (49_600, 197_000_000)],
];

#[derive(Clone, Debug, Default)]
pub struct SimEnergy {
    pub capture_id: Option<crate::CaptureId>,
    counter: u64,
    boot: u64,
    pub supply_on: bool,
    /// The capture since the last start, `None` before the first.
    pub capture: Option<Capture>,
    /// An acquisition fault, which stops integrating and lasts until a start succeeds.
    pub fault: bool,
    pub lose_next_edges: bool,
    pub dio_outputs: u8,
    pub dio_levels: u8,
    pub dio_held: u8,
    /// Verge Aux's stream of the capture.
    pub stream: Option<Stream>,
    /// `0x90` mode 14: the next start holds the high range, and clears this.
    pub high_range_next: bool,
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
    /// The capture holds the high range (`0x90` mode 14).
    pub high_range_only: bool,
}

impl Capture {
    fn new(since: Duration, rate: u32, high_range_only: bool) -> Self {
        Self {
            since,
            rate,
            stopped_at: None,
            banked: (0, 0),
            high_range_only,
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
    pub fn set_dio(&mut self, outputs: u8, levels: u8, held: u8, now: Duration) {
        let elapsed = self.capture.map_or(0, |capture| {
            now.saturating_sub(capture.since).as_micros() as u64
        });
        let before = self.marker(elapsed);
        let due = self.stream_messages(now);
        self.outbox.extend(due);
        self.dio_outputs = outputs;
        self.dio_levels = levels;
        self.dio_held = held;
        let after = self.marker(elapsed);
        if let Some(stream) = self.stream
            && (before ^ after) & stream.pins != 0
        {
            self.outbox.push(
                Edges::new(
                    0,
                    vec![Edge {
                        time_ns: elapsed * 1000,
                        levels: after & stream.pins,
                    }],
                )
                .to_bytes(),
            );
        }
    }

    /// What a restart leaves: supply off, no capture, no total.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Supply on and a capture running since `now` at `rate`.
    pub fn running(now: Duration, rate: u32) -> Self {
        let mut state = Self {
            supply_on: true,
            ..Self::default()
        };
        let mut request = vec![0];
        request.extend_from_slice(&rate.to_le_bytes());
        state.energy(true, now, &request);
        state
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
        match request {
            [8, rate @ ..] if rate.len() == 4 => {
                let mut legacy = vec![0];
                legacy.extend_from_slice(rate);
                let mut reply = self.energy(pro, now, &legacy);
                reply.extend_from_slice(if reply[2] == OK {
                    self.capture_id.as_ref().unwrap().as_bytes()
                } else {
                    &[0; 24]
                });
                return reply;
            }
            [11, bytes @ ..] if bytes.len() == 24 => {
                if !pro {
                    return vec![ENERGY_COMMAND, DAP_OK, UNAVAILABLE];
                }
                if self.capture_id.as_ref().map(|id| id.as_bytes().as_slice()) != Some(bytes) {
                    return vec![ENERGY_COMMAND, DAP_OK, 8];
                }
                return self.energy(pro, now, &[2]);
            }
            [12] => {
                let running = pro
                    && !self.fault
                    && self
                        .capture
                        .is_some_and(|capture| capture.stopped_at.is_none());
                let result = if !pro {
                    UNAVAILABLE
                } else if self.fault {
                    DMA_ERROR
                } else if running {
                    OK
                } else {
                    STOPPED
                };
                let mut reply = vec![ENERGY_COMMAND, DAP_OK, result, u8::from(running)];
                reply.extend_from_slice(
                    &if running {
                        self.capture.unwrap().rate
                    } else {
                        0
                    }
                    .to_le_bytes(),
                );
                reply.extend_from_slice(if pro {
                    self.capture_id
                        .as_ref()
                        .map_or(&[0; 24][..], |id| id.as_bytes().as_slice())
                } else {
                    &[0; 24]
                });
                return reply;
            }
            _ => {}
        }
        let mut reply = vec![ENERGY_COMMAND, DAP_OK];
        match request {
            [0, rate @ ..] if rate.len() == 4 => {
                let rate = u32::from_le_bytes(rate.try_into().expect("four bytes"));
                let result = if !pro {
                    UNAVAILABLE
                } else if rate == 0 || rate > MAX_RATE {
                    BAD_RATE
                } else if self.counter == u64::MAX {
                    9
                } else {
                    // The probe streams the replaced capture's last pairs before the end that
                    // says it restarted.
                    let tail = self.stream_messages(now);
                    self.outbox.extend(tail);
                    let high_range_only = std::mem::take(&mut self.high_range_next);
                    self.capture = Some(Capture::new(now, rate, high_range_only));
                    if self.boot == 0 {
                        static BOOT: std::sync::atomic::AtomicU64 =
                            std::sync::atomic::AtomicU64::new(1);
                        self.boot = BOOT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    }
                    self.counter += 1;
                    let mut bytes = [0; 24];
                    bytes[..8].copy_from_slice(&self.boot.to_le_bytes());
                    bytes[16..].copy_from_slice(&self.counter.to_le_bytes());
                    self.capture_id = crate::CaptureId::from_bytes(bytes);
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
            [14, ranges] => reply.push(if !pro {
                UNAVAILABLE
            } else if *ranges > 1 {
                BAD_ARGUMENT
            } else {
                self.high_range_next = *ranges == 1;
                OK
            }),
            _ => reply[1] = DAP_ERROR,
        }
        reply
    }

    /// Pair `n`'s codes and range, as the probe would read the target's current through its sense
    /// resistor, the voltage divider's current included.
    fn pair(&self, capture: &Capture, n: u64) -> ([u8; 5], bool) {
        let millivolts = self.millivolts();
        let shunt = self.nanoamps_at(capture.micros(n)) + ((millivolts * 99_297) >> 16);
        let low = !capture.high_range_only && shunt <= LOW_RANGE_NANOAMPS;
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
        let under = !low && current_code < UNDER_RANGE_CODE;
        ([v0, v1, c0, c1, u8::from(low) | u8::from(under) << 1], low)
    }

    /// The marker's level `micros` into the capture.
    fn marker(&self, micros: u64) -> u8 {
        let target = if self.supply_on && micros % PERIOD_MICROS < AWAKE_MICROS {
            MARKER
        } else {
            0
        };
        ((target | self.dio_held) & !self.dio_outputs) | (self.dio_levels & self.dio_outputs)
    }

    /// The edges of `pins` after `after` µs into the capture and up to `until`, as one message
    /// each 99, flagged as the probe flags them.
    fn edges(&mut self, pins: u8, after: u64, until: u64) -> Vec<Vec<u8>> {
        if pins & MARKER == 0 || !self.supply_on || until <= after {
            return Vec::new();
        }
        let mut edges = Vec::new();
        let mut period = after / PERIOD_MICROS * PERIOD_MICROS;
        while period <= until {
            for at in [period, period + AWAKE_MICROS] {
                if at > after
                    && at <= until
                    && (self.marker(at) ^ self.marker(at.saturating_sub(1))) & pins != 0
                {
                    edges.push(Edge {
                        time_ns: at * 1_000,
                        levels: self.marker(at) & pins,
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
            .map(|edges| {
                Edges::new(
                    if std::mem::take(&mut self.lose_next_edges) {
                        Edges::LOST
                    } else {
                        0
                    },
                    edges,
                )
                .to_bytes()
            })
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
        if capture.high_range_only {
            // Step 5's fields: no weight, the high range held (flag bit 2), no current before.
            header.extend_from_slice(&[0, 0, 4, 0, 0, 0, 0]);
        }
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
            messages.push(Self::header(
                &capture,
                1,
                !capture.high_range_only,
                pins,
                self.marker(0),
            ));
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

    pub fn identified_stream(
        &mut self,
        pro: bool,
        now: Duration,
        rate: Option<[u8; 4]>,
        pins: u8,
    ) -> Vec<Vec<u8>> {
        let messages = match rate {
            Some(rate) => self.start_stream(pro, now, rate, pins),
            None => self.stream_running(pro, now, pins),
        };
        let Some(mut reply) = messages
            .iter()
            .find(|message| message.first() == Some(&ENERGY_COMMAND))
            .cloned()
        else {
            return messages;
        };
        if reply.get(2) != Some(&OK) {
            if rate.is_some() {
                reply.extend_from_slice(&[0; 24]);
                return vec![reply];
            }
            return messages;
        }
        let id = self
            .capture_id
            .expect("successful start/attach has an identity");
        if rate.is_some() {
            reply.extend_from_slice(id.as_bytes());
        }
        let mut header = messages
            .into_iter()
            .rev()
            .find(|message| message.first() == Some(&0xa1))
            .expect("successful stream has a header");
        header[1] = 2;
        // The identity goes before step 5's fields, where format 2 puts it.
        header.splice(68..68, id.as_bytes().iter().copied());
        vec![reply, header]
    }

    pub fn stop_stream_identified(
        &mut self,
        pro: bool,
        now: Duration,
        bytes: &[u8],
    ) -> Vec<Vec<u8>> {
        if !pro {
            return vec![vec![ENERGY_COMMAND, DAP_OK, UNAVAILABLE]];
        }
        if bytes.len() != 24 {
            return vec![vec![crate::dap::ID_DAP_INVALID]];
        }
        if self.capture_id.as_ref().map(|id| id.as_bytes().as_slice()) != Some(bytes) {
            return vec![vec![ENERGY_COMMAND, DAP_OK, 8]];
        }
        self.stop_stream(pro, now)
    }

    /// `0x94` (`request` after the command byte): a marker stamped `now` goes into the stream, after
    /// the pairs due by then. Its reply. The probe keeps 16 waiting; here they never wait.
    pub fn mark(&mut self, pro: bool, now: Duration, request: &[u8]) -> Vec<u8> {
        let [n0, n1, n2, n3, length, label @ ..] = request else {
            return vec![crate::dap::ID_DAP_INVALID];
        };
        let length = usize::from(*length);
        if label.len() < length {
            return vec![crate::dap::ID_DAP_INVALID];
        }
        if !pro || length > LABEL_MAX {
            return vec![MARK_COMMAND, DAP_ERROR];
        }
        let due = self.stream_messages(now);
        self.outbox.extend(due);
        let running = self
            .capture
            .filter(|capture| capture.stopped_at.is_none() && !self.fault);
        let result = match (running, self.stream) {
            (None, _) => MARK_NO_CAPTURE,
            (Some(_), None) => MARK_NO_STREAM,
            (Some(capture), Some(_)) => {
                let marker = Marker {
                    time_ns: now.saturating_sub(capture.since).as_nanos() as u64,
                    number: u32::from_le_bytes([*n0, *n1, *n2, *n3]),
                    label: label[..length].to_vec(),
                };
                self.outbox.push(Markers::new(0, vec![marker]).to_bytes());
                MARK_SENT
            }
        };
        vec![MARK_COMMAND, DAP_OK, result]
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

#[cfg(test)]
mod identity_tests {
    use super::*;

    #[test]
    fn counter_exhaustion_refuses_start_without_disturbing_the_capture() {
        let mut energy = SimEnergy::running(Duration::ZERO, 1000);
        let token = energy.capture_id.unwrap();
        energy.counter = u64::MAX;
        let reply = energy.energy(true, Duration::from_secs(1), &[8, 0xe8, 3, 0, 0]);
        assert_eq!(&reply[..3], &[0x90, 0, 9]);
        assert!(reply[3..].iter().all(|&byte| byte == 0));
        assert_eq!(energy.capture_id, Some(token));
        assert!(energy.capture.unwrap().stopped_at.is_none());
        let status = energy.energy(true, Duration::from_secs(1), &[12]);
        assert_eq!(&status[..4], &[0x90, 0, 0, 1]);
        assert_eq!(&status[8..], token.as_bytes());
    }
}
