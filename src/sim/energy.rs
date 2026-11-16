//! A simulated Pro's target supply (`0x8F`) and energy capture (`0x90`), as docs/host-interface.md
//! describes them.
//!
//! The target is a sleeping microcontroller that wakes for 10 ms every 100 ms: 20 µA asleep and
//! 5 mA awake, at 3.3 V while the Pro's supply is on and nothing while it is off. It holds DIO3 high
//! while awake, a marker a stream can report. Its values are made up; only their shape matters to a
//! host.

use std::time::Duration;

use crate::protocol::{ENERGY_COMMAND, MARK_COMMAND, TARGET_POWER_COMMAND};
use crate::stream::{DioLost, DioTime, Edge, Edges, LABEL_MAX, Marker, Markers};

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
    /// A DIO stream (`0x90` mode 15) on a board without energy capture: when it started, and the
    /// pins it reports.
    pub dio_stream: Option<(Duration, u8)>,
    /// Input pins something attached toggles every 50 ms, which a DIO stream reports; a restart
    /// keeps them.
    pub dio_toggle: u8,
    /// How far into the DIO stream its toggles have gone, and the input pins they left flipped.
    toggled_at: Duration,
    toggled: u8,
    /// The DIO stream's latest time, in µs: a later message comes after it, as the probe's do.
    dio_stamped: Option<u64>,
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
/// How often `dio_toggle`'s pins change.
const TOGGLE_EVERY: Duration = Duration::from_millis(50);
/// How long a DIO stream stays quiet before it sends its time, as the probe's does.
const QUIET_FOR_US: u64 = 200_000;
/// The resolution a stream header gives for edges, as the Pro's 96 MHz stamping clock's.
const EDGE_RESOLUTION_NS: u16 = 11;
/// A DIO stream's clock: the simulator times its edges to the microsecond.
const DIO_CLOCK_HZ: u32 = 1_000_000;

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
        if let Some((since, pins)) = self.dio_stream {
            let toggles = self.dio_toggles(now);
            self.outbox.extend(toggles);
            let before = self.dio_read();
            self.dio_outputs = outputs;
            self.dio_levels = levels;
            self.dio_held = held;
            let after = self.dio_read();
            if (before ^ after) & pins != 0 {
                let at = self.dio_stamp(now.saturating_sub(since).as_micros() as u64);
                let edge = Edge {
                    time_ns: at * 1000,
                    levels: after & pins,
                };
                self.outbox.push(Edges::new(0, vec![edge]).to_bytes());
            }
            return;
        }
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
        *self = Self {
            dio_toggle: self.dio_toggle,
            ..Self::default()
        };
    }

    /// The pins as the probe reads them without an energy capture: what drives or holds each, with
    /// `dio_toggle`'s inputs flipped as far as they have gone in a DIO stream.
    fn dio_read(&self) -> u8 {
        let driven = (self.dio_held & !self.dio_outputs) | (self.dio_levels & self.dio_outputs);
        driven ^ (self.toggled & !self.dio_outputs)
    }

    /// The pins `dio_toggle`'s toggles have flipped, inputs or not.
    pub(super) fn toggled(&self) -> u8 {
        self.toggled
    }

    /// A DIO stream's time for something at `micros`: after the latest, as a later interrupt
    /// stamps later.
    fn dio_stamp(&mut self, micros: u64) -> u64 {
        let at = self.dio_stamped.map_or(micros, |last| micros.max(last + 1));
        self.dio_stamped = Some(at);
        at
    }

    /// A DIO stream's time once nothing has been stamped for 200 ms: now, after all it sent. As
    /// the probe's, none while edges wait to be sent, and none for a stream of no pins.
    fn dio_time(&mut self, now: Duration) -> Option<Vec<u8>> {
        let (since, pins) = self.dio_stream?;
        let elapsed = now.saturating_sub(since);
        let micros = elapsed.as_micros() as u64;
        let toggles_due = self.dio_toggle & pins != 0 && self.toggled_at + TOGGLE_EVERY <= elapsed;
        if pins == 0 || toggles_due || micros < self.dio_stamped.unwrap_or(0) + QUIET_FOR_US {
            return None;
        }
        let at = self.dio_stamp(micros);
        Some(DioTime { time_ns: at * 1000 }.to_bytes())
    }

    /// The edges of the pins `dio_toggle` names, up to `now` into a DIO stream. What is attached
    /// keeps toggling while the probe drives a pin, which reads its drive meanwhile. With
    /// `lose_next_edges`, the first is lost: a DIO lost message from just after the latest time,
    /// or the header's, stands for it.
    fn dio_toggles(&mut self, now: Duration) -> Vec<Vec<u8>> {
        let Some((since, pins)) = self.dio_stream else {
            return Vec::new();
        };
        let toggle = self.dio_toggle & pins;
        let mut out = Vec::new();
        let mut edges: Vec<Edge> = Vec::new();
        // At most 99 a call, and each message spans what its 32-bit offsets hold, as the probe's.
        let mut sent = 0;
        while toggle != 0
            && sent < 99
            && self.toggled_at + TOGGLE_EVERY <= now.saturating_sub(since)
        {
            self.toggled_at += TOGGLE_EVERY;
            self.toggled ^= toggle;
            if toggle & !self.dio_outputs == 0 {
                continue;
            }
            let from = self.dio_stamped.map_or(1, |last| last + 1);
            let at = self.dio_stamp(self.toggled_at.as_micros() as u64);
            let levels = self.dio_read() & pins;
            if std::mem::take(&mut self.lose_next_edges) {
                out.push(
                    DioLost {
                        from_ns: from * 1000,
                        to_ns: at * 1000,
                        levels,
                    }
                    .to_bytes(),
                );
                continue;
            }
            if edges
                .first()
                .is_some_and(|first| at * 1000 - first.time_ns > u64::from(u32::MAX))
            {
                out.push(Edges::new(0, std::mem::take(&mut edges)).to_bytes());
            }
            edges.push(Edge {
                time_ns: at * 1000,
                levels,
            });
            sent += 1;
        }
        if !edges.is_empty() {
            out.push(Edges::new(0, edges).to_bytes());
        }
        out
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
        let mut out = Vec::new();
        for mut edges in messages {
            // A loss as the probe reports it: edges from the window's start may be missing, and the
            // levels are known again at what would have been the first edge, which the loss stands
            // for.
            if std::mem::take(&mut self.lose_next_edges) {
                let first = edges.remove(0);
                out.push(
                    DioLost {
                        from_ns: after * 1_000,
                        to_ns: first.time_ns,
                        levels: first.levels,
                    }
                    .to_bytes(),
                );
            }
            if !edges.is_empty() {
                out.push(Edges::new(0, edges).to_bytes());
            }
        }
        out
    }

    fn header(
        capture: &Capture,
        first_sequence: u32,
        previous_low: bool,
        pins: u8,
        levels: u8,
    ) -> Vec<u8> {
        // The simulator's timer runs at the rate itself: one count a pair.
        let mut header = vec![0xA1, 3];
        header.extend_from_slice(&capture.rate.to_le_bytes());
        header.extend_from_slice(&1u32.to_le_bytes());
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
        // Step 5's fields: no weight and no current before; flag bit 2 when the high range is held.
        header.extend_from_slice(&[
            0,
            0,
            if capture.high_range_only { 4 } else { 0 },
            0,
            0,
            0,
            0,
        ]);
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
        messages.extend(self.dio_toggles(now));
        messages.extend(self.dio_time(now));
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
        header[1] = 4;
        // The identity goes before step 5's fields, where format 4 puts it.
        header.splice(72..72, id.as_bytes().iter().copied());
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
        self.stop_stream(now)
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

    /// `0x90` mode 7: the stream stops, the capture carries on. The reply, then the end. A board
    /// without energy capture answers it too: it ends a DIO stream there.
    pub fn stop_stream(&mut self, now: Duration) -> Vec<Vec<u8>> {
        let mut messages = self.stream_messages(now);
        messages.push(vec![ENERGY_COMMAND, DAP_OK, OK]);
        if let Some(stream) = self.stream.take() {
            messages.push(Self::end(ASKED, stream.next));
        }
        messages.extend(self.end_dio_stream(now));
        messages
    }

    /// A DIO stream's end, which says when: no pair marks it.
    fn end_dio_stream(&mut self, now: Duration) -> Option<Vec<u8>> {
        let (since, _) = self.dio_stream.take()?;
        let mut end = Self::end(ASKED, 0);
        let at = (now.saturating_sub(since).as_micros() as u64).max(self.dio_stamped.unwrap_or(0));
        end.extend_from_slice(&(at * 1000).to_le_bytes());
        Some(end)
    }

    /// `0x90` mode 15: a DIO stream of `pins`, where the board has DIO pins and no energy capture.
    /// It ends one already running. The reply, then the header.
    pub fn start_dio_stream(
        &mut self,
        energy: bool,
        dio: bool,
        now: Duration,
        pins: u8,
    ) -> Vec<Vec<u8>> {
        let mut messages = self.stream_messages(now);
        if energy || !dio {
            messages.push(vec![ENERGY_COMMAND, DAP_OK, UNAVAILABLE]);
            return messages;
        }
        // The end goes with the queue the reply drops.
        let _ = self.end_dio_stream(now);
        let pins = pins & 0x0F;
        messages.push(vec![ENERGY_COMMAND, DAP_OK, OK]);
        let mut header = vec![0xA1, 5];
        header.extend_from_slice(&DIO_CLOCK_HZ.to_le_bytes());
        self.toggled_at = Duration::ZERO;
        self.toggled = 0;
        self.dio_stamped = None;
        header.extend_from_slice(&[pins, self.dio_read() & pins]);
        let resolution: u16 = if pins == 0 { 0 } else { 1000 };
        header.extend_from_slice(&resolution.to_le_bytes());
        messages.push(header);
        self.dio_stream = Some((now, pins));
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

#[cfg(test)]
mod dio_stream_tests {
    use super::*;
    use crate::stream::{Message, is_stream_message, parse};

    /// A DIO stream's messages after its header, parsed.
    fn parsed(messages: &[Vec<u8>]) -> Vec<Message> {
        messages
            .iter()
            .filter(|m| is_stream_message(m))
            .map(|m| parse(m).unwrap())
            .filter(|m| !matches!(m, Message::DioHeader(_)))
            .collect()
    }

    /// Each edges message's edges, as (µs, levels).
    fn edges(messages: &[Message]) -> Vec<(u64, u8)> {
        messages
            .iter()
            .flat_map(|m| match m {
                Message::Edges(message) => message
                    .edges
                    .iter()
                    .map(|e| (e.time_ns / 1000, e.levels))
                    .collect(),
                _ => Vec::new(),
            })
            .collect()
    }

    /// What a DIO stream sends in a later message comes after the last one's time, as the probe
    /// stamps a later interrupt later. Here: an edge, a lost toggle, which starts just after that
    /// edge, a pin driven three times in one microsecond, and the toggle due in the next one,
    /// which comes after them. The end comes no earlier than the last edge.
    #[test]
    fn a_dio_streams_later_messages_come_after_the_last_ones_time() {
        let mut energy = SimEnergy {
            dio_toggle: 0b0010,
            ..SimEnergy::default()
        };
        let start = Duration::from_secs(1);
        let mut messages = energy.start_dio_stream(false, true, start, 0x0F);
        energy.set_dio(0b0001, 0b0001, 0, start + Duration::from_millis(10));
        energy.lose_next_edges = true;
        let at = start + 2 * TOGGLE_EVERY - Duration::from_micros(1);
        for level in [0b0000, 0b0001, 0b0000] {
            energy.set_dio(0b0001, level, 0, at);
        }
        messages.extend(energy.stream_messages(start + 2 * TOGGLE_EVERY));
        messages.extend(energy.end_dio_stream(start + 2 * TOGGLE_EVERY));
        let messages = parsed(&messages);
        let (mut last, mut lost) = (None::<u64>, Vec::new());
        for message in &messages {
            let (first, latest) = match message {
                Message::Edges(message) => (
                    message.edges[0].time_ns,
                    message.edges.last().unwrap().time_ns,
                ),
                Message::DioLost(loss) => {
                    lost.push((loss.from_ns / 1000, loss.to_ns / 1000));
                    (loss.from_ns, loss.to_ns)
                }
                Message::End(end) => {
                    assert!(end.time_ns >= last, "{end:?} before {last:?}");
                    continue;
                }
                other => panic!("{other:?}"),
            };
            assert!(
                last.is_none_or(|last| first > last),
                "{first} after {last:?}"
            );
            last = Some(latest);
        }
        assert_eq!(lost, [(10_001, 50_000)]);
        let times: Vec<u64> = edges(&messages).iter().map(|&(t, _)| t).collect();
        assert_eq!(times, [10_000, 99_999, 100_000, 100_001, 100_002]);
    }

    /// A stream's first loss starts after its header's time, as the probe's do: a pin that starts
    /// high is known high at the start.
    #[test]
    fn a_dio_streams_first_loss_starts_after_its_header() {
        let mut energy = SimEnergy {
            dio_toggle: 0b0010,
            lose_next_edges: true,
            ..SimEnergy::default()
        };
        let start = Duration::from_secs(1);
        let mut messages = energy.start_dio_stream(false, true, start, 0x0F);
        messages.extend(energy.stream_messages(start + TOGGLE_EVERY));
        let Some(Message::DioLost(loss)) = parsed(&messages).into_iter().next() else {
            panic!("no loss");
        };
        assert!(loss.from_ns > 0 && loss.from_ns <= loss.to_ns, "{loss:?}");
    }

    /// Without an energy capture there is no wake marker: a Pro's supply turned on leaves its DIO
    /// pins as what holds them.
    #[test]
    fn a_dio_stream_has_no_wake_marker() {
        let mut energy = SimEnergy {
            supply_on: true,
            ..SimEnergy::default()
        };
        let messages = energy.start_dio_stream(false, true, Duration::from_secs(1), 0x0F);
        let Some(Message::DioHeader(header)) = messages
            .iter()
            .filter(|m| is_stream_message(m))
            .map(|m| parse(m).unwrap())
            .next()
        else {
            panic!("no header");
        };
        assert_eq!(header.dio_levels, 0);
    }

    /// A DIO stream quiet for 200 ms sends its time, now, and again 200 ms after that; any
    /// message stamped meanwhile starts the quiet again.
    #[test]
    fn a_quiet_dio_stream_sends_its_time() {
        let mut energy = SimEnergy::default();
        let start = Duration::from_secs(1);
        energy.start_dio_stream(false, true, start, 0x0F);
        let times = |messages: &[Vec<u8>]| -> Vec<u64> {
            parsed(messages)
                .iter()
                .filter_map(|m| match m {
                    Message::DioTime(time) => Some(time.time_ns / 1000),
                    _ => None,
                })
                .collect()
        };
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_micros(199_999))),
            [0u64; 0]
        );
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_millis(200))),
            [200_000]
        );
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_millis(399))),
            [0u64; 0]
        );
        energy.set_dio(0b0001, 0b0001, 0, start + Duration::from_millis(390));
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_millis(589))),
            [0u64; 0]
        );
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_millis(590))),
            [590_000]
        );

        // A stream of no pins says nothing.
        let mut energy = SimEnergy::default();
        energy.start_dio_stream(false, true, start, 0);
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_secs(1))),
            [0u64; 0]
        );
        // Six seconds of toggles are more than one call sends: the time waits for the rest.
        let mut energy = SimEnergy {
            dio_toggle: 0b0010,
            ..SimEnergy::default()
        };
        energy.start_dio_stream(false, true, start, 0x0F);
        assert_eq!(
            times(&energy.stream_messages(start + Duration::from_secs(6))),
            [0u64; 0]
        );
    }

    /// A toggling pin a host drives reads its drive. What is attached keeps toggling meanwhile, so
    /// the pin shows where that has got to once the host lets go, with no toggles replayed.
    #[test]
    fn a_toggled_pin_driven_reads_its_drive() {
        let mut energy = SimEnergy {
            dio_toggle: 0b0010,
            ..SimEnergy::default()
        };
        let start = Duration::from_secs(1);
        let mut messages = energy.start_dio_stream(false, true, start, 0x0F);
        energy.set_dio(0b0010, 0b0000, 0, start + Duration::from_millis(70));
        energy.set_dio(0, 0, 0, start + Duration::from_millis(1170));
        messages.extend(energy.stream_messages(start + Duration::from_millis(1180)));
        // High at the toggle at 50 ms, driven low at 70 ms, then 22 toggles while driven leave it
        // high when let go at 1,170 ms.
        assert_eq!(
            edges(&parsed(&messages)),
            [(50_000, 0b0010), (70_000, 0b0000), (1_170_000, 0b0010)]
        );
    }
}
