//! A recording's pairs, pins and markers out to a CSV or sigrok session: whole pairs whose support
//! overlaps the interval, with their own times, and the pins' levels and markers over it.

use super::{Error, Result, header_from, nanos};
use crate::record::{Sink, temp_beside};
use crate::stream::{CurrentRange, Header, Sample};
use escapement_tickwave_view::Recording;
use escapement_tickwave_view::runs::{Runs, place_after};
use escapement_tickwave_view::spans::ChannelPlace;
use std::collections::BTreeMap;
use std::ops::Range;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};

/// How many pair periods one pass reads.
const WINDOW_PAIRS: i64 = 1 << 16;
/// Markers asked for at a time.
const MARKER_PAGE: usize = 512;

fn cancelled(cancel: &AtomicBool) -> Result<()> {
    if cancel.load(Ordering::Relaxed) {
        return Err(Error::Cancelled);
    }
    Ok(())
}

/// What the export reads of a recording a Recorder wrote.
struct Layout<'a> {
    recording: &'a Recording,
    header: Header,
    extra: &'a BTreeMap<String, String>,
    /// The probe's ns at tick 0.
    base: i64,
    v: usize,
    i: usize,
    low: usize,
    under: usize,
    /// Each reported pin, lowest first, and its channel.
    pins: Vec<(u8, usize)>,
}

impl<'a> Layout<'a> {
    fn new(recording: &'a Recording) -> Result<Self> {
        let source = recording.sources().first().ok_or(Error::NotVerge)?;
        let header = header_from(&source.extra).ok_or(Error::NotVerge)?;
        let channel = |name: &str| recording.channel(name).ok_or(Error::NotVerge);
        let mut pins = Vec::new();
        for k in (0..4).filter(|k| header.dio_mask & (1 << k) != 0) {
            pins.push((k, channel(&format!("DIO{k}"))?));
        }
        Ok(Self {
            recording,
            base: -source.origin,
            extra: &source.extra,
            v: channel("V")?,
            i: channel("I")?,
            low: channel("low_range")?,
            under: channel("under_range")?,
            pins,
            header,
        })
    }

    fn ns(&self, tick: i64) -> u64 {
        nanos(tick + self.base, self.header.clock_hz)
    }

    fn count(&self, key: &str) -> u64 {
        self.extra
            .get(key)
            .and_then(|n| n.parse().ok())
            .unwrap_or(0)
    }

    fn runs(&self) -> Result<Runs<'a>> {
        Ok(self.recording.runs(self.v)?)
    }
}

/// A pin's history from a tick on: its level, whether it is known, and what changes it next.
struct Line {
    channel: usize,
    level: bool,
    known: bool,
    /// Sorted by tick, then by kind: what is unknown from, the level known again, then an edge.
    events: Vec<(i64, u8, Option<bool>)>,
    next: usize,
}

const UNKNOWN: u8 = 0;
const KNOWN: u8 = 1;
const EDGE: u8 = 2;

impl Line {
    /// The line as it is at `from`, before anything at `from` changes it. A gap from tick 0 reads
    /// as known at its level: the Recorder gives it the header's levels, which an export takes as
    /// the pins' levels until the stream's first report.
    fn at(layout: &Layout, channel: usize, from: i64, cancel: &AtomicBool) -> Result<Self> {
        let ChannelPlace::Digital { start, gaps, .. } = layout.recording.known(channel)? else {
            return Err(Error::NotVerge);
        };
        let known = !gaps.iter().any(|g| g.from <= from && from < g.to);
        let (level, _) =
            layout
                .recording
                .edges(channel, from.max(start), from.max(start) + 1, cancel)?;
        Ok(Self {
            channel,
            level,
            known,
            events: Vec::new(),
            next: 0,
        })
    }

    /// Takes in what changes the line in [from, to).
    fn load(&mut self, layout: &Layout, from: i64, to: i64, cancel: &AtomicBool) -> Result<()> {
        let (_, edges) = layout.recording.edges(self.channel, from, to, cancel)?;
        let mut events: Vec<(i64, u8, Option<bool>)> =
            edges.into_iter().map(|t| (t, EDGE, None)).collect();
        let ChannelPlace::Digital { gaps, .. } = layout.recording.known(self.channel)? else {
            return Err(Error::NotVerge);
        };
        for g in gaps {
            if (from..to).contains(&g.from) {
                events.push((g.from, UNKNOWN, None));
            }
            if (from..to).contains(&g.to) {
                events.push((g.to, KNOWN, Some(g.level)));
            }
        }
        events.sort_by_key(|&(t, kind, _)| (t, kind));
        self.events = events;
        self.next = 0;
        Ok(())
    }

    /// Takes every change at or before `tick`.
    fn advance(&mut self, tick: i64) {
        while let Some(&(t, kind, level)) = self.events.get(self.next) {
            if t > tick {
                break;
            }
            match kind {
                UNKNOWN => self.known = false,
                KNOWN => {
                    self.known = true;
                    self.level = level.expect("a known level");
                }
                _ => self.level = !self.level,
            }
            self.next += 1;
        }
    }

    /// Takes what is unknown or known again at `tick`, and the edges before it; returns how many
    /// edges come at `tick`, not yet taken.
    fn status_at(&mut self, tick: i64) -> usize {
        self.advance(tick - 1);
        while let Some(&(t, kind, level)) = self.events.get(self.next) {
            if t != tick || kind == EDGE {
                break;
            }
            self.known = kind == KNOWN;
            if let Some(level) = level {
                self.level = level;
            }
            self.next += 1;
        }
        self.events[self.next..]
            .iter()
            .take_while(|e| e.0 == tick)
            .count()
    }

    /// Takes one edge.
    fn toggle(&mut self) {
        self.level = !self.level;
        self.next += 1;
    }

    /// The next change's tick.
    fn upcoming(&self) -> Option<i64> {
        self.events.get(self.next).map(|e| e.0)
    }
}

/// Every pin together: the levels as one mask, or none while any pin is unknown.
struct Pins {
    lines: Vec<(u8, Line)>,
}

impl Pins {
    fn at(layout: &Layout, from: i64, cancel: &AtomicBool) -> Result<Self> {
        let mut lines = Vec::new();
        for &(k, c) in &layout.pins {
            lines.push((k, Line::at(layout, c, from, cancel)?));
        }
        Ok(Self { lines })
    }
    fn load(&mut self, layout: &Layout, from: i64, to: i64, cancel: &AtomicBool) -> Result<()> {
        for (_, line) in &mut self.lines {
            line.load(layout, from, to, cancel)?;
        }
        Ok(())
    }
    fn levels(&self) -> Option<u8> {
        let mut levels = 0;
        for (k, line) in &self.lines {
            if !line.known {
                return None;
            }
            levels |= u8::from(line.level) << k;
        }
        Some(levels)
    }
    fn upcoming(&self) -> Option<i64> {
        self.lines.iter().filter_map(|(_, l)| l.upcoming()).min()
    }
    fn advance(&mut self, tick: i64) {
        for (_, line) in &mut self.lines {
            line.advance(tick);
        }
    }
    /// The levels after each change at `tick`: what is unknown or known again, then the edges a
    /// round at a time, one a pin each round. A stream's edge report can move a pin twice at one
    /// time, a pulse shorter than its resolution, and each report reads as its own change.
    fn changes_at(&mut self, tick: i64) -> Vec<Option<u8>> {
        let edges: Vec<usize> = self
            .lines
            .iter_mut()
            .map(|(_, line)| line.status_at(tick))
            .collect();
        let mut states = vec![self.levels()];
        for round in 0..edges.iter().copied().max().unwrap_or(0) {
            for ((_, line), &n) in self.lines.iter_mut().zip(&edges) {
                if round < n {
                    line.toggle();
                }
            }
            states.push(self.levels());
        }
        states
    }
}

/// Pairs by index, with their sequence numbers and ranges.
struct Pairs<'a> {
    layout: &'a Layout<'a>,
    runs: Runs<'a>,
    /// The sequence number of the pair before index 0's, as the stream counted.
    first_sequence: u32,
    low: Line,
    under: Line,
}

impl<'a> Pairs<'a> {
    fn new(layout: &'a Layout<'a>, from: i64, cancel: &AtomicBool) -> Result<Self> {
        // The first pair is the header's first sequence plus what was lost before it.
        let first_sequence = layout
            .header
            .first_sequence
            .wrapping_add(layout.recording.lost_before(layout.v) as u32);
        Ok(Self {
            layout,
            runs: layout.runs()?,
            first_sequence,
            low: Line::at(layout, layout.low, from, cancel)?,
            under: Line::at(layout, layout.under, from, cancel)?,
        })
    }

    /// The pairs `i0..i1`, whose ticks lie in [from, to), in order.
    fn read(
        &mut self,
        i0: u64,
        i1: u64,
        from: i64,
        to: i64,
        cancel: &AtomicBool,
    ) -> Result<Vec<(i64, Sample)>> {
        let l = self.layout;
        self.low.load(l, from, to, cancel)?;
        self.under.load(l, from, to, cancel)?;
        let mut codes = [Vec::new(), Vec::new()];
        for (k, c) in [l.v, l.i].into_iter().enumerate() {
            let out = &mut codes[k];
            out.reserve((i1 - i0) as usize);
            l.recording.each_sample(c, i0, i1, cancel, |_, _, code| {
                out.push(code);
                Ok(())
            })?;
        }
        let mut lost = self
            .runs
            .breaks
            .iter()
            .take_while(|b| b.index <= i0)
            .map(|b| b.lost)
            .sum::<u64>();
        let mut breaks = self.runs.breaks.partition_point(|b| b.index <= i0);
        let mut pairs = Vec::with_capacity((i1 - i0) as usize);
        for (n, index) in (i0..i1).enumerate() {
            while let Some(b) = self.runs.breaks.get(breaks)
                && b.index <= index
            {
                lost += b.lost;
                breaks += 1;
            }
            let tick = self.runs.tick(index);
            self.low.advance(tick);
            self.under.advance(tick);
            let code = |k: usize| {
                codes[k]
                    .get(n)
                    .copied()
                    .flatten()
                    .and_then(|c| u32::try_from(c).ok())
                    .ok_or(Error::NotVerge)
            };
            pairs.push((
                tick,
                Sample {
                    sequence: self
                        .first_sequence
                        .wrapping_add(index as u32)
                        .wrapping_add(lost as u32),
                    time_ns: l.ns(tick),
                    millivolts: code(0)?,
                    nanoamps: code(1)?,
                    range: Some(if self.low.level {
                        CurrentRange::Low
                    } else {
                        CurrentRange::High
                    }),
                    under_range: self.under.level,
                },
            ));
        }
        Ok(pairs)
    }
}

/// The index of the first pair whose support ends after `tick`.
fn first_ending_after(layout: &Layout, runs: &Runs, tick: i64) -> Result<u64> {
    let j = layout.recording.index_at(layout.v, tick)?;
    Ok(if j > 0 && place_after(runs, j - 1) > tick {
        j - 1
    } else {
        j
    })
}

/// Writes the pairs whose support overlaps `interval`, in ticks, whole and with their own times,
/// and the pins' levels and the markers over it, to a new CSV or sigrok session at `path`.
pub fn export(
    recording: &Recording,
    path: &Path,
    csv: bool,
    interval: Range<i64>,
    cancel: &AtomicBool,
) -> Result<()> {
    if path.exists() {
        return Err(Error::Exists(path.to_owned()));
    }
    let layout = Layout::new(recording)?;
    // Only the sink holds the file open, so a CSV marker merge can rename over it.
    let (file, temp) = temp_beside(path)?.into_parts();
    let mut sink = Sink::from_file(file, &temp, csv)?;
    let pins = if layout.pins.is_empty() {
        0
    } else {
        layout.header.dio_mask
    };
    if recording.marker_count() as u64 + layout.count("marker_losses") > 0 {
        sink.include_markers()?;
    }
    sink.begin_with_dio_gaps(layout.header.whole_rate(), pins, layout.header.dio_levels)?;
    let extent = recording.extent().unwrap_or(0..0);
    let clipped = interval.start.max(extent.start)..interval.end.min(extent.end);
    export_markers(&layout, &mut sink, clipped.clone(), cancel)?;
    if pins == 0 {
        export_pairs(&layout, &mut sink, interval, cancel)?;
    } else {
        export_with_pins(&layout, &mut sink, clipped, cancel)?;
    }
    sink.finish_with_cancel(cancel)?;
    cancelled(cancel)?;
    temp.persist_noclobber(path).map_err(|error| error.error)?;
    Ok(())
}

fn export_markers(
    layout: &Layout,
    sink: &mut Sink,
    interval: Range<i64>,
    cancel: &AtomicBool,
) -> Result<()> {
    let recording = layout.recording;
    let is_marker = |kind: Option<&str>| matches!(kind, Some("marker" | "marker-after-loss"));
    if interval.end > interval.start {
        let mut after = None;
        loop {
            cancelled(cancel)?;
            let page = recording.markers(interval.clone(), after, MARKER_PAGE);
            for marker in &page.entries {
                let a = &marker.annotation;
                if !is_marker(a.kind.as_deref()) {
                    continue;
                }
                let number = a
                    .values
                    .as_ref()
                    .and_then(|v| v.get("number"))
                    .and_then(|n| n.parse().ok())
                    .ok_or(Error::NotVerge)?;
                sink.marker_text(
                    layout.ns(a.start),
                    number,
                    a.label.as_deref().unwrap_or(""),
                    a.kind.as_deref() == Some("marker-after-loss"),
                )?;
            }
            after = page.next;
            if after.is_none() {
                break;
            }
        }
    }
    // The loss reports that came with no marker have no time, so every export carries them.
    let all = recording.markers(i64::MIN..i64::MAX, None, usize::MAX);
    let attached = all
        .entries
        .iter()
        .filter(|m| m.annotation.kind.as_deref() == Some("marker-after-loss"))
        .count() as u64;
    for _ in attached..layout.count("marker_losses") {
        sink.marker_loss();
    }
    Ok(())
}

/// Ticks in windows of so many pair periods, from `from` to `to`.
fn windows(layout: &Layout, from: i64, to: i64) -> impl Iterator<Item = (i64, i64)> {
    let width = (i64::from(layout.header.period) * WINDOW_PAIRS).max(1);
    let mut at = from;
    std::iter::from_fn(move || {
        (at < to).then(|| {
            let w = (at, at.saturating_add(width).min(to));
            at = w.1;
            w
        })
    })
}

/// A capture with no pins: the pairs whose support overlaps `interval`.
fn export_pairs(
    layout: &Layout,
    sink: &mut Sink,
    interval: Range<i64>,
    cancel: &AtomicBool,
) -> Result<()> {
    let runs = layout.runs()?;
    if runs.count == 0 || interval.end <= interval.start {
        return Ok(());
    }
    let i0 = first_ending_after(layout, &runs, interval.start)?;
    let i1 = layout.recording.index_at(layout.v, interval.end)?;
    if i1 <= i0 {
        return Ok(());
    }
    let (from, to) = (runs.tick(i0), runs.tick(i1 - 1) + 1);
    let mut pairs = Pairs::new(layout, from, cancel)?;
    for (w0, w1) in windows(layout, from, to) {
        cancelled(cancel)?;
        let a = layout.recording.index_at(layout.v, w0)?.max(i0);
        let b = layout.recording.index_at(layout.v, w1)?.min(i1);
        let samples: Vec<Sample> = pairs
            .read(a, b.max(a), w0, w1, cancel)?
            .into_iter()
            .map(|(_, s)| s)
            .collect();
        sink.samples(&samples)?;
    }
    Ok(())
}

/// A capture with pins, over `interval` clipped to the recording: its pairs, and the pins' levels
/// from the earlier of the first pair and the interval's start.
fn export_with_pins(
    layout: &Layout,
    sink: &mut Sink,
    interval: Range<i64>,
    cancel: &AtomicBool,
) -> Result<()> {
    if interval.end <= interval.start {
        return Ok(());
    }
    let runs = layout.runs()?;
    let i0 = first_ending_after(layout, &runs, interval.start)?;
    let i1 = layout.recording.index_at(layout.v, interval.end)?.max(i0);
    let first_tick = (i0 < i1).then(|| runs.tick(i0));
    let start = first_tick.map_or(interval.start, |t| t.min(interval.start));

    // The levels just before `start`, which the export starts from without an edge.
    let initial = if start > 0 {
        let mut before = Pins::at(layout, start - 1, cancel)?;
        before.load(layout, start - 1, start, cancel)?;
        before.advance(start - 1);
        before.levels()
    } else {
        Some(layout.header.dio_levels)
    };
    sink.seed_dio(initial);

    // With no pair at `start`, the last pair before it, moved to `start`, opens the export as a
    // pair the probe didn't measure.
    let mut gap_seed = if first_tick.is_none_or(|t| t > start) {
        let j = layout.recording.index_at(layout.v, start + 1)?;
        let mut sample = if j > 0 {
            let at = runs.tick(j - 1);
            let mut seed = Pairs::new(layout, at, cancel)?;
            seed.read(j - 1, j, at, at + 1, cancel)?[0].1
        } else {
            Sample {
                sequence: 0,
                time_ns: 0,
                millivolts: 0,
                nanoamps: 0,
                range: None,
                under_range: false,
            }
        };
        sample.time_ns = layout.ns(start);
        Some(sample)
    } else {
        None
    };

    let mut pins = Pins::at(layout, start, cancel)?;
    let mut levels = initial;
    let mut pairs = Pairs::new(layout, start, cancel)?;
    // The pins at `start` can differ from those just before it with no change at `start`: an
    // export from tick 0 starts from the header's levels, which a stream attached later doesn't
    // know there.
    let mut at_start = true;
    for (w0, w1) in windows(layout, start, interval.end) {
        cancelled(cancel)?;
        pins.load(layout, w0, w1, cancel)?;
        let a = layout.recording.index_at(layout.v, w0)?.clamp(i0, i1);
        let b = layout.recording.index_at(layout.v, w1)?.clamp(a, i1);
        let mut window = pairs.read(a, b, w0, w1, cancel)?.into_iter().peekable();
        loop {
            let at = if std::mem::take(&mut at_start) {
                start
            } else if let Some(at) = pins.upcoming() {
                at
            } else {
                break;
            };
            for now in pins.changes_at(at) {
                if now == levels {
                    continue;
                }
                if at > start
                    && let Some(sample) = gap_seed.take()
                {
                    sink.gap_sample(sample)?;
                }
                while let Some((_, sample)) = window.next_if(|(t, _)| *t < at) {
                    sink.samples(&[sample])?;
                }
                let at_ns = layout.ns(at);
                sink.advance_to(at_ns)?;
                sink.dio_state(at_ns, now);
                levels = now;
            }
        }
        if let Some(sample) = gap_seed.take() {
            sink.gap_sample(sample)?;
        }
        for (_, sample) in window {
            sink.samples(&[sample])?;
        }
    }
    if let Some(sample) = gap_seed.take() {
        sink.gap_sample(sample)?;
    }
    sink.advance_to(layout.ns(interval.end))?;
    Ok(())
}
