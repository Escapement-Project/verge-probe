use super::store::{BINS, FAN, bin_interval, cancelled, decode, encode};
use super::{Archive, CaptureError, Interval, Result};
use crate::stream::Edge;
use rusqlite::{OptionalExtension, params};
use serde::{Deserialize, Serialize};
use std::sync::atomic::AtomicBool;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct DigitalSpan {
    pub interval: Interval,
    pub levels: Option<u8>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct DigitalBin {
    pub interval: Interval,
    pub high: u8,
    pub low: u8,
    pub gap: bool,
}
impl DigitalBin {
    fn add(&mut self, other: &Self) {
        self.high |= other.high;
        self.low |= other.low;
        self.gap |= other.gap;
    }
    fn span(span: DigitalSpan, pins: u8) -> Self {
        Self {
            interval: span.interval,
            high: span.levels.unwrap_or(0) & pins,
            low: span.levels.map_or(0, |levels| !levels & pins),
            gap: span.levels.is_none(),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct DigitalView {
    pub pins: u8,
    pub bins: Vec<DigitalBin>,
    pub loss_events: u64,
}

#[derive(Default)]
pub(super) struct DigitalWriter {
    spans: Vec<DigitalSpan>,
    pub tail: Option<(u64, Option<u8>)>,
    blocks: u64,
    pub losses: u64,
}
impl DigitalWriter {
    pub fn edge(&mut self, edge: Edge, lost: bool) -> Result<()> {
        if let Some((start, levels)) = self.tail {
            if edge.time_ns < start {
                return Err(CaptureError::Timestamps);
            }
            if lost && levels.is_some() {
                self.spans.push(DigitalSpan {
                    interval: Interval::new(start, start),
                    levels,
                });
            }
            if edge.time_ns >= start {
                self.spans.push(DigitalSpan {
                    interval: Interval::new(start, edge.time_ns),
                    levels: if lost { None } else { levels },
                });
            }
        }
        self.tail = Some((edge.time_ns, Some(edge.levels)));
        Ok(())
    }
    pub fn lost(&mut self) {
        if let Some((at, levels)) = &mut self.tail {
            if levels.is_some() {
                self.spans.push(DigitalSpan {
                    interval: Interval::new(*at, *at),
                    levels: *levels,
                });
            }
            *levels = None;
        }
    }
    pub fn flush(&self, db: &rusqlite::Connection, pins: u8) -> Result<()> {
        if !self.spans.is_empty() {
            let mut node = DigitalBin {
                interval: Interval::new(
                    self.spans[0].interval.start_ns,
                    self.spans.last().unwrap().interval.end_ns,
                ),
                ..Default::default()
            };
            for &span in &self.spans {
                node.add(&DigitalBin::span(span, pins));
            }
            db.execute(
                "INSERT INTO dio_blocks VALUES(?,?,?,?)",
                params![
                    self.blocks as i64,
                    time(node.interval.start_ns)?,
                    time(node.interval.end_ns)?,
                    encode(&self.spans)?
                ],
            )?;
            put(db, 0, self.blocks, &node)?;
            let mut index = self.blocks;
            let mut level = 0;
            loop {
                index /= FAN;
                level += 1;
                let mut statement = db.prepare(
                    "SELECT data FROM dio_nodes WHERE level=? AND n>=? AND n<? ORDER BY n",
                )?;
                let rows = statement.query_map(
                    params![level - 1, (index * FAN) as i64, ((index + 1) * FAN) as i64],
                    |r| r.get::<_, Vec<u8>>(0),
                )?;
                let mut parent: Option<DigitalBin> = None;
                for row in rows {
                    let child: DigitalBin = decode(&row?)?;
                    match &mut parent {
                        None => parent = Some(child),
                        Some(parent) => {
                            parent.interval.end_ns = child.interval.end_ns;
                            parent.add(&child);
                        }
                    }
                }
                put(db, level, index, &parent.unwrap())?;
                if index == 0 {
                    break;
                }
            }
        }
        for (key, value) in [
            (
                "dio_blocks",
                encode(&(self.blocks + u64::from(!self.spans.is_empty())))?,
            ),
            ("dio_tail", encode(&self.tail)?),
            ("dio_losses", encode(&self.losses)?),
        ] {
            db.execute(
                "INSERT OR REPLACE INTO metadata VALUES(?,?)",
                params![key, value],
            )?;
        }
        Ok(())
    }
    pub fn committed(&mut self) {
        self.blocks += u64::from(!self.spans.is_empty());
        self.spans.clear();
    }
}
fn time(value: u64) -> Result<i64> {
    i64::try_from(value).map_err(|_| CaptureError::Time)
}
fn put(db: &rusqlite::Connection, level: u32, index: u64, node: &DigitalBin) -> Result<()> {
    db.execute(
        "INSERT OR REPLACE INTO dio_nodes VALUES(?,?,?)",
        params![level, index as i64, encode(node)?],
    )?;
    Ok(())
}
struct DigitalQuery<'a> {
    interval: Interval,
    coverage: Interval,
    pins: u8,
    bins: &'a mut [DigitalBin],
    cancel: &'a AtomicBool,
}
impl Archive {
    pub fn digital(
        &self,
        interval: Interval,
        pixels: usize,
        cancel: &AtomicBool,
    ) -> Result<DigitalView> {
        self.db.execute_batch("BEGIN")?;
        let result = self.digital_inner(interval, pixels, cancel);
        self.db.execute_batch("ROLLBACK")?;
        result
    }
    fn digital_inner(
        &self,
        interval: Interval,
        pixels: usize,
        cancel: &AtomicBool,
    ) -> Result<DigitalView> {
        cancelled(cancel)?;
        if self.version < 2 || self.extent()?.duration_ns() == 0 {
            return Ok(DigitalView::default());
        }
        let pins = self.header()?.dio_mask;
        let count = pixels
            .clamp(1, BINS)
            .min(interval.duration_ns().max(1).min(BINS as u64) as usize);
        let mut bins: Vec<_> = (0..count)
            .map(|i| DigitalBin {
                interval: bin_interval(interval, count, i),
                ..Default::default()
            })
            .collect();
        let blocks: u64 = self.meta("dio_blocks")?;
        let mut level = 0;
        let mut covered = 1u64;
        while covered < blocks {
            covered = covered.saturating_mul(FAN);
            level += 1;
        }
        let extent = self.extent()?;
        self.digital_visit(
            level,
            0,
            &mut DigitalQuery {
                interval,
                coverage: extent,
                pins,
                bins: &mut bins,
                cancel,
            },
        )?;
        let extent = self.extent()?;
        let tail: Option<(u64, Option<u8>)> = self.meta("dio_tail")?;
        if let Some((start, levels)) = tail {
            add_span(
                &mut bins,
                DigitalSpan {
                    interval: Interval::new(start, extent.end_ns),
                    levels,
                },
                pins,
                extent,
            );
        }
        bins.retain(|bin| bin.high != 0 || bin.low != 0 || bin.gap);
        Ok(DigitalView {
            pins,
            bins,
            loss_events: self.meta("dio_losses")?,
        })
    }
    fn digital_visit(&self, level: u32, index: u64, query: &mut DigitalQuery<'_>) -> Result<()> {
        cancelled(query.cancel)?;
        let bytes: Option<Vec<u8>> = self
            .db
            .query_row(
                "SELECT data FROM dio_nodes WHERE level=? AND n=?",
                params![level, index as i64],
                |r| r.get(0),
            )
            .optional()?;
        let Some(bytes) = bytes else {
            return Ok(());
        };
        let node: DigitalBin = decode(&bytes)?;
        let interval = query.interval;
        if node.interval.end_ns < interval.start_ns || node.interval.start_ns >= interval.end_ns {
            return Ok(());
        }
        let first = bin_at(
            interval,
            query.bins.len(),
            node.interval.start_ns.max(interval.start_ns),
        );
        let last = bin_at(
            interval,
            query.bins.len(),
            node.interval.end_ns.min(interval.end_ns.saturating_sub(1)),
        );
        if first == last
            && node.interval.start_ns >= interval.start_ns
            && node.interval.end_ns <= interval.end_ns
            && node.interval.start_ns >= query.coverage.start_ns
            && node.interval.end_ns <= query.coverage.end_ns
        {
            query.bins[first].add(&node);
        } else if level > 0 {
            for child in index * FAN..(index + 1) * FAN {
                self.digital_visit(level - 1, child, query)?;
            }
        } else {
            let data: Vec<u8> = self.db.query_row(
                "SELECT data FROM dio_blocks WHERE n=?",
                [index as i64],
                |r| r.get(0),
            )?;
            for span in decode::<Vec<DigitalSpan>>(&data)? {
                add_span(query.bins, span, query.pins, query.coverage);
            }
        }
        Ok(())
    }
    pub(super) fn export_digital(
        &self,
        sink: &mut crate::record::Sink,
        interval: Interval,
        cancel: &AtomicBool,
    ) -> Result<()> {
        let extent = self.extent()?;
        let interval = Interval::new(
            interval.start_ns.max(extent.start_ns),
            interval.end_ns.min(extent.end_ns),
        );
        if interval.duration_ns() == 0 {
            return Ok(());
        }
        let mut statement = self
            .db
            .prepare("SELECT data FROM blocks WHERE end_ns>=? AND start_ns<? ORDER BY n")?;
        let mut rows = statement.query(params![
            time(interval.start_ns)?,
            time(interval.end_ns.min(i64::MAX as u64))?
        ])?;
        let mut samples = Vec::<super::Supported>::new().into_iter();
        let mut next = || -> Result<Option<crate::stream::Sample>> {
            loop {
                if let Some(s) = samples.next() {
                    if s.end_ns > interval.start_ns && s.sample.time_ns < interval.end_ns {
                        return Ok(Some(s.sample));
                    }
                } else if let Some(row) = rows.next()? {
                    cancelled(cancel)?;
                    samples = self.decode_block(&row.get::<_, Vec<u8>>(0)?)?.into_iter();
                } else {
                    return Ok(None);
                }
            }
        };
        let mut pending = next()?;
        let start = pending.map_or(interval.start_ns, |sample| {
            sample.time_ns.min(interval.start_ns)
        });
        let mut initial = Some(self.header()?.dio_levels);
        if start > 0 {
            self.digital_spans(Interval::new(start - 1, start), cancel, |span| {
                initial = span.levels;
                Ok(())
            })?;
        }
        sink.seed_dio(initial);
        let mut gap_seed = None;
        if pending.is_none_or(|sample| sample.time_ns > start) {
            let data: Option<Vec<u8>> = self
                .db
                .query_row(
                    "SELECT data FROM blocks WHERE start_ns<=? ORDER BY n DESC LIMIT 1",
                    [time(start)?],
                    |r| r.get(0),
                )
                .optional()?;
            let previous = data
                .map(|data| self.decode_block(&data))
                .transpose()?
                .and_then(|samples| {
                    samples
                        .into_iter()
                        .rev()
                        .find(|s| s.sample.time_ns <= start)
                });
            let mut sample = previous.map_or(
                crate::stream::Sample {
                    sequence: 0,
                    time_ns: start,
                    millivolts: 0,
                    nanoamps: 0,
                    range: None,
                    under_range: false,
                },
                |s| s.sample,
            );
            sample.time_ns = start;
            gap_seed = Some(sample);
        }
        let header_levels = self.header()?.dio_levels;
        let mut first_span = true;
        self.digital_spans(Interval::new(start, interval.end_ns), cancel, |span| {
            if span.interval.start_ns < start {
                return Ok(());
            }
            let at = span.interval.start_ns;
            if at > start
                && let Some(sample) = gap_seed.take()
            {
                sink.gap_sample(sample)?;
            }
            while let Some(sample) = pending
                && sample.time_ns < at
            {
                sink.samples(&[sample])?;
                pending = next()?;
            }
            sink.advance_to(at)?;
            if !first_span || at != 0 || span.levels != Some(header_levels) {
                sink.dio_state(at, span.levels);
            }
            first_span = false;
            Ok(())
        })?;
        if let Some(sample) = gap_seed {
            sink.gap_sample(sample)?;
        }
        while let Some(sample) = pending {
            sink.samples(&[sample])?;
            pending = next()?;
        }
        sink.advance_to(interval.end_ns)?;
        Ok(())
    }
    pub(super) fn digital_spans(
        &self,
        interval: Interval,
        cancel: &AtomicBool,
        mut each: impl FnMut(DigitalSpan) -> Result<()>,
    ) -> Result<()> {
        if self.version < 2 {
            return Ok(());
        }
        let mut statement = self
            .db
            .prepare("SELECT data FROM dio_blocks WHERE end_ns>=? AND start_ns<? ORDER BY n")?;
        let mut rows = statement.query(params![
            time(interval.start_ns)?,
            time(interval.end_ns.min(i64::MAX as u64))?
        ])?;
        while let Some(row) = rows.next()? {
            cancelled(cancel)?;
            for span in decode::<Vec<DigitalSpan>>(&row.get::<_, Vec<u8>>(0)?)? {
                if overlaps(span.interval, interval) {
                    each(span)?;
                }
            }
        }
        if let Some((start, levels)) = self.meta::<Option<(u64, Option<u8>)>>("dio_tail")? {
            let end = self.extent()?.end_ns;
            if end > interval.start_ns && start < interval.end_ns {
                each(DigitalSpan {
                    interval: Interval::new(start, end),
                    levels,
                })?;
            }
        }
        Ok(())
    }
}
fn bin_at(interval: Interval, count: usize, time: u64) -> usize {
    (((u128::from(time.saturating_sub(interval.start_ns)) + 1) * count as u128 - 1)
        / u128::from(interval.duration_ns().max(1)))
    .min((count - 1) as u128) as usize
}
fn add_span(bins: &mut [DigitalBin], mut span: DigitalSpan, pins: u8, coverage: Interval) {
    if !overlaps(span.interval, coverage) {
        return;
    }
    span.interval.start_ns = span.interval.start_ns.max(coverage.start_ns);
    span.interval.end_ns = span.interval.end_ns.min(coverage.end_ns);
    let interval = Interval::new(
        bins[0].interval.start_ns,
        bins.last().unwrap().interval.end_ns,
    );
    if !overlaps(span.interval, interval) {
        return;
    }
    let first = bin_at(
        interval,
        bins.len(),
        span.interval.start_ns.max(interval.start_ns),
    );
    let last = bin_at(
        interval,
        bins.len(),
        span.interval
            .end_ns
            .min(interval.end_ns)
            .saturating_sub(1)
            .max(span.interval.start_ns),
    );
    for bin in &mut bins[first..=last] {
        bin.add(&DigitalBin::span(span, pins));
    }
}

fn overlaps(span: Interval, interval: Interval) -> bool {
    if span.duration_ns() == 0 {
        span.start_ns >= interval.start_ns && span.start_ns < interval.end_ns
    } else {
        span.end_ns > interval.start_ns && span.start_ns < interval.end_ns
    }
}
