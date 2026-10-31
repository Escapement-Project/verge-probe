use super::store::{cancelled, decode, encode};
use super::{Archive, CaptureError, Interval, Result};
use crate::stream::{Marker, Markers};
use rusqlite::params;
use serde::{Deserialize, Serialize};
use std::sync::atomic::AtomicBool;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MarkerCursor {
    pub time_ns: u64,
    pub id: u64,
}
#[derive(Clone, Debug)]
pub struct StoredMarker {
    pub id: u64,
    pub marker: Marker,
    pub lost_before: bool,
}
#[derive(Clone, Debug, Default)]
pub struct MarkerPage {
    pub entries: Vec<StoredMarker>,
    pub next: Option<MarkerCursor>,
    pub total: u64,
    pub loss_events: u64,
}
#[derive(Default)]
pub(super) struct MarkerWriter {
    pending: Vec<Row>,
    pub count: u64,
    pub losses: u64,
    rows: u64,
}
#[derive(Serialize, Deserialize)]
struct Row {
    marker: Option<Marker>,
    lost: bool,
}
impl MarkerWriter {
    pub fn ingest(&mut self, message: &Markers) {
        self.count += message.markers.len() as u64;
        self.losses += u64::from(message.lost());
        if message.markers.is_empty() && message.lost() {
            self.pending.push(Row {
                marker: None,
                lost: true,
            });
        }
        for (index, marker) in message.markers.iter().enumerate() {
            self.pending.push(Row {
                marker: Some(marker.clone()),
                lost: index == 0 && message.lost(),
            });
        }
    }
    pub fn flush(&self, db: &rusqlite::Connection) -> Result<()> {
        for (offset, row) in self.pending.iter().enumerate() {
            let at = row
                .marker
                .as_ref()
                .map(|marker| time(marker.time_ns))
                .transpose()?;
            db.execute(
                "INSERT INTO markers VALUES(?,?,?)",
                params![(self.rows + offset as u64) as i64, at, encode(row)?],
            )?;
        }
        for (key, value) in [("marker_count", self.count), ("marker_losses", self.losses)] {
            db.execute(
                "INSERT OR REPLACE INTO metadata VALUES(?,?)",
                params![key, encode(&value)?],
            )?;
        }
        Ok(())
    }
    pub fn committed(&mut self) {
        self.rows += self.pending.len() as u64;
        self.pending.clear();
    }
}
fn time(value: u64) -> Result<i64> {
    i64::try_from(value).map_err(|_| CaptureError::Time)
}
impl Archive {
    /// Markers in time and arrival order, with at most 512 entries per page.
    /// Continue with the returned cursor to include every marker at a shared timestamp.
    pub fn markers(
        &self,
        interval: Interval,
        after: Option<MarkerCursor>,
        limit: usize,
        cancel: &AtomicBool,
    ) -> Result<MarkerPage> {
        cancelled(cancel)?;
        if self.version < 3 {
            return Ok(MarkerPage::default());
        }
        let limit = limit.clamp(1, 512);
        let mut statement = self.db.prepare("SELECT n,data FROM markers WHERE time_ns>=? AND time_ns<? AND (time_ns,n)>(?,?) ORDER BY time_ns,n LIMIT ?")?;
        let mut rows = statement.query(params![
            time(interval.start_ns)?,
            time(interval.end_ns.min(i64::MAX as u64))?,
            after.map_or(-1, |c| c.time_ns.min(i64::MAX as u64) as i64),
            after.map_or(-1, |c| c.id as i64),
            (limit + 1) as i64
        ])?;
        let mut entries = Vec::new();
        while let Some(row) = rows.next()? {
            cancelled(cancel)?;
            let data: Row = decode(&row.get::<_, Vec<u8>>(1)?)?;
            if let Some(marker) = data.marker {
                entries.push(StoredMarker {
                    id: row.get::<_, i64>(0)? as u64,
                    marker,
                    lost_before: data.lost,
                });
            }
        }
        let next = if entries.len() > limit {
            entries.pop();
            entries.last().map(|entry| MarkerCursor {
                time_ns: entry.marker.time_ns,
                id: entry.id,
            })
        } else {
            None
        };
        Ok(MarkerPage {
            entries,
            next,
            total: self.meta("marker_count")?,
            loss_events: self.meta("marker_losses")?,
        })
    }
    pub(super) fn export_markers(
        &self,
        sink: &mut crate::record::Sink,
        interval: Interval,
        cancel: &AtomicBool,
    ) -> Result<()> {
        if self.version < 3 {
            return Ok(());
        }
        let mut after = None;
        loop {
            let page = self.markers(interval, after, 512, cancel)?;
            for entry in page.entries {
                sink.markers(&Markers::new(
                    if entry.lost_before { Markers::LOST } else { 0 },
                    vec![entry.marker],
                ))?;
            }
            after = page.next;
            if after.is_none() {
                break;
            }
        }
        let mut statement = self
            .db
            .prepare("SELECT data FROM markers WHERE time_ns IS NULL ORDER BY n")?;
        let mut rows = statement.query([])?;
        while let Some(row) = rows.next()? {
            cancelled(cancel)?;
            let data: Row = decode(&row.get::<_, Vec<u8>>(0)?)?;
            if data.lost {
                sink.markers(&Markers::new(Markers::LOST, vec![]))?;
            }
        }
        Ok(())
    }
}
