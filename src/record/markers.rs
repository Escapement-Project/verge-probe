use super::{RecordError, Result};
use crate::stream::Markers;
use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::Path;

pub(super) struct Spool {
    rows: csv::Writer<File>,
    last: Option<u64>,
    untimed_losses: u64,
    pub first: Option<u64>,
}
impl Spool {
    pub fn new() -> Result<Self> {
        Ok(Self {
            rows: csv::Writer::from_writer(tempfile::tempfile()?),
            last: None,
            untimed_losses: 0,
            first: None,
        })
    }
    pub fn append(&mut self, message: &Markers) -> Result<()> {
        let lost = message.lost();
        if message.markers.is_empty() && lost {
            self.untimed_losses += 1;
        }
        for (index, marker) in message.markers.iter().enumerate() {
            self.append_text(
                marker.time_ns,
                marker.number,
                &label(&marker.label),
                index == 0 && lost,
            )?;
        }
        Ok(())
    }
    /// A marker whose label is already the text [`label`] makes of it.
    pub fn append_text(
        &mut self,
        time_ns: u64,
        number: u32,
        label: &str,
        after_loss: bool,
    ) -> Result<()> {
        if self.last.is_some_and(|last| time_ns < last) {
            return Err(RecordError::MarkerTime);
        }
        self.rows.write_record([
            time_ns.to_string(),
            number.to_string(),
            label.to_owned(),
            u8::from(after_loss).to_string(),
        ])?;
        self.last = Some(time_ns);
        self.first.get_or_insert(time_ns);
        Ok(())
    }
    /// A loss report that came with no marker.
    pub fn untimed_loss(&mut self) {
        self.untimed_losses += 1;
    }
    /// Nothing to merge: no marker and no loss report arrived.
    pub fn is_empty(&self) -> bool {
        self.first.is_none() && self.untimed_losses == 0
    }
    pub fn reader(self) -> Result<csv::Reader<File>> {
        Ok(csv::ReaderBuilder::new()
            .has_headers(false)
            .from_reader(self.into_file()?))
    }
    pub fn into_file(mut self) -> Result<File> {
        for _ in 0..self.untimed_losses {
            self.rows.write_record(["", "", "", "1"])?;
        }
        self.rows.flush()?;
        let mut file = self.rows.into_inner().map_err(|e| e.into_error())?;
        file.seek(SeekFrom::Start(0))?;
        Ok(file)
    }
}

/// UTF-8 labels keep their text; backslashes, controls and non-UTF-8 bytes are escaped.
pub fn label(bytes: &[u8]) -> String {
    let mut text = String::new();
    for chunk in bytes.utf8_chunks() {
        for ch in chunk.valid().chars() {
            if ch == '\\' || ch.is_control() {
                text.extend(ch.escape_default());
            } else {
                text.push(ch);
            }
        }
        for byte in chunk.invalid() {
            text.push_str(&format!("\\x{byte:02x}"));
        }
    }
    text
}

pub(super) struct CsvMarkers {
    pub spool: Spool,
}
impl CsvMarkers {
    /// Writes the pairs with the markers merged in by time to a file beside `path`, then renames
    /// it over `path`. Until the rename, `path` keeps the pairs as they were written.
    pub fn finish(
        self,
        mut analog: BufWriter<File>,
        path: &Path,
        check: &impl Fn() -> Result<()>,
    ) -> Result<()> {
        analog.flush()?;
        if self.spool.is_empty() {
            return Ok(());
        }
        let mut file = analog.into_inner().map_err(|e| e.into_error())?;
        file.seek(SeekFrom::Start(0))?;
        let target = std::fs::canonicalize(path)?;
        let merging = |source| RecordError::Merge {
            path: target.clone(),
            source,
        };
        let merged = super::temp_beside(&target).map_err(merging)?;
        merged
            .as_file()
            .set_permissions(file.metadata()?.permissions())?;
        let mut analog = csv::Reader::from_reader(&mut file);
        let mut output = csv::Writer::from_writer(merged.as_file());
        let mut header = analog.headers()?.clone();
        let columns = header.len();
        header.extend(["marker_number", "marker_label", "marker_lost"]);
        output.write_record(&header)?;
        let mut markers = self.spool.reader()?;
        let mut rows = markers.records().peekable();
        let marker_row = |row: csv::StringRecord| {
            let mut fields = vec![String::new(); columns + 3];
            fields[1] = row[0].to_owned();
            for index in 0..3 {
                fields[columns + index] = row[index + 1].to_owned();
            }
            fields
        };
        for row in analog.records() {
            check()?;
            let mut row = row?;
            let at = row[1].parse::<u64>().map_err(|_| RecordError::MarkerTime)?;
            while let Some(next) = rows.peek() {
                check()?;
                let next = next
                    .as_ref()
                    .map_err(|e| std::io::Error::other(e.to_string()))?;
                if next[0].is_empty()
                    || next[0]
                        .parse::<u64>()
                        .map_err(|_| RecordError::MarkerTime)?
                        > at
                {
                    break;
                }
                output.write_record(marker_row(rows.next().unwrap()?))?;
            }
            row.extend(["", "", ""]);
            output.write_record(&row)?;
        }
        for row in rows {
            check()?;
            output.write_record(marker_row(row?))?;
        }
        output.flush()?;
        drop(output);
        merged.as_file().sync_all()?;
        drop(analog);
        check()?;
        // The name may have moved on during recording, to another file or to none.
        let written = same_file::Handle::from_file(file)?;
        if same_file::Handle::from_path(&target).ok().as_ref() != Some(&written) {
            return Err(RecordError::Replaced { path: target });
        }
        // Windows cannot rename over a file that is still open.
        drop(written);
        merged
            .persist(&target)
            .map_err(|error| merging(error.error))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record::Sink;
    use crate::stream::{Marker, Sample};
    use std::io::Read;

    fn pairs(sink: &mut Sink, count: u32) {
        for sequence in 0..count {
            sink.samples(&[Sample {
                sequence,
                time_ns: u64::from(sequence) * 1_000_000,
                millivolts: 3000,
                nanoamps: 10,
                range: None,
                under_range: false,
            }])
            .unwrap();
        }
    }

    fn marker(time_ns: u64) -> Markers {
        Markers::new(
            0,
            vec![Marker {
                time_ns,
                number: 1,
                label: b"m".to_vec(),
            }],
        )
    }

    #[test]
    fn an_interrupted_marker_merge_leaves_the_pairs_as_written() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("partial.csv");
        let mut sink = Sink::create(&path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        pairs(&mut sink, 1000);
        assert!(sink.markers(&marker(500_000_000)).unwrap());
        let Sink::Csv(mut csv) = sink else {
            unreachable!()
        };
        csv.file.flush().unwrap();
        let before = std::fs::read(&path).unwrap();
        assert_eq!(before.iter().filter(|&&byte| byte == b'\n').count(), 1001);
        let checks = std::cell::Cell::new(0);
        let result = csv.markers.take().unwrap().finish(csv.file, &path, &|| {
            checks.set(checks.get() + 1);
            if checks.get() == 100 {
                Err(RecordError::Cancelled)
            } else {
                Ok(())
            }
        });
        assert!(matches!(result, Err(RecordError::Cancelled)));
        assert_eq!(std::fs::read(&path).unwrap(), before);
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
    }

    #[test]
    fn a_destination_replaced_during_recording_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("a.csv");
        let moved = dir.path().join("moved.csv");
        let mut sink = Sink::create(&path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        pairs(&mut sink, 10);
        sink.markers(&marker(5_000_000)).unwrap();
        std::fs::rename(&path, &moved).unwrap();
        std::fs::write(&path, b"someone else's\n").unwrap();
        assert!(matches!(sink.finish(), Err(RecordError::Replaced { .. })));
        assert_eq!(std::fs::read(&path).unwrap(), b"someone else's\n");
        let kept = std::fs::read_to_string(&moved).unwrap();
        assert_eq!(kept.lines().count(), 11, "{kept}");
        assert!(!kept.contains("marker"), "{kept}");
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 2);
    }

    #[test]
    fn a_marker_csv_with_no_markers_keeps_its_file_and_columns() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plain.csv");
        let mut sink = Sink::create(&path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        pairs(&mut sink, 10);
        #[cfg(unix)]
        let inode = std::os::unix::fs::MetadataExt::ino(&std::fs::metadata(&path).unwrap());
        sink.finish().unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(!text.contains("marker"), "{text}");
        #[cfg(unix)]
        assert_eq!(
            std::os::unix::fs::MetadataExt::ino(&std::fs::metadata(&path).unwrap()),
            inode
        );
    }

    #[cfg(unix)]
    #[test]
    fn a_csv_that_is_not_a_file_drops_markers_and_still_records() {
        let path = std::path::Path::new("/dev/null");
        let file = File::options().read(true).write(true).open(path).unwrap();
        let mut sink = Sink::from_file(file, path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        pairs(&mut sink, 10);
        assert!(!sink.markers(&marker(5_000_000)).unwrap());
        sink.finish().unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn a_merge_through_a_symlink_replaces_the_file_it_names() {
        let dir = tempfile::tempdir().unwrap();
        let real = dir.path().join("real.csv");
        let link = dir.path().join("link.csv");
        std::fs::write(&real, b"").unwrap();
        std::os::unix::fs::symlink(&real, &link).unwrap();
        let mut sink = Sink::create(&link, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        pairs(&mut sink, 10);
        sink.markers(&marker(5_000_000)).unwrap();
        sink.finish().unwrap();
        assert!(std::fs::symlink_metadata(&link).unwrap().is_symlink());
        let text = std::fs::read_to_string(&real).unwrap();
        assert!(text.starts_with("sequence,"), "{text}");
        assert!(
            text.lines().next().unwrap().ends_with("marker_lost"),
            "{text}"
        );
        assert_eq!(text.lines().count(), 12, "{text}");
    }

    #[test]
    fn sigrok_marker_origin_handles_early_markers_and_marker_only_recordings() {
        for analog in [true, false] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("origin.sr");
            let mut sink = Sink::create(&path, false).unwrap();
            sink.include_markers().unwrap();
            sink.begin(1000, 0, 0).unwrap();
            if analog {
                sink.samples(&[Sample {
                    sequence: 40,
                    time_ns: 40_000_000,
                    millivolts: 3000,
                    nanoamps: 10,
                    range: None,
                    under_range: false,
                }])
                .unwrap();
            }
            sink.markers(&Markers::new(
                0,
                vec![Marker {
                    time_ns: 39_000_000,
                    number: 1,
                    label: b"before".to_vec(),
                }],
            ))
            .unwrap();
            sink.finish().unwrap();
            let mut zip = zip::ZipArchive::new(File::open(path).unwrap()).unwrap();
            let mut sidecar = String::new();
            zip.by_name("markers.csv")
                .unwrap()
                .read_to_string(&mut sidecar)
                .unwrap();
            assert!(
                sidecar.ends_with(if analog { ",-1000000\n" } else { ",0\n" }),
                "{sidecar}"
            );
            let mut metadata = String::new();
            zip.by_name("metadata")
                .unwrap()
                .read_to_string(&mut metadata)
                .unwrap();
            assert!(metadata.contains(if analog {
                "verge capture origin ns=40000000"
            } else {
                "verge capture origin ns=39000000"
            }));
        }
    }

    #[test]
    fn late_markers_merge_in_time_order_with_quoted_lossless_labels() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("markers.csv");
        let mut sink = Sink::create(&path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 8, 0).unwrap();
        sink.samples(&[
            Sample {
                sequence: 0,
                time_ns: 0,
                millivolts: 3000,
                nanoamps: 10,
                range: None,
                under_range: false,
            },
            Sample {
                sequence: 1,
                time_ns: 1_000_000,
                millivolts: 3000,
                nanoamps: 20,
                range: None,
                under_range: false,
            },
        ])
        .unwrap();
        sink.markers(&Markers::new(
            Markers::LOST,
            vec![
                Marker {
                    time_ns: 500_000,
                    number: u32::MAX,
                    label: b"comma, quote\"\n\\\xff".to_vec(),
                },
                Marker {
                    time_ns: 500_000,
                    number: 0,
                    label: vec![],
                },
            ],
        ))
        .unwrap();
        sink.finish().unwrap();
        let mut csv = csv::Reader::from_path(path).unwrap();
        assert_eq!(
            csv.headers().unwrap().iter().collect::<Vec<_>>(),
            [
                "sequence",
                "time_ns",
                "millivolts",
                "nanoamps",
                "range",
                "dio3",
                "marker_number",
                "marker_label",
                "marker_lost"
            ]
        );
        let rows: Vec<_> = csv.records().map(|row| row.unwrap()).collect();
        assert_eq!(
            rows.iter().map(|r| &r[1]).collect::<Vec<_>>(),
            ["0", "500000", "500000", "1000000"]
        );
        assert_eq!(&rows[1][6], "4294967295");
        assert_eq!(&rows[1][7], "comma, quote\"\\n\\\\\\xff");
        assert_eq!(&rows[1][8], "1");
        assert_eq!(&rows[2][6], "0");
        assert_eq!(&rows[3][3], "20");
    }
    #[test]
    fn sigrok_embeds_markers_without_changing_sample_channels() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("markers.sr");
        let mut sink = Sink::create(&path, false).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        sink.samples(&[Sample {
            sequence: 0,
            time_ns: 40_000_000,
            millivolts: 3000,
            nanoamps: 10,
            range: None,
            under_range: false,
        }])
        .unwrap();
        sink.markers(&Markers::new(
            0,
            vec![Marker {
                time_ns: 40_100_000,
                number: 7,
                label: "μs, go".as_bytes().to_vec(),
            }],
        ))
        .unwrap();
        sink.finish().unwrap();
        let mut zip = zip::ZipArchive::new(File::open(path).unwrap()).unwrap();
        let mut text = String::new();
        zip.by_name("markers.csv")
            .unwrap()
            .read_to_string(&mut text)
            .unwrap();
        assert_eq!(
            text,
            "time_ns,marker_number,marker_label,marker_lost,session_time_ns\n40100000,7,\"μs, go\",0,100000\n"
        );
        assert_eq!(zip.by_name("logic-1-1").unwrap().size(), 1);
    }
}
