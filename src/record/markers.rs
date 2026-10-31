use super::{RecordError, Result};
use crate::stream::Markers;
use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};

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
            if self.last.is_some_and(|last| marker.time_ns < last) {
                return Err(RecordError::MarkerTime);
            }
            self.rows.write_record([
                marker.time_ns.to_string(),
                marker.number.to_string(),
                label(&marker.label),
                u8::from(index == 0 && lost).to_string(),
            ])?;
            self.last = Some(marker.time_ns);
            self.first.get_or_insert(marker.time_ns);
        }
        Ok(())
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
    pub fn finish(
        self,
        mut analog: BufWriter<File>,
        check: &impl Fn() -> Result<()>,
    ) -> Result<()> {
        analog.flush()?;
        let mut file = analog.into_inner().map_err(|e| e.into_error())?;
        file.seek(SeekFrom::Start(0))?;
        let mut analog = csv::Reader::from_reader(&mut file);
        let mut output = csv::Writer::from_writer(tempfile::tempfile()?);
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
        drop(analog);
        let mut merged = output.into_inner().map_err(|e| e.into_error())?;
        merged.seek(SeekFrom::Start(0))?;
        check()?;
        file.seek(SeekFrom::Start(0))?;
        let mut bytes = [0; 64 * 1024];
        loop {
            use std::io::Read;
            check()?;
            let count = merged.read(&mut bytes)?;
            if count == 0 {
                break;
            }
            file.write_all(&bytes[..count])?;
        }
        let length = file.stream_position()?;
        file.set_len(length)?;
        file.flush()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record::Sink;
    use crate::stream::{Marker, Sample};
    use std::io::Read;

    #[test]
    fn interrupted_marker_csv_keeps_received_pairs_and_cancelled_merge_keeps_them() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("partial.csv");
        let mut sink = Sink::create(&path, true).unwrap();
        sink.include_markers().unwrap();
        sink.begin(1000, 0, 0).unwrap();
        for sequence in 0..1000 {
            sink.samples(&[Sample {
                sequence,
                time_ns: u64::from(sequence) * 1_000_000,
                millivolts: 3000,
                nanoamps: 10,
            }])
            .unwrap();
        }
        let Sink::Csv(mut csv) = sink else {
            unreachable!()
        };
        csv.file.flush().unwrap();
        let before = std::fs::read(&path).unwrap();
        assert_eq!(before.iter().filter(|&&byte| byte == b'\n').count(), 1001);
        let checks = std::cell::Cell::new(0);
        let result = csv.markers.take().unwrap().finish(csv.file, &|| {
            checks.set(checks.get() + 1);
            if checks.get() == 100 {
                Err(RecordError::Cancelled)
            } else {
                Ok(())
            }
        });
        assert!(matches!(result, Err(RecordError::Cancelled)));
        assert_eq!(checks.get(), 100);
        assert_eq!(std::fs::read(path).unwrap(), before);
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
            },
            Sample {
                sequence: 1,
                time_ns: 1_000_000,
                millivolts: 3000,
                nanoamps: 20,
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
        assert_eq!(&rows[1][5], "4294967295");
        assert_eq!(&rows[1][6], "comma, quote\"\\n\\\\\\xff");
        assert_eq!(&rows[1][7], "1");
        assert_eq!(&rows[2][5], "0");
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
