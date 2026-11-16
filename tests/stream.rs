//! The stream converter against the vectors mculink-verge's own converter writes
//! (`tools/energy_vectors.json`, copied to `tests/fixtures`): samples and whole-range digests of each
//! pair's conversion, and pair sequences with pauses and the totals they give.

use escapement_verge_probe::stream::{self, Converter, Message, Pair, Pairs};
use serde_json::Value;

fn vectors() -> Value {
    serde_json::from_str(include_str!("fixtures/energy_vectors.json")).expect("vectors parse")
}

fn points(value: &Value) -> Vec<(u16, u32)> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|p| (p[0].as_u64().unwrap() as u16, p[1].as_u64().unwrap() as u32))
        .collect()
}

/// A format 3 header as the probe sends it, up to step 5's fields: the clock and the pair period,
/// then the stream's tables.
fn header_prefix(
    clock_hz: u32,
    period: u32,
    full_scale: u16,
    low: &[(u16, u32)],
    high: &[(u16, u32)],
) -> Vec<u8> {
    let mut bytes = vec![stream::HEADER, 3];
    bytes.extend_from_slice(&clock_hz.to_le_bytes());
    bytes.extend_from_slice(&period.to_le_bytes());
    bytes.push(1);
    bytes.extend_from_slice(&full_scale.to_le_bytes());
    bytes.extend_from_slice(&1u32.to_le_bytes());
    bytes.push(1);
    for table in [low, high] {
        bytes.push(table.len() as u8);
        for i in 0..4 {
            let (code, na) = table.get(i).copied().unwrap_or((0, 0));
            bytes.extend_from_slice(&code.to_le_bytes());
            bytes.extend_from_slice(&na.to_le_bytes());
        }
    }
    bytes.extend_from_slice(&[0, 0, 0, 0]);
    assert_eq!(bytes.len(), 72);
    bytes
}

/// A whole format 3 header at `rate` pairs a second, which the 96 MHz clock divides, with step 5's
/// fields zero.
fn header_bytes(rate: u32, full_scale: u16, low: &[(u16, u32)], high: &[(u16, u32)]) -> Vec<u8> {
    with_step_5(
        header_prefix(96_000_000, 96_000_000 / rate, full_scale, low, high),
        0,
        false,
        None,
    )
}

fn converter(rate: u32, full_scale: u16, low: &[(u16, u32)], high: &[(u16, u32)]) -> Converter {
    match stream::parse(&header_bytes(rate, full_scale, low, high)).unwrap() {
        Message::Header(header) => Converter::new(&header),
        other => panic!("{other:?}"),
    }
}

/// Step 5's fields after a header's prefix (or its identity): weight, flags and the current before.
fn with_step_5(mut header: Vec<u8>, weight: u16, between: bool, before: Option<u32>) -> Vec<u8> {
    header.extend_from_slice(&weight.to_le_bytes());
    header.push(u8::from(between) | if before.is_some() { 2 } else { 0 });
    header.extend_from_slice(&before.unwrap_or(0).to_le_bytes());
    header
}

fn fnv(mut hash: u64, value: u32) -> u64 {
    for byte in value.to_le_bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

#[test]
fn each_pair_converts_as_the_firmware_converts_it() {
    let vectors = vectors();
    for case in vectors["convert"].as_array().unwrap() {
        let full_scale = case["full_scale_mv"].as_u64().unwrap() as u16;
        let (low, high) = (points(&case["low"]), points(&case["high"]));
        let convert = converter(1000, full_scale, &low, &high);
        for sample in case["samples"].as_array().unwrap() {
            let field = |i: usize| sample[i].as_u64().unwrap();
            let pair = Pair {
                voltage_code: field(0) as u16,
                current_code: field(1) as u16,
                low: field(2) == 1,
                under_range: false,
            };
            let reading = convert.reading(&pair);
            assert_eq!(
                (
                    u64::from(reading.millivolts),
                    u64::from(reading.shunt_nanoamps),
                    u64::from(reading.nanoamps)
                ),
                (field(3), field(4), field(5)),
                "{sample}"
            );
        }
        let (mut low_hash, mut high_hash, mut voltage_hash) = (
            0xcbf2_9ce4_8422_2325u64,
            0xcbf2_9ce4_8422_2325u64,
            0xcbf2_9ce4_8422_2325u64,
        );
        for code in 0..=u16::MAX {
            let at = |low| {
                convert
                    .reading(&Pair {
                        voltage_code: 0,
                        current_code: code,
                        low,
                        under_range: false,
                    })
                    .shunt_nanoamps
            };
            low_hash = fnv(low_hash, at(true));
            high_hash = fnv(high_hash, at(false));
            let mv = stream::millivolts(code, full_scale);
            voltage_hash = fnv(
                fnv(voltage_hash, mv),
                u32::MAX - stream::target_nanoamps(u32::MAX, mv),
            );
        }
        let digests = &case["sweep_fnv1a64"];
        assert_eq!(
            format!("{low_hash:016x}"),
            digests["low"],
            "low range sweep"
        );
        assert_eq!(
            format!("{high_hash:016x}"),
            digests["high"],
            "high range sweep"
        );
        assert_eq!(
            format!("{voltage_hash:016x}"),
            digests["voltage"],
            "voltage sweep"
        );
    }
}

#[test]
fn a_stream_sums_to_the_firmware_total() {
    let vectors = vectors();
    for case in vectors["sequences"].as_array().unwrap() {
        let clock = case["clock_hz"].as_u64().unwrap() as u32;
        let period = case["period"].as_u64().unwrap() as u32;
        let full_scale = case["full_scale_mv"].as_u64().unwrap() as u16;
        let header = with_step_5(
            header_prefix(
                clock,
                period,
                full_scale,
                &points(&case["low"]),
                &points(&case["high"]),
            ),
            case["switch_weight_q16"].as_u64().unwrap() as u16,
            case["rearm_between_pairs"].as_bool().unwrap(),
            None,
        );
        let Message::Header(header) = stream::parse(&header).unwrap() else {
            panic!("header")
        };
        let mut convert = Converter::new(&header);
        let mut paused = false;
        let mut sequence = 0;
        for event in case["events"].as_array().unwrap() {
            if event == "pause" {
                paused = true;
                continue;
            }
            sequence += 1;
            let message = Pairs::new(
                u8::from(paused),
                sequence,
                0,
                vec![Pair {
                    voltage_code: event[0].as_u64().unwrap() as u16,
                    current_code: event[1].as_u64().unwrap() as u16,
                    low: event[2] == 1,
                    under_range: false,
                }],
            );
            convert.add_message(&message);
            paused = false;
        }
        convert.end();
        let total = convert.total();
        // The samples are the pairs the total counted, each once and in order, and sum to it.
        let samples = convert.take_samples();
        assert_eq!(samples.len() as u64, total.pairs);
        assert!(samples.iter().zip(1..).all(|(s, n)| s.sequence == n));
        // docs/host-interface.md's rule: mv × nA per pair, scaled by floor(2^32 × period / clock).
        let scale = (u128::from(period) << 32) / u128::from(clock);
        let q32 = samples.iter().fold(0u128, |sum, s| {
            sum + u128::from(s.millivolts) * u128::from(s.nanoamps) * scale
        });
        assert_eq!(
            (q32 >> 32, q32 & 0xFFFF_FFFF),
            (u128::from(total.picojoules), u128::from(total.fraction_q32)),
            "the samples at a period of {period}"
        );
        assert_eq!(
            (total.picojoules, u64::from(total.fraction_q32), total.pairs),
            (
                case["pj"].as_u64().unwrap(),
                case["fraction_q32"].as_u64().unwrap(),
                case["pairs"].as_u64().unwrap()
            ),
            "the sequence at a period of {period}"
        );
    }
}

#[test]
fn messages_parse_as_the_probe_lays_them_out() {
    let mut pairs = vec![stream::PAIRS, 3];
    pairs.extend_from_slice(&42u32.to_le_bytes());
    pairs.extend_from_slice(&1_500_000u64.to_le_bytes());
    pairs.extend_from_slice(&2u16.to_le_bytes());
    pairs.extend_from_slice(&[0x34, 0x12, 0x78, 0x56, 1, 0xCD, 0xAB, 0x01, 0xEF, 0]);
    let Message::Pairs(message) = stream::parse(&pairs).unwrap() else {
        panic!("not pairs");
    };
    assert!(message.paused() && message.held());
    assert_eq!((message.first_sequence, message.time_ns), (42, 1_500_000));
    assert_eq!(
        message.pairs,
        [
            Pair {
                voltage_code: 0x1234,
                current_code: 0x5678,
                low: true,
                under_range: false,
            },
            Pair {
                voltage_code: 0xABCD,
                current_code: 0xEF01,
                low: false,
                under_range: false,
            },
        ]
    );
    let Message::Header(at_1000) = stream::parse(&header_bytes(1000, 3000, &[], &[])).unwrap()
    else {
        panic!("header")
    };
    assert_eq!(message.time_of(1, &at_1000), 2_500_000);
    // Asked for 7 pairs a second the timer runs at 6.99999985: the period, not a rounded rate.
    let Message::Header(at_7) = stream::parse(&with_step_5(
        header_prefix(96_000_000, 13_714_286, 3000, &[], &[]),
        0,
        false,
        None,
    ))
    .unwrap() else {
        panic!("header")
    };
    assert_eq!(message.time_of(1, &at_7), 1_500_000 + 142_857_145);
    assert!((at_7.rate() - 6.999_999_854).abs() < 1e-9);
    assert_eq!(
        stream::parse(&[stream::END, 3, 7, 0, 0, 0]).unwrap(),
        Message::End(stream::End {
            reason: stream::EndReason::Restarted,
            last_sequence: 7,
            time_ns: None,
        })
    );
    assert_eq!(
        stream::parse(&pairs[..20]),
        Err(stream::StreamError::Short(20))
    );
    assert_eq!(
        stream::parse(&[0x90, 0, 0]),
        Err(stream::StreamError::Unknown(0x90))
    );
    let header = header_bytes(1000, 3000, &[(1, 2), (3, 4)], &[(5, 6), (7, 8)]);
    let Message::Header(parsed) = stream::parse(&header).unwrap() else {
        panic!("header")
    };
    assert_eq!(
        (
            parsed.clock_hz,
            parsed.period,
            parsed.full_scale_mv,
            parsed.first_sequence
        ),
        (96_000_000, 96_000, 3000, 1)
    );
    assert_eq!((parsed.low.len(), parsed.high[1].current_nanoamps), (2, 8));
    assert_eq!(
        stream::parse(&header[..78]),
        Err(stream::StreamError::Short(78))
    );
    // Format 4 carries the identity, then step 5's fields.
    let mut identified = header_prefix(
        96_000_000,
        96_000,
        3000,
        &[(1, 2), (3, 4)],
        &[(5, 6), (7, 8)],
    );
    identified[1] = 4;
    identified.extend_from_slice(&[0; 24]);
    let mut current = with_step_5(identified.clone(), 9786, true, Some(1_234_567));
    assert_eq!(stream::parse(&current), Err(stream::StreamError::Identity));
    current[88] = 1;
    let Message::Header(parsed) = stream::parse(&current).unwrap() else {
        panic!("header")
    };
    assert_eq!(parsed.capture_id.unwrap().as_bytes(), &current[72..96]);
    assert_eq!(
        (
            parsed.switch_weight_q16,
            parsed.rearm_between_pairs,
            parsed.previous_shunt_nanoamps
        ),
        (9786, true, Some(1_234_567))
    );
    assert_eq!(
        stream::parse(&current[..102]),
        Err(stream::StreamError::Short(102))
    );
    current[98] = 0;
    let Message::Header(parsed) = stream::parse(&current).unwrap() else {
        panic!("header")
    };
    assert_eq!(
        (parsed.rearm_between_pairs, parsed.previous_shunt_nanoamps),
        (false, None)
    );
    // Formats 1 and 2 weighed pairs by a rate rounded down: refused, not misread. So is one
    // past what this reads.
    for format in [1, 2, 6] {
        let mut other = header.clone();
        other[1] = format;
        assert_eq!(
            stream::parse(&other),
            Err(stream::StreamError::Format(format))
        );
    }
    // A DIO stream's header, format 5, and its end, which says when.
    let mut dio = vec![0xA1, 5];
    dio.extend_from_slice(&150_000_000u32.to_le_bytes());
    dio.extend_from_slice(&[0b0101, 0b0001, 7, 0]);
    let Message::DioHeader(parsed) = stream::parse(&dio).unwrap() else {
        panic!("DIO header")
    };
    assert_eq!(
        (
            parsed.clock_hz,
            parsed.dio_mask,
            parsed.dio_levels,
            parsed.edge_resolution_ns
        ),
        (150_000_000, 0b0101, 0b0001, 7)
    );
    assert_eq!(stream::parse(&dio[..9]), Err(stream::StreamError::Short(9)));
    let mut end = vec![stream::END, 1, 0, 0, 0, 0];
    end.extend_from_slice(&2_500_000_000u64.to_le_bytes());
    let Message::End(end) = stream::parse(&end).unwrap() else {
        panic!("end")
    };
    assert_eq!((end.last_sequence, end.time_ns), (0, Some(2_500_000_000)));
    assert!(stream::is_stream_message(&pairs) && !stream::is_stream_message(&[0x90, 0]));

    let mut edges = vec![stream::EDGES, 1];
    edges.extend_from_slice(&(u64::from(u32::MAX) + 9).to_le_bytes());
    edges.extend_from_slice(&2u16.to_le_bytes());
    edges.extend_from_slice(&[0, 0, 0, 0, 0x05, 0xFF, 0xFF, 0xFF, 0xFF, 0x04]);
    let Message::Edges(message) = stream::parse(&edges).unwrap() else {
        panic!("not edges");
    };
    // An old recording's flag, which firmware that sends DIO lost messages never sets.
    assert!(message.lost_before());
    let first = u64::from(u32::MAX) + 9;
    assert_eq!(
        message.edges,
        [
            stream::Edge {
                time_ns: first,
                levels: 5
            },
            stream::Edge {
                time_ns: first + u64::from(u32::MAX),
                levels: 4
            },
        ]
    );
    assert_eq!(message.to_bytes(), edges);
    assert_eq!(
        stream::parse(&edges[..21]),
        Err(stream::StreamError::Short(21))
    );
    assert_eq!(
        stream::parse(&edges[..11]),
        Err(stream::StreamError::Short(11))
    );

    // DIO lost: the two times, then the levels from the second on.
    let mut lost = vec![stream::DIO_LOST, 0];
    lost.extend_from_slice(&3_000u64.to_le_bytes());
    lost.extend_from_slice(&(u64::from(u32::MAX) + 2).to_le_bytes());
    lost.push(0x0A);
    let Message::DioLost(message) = stream::parse(&lost).unwrap() else {
        panic!("not a DIO lost message");
    };
    assert_eq!(
        message,
        stream::DioLost {
            from_ns: 3_000,
            to_ns: u64::from(u32::MAX) + 2,
            levels: 0x0A
        }
    );
    assert!(stream::is_stream_message(&lost) && message.to_bytes() == lost);
    assert_eq!(
        stream::parse(&lost[..18]),
        Err(stream::StreamError::Short(18))
    );

    // DIO time: a quiet DIO stream's time.
    let mut time = vec![stream::DIO_TIME, 0];
    time.extend_from_slice(&(u64::from(u32::MAX) + 9).to_le_bytes());
    let Message::DioTime(message) = stream::parse(&time).unwrap() else {
        panic!("not a DIO time message");
    };
    assert_eq!(message.time_ns, u64::from(u32::MAX) + 9);
    assert!(stream::is_stream_message(&time) && message.to_bytes() == time);
    assert_eq!(
        stream::parse(&time[..9]),
        Err(stream::StreamError::Short(9))
    );

    // Markers: count, then each marker's time, number, label length and label.
    let mut marks = vec![stream::MARKS, 1, 2];
    marks.extend_from_slice(&(u64::from(u32::MAX) + 5).to_le_bytes());
    marks.extend_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    marks.push(0);
    marks.extend_from_slice(&7u64.to_le_bytes());
    marks.extend_from_slice(&2u32.to_le_bytes());
    marks.push(32);
    marks.extend_from_slice(b"0123456789abcdef0123456789ABCDEF");
    let Message::Markers(message) = stream::parse(&marks).unwrap() else {
        panic!("not markers");
    };
    assert!(message.lost() && stream::is_stream_message(&marks));
    assert_eq!(
        message.markers,
        [
            stream::Marker {
                time_ns: u64::from(u32::MAX) + 5,
                number: 0xDEAD_BEEF,
                label: vec![]
            },
            stream::Marker {
                time_ns: 7,
                number: 2,
                label: b"0123456789abcdef0123456789ABCDEF".to_vec()
            },
        ]
    );
    assert_eq!(message.to_bytes(), marks);
    // Short anywhere: before the count, inside a marker's fixed part, and inside its label.
    for length in [2, 15, 28, marks.len() - 1] {
        assert_eq!(
            stream::parse(&marks[..length]),
            Err(stream::StreamError::Short(length)),
            "{length} bytes"
        );
    }
    assert!(!stream::is_stream_message(&[stream::DIO_TIME + 1]));
}

/// Step 5 in the samples: a pair at a range switch keeps its time and voltage, takes the next
/// pair's current, and comes out with that pair.
#[test]
fn a_switching_pair_takes_the_next_pairs_current() {
    let low = [(0, 0), (60_000, 60_000)];
    let high = [(0, 0), (60_000, 6_000_000)];
    let mut convert = converter(1000, 3300, &low, &high);
    let pair = |voltage_code, current_code, low| Pair {
        voltage_code,
        current_code,
        low,
        under_range: false,
    };
    // The header says the range before was low.
    convert.add_message(&Pairs::new(
        0,
        7,
        5_000_000,
        vec![pair(30_000, 100, true), pair(31_000, 200, false)],
    ));
    let first = convert.take_samples();
    assert_eq!(first.len(), 1, "the switching pair waits: {first:?}");
    convert.add_message(&Pairs::new(0, 9, 7_000_000, vec![pair(32_000, 300, false)]));
    let next = convert.take_samples();
    let taken = convert.reading(&pair(31_000, 300, false));
    assert_eq!(
        next[0],
        stream::Sample {
            sequence: 8,
            time_ns: 6_000_000,
            millivolts: taken.millivolts,
            nanoamps: taken.nanoamps,
            range: Some(stream::CurrentRange::High),
            under_range: false,
        }
    );
    assert_eq!(
        (next[1].sequence, next[1].time_ns, next.len()),
        (9, 7_000_000, 2)
    );
}

/// A capture that holds the high range (flag bit 2) has nothing to switch: its first pair after a
/// pause keeps its own current, where an automatic capture's would wait as a switch up.
#[test]
fn a_held_high_range_stays_high_across_a_pause() {
    let low = [(0, 0), (60_000, 60_000)];
    let high = [(0, 0), (60_000, 6_000_000)];
    let mut header = with_step_5(
        header_prefix(96_000_000, 96_000, 3300, &low, &high),
        0,
        false,
        None,
    );
    header[17] = 0;
    header[74] |= 4;
    let Message::Header(header) = stream::parse(&header).unwrap() else {
        panic!("not a header");
    };
    assert!(header.high_range_only && !header.previous_low);
    let mut convert = Converter::new(&header);
    let pair = |current_code| Pair {
        voltage_code: 30_000,
        current_code,
        low: false,
        under_range: false,
    };
    convert.add_message(&Pairs::new(0, 1, 0, vec![pair(100)]));
    convert.add_message(&Pairs::new(
        Pairs::PAUSED,
        2,
        5_000_000,
        vec![pair(200), pair(300)],
    ));
    let samples = convert.take_samples();
    assert_eq!(samples.len(), 3, "a pair waited: {samples:?}");
    assert_eq!(samples[1].nanoamps, convert.reading(&pair(200)).nanoamps);
}

#[test]
fn a_switch_up_weighs_the_pair_before_and_a_clean_switch_down_keeps_its_own() {
    let low = [(0, 0), (60_000, 60_000)];
    let high = [(0, 0), (60_000, 6_000_000)];
    // An attached stream: the pair before the first one sent read 400 nA on the low range.
    let header = with_step_5(
        header_prefix(96_000_000, 96_000, 3300, &low, &high),
        16384,
        true,
        Some(400),
    );
    let Message::Header(header) = stream::parse(&header).unwrap() else {
        panic!("header")
    };
    let mut convert = Converter::new(&header);
    let pair = |current_code, low| Pair {
        voltage_code: 30_000,
        current_code,
        low,
        under_range: false,
    };
    convert.add_message(&Pairs::new(
        0,
        1,
        0,
        vec![
            pair(59_000, false),
            pair(300, false),
            pair(200, true),
            pair(250, true),
        ],
    ));
    let samples = convert.take_samples();
    let shunt = |code, low| convert.reading(&pair(code, low)).shunt_nanoamps;
    let mv = convert.reading(&pair(0, true)).millivolts;
    // Three quarters of the next pair's current and a quarter of the one before, rounded.
    let up = ((u64::from(shunt(300, false)) * 49152 + 400 * 16384 + 32768) >> 16) as u32;
    assert_eq!(samples[0].nanoamps, stream::target_nanoamps(up, mv));
    // Down between pairs: its own, at once.
    assert_eq!(samples.len(), 4);
    assert_eq!(
        samples[2].nanoamps,
        stream::target_nanoamps(shunt(200, true), mv)
    );
}

#[test]
fn a_hold_keeps_the_pair_before_and_a_gap_forgets_it() {
    let low = [(0, 0), (60_000, 60_000)];
    let high = [(0, 0), (60_000, 6_000_000)];
    let header = with_step_5(
        header_prefix(96_000_000, 96_000, 3300, &low, &high),
        16384,
        true,
        None,
    );
    let Message::Header(header) = stream::parse(&header).unwrap() else {
        panic!("header")
    };
    let pair = |current_code, low| Pair {
        voltage_code: 30_000,
        current_code,
        low,
        under_range: false,
    };
    let up_after = |flags| {
        let mut convert = Converter::new(&header);
        convert.add_message(&Pairs::new(0, 1, 0, vec![pair(40_000, true)]));
        convert.add_message(&Pairs::new(
            flags,
            2,
            5_000_000,
            vec![pair(59_000, false), pair(300, false)],
        ));
        let before = convert.reading(&pair(40_000, true)).shunt_nanoamps;
        let next = convert.reading(&pair(300, false)).shunt_nanoamps;
        let samples = convert.take_samples();
        (samples[1].nanoamps, before, next, convert)
    };
    // A hold between the pair before and the switch up: the stand-in still weighs it.
    let (taken, before, next, convert) = up_after(Pairs::HELD);
    let mv = convert.reading(&pair(0, true)).millivolts;
    let weighed = ((u64::from(next) * 49152 + u64::from(before) * 16384 + 32768) >> 16) as u32;
    assert_eq!(taken, stream::target_nanoamps(weighed, mv));
    // A pause forgets it: the next pair's current alone.
    let (taken, _, next, _) = up_after(Pairs::PAUSED);
    assert_eq!(taken, stream::target_nanoamps(next, mv));
    // So do lost pairs.
    let mut convert = Converter::new(&header);
    convert.add_message(&Pairs::new(0, 1, 0, vec![pair(40_000, true)]));
    convert.discontinuity(true);
    convert.add_message(&Pairs::new(
        0,
        5,
        5_000_000,
        vec![pair(59_000, false), pair(300, false)],
    ));
    let samples = convert.take_samples();
    assert_eq!(samples[1].nanoamps, stream::target_nanoamps(next, mv));
}
