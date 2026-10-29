//! The stream converter against the vectors mculink-verge's own converter writes
//! (`tools/energy_vectors.json`, copied to `tests/fixtures`): samples and whole-range digests of each
//! pair's conversion, and pair sequences with pauses and the totals they give.

use serde_json::Value;
use verge_probe::stream::{self, Converter, Message, Pair, Pairs};

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

/// A header as the probe sends it, with the stream's tables.
fn header_bytes(rate: u32, full_scale: u16, low: &[(u16, u32)], high: &[(u16, u32)]) -> Vec<u8> {
    let mut bytes = vec![stream::HEADER, 1];
    bytes.extend_from_slice(&rate.to_le_bytes());
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
    assert_eq!(bytes.len(), 68);
    bytes
}

fn converter(rate: u32, full_scale: u16, low: &[(u16, u32)], high: &[(u16, u32)]) -> Converter {
    match stream::parse(&header_bytes(rate, full_scale, low, high)).unwrap() {
        Message::Header(header) => Converter::new(&header),
        other => panic!("{other:?}"),
    }
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
        let rate = case["rate"].as_u64().unwrap() as u32;
        let full_scale = case["full_scale_mv"].as_u64().unwrap() as u16;
        let mut convert = converter(
            rate,
            full_scale,
            &points(&case["low"]),
            &points(&case["high"]),
        );
        let mut paused = false;
        for event in case["events"].as_array().unwrap() {
            if event == "pause" {
                paused = true;
                continue;
            }
            let message = Pairs::new(
                u8::from(paused),
                0,
                0,
                vec![Pair {
                    voltage_code: event[0].as_u64().unwrap() as u16,
                    current_code: event[1].as_u64().unwrap() as u16,
                    low: event[2] == 1,
                }],
            );
            convert.add_message(&message);
            paused = false;
        }
        let total = convert.finish();
        assert_eq!(
            (total.picojoules, u64::from(total.fraction_q32), total.pairs),
            (
                case["pj"].as_u64().unwrap(),
                case["fraction_q32"].as_u64().unwrap(),
                case["pairs"].as_u64().unwrap()
            ),
            "the sequence at {rate} pairs/s"
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
                low: true
            },
            Pair {
                voltage_code: 0xABCD,
                current_code: 0xEF01,
                low: false
            },
        ]
    );
    assert_eq!(message.time_of(1, 1000), 2_500_000);
    assert_eq!(
        stream::parse(&[stream::END, 3, 7, 0, 0, 0]).unwrap(),
        Message::End(stream::End {
            reason: stream::EndReason::Restarted,
            last_sequence: 7
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
    let mut header = header_bytes(1000, 3000, &[(1, 2), (3, 4)], &[(5, 6), (7, 8)]);
    header[1] = 2;
    assert_eq!(stream::parse(&header), Err(stream::StreamError::Format(2)));
    assert!(stream::is_stream_message(&pairs) && !stream::is_stream_message(&[0x90, 0]));
}
