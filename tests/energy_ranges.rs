//! `0x90` mode 14 on the simulated Pro: a capture that switches automatically, the default, and
//! one that holds the high range, whose pairs near the amplifier's floor come marked under range.
//! The setting is the next start's alone.
use escapement_verge_probe::aux::Aux;
use escapement_verge_probe::dap::Ours;
use escapement_verge_probe::sim::{SimAux, SimBus};
use escapement_verge_probe::stream::{Converter, Header, Message, Pair};
use escapement_verge_probe::{Bus, EnergyRanges, VergeControl, detect};
use std::time::Duration;

/// A Pro's Verge Aux with the target supply on.
fn pro() -> Aux<SimAux> {
    let bus = SimBus::default();
    let unit = bus.add_ours("ranges");
    bus.set_board(unit, Ours::BOARD_MCU_LINK_PRO);
    let device = detect(&bus).unwrap().remove(0).device;
    let mut aux: Aux<SimAux> = Aux::open(bus.open_aux(&device).unwrap()).unwrap();
    aux.exchange(&[0x8F, 1]).unwrap();
    aux
}

/// Starts a capture streaming at 1,000 pairs a second with `ranges` asked for first, or nothing
/// asked, and returns its header.
fn start(aux: &mut Aux<SimAux>, ranges: Option<EnergyRanges>) -> Header {
    if let Some(ranges) = ranges {
        VergeControl::new(aux).set_energy_ranges(ranges).unwrap();
    }
    VergeControl::new(aux)
        .start_energy_stream_identified(1000, 0)
        .unwrap();
    // A replaced capture's last pairs and its end come first.
    loop {
        match aux.message(Duration::from_secs(1)).unwrap() {
            Some(Message::Header(header)) => return header,
            Some(_) => {}
            None => panic!("no header"),
        }
    }
}

/// A capture on a fresh Pro, as `start`, and its first 300 pairs.
fn capture(ranges: Option<EnergyRanges>) -> (Header, Vec<Pair>, Converter) {
    let mut aux = pro();
    let header = start(&mut aux, ranges);
    let mut converter = Converter::new(&header);
    let mut pairs = Vec::new();
    // Three of the simulated target's 100 ms periods: 10 ms awake at 5 mA, then asleep at 20 µA.
    while pairs.len() < 300 {
        match aux.message(Duration::from_secs(1)).unwrap() {
            Some(Message::Pairs(message)) => {
                converter.add_message(&message);
                pairs.extend(message.pairs);
            }
            Some(_) => {}
            None => panic!("the stream went quiet"),
        }
    }
    (header, pairs, converter)
}

#[test]
fn a_capture_switches_automatically_unless_asked() {
    for ranges in [None, Some(EnergyRanges::Auto)] {
        let (header, pairs, _) = capture(ranges);
        assert!(!header.high_range_only && header.previous_low, "{ranges:?}");
        assert!(pairs.iter().any(|p| p.low) && pairs.iter().any(|p| !p.low));
    }
}

#[test]
fn the_setting_is_the_next_start_s_alone() {
    let mut aux = pro();
    assert!(start(&mut aux, Some(EnergyRanges::High)).high_range_only);
    assert!(!start(&mut aux, None).high_range_only);
    assert!(start(&mut aux, Some(EnergyRanges::High)).high_range_only);
    assert!(!start(&mut aux, Some(EnergyRanges::Auto)).high_range_only);
}

#[test]
fn a_capture_that_holds_the_high_range_marks_the_floor() {
    let (header, pairs, mut converter) = capture(Some(EnergyRanges::High));
    assert!(header.high_range_only && !header.previous_low);
    assert!(pairs.iter().all(|p| !p.low));
    // Asleep, 20 µA is under the floor; awake, 5 mA is not.
    let under = pairs.iter().filter(|p| p.under_range).count();
    assert!(
        under > 200 && under < pairs.len(),
        "{under} of {}",
        pairs.len()
    );
    let samples = converter.take_samples();
    assert_eq!(
        samples.iter().filter(|s| s.under_range).count(),
        pairs[..samples.len()]
            .iter()
            .filter(|p| p.under_range)
            .count()
    );
}
