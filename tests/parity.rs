//! What `verge` does in a call or two: picking a unit, changing the DIO pins with the probe's
//! checks, stopping whatever capture runs, and naming a pair's sense path.

use escapement_verge_probe::detect::{PickError, pick};
use escapement_verge_probe::dio::{self, DioChange, DioError};
use escapement_verge_probe::pro::{self, Feature, FindError, ProError};
use escapement_verge_probe::sim::{SimAux, SimBus};
use escapement_verge_probe::stream::CurrentRange;
use escapement_verge_probe::verge_aux::Aux;
use escapement_verge_probe::{Bus, EnergyStop, SensePath, Unit, VergeControl, detect};

fn bench(scenario: &str) -> (SimBus, Vec<Unit>) {
    let bus = SimBus::scenario(scenario).unwrap();
    let units = detect(&bus).unwrap();
    (bus, units)
}

fn open(bus: &SimBus, unit: &Unit) -> Aux<SimAux> {
    Aux::open(bus.open_aux(&unit.device).unwrap()).unwrap()
}

#[test]
fn a_pick_takes_the_unit_named_or_the_only_one_that_fits() {
    let (_bus, units) = bench("ours-pro-and-plain");
    let is_pro = |unit: &Unit| pro::validate(unit, Feature::Energy).is_ok();
    let pro = pick(&units, None, is_pro).unwrap();
    assert!(is_pro(pro));
    assert!(matches!(
        pick(&units, None, |_| true),
        Err(PickError::Several(several)) if several.len() == 2
    ));
    let plain = units.iter().find(|unit| !is_pro(unit)).unwrap();
    let serial = plain.serial().unwrap();
    assert_eq!(pick(&units, Some(serial), is_pro).unwrap(), plain);
    assert_eq!(
        pick(&units, Some("NOSUCH"), |_| true),
        Err(PickError::NoSuchSerial("NOSUCH".into()))
    );
    assert!(matches!(
        pick(&units, None, |_| false),
        Err(PickError::NoneFits(all)) if all.len() == 2
    ));
    assert_eq!(pick(&[], None, |_| true), Err(PickError::NoneAttached));
}

#[test]
fn find_takes_the_one_unit_with_the_feature_or_says_why_not() {
    let (bus, units) = bench("ours-pro-and-plain");
    let found = pro::find(&bus, None, Feature::Energy).unwrap();
    assert!(pro::validate(&found, Feature::Energy).is_ok());
    assert!(matches!(
        pro::find(&bus, None, Feature::Dio),
        Err(FindError::Several { units, .. }) if units.len() == 2
    ));
    let plain = units
        .iter()
        .find(|unit| pro::validate(unit, Feature::Energy).is_err())
        .unwrap();
    assert!(matches!(
        pro::find(&bus, plain.serial(), Feature::Supply),
        Err(FindError::Refused(ProError::NotPro { .. }))
    ));
    assert!(matches!(
        pro::find(&bus, Some("NOSUCH"), Feature::Dio),
        Err(FindError::NoSuchSerial(_))
    ));
    // A lone unit that cannot give the feature is refused saying why.
    let (bus, _) = bench("pro");
    assert!(matches!(
        pro::find(&bus, None, Feature::Energy),
        Err(FindError::Refused(ProError::NotVerge { .. }))
    ));
    assert!(matches!(
        pro::find(&SimBus::default(), None, Feature::Energy),
        Err(FindError::NoneAttached)
    ));
}

#[test]
fn pins_are_named_and_parsed_as_the_command_line_takes_them() {
    assert_eq!(dio::names(0b1011), "DIO0, DIO1, DIO3");
    assert_eq!(dio::parse_pin("2"), Ok(0b0100));
    assert_eq!(dio::parse_pin("DIO3"), Ok(0b1000));
    assert_eq!(dio::parse_pin("All"), Ok(dio::ALL));
    for bad in ["4", "dio", "dio-1", "+1", ""] {
        assert!(dio::parse_pin(bad).is_err(), "{bad}");
    }
    assert_eq!(dio::parse_pins("0, dio3"), Ok(0b1001));
    assert!(dio::parse_pins("0,,3").is_err());
}

/// A change is checked against the pins' state before it is sent: a group it would split, pins
/// the bench marker holds, pins named twice. An empty change reads.
#[test]
fn a_dio_change_is_refused_before_it_is_sent_where_the_probe_would() {
    let (bus, units) = bench("ours-pro");
    let mut aux = open(&bus, &units[0]);
    let mut control = VergeControl::new(&mut aux);
    let before = control.change_dio(&DioChange::default()).unwrap();
    assert_eq!(before.outputs, 0);
    assert!(matches!(
        control.change_dio(&DioChange::default().high(0b0001)),
        Err(DioError::Split(0b0111))
    ));
    assert!(matches!(
        control.change_dio(&DioChange::default().high(0b1000).input(0b1000)),
        Err(DioError::Overlap(0b1000))
    ));
    let sent = |bus: &SimBus| {
        bus.log()
            .iter()
            .filter(|line| line.contains("AUX 8e"))
            .count()
    };
    let reads = sent(&bus);
    let after = control
        .change_dio(&DioChange::default().high(0b1000).low(0b0111))
        .unwrap();
    assert_eq!((after.outputs, after.levels), (0b1111, 0b1000));
    // One read for the checks, one for the change.
    assert_eq!(sent(&bus) - reads, 2);

    let (bus, units) = bench("ours-plain-marked");
    let mut aux = open(&bus, &units[0]);
    let mut control = VergeControl::new(&mut aux);
    assert!(matches!(
        control.change_dio(&DioChange::default().low(0b0101)),
        Err(DioError::Held(0b0100))
    ));
    let after = control
        .change_dio(&DioChange::default().low(0b0001))
        .unwrap();
    assert_eq!(after.outputs, 0b0001);
}

#[test]
fn stopping_whatever_runs_says_what_it_found() {
    let (bus, units) = bench("ours-pro-energy");
    let mut aux = open(&bus, &units[0]);
    let mut control = VergeControl::new(&mut aux);
    assert_eq!(
        control.stop_running_energy_capture().unwrap(),
        EnergyStop::Stopped
    );
    assert_eq!(
        control.stop_running_energy_capture().unwrap(),
        EnergyStop::NotRunning
    );
    let (bus, units) = bench("ours-pro-energy");
    bus.fail_energy(0);
    let mut aux = open(&bus, &units[0]);
    assert_eq!(
        VergeControl::new(&mut aux)
            .stop_running_energy_capture()
            .unwrap(),
        EnergyStop::AlreadyFailed
    );
}

#[test]
fn a_range_id_names_its_range_and_setting() {
    for (id, range, setting_ma) in [
        (1, CurrentRange::Low, 350),
        (2, CurrentRange::High, 350),
        (3, CurrentRange::Low, 50),
        (4, CurrentRange::High, 50),
    ] {
        let path = SensePath::from_id(id).unwrap();
        assert_eq!((path.range, path.setting_ma), (range, setting_ma), "{id}");
    }
    assert_eq!(
        SensePath::from_id(4).unwrap().to_string(),
        "high range, 50 mA setting"
    );
    assert_eq!(SensePath::from_id(0), None);
    assert_eq!(SensePath::from_id(5), None);
}
