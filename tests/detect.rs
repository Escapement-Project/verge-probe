//! Detection and path choice against the simulated bench, whose probes answer from the recorded
//! identities.

use escapement_verge_probe::board::Board;
use escapement_verge_probe::dap::Ours;
use escapement_verge_probe::detect::{Guess, Hardware, Jumpered, State};
use escapement_verge_probe::sim::{SimBus, SimFirmware};
use escapement_verge_probe::version::StockVersion;
use escapement_verge_probe::{Bus, Firmware, Path, detect, offer};
use serde_json::Value;

/// The recorded `DAP_Info` block for `label`, read here rather than through the simulator.
fn recorded(label: &str) -> Value {
    let text = include_str!("../src/sim/recorded.json");
    let all: Value = serde_json::from_str(text).unwrap();
    all["records"]
        .as_array()
        .unwrap()
        .iter()
        .find(|r| r["label"] == label)
        .unwrap()
        .clone()
}

#[test]
fn the_bench_reads_back_as_recorded() {
    let bus = SimBus::scenario("bench").unwrap();
    let units = detect(&bus).unwrap();
    assert_eq!(units.len(), 2);

    for (unit, label) in units.iter().zip(["gcc", "stock"]) {
        let record = recorded(label);
        let info = &record["dap_info"];
        let State::Probe { identity, firmware } = &unit.state else {
            panic!("{label}: {:?}", unit.state);
        };
        assert_eq!(
            identity.vendor.as_deref(),
            info["vendor"].as_str(),
            "{label}"
        );
        assert_eq!(
            identity.product.as_deref(),
            info["product"].as_str(),
            "{label}"
        );
        assert_eq!(
            identity.serial.as_deref(),
            info["serial"].as_str(),
            "{label}"
        );
        assert_eq!(
            identity.protocol.as_deref(),
            info["protocol version"].as_str(),
            "{label}"
        );
        assert_eq!(
            identity.firmware_version.as_deref(),
            info["firmware version"].as_str(),
            "{label}"
        );
        assert_eq!(
            identity.packet_count.map(u64::from),
            info["max packet count"].as_u64(),
            "{label}"
        );
        assert_eq!(
            identity.packet_size.map(u64::from),
            info["max packet size"].as_u64(),
            "{label}"
        );
        match (firmware, &record["ours"]) {
            (Firmware::Ours(ours), Value::Object(recorded)) => {
                assert_eq!(
                    u64::from(ours.version),
                    recorded["format"].as_u64().unwrap()
                );
                assert_eq!(
                    u64::from(ours.capabilities),
                    recorded["flags"].as_u64().unwrap()
                );
                assert_eq!(ours.firmware, recorded["build"].as_str().unwrap());
            }
            (Firmware::Stock { version }, Value::Null) => {
                assert_eq!(*version, StockVersion::parse("3.172"));
            }
            other => panic!("{label}: {other:?}"),
        }
    }
    assert_eq!(units[0].state.to_string(), "Verge, build 14d05d4");
    assert_eq!(units[1].state.to_string(), "NXP's firmware V3.172");
}

/// A stock version other than the recorded one must not read back as the recorded strings.
#[test]
fn another_stock_version_reads_back_as_itself() {
    let bus = SimBus::default();
    bus.add_stock("3.108");
    let units = detect(&bus).unwrap();
    let State::Probe { identity, firmware } = &units[0].state else {
        panic!("{:?}", units[0].state);
    };
    assert_eq!(identity.firmware_version.as_deref(), Some("3.108"));
    assert_ne!(
        identity.product,
        recorded("stock")["dap_info"]["product"]
            .as_str()
            .map(str::to_owned)
    );
    assert_eq!(
        *firmware,
        Firmware::Stock {
            version: StockVersion::parse("3.108")
        }
    );
}

#[test]
fn replies_left_by_an_earlier_host_are_drained_first() {
    let bus = SimBus::default();
    let unit = bus.add_ours("14d05d4");
    bus.leave_stale_replies(unit, vec![vec![0x05, 0x00, 0x01], vec![0x06, 0x00]]);
    let units = detect(&bus).unwrap();
    let State::Probe { identity, .. } = &units[0].state else {
        panic!("{:?}", units[0].state);
    };
    assert_eq!(identity.serial.as_deref(), Some("CJ3E7A91C2"));
}

#[test]
fn a_probe_held_by_another_program_is_named_and_guessed() {
    let bus = SimBus::scenario("busy").unwrap();
    let units = detect(&bus).unwrap();
    assert_eq!(
        units[0].state,
        State::Unreadable {
            reason: "another program has it open".into(),
            guess: Guess::Ours,
        }
    );
    let offer = offer(&units[0], &units);
    assert_eq!(offer.paths, [Path::Jumper]);
}

/// A debugger holding CMSIS-DAP leaves Verge Aux, which says what the Pro runs; nothing that goes
/// through CMSIS-DAP is offered.
#[test]
fn a_pro_held_by_a_debugger_is_asked_over_verge_aux() {
    let bus = SimBus::scenario("ours-pro-debugger").unwrap();
    let units = detect(&bus).unwrap();
    let State::Held { ours } = &units[0].state else {
        panic!("{:?}", units[0].state);
    };
    assert_eq!(ours.firmware, "26ea505");
    assert_eq!(units[0].hardware(), Hardware::Board(Board::McuLinkPro));
    assert_eq!(
        units[0].build(),
        Some(escapement_verge_probe::version::Build::parse("26ea505"))
    );
    assert_eq!(units[0].serial(), units[0].device.serial.as_deref());
    // No update while the debugger holds it, and not the jumper either: closing it is enough.
    let offer = offer(&units[0], &units);
    assert_eq!(offer.paths, []);
    assert!(
        offer.notes[0].starts_with("Close the debugger first."),
        "{:?}",
        offer.notes
    );

    bus.set_aux_busy(0, true);
    assert_eq!(
        detect(&bus).unwrap()[0].state,
        State::Unreadable {
            reason: "another program has it open".into(),
            guess: Guess::Ours,
        }
    );
}

#[test]
fn stock_v2_speaks_hid_only_and_gets_the_jumper() {
    let bus = SimBus::scenario("stock-hid").unwrap();
    let units = detect(&bus).unwrap();
    let State::Unreadable { guess, .. } = &units[0].state else {
        panic!("{:?}", units[0].state);
    };
    assert_eq!(*guess, Guess::Stock(StockVersion::parse("2.245")));
    assert_eq!(offer(&units[0], &units).paths, [Path::Jumper]);
}

#[test]
fn every_scenario_offers_what_its_state_allows() {
    let cases: &[(&str, &[&[Path]])] = &[
        (
            "bench",
            &[
                &[Path::OursJumperFree, Path::Jumper],
                &[Path::StockJumperFree, Path::Jumper],
            ],
        ),
        ("ours", &[&[Path::OursJumperFree, Path::Jumper]]),
        ("stock", &[&[Path::StockJumperFree, Path::Jumper]]),
        ("stock-old", &[&[Path::Jumper]]),
        ("daplink-rs", &[&[Path::DaplinkRsJumperFree, Path::Jumper]]),
        ("isp", &[&[Path::FlashNow]]),
        ("isp-jumper", &[&[Path::FlashNow]]),
        ("two-isp", &[&[], &[]]),
        ("mini", &[&[]]),
        ("isp-mini", &[&[], &[]]),
    ];
    for (name, expected) in cases {
        let bus = SimBus::scenario(name).unwrap();
        let units = detect(&bus).unwrap();
        let got: Vec<Vec<Path>> = units.iter().map(|u| offer(u, &units).paths).collect();
        let expected: Vec<Vec<Path>> = expected.iter().map(|p| p.to_vec()).collect();
        assert_eq!(got, expected, "{name}");
    }
}

#[test]
fn ours_is_told_from_stock_by_the_identify_reply_alone() {
    // A unit whose USB strings look like stock but which answers the identify command is ours.
    let bus = SimBus::default();
    bus.add_running(SimFirmware::Ours {
        build: "v1.0.0+armwire".into(),
    });
    let units = detect(&bus).unwrap();
    let build = units[0].build().unwrap();
    assert_eq!(build.variant.as_deref(), Some("armwire"));
}

/// LinkServer's Readme: `MCU-LINK [Pro|MR|on-board|<board>] (r<RRR>) CMSIS-DAP V<V>.<BBB>`. The
/// first row is the string recorded from the bench's stock unit.
#[test]
fn stocks_product_string_names_the_board() {
    let other = |name: &str| Hardware::Other(name.to_owned());
    for (product, board) in [
        (
            "MCU-LINK (r0FB) CMSIS-DAP V3.172",
            Hardware::Board(Board::McuLink),
        ),
        (
            "MCU-LINK Pro (r1E2) CMSIS-DAP V3.172",
            Hardware::Board(Board::McuLinkPro),
        ),
        (
            "MCU-LINK MR (r0C3) CMSIS-DAP V3.172",
            other("an MCU-Link MR"),
        ),
        (
            "MCU-LINK on-board (r0E7) CMSIS-DAP V3.122",
            other("an on-board MCU-Link"),
        ),
        (
            "MCU-LINK FRDM-MCXN947 (r0E7) CMSIS-DAP V3.172",
            other("the MCU-Link on a FRDM-MCXN947"),
        ),
        // Not the Readme's form, so nothing is known.
        ("MCU-LINK CMSIS-DAP V2.245", Hardware::Unknown),
        ("Verge CMSIS-DAP", Hardware::Unknown),
    ] {
        assert_eq!(Hardware::from_stock_product(product), board, "{product}");
    }
}

#[test]
fn a_label_names_the_board_without_an_article() {
    for (hardware, label) in [
        (Hardware::Board(Board::McuLink), "standalone MCU-Link"),
        (Hardware::Board(Board::McuLinkPro), "MCU-Link Pro"),
        (
            Hardware::from_stock_product("MCU-LINK MR (r0C3) CMSIS-DAP V3.172"),
            "MCU-Link MR",
        ),
        (
            Hardware::from_stock_product("MCU-LINK on-board (r0E7) CMSIS-DAP V3.122"),
            "on-board MCU-Link",
        ),
        (
            Hardware::from_stock_product("MCU-LINK FRDM-MCXN947 (r0E7) CMSIS-DAP V3.172"),
            "MCU-Link on a FRDM-MCXN947",
        ),
        (Hardware::Unknown, "not reported"),
    ] {
        assert_eq!(hardware.name(), label);
    }
}

#[test]
fn each_unit_says_which_board_it_is() {
    let bus = SimBus::default();
    let plain_ours = bus.add_ours("v0.3.0");
    let pro_ours = bus.add_ours("v0.3.0");
    bus.set_board(pro_ours, Ours::BOARD_MCU_LINK_PRO);
    let on_board_ours = bus.add_ours("v0.3.0");
    bus.set_board(on_board_ours, Ours::BOARD_ON_BOARD);
    let pro_stock = bus.add_stock("3.172");
    bus.set_board(pro_stock, Ours::BOARD_MCU_LINK_PRO);
    bus.add_stock("3.172");
    bus.add_bootloader(false);
    let boards: Vec<Hardware> = detect(&bus).unwrap().iter().map(|u| u.hardware()).collect();
    assert_eq!(
        boards,
        [
            // Format 2, as recorded: built only for the plain board.
            Hardware::Board(Board::McuLink),
            Hardware::Board(Board::McuLinkPro),
            Hardware::Other("an on-board MCU-Link".into()),
            Hardware::Board(Board::McuLinkPro),
            Hardware::Board(Board::McuLink),
            Hardware::Unknown,
        ]
    );
    let _ = plain_ours;
}

/// The SWD jumper takes the CMSIS-DAP interfaces away, so the tool reads what such a unit runs,
/// and its board, from its USB descriptors alone. Ours names only the Pro there: its full-board
/// string is an on-board MCU-Link's too. Ours can still be sent to its bootloader, except on
/// Windows; stock cannot.
#[test]
fn a_unit_with_its_swd_jumper_is_read_from_its_usb_descriptors() {
    let v3172 = StockVersion::parse("3.172");
    let ours_paths = if cfg!(windows) {
        vec![Path::Jumper]
    } else {
        vec![Path::OursJumperFree, Path::Jumper]
    };
    for (scenario, firmware, board, paths, shown) in [
        (
            "swd-jumper",
            Jumpered::Ours,
            Hardware::Unknown,
            ours_paths.clone(),
            "Verge, SWD jumper fitted",
        ),
        (
            "pro-swd-jumper",
            Jumpered::Ours,
            Hardware::Board(Board::McuLinkPro),
            ours_paths.clone(),
            "Verge, SWD jumper fitted",
        ),
        (
            "stock-swd-jumper",
            Jumpered::Stock(v3172),
            Hardware::Board(Board::McuLink),
            vec![Path::Jumper],
            "NXP's firmware V3.172, SWD jumper fitted",
        ),
    ] {
        let bus = SimBus::scenario(scenario).unwrap();
        let units = detect(&bus).unwrap();
        let unit = &units[0];
        assert_eq!(unit.state, State::SwdJumper { firmware }, "{scenario}");
        assert_eq!(unit.hardware(), board, "{scenario}");
        assert_eq!(unit.state.to_string(), shown, "{scenario}");
        let offer = offer(unit, &units);
        assert_eq!(offer.paths, paths, "{scenario}");
        assert!(
            offer.notes[0].starts_with("The SWD jumper is fitted"),
            "{scenario}: {:?}",
            offer.notes
        );
        let log = bus.log();
        assert!(!log.iter().any(|l| l.contains(": DAP ")), "{log:#?}");
    }
}

/// Stock before V3.122 has no jumper-free update, so taking the SWD jumper off is not the advice.
#[test]
fn old_stock_with_its_swd_jumper_is_not_told_to_take_it_off() {
    let bus = SimBus::default();
    let unit = bus.add_stock("3.108");
    bus.set_swd_jumper(unit, true);
    bus.unplug(unit);
    bus.plug_back_later(std::time::Duration::ZERO, unit);
    let units = detect(&bus).unwrap();
    assert_eq!(
        units[0].state,
        State::SwdJumper {
            firmware: Jumpered::Stock(StockVersion::parse("3.108"))
        }
    );
    let offer = offer(&units[0], &units);
    assert_eq!(offer.paths, [Path::Jumper]);
    assert!(
        offer.notes[0].contains("NXP's firmware V3.108 has no jumper-free update"),
        "{:?}",
        offer.notes
    );
    assert!(
        !offer
            .notes
            .iter()
            .any(|n| n.contains("take the SWD jumper off"))
    );
}

#[test]
fn naming_a_unit_leaves_a_unit_another_program_holds_unopened() {
    let bus = SimBus::default();
    let mine = bus.add_ours("v0.3.0");
    let theirs = bus.add_ours("v0.3.0");
    bus.set_serials(mine, "CJ00000001", "NXP0000001");
    bus.set_serials(theirs, "CJ00000002", "NXP0000002");
    bus.set_busy(theirs, true);
    let units = escapement_verge_probe::detect::detect_named(&bus, Some("CJ00000001")).unwrap();
    assert_eq!(units.len(), 1, "the other unit was asked");
    assert_eq!(units[0].serial(), Some("CJ00000001"));
    // A serial no USB descriptor carries asks every unit, as DAP_Info may still name it.
    let units = escapement_verge_probe::detect::detect_named(&bus, Some("CJ0000000X")).unwrap();
    assert_eq!(units.len(), 2);
    assert!(
        units
            .iter()
            .any(|u| matches!(u.state, State::Unreadable { .. }))
    );
}

#[test]
fn daplink_rs_is_asked_only_its_own_discovery() {
    let bus = SimBus::scenario("daplink-rs").unwrap();
    let units = detect(&bus).unwrap();
    let [unit] = units.as_slice() else {
        panic!("{units:?}");
    };
    let State::Probe { identity, firmware } = &unit.state else {
        panic!("{:?}", unit.state);
    };
    assert_eq!(
        *firmware,
        Firmware::DaplinkRs {
            reboot: Some(escapement_verge_probe::daplink_rs::Reboot {
                version: 1,
                kind: escapement_verge_probe::daplink_rs::KIND_NXP_ISP
            })
        }
    );
    // dap-rs leaves the serial out of DAP_Info, so the USB serial names the unit.
    assert_eq!(identity.serial, None);
    assert_eq!(unit.serial(), Some("0000AA2B3E7A91C2"));
    assert_eq!(unit.hardware(), Hardware::Unknown);
    assert_eq!(unit.state.to_string(), "daplink-rs 2.1.1.g507437947680");
    // Another DAPLink gives 0x82 a meaning of its own, so detection never sends it.
    let log = bus.log();
    assert!(log.iter().any(|l| l == "1-1: DAP 9f00"), "{log:#?}");
    assert!(
        !log.iter().any(|l| l.starts_with("1-1: DAP 82")),
        "{log:#?}"
    );
}

#[test]
fn naming_another_unit_leaves_daplink_rs_unopened() {
    let bus = SimBus::default();
    bus.add_ours("v0.3.0");
    bus.add_daplink_rs("507437947680");
    let units = escapement_verge_probe::detect::detect_named(&bus, Some("CJ3E7A91C2")).unwrap();
    assert_eq!(units.len(), 1, "{units:?}");
    assert!(
        !bus.log().iter().any(|l| l.starts_with("1-2: DAP")),
        "{:#?}",
        bus.log()
    );
    // Named by its USB serial, it alone is asked.
    let serial = bus.devices().unwrap()[1].serial.clone().unwrap();
    let units = escapement_verge_probe::detect::detect_named(&bus, Some(&serial)).unwrap();
    let [unit] = units.as_slice() else {
        panic!("{units:?}");
    };
    assert!(
        matches!(
            unit.state,
            State::Probe {
                firmware: Firmware::DaplinkRs { .. },
                ..
            }
        ),
        "{unit:?}"
    );
}

/// A unit running Verge takes a restart request even while another program holds it, as its
/// product string says; one this user may not open, or one running NXP's firmware, does not.
#[test]
fn a_restart_goes_to_verge_unless_the_unit_cannot_be_opened() {
    let takes = |name: &str| {
        detect(&SimBus::scenario(name).unwrap())
            .unwrap()
            .remove(0)
            .takes_restart()
    };
    assert!(takes("ours"));
    assert!(takes("busy"));
    assert!(!takes("denied"));
    assert!(!takes("stock"));
}
