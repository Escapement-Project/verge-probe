//! The update paths a unit's state allows.

use std::fmt;

use crate::detect::{Firmware, Guess, Jumpered, State, Unit};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Path {
    /// Ours reboots into the bootloader on request, then the tool flashes and resets it.
    OursJumperFree,
    /// Stock V3.122 and later does the same on the request LinkServer sends.
    StockJumperFree,
    /// Fit the ISP jumper before plugging the unit in; the ROM samples the pin at reset.
    Jumper,
    /// The unit is in ISP mode already, so the tool can flash it now.
    FlashNow,
}

impl Path {
    /// Whether the tool can carry the path out by itself.
    #[must_use]
    pub fn is_jumper_free(self) -> bool {
        matches!(
            self,
            Path::OursJumperFree | Path::StockJumperFree | Path::FlashNow
        )
    }

    /// What the path involves, in a sentence or two for a user.
    #[must_use]
    pub fn describe(self) -> &'static str {
        match self {
            Path::OursJumperFree => {
                "No jumper needed. The tool asks the firmware to restart into its bootloader, flashes it, and restarts it."
            }
            Path::StockJumperFree => {
                "No jumper needed. The tool sends the request NXP's LinkServer uses, then flashes and restarts the unit."
            }
            Path::Jumper => {
                "Unplug the unit, fit the ISP jumper, and plug it back in so that its bootloader starts. Flash it, then unplug it and take the jumper off."
            }
            Path::FlashNow => "The unit is in ISP mode. The tool can flash it now.",
        }
    }
}

impl fmt::Display for Path {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Path::OursJumperFree => "jumper-free (ours)",
            Path::StockJumperFree => "jumper-free (stock)",
            Path::Jumper => "ISP jumper",
            Path::FlashNow => "flash now",
        })
    }
}

/// Said before flashing a unit that was in ISP mode already. A jumper-free update flashes the
/// bootloader that appears on the port of the unit it restarted, so it needs no such warning.
pub const ISP_IDENTITY_WARNING: &str =
    "The tool cannot tell an MCU-Link from any other LPC55S69 board in ISP mode.";

/// The paths open to one unit, best first, and what the user should know about the rest.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Offer {
    pub paths: Vec<Path>,
    pub notes: Vec<String>,
}

impl Offer {
    /// The path an update takes when the user does not pick one.
    #[must_use]
    pub fn best(&self) -> Option<Path> {
        self.paths.first().copied()
    }
}

/// The paths `unit` allows, given every attached unit.
///
/// Every LPC55S69 ROM enumerates alike, so the tool flashes a bootloader only when it is the only
/// one, and starts a jumper-free entry only when no other unit is in ISP mode.
#[must_use]
pub fn offer(unit: &Unit, all: &[Unit]) -> Offer {
    let in_isp = all.iter().filter(|other| other.is_bootloader()).count();
    let minis = all
        .iter()
        .filter(|other| other.state == State::MiniBootloader)
        .count();
    let mut offer = Offer::default();
    let jumper_free = |offer: &mut Offer, path: Path| {
        if in_isp + minis == 0 {
            offer.paths.push(path);
        } else {
            offer.notes.push(
                "Another unit is in ISP mode, so the jumper-free path waits until it is flashed or unplugged: the tool flashes only while one bootloader, and no other, is attached."
                    .into(),
            );
        }
    };
    match &unit.state {
        State::Probe {
            firmware: Firmware::Ours(_),
            ..
        } => jumper_free(&mut offer, Path::OursJumperFree),
        State::Probe {
            firmware: Firmware::Stock { version },
            ..
        } => match version {
            Some(version) if version.has_jumper_free_update() => {
                jumper_free(&mut offer, Path::StockJumperFree);
            }
            Some(version) => offer.notes.push(format!(
                "Stock V{version} has no jumper-free update. NXP added it in V3.122."
            )),
            None => offer.notes.push(
                "Stock did not report its version, so the tool will not try the jumper-free update.".into(),
            ),
        },
        State::Unreadable { reason, guess } => {
            offer
                .notes
                .push(format!("The tool could not ask this unit what it runs: {reason}."));
            if *guess == Guess::Ours || matches!(guess, Guess::Stock(Some(v)) if v.has_jumper_free_update())
            {
                offer.notes.push(
                    "Close whatever has it open and look again for the jumper-free path.".into(),
                );
            }
        }
        State::SwdJumper {
            firmware: Jumpered::Ours,
        } => {
            offer.notes.push(
                "The SWD jumper is fitted. It takes away the CMSIS-DAP interface, so the tool cannot ask which build this unit runs.".into(),
            );
            if cfg!(windows) {
                offer.notes.push(
                    "On Windows the tool can send the restart request only through that interface. Take the SWD jumper off to update the unit without the ISP jumper.".into(),
                );
            } else {
                jumper_free(&mut offer, Path::OursJumperFree);
            }
        }
        State::SwdJumper {
            firmware: Jumpered::Stock(version),
        } => offer.notes.push(match version {
            Some(version) if version.has_jumper_free_update() => {
                "The SWD jumper is fitted. It takes away the CMSIS-DAP interface that stock's jumper-free update goes through: unplug the unit, take the SWD jumper off and plug it back in to use that update.".into()
            }
            Some(version) => format!(
                "The SWD jumper is fitted. Stock V{version} has no jumper-free update either way. NXP added it in V3.122."
            ),
            None => "The SWD jumper is fitted, and stock's product string gives no version, so the tool does not know whether it has a jumper-free update.".into(),
        }),
        State::Bootloader if in_isp > 1 => {
            offer.notes.push(format!(
                "{in_isp} units are in ISP mode and every LPC55S69 bootloader looks the same. Unplug all but one."
            ));
            return offer;
        }
        State::Bootloader if minis > 0 => {
            offer.notes.push(
                "An MCU-Link mini is in ISP mode too. The tool flashes only while one bootloader, and no other, is attached: unplug the mini first."
                    .into(),
            );
            return offer;
        }
        State::Bootloader => {
            offer.paths.push(Path::FlashNow);
            return offer;
        }
        State::MiniBootloader => {
            offer.notes.push(
                "This is an MCU-Link mini, a different board. This firmware is for the full MCU-Link.".into(),
            );
            return offer;
        }
    }
    offer.paths.push(Path::Jumper);
    offer
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::UsbDevice;
    use crate::dap::{Identity, Ours};
    use crate::version::StockVersion;

    fn unit(state: State) -> Unit {
        Unit {
            device: UsbDevice {
                vid: 0x1fc9,
                pid: 0x0143,
                location: None,
                instance: "1".into(),
                serial: None,
                manufacturer: None,
                product: None,
                bcd_device: 0,
                dap_v2: true,
                hid: false,
            },
            state,
        }
    }

    fn ours() -> Unit {
        unit(State::Probe {
            identity: Identity::default(),
            firmware: Firmware::Ours(Ours {
                format: 2,
                flags: 0,
                build: "14d05d4".into(),
                board: None,
                scan_formats: None,
            }),
        })
    }

    fn stock(minor: u16) -> Unit {
        unit(State::Probe {
            identity: Identity::default(),
            firmware: Firmware::Stock {
                version: Some(StockVersion { major: 3, minor }),
            },
        })
    }

    #[test]
    fn each_state_gets_its_paths() {
        let isp = unit(State::Bootloader);
        let mini = unit(State::MiniBootloader);
        let cases = [
            (ours(), vec![Path::OursJumperFree, Path::Jumper]),
            (stock(172), vec![Path::StockJumperFree, Path::Jumper]),
            (stock(122), vec![Path::StockJumperFree, Path::Jumper]),
            (stock(121), vec![Path::Jumper]),
            (isp.clone(), vec![Path::FlashNow]),
            (mini.clone(), vec![]),
        ];
        for (unit, paths) in cases {
            let all = [unit.clone()];
            assert_eq!(offer(&unit, &all).paths, paths, "{}", unit.state);
        }
    }

    #[test]
    fn two_bootloaders_get_nothing() {
        let isp = unit(State::Bootloader);
        let all = [isp.clone(), isp.clone()];
        let offer = offer(&isp, &all);
        assert!(offer.paths.is_empty());
        assert!(offer.notes[0].contains("2 units are in ISP mode"));
    }

    #[test]
    fn a_bootloader_elsewhere_holds_back_jumper_free_entry() {
        for probe in [ours(), stock(172)] {
            let all = [probe.clone(), unit(State::Bootloader)];
            assert_eq!(offer(&probe, &all).paths, [Path::Jumper]);
        }
    }

    #[test]
    fn a_mini_elsewhere_holds_back_jumper_free_entry_too() {
        for probe in [ours(), stock(172)] {
            let all = [probe.clone(), unit(State::MiniBootloader)];
            assert_eq!(offer(&probe, &all).paths, [Path::Jumper]);
        }
    }

    #[test]
    fn a_bootloader_beside_a_mini_gets_nothing() {
        let isp = unit(State::Bootloader);
        let all = [isp.clone(), unit(State::MiniBootloader)];
        let offer = offer(&isp, &all);
        assert!(offer.paths.is_empty());
        assert!(
            offer.notes[0].contains("unplug the mini first"),
            "{:?}",
            offer.notes
        );
    }
}
