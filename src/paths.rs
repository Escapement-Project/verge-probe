//! The update paths a unit's state allows.

use std::fmt;

use crate::detect::{Firmware, Guess, Jumpered, State, Unit};

/// A way to update a unit, as [`offer`] lists them for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Path {
    /// Ours reboots into the bootloader on request, then the tool flashes and resets it.
    OursJumperFree,
    /// Stock V3.122 and later does the same on the request LinkServer sends.
    StockJumperFree,
    /// daplink-rs does the same on its own reboot command.
    DaplinkRsJumperFree,
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
            Path::OursJumperFree
                | Path::StockJumperFree
                | Path::DaplinkRsJumperFree
                | Path::FlashNow
        )
    }

    /// What the path involves, in a sentence or two for a user.
    #[must_use]
    pub fn describe(self) -> &'static str {
        match self {
            Path::OursJumperFree => {
                "The tool asks the firmware to restart into its bootloader, flashes it, and restarts it."
            }
            Path::StockJumperFree => {
                "The tool sends the request NXP's LinkServer uses, then flashes and restarts the unit."
            }
            Path::DaplinkRsJumperFree => {
                "The tool asks daplink-rs to restart into its bootloader, flashes it, and restarts it."
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
            Path::OursJumperFree => "jumper-free (Verge)",
            Path::StockJumperFree => "jumper-free (NXP's firmware)",
            Path::DaplinkRsJumperFree => "jumper-free (daplink-rs)",
            Path::Jumper => "ISP jumper",
            Path::FlashNow => "flash now",
        })
    }
}

/// Where Getting started says how to let a user open the probe on Linux: "USB permissions on
/// Linux".
pub const USB_PERMISSIONS: &str =
    "https://escapement-project.github.io/verge/docs/getting-started/#usb-permissions-on-linux";

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

/// Where an offer with no path names the unit's ports.
fn ports(units: &[&Unit]) -> String {
    let ports: Vec<&str> = units
        .iter()
        .map(|unit| {
            unit.device
                .location
                .as_deref()
                .unwrap_or("a port it does not report")
        })
        .collect();
    match ports.as_slice() {
        [] => String::new(),
        [one] => (*one).to_owned(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

/// Why a running unit waits while `bootloaders` and `minis` are attached: the tool flashes only
/// while one bootloader is, and an update or the jumper would make another.
fn waits_for(bootloaders: &[&Unit], minis: &[&Unit]) -> String {
    match (bootloaders, minis) {
        (_, [_, ..]) => format!(
            "An MCU-Link mini is in ISP mode, at {}. Unplug it, then update this one: the tool works with a bootloader only while no other is attached.",
            ports(minis)
        ),
        ([_], []) => format!(
            "Another unit is in ISP mode, at {}. Flash that one or unplug it, then update this one: the tool flashes only while one bootloader is attached.",
            ports(bootloaders)
        ),
        (_, []) => format!(
            "{} other units are in ISP mode, at {}. Flash them or unplug them, then update this one: the tool flashes only while one bootloader is attached.",
            bootloaders.len(),
            ports(bootloaders)
        ),
    }
}

/// Whether a unit the tool could not ask may have a jumper-free path once it can be asked: what
/// its USB strings suggest it runs has one.
#[must_use]
pub fn asking_again_may_free(guess: &Guess) -> bool {
    matches!(guess, Guess::Ours | Guess::DaplinkRs)
        || matches!(guess, Guess::Stock(Some(v)) if v.has_jumper_free_update())
}

/// The paths `unit` allows, given every attached unit. An offer with no path has a note first
/// that says why.
///
/// Every LPC55S69 ROM enumerates alike, so the tool flashes a bootloader only when it is the only
/// one, and offers a running unit no path while another unit is in ISP mode: either path would
/// put a second one there.
#[must_use]
pub fn offer(unit: &Unit, all: &[Unit]) -> Offer {
    let bootloaders: Vec<&Unit> = all.iter().filter(|other| other.is_bootloader()).collect();
    let minis: Vec<&Unit> = all
        .iter()
        .filter(|other| other.state == State::MiniBootloader)
        .collect();
    let in_isp = bootloaders.len();
    let mut offer = Offer::default();
    let running = !matches!(unit.state, State::Bootloader | State::MiniBootloader);
    if running && in_isp + minis.len() > 0 {
        offer.notes.push(waits_for(&bootloaders, &minis));
        return offer;
    }
    match &unit.state {
        State::Probe {
            firmware: Firmware::Ours(_),
            ..
        } => offer.paths.push(Path::OursJumperFree),
        State::Probe {
            firmware: Firmware::Stock { version },
            ..
        } => match version {
            Some(version) if version.has_jumper_free_update() => {
                offer.paths.push(Path::StockJumperFree);
            }
            Some(version) => offer.notes.push(format!(
                "NXP's firmware V{version} has no jumper-free update. NXP added it in V3.122."
            )),
            None => offer.notes.push(
                "NXP's firmware did not report its version, so the tool will not try the jumper-free update.".into(),
            ),
        },
        State::Probe {
            firmware: Firmware::DaplinkRs { reboot },
            ..
        } => match reboot {
            Some(reboot) if reboot.reaches_isp() => {
                offer.paths.push(Path::DaplinkRsJumperFree);
            }
            Some(reboot) if reboot.kind == crate::daplink_rs::KIND_NONE => offer.notes.push(
                "This daplink-rs build has no bootloader to restart into.".into(),
            ),
            Some(_) => offer.notes.push(
                "This daplink-rs build restarts into a bootloader other than the LPC55S69's ROM, so the tool does not ask it to.".into(),
            ),
            None => offer.notes.push(
                "This daplink-rs build has no command to restart into its bootloader.".into(),
            ),
        },
        // The bootloader would refuse the same user, so no path is open until it may.
        State::Unreadable { reason, .. } if reason == crate::detect::DENIED => {
            offer.notes.push(if cfg!(target_os = "linux") {
                format!(
                    "This user may not open the unit. The udev rules in Getting started let it: {USB_PERMISSIONS}"
                )
            } else {
                "This user may not open the unit.".to_owned()
            });
            return offer;
        }
        State::Unreadable { reason, guess } => {
            offer
                .notes
                .push(format!("The tool could not ask this unit what it runs: {reason}."));
            if asking_again_may_free(guess) {
                offer.notes.push(
                    "Close whatever has it open and look again for the jumper-free path.".into(),
                );
            }
        }
        State::Held { .. } => {
            offer.notes.push(
                "Close the debugger first. Another program holds its CMSIS-DAP interface, and the tool does not update a unit in use.".into(),
            );
            return offer;
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
                offer.paths.push(Path::OursJumperFree);
            }
        }
        State::SwdJumper {
            firmware: Jumpered::Stock(version),
        } => offer.notes.push(match version {
            Some(version) if version.has_jumper_free_update() => {
                "The SWD jumper is fitted. It takes away the CMSIS-DAP interface that NXP's jumper-free update goes through: unplug the unit, take the SWD jumper off and plug it back in to use that update.".into()
            }
            Some(version) => format!(
                "The SWD jumper is fitted. NXP's firmware V{version} has no jumper-free update either way. NXP added it in V3.122."
            ),
            None => "The SWD jumper is fitted, and NXP's product string gives no version, so the tool does not know whether it has a jumper-free update.".into(),
        }),
        State::Bootloader if in_isp > 1 => {
            // Only Linux ties a bootloader to its port (`Bus::tells_bootloaders_apart`).
            let pick = match &unit.device.location {
                Some(port) if cfg!(target_os = "linux") => format!(
                    "`verge isp flash --port {port}` flashes this one, or unplug all but one."
                ),
                _ => "Unplug all but one.".to_owned(),
            };
            offer.notes.push(format!(
                "{in_isp} units are in ISP mode, at {}, and every LPC55S69 bootloader looks the same. {pick}",
                ports(&bootloaders),
            ));
            return offer;
        }
        State::Bootloader if !minis.is_empty() => {
            offer.notes.push(
                "An MCU-Link mini is in ISP mode too, and the tool works with a bootloader only while no other is attached: unplug the mini first."
                    .into(),
            );
            return offer;
        }
        State::Bootloader => {
            offer.paths.push(Path::FlashNow);
            return offer;
        }
        State::MiniBootloader => {
            offer
                .notes
                .push("verge does not work with the MCU-Link mini.".into());
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
                aux: false,
            },
            state,
        }
    }

    fn ours() -> Unit {
        unit(State::Probe {
            identity: Identity::default(),
            firmware: Firmware::Ours(Ours {
                version: 2,
                capabilities: 0,
                firmware: "14d05d4".into(),
                board: None,
                scan_formats: 0,
                extended_capabilities: 0,
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

    /// The jumper would put a second unit in ISP mode as surely as the jumper-free entry, so
    /// neither is offered: the note says what the unit waits for.
    #[test]
    fn a_bootloader_elsewhere_holds_back_every_path() {
        for probe in [ours(), stock(172), stock(108)] {
            let all = [probe.clone(), unit(State::Bootloader)];
            let offer = offer(&probe, &all);
            assert_eq!(offer.paths, [], "{}", probe.state);
            assert!(
                offer.notes[0].starts_with("Another unit is in ISP mode"),
                "{:?}",
                offer.notes
            );
        }
    }

    #[test]
    fn a_mini_elsewhere_holds_back_every_path_too() {
        for probe in [ours(), stock(172), stock(108)] {
            let all = [probe.clone(), unit(State::MiniBootloader)];
            let offer = offer(&probe, &all);
            assert_eq!(offer.paths, [], "{}", probe.state);
            assert!(
                offer.notes[0].starts_with("An MCU-Link mini is in ISP mode"),
                "{:?}",
                offer.notes
            );
        }
    }

    /// A unit a debugger holds is not sent to the jumper: closing the debugger is enough.
    #[test]
    fn a_held_unit_waits_for_its_debugger() {
        let held = unit(State::Held {
            ours: Ours {
                version: 2,
                capabilities: 0,
                firmware: "14d05d4".into(),
                board: None,
                scan_formats: 0,
                extended_capabilities: 0,
            },
        });
        let offer = offer(&held, std::slice::from_ref(&held));
        assert_eq!(offer.paths, []);
        assert!(
            offer.notes[0].starts_with("Close the debugger first."),
            "{:?}",
            offer.notes
        );
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
