//! Opening a selected Pro, or a standalone MCU-Link for its DIO pins, without competing with its
//! debugger.

use crate::aux::{Aux, AuxError, AuxLink};
use crate::bus::BusError;
use crate::dap::LinkError;
use crate::stream::Message;
use crate::{Bus, CommandError, DapTransport, Unit, VergeControl, VergeInfo, VergeTransport};
use std::convert::Infallible;
use std::time::Duration;

/// The way to a Pro, and its energy stream where it has one: only Verge Aux does.
pub trait ProLink: VergeTransport {
    fn streams(&self) -> bool;
    /// The next stream message, waiting up to `timeout`; `None` if none came.
    fn message(&mut self, timeout: Duration) -> Result<Option<Message>, AuxError>;
}

impl<L: AuxLink> ProLink for Aux<L> {
    fn streams(&self) -> bool {
        true
    }
    fn message(&mut self, timeout: Duration) -> Result<Option<Message>, AuxError> {
        Aux::message(self, timeout)
    }
}

impl ProLink for DapTransport<'_> {
    fn streams(&self) -> bool {
        false
    }
    fn message(&mut self, _: Duration) -> Result<Option<Message>, AuxError> {
        Ok(None)
    }
}

/// What a unit running Verge offers that a frontend reaches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Feature {
    Supply,
    Energy,
    Dio,
}

impl Feature {
    pub fn name(self) -> &'static str {
        match self {
            Self::Supply => "target supply",
            Self::Energy => "energy measurement",
            Self::Dio => "DIO pins",
        }
    }
    /// Whether `board` has it: DIO pins on the Pro and the standalone MCU-Link, the rest on the
    /// Pro alone.
    #[must_use]
    pub fn on(self, board: crate::BoardKind) -> bool {
        matches!(
            (self, board),
            (_, crate::BoardKind::Pro) | (Self::Dio, crate::BoardKind::Plain)
        )
    }
    fn offered(self, info: &VergeInfo) -> bool {
        match self {
            Self::Supply => info.supports_target_power(),
            Self::Energy => info.supports_energy_capture(),
            Self::Dio => info.supports_dio(),
        }
    }
}

impl std::fmt::Display for Feature {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// Why a unit cannot give a frontend a Pro feature. Each message names the unit.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum ProError {
    #[error("the tool could not ask {name} what it runs: {reason}")]
    Unreadable { name: String, reason: String },
    #[error("{name} is {state}, not running Verge, so the tool cannot reach its {feature}")]
    NotVerge {
        name: String,
        state: String,
        feature: Feature,
    },
    #[error("{name} is not an MCU-Link Pro, so it has no {feature}")]
    NotPro { name: String, feature: Feature },
    #[error("{name} has no DIO pins: only an MCU-Link Pro and a standalone MCU-Link have them")]
    NoDio { name: String },
    #[error(
        "{name} reports no energy measurement: its HW_VER_5 strap says the circuit is not fitted"
    )]
    NotFitted { name: String },
    #[error("{name} runs build {firmware}, which predates its {feature}: update it first")]
    Predates {
        name: String,
        firmware: String,
        feature: Feature,
    },
    #[error("{name} has no Verge Aux: update the firmware first")]
    NoAux { name: String },
    #[error("{name} has DIO disabled: remove J15 and restart the probe")]
    DioDisabled { name: String },
    #[error("{name} runs build {firmware}, which predates DIO on Verge Aux: update it first")]
    NoAuxDio { name: String, firmware: String },
    #[error("{name} no longer answers as Verge")]
    Gone { name: String },
    #[error("asking {name} what it runs")]
    Info {
        name: String,
        #[source]
        source: CommandError<Infallible>,
    },
    #[error("opening {name}'s Verge Aux")]
    Aux {
        name: String,
        #[source]
        source: LinkError,
    },
    #[error("opening {name}'s {interface}")]
    Open {
        name: String,
        interface: &'static str,
        #[source]
        source: BusError,
    },
}

fn name(unit: &Unit) -> String {
    unit.serial().unwrap_or("the unit").to_owned()
}

/// Refuses, in a sentence that says why, a unit running Verge on a board without `feature`, or
/// not running Verge.
pub fn validate(unit: &Unit, feature: Feature) -> Result<(), ProError> {
    let name = name(unit);
    if let crate::detect::State::Unreadable { reason, .. } = &unit.state {
        return Err(ProError::Unreadable {
            name,
            reason: reason.clone(),
        });
    }
    let Some(ours) = unit.verge() else {
        return Err(ProError::NotVerge {
            name,
            state: unit.state.to_string(),
            feature,
        });
    };
    match ours.board {
        Some(board) if !feature.on(board) => Err(match feature {
            Feature::Dio => ProError::NoDio { name },
            _ => ProError::NotPro { name, feature },
        }),
        _ => Ok(()),
    }
}

/// Refuses a Pro whose firmware does not offer `feature`.
pub fn check_offered<T: VergeTransport + ?Sized>(
    name: &str,
    pro: &mut VergeControl<'_, T>,
    feature: Feature,
) -> Result<(), ProError> {
    let aux = pro.transport().is_aux();
    let info = pro
        .info()
        .map_err(|source| ProError::Info {
            name: name.to_owned(),
            source,
        })?
        .ok_or_else(|| ProError::Gone {
            name: name.to_owned(),
        })?;
    if feature == Feature::Dio {
        // Only the Pro has a jumper that turns them off; a standalone MCU-Link without them runs a
        // build from before they were its.
        if info.board == Some(crate::BoardKind::Pro) && info.version >= 6 && !info.supports_dio() {
            return Err(ProError::DioDisabled {
                name: name.to_owned(),
            });
        }
        if aux && !info.supports_dio_on_aux() {
            return Err(ProError::NoAuxDio {
                name: name.to_owned(),
                firmware: info.firmware.clone(),
            });
        }
    }
    if feature.offered(info) {
        return Ok(());
    }
    // Firmware that reports extended capabilities (format 6) offers energy capture on every Pro
    // whose HW_VER_5 strap is low (mculink-verge BOARDS.md).
    Err(if feature == Feature::Energy && info.version >= 6 {
        ProError::NotFitted {
            name: name.to_owned(),
        }
    } else {
        ProError::Predates {
            name: name.to_owned(),
            firmware: info.firmware.clone(),
            feature,
        }
    })
}

pub type Pro<'a> = VergeControl<'a, dyn ProLink + 'a>;

/// Runs `f` on `unit` through Verge Aux, or through CMSIS-DAP when another program holds Aux.
pub fn with_pro<B: Bus, T, E: From<ProError>>(
    bus: &B,
    unit: &Unit,
    feature: Feature,
    f: impl FnOnce(&str, &mut Pro<'_>) -> Result<T, E>,
) -> Result<T, E> {
    validate(unit, feature)?;
    let name = name(unit);
    if unit.device.aux {
        match bus.open_aux(&unit.device) {
            Ok(link) => {
                let mut aux = Aux::open(link).map_err(|source| ProError::Aux {
                    name: name.clone(),
                    source,
                })?;
                let mut pro = VergeControl::new(&mut aux as &mut dyn ProLink);
                check_offered(&name, &mut pro, feature)?;
                return f(&name, &mut pro);
            }
            Err(BusError::Busy { .. }) => tracing::info!(
                "another program holds {name}'s Verge Aux, so the tool uses CMSIS-DAP"
            ),
            Err(source) => {
                return Err(ProError::Open {
                    name,
                    interface: "Verge Aux",
                    source,
                }
                .into());
            }
        }
    }
    let mut link = bus
        .open_dap(&unit.device)
        .map_err(|source| ProError::Open {
            name: name.clone(),
            interface: "CMSIS-DAP interface",
            source,
        })?;
    let mut transport = DapTransport(&mut link);
    let mut pro = VergeControl::new(&mut transport as &mut dyn ProLink);
    check_offered(&name, &mut pro, feature)?;
    f(&name, &mut pro)
}

/// Verge Aux on `unit`, checked to offer energy measurement: the only way to its stream.
pub fn open_stream<B: Bus>(bus: &B, unit: &Unit) -> Result<Aux<B::Aux>, ProError> {
    open_aux(bus, unit, Feature::Energy)
}

/// Verge Aux on `unit`, checked to offer `feature`. A frontend that holds it leaves CMSIS-DAP to a
/// debugger.
pub fn open_aux<B: Bus>(bus: &B, unit: &Unit, feature: Feature) -> Result<Aux<B::Aux>, ProError> {
    validate(unit, feature)?;
    let name = name(unit);
    if !unit.device.aux {
        return Err(ProError::NoAux { name });
    }
    let link = bus
        .open_aux(&unit.device)
        .map_err(|source| ProError::Open {
            name: name.clone(),
            interface: "Verge Aux",
            source,
        })?;
    let mut aux = Aux::open(link).map_err(|source| ProError::Aux {
        name: name.clone(),
        source,
    })?;
    check_offered(&name, &mut VergeControl::new(&mut aux), feature)?;
    Ok(aux)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dap::Ours;
    use crate::sim::SimBus;

    fn unit(board: u8) -> Unit {
        let bus = SimBus::default();
        let unit = bus.add_ours("26ea505");
        bus.set_board(unit, board);
        crate::detect::detect(&bus).unwrap().remove(0)
    }

    /// DIO pins on the Pro and the standalone MCU-Link, the supply and energy on the Pro alone, and
    /// each refusal saying which.
    #[test]
    fn each_board_offers_its_own_features() {
        for feature in [Feature::Dio, Feature::Supply, Feature::Energy] {
            assert!(
                validate(&unit(Ours::BOARD_MCU_LINK_PRO), feature).is_ok(),
                "{feature}"
            );
        }
        let plain = unit(Ours::BOARD_MCU_LINK);
        assert!(validate(&plain, Feature::Dio).is_ok());
        for feature in [Feature::Supply, Feature::Energy] {
            assert!(
                matches!(validate(&plain, feature), Err(ProError::NotPro { .. })),
                "{feature}"
            );
        }
        assert!(matches!(
            validate(&unit(Ours::BOARD_ON_BOARD), Feature::Dio),
            Err(ProError::NoDio { .. })
        ));
    }
}
