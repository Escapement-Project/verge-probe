//! A simulated probe's CMSIS-DAP v2 interface.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{Event, Mode, SIM_SCAN_FORMATS, SimFirmware, SimUnit, World};
use crate::dap::{self, DapLink, LinkError};
use crate::daplink_rs;
use crate::protocol::{
    CAP_CALIBRATED_ENERGY, CAP_CAPTURE_ID, CAP_DIO_AUX, CAP_ENERGY_ACCUMULATOR, CAP_ENERGY_CAPTURE,
    CAP_MARKS, CAP_TARGET_POWER, ENERGY_COMMAND, MARK_COMMAND, TARGET_POWER_COMMAND,
};

/// Each command costs this much simulated time.
const COMMAND_TIME: Duration = Duration::from_micros(250);
/// How long a write waits for an OUT endpoint the probe keeps shut: usb.rs's command timeout.
const AUX_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// The Pro's DIO pins and the groups that change direction together, as the firmware reports them.
const DIO_PINS: u8 = 0x0F;
const DIO_GROUPS: [u8; 2] = [0x07, 0x08];

/// Ours' answer to the DIO command: a Pro sets its pins as a group or not at all, and any other
/// board has none. An input reads what is attached holds it at.
fn dio_reply(u: &mut SimUnit, mask: u8, outputs: u8, levels: u8, now: Duration) -> Vec<u8> {
    let pro = u.board == Some(dap::Ours::BOARD_MCU_LINK_PRO) && !u.dio_disabled;
    let next = (u.dio_outputs & !mask) | (outputs & mask);
    let accepted = pro
        && mask & !DIO_PINS == 0
        && DIO_GROUPS
            .iter()
            .all(|&group| next & group == 0 || next & group == group);
    if accepted {
        u.dio_outputs = next;
        u.dio_levels = (u.dio_levels & !mask) | (levels & mask);
        u.energy
            .set_dio(u.dio_outputs, u.dio_levels, u.dio_held, now);
    }
    let mut reply = vec![
        dap::VENDOR_DIO,
        if accepted { dap::DAP_OK } else { 0xFF },
        u.dio_outputs,
        u.dio_levels,
        (u.dio_levels & u.dio_outputs) | (u.dio_held & !u.dio_outputs),
    ];
    if pro {
        reply.extend_from_slice(&[DIO_PINS, DIO_GROUPS.len() as u8]);
        reply.extend_from_slice(&DIO_GROUPS);
    } else {
        reply.extend_from_slice(&[0, 0]);
    }
    reply
}

pub struct SimDap {
    world: Arc<Mutex<World>>,
    unit: usize,
    generation: u64,
    /// Replies on the bulk IN endpoint, oldest first. An earlier host's leftovers come first.
    pending: VecDeque<Vec<u8>>,
}

impl SimDap {
    pub(super) fn new(
        world: Arc<Mutex<World>>,
        unit: usize,
        generation: u64,
        stale: Vec<Vec<u8>>,
    ) -> SimDap {
        SimDap {
            world,
            unit,
            generation,
            pending: stale.into(),
        }
    }
}

fn string_reply(text: &str) -> Vec<u8> {
    if text.is_empty() {
        return vec![dap::DAP_INFO, 0];
    }
    // The firmware's length counts the terminating NUL (`sizeof` of the string in DAP_config.h).
    let mut reply = vec![dap::DAP_INFO, (text.len() + 1) as u8];
    reply.extend_from_slice(text.as_bytes());
    reply.push(0);
    reply
}

fn answer(world: &mut World, unit: usize, request: &[u8]) -> Vec<u8> {
    let u = &world.units[unit];
    let Mode::Firmware(firmware) = &u.mode else {
        return vec![dap::ID_DAP_INVALID];
    };
    let firmware = firmware.clone();
    let (info, serial, version) = match &firmware {
        SimFirmware::Ours { build } => (world.ours.0.clone(), u.ours_serial.clone(), build.clone()),
        SimFirmware::Stock { version } | SimFirmware::StockHid { version } => {
            let mut info = world.stock.clone();
            info.product = format!("MCU-Link CMSIS-DAP V{version}");
            (info, u.stock_serial.clone(), version.to_string())
        }
        // As dap-rs answers: no vendor, product, serial or firmware version, and the revision in
        // the protocol version.
        SimFirmware::DaplinkRs { revision } => (
            super::RecordedInfo {
                vendor: String::new(),
                product: String::new(),
                protocol: format!("2.1.1.g{revision}"),
                packet_count: 8,
                packet_size: 512,
            },
            String::new(),
            String::new(),
        ),
        // The bus lists no device on an ID the tool does not know, so nothing opens one.
        SimFirmware::OtherId { .. } => return vec![dap::ID_DAP_INVALID],
    };
    match request {
        [dap::DAP_INFO, id] => match *id {
            0x01 => string_reply(&info.vendor),
            0x02 => string_reply(&info.product),
            0x03 => string_reply(&serial),
            0x04 => string_reply(&info.protocol),
            0x09 => string_reply(&version),
            0xFE => vec![dap::DAP_INFO, 1, info.packet_count],
            0xFF => {
                let [lo, hi] = info.packet_size.to_le_bytes();
                vec![dap::DAP_INFO, 2, lo, hi]
            }
            _ => vec![dap::DAP_INFO, 0],
        },
        [dap::VENDOR_IDENTIFY] => match &firmware {
            SimFirmware::Ours { build } => {
                // A unit with a board answers as the current image does, in format 6; one without
                // as recorded.
                let board = world.units[unit].board;
                let format = if board.is_some() {
                    6
                } else {
                    world.ours.1.format
                };
                let pro = board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
                let flags = if pro {
                    world.ours.1.flags
                        | CAP_TARGET_POWER
                        | if world.units[unit].dio_disabled {
                            0
                        } else {
                            dap::Ours::FLAG_DIO
                        }
                } else {
                    world.ours.1.flags
                };
                let mut reply = vec![dap::VENDOR_IDENTIFY];
                reply.extend_from_slice(dap::IDENTIFY_MAGIC);
                reply.extend_from_slice(&[format, flags, build.len() as u8]);
                reply.extend_from_slice(build.as_bytes());
                if let Some(board) = board {
                    reply.push(board);
                    reply.extend_from_slice(&world.units[unit].scan_formats.to_le_bytes());
                    let extended = if pro {
                        CAP_ENERGY_CAPTURE
                            | CAP_ENERGY_ACCUMULATOR
                            | CAP_CALIBRATED_ENERGY
                            | if world.units[unit].dio_aux && !world.units[unit].dio_disabled {
                                CAP_DIO_AUX
                            } else {
                                0
                            }
                            | if world.units[unit].marks {
                                CAP_MARKS | CAP_CAPTURE_ID
                            } else {
                                0
                            }
                    } else {
                        0
                    };
                    reply.extend_from_slice(&extended.to_le_bytes());
                }
                reply
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [0x81, 0x01] => match &firmware {
            SimFirmware::Stock { version } if version.has_jumper_free_update() => {
                world.schedule(Duration::from_millis(2100), unit, Event::Leave);
                world.schedule(
                    Duration::from_millis(2400),
                    unit,
                    Event::Appear(Mode::Bootloader),
                );
                dap::STOCK_BOOTLOADER_ACCEPTED.to_vec()
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [dap::VENDOR_CJTAG_CONTROL, dap::CJTAG_SET_FORMATS, mask @ ..] if mask.len() == 4 => {
            match &firmware {
                SimFirmware::Ours { .. } => {
                    let mask = u32::from_le_bytes(mask.try_into().expect("four bytes"));
                    world.units[unit].scan_formats =
                        (mask & SIM_SCAN_FORMATS) | (1 << dap::SCNFMT_OSCAN1);
                    vec![dap::VENDOR_CJTAG_CONTROL, dap::DAP_OK]
                }
                _ => vec![dap::ID_DAP_INVALID],
            }
        }
        // A unit without a board answers as the recorded image, which predates the command.
        [dap::VENDOR_DIO, mask, outputs, levels] => match &firmware {
            SimFirmware::Ours { .. } if world.units[unit].board.is_some() => {
                dio_reply(&mut world.units[unit], *mask, *outputs, *levels, world.now)
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [TARGET_POWER_COMMAND, mode] => match &firmware {
            SimFirmware::Ours { .. } if world.units[unit].board.is_some() => {
                let pro = world.units[unit].board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
                let now = world.now;
                world.units[unit].energy.target_power(pro, *mode, now)
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [ENERGY_COMMAND, mode, ..] if *mode >= 8 && !world.units[unit].marks => {
            vec![ENERGY_COMMAND, 0xff]
        }
        [ENERGY_COMMAND, request @ ..] => match &firmware {
            SimFirmware::Ours { .. } if world.units[unit].board.is_some() => {
                let pro = world.units[unit].board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
                let now = world.now;
                world.units[unit].energy.energy(pro, now, request)
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [MARK_COMMAND, request @ ..] => match &firmware {
            SimFirmware::Ours { .. }
                if world.units[unit].board.is_some() && world.units[unit].marks =>
            {
                let pro = world.units[unit].board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
                let now = world.now;
                world.units[unit].energy.mark(pro, now, request)
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
        [daplink_rs::REBOOT_COMMAND, request @ ..]
            if matches!(firmware, SimFirmware::DaplinkRs { .. }) =>
        {
            match request {
                [0x00, ..] => b"\x9fdlrs\x01\x03".to_vec(),
                [0x01, b'B', b'O', b'O', b'T', ..] => {
                    // It answers, then enters the ROM from its idle loop 50 ms later.
                    world.schedule(Duration::from_millis(50), unit, Event::Leave);
                    world.schedule(
                        Duration::from_millis(550),
                        unit,
                        Event::Appear(Mode::Bootloader),
                    );
                    vec![daplink_rs::REBOOT_COMMAND, 0x00]
                }
                [0x01, ..] => vec![daplink_rs::REBOOT_COMMAND, 0x02],
                _ => vec![dap::ID_DAP_INVALID],
            }
        }
        [dap::DAP_DISCONNECT] => vec![dap::DAP_DISCONNECT, dap::DAP_OK],
        [dap::DAP_HOST_STATUS, 0 | 1, on @ (0 | 1)] => {
            world.units[unit].host_status += 1;
            world.units[unit].leds_on = *on == 1;
            vec![dap::DAP_HOST_STATUS, dap::DAP_OK]
        }
        _ => vec![dap::ID_DAP_INVALID],
    }
}

impl SimDap {
    fn live(&self, world: &mut World) -> Result<(), LinkError> {
        world.settle();
        if world.is_current(self.unit, self.generation) {
            Ok(())
        } else {
            Err(LinkError::Disconnected)
        }
    }
}

impl DapLink for SimDap {
    fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError> {
        let mut world = self.world.lock().expect("the bench lock is not poisoned");
        self.live(&mut world)?;
        world.advance(COMMAND_TIME);
        let location = world.units[self.unit].location.clone();
        world
            .log
            .push(format!("{location}: DAP {}", hex::encode(request)));
        let reply = answer(&mut world, self.unit, request);
        self.pending.push_back(reply);
        Ok(self.pending.pop_front().expect("a reply was just queued"))
    }

    fn drain(&mut self) -> Result<usize, LinkError> {
        let mut world = self.world.lock().expect("the bench lock is not poisoned");
        self.live(&mut world)?;
        let count = self.pending.len();
        self.pending.clear();
        Ok(count)
    }

    fn control_out(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<(), LinkError> {
        let mut world = self.world.lock().expect("the bench lock is not poisoned");
        self.live(&mut world)?;
        world.control_out(self.unit, request_type, request, value, index)
    }
}

/// A simulated Pro's Verge Aux interface: `0x82`, `0x8E`, `0x8F`, `0x90` and `0x94` as on CMSIS-DAP, `0x90` modes 5
/// to 7 besides, and the energy stream.
pub struct SimAux {
    world: Arc<Mutex<World>>,
    unit: usize,
    generation: u64,
}

/// While the host waits on IN, simulated time passes in steps of this, and the stream with it.
const WAIT_STEP: Duration = Duration::from_millis(10);

/// The pairs messages the probe keeps for a host that has not read them: 16 places, two kept for
/// a header and an end. It drops new pairs messages while they are full (HOST-INTERFACE.md, "When
/// the host falls behind").
const KEPT_PAIRS_MESSAGES: usize = 14;

/// Puts replies and stream messages on a unit's IN endpoint, dropping pairs messages the probe
/// would have no room for.
fn queue(aux_in: &mut VecDeque<Vec<u8>>, messages: Vec<Vec<u8>>) {
    for message in messages {
        let pairs = |m: &Vec<u8>| m.first() == Some(&crate::stream::PAIRS);
        if pairs(&message) && aux_in.iter().filter(|m| pairs(m)).count() >= KEPT_PAIRS_MESSAGES {
            continue;
        }
        aux_in.push_back(message);
    }
}

impl SimAux {
    pub(super) fn new(world: Arc<Mutex<World>>, unit: usize, generation: u64) -> SimAux {
        SimAux {
            world,
            unit,
            generation,
        }
    }

    fn live(&self, world: &mut World) -> Result<(), LinkError> {
        world.settle();
        if world.is_current(self.unit, self.generation) {
            Ok(())
        } else {
            Err(LinkError::Disconnected)
        }
    }
}

impl crate::aux::AuxLink for SimAux {
    fn write(&mut self, request: &[u8]) -> Result<(), LinkError> {
        let mut world = self.world.lock().expect("the bench lock is not poisoned");
        self.live(&mut world)?;
        if world.units[self.unit].aux_out_shut {
            world.advance(AUX_WRITE_TIMEOUT);
            return Err(LinkError::Timeout);
        }
        world.advance(COMMAND_TIME);
        let location = world.units[self.unit].location.clone();
        world
            .log
            .push(format!("{location}: AUX {}", hex::encode(request)));
        let now = world.now;
        let unit = &mut world.units[self.unit];
        let pro = unit.board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
        unit.energy.dio_outputs = unit.dio_outputs;
        unit.energy.dio_levels = unit.dio_levels;
        unit.energy.dio_held = unit.dio_held;
        let messages = match request {
            [ENERGY_COMMAND, mode, ..] if *mode >= 8 && !unit.marks => {
                vec![vec![ENERGY_COMMAND, 0xff]]
            }
            [ENERGY_COMMAND, 9, rate @ .., pins] if rate.len() == 4 => unit
                .energy
                .identified_stream(pro, now, Some(rate.try_into().expect("four bytes")), *pins),
            [ENERGY_COMMAND, 10, pins] => unit.energy.identified_stream(pro, now, None, *pins),
            [ENERGY_COMMAND, 13, bytes @ ..] => unit.energy.stop_stream_identified(pro, now, bytes),
            [ENERGY_COMMAND, 5, rate @ .., pins] if rate.len() == 4 => {
                unit.energy
                    .start_stream(pro, now, rate.try_into().expect("four bytes"), *pins)
            }
            [ENERGY_COMMAND, 6, pins] => unit.energy.stream_running(pro, now, *pins),
            [ENERGY_COMMAND, 7] => unit.energy.stop_stream(pro, now),
            [dap::VENDOR_DIO, ..] if !unit.dio_aux => vec![vec![dap::ID_DAP_INVALID]],
            [dap::VENDOR_DIO, ..]
            | [dap::VENDOR_IDENTIFY, ..]
            | [TARGET_POWER_COMMAND, ..]
            | [ENERGY_COMMAND, ..]
            | [MARK_COMMAND, ..] => {
                let mut messages = unit.energy.stream_messages(now);
                messages.push(answer(&mut world, self.unit, request));
                messages
            }
            _ => vec![vec![dap::ID_DAP_INVALID]],
        };
        if matches!(request.get(..2), Some([ENERGY_COMMAND, 9 | 10]))
            && messages.first().and_then(|reply| reply.get(2)) == Some(&0)
        {
            world.units[self.unit].aux_in.clear();
        }
        // The probe takes the next request only once this reply is on its way, and the reply waits
        // for whatever is ahead of it on IN to be read.
        let ahead = world.units[self.unit].aux_in.len()
            + messages
                .iter()
                .take_while(|message| crate::stream::is_stream_message(message))
                .count();
        world.units[self.unit].aux_out_shut = ahead > 0;
        queue(&mut world.units[self.unit].aux_in, messages);
        Ok(())
    }

    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError> {
        let mut waited = Duration::ZERO;
        loop {
            let real = {
                let mut world = self.world.lock().expect("the bench lock is not poisoned");
                self.live(&mut world)?;
                let now = world.now;
                let unit = &mut world.units[self.unit];
                let due = unit.energy.stream_messages(now);
                queue(&mut unit.aux_in, due);
                if let Some(message) = unit.aux_in.pop_front() {
                    unit.aux_out_shut = false;
                    return Ok(message);
                }
                // Without a stream nothing will come, and waiting for it would move every
                // scenario's capture on.
                if waited >= timeout || world.units[self.unit].energy.stream.is_none() {
                    return Err(LinkError::Timeout);
                }
                let step = WAIT_STEP.min(timeout - waited);
                waited += step;
                world.advance(step)
            };
            if !real.is_zero() {
                std::thread::sleep(real);
            }
        }
    }
}
