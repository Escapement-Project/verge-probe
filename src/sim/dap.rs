//! A simulated probe's CMSIS-DAP v2 interface.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{Event, Mode, SIM_SCAN_FORMATS, SimFirmware, SimUnit, World};
use crate::dap::{self, DapLink, LinkError};

/// Each command costs this much simulated time.
const COMMAND_TIME: Duration = Duration::from_micros(250);

/// The Pro's DIO pins and the groups that change direction together, as the firmware reports them.
const DIO_PINS: u8 = 0x0F;
const DIO_GROUPS: [u8; 2] = [0x07, 0x08];

/// Ours' answer to the DIO command: a Pro sets its pins as a group or not at all, and any other
/// board has none. An input reads what is attached holds it at.
fn dio_reply(u: &mut SimUnit, mask: u8, outputs: u8, levels: u8) -> Vec<u8> {
    let pro = u.board == Some(dap::Ours::BOARD_MCU_LINK_PRO);
    let next = (u.dio_outputs & !mask) | (outputs & mask);
    let accepted = pro
        && mask & !DIO_PINS == 0
        && DIO_GROUPS
            .iter()
            .all(|&group| next & group == 0 || next & group == group);
    if accepted {
        u.dio_outputs = next;
        u.dio_levels = (u.dio_levels & !mask) | (levels & mask);
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
                // A unit with a board answers as the current image does, in format 4; one without
                // as recorded.
                let board = world.units[unit].board;
                let format = if board.is_some() {
                    4
                } else {
                    world.ours.1.format
                };
                let flags = if board == Some(dap::Ours::BOARD_MCU_LINK_PRO) {
                    world.ours.1.flags | dap::Ours::FLAG_DIO
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
                dio_reply(&mut world.units[unit], *mask, *outputs, *levels)
            }
            _ => vec![dap::ID_DAP_INVALID],
        },
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
