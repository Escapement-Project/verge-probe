//! A simulated ROM bootloader, at the layer where NXP's McuBoot client hands over packets.
//!
//! The client frames every packet for UART (`5A`, type, length, CRC16); its USB transport strips
//! that and sends the payload in a HID report. This does the same, so what it sees is what the
//! ROM sees over USB.
//!
//! Assumed rather than measured: the version (`K2.1.0`, the value in UM11126 figure 26's example
//! reply), flash start 0 and size 630 KB, and how long each command takes. UM11126 gives the
//! 56-byte payload and the status codes.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use mboot::protocols::{CommunicationError, Protocol};

use super::{Event, World};
use crate::image::{APP_FLASH_END, PAGE};

const CMD: u8 = 0xA4;
const DATA: u8 = 0xA5;
const GENERIC: u8 = 0xA0;
const READ_MEMORY_RESPONSE: u8 = 0xA3;
const GET_PROPERTY_RESPONSE: u8 = 0xA7;

const FLASH_ERASE_REGION: u8 = 0x02;
const READ_MEMORY: u8 = 0x03;
const WRITE_MEMORY: u8 = 0x04;
const GET_PROPERTY: u8 = 0x07;
const RESET: u8 = 0x0B;

const SUCCESS: u32 = 0;
const FLASH_ALIGNMENT_ERROR: u32 = 101;
const FLASH_ADDRESS_ERROR: u32 = 102;
const FLASH_COMMAND_FAILURE: u32 = 105;
const FLASH_ECC_ERROR: u32 = 116;
const UNKNOWN_COMMAND: u32 = 10000;
const MEMORY_RANGE_INVALID: u32 = 10200;
const MEMORY_CUMULATIVE_WRITE: u32 = 10203;
const UNKNOWN_PROPERTY: u32 = 10300;

/// UM11126 8.11.1.3: a 60-byte report with a 4-byte header.
pub const MAX_PAYLOAD: usize = 56;
pub const VERSION: u32 = u32::from_be_bytes([b'K', 2, 1, 0]);

/// One USB HID report each way.
const PACKET_TIME: Duration = Duration::from_millis(1);
const PAGE_ERASE_TIME: Duration = Duration::from_millis(2);

struct Incoming {
    address: u32,
    remaining: usize,
    data: Vec<u8>,
}

pub struct SimRom {
    world: Arc<Mutex<World>>,
    unit: usize,
    instance: String,
    generation: u64,
    replies: VecDeque<(u8, Vec<u8>)>,
    incoming: Option<Incoming>,
    /// Real time to sleep once the bench is unlocked, when the bench is paced.
    owed: Duration,
}

fn words(tag: u8, flags: u8, params: &[u32]) -> Vec<u8> {
    let mut packet = vec![tag, flags, 0, params.len() as u8];
    for param in params {
        packet.extend_from_slice(&param.to_le_bytes());
    }
    packet
}

fn io_error(text: &str) -> CommunicationError {
    CommunicationError::IOError(std::io::Error::other(text.to_owned()))
}

/// The bench, locked, provided this bootloader is still the one on the bus.
fn lock(
    world: &Mutex<World>,
    unit: usize,
    generation: u64,
) -> Result<MutexGuard<'_, World>, CommunicationError> {
    let mut world = world.lock().expect("the bench lock is not poisoned");
    world.settle();
    if world.is_current(unit, generation) {
        Ok(world)
    } else {
        Err(io_error("the bootloader left the bus"))
    }
}

impl SimRom {
    pub(super) fn new(
        world: Arc<Mutex<World>>,
        unit: usize,
        instance: String,
        generation: u64,
    ) -> SimRom {
        SimRom {
            world,
            unit,
            instance,
            generation,
            replies: VecDeque::new(),
            incoming: None,
            owed: Duration::ZERO,
        }
    }

    fn generic(&mut self, status: u32, tag: u8) {
        self.replies
            .push_back((CMD, words(GENERIC, 0, &[status, u32::from(tag)])));
    }

    fn command(&mut self, payload: &[u8]) -> Result<(), CommunicationError> {
        if payload.len() < 4 {
            return Err(CommunicationError::InvalidHeader);
        }
        let tag = payload[0];
        let count = usize::from(payload[3]);
        let params: Vec<u32> = payload[4..]
            .as_chunks::<4>()
            .0
            .iter()
            .take(count)
            .map(|b| u32::from_le_bytes(*b))
            .collect();
        let param = |i: usize| params.get(i).copied().unwrap_or(0);
        let mut world = lock(&self.world, self.unit, self.generation)?;
        let location = world.units[self.unit].location.clone();
        world
            .log
            .push(format!("{location}: ROM command 0x{tag:02x} {params:x?}"));
        self.owed += world.advance(PACKET_TIME);
        match tag {
            GET_PROPERTY => {
                let value = match param(0) {
                    0x01 => Some(VERSION),
                    0x03 => Some(0),
                    0x04 => Some(APP_FLASH_END),
                    0x0B => Some(MAX_PAYLOAD as u32),
                    _ => None,
                };
                drop(world);
                let reply = match value {
                    Some(value) => words(GET_PROPERTY_RESPONSE, 0, &[SUCCESS, value]),
                    None => words(GET_PROPERTY_RESPONSE, 0, &[UNKNOWN_PROPERTY]),
                };
                self.replies.push_back((CMD, reply));
            }
            FLASH_ERASE_REGION => {
                let (start, len) = (param(0), param(1));
                let status = if start % 4 != 0 || len % 4 != 0 {
                    FLASH_ALIGNMENT_ERROR
                } else if u64::from(start) + u64::from(len) > u64::from(APP_FLASH_END) {
                    FLASH_ADDRESS_ERROR
                } else {
                    let first = start / PAGE * PAGE;
                    let end = (start + len).div_ceil(PAGE) * PAGE;
                    let mut pages = 0;
                    for base in (first..end).step_by(PAGE as usize) {
                        world.units[self.unit].flash.remove(&base);
                        pages += 1;
                    }
                    self.owed += world.advance(PAGE_ERASE_TIME * pages);
                    SUCCESS
                };
                drop(world);
                self.generic(status, tag);
            }
            WRITE_MEMORY => {
                let (start, len) = (param(0), param(1) as usize);
                let end = u64::from(start) + len.div_ceil(PAGE as usize) as u64 * u64::from(PAGE);
                let status = if world.units[self.unit].fail_writes {
                    FLASH_COMMAND_FAILURE
                } else if start % PAGE != 0 {
                    FLASH_ALIGNMENT_ERROR
                } else if end > u64::from(APP_FLASH_END) {
                    MEMORY_RANGE_INVALID
                } else if (start..end as u32)
                    .step_by(PAGE as usize)
                    .any(|base| world.units[self.unit].flash.contains_key(&base))
                {
                    MEMORY_CUMULATIVE_WRITE
                } else {
                    SUCCESS
                };
                drop(world);
                self.generic(status, tag);
                if status == SUCCESS {
                    self.incoming = Some(Incoming {
                        address: start,
                        remaining: len,
                        data: Vec::with_capacity(len),
                    });
                }
            }
            READ_MEMORY => {
                let (start, len) = (param(0), param(1) as usize);
                let first = start / PAGE * PAGE;
                let end =
                    (u64::from(start) + len as u64).div_ceil(u64::from(PAGE)) * u64::from(PAGE);
                let flash = &world.units[self.unit].flash;
                let erased = (u64::from(first)..end)
                    .step_by(PAGE as usize)
                    .any(|base| !flash.contains_key(&(base as u32)));
                if erased {
                    // The firmware's BENCH.md: "FLASH Driver: ECC Error", and the bootloader
                    // drops off the bus.
                    world.log.push(format!(
                        "{location}: ROM read of an erased page: ECC error, the bootloader drops off the bus"
                    ));
                    world.schedule(Duration::from_millis(1), self.unit, Event::Leave);
                    drop(world);
                    self.generic(FLASH_ECC_ERROR, tag);
                    return Ok(());
                }
                let mut bytes = Vec::with_capacity(len);
                for offset in 0..len {
                    let address = start + offset as u32;
                    let page = &flash[&(address / PAGE * PAGE)];
                    bytes.push(page[(address % PAGE) as usize]);
                }
                drop(world);
                self.replies
                    .push_back((CMD, words(READ_MEMORY_RESPONSE, 1, &[SUCCESS, len as u32])));
                for packet in bytes.chunks(MAX_PAYLOAD) {
                    self.replies.push_back((DATA, packet.to_vec()));
                }
                self.generic(SUCCESS, tag);
            }
            RESET => {
                if !world.units[self.unit].ignores_reset {
                    let boot = world.boot_after_reset;
                    world.schedule(Duration::from_millis(10), self.unit, Event::Leave);
                    world.schedule(boot, self.unit, Event::Boot);
                }
                drop(world);
                self.generic(SUCCESS, tag);
            }
            _ => {
                drop(world);
                self.generic(UNKNOWN_COMMAND, tag);
            }
        }
        Ok(())
    }

    fn data(&mut self, payload: &[u8]) -> Result<(), CommunicationError> {
        if payload.len() > MAX_PAYLOAD {
            return Err(io_error("a data packet larger than a HID report"));
        }
        let Some(incoming) = self.incoming.as_mut() else {
            return Err(CommunicationError::InvalidPacketReceived);
        };
        let take = payload.len().min(incoming.remaining);
        incoming.data.extend_from_slice(&payload[..take]);
        incoming.remaining -= take;
        let mut world = lock(&self.world, self.unit, self.generation)?;
        self.owed += world.advance(PACKET_TIME);
        if self.incoming.as_ref().is_some_and(|i| i.remaining == 0) {
            let Incoming { address, data, .. } = self.incoming.take().expect("checked above");
            for (index, chunk) in data.chunks(PAGE as usize).enumerate() {
                let mut page = vec![0xFF; PAGE as usize];
                page[..chunk.len()].copy_from_slice(chunk);
                world.units[self.unit]
                    .flash
                    .insert(address + index as u32 * PAGE, page);
            }
            drop(world);
            self.generic(SUCCESS, WRITE_MEMORY);
        }
        Ok(())
    }
}

impl Protocol for SimRom {
    fn get_timeout(&self) -> Duration {
        Duration::from_secs(5)
    }

    fn get_polling_interval(&self) -> Duration {
        Duration::from_millis(1)
    }

    fn get_identifier(&self) -> &str {
        &self.instance
    }

    fn read(&mut self, _bytes: usize) -> Result<Vec<u8>, CommunicationError> {
        Err(CommunicationError::UnsupportedPlatform)
    }

    fn write_packet_raw(&mut self, data: &[u8]) -> Result<(), CommunicationError> {
        if data.len() < 6 || data[0] != 0x5A {
            return Err(CommunicationError::InvalidHeader);
        }
        let len = usize::from(u16::from_le_bytes([data[2], data[3]]));
        let payload = data
            .get(6..6 + len)
            .ok_or(CommunicationError::InvalidData)?;
        let result = match data[1] {
            CMD => self.command(payload),
            DATA => self.data(payload),
            _ => Err(CommunicationError::InvalidHeader),
        };
        let owed = std::mem::take(&mut self.owed);
        if !owed.is_zero() {
            std::thread::sleep(owed);
        }
        result
    }

    fn read_packet_raw(&mut self, packet_code: u8) -> Result<Vec<u8>, CommunicationError> {
        drop(lock(&self.world, self.unit, self.generation)?);
        match self.replies.pop_front() {
            Some((code, payload)) if code == packet_code => Ok(payload),
            Some(_) => Err(CommunicationError::InvalidPacketReceived),
            None => Err(CommunicationError::Timeout),
        }
    }
}
