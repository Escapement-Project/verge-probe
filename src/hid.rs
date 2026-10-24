//! The ROM bootloader's USB HID transport, for NXP's McuBoot client, over a HID device the tool
//! opened itself. The client's own USB transport opens the first device with the bootloader's
//! VID:PID, which need not be the unit the tool checked.
//!
//! UM11126 8.11.1.3: each packet travels in one report, `report id, 0, length (LE16), packet`.
//! Reports 1 and 2 carry commands and data to the ROM, 3 and 4 back from it. A data report of
//! length 0 aborts the data phase. The client frames packets for UART (`5A`, type, length,
//! CRC16); over USB the frame is dropped and the report header stands in for it.

use std::path::Path;
use std::time::Duration;

use mboot::protocols::{CommunicationError, Protocol};

const FRAME_START: u8 = 0x5A;
const COMMAND: u8 = 0xA4;
const DATA: u8 = 0xA5;

const COMMAND_OUT: u8 = 1;
const DATA_OUT: u8 = 2;
const COMMAND_IN: u8 = 3;
const DATA_IN: u8 = 4;

/// Larger than any report the ROM sends, so a read never truncates one.
const READ_BUFFER: usize = 256;

/// A command packet's header: tag, flags, a reserved byte, and the number of parameters.
const HEADER: usize = 4;
/// The flag that says a data phase follows, with its length the second parameter.
const HAS_DATA_PHASE: u8 = 1;
const GENERIC_RESPONSE: u8 = 0xA0;
const READ_MEMORY_RESPONSE: u8 = 0xA3;
const GET_PROPERTY_RESPONSE: u8 = 0xA7;
/// Response tags McuBoot's client knows but has no parser for: it panics on them.
const UNPARSED_RESPONSES: [u8; 2] = [0xB0, 0xB3];

/// Refuses a command packet McuBoot's client (rblhost 0.2.0) would panic on rather than reject:
/// one that is not its header and its parameters exactly, or has no status; a data phase, or a
/// generic response, without the second parameter it reads; a successful GetProperty response
/// with no value; a ReadMemory response with no data phase, which is how the ROM aborts one
/// (UM11126 8.4); or a tag it has no parser for.
fn check_command(packet: &[u8]) -> Result<(), CommunicationError> {
    let [tag, flags, _, count, ..] = *packet else {
        return Err(CommunicationError::InvalidData);
    };
    let count = usize::from(count);
    let success = packet.get(4..8) == Some([0; 4].as_slice());
    let refused = packet.len() != HEADER + 4 * count
        || count < 1
        || (flags & HAS_DATA_PHASE != 0 && count < 2)
        || (tag == GENERIC_RESPONSE && count < 2)
        || (tag == GET_PROPERTY_RESPONSE && success && count < 2)
        || (tag == READ_MEMORY_RESPONSE && flags & HAS_DATA_PHASE == 0)
        || UNPARSED_RESPONSES.contains(&tag);
    if refused {
        Err(CommunicationError::InvalidData)
    } else {
        Ok(())
    }
}

/// The two calls the transport makes on a HID device: hidapi's, or a test's.
pub trait HidPipe {
    /// Writes one report, its id first. Returns the bytes written.
    fn write(&self, report: &[u8]) -> Result<usize, String>;
    /// Reads one report, its id first, waiting up to `timeout_ms`. Returns 0 on a timeout.
    fn read_timeout(&self, buffer: &mut [u8], timeout_ms: i32) -> Result<usize, String>;
}

impl HidPipe for hidapi::HidDevice {
    fn write(&self, report: &[u8]) -> Result<usize, String> {
        hidapi::HidDevice::write(self, report).map_err(|error| error.to_string())
    }

    fn read_timeout(&self, buffer: &mut [u8], timeout_ms: i32) -> Result<usize, String> {
        hidapi::HidDevice::read_timeout(self, buffer, timeout_ms).map_err(|error| error.to_string())
    }
}

/// One ROM bootloader, as McuBoot's client sees it.
pub struct HidRom<P = hidapi::HidDevice> {
    pipe: P,
    name: String,
    timeout: Duration,
}

impl<P: HidPipe> HidRom<P> {
    /// `name` identifies the device in the client's messages; `timeout` bounds each reply.
    pub fn new(pipe: P, name: String, timeout: Duration) -> HidRom<P> {
        HidRom {
            pipe,
            name,
            timeout,
        }
    }
}

fn io_error(text: String) -> CommunicationError {
    CommunicationError::IOError(std::io::Error::other(text))
}

impl<P: HidPipe> Protocol for HidRom<P> {
    fn get_timeout(&self) -> Duration {
        self.timeout
    }

    fn get_polling_interval(&self) -> Duration {
        Duration::from_millis(1)
    }

    fn get_identifier(&self) -> &str {
        &self.name
    }

    /// Only the client's UART transport reads raw bytes.
    fn read(&mut self, _bytes: usize) -> Result<Vec<u8>, CommunicationError> {
        Err(CommunicationError::UnsupportedPlatform)
    }

    fn write_packet_raw(&mut self, frame: &[u8]) -> Result<(), CommunicationError> {
        let [FRAME_START, kind, lo, hi, _, _, rest @ ..] = frame else {
            return Err(CommunicationError::InvalidHeader);
        };
        let id = match *kind {
            COMMAND => COMMAND_OUT,
            DATA => DATA_OUT,
            _ => return Err(CommunicationError::InvalidHeader),
        };
        let packet = rest
            .get(..usize::from(u16::from_le_bytes([*lo, *hi])))
            .ok_or(CommunicationError::InvalidData)?;
        let mut report = vec![id, 0, *lo, *hi];
        report.extend_from_slice(packet);
        tracing::debug!(target: "rom", "{}: out {}", self.name, hex::encode(&report));
        let written = self.pipe.write(&report).map_err(io_error)?;
        // Windows pads a report to the length the device declares, and counts the padding.
        if written < report.len() {
            return Err(io_error(format!(
                "wrote {written} of a {}-byte report",
                report.len()
            )));
        }
        Ok(())
    }

    fn read_packet_raw(&mut self, kind: u8) -> Result<Vec<u8>, CommunicationError> {
        let mut buffer = vec![0; READ_BUFFER];
        let timeout = i32::try_from(self.timeout.as_millis()).unwrap_or(i32::MAX);
        let got = self
            .pipe
            .read_timeout(&mut buffer, timeout)
            .map_err(io_error)?;
        if got == 0 {
            return Err(CommunicationError::Timeout);
        }
        let report = &buffer[..got];
        tracing::debug!(target: "rom", "{}: in {}", self.name, hex::encode(report));
        let [id, _, lo, hi, rest @ ..] = report else {
            return Err(CommunicationError::InvalidHeader);
        };
        let expected = match kind {
            COMMAND => COMMAND_IN,
            DATA => DATA_IN,
            _ => return Err(CommunicationError::InvalidHeader),
        };
        if *id != expected {
            return Err(CommunicationError::InvalidPacketReceived);
        }
        let len = usize::from(u16::from_le_bytes([*lo, *hi]));
        if len == 0 {
            return Err(CommunicationError::Aborted);
        }
        let packet = rest.get(..len).ok_or(CommunicationError::InvalidData)?;
        if kind == COMMAND {
            check_command(packet)?;
        }
        Ok(packet.to_vec())
    }
}

/// Whether the hidraw device at sysfs path `hidraw` is an interface of the USB device at `usb`.
/// Both paths are canonical. A device on a hub port nests under the hub's directory, so the
/// first component past `usb` has to be one of its interfaces, `<usb>:<config>.<interface>`.
#[must_use]
pub fn belongs_to(hidraw: &Path, usb: &Path) -> bool {
    let Some(name) = usb.file_name().and_then(|n| n.to_str()) else {
        return false;
    };
    hidraw
        .strip_prefix(usb)
        .ok()
        .and_then(|rest| rest.components().next())
        .and_then(|first| first.as_os_str().to_str())
        .is_some_and(|first| {
            first
                .strip_prefix(name)
                .is_some_and(|tail| tail.starts_with(':'))
        })
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::path::PathBuf;

    use mboot::McuBoot;

    use super::*;

    /// A HID device that expects these writes, in order, and answers with these reports.
    #[derive(Default)]
    struct Script {
        writes: RefCell<VecDeque<Vec<u8>>>,
        reads: RefCell<VecDeque<Vec<u8>>>,
    }

    impl HidPipe for Script {
        fn write(&self, report: &[u8]) -> Result<usize, String> {
            let expected = self.writes.borrow_mut().pop_front();
            assert_eq!(expected.as_deref(), Some(report), "an unexpected write");
            Ok(report.len())
        }

        fn read_timeout(&self, buffer: &mut [u8], _timeout_ms: i32) -> Result<usize, String> {
            Ok(match self.reads.borrow_mut().pop_front() {
                Some(report) => {
                    buffer[..report.len()].copy_from_slice(&report);
                    report.len()
                }
                None => 0,
            })
        }
    }

    fn bytes(text: &str) -> Vec<u8> {
        text.split_whitespace()
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect()
    }

    fn rom(writes: &[&str], reads: &[&str]) -> McuBoot<HidRom<Script>> {
        let script = Script {
            writes: RefCell::new(writes.iter().map(|w| bytes(w)).collect()),
            reads: RefCell::new(reads.iter().map(|r| bytes(r)).collect()),
        };
        McuBoot::new(HidRom::new(script, "test".into(), Duration::from_secs(1)))
    }

    /// UM11126 figure 26 gives GetProperty's packets in UART framing; table 253 the report
    /// that carries a packet over USB.
    #[test]
    fn get_property_travels_as_um11126_shows_it() {
        let mut boot = rom(
            &["01 00 0c 00 07 00 00 02 01 00 00 00 00 00 00 00"],
            &["03 00 0c 00 a7 00 00 02 00 00 00 00 00 01 02 4b"],
        );
        assert_eq!(crate::isp::rom_version(&mut boot).unwrap(), "K2.1.0");
    }

    /// UM11126 figure 30: ReadMemory, with a data phase from the ROM.
    #[test]
    fn read_memory_takes_its_data_phase_from_report_4() {
        let mut boot = rom(
            &["01 00 10 00 03 00 00 03 00 04 00 20 10 00 00 00 00 00 00 00"],
            &[
                "03 00 0c 00 a3 01 00 02 00 00 00 00 10 00 00 00",
                "04 00 10 00 d1 00 20 40 11 2e 01 20 00 00 00 00 00 00 00 00",
                "03 00 0c 00 a0 00 00 02 00 00 00 00 03 00 00 00",
            ],
        );
        let read = boot.read_memory(0x2000_0400, 0x10, 0).unwrap();
        assert_eq!(
            *read.bytes,
            bytes("d1 00 20 40 11 2e 01 20 00 00 00 00 00 00 00 00")
        );
    }

    #[test]
    fn a_report_of_the_wrong_kind_or_none_is_an_error() {
        let mut hid = HidRom::new(
            Script {
                reads: RefCell::new([bytes("04 00 01 00 aa"), bytes("04 00 00 00")].into()),
                ..Script::default()
            },
            "test".into(),
            Duration::from_secs(1),
        );
        assert!(matches!(
            hid.read_packet_raw(COMMAND),
            Err(CommunicationError::InvalidPacketReceived)
        ));
        assert!(matches!(
            hid.read_packet_raw(DATA),
            Err(CommunicationError::Aborted)
        ));
        assert!(matches!(
            hid.read_packet_raw(DATA),
            Err(CommunicationError::Timeout)
        ));
    }

    /// The second review's proof: a command report whose length field says 4. McuBoot reads a
    /// command packet's status at bytes 4 to 8 and its parameters from byte 8, so it panicked.
    #[test]
    fn a_short_command_report_is_an_error_not_a_panic() {
        for reply in [
            "03 00 04 00 a7 00 00 01",
            "03 00 07 00 a7 00 00 01 00 00 00",
        ] {
            let mut boot = rom(
                &["01 00 0c 00 07 00 00 02 01 00 00 00 00 00 00 00"],
                &[reply],
            );
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                crate::isp::rom_version(&mut boot)
            }));
            assert!(
                matches!(result, Ok(Err(CommunicationError::InvalidData))),
                "{reply}: {:?}",
                result.map_err(|_| "McuBoot panicked")
            );
        }
        // A data report carries any length, and is handed over as it is.
        let mut hid = HidRom::new(
            Script {
                reads: RefCell::new([bytes("04 00 04 00 01 02 03 04")].into()),
                ..Script::default()
            },
            "test".into(),
            Duration::from_secs(1),
        );
        assert_eq!(hid.read_packet_raw(DATA).unwrap(), [1, 2, 3, 4]);
    }

    /// The third review's proof, with the cases it lists and two more found in rblhost's
    /// parsers: command reports of eight bytes or more that McuBoot panics on. Each must come
    /// back as an error, as R3's shorter reports do.
    #[test]
    fn no_command_report_makes_mcuboot_panic() {
        let get_property = "01 00 0c 00 07 00 00 02 01 00 00 00 00 00 00 00";
        let read_memory = "01 00 10 00 03 00 00 03 00 04 00 20 10 00 00 00 00 00 00 00";
        let mut answered = Vec::new();
        for (what, write, reply) in [
            // The data-phase flag, and no parameter for the data phase's length.
            (
                "data phase, no length",
                get_property,
                "03 00 08 00 a7 01 00 00 00 00 00 00",
            ),
            // A generic response with no parameter after its status.
            (
                "generic, no parameter",
                get_property,
                "03 00 08 00 a0 00 00 00 00 00 00 00",
            ),
            // UM11126 8.4: the ROM aborts ReadMemory's data phase by sending its response without
            // the data-phase flag.
            (
                "read memory, data phase aborted",
                read_memory,
                "03 00 0c 00 a3 00 00 02 01 00 00 00 10 00 00 00",
            ),
            // A successful GetProperty response with no value: the property's parser reads one.
            (
                "get property, no value",
                get_property,
                "03 00 08 00 a7 00 00 01 00 00 00 00",
            ),
            // A tag McuBoot knows and has no parser for.
            (
                "flash read resource",
                get_property,
                "03 00 08 00 b0 00 00 01 00 00 00 00",
            ),
            // The same three again, each with a parameter count that matches its length, so that
            // only the rule for its own case refuses it.
            (
                "data phase, status only",
                read_memory,
                "03 00 08 00 a3 01 00 01 00 00 00 00",
            ),
            (
                "generic, status only",
                get_property,
                "03 00 08 00 a0 00 00 01 00 00 00 00",
            ),
            // A count that says two parameters, in a packet with room for one.
            (
                "generic, count longer than the packet",
                get_property,
                "03 00 08 00 a0 00 00 02 00 00 00 00",
            ),
            // A header with no status: McuBoot reads it from bytes 4 to 8.
            ("no status", get_property, "03 00 04 00 a7 00 00 00"),
        ] {
            let mut boot = rom(&[write], &[reply]);
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                if write == read_memory {
                    boot.read_memory(0x2000_0400, 0x10, 0).map(|_| ())
                } else {
                    crate::isp::rom_version(&mut boot).map(|_| ())
                }
            }));
            match result {
                Ok(Err(CommunicationError::InvalidData)) => {}
                Ok(other) => answered.push(format!("{what}: {other:?}")),
                Err(_) => answered.push(format!("{what}: McuBoot panicked")),
            }
        }
        assert!(answered.is_empty(), "{answered:#?}");
        // A failed GetProperty carries only its status, and McuBoot reports that status.
        let mut boot = rom(&[get_property], &["03 00 08 00 a7 00 00 01 3c 28 00 00"]);
        let failed = crate::isp::rom_version(&mut boot);
        assert!(
            matches!(failed, Err(CommunicationError::UnexpectedStatus(_, 10300))),
            "{failed:?}"
        );
    }

    #[test]
    fn a_hidraw_device_belongs_to_its_own_usb_device_only() {
        let root = PathBuf::from("/sys/devices/pci0000:00/0000:00:14.0/usb1");
        let at = |usb: &str, interface: &str| {
            root.join(usb)
                .join(interface)
                .join("0003:1FC9:0021.0001/hidraw/hidraw0")
        };
        let on_1_1 = at("1-1", "1-1:1.0");
        let on_1_10 = at("1-10", "1-10:1.0");
        assert!(belongs_to(&on_1_1, &root.join("1-1")));
        assert!(!belongs_to(&on_1_10, &root.join("1-1")));
        assert!(belongs_to(&on_1_10, &root.join("1-10")));
        // A device on port 2 of a hub at 1-1 nests under the hub's directory.
        let behind_hub = root.join("1-1/1-1.2/1-1.2:1.0/0003:1FC9:0021.0002/hidraw/hidraw1");
        assert!(!belongs_to(&behind_hub, &root.join("1-1")));
        assert!(belongs_to(&behind_hub, &root.join("1-1/1-1.2")));
    }
}
