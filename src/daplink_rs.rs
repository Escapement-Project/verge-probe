//! daplink-rs, a Rust CMSIS-DAP firmware with a build for the MCU-Link
//! (github.com/mbrossard/DAPLink, branch `rust`). The tool knows it so that a unit running it can
//! be updated without the ISP jumper.
//!
//! It enumerates with Arm's DAPLink ID, which every DAPLink probe shares. A device with that ID
//! is taken for daplink-rs on an MCU-Link only when its USB strings say so, and nothing is sent to
//! one whose strings do not: another DAPLink gives Verge's vendor command numbers other meanings,
//! 0x89 resetting its target.

use crate::bus::UsbDevice;
use crate::dap::{DapError, DapLink, EntryError};

/// The manufacturer string of every daplink-rs build.
pub const MANUFACTURER: &str = "daplink-rs";
/// The product string of its MCU-Link build, `product_string!("mculink")` in its
/// `firmware/mculink`.
pub const PRODUCT: &str = "daplink-rs CMSIS-DAP (mculink)";

/// Its reboot-to-bootloader command (`crates/daplink-rs/src/reboot.rs`). Discovery answers
/// `9f 'd' 'l' 'r' 's' version kind`. The reboot carries a magic and answers a status.
pub const REBOOT_COMMAND: u8 = 0x9F;
const DISCOVER: [u8; 2] = [REBOOT_COMMAND, 0x00];
const REBOOT: [u8; 6] = [REBOOT_COMMAND, 0x01, b'B', b'O', b'O', b'T'];
const MAGIC: &[u8; 4] = b"dlrs";
/// A build with no bootloader to restart into.
pub const KIND_NONE: u8 = 0x00;
/// The bootloader kind that is the LPC55S69 ROM's USB HID ISP, the one this tool flashes.
pub const KIND_NXP_ISP: u8 = 0x03;
const STATUS_ARMED: u8 = 0x00;
const STATUS_UNSUPPORTED: u8 = 0x01;
const STATUS_BAD_MAGIC: u8 = 0x02;
const STATUS_BUSY: u8 = 0x03;

/// Whether `device`'s strings say it runs daplink-rs's MCU-Link build. The product string decides,
/// and a manufacturer string must agree where the platform reports one: on Windows nusb knows it
/// only by asking the device, which enumeration does not do.
#[must_use]
pub fn is_mculink_build(device: &UsbDevice) -> bool {
    device.product.as_deref() == Some(PRODUCT)
        && device
            .manufacturer
            .as_deref()
            .is_none_or(|manufacturer| manufacturer == MANUFACTURER)
}

/// What a build's reboot command says it restarts into.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reboot {
    pub version: u8,
    pub kind: u8,
}

impl Reboot {
    /// Whether it restarts into the bootloader this tool flashes.
    #[must_use]
    pub fn reaches_isp(self) -> bool {
        self.kind == KIND_NXP_ISP
    }
}

/// Asks the reboot command what it restarts into, which changes nothing. `None` for a build
/// without the command, which answers it as unknown.
pub fn discover(link: &mut dyn DapLink) -> Result<Option<Reboot>, DapError> {
    Ok(parse_discover(&link.exchange(&DISCOVER)?))
}

fn parse_discover(reply: &[u8]) -> Option<Reboot> {
    match reply.get(..7)? {
        [REBOOT_COMMAND, magic @ .., version, kind] if magic == MAGIC => Some(Reboot {
            version: *version,
            kind: *kind,
        }),
        _ => None,
    }
}

/// Asks daplink-rs to restart into its bootloader. It leaves the bus about 50 ms after it
/// answers.
pub fn request_bootloader(link: &mut dyn DapLink) -> Result<(), EntryError> {
    let reply = link.exchange(&REBOOT).map_err(DapError::from)?;
    match reply.as_slice() {
        [REBOOT_COMMAND, STATUS_ARMED, ..] => Ok(()),
        [REBOOT_COMMAND, STATUS_BUSY, ..] => Err(EntryError::DaplinkRsBusy),
        [REBOOT_COMMAND, status, ..] => Err(EntryError::DaplinkRsRefused(match *status {
            STATUS_UNSUPPORTED => "it has no bootloader to restart into".to_owned(),
            STATUS_BAD_MAGIC => "it did not take the request as a restart".to_owned(),
            other => format!("it answered status {other}"),
        })),
        other => Err(EntryError::DaplinkRsRefused(format!(
            "it answered {}",
            hex::encode(&other[..other.len().min(8)])
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dap::LinkError;

    /// Answers every command with one reply.
    struct Answer(Vec<u8>);

    impl DapLink for Answer {
        fn exchange(&mut self, _: &[u8]) -> Result<Vec<u8>, LinkError> {
            Ok(self.0.clone())
        }
        fn drain(&mut self) -> Result<usize, LinkError> {
            Ok(0)
        }
        fn control_out(&mut self, _: u8, _: u8, _: u16, _: u16) -> Result<(), LinkError> {
            Err(LinkError::Usb("stalled".into()))
        }
    }

    fn device(manufacturer: Option<&str>, product: &str) -> UsbDevice {
        UsbDevice {
            vid: 0x0d28,
            pid: 0x0204,
            location: None,
            instance: "1".into(),
            serial: Some("0000AA2B6B19D4E7".into()),
            manufacturer: manufacturer.map(str::to_owned),
            product: Some(product.into()),
            bcd_device: 0x0010,
            dap_v2: true,
            hid: false,
            aux: false,
        }
    }

    #[test]
    fn only_its_mculink_build_has_the_role() {
        use crate::ids::Role;
        assert_eq!(
            device(Some(MANUFACTURER), PRODUCT).role(),
            Some(Role::DaplinkRs)
        );
        // As Windows enumerates it, with no manufacturer string.
        assert_eq!(device(None, PRODUCT).role(), Some(Role::DaplinkRs));
        // Its other builds, and every other DAPLink, share the ID and are left alone.
        assert_eq!(
            device(Some(MANUFACTURER), "daplink-rs CMSIS-DAP (rp2040)").role(),
            None
        );
        assert_eq!(device(Some("ARM"), "DAPLink CMSIS-DAP").role(), None);
        assert_eq!(device(Some("ARM"), PRODUCT).role(), None);
    }

    #[test]
    fn a_refused_restart_says_why() {
        assert_eq!(request_bootloader(&mut Answer(vec![0x9F, 0x00])), Ok(()));
        let refused = |reply: Vec<u8>| match request_bootloader(&mut Answer(reply)) {
            Err(EntryError::DaplinkRsRefused(why)) => why,
            other => panic!("{other:?}"),
        };
        assert_eq!(
            request_bootloader(&mut Answer(vec![0x9F, 0x03])),
            Err(EntryError::DaplinkRsBusy)
        );
        assert!(refused(vec![0x9F, 0x01]).contains("no bootloader"));
        assert_eq!(refused(vec![0xFF]), "it answered ff");
    }

    #[test]
    fn discovery_is_read_only_as_its_firmware_answers_it() {
        assert_eq!(
            parse_discover(b"\x9fdlrs\x01\x03"),
            Some(Reboot {
                version: 1,
                kind: KIND_NXP_ISP
            })
        );
        // A build without the command, and a reply of the wrong length.
        assert_eq!(parse_discover(&[0xFF]), None);
        assert_eq!(parse_discover(b"\x9fdlrs\x01"), None);
        assert_eq!(parse_discover(b"\x9fDLRS\x01\x03"), None);
    }
}
