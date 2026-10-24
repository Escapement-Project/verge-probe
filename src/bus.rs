//! What the tool needs from USB, as a trait the real backend and the simulator both implement.

use std::fmt;
use std::time::Duration;

use mboot::protocols::Protocol;

use crate::dap::{DapLink, LinkError};
use crate::ids::{self, Role};

/// A device as enumeration describes it, before anything opens it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UsbDevice {
    pub vid: u16,
    pub pid: u16,
    /// The physical port, stable while the unit stays plugged into it, `None` where the platform
    /// does not say.
    pub location: Option<String>,
    /// The device while it stays on the bus. On Linux and macOS it changes whenever the device
    /// leaves the bus and comes back. On Windows it is the device instance ID's devnode, which a
    /// device that comes back keeps: by its serial number, or by its port without one. There a
    /// return is seen only as a change of ID, or by looking while the device is gone.
    pub instance: String,
    pub serial: Option<String>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    /// bcdDevice. Ours puts the functions its jumpers leave in the low byte, bit 0 the debug
    /// interfaces.
    pub bcd_device: u16,
    /// Offers a vendor-class interface whose string contains "CMSIS-DAP": the v2 bulk pair.
    pub dap_v2: bool,
    /// Offers a HID interface, which is all stock V2.x and the ROM bootloader have.
    pub hid: bool,
}

impl UsbDevice {
    #[must_use]
    pub fn role(&self) -> Option<Role> {
        ids::role(self.vid, self.pid)
    }
}

impl fmt::Display for UsbDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04x}:{:04x}", self.vid, self.pid)?;
        if let Some(location) = &self.location {
            write!(f, " at {location}")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum BusError {
    #[error("could not list USB devices: {0}")]
    Enumerate(String),
    #[error("{device} is in use by another program")]
    Busy { device: String },
    #[error("{device} is no longer attached")]
    Gone { device: String },
    #[error("could not open {device}: {reason}")]
    Open { device: String, reason: String },
}

/// USB as the tool uses it.
///
/// Time is part of the bus so that the simulator can run an update's waits in simulated time;
/// every wait in the tool goes through [`Bus::sleep`].
pub trait Bus {
    type Dap: DapLink;
    type Isp: Protocol;

    /// Every device with an ID in [`ids::KNOWN_IDS`].
    fn devices(&self) -> Result<Vec<UsbDevice>, BusError>;
    /// Claims a probe's CMSIS-DAP v2 interface.
    fn open_dap(&self, device: &UsbDevice) -> Result<Self::Dap, BusError>;
    /// Sends a control request with no data stage to the device itself, claiming no interface:
    /// how the tool reaches ours once the SWD jumper has taken away the interface it would claim.
    /// The outer error says the device could not be opened, the inner one that the request failed.
    /// The request fails on Windows, where WinUSB sends control requests only through a claimed
    /// interface.
    fn device_control_out(
        &self,
        device: &UsbDevice,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<Result<(), LinkError>, BusError>;
    /// Opens the ROM bootloader's HID interface.
    fn open_isp(&self, device: &UsbDevice) -> Result<Self::Isp, BusError>;
    /// Time since an arbitrary start.
    fn now(&self) -> Duration;
    fn sleep(&self, duration: Duration);
    /// Whether a device that leaves the bus and comes back keeps its [`UsbDevice::instance`], as
    /// on Windows. Where it does not, an instance still there means the device never left.
    fn keeps_instances(&self) -> bool;
    /// Whether [`Bus::open_isp`] opens the bootloader of the device it is given, so one among
    /// several can be chosen. Where it does not, it opens the first bootloader it finds.
    fn tells_bootloaders_apart(&self) -> bool {
        false
    }
}
