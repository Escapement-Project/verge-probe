//! The real bus: `nusb` for enumeration and the CMSIS-DAP v2 interface, and NXP's McuBoot client
//! over `hidapi` for the ROM bootloader, which is a HID device.

use std::time::{Duration, Instant};

use nusb::descriptors::TransferType;
use nusb::transfer::{
    Buffer, Bulk, ControlOut, ControlType, Direction, In, Out, Recipient, TransferError,
};
use nusb::{DeviceInfo, ErrorKind, MaybeFuture};

use crate::bus::{Bus, BusError, UsbDevice};
use crate::dap::{DapLink, LinkError};
use crate::hid::HidRom;
use crate::ids;

const USB_CLASS_HID: u8 = 0x03;
const USB_CLASS_VENDOR: u8 = 0xFF;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(2);
const DRAIN_TIMEOUT: Duration = Duration::from_millis(50);
/// Erasing and writing a 4 KB chunk is one bootloader command; this bounds each reply.
const ROM_TIMEOUT: Duration = Duration::from_secs(10);

/// USB as the operating system presents it.
pub struct UsbBus {
    started: Instant,
}

impl Default for UsbBus {
    fn default() -> UsbBus {
        UsbBus {
            started: Instant::now(),
        }
    }
}

fn is_cmsis_dap(text: &str) -> bool {
    text.to_ascii_lowercase().contains("cmsis-dap")
}

fn describe(info: &DeviceInfo) -> UsbDevice {
    let product = info.product_string().map(str::to_owned);
    let dap_v2 = info.interfaces().any(|interface| {
        interface.class() == USB_CLASS_VENDOR
            && match interface.interface_string() {
                Some(name) => is_cmsis_dap(name),
                None => product.as_deref().is_some_and(is_cmsis_dap),
            }
    });
    let location = (!info.port_chain().is_empty()).then(|| {
        let ports: Vec<String> = info.port_chain().iter().map(u8::to_string).collect();
        format!("{}-{}", info.bus_id(), ports.join("."))
    });
    UsbDevice {
        vid: info.vendor_id(),
        pid: info.product_id(),
        location,
        instance: format!("{:?}", info.id()),
        serial: info.serial_number().map(str::to_owned),
        manufacturer: info.manufacturer_string().map(str::to_owned),
        product,
        bcd_device: info.device_version(),
        dap_v2,
        hid: info
            .interfaces()
            .any(|interface| interface.class() == USB_CLASS_HID),
    }
}

impl UsbBus {
    fn find(&self, device: &UsbDevice) -> Result<DeviceInfo, BusError> {
        nusb::list_devices()
            .wait()
            .map_err(|error| BusError::Enumerate(error.to_string()))?
            .find(|info| format!("{:?}", info.id()) == device.instance)
            .ok_or_else(|| BusError::Gone {
                device: device.to_string(),
            })
    }
}

/// The hidraw node of `info`'s HID interface, matched through sysfs: the node's device directory
/// sits under the USB device's own.
#[cfg(target_os = "linux")]
fn rom_path<'a>(
    info: &DeviceInfo,
    candidates: &[&'a hidapi::DeviceInfo],
) -> Result<&'a std::ffi::CStr, String> {
    use std::path::Path;
    let usb = std::fs::canonicalize(info.sysfs_path())
        .map_err(|error| format!("{}: {error}", info.sysfs_path().display()))?;
    let theirs: Vec<&std::ffi::CStr> = candidates
        .iter()
        .map(|candidate| candidate.path())
        .filter(|path| {
            path.to_str()
                .ok()
                .and_then(|node| Path::new(node).file_name())
                .and_then(|name| {
                    std::fs::canonicalize(Path::new("/sys/class/hidraw").join(name)).ok()
                })
                .is_some_and(|hidraw| crate::hid::belongs_to(&hidraw, &usb))
        })
        .collect();
    match theirs.as_slice() {
        [path] => Ok(path),
        [] => Err("its HID interface is not ready yet".into()),
        several => Err(format!("{} HID interfaces belong to it", several.len())),
    }
}

/// Elsewhere this takes the first device with the bootloader's ID, as NXP's client does. On
/// Windows neither crate ties a HID device to its USB device. macOS could match hidapi's location
/// ID to nusb's, which is not written yet. [`crate::update::flash_bootloader`] then checks that
/// the bootloader that answered is still the only one, and the one it checked.
#[cfg(not(target_os = "linux"))]
fn rom_path<'a>(
    _info: &DeviceInfo,
    candidates: &[&'a hidapi::DeviceInfo],
) -> Result<&'a std::ffi::CStr, String> {
    candidates
        .first()
        .map(|candidate| candidate.path())
        .ok_or_else(|| "its HID interface is not ready yet".into())
}

impl Bus for UsbBus {
    type Dap = NusbDap;
    type Isp = HidRom;

    fn devices(&self) -> Result<Vec<UsbDevice>, BusError> {
        Ok(nusb::list_devices()
            .wait()
            .map_err(|error| BusError::Enumerate(error.to_string()))?
            .filter(|info| ids::role(info.vendor_id(), info.product_id()).is_some())
            .map(|info| describe(&info))
            .collect())
    }

    fn open_dap(&self, device: &UsbDevice) -> Result<NusbDap, BusError> {
        let name = device.to_string();
        let open_error = |reason: String| BusError::Open {
            device: name.clone(),
            reason,
        };
        let info = self.find(device)?;
        let usb = info.open().wait().map_err(|error| match error.kind() {
            ErrorKind::Busy => BusError::Busy {
                device: name.clone(),
            },
            _ => open_error(error.to_string()),
        })?;
        let configuration = usb
            .active_configuration()
            .map_err(|error| open_error(error.to_string()))?;
        for interface in configuration.interfaces() {
            let number = interface.interface_number();
            let named_dap = info
                .interfaces()
                .find(|i| i.interface_number() == number)
                .is_some_and(|i| match i.interface_string() {
                    Some(name) => is_cmsis_dap(name),
                    None => device.product.as_deref().is_some_and(is_cmsis_dap),
                });
            for alt in interface.alt_settings() {
                if alt.class() != USB_CLASS_VENDOR || !named_dap {
                    continue;
                }
                let endpoints: Vec<_> = alt.endpoints().collect();
                let [out_ep, in_ep, ..] = endpoints.as_slice() else {
                    continue;
                };
                if out_ep.transfer_type() != TransferType::Bulk
                    || out_ep.direction() != Direction::Out
                    || in_ep.transfer_type() != TransferType::Bulk
                    || in_ep.direction() != Direction::In
                {
                    continue;
                }
                let (out_address, in_address) = (out_ep.address(), in_ep.address());
                let claimed =
                    usb.claim_interface(number)
                        .wait()
                        .map_err(|error| match error.kind() {
                            ErrorKind::Busy => BusError::Busy {
                                device: name.clone(),
                            },
                            _ => open_error(error.to_string()),
                        })?;
                let out = claimed
                    .endpoint::<Bulk, Out>(out_address)
                    .map_err(|error| open_error(error.to_string()))?;
                let input = claimed
                    .endpoint::<Bulk, In>(in_address)
                    .map_err(|error| open_error(error.to_string()))?;
                return Ok(NusbDap {
                    interface: claimed,
                    out,
                    input,
                });
            }
        }
        Err(open_error("it has no CMSIS-DAP v2 bulk interface".into()))
    }

    fn device_control_out(
        &self,
        device: &UsbDevice,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<Result<(), LinkError>, BusError> {
        let usb = self
            .find(device)?
            .open()
            .wait()
            .map_err(|error| BusError::Open {
                device: device.to_string(),
                reason: error.to_string(),
            })?;
        Ok(control_out(request_type, request, value, index)
            .and_then(|control| to_device(&usb, control)))
    }

    fn open_isp(&self, device: &UsbDevice) -> Result<HidRom, BusError> {
        let open_error = |reason: String| BusError::Open {
            device: device.to_string(),
            reason,
        };
        let info = self.find(device)?;
        let api = hidapi::HidApi::new().map_err(|error| open_error(error.to_string()))?;
        let candidates: Vec<&hidapi::DeviceInfo> = api
            .device_list()
            .filter(|d| d.vendor_id() == device.vid && d.product_id() == device.pid)
            .collect();
        let path = rom_path(&info, &candidates).map_err(open_error)?;
        let hid = api
            .open_path(path)
            .map_err(|error| open_error(error.to_string()))?;
        Ok(HidRom::new(hid, device.to_string(), ROM_TIMEOUT))
    }

    fn now(&self) -> Duration {
        self.started.elapsed()
    }

    fn sleep(&self, duration: Duration) {
        std::thread::sleep(duration);
    }

    /// nusb's `DeviceId` on Windows is the devnode of the device instance ID, which a device that
    /// comes back keeps.
    fn keeps_instances(&self) -> bool {
        cfg!(windows)
    }

    /// Only Linux ties a HID device to its USB device ([`rom_path`]).
    fn tells_bootloaders_apart(&self) -> bool {
        cfg!(target_os = "linux")
    }
}

/// A claimed CMSIS-DAP v2 interface. Dropping it releases the interface.
pub struct NusbDap {
    interface: nusb::Interface,
    out: nusb::Endpoint<Bulk, Out>,
    input: nusb::Endpoint<Bulk, In>,
}

fn link_error(error: TransferError) -> LinkError {
    match error {
        TransferError::Cancelled => LinkError::Timeout,
        TransferError::Disconnected => LinkError::Disconnected,
        other => LinkError::Usb(other.to_string()),
    }
}

impl NusbDap {
    fn read(&mut self, timeout: Duration) -> Result<Vec<u8>, LinkError> {
        // A reply as long as the packet ends without a short packet, so ask for exactly one packet.
        let len = self.input.max_packet_size();
        let completion = self.input.transfer_blocking(Buffer::new(len), timeout);
        completion
            .into_result()
            .map(Buffer::into_vec)
            .map_err(link_error)
    }
}

/// A control request with no data stage, from its setup fields.
fn control_out(
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
) -> Result<ControlOut<'static>, LinkError> {
    let control_type = match (request_type >> 5) & 0x3 {
        0 => ControlType::Standard,
        1 => ControlType::Class,
        2 => ControlType::Vendor,
        _ => {
            return Err(LinkError::Usb(format!(
                "bmRequestType 0x{request_type:02x}"
            )));
        }
    };
    let recipient = match request_type & 0x1f {
        0 => Recipient::Device,
        1 => Recipient::Interface,
        2 => Recipient::Endpoint,
        _ => Recipient::Other,
    };
    if request_type & 0x80 != 0 {
        return Err(LinkError::Usb("an IN request on control_out".into()));
    }
    Ok(ControlOut {
        control_type,
        recipient,
        request,
        value,
        index,
        data: &[],
    })
}

#[cfg(not(windows))]
fn to_device(usb: &nusb::Device, control: ControlOut<'_>) -> Result<(), LinkError> {
    usb.control_out(control, COMMAND_TIMEOUT)
        .wait()
        .map_err(link_error)
}

#[cfg(windows)]
fn to_device(_usb: &nusb::Device, _control: ControlOut<'_>) -> Result<(), LinkError> {
    Err(LinkError::Usb(
        "on Windows a control request reaches a device only through an interface the tool has claimed".into(),
    ))
}

impl DapLink for NusbDap {
    fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError> {
        self.out
            .transfer_blocking(Buffer::from(request), COMMAND_TIMEOUT)
            .into_result()
            .map_err(link_error)?;
        self.read(COMMAND_TIMEOUT)
    }

    fn drain(&mut self) -> Result<usize, LinkError> {
        let mut count = 0;
        loop {
            match self.read(DRAIN_TIMEOUT) {
                Ok(_) => count += 1,
                Err(LinkError::Timeout) => return Ok(count),
                Err(error) => return Err(error),
            }
        }
    }

    fn control_out(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<(), LinkError> {
        self.interface
            .control_out(
                control_out(request_type, request, value, index)?,
                COMMAND_TIMEOUT,
            )
            .wait()
            .map_err(link_error)
    }
}
