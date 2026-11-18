//! Batches on a bus that refuses a second open of a unit already open, as real USB does
//! (nusb's claim, WinUSB and IOKit are exclusive); the simulated bench alone lets two handles open
//! one unit. Each command holds its unit for 2 ms of real time, so the blink thread meets the
//! update's own opens. The third review's evidence for decision 10, kept as a test.

use std::collections::HashSet;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use escapement_verge_probe::batch::{
    Armed, Batch, Board, Filter, Observer, OursFilter, Record, Settled, StockFilter,
};
use escapement_verge_probe::bus::{BusError, UsbDevice};
use escapement_verge_probe::dap::{DapLink, LinkError};
use escapement_verge_probe::progress::Progress;
use escapement_verge_probe::sim::{SimBus, SimDap, SimFirmware, SimRom, firmware_image};
use escapement_verge_probe::source::{ImageVersion, Source};
use escapement_verge_probe::update::Timing;
use escapement_verge_probe::version::StockVersion;
use escapement_verge_probe::{Bus, Cancel};

struct Held {
    dap: SimDap,
    instance: String,
    open: Arc<Mutex<HashSet<String>>>,
}

impl Drop for Held {
    fn drop(&mut self) {
        self.open.lock().unwrap().remove(&self.instance);
    }
}

impl DapLink for Held {
    fn exchange(&mut self, request: &[u8]) -> Result<Vec<u8>, LinkError> {
        std::thread::sleep(Duration::from_millis(2));
        self.dap.exchange(request)
    }
    fn drain(&mut self) -> Result<usize, LinkError> {
        self.dap.drain()
    }
    fn control_out(&mut self, a: u8, b: u8, c: u16, d: u16) -> Result<(), LinkError> {
        self.dap.control_out(a, b, c, d)
    }
}

struct Exclusive<'a> {
    bus: &'a SimBus,
    open: Arc<Mutex<HashSet<String>>>,
    busy: AtomicUsize,
}

impl Bus for Exclusive<'_> {
    type Aux = escapement_verge_probe::sim::SimAux;
    type Dap = Held;
    type Isp = SimRom;
    fn devices(&self) -> Result<Vec<UsbDevice>, BusError> {
        self.bus.devices()
    }
    fn every_device(&self) -> Result<Vec<UsbDevice>, BusError> {
        self.bus.every_device()
    }
    fn open_dap(&self, device: &UsbDevice) -> Result<Held, BusError> {
        if !self.open.lock().unwrap().insert(device.instance.clone()) {
            self.busy.fetch_add(1, Ordering::Relaxed);
            return Err(BusError::Busy {
                device: device.to_string(),
            });
        }
        match self.bus.open_dap(device) {
            Ok(dap) => Ok(Held {
                dap,
                instance: device.instance.clone(),
                open: self.open.clone(),
            }),
            Err(error) => {
                self.open.lock().unwrap().remove(&device.instance);
                Err(error)
            }
        }
    }
    fn open_aux(
        &self,
        device: &UsbDevice,
    ) -> Result<escapement_verge_probe::sim::SimAux, BusError> {
        self.bus.open_aux(device)
    }

    fn device_control_out(
        &self,
        device: &UsbDevice,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    ) -> Result<Result<(), LinkError>, BusError> {
        self.bus
            .device_control_out(device, request_type, request, value, index)
    }
    fn open_isp(&self, device: &UsbDevice) -> Result<SimRom, BusError> {
        self.bus.open_isp(device)
    }
    fn now(&self) -> Duration {
        self.bus.now()
    }
    fn sleep(&self, duration: Duration) {
        self.bus.sleep(duration);
    }
    fn keeps_instances(&self) -> bool {
        self.bus.keeps_instances()
    }
}

#[derive(Default)]
struct Seen {
    records: Vec<Record>,
}

impl Progress for Seen {}

impl Observer for Seen {
    fn record(&mut self, record: &Record) {
        self.records.push(record.clone());
    }
}

#[test]
fn blinking_beside_an_update_with_exclusive_opens() {
    let stock = || SimFirmware::Stock {
        version: StockVersion::parse("3.172").unwrap(),
    };
    let sim = SimBus::default().paced(0.02);
    let bus = Exclusive {
        bus: &sim,
        open: Arc::default(),
        busy: AtomicUsize::new(0),
    };
    let timing = Timing {
        poll: Duration::from_millis(10),
        ..Timing::default()
    };
    let armed = Armed {
        image: firmware_image(&SimFirmware::Ours {
            build: "v0.3.0".into(),
        }),
        name: "release v0.3.0".into(),
        version: ImageVersion::Ours("v0.3.0".into()),
        source: Source::Release(None),
        repo: Default::default(),
    };
    let filter = Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Older,
        bootloaders: false,
        attached: false,
    };
    let mut batch = Batch::arm(
        &bus,
        armed,
        filter,
        timing,
        Cancel::new(),
        Cancel::new(),
        std::num::NonZeroUsize::new(4),
    )
    .unwrap();
    let mut seen = Seen::default();
    batch.step(&mut seen).unwrap();
    for _ in 0..4 {
        sim.plug_in_later(Duration::from_secs(1), stock());
    }
    batch.run_to_end(&mut seen).unwrap();
    let results: Vec<Settled> = seen.records.iter().map(|r| r.result).collect();
    assert_eq!(results, [Settled::Updated; 4], "{:?}", seen.records);
    // The opens did meet: otherwise this proves nothing about contention.
    assert!(bus.busy.load(Ordering::Relaxed) > 0, "no open was refused");
}
