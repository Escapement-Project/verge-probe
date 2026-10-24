# verge-probe

`verge-probe` is the shared library for the Verge CLI and GUI. It provides probe discovery,
firmware identification, debug commands, GPIO and bus bridges, energy monitoring, reset and
bootloader entry, firmware image and release handling, updates and batch operations. It contains
no user interface.

The library keeps update and ROM-bootloader operations behind its `Bus` API. For target debugging,
it uses probe-rs. `VergeSession` adds typed commands for Verge firmware while keeping the attached
probe-rs session available for normal target access:

```rust,no_run
use verge_probe::probe_rs::{Permissions, probe::list::Lister};
use verge_probe::{VergeProbe, VergeSession, install_cjtag_support};

let lister = Lister::new();
let info = lister.list_all().into_iter().next().expect("no probe found");
let mut probe = info.open()?;
install_cjtag_support(&mut probe)?;
{
    let mut verge = VergeProbe::new(&mut probe);
    let info = verge.info()?.expect("probe does not run Verge firmware");
    if info.supports_target_power() {
        verge.set_target_power(true)?;
    }
}
let mut session = probe.attach("MSPM0L1306", Permissions::default())?;
let mut verge = VergeSession::new(&mut session);
let info = verge.info()?;
let target_sense = verge.read_target_sense()?;
let spi = verge.spi_transfer(100_000, verge_probe::SpiMode::Mode0, false, &[0xA5])?;
let i2c = verge.i2c_transfer(0x50, 100_000, &[0], 2)?;
let _target_core = verge.session().core(0)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

The same library exposes the workflows used by both frontends:

```rust,no_run
use std::path::{Path, PathBuf};
use verge_probe::{Source, UsbBus, detect, image::Image};

let bus = UsbBus::default();
let units = detect(&bus)?;
let image = Image::from_path(Path::new("firmware.s19"))?;
let source = Source::File(PathBuf::from("firmware.s19"));
let _ = (units, source);
# Ok::<(), Box<dyn std::error::Error>>(())
```

`examples/pro_bridges.rs` requires a probe serial and runs only the operations named on its command
line. It can send a one-byte SPI transfer, read one byte from a selected 7-bit I2C address, or run a
cancellable energy monitor for five seconds. It does not change target power. The monitor runs in
the calling thread; UI code should run it in a worker. Firmware acquisition stays active between
host polls, and the probe's accumulator covers pairs the host did not poll individually.

```sh
cargo run --example pro_bridges -- CJ8C2F60A3 --spi
cargo run --example pro_bridges -- CJ8C2F60A3 --i2c 0x50
cargo run --example pro_bridges -- CJ8C2F60A3 --energy
```

Use `--i2c` only with a known device on the bridge pins. The example performs a read-only
transaction.

Run the library tests with `cargo test --features simulate`. The simulation backend is available
only with that feature; release builds leave it disabled.

The probe-rs APIs needed for CMSIS-DAP vendor commands and cJTAG are not in a published release
yet. During development, `Cargo.toml` points to the probe-rs checkout. The release dependency must
pin the probe-rs commit that provides those APIs.
