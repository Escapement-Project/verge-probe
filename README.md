# verge-probe

`verge-probe` is the shared library for the Verge CLI and GUI. It provides probe discovery,
firmware identification, debug commands, GPIO and bus bridges, energy monitoring, reset and
bootloader entry, firmware image and release handling, updates and batch operations.

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

Typed commands fail with `CommandError`, which says whether the connection failed, the firmware
lacks the command, the request was out of range, the reply broke the protocol, or the firmware
refused or failed it. A refusal carries the state the firmware reported with it. SPI and I2C calls
return the received bytes. The latest energy reading is a sample, no sample yet, or stopped. The
accumulator also keeps its total after an acquisition fault.

On an MCU-Link Pro, the supply and energy commands are also on Verge Aux, a second vendor interface
that debuggers leave alone. `Bus::open_aux` claims it, and `aux::Aux` sends requests over it. `Aux`
is a `VergeTransport`, so the same typed calls run on it while a debugger holds CMSIS-DAP. The probe
also streams every energy pair through it: `stream` parses the messages and converts each pair as
the firmware does, checked against the vectors the firmware's own converter writes.

Both frontends use the library for discovery, image loading and updates:

```rust,no_run
use std::path::{Path, PathBuf};
use verge_probe::{Source, detect, image::Image};
use verge_probe::usb::UsbBus;

let bus = UsbBus::default();
let units = detect(&bus)?;
let image = Image::from_path(Path::new("firmware.s19"))?;
let source = Source::File(PathBuf::from("firmware.s19"));
let _ = (units, source);
# Ok::<(), Box<dyn std::error::Error>>(())
```

`examples/pro_bridges.rs` requires a probe serial number and runs the operations selected on its
command line. It can check GPIO direction and readback, test all four SPI modes with a two-byte
MOSI-to-MISO loopback, read one byte from a selected 7-bit I2C address, or run a cancellable energy
monitor for five seconds. For the SPI check, connect J19 pin 3 (MOSI) to pin 5 (MISO). The example
does not change target power. The monitor runs in the calling thread; UI code should run it in a worker.
Firmware acquisition stays active between host polls, and the probe's accumulator covers pairs the
host did not poll individually.

```sh
cargo run --example pro_bridges -- CJ8C2F60A3 --spi
cargo run --example pro_bridges -- CJ8C2F60A3 --dio
cargo run --example pro_bridges -- CJ8C2F60A3 --i2c 0x50
cargo run --example pro_bridges -- CJ8C2F60A3 --energy
```

Use `--i2c` only with a known device on the bridge pins. The example performs a read-only
transaction.

The simulation backend sits behind the `simulate` feature, which release builds leave off. The
tests turn it on themselves.

Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo test --locked --all-targets --all-features
```

`Cargo.toml` pins the probe-rs Git revision that provides CMSIS-DAP vendor commands and cJTAG.
