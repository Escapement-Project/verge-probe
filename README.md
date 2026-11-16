# verge-probe

`verge-probe` is the shared library for the Verge CLI and GUI. It provides probe discovery,
firmware identification, debug commands, GPIO and bus bridges, energy monitoring, reset and
bootloader entry, firmware image and release handling, updates and batch operations.

The library keeps update and ROM-bootloader operations behind its `Bus` API. For target debugging,
it uses probe-rs. `VergeSession` adds typed commands for Verge firmware while keeping the attached
probe-rs session available for normal target access. [docs/library.md](docs/library.md) has
examples for a tool built on the library, which `cargo test --doc` builds.

Typed commands fail with `CommandError`, which says whether the connection failed, the firmware
lacks the command, the request was out of range, the reply broke the protocol, or the firmware
refused or failed it. A refusal carries the state the firmware reported with it. SPI and I2C calls
return the received bytes. The latest energy reading is a sample, no sample yet, or stopped. The
accumulator also keeps its total after an acquisition fault. `set_energy_ranges` picks the current
ranges for the next capture alone: automatic switching, the default, or the high range held. In
either, a pair read on the high range too near the amplifier's floor comes marked under range (the
firmware's docs/host-interface.md, `0x90` mode 14).

The identify and DIO commands are also on Verge Aux, a second vendor interface that debuggers leave
alone. On an MCU-Link Pro, so are the supply, energy and marker commands. `Bus::open_aux` claims
Verge Aux, and `aux::Aux` sends requests over it. `Aux` is a `VergeTransport`, so the same typed
calls run on it while a debugger holds CMSIS-DAP. The probe also streams every energy pair through
it: `stream` parses the messages and converts each pair as the firmware does, checked against the
vectors the firmware's own converter writes.

`pro::with_pro` runs a call through Aux, or through CMSIS-DAP when another program holds Aux.
`pro::open_aux` opens Aux alone, for a frontend that keeps it open beside a debugger.
`pro::open_stream` opens it for the energy stream, which only Aux carries. All three refuse a unit
that is not running Verge, whose board lacks the feature or whose firmware does, with a `ProError`
that says which. The DIO pins are on the Pro and the standalone MCU-Link; the supply and energy
capture are the Pro's. A board with DIO pins and no energy capture, a standalone MCU-Link or a Pro
without its energy circuit, streams its DIO pins alone (`start_dio_stream`). `record::Sink` writes
pairs and DIO edges as CSV or as a sigrok session.

The `capture` feature adds disk-backed captures, in SQLite. `capture::Recorder` takes stream
messages without waiting for the disk, and fails rather than drop one when the disk falls behind.
A capture keeps the original messages, the converted pairs, the pairs lost, and sums at several
scales. `capture::Archive` reads it while a recording continues: statistics for any interval, an
envelope of at most a set number of bins for drawing one, and exports of an interval as CSV or
sigrok, with voltage, current and GAP but no DIO.

An interval includes its start and excludes its end. Each pair covers its own period, clipped at
the next pair's time, and time without pairs adds nothing to charge or energy. The sums are whole
numbers, so an interval's statistics do not depend on how it was split to compute them. An export
keeps every pair whose period overlaps the interval, so its first can start before the interval.

Both frontends use the library for discovery, image loading and updates:

```rust,no_run
use std::path::{Path, PathBuf};
use escapement_verge_probe::{Source, detect, image::Image};
use escapement_verge_probe::usb::UsbBus;

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

## Licence

Apache-2.0, in `LICENSE`.
