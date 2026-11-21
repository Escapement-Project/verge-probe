# verge-probe

`verge-probe` is the library that verge's command line and desktop app share. It finds probes and
says what firmware each runs. It sends Verge's debug commands, drives the DIO pins and the SPI and
I2C bridges, and reads energy captures. It restarts probes and sends them into their bootloader. It
reads firmware images and releases, and it updates probes, one at a time or in a batch.

Updates and the ROM bootloader's commands go through the library's `Bus` API. Target debugging goes
through probe-rs. `VergeSession` adds typed Verge commands to an attached probe-rs session, and
leaves that session open for ordinary target access. [docs/library.md](docs/library.md) has
examples for a tool built on the library, and `cargo test --doc` builds them.

Typed commands fail with `CommandError`. It says which of these happened: the connection failed,
the firmware lacks the command, the request was out of range, the reply broke the protocol, or the
firmware refused or failed it. A refusal carries the state the firmware reported. SPI and I2C calls
return the bytes received. The latest energy reading is a sample, no sample yet, or stopped. The
accumulator keeps its total after an acquisition fault too. `set_energy_ranges` picks the current
ranges for the next capture alone: automatic switching, the default, or the high range held. In
either, a sample read on the high range too near the amplifier's floor comes marked under range
(the firmware's docs/host-interface.md, `0x90` mode 14).

Verge Aux, a second vendor interface that debuggers leave alone, carries the identify and DIO
commands on both probes, and the supply, energy and marker commands on a Pro.
[docs/library.md](docs/library.md) shows how to pick a probe, drive its DIO pins and supply, read
energy, and record a stream to a file.

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

`examples/pro_bridges.rs` needs a probe's serial number, and runs the operations its command line
names. It can check GPIO direction and readback. It can test all four SPI modes with a two-byte
MOSI-to-MISO loopback, read one byte from a 7-bit I2C address, or run a cancellable energy monitor
for five seconds. For the SPI check, connect J19 pin 3 (MOSI) to pin 5 (MISO). The example does
not change target power. The monitor runs in the calling thread; UI code should run it in a worker.
The capture keeps running in the firmware between polls, and the probe's accumulator covers the
samples the host did not poll.

```sh
cargo run --example pro_bridges -- EVF2551AD5 --spi
cargo run --example pro_bridges -- EVF2551AD5 --dio
cargo run --example pro_bridges -- EVF2551AD5 --i2c 0x50
cargo run --example pro_bridges -- EVF2551AD5 --energy
```

Use `--i2c` only with a known device on the bridge pins. The example's transaction only reads.

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
