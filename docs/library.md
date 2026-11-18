# The verge-probe library

verge-probe is the Rust library that verge and verge-gui are built on. A tool of your own can use it
to find MCU-Links, debug a target through probe-rs with Verge's commands beside it, drive the DIO
pins, switch the Pro's target supply, run its SPI and I2C bridges, and read or record its energy
capture.

Each example here is a whole program. `cargo test --doc` builds every one of them, so they stay in
step with the library. [Host
interface](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/host-interface.md) has
the commands underneath, for a tool in another language.

## Adding it

```toml
[dependencies.escapement-verge-probe]
git = "https://github.com/Escapement-Project/verge-probe"
features = ["tickwave"]
```

The crate's name in code is `escapement_verge_probe`. The `tickwave` feature adds writing and
exporting `.tickwave` recordings, the format of tickwave, the project's viewer for captures. Leave
it out if you don't use them.

verge-probe needs a probe-rs with CMSIS-DAP's vendor commands and cJTAG, which upstream probe-rs
doesn't have. It pins one, and hands it on as `escapement_verge_probe::probe_rs`. Use that one
rather than adding probe-rs yourself. A second probe-rs is a different crate, and its types don't
fit verge-probe's.

## Finding the probes

`detect` lists every MCU-Link on USB and says what each one runs. It opens each probe only to ask,
and changes nothing.

```rust,no_run
use escapement_verge_probe::detect;
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    for unit in detect(&UsbBus::default())? {
        let serial = unit.serial().unwrap_or("no serial");
        println!("{serial}: {}, {}", unit.hardware(), unit.state);
    }
    Ok(())
}
```

A line reads like `CJF2551AD5: an MCU-Link Pro, Verge, build d4c4c41`. A probe running NXP's
firmware reads `stock V3.174` instead. `unit.verge()` gives the identity of a probe that answered as
Verge, and `None` for any other.

## Debugging, with Verge's commands beside it

`VergeSession` wraps a probe-rs session and sends Verge's commands through it. Its `session()` hands
the session back, for everything probe-rs does. Send Verge's commands only to a probe `is_mcu_link`
accepts. Other firmware can read their numbers as commands of its own: DAPLink's `0x82` sets its
UART.

```rust,no_run
use escapement_verge_probe::probe_rs::probe::WireProtocol;
use escapement_verge_probe::probe_rs::probe::list::Lister;
use escapement_verge_probe::probe_rs::{MemoryInterface, Permissions};
use escapement_verge_probe::{VergeSession, is_mcu_link};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // probe-rs's name for the target, such as MSPM0L1306.
    let chip =
        std::env::args().nth(1).ok_or("name the target's chip")?;
    let info = Lister::new()
        .list_all()
        .into_iter()
        .find(is_mcu_link)
        .ok_or("no MCU-Link found")?;
    let mut probe = info.open()?;
    probe.select_protocol(WireProtocol::Swd)?;
    let mut session =
        probe.attach(chip.as_str(), Permissions::default())?;
    let mut verge = VergeSession::new(&mut session);

    let sense = verge.read_target_sense()?;
    if sense.detected {
        println!("target at {} mV", sense.millivolts);
    } else {
        println!("nothing plugged into the debug connector");
    }

    let mut core = verge.session().core(0)?;
    let cpuid = core.read_word_32(0xE000_ED00)?;
    println!("CPUID {cpuid:#010x}");
    Ok(())
}
```

`read_target_sense` reads the voltage the target supplies on the debug connector's pin 1, and
whether a target grounds pin 9. The voltage alone doesn't tell an empty connector from a powered
target, so check `detected`.

`attach` opens a probe and attaches in one call: the probe with a serial number, or the only one
attached, over the protocol you name, at an optional clock in kHz. It takes any probe for SWD and
JTAG, so check it with `is_mcu_link` before you send Verge's commands. For cJTAG it refuses a probe
that isn't an MCU-Link running Verge, and brings the target up in the scan format you name, or
OScan1. To bring a cJTAG target up in OScan2 at 1 MHz:

```rust,no_run
use escapement_verge_probe::attach;
use escapement_verge_probe::probe_rs::probe::WireProtocol;
use escapement_verge_probe::probe_rs::probe::cmsisdap::ScanFormat;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let chip =
        std::env::args().nth(1).ok_or("name the target's chip")?;
    let session = attach(
        None,
        &chip,
        WireProtocol::CJtag,
        Some(1_000),
        Some(ScanFormat::OScan2),
    )?;
    println!("attached to {}", session.target().name);
    Ok(())
}
```

Nothing checks that the target supports the format you ask for. A TI target needs more than this
before it answers: [cJTAG targets](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/cjtag-targets.md).

## The DIO pins and the Pro's supply

The DIO commands also go through Verge Aux, a second vendor interface. On a Pro, so do the supply
and energy commands. A debugger leaves Verge Aux alone, so a tool can use it while a debugger holds
the probe. `pro::with_pro` runs your code through Verge Aux, or through CMSIS-DAP when another
program holds Verge Aux. It refuses a probe that isn't running Verge, whose board lacks the feature
or whose firmware does, with an error that says which. `pro::open_aux` opens Verge Aux alone, for a
tool that keeps it open beside a debugger.

```rust,no_run
use std::error::Error;

use escapement_verge_probe::detect;
use escapement_verge_probe::pro::{Feature, validate, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = detect(&bus)?
        .into_iter()
        .find(|unit| validate(unit, Feature::Supply).is_ok())
        .ok_or("no MCU-Link Pro running Verge")?;

    with_pro(&bus, &pro, Feature::Supply, |name, pro| {
        let on = pro.set_target_power(true)?;
        println!(
            "{name}: target supply {}",
            if on { "on" } else { "off" }
        );
        Ok::<_, Box<dyn Error>>(())
    })?;

    with_pro(&bus, &pro, Feature::Dio, |name, pro| {
        // Make DIO3 an output and drive it high, then make it an input
        // again. The other pins stay as they are.
        let state = pro.access_dio(0b1000, 0b1000, 0b1000)?;
        println!("{name}: the pins read {:04b}", state.read);
        pro.access_dio(0b1000, 0, 0)?;
        Ok::<_, Box<dyn Error>>(())
    })?;
    Ok(())
}
```

The supply runs at 3.3 V, or at 1.8 V with J5 fitted, and J6 connects it to the target. Give the
target's rail time to settle before you talk to the target.

`access_dio(mask, outputs, levels)` changes the pins whose bits are set in `mask`: a bit set in
`outputs` makes that pin an output, and the same bit in `levels` drives it high. A zero mask only
reads. On a Pro, DIO0 to DIO2 change direction together, and DIO3 on its own. On a standalone
MCU-Link, each pin changes direction on its own. `groups` in the state lists the pins that change
together, and the probe refuses a change that splits a group. Its `Refused` error carries the pins'
state, which the refusal left unchanged. The pins are described in [The
Pro](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/pro.md#dio-pins) and [Probes
and jumpers](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/probes.md#the-mcu-links-dio-pins).

The supply and the pins keep their state when your tool exits, until something changes them or the
probe restarts. A pin left an output keeps the SPI bridge from running.

## SPI and I2C

The bridges run through CMSIS-DAP only: Verge Aux doesn't carry them. `VergeProbe` sends Verge's
commands to a probe that probe-rs opened, before it attaches to a target. This reads a SPI flash's
JEDEC ID, then two bytes from an I2C EEPROM at address 0x50.

```rust,no_run
use escapement_verge_probe::probe_rs::probe::list::Lister;
use escapement_verge_probe::{SpiMode, VergeProbe, is_mcu_link};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let info = Lister::new()
        .list_all()
        .into_iter()
        .find(is_mcu_link)
        .ok_or("no MCU-Link found")?;
    let mut probe = info.open()?;
    let mut verge = VergeProbe::new(&mut probe);

    // One transaction at 1 MHz, mode 0, most significant bit first.
    // The reply has as many bytes as the request.
    let id = verge.spi_transfer(
        1_000_000,
        SpiMode::Mode0,
        false,
        &[0x9F, 0, 0, 0],
    )?;
    println!("JEDEC ID {:02x?}", &id[1..]);

    // Write the address 0, then read two bytes, at 100 kHz.
    let bytes = verge.i2c_transfer(0x50, 100_000, &[0x00], 2)?;
    println!("EEPROM {bytes:02x?}");
    Ok(())
}
```

SPI runs on the DIO pins, so the probe refuses it while any DIO pin is an output. I2C shares its
peripheral with SWO, so the probe refuses it while SWO is set up. [The
Pro](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/pro.md#spi-and-i2c-bridges)
has the pins and the clock ranges. [`examples/pro_bridges.rs`](../examples/pro_bridges.rs) checks
both bridges and the DIO pins on a Pro of your own. Its SPI check needs J19 pin 3 wired to pin 5,
and its I2C check a device you know is on the bus.

## Reading energy

`monitor_energy` starts a capture, hands your code each new reading it polls, and stops the capture
when your code says so. Once the start has answered, it stops the capture on every way out, a panic
included. The probe keeps a running total of energy between polls, so polling slowly loses nothing
from the total.

A start replaces any capture already running, and clears its total, whichever program started it.
So a tool that starts a capture ends another program's recording. verge joins a running capture
instead, with `stream_energy_capture_identified`.

```rust,no_run
use std::error::Error;
use std::time::{Duration, Instant};

use escapement_verge_probe::EnergyTotal;
use escapement_verge_probe::detect;
use escapement_verge_probe::pro::{Feature, validate, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = detect(&bus)?
        .into_iter()
        .find(|unit| validate(unit, Feature::Energy).is_ok())
        .ok_or("no MCU-Link Pro running Verge")?;

    with_pro(&bus, &pro, Feature::Energy, |name, pro| {
        let end = Instant::now() + Duration::from_secs(5);
        let start = pro.monitor_energy(
            25_000,
            Duration::from_millis(250),
            || Instant::now() >= end,
            |sample| {
                if let Some(reading) = sample.calibrated {
                    println!(
                        "{} mV, {} nA",
                        reading.voltage_millivolts,
                        reading.current_nanoamps
                    );
                }
            },
        )?;
        println!(
            "{name}: {} pairs a second",
            start.actual_rate_pairs_per_second
        );
        if let EnergyTotal::Total(total) =
            pro.read_energy_accumulator()?
        {
            println!(
                "{} nJ over {} pairs",
                total.energy_nanojoules, total.integrated_pairs
            );
        }
        Ok::<_, Box<dyn Error>>(())
    })?;
    Ok(())
}
```

The rate is in voltage and current pairs a second, from 1 to 100,000. `actual_rate_pairs_per_second`
gives the rate the probe's timer runs at, to the nearest pair a second. A reading's current is in
nanoamps, positive into the target. The total covers every pair since the capture started, whatever
was polled, and stays readable after the capture stops.

`start_energy_capture_identified`, `latest_energy_sample` and `stop_tracked_energy_capture` do the
same steps one at a time. Call them on the same `pro`: it holds the capture's identity, which the
stop needs.

## Recording energy to a file

On a Pro, the probe also streams every pair it measures, with the DIO pins' edges and any markers,
through Verge Aux. `pro::open_stream` opens it. `tickwave::Recorder` turns the stream into a
`.tickwave` recording, which tickwave opens. `tickwave::export` writes all or part of one, opened as
a `tickwave::view::Recording`, as CSV or a sigrok session.

```rust,no_run
use std::error::Error;
use std::path::Path;
use std::time::{Duration, Instant};

use escapement_verge_probe::VergeControl;
use escapement_verge_probe::aux::{Aux, AuxLink};
use escapement_verge_probe::detect;
use escapement_verge_probe::pro::{Feature, open_stream, validate};
use escapement_verge_probe::stream::Message;
use escapement_verge_probe::tickwave::{Context, Recorder};
use escapement_verge_probe::usb::UsbBus;

/// How long the probe gets to send the stream's header, and its end.
const WAIT: Duration = Duration::from_secs(2);

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let unit = detect(&bus)?
        .into_iter()
        .find(|unit| validate(unit, Feature::Energy).is_ok())
        .ok_or("no MCU-Link Pro running Verge")?;
    let mut aux = open_stream(&bus, &unit)?;
    let mut pro = VergeControl::new(&mut aux);

    // Pairs at 100,000 a second, and the edges of all four DIO pins.
    pro.start_energy_stream_identified(100_000, 0b1111)?;
    let recorded = record(&mut pro, Path::new("capture.tickwave"));
    if recorded.is_err() {
        // The capture this started stops, whatever went wrong.
        let _ = pro.stop_tracked_energy_capture();
    }
    recorded
}

/// Ten seconds of the stream into `path`, then the stop and the rest.
fn record<L: AuxLink>(
    pro: &mut VergeControl<'_, Aux<L>>,
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    // The file is written on a thread of its own, so this one reads
    // the stream as fast as the probe sends it.
    let mut recorder = Recorder::create(path, Context::default())?;

    let start = Instant::now();
    let mut began = false;
    let mut stopped = None;
    loop {
        let now = Instant::now();
        if !began && now - start > WAIT {
            return Err("the probe sent no stream header".into());
        }
        if stopped.is_none() && now - start >= Duration::from_secs(10) {
            // The probe ends the stream after the capture's last pair.
            pro.stop_tracked_energy_capture()?;
            stopped = Some(now);
        }
        if stopped.is_some_and(|at| now - at > WAIT) {
            return Err("the probe didn't end the stream".into());
        }
        let Some(message) =
            pro.transport().message(Duration::from_millis(100))?
        else {
            continue;
        };
        if let Message::Header(header) = &message {
            pro.accept_energy_header(header)?;
            began = true;
        }
        let ended = began && matches!(message, Message::End(_));
        recorder.send(message)?;
        if ended {
            break;
        }
    }
    recorder.finish()?;
    Ok(())
}
```

The probe keeps 16 stream messages your tool hasn't read, two of them for the stream's header and
end. When the rest fill, it drops pairs, edges and markers until there is room again, and the
recording marks the gap. At 100,000 pairs a second, the other 14 places hold about 14 ms of pairs.
Writing the file can take longer than that whenever a block is written out. So `Recorder` writes
the file on a thread of its own, as verge does. Its queue holds 256 messages, and `send` fails the
recording rather than drop one when the disk falls that far behind.

The recorder writes the recording as it goes, at least once a second, so a crash loses little.
`finish` rewrites it with full-size blocks. If your tool stops before `finish`, `tickwave::compact`
recovers what was written. `tickwave::FileWriter` does the same work on the calling thread. A
capture your tool started keeps running when your tool fails, so the example stops it on every way
out once the start has answered.

A board with DIO pins and no energy capture, a standalone MCU-Link or a Pro without its energy
circuit, streams the pins alone. `start_dio_stream` starts it over
`pro::open_aux(&bus, &unit, Feature::Dio)`, and `end_dio_stream` ends it. Its first message is a
`Message::DioHeader`, its end says when it ended, and it has no pairs or markers. While its pins
are quiet it sends a `Message::DioTime` every 200 ms: every edge before that time has arrived.
`tickwave::FileWriter` records it as a `verge-dio` recording of the pins alone.
`VergeInfo::supports_dio_stream` says whether the probe has it.

To mark a moment in the recording, such as the start of a test, send `mark` while the stream runs.
A tool debugging the target calls `mark` on its `VergeSession`. Another program uses `with_pro`,
which goes through CMSIS-DAP while the recorder holds Verge Aux:

```rust,no_run
use std::error::Error;

use escapement_verge_probe::Marked;
use escapement_verge_probe::detect;
use escapement_verge_probe::pro::{Feature, validate, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = detect(&bus)?
        .into_iter()
        .find(|unit| validate(unit, Feature::Energy).is_ok())
        .ok_or("no MCU-Link Pro running Verge")?;
    with_pro(&bus, &pro, Feature::Energy, |name, pro| {
        match pro.mark(1, b"test starts")? {
            Marked::Sent => println!("{name}: marked"),
            other => println!("{name}: not marked: {other:?}"),
        }
        Ok::<_, Box<dyn Error>>(())
    })?;
    Ok(())
}
```

A marker carries a number and a label of up to 32 bytes. The probe stamps it with the time it runs
the command. `Marked` says when a marker went nowhere: no capture runs, no stream is open, or the
stream had no room.

## When a command fails

The commands of `VergeProbe`, `VergeSession` and `with_pro`'s `pro` fail with a `CommandError`,
which says what went wrong:

| Variant | What happened |
| --- | --- |
| `Probe` | The probe or its USB connection failed. |
| `Unsupported` | The probe doesn't offer the command: its firmware predates it, its board has none, or a jumper turned it off. |
| `InvalidRequest` | The request is outside what the command takes. The command wasn't sent. |
| `Malformed` | The reply doesn't follow the command's protocol. |
| `Firmware` | The firmware refused or failed the command, with its reason. |

`attach`, `with_pro`, `open_aux`, `open_stream` and `monitor_energy` have error types of their
own, which say which step failed.

A refusal carries the state the firmware reported. The probe refuses a DIO change that splits a
direction group, for one, and its error holds the pins as they stayed:

```rust,no_run
use escapement_verge_probe::probe_rs::probe::list::Lister;
use escapement_verge_probe::{
    CommandError, Refused, VergeProbe, is_mcu_link,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let info = Lister::new()
        .list_all()
        .into_iter()
        .find(is_mcu_link)
        .ok_or("no MCU-Link found")?;
    let mut probe = info.open()?;
    let mut verge = VergeProbe::new(&mut probe);
    // DIO0 alone, though DIO1 and DIO2 share its direction.
    match verge.access_dio(0b0001, 0b0001, 0b0001) {
        Ok(state) => println!("the pins read {:04b}", state.read),
        Err(CommandError::Firmware(Refused(state))) => {
            println!("refused; the outputs stay {:04b}", state.outputs)
        }
        Err(error @ CommandError::Unsupported(_)) => {
            println!("{error}")
        }
        Err(other) => return Err(other.into()),
    }
    Ok(())
}
```
