# The verge-probe library

verge-probe is the Rust library that verge and verge-gui are built on. Everything verge does is a
call or two here: finding MCU-Links, updating their firmware, debugging a target through probe-rs
with Verge's commands beside it, driving the DIO pins, switching the Pro's target supply, running
its SPI and I2C bridges, and reading or recording its energy capture.

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

A line reads like `EVF2551AD5: an MCU-Link Pro, Verge, build d4c4c41`. A probe running NXP's
firmware reads `NXP's firmware V3.174` instead. `unit.verge()` gives the identity of a probe that
answered as Verge, and `None` for any other.

To act on one probe, `pro::find` picks it as verge does. Given a serial, it takes the probe with
that serial. Without one, it takes the only attached probe running Verge on a board with the
feature you ask for. A lone probe attached is taken whatever it runs, and then checked. Its
`FindError` says why it found none: nothing is attached, no probe has that serial, several probes
fit or none does (each with the probes it means), a probe can't give the feature (and why), or USB
failed to list them. `detect::pick` chooses from a list you already have, with a test of your own,
and checks nothing else.

## Updating the firmware

A program updates a probe in two calls. `source::load` gets the image: a Verge release from
GitHub, a file, or NXP's firmware from your LinkServer install. `update::install` puts it on the
probe, the best way the probe's state allows. A probe running Verge, NXP's firmware V3.122 or
later, or daplink-rs restarts into its bootloader when asked. A probe in ISP mode is flashed as it
is.

```rust,no_run
use escapement_verge_probe::cache::Cache;
use escapement_verge_probe::detect::{detect, pick};
use escapement_verge_probe::progress::{Cancel, Quiet};
use escapement_verge_probe::release::{Download, GitHub, Repo, Ureq};
use escapement_verge_probe::source::{self, Source};
use escapement_verge_probe::update::{self, Outcome, Timing};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bus = UsbBus::default();
    let units = detect(&bus)?;
    // The only probe attached; name one by serial where there are more.
    let unit = pick(&units, None, |_| true)?;

    let github = GitHub::new(Ureq::default(), Repo::default(), None)
        .with_cache(Cache::user());
    // The latest release, refused if the probe runs it or a newer one.
    let loaded = source::load(
        &Source::Release(None),
        &github,
        Some(unit),
        false,
        Download::Agreed,
        &mut Quiet,
    )?;

    let cancel = Cancel::new();
    let outcome = update::install(
        &bus,
        unit,
        loaded.payload(),
        &Timing::default(),
        &mut Quiet,
        &cancel,
        &cancel,
    )?;
    match outcome {
        Outcome::Running(unit) => println!("it runs {}", unit.state),
        Outcome::JumperStillFitted => {
            println!("flashed, but back in ISP mode: take the jumper off")
        }
        Outcome::NotRestarted { maybe_jumper } => {
            println!("{}", update::not_restarted(maybe_jumper))
        }
    }
    Ok(())
}
```

`load` checks a Verge image against the probe before anything is sent. It refuses an image that
doesn't declare the probe's board, and a release the probe runs already unless you pass `true` for
`force`. A release is signed, and `load` checks the signature and the checksum. `Download::Agreed`
says the user agreed to the download; `CachedOnly` takes a copy kept from an earlier download, or
fails. `source::load_asking` asks a closure of yours first, and only where a download is needed.
`Quiet` reports no progress. `Progress` is the trait to implement for a bar of your own: it hears
each step, such as erasing and writing, and how far each has got.

The `Cancel` is how another thread stops an update. `install` takes two: the first stops it at the
next safe point, and the second only before anything is sent. `UpdateError::Cancelled` says what a
stop left behind, and its `recovery` says what to do about it.

`offer(unit, &units)` says how a probe can be updated, without changing anything, as `verge list`
shows it. A probe running NXP's firmware older than V3.122 needs the ISP jumper: unplug it, fit the
jumper, and plug it back in. It then comes up in ISP mode.

### ISP mode

A probe in ISP mode runs its ROM bootloader. The `update` module works with it directly:

- `enter_bootloader` restarts a running probe into ISP mode and leaves it there.
- `flash_now` flashes the probe in ISP mode, and restarts it into the new image.
- `reset_now` restarts it into the image it holds, without flashing.
- `rom_info_now` asks the bootloader its version and flash size.

In ISP mode a probe can't say which board it is, so `flash_now` can't check that a Verge image is
built for it. `enter_bootloader` refuses a probe known to be a board other than the standalone
MCU-Link, unless you say NXP's firmware comes next, which goes on any board. Where several probes
are in ISP mode, `flash_now`, `reset_now` and `rom_info_now` take the USB port of the one to use,
as `UsbDevice::location` names it.

```rust,no_run
use std::path::Path;

use escapement_verge_probe::Image;
use escapement_verge_probe::progress::{Cancel, Quiet};
use escapement_verge_probe::update::{self, Timing};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let image = Image::from_path(Path::new("mculink-verge.elf"))?;
    let outcome = update::flash_now(
        &UsbBus::default(),
        &image,
        None,
        &Timing::default(),
        &mut Quiet,
        &Cancel::new(),
    )?;
    println!("{outcome:?}");
    Ok(())
}
```

### Restarting a probe

`update::restart` restarts a probe running Verge, as unplugging it would, and waits for it to come
back. The request goes to the probe's USB core rather than through CMSIS-DAP, so it works while a
debugger holds the probe. `Unit::takes_restart` says whether a probe takes one. Everything the
probe holds is lost: the DIO pins go back to inputs, a Pro's target supply turns off, and an energy
capture ends.

### Many probes in a row

`batch::Batch` updates every probe of a kind as it is plugged in, one at a time, until you stop it.
A `Filter` says which probes it takes. An `Armed` holds the image it puts on them. Your `Observer`
hears each probe start and settle. The batch writes nothing itself: `RecordWriter` writes each
settled probe's `Record` as a line of JSON, as `verge batch` does.

```rust,no_run
use std::num::NonZeroUsize;

use escapement_verge_probe::batch::{
    Armed, Batch, Board, Filter, Observer, OursFilter, Record, StockFilter,
};
use escapement_verge_probe::progress::{Cancel, Progress, Quiet};
use escapement_verge_probe::release::{Download, GitHub, Repo, Ureq};
use escapement_verge_probe::source::{self, Source};
use escapement_verge_probe::update::Timing;
use escapement_verge_probe::usb::UsbBus;

struct Print;

impl Progress for Print {}

impl Observer for Print {
    fn record(&mut self, record: &Record) {
        println!("{}: {}", record.port, record.result.as_str());
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bus = UsbBus::default();
    let github = GitHub::new(Ureq::default(), Repo::default(), None);
    let source = Source::Release(None);
    let loaded = source::load(
        &source,
        &github,
        None,
        true,
        Download::Agreed,
        &mut Quiet,
    )?;
    let armed = Armed::new(loaded, source, github.repo().clone());
    // Standalone MCU-Links running NXP's firmware, or Verge older than
    // the release, from those plugged in from now on.
    let filter = Filter {
        board: Board::McuLink,
        stock: StockFilter::Any,
        ours: OursFilter::Older,
        bootloaders: false,
        attached: false,
    };
    // A stop takes no new probes, and lets the one under way finish.
    let stop = Cancel::new();
    let mut batch = Batch::arm(
        &bus,
        armed,
        filter,
        Timing::default(),
        stop,
        Cancel::new(),
        NonZeroUsize::new(10),
    )?;
    batch.run_to_end(&mut Print)?;
    println!("{}", batch.tally());
    Ok(())
}
```

### Releases and images

`GitHub::latest` and `GitHub::tagged` look up a release without downloading it. `Unit::build` gives
the build a probe running Verge reports, and its `compare` says how it stands against a release's
tag: up to date, the same release built another way, behind, ahead, or unknown.

`Image::from_path` reads an ELF or S-record file without touching any device. It says what goes into
flash and the file's SHA-256. `Image::declaration` finds the record in a Verge image that names the
boards it is built for, and `Image::boards` lists them. `Image::check_for_mculink` says whether it
fits the probe's flash and has a vector table.

```rust,no_run
use escapement_verge_probe::detect;
use escapement_verge_probe::release::{GitHub, Repo, Ureq};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let github = GitHub::new(Ureq::default(), Repo::default(), None);
    let release = github.latest()?;
    println!("{} ({} bytes)", release.tag, release.image.size);
    for unit in detect(&UsbBus::default())? {
        if let Some(build) = unit.build() {
            let serial = unit.serial().unwrap_or("-");
            println!("{serial}: {:?}", build.compare(&release.tag));
        }
    }
    Ok(())
}
```

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

use escapement_verge_probe::dio::DioChange;
use escapement_verge_probe::pro::{self, Feature, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = pro::find(&bus, None, Feature::Supply)?;

    with_pro(&bus, &pro, Feature::Supply, |name, pro| {
        let on = pro.set_target_power(true)?;
        println!(
            "{name}: target supply {}",
            if on { "on" } else { "off" }
        );
        Ok::<_, Box<dyn Error>>(())
    })?;

    with_pro(&bus, &pro, Feature::Dio, |name, pro| {
        // Drive DIO3 high, then make it an input again. The other
        // pins stay as they are.
        let state = pro.change_dio(&DioChange::default().high(0b1000))?;
        println!("{name}: the pins read {:04b}", state.read);
        pro.change_dio(&DioChange::default().input(0b1000))?;
        Ok::<_, Box<dyn Error>>(())
    })?;
    Ok(())
}
```

The supply runs at 3.3 V, or at 1.8 V with J5 fitted, and J6 connects it to the target. Give the
target's rail time to settle before you talk to the target.

A pin set is a mask, bit `k` for DIO`k`. `DioChange` names the pins to drive high, to drive low, and
to make inputs. `change_dio` reads the pins first, and refuses a change the probe would refuse
before it sends it. On a Pro, DIO0 to DIO2 change direction together, and DIO3 on its own. On a
standalone MCU-Link, each pin changes direction on its own, but a bench tool can give pins to its
command marker, which keeps them until the probe restarts. A `DioError` says why: a group the change
would split, pins the marker holds, the Pro's pins turned off by J15, or pins named twice, all found
before anything is sent. Its `Command` variant is the command's own failure, the probe's refusal
among them. An empty change only reads, and still says when the marker holds every pin or J15
turned them off. `dio::names` and `dio::parse_pins` turn masks into names and back, as
`DIO0, DIO3` and `0,dio3`.

`access_dio(mask, outputs, levels)` sends the command as it is: a bit set in `mask` selects a
pin, the same bit in `outputs` makes it an output, and in `levels` drives it high. The pins are
described in [The
Pro](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/pro.md#dio-pins) and [Probes
and
jumpers](https://github.com/Escapement-Project/mculink-verge/blob/HEAD/docs/probes.md#the-standalone-mcu-links-dio-pins).

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
So a tool that starts a capture ends another program's recording. `stream::Reader`, below, joins a
running capture instead.

```rust,no_run
use std::error::Error;
use std::time::{Duration, Instant};

use escapement_verge_probe::EnergyTotal;
use escapement_verge_probe::pro::{self, Feature, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = pro::find(&bus, None, Feature::Energy)?;

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
            "{name}: {} samples a second",
            start.actual_rate_pairs_per_second
        );
        if let EnergyTotal::Total(total) =
            pro.read_energy_accumulator()?
        {
            println!(
                "{} nJ over {} samples",
                total.energy_nanojoules, total.integrated_pairs
            );
        }
        Ok::<_, Box<dyn Error>>(())
    })?;
    Ok(())
}
```

The rate is in voltage and current samples a second, from 1 to 100,000.
`actual_rate_pairs_per_second` gives the rate the probe's timer runs at, to the nearest sample a
second. A reading's current is in nanoamps, positive into the target. Its `sense()` says which
current range read it, and in which setting of the range jumper. The total covers every sample
since the capture started, whatever was polled, and stays readable after the capture stops.

`start_energy_capture_identified`, `latest_energy_sample` and `stop_tracked_energy_capture` do the
same steps one at a time. Call them on the same `pro`: it holds the capture's identity, which the
stop needs. `stop_running_energy_capture` stops whatever capture runs, by its identity, so a
capture another host starts meanwhile runs on. `read_energy_calibration` gives the points each
current range was calibrated at.

## Recording a stream

On a Pro, the probe also streams every sample it measures through Verge Aux, with the DIO pins'
edges and any markers. A board with DIO pins and no energy capture, a standalone MCU-Link or a Pro
without its energy circuit, streams its pins alone.

`stream::Reader` runs a stream from its start to its end. `record::Writer` writes one to a file: a
tickwave recording, which tickwave opens; a sigrok session, which PulseView opens; or CSV.

```rust,no_run
use std::error::Error;
use std::path::Path;
use std::time::{Duration, Instant};

use escapement_verge_probe::pro::{self, Feature};
use escapement_verge_probe::record::{Format, Writer};
use escapement_verge_probe::stream::{EnergyStream, Reader};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let unit = pro::find(&bus, None, Feature::Energy)?;
    let mut reader = Reader::new(pro::open_stream(&bus, &unit)?);

    // A capture at 100,000 samples a second, or the one that runs
    // already, with the edges of all four DIO pins.
    reader.start_energy(&EnergyStream::new(100_000).dio(0b1111))?;
    let path = Path::new("capture.tickwave");
    let mut writer =
        Writer::create(path, Format::Tickwave, reader.context())?;

    // Ten seconds, then the stream ends: a capture the reader
    // started stops, and one it joined runs on.
    let end = Instant::now() + Duration::from_secs(10);
    for message in reader.messages(|| Instant::now() >= end) {
        writer.send(message?)?;
    }
    writer.finish()?;

    let counts = reader.counts();
    println!("{} samples, {} lost", counts.total.pairs, counts.lost);
    Ok(())
}
```

`start_energy` follows a capture that runs already, at its own rate and current ranges, and leaves
it running at the end. With none running, it starts one, which the end stops.
`EnergyStream::join(false)` turns the following off, so the start replaces whatever runs and clears
its total. `Began` says which happened. A start answers before anything is read, so a refused one
leaves nothing to clean up. Make the file after the start, and a refused start leaves an earlier
file as it was.

`messages` gives the stream's messages from its header to its end. Once your closure returns true,
it asks the probe to end the stream, and the probe sends the rest and then the end. It fails when
the probe sends no header within two seconds of the start, or no end within two seconds of being
asked. A header or an end that has come is taken however late you read it. If your tool stops early,
by an error or a panic, dropping the reader ends the stream: a capture the reader started stops, and
the stream of one it joined ends. `abandon` does the same without dropping it. A capture a start
replaced, with joining turned off, does not come back. To read at your own pace, as a window does
between frames, call `read` with a timeout instead, and `end` when you want the stream to end.
`control` sends the probe's other commands meanwhile.

`counts` says what the stream carried: its samples and their energy, samples lost, samples read
under the high range's floor, and each pin's edges. A lost sample is one the probe dropped because
your tool didn't read in time. The probe keeps 16 stream messages your tool hasn't read, two of
them for the stream's header and end. When the other 14 fill, it drops samples, edges and markers
until there is room again, and the recording marks the gap. At 100,000 samples a second, those 14
places hold about 14 ms of samples. Writing a file can take longer than that, so the writer works
on a thread of its own. Its queue has no bound: a disk that falls behind costs memory rather than
the recording.

The writer writes a tickwave recording as it goes, at least once a second, so a crash loses little.
`finish` rewrites it with full-size blocks. If your tool stops before `finish`, `tickwave::compact`
recovers what was written. `tickwave::Recorder` is the tickwave writer alone, with a snapshot of
what is written for a view in the same program. Its queue holds 256 messages. When the disk falls
that far behind, `send` fails the recording rather than let memory grow. `Format::named_by` gives
the format a file's extension names. A sigrok session or CSV records an energy stream only.

`start_dio` starts the DIO stream of a board without energy capture, over
`pro::open_aux(&bus, &unit, Feature::Dio)`. Its first message is a `Message::DioHeader`, its end
says when it ended, and it has no samples or markers. While its pins are quiet it sends a
`Message::DioTime` every 200 ms, which says every edge before that time has arrived. A tickwave
recording of it holds the pins alone. `VergeInfo::supports_dio_stream` says whether the probe has
it.

To mark a moment in the recording, such as the start of a test, send `mark` while the stream runs.
A tool debugging the target calls `mark` on its `VergeSession`. Another program uses `with_pro`,
which goes through CMSIS-DAP while the reader holds Verge Aux:

```rust,no_run
use std::error::Error;

use escapement_verge_probe::Marked;
use escapement_verge_probe::pro::{self, Feature, with_pro};
use escapement_verge_probe::usb::UsbBus;

fn main() -> Result<(), Box<dyn Error>> {
    let bus = UsbBus::default();
    let pro = pro::find(&bus, None, Feature::Energy)?;
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

`attach`, `with_pro`, `open_aux`, `open_stream`, `pro::find`, `change_dio`, `monitor_energy`, the
update functions, `stream::Reader` and `record::Writer` have error types of their own, which say
which step failed.

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
