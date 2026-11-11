use std::error::Error;
use std::time::{Duration, Instant};

use clap::Parser;
use escapement_verge_probe::probe_rs::probe::list::Lister;
use escapement_verge_probe::{
    CommandError, ProBusError, Refused, SpiMode, VergeProbe, install_cjtag_support, is_mcu_link,
};

fn dio_state(label: &str, state: &escapement_verge_probe::DioState) {
    println!(
        "DIO {label}: status={:?} outputs={:x} levels={:x} read={:x}",
        state.status, state.outputs, state.levels, state.read
    );
}

/// The pins the SPI loopback wire joins, J19 pin 3 (DIO1) and pin 5 (DIO3), each with its partner.
const LOOP: [(u8, u8); 2] = [(0x02, 0x08), (0x08, 0x02)];

fn run_dio_smoke(verge: &mut VergeProbe<'_>, looped: bool) -> Result<(), Box<dyn Error>> {
    let checks: Result<(), Box<dyn Error>> = (|| {
        let initial = verge.access_dio(0, 0, 0)?;
        dio_state("initial", &initial);
        if initial.outputs != 0 {
            return Err("DIO pins were not inputs before the smoke check".into());
        }

        match verge.access_dio(0x07, 0x01, 0x01) {
            Err(CommandError::Firmware(Refused(split))) => {
                dio_state("split-group request", &split);
                if split.outputs != 0 {
                    return Err("DIO changed its state on a refused split request".into());
                }
            }
            Ok(_) => return Err("DIO accepted a split direction group".into()),
            Err(error) => return Err(error.into()),
        }

        // DIO1 and DIO3 are in different direction groups, and the loopback wire joins them, so
        // a group drives only while the other is an input.
        for (label, mask, other) in [("DIO0-DIO2", 0x07, 0x08), ("DIO3", 0x08, 0x07)] {
            verge.access_dio(other, 0, 0)?;
            for level in [mask, 0] {
                let state = verge.access_dio(mask, mask, level)?;
                dio_state(label, &state);
                if state.outputs != mask
                    || state.levels & mask != level
                    || state.read & mask != level
                {
                    return Err(format!("{label} did not read back its output level").into());
                }
                for (driven, partner) in LOOP {
                    let expected = if level & driven != 0 { partner } else { 0 };
                    if looped && mask & driven != 0 && state.read & partner != expected {
                        return Err(format!(
                            "DIO{} did not read DIO{}'s level through the loop",
                            partner.trailing_zeros(),
                            driven.trailing_zeros()
                        )
                        .into());
                    }
                }
            }
        }
        Ok(())
    })();

    let restored = verge.access_dio(0x0f, 0, 0);
    checks?;
    let restored = restored?;
    dio_state("restored inputs", &restored);
    if restored.outputs != 0 {
        return Err("DIO pins were not restored to inputs".into());
    }
    Ok(())
}

fn expect_nack(
    result: Result<Vec<u8>, CommandError<ProBusError>>,
    what: &str,
) -> Result<(), Box<dyn Error>> {
    match result {
        Err(CommandError::Firmware(ProBusError::Nack)) => Ok(()),
        Ok(data) => Err(format!("{what} answered {data:02x?} instead of a NACK").into()),
        Err(error) => Err(format!("{what} failed with {error} instead of a NACK").into()),
    }
}

fn run_eeprom_check(verge: &mut VergeProbe<'_>, address: u8) -> Result<(), Box<dyn Error>> {
    for (rate, clock_hz) in [100_000, 400_000, 1_000_000].into_iter().enumerate() {
        let at = (0x0100 + 0x20 * rate as u16).to_be_bytes();
        let data: Vec<u8> = (0..32u8)
            .map(|k| k.wrapping_mul(37) ^ (rate as u8 + 1))
            .collect();
        let mut write = at.to_vec();
        write.extend_from_slice(&data);
        verge.i2c_transfer(address, clock_hz, &write, 0)?;
        let written = Instant::now();
        expect_nack(
            verge.i2c_transfer(address, clock_hz, &at, 1),
            "a read during the write cycle",
        )?;
        let read = loop {
            match verge.i2c_transfer(address, clock_hz, &at, data.len()) {
                Ok(read) => break read,
                Err(CommandError::Firmware(ProBusError::Nack))
                    if written.elapsed() < Duration::from_millis(50) => {}
                Err(error) => return Err(error.into()),
            }
        };
        let cycle = written.elapsed();
        if read != data {
            return Err(format!("{clock_hz} Hz: read back {read:02x?}, wrote {data:02x?}").into());
        }

        // Reads without an address continue from where the last one stopped.
        verge.i2c_transfer(address, clock_hz, &at, 0)?;
        let mut sequential = verge.i2c_transfer(address, clock_hz, &[], 5)?;
        sequential.extend(verge.i2c_transfer(address, clock_hz, &[], data.len() - 5)?);
        if sequential != data {
            return Err(
                format!("{clock_hz} Hz: reads without an address gave {sequential:02x?}").into(),
            );
        }

        expect_nack(
            verge.i2c_transfer(address ^ 1, clock_hz, &[], 1),
            &format!("address {:#04x}", address ^ 1),
        )?;
        println!(
            "I2C EEPROM at {clock_hz} Hz: page written, NACKed for {cycle:?}, read back, \
             current-address reads continue, {:#04x} NACKed",
            address ^ 1
        );
    }
    Ok(())
}

fn parse_i2c_address(value: &str) -> Result<u8, String> {
    let address = match value.strip_prefix("0x") {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => value.parse(),
    }
    .map_err(|error| error.to_string())?;
    if address > 0x7f {
        return Err("an I2C address has 7 bits".into());
    }
    Ok(address)
}

/// Checks the MCU-Link Pro's bridges through the Verge firmware. Runs only the checks named.
#[derive(Parser)]
struct Args {
    /// The probe's serial number.
    serial: String,
    /// Check DIO direction and readback, one direction group at a time, then return the pins to
    /// inputs.
    #[arg(long)]
    dio: bool,
    /// With --dio, also check each group's levels through J19 pin 3 wired to pin 5.
    #[arg(long, requires = "dio")]
    looped: bool,
    /// Loop two bytes through all four SPI modes. Needs J19 pin 3 (MOSI) wired to pin 5 (MISO).
    #[arg(long)]
    spi: bool,
    /// Read from this 7-bit I2C address, given as 0x50 or 80, at 100 kHz.
    #[arg(long, value_name = "ADDRESS", value_parser = parse_i2c_address)]
    i2c: Option<u8>,
    /// With --i2c, the number of bytes to read.
    #[arg(long, requires = "i2c", default_value_t = 1)]
    i2c_len: usize,
    /// Write, poll and read back a 24C32-style EEPROM (two-byte word address, 32-byte pages) at
    /// this address, at 100 kHz, 400 kHz and 1 MHz. Nothing may answer at the address with its
    /// lowest bit flipped.
    #[arg(long, value_name = "ADDRESS", value_parser = parse_i2c_address)]
    eeprom: Option<u8>,
    /// Print energy samples for five seconds.
    #[arg(long)]
    energy: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let serial = args.serial;

    let info = Lister::new()
        .list_all()
        .into_iter()
        .find(|probe| probe.serial_number.as_deref() == Some(serial.as_str()) && is_mcu_link(probe))
        .ok_or_else(|| std::io::Error::other(format!("no MCU-Link with serial {serial}")))?;
    let mut probe = info.open()?;
    install_cjtag_support(&mut probe)?;
    let mut verge = VergeProbe::new(&mut probe);
    let capabilities = verge
        .info()?
        .cloned()
        .ok_or_else(|| std::io::Error::other("probe does not run Verge firmware"))?;

    println!(
        "{} on {:?}, firmware {}",
        serial, capabilities.board, capabilities.firmware
    );
    if args.dio {
        if !capabilities.supports_dio() {
            return Err("probe does not support DIO".into());
        }
        run_dio_smoke(&mut verge, args.looped)?;
    }
    if args.spi {
        if !capabilities.supports_spi() {
            return Err("probe does not support SPI".into());
        }
        let tx = [0xA5, 0x5A];
        for (mode, name) in [
            (SpiMode::Mode0, "0"),
            (SpiMode::Mode1, "1"),
            (SpiMode::Mode2, "2"),
            (SpiMode::Mode3, "3"),
        ] {
            let result = verge.spi_transfer(100_000, mode, false, &tx)?;
            println!("SPI mode {name} at 100 kHz: RX {:02x?}", result);
            if result != tx {
                return Err(
                    format!("SPI mode {name} loopback mismatch: expected {tx:02x?}").into(),
                );
            }
        }
    }

    if let Some(address) = args.i2c {
        if !capabilities.supports_i2c() {
            return Err("probe does not support I2C".into());
        }
        let result = verge.i2c_transfer(address, 100_000, &[], args.i2c_len)?;
        println!("I2C read at 0x{address:02x}, 100 kHz: RX {:02x?}", result);
    }

    if let Some(address) = args.eeprom {
        if !capabilities.supports_i2c() {
            return Err("probe does not support I2C".into());
        }
        run_eeprom_check(&mut verge, address)?;
    }

    if args.energy {
        if !capabilities.supports_energy_capture() {
            return Err("probe does not support energy capture".into());
        }
        let started = Instant::now();
        verge.monitor_energy(
            1000,
            Duration::from_millis(10),
            || started.elapsed() >= Duration::from_secs(5),
            |sample| println!("energy sample: {sample:?}"),
        )?;
    }

    Ok(())
}
