use std::error::Error;
use std::time::{Duration, Instant};

use verge_probe::probe_rs::probe::list::Lister;
use verge_probe::{ProBusStatus, SpiMode, VendorStatus, VergeProbe, install_cjtag_support};

fn dio_state(label: &str, state: &verge_probe::DioState) {
    println!(
        "DIO {label}: status={:?} outputs={:x} levels={:x} read={:x}",
        state.status, state.outputs, state.levels, state.read
    );
}

fn run_dio_smoke(verge: &mut VergeProbe<'_>) -> Result<(), Box<dyn Error>> {
    let checks: Result<(), Box<dyn Error>> = (|| {
        let initial = verge.access_dio(0, 0, 0)?;
        dio_state("initial", &initial);
        if initial.status != VendorStatus::Ok || initial.outputs != 0 {
            return Err("DIO pins were not inputs before the smoke check".into());
        }

        let split = verge.access_dio(0x07, 0x01, 0x01)?;
        dio_state("split-group request", &split);
        if split.status != VendorStatus::Error || split.outputs != 0 {
            return Err("DIO accepted a split direction group or changed its state".into());
        }

        for (label, mask) in [("DIO0-DIO2", 0x07), ("DIO3", 0x08)] {
            for level in [mask, 0] {
                let state = verge.access_dio(mask, mask, level)?;
                dio_state(label, &state);
                if state.status != VendorStatus::Ok
                    || state.outputs & mask != mask
                    || state.levels & mask != level
                    || state.read & mask != level
                {
                    return Err(format!("{label} did not read back its output level").into());
                }
            }
        }
        Ok(())
    })();

    let restored = verge.access_dio(0x0f, 0, 0);
    checks?;
    let restored = restored?;
    dio_state("restored inputs", &restored);
    if restored.status != VendorStatus::Ok || restored.outputs != 0 {
        return Err("DIO pins were not restored to inputs".into());
    }
    Ok(())
}

fn parse_i2c_address(value: &str) -> Result<u8, Box<dyn Error>> {
    let address = if let Some(hex) = value.strip_prefix("0x") {
        u8::from_str_radix(hex, 16)?
    } else {
        value.parse()?
    };
    if address > 0x7f {
        return Err("I2C address must be a 7-bit address".into());
    }
    Ok(address)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let serial = args
        .next()
        .ok_or("usage: pro_bridges <serial> [--dio] [--spi] [--i2c <7-bit-address>] [--energy]")?;
    let mut run_dio = false;
    let mut run_spi = false;
    let mut i2c_address = None;
    let mut run_energy = false;

    while let Some(option) = args.next() {
        match option.as_str() {
            "--dio" => run_dio = true,
            "--spi" => run_spi = true,
            "--i2c" => {
                let value = args.next().ok_or("--i2c requires a 7-bit address")?;
                i2c_address = Some(parse_i2c_address(&value)?);
            }
            "--energy" => run_energy = true,
            _ => return Err(format!("unknown option: {option}").into()),
        }
    }

    let info = Lister::new()
        .list_all()
        .into_iter()
        .find(|probe| probe.serial_number.as_deref() == Some(serial.as_str()))
        .ok_or_else(|| std::io::Error::other(format!("probe {serial} not found")))?;
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
    if run_dio {
        if !capabilities.supports_dio() {
            return Err("probe does not support DIO".into());
        }
        run_dio_smoke(&mut verge)?;
    }
    if run_spi {
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
            println!(
                "SPI mode {name} at 100 kHz: {:?}, RX {:02x?}",
                result.status, result.rx
            );
            if result.status != ProBusStatus::Ok {
                return Err(format!("SPI mode {name} returned {:?}", result.status).into());
            }
            if result.rx != tx {
                return Err(
                    format!("SPI mode {name} loopback mismatch: expected {tx:02x?}").into(),
                );
            }
        }
    }

    if let Some(address) = i2c_address {
        if !capabilities.supports_i2c() {
            return Err("probe does not support I2C".into());
        }
        let result = verge.i2c_transfer(address, 100_000, &[], 1)?;
        println!(
            "I2C read at 0x{address:02x}, 100 kHz: {:?}, RX {:02x?}",
            result.status, result.rx
        );
        if result.status != ProBusStatus::Ok {
            return Err(format!("I2C transfer returned {:?}", result.status).into());
        }
    }

    if run_energy {
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
