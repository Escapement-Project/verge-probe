use std::error::Error;
use std::time::{Duration, Instant};

use verge_probe::probe_rs::probe::list::Lister;
use verge_probe::{ProBusStatus, SpiMode, VergeProbe, install_cjtag_support};

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
    let serial = args.next().ok_or(
        "usage: pro_bridges <serial> [--spi] [--i2c <7-bit-address>] [--energy]",
    )?;
    let mut run_spi = false;
    let mut i2c_address = None;
    let mut run_energy = false;

    while let Some(option) = args.next() {
        match option.as_str() {
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

    println!("{} on {:?}, firmware {}", serial, capabilities.board, capabilities.firmware);
    if run_spi {
        if !capabilities.supports_spi() {
            return Err("probe does not support SPI".into());
        }
        let result = verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0xA5])?;
        println!("SPI mode 0 at 100 kHz: {:?}, RX {:02x?}", result.status, result.rx);
        if result.status != ProBusStatus::Ok {
            return Err(format!("SPI transfer returned {:?}", result.status).into());
        }
    }

    if let Some(address) = i2c_address {
        if !capabilities.supports_i2c() {
            return Err("probe does not support I2C".into());
        }
        let result = verge.i2c_transfer(address, 100_000, &[], 1)?;
        println!("I2C read at 0x{address:02x}, 100 kHz: {:?}, RX {:02x?}", result.status, result.rx);
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
