use std::error::Error;
use std::time::{Duration, Instant};

use verge_probe::probe_rs::probe::list::Lister;
use verge_probe::{ProBusStatus, SpiMode, VergeProbe, install_cjtag_support};

fn main() -> Result<(), Box<dyn Error>> {
    let info = Lister::new()
        .list_all()
        .into_iter()
        .next()
        .ok_or_else(|| std::io::Error::other("no probe found"))?;
    let mut probe = info.open()?;
    install_cjtag_support(&mut probe)?;
    let mut verge = VergeProbe::new(&mut probe);
    let capabilities = verge
        .info()?
        .cloned()
        .ok_or_else(|| std::io::Error::other("probe does not run Verge firmware"))?;

    if capabilities.supports_target_power() {
        verge.set_target_power(true)?;
    }

    if capabilities.supports_spi() {
        let result = verge.spi_transfer(100_000, SpiMode::Mode0, false, &[0xA5])?;
        if result.status != ProBusStatus::Ok {
            eprintln!("SPI returned {:?}", result.status);
        }
    }

    if capabilities.supports_i2c() {
        let result = verge.i2c_transfer(0x50, 100_000, &[0], 2)?;
        if result.status != ProBusStatus::Ok {
            eprintln!("I2C returned {:?}", result.status);
        }
    }

    if capabilities.supports_energy_capture() {
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
