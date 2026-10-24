//! Image reading held to `objcopy`, which is what the firmware repository's `tools/flash.py` used
//! to turn an ELF into the S-record that `rblhost` flashed.

use std::path::{Path, PathBuf};
use std::process::Command;

use verge_probe::image::{Format, Image};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

#[test]
fn elf_reads_as_objcopy_writes_it() {
    let elf = Image::from_path(&fixture("layout.elf")).unwrap();
    let srec = Image::from_path(&fixture("layout.s19")).unwrap();
    assert_eq!(elf.format, Format::Elf);
    assert_eq!(srec.format, Format::SRecord);
    assert_eq!(elf.chunks, srec.chunks);
    // The layout this fixture exists for: vectors, a gap, code with .data after it, and CPU 1.
    let starts: Vec<u32> = srec.chunks.iter().map(|c| c.address).collect();
    assert_eq!(starts, [0x0, 0x200, 0x40000]);
    assert_eq!(elf.page_runs(), srec.page_runs());
    elf.check_for_mculink().unwrap();
}

/// What `objcopy` makes of a file, read back as an S-record.
fn objcopy_srec(input: &Path, input_format: Option<&str>) -> Image {
    let out = std::env::temp_dir().join(format!(
        "verge-objcopy-{}-{}.s19",
        std::process::id(),
        input.file_name().unwrap().to_string_lossy()
    ));
    let mut command = Command::new("arm-none-eabi-objcopy");
    if let Some(format) = input_format {
        command.args(["-I", format]);
    }
    let status = command
        .args(["-O", "srec"])
        .arg(input)
        .arg(&out)
        .status()
        .expect("arm-none-eabi-objcopy runs");
    assert!(status.success());
    let image = Image::from_path(&out).unwrap();
    std::fs::remove_file(&out).unwrap();
    image
}

/// Our real image, from a firmware build.
#[test]
#[ignore = "needs a firmware build at $VERGE_FIRMWARE_ELF and arm-none-eabi-objcopy"]
fn our_firmware_elf_reads_as_objcopy_writes_it() {
    let path = PathBuf::from(std::env::var("VERGE_FIRMWARE_ELF").expect("VERGE_FIRMWARE_ELF"));
    let elf = Image::from_path(&path).unwrap();
    let srec = objcopy_srec(&path, None);
    assert_eq!(elf.chunks, srec.chunks);
    elf.check_for_mculink().unwrap();
    assert!(
        elf.chunks.iter().any(|c| c.address == 0x40000),
        "no CPU 1 image at 0x40000"
    );
}

/// NXP's image from the local LinkServer install, never copied into this repository.
#[test]
#[ignore = "needs LinkServer's MCU-LINK_CMSIS-DAP_V*.s19 at $VERGE_STOCK_S19"]
fn stock_srecord_reads_as_objcopy_reads_it() {
    let path = PathBuf::from(std::env::var("VERGE_STOCK_S19").expect("VERGE_STOCK_S19"));
    let ours = Image::from_path(&path).unwrap();
    let theirs = objcopy_srec(&path, Some("srec"));
    assert_eq!(ours.chunks, theirs.chunks);
    ours.check_for_mculink().unwrap();
    println!(
        "{}: {} bytes in {} chunks, header {:?}",
        path.display(),
        ours.data_len(),
        ours.chunks.len(),
        ours.header
    );
}
