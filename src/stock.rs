//! Stock images, from the user's own LinkServer install. NXP's image is not ours to redistribute,
//! so the tool never bundles or downloads it.

use std::path::{Path, PathBuf};

use crate::version::StockVersion;

/// A stock image file and the version its name gives.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StockImage {
    pub path: PathBuf,
    pub version: StockVersion,
}

/// The version in a LinkServer file name such as `MCU-LINK_CMSIS-DAP_V3_172.s19`.
///
/// Only the full MCU-Link's image matches; the mini's (`MCU-MINI_...`) and the J-Link images do
/// not.
#[must_use]
pub fn version_of(file_name: &str) -> Option<StockVersion> {
    let rest = file_name
        .strip_prefix("MCU-LINK_CMSIS-DAP_V")?
        .strip_suffix(".s19")?;
    let (major, minor) = rest.split_once('_')?;
    Some(StockVersion {
        major: major.parse().ok()?,
        minor: minor.parse().ok()?,
    })
}

/// Where LinkServer keeps probe firmware, for each install found.
///
/// LinkServer installs as `LinkServer_<version>` (or `linkserver`) under one of a few roots, with
/// the images in `MCU-LINK_installer/probe_firmware`; some installs keep them in
/// `MCU-LINK/probe_firmware` instead.
#[must_use]
pub fn search_dirs() -> Vec<PathBuf> {
    let roots: &[&str] = if cfg!(windows) {
        &["C:\\NXP"]
    } else if cfg!(target_os = "macos") {
        &["/Applications"]
    } else {
        &["/opt", "/usr/local"]
    };
    let mut dirs = Vec::new();
    for root in roots {
        let Ok(entries) = std::fs::read_dir(root) else {
            continue;
        };
        let mut installs: Vec<PathBuf> = entries
            .filter_map(Result::ok)
            .filter(|entry| {
                entry
                    .file_name()
                    .to_string_lossy()
                    .to_ascii_lowercase()
                    .starts_with("linkserver")
            })
            .map(|entry| entry.path())
            .collect();
        installs.sort();
        for install in installs {
            for sub in ["MCU-LINK_installer", "MCU-LINK"] {
                let dir = install.join(sub).join("probe_firmware");
                if dir.is_dir() {
                    dirs.push(dir);
                }
            }
        }
    }
    dirs
}

fn images(dirs: &[PathBuf]) -> impl Iterator<Item = StockImage> + '_ {
    dirs.iter()
        .filter_map(|dir| std::fs::read_dir(dir).ok())
        .flatten()
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let version = version_of(&entry.file_name().to_string_lossy())?;
            Some(StockImage {
                path: entry.path(),
                version,
            })
        })
}

fn newest_first(a: &StockImage, b: &StockImage) -> std::cmp::Ordering {
    b.version.cmp(&a.version).then_with(|| a.path.cmp(&b.path))
}

/// Every stock image in `dirs`, newest version first.
#[must_use]
pub fn find(dirs: &[PathBuf]) -> Vec<StockImage> {
    let mut images: Vec<_> = images(dirs).collect();
    images.sort_by(newest_first);
    images
}

/// The newest image in `dirs`, choosing the first path alphabetically on a version tie.
#[must_use]
pub fn newest_from(dirs: &[PathBuf]) -> Option<StockImage> {
    images(dirs).min_by(newest_first)
}

/// The newest stock image in LinkServer's usual places.
#[must_use]
pub fn newest() -> Option<StockImage> {
    newest_from(&search_dirs())
}

/// The stock image in a directory the user named.
#[must_use]
pub fn newest_in(dir: &Path) -> Option<StockImage> {
    newest_from(&[dir.to_path_buf()])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn newest_uses_the_version_then_the_path() {
        let root = tempfile::tempdir().unwrap();
        let dirs = [root.path().join("z"), root.path().join("a")];
        for dir in &dirs {
            std::fs::create_dir(dir).unwrap();
            for version in ["3_99", "3_172"] {
                std::fs::write(dir.join(format!("MCU-LINK_CMSIS-DAP_V{version}.s19")), b"")
                    .unwrap();
            }
        }
        assert_eq!(
            newest_from(&dirs).unwrap().path,
            dirs[1].join("MCU-LINK_CMSIS-DAP_V3_172.s19")
        );
        assert_eq!(newest_from(&[]), None);
    }

    #[test]
    fn names() {
        assert_eq!(
            version_of("MCU-LINK_CMSIS-DAP_V3_172.s19"),
            Some(StockVersion {
                major: 3,
                minor: 172
            })
        );
        assert_eq!(version_of("MCU-MINI_CMSIS-DAP_V3_172.s19"), None);
        assert_eq!(version_of("Firmware_J-Link-MCU-Link_240702.s19"), None);
        assert_eq!(version_of("MCU-LINK_CMSIS-DAP_V3_172.bin"), None);
    }

    #[test]
    fn newest_first() {
        let dir = std::env::temp_dir().join(format!("verge-stock-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for name in [
            "MCU-LINK_CMSIS-DAP_V3_99.s19",
            "MCU-LINK_CMSIS-DAP_V3_172.s19",
            "MCU-LINK_CMSIS-DAP_V3_122.s19",
            "MCU-MINI_CMSIS-DAP_V3_200.s19",
        ] {
            std::fs::write(dir.join(name), b"").unwrap();
        }
        let found = find(std::slice::from_ref(&dir));
        std::fs::remove_dir_all(&dir).unwrap();
        let versions: Vec<String> = found
            .iter()
            .map(|image| image.version.to_string())
            .collect();
        assert_eq!(versions, ["3.172", "3.122", "3.99"]);
    }
}
