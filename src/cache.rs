//! Release assets kept between runs, so that a release downloaded once is not downloaded again:
//! `<root>/<owner>/<name>/<tag>/<asset>`. A cached asset is only a copy. It is checked every time
//! it is used, as a fresh download is ([`crate::release::GitHub::download`]), so deleting the
//! cache is always safe.

use std::path::{Path, PathBuf};

use crate::release::{Asset, Release, Repo};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Cache {
    root: PathBuf,
}

impl Cache {
    #[must_use]
    pub fn at(root: impl Into<PathBuf>) -> Cache {
        Cache { root: root.into() }
    }

    /// `verge` in the user's cache directory: `~/.cache/verge` on Linux,
    /// `~/Library/Caches/verge` on macOS, `%LOCALAPPDATA%\verge` on Windows. `None` where the
    /// system names none.
    #[must_use]
    pub fn user() -> Option<Cache> {
        dirs::cache_dir().map(|dir| Cache::at(dir.join("verge")))
    }

    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Where the assets of `repo`'s release `tag` are kept.
    #[must_use]
    pub fn release_dir(&self, repo: &Repo, tag: &str) -> PathBuf {
        self.root
            .join(component(&repo.owner))
            .join(component(&repo.name))
            .join(component(tag))
    }

    /// Where the asset `name` of `repo`'s release `tag` is kept.
    #[must_use]
    pub fn path(&self, repo: &Repo, tag: &str, name: &str) -> PathBuf {
        self.release_dir(repo, tag).join(component(name))
    }

    /// The cached copy of `asset`, if there is one of the size the release gives it.
    #[must_use]
    pub fn read(&self, repo: &Repo, tag: &str, asset: &Asset) -> Option<Vec<u8>> {
        let bytes = std::fs::read(self.path(repo, tag, &asset.name)).ok()?;
        (bytes.len() as u64 == asset.size).then_some(bytes)
    }

    /// Keeps a copy of the asset `name`, written whole or not at all.
    pub fn write(&self, repo: &Repo, tag: &str, name: &str, bytes: &[u8]) -> std::io::Result<()> {
        let dir = self.release_dir(repo, tag);
        std::fs::create_dir_all(&dir)?;
        let partial = dir.join(format!(
            ".{}.{}.partial",
            component(name),
            std::process::id()
        ));
        std::fs::write(&partial, bytes)?;
        std::fs::rename(&partial, self.path(repo, tag, name))
    }

    /// Deletes every cached asset of `repo`'s release `tag`.
    pub fn discard(&self, repo: &Repo, tag: &str) -> std::io::Result<()> {
        match std::fs::remove_dir_all(self.release_dir(repo, tag)) {
            Err(error) if error.kind() != std::io::ErrorKind::NotFound => Err(error),
            _ => Ok(()),
        }
    }

    /// Whether every asset `release` carries can be read from the cache, as using it reads them.
    /// Whether they check out is known only once they are used.
    #[must_use]
    pub fn holds(&self, repo: &Repo, release: &Release) -> bool {
        release
            .assets()
            .into_iter()
            .all(|asset| self.read(repo, &release.tag, asset).is_some())
    }
}

/// A name as one path component: anything but letters, digits and `.`, `_`, `+` and `-` is
/// written as `%XX`, and so is a leading dot, so that no tag or asset name can climb out of its
/// directory or collide with a partial write.
fn component(name: &str) -> String {
    let mut out = String::new();
    for (i, byte) in name.bytes().enumerate() {
        let plain = byte.is_ascii_alphanumeric()
            || matches!(byte, b'_' | b'+' | b'-')
            || (byte == b'.' && i > 0);
        if plain {
            out.push(char::from(byte));
        } else {
            out.push_str(&format!("%{byte:02X}"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_name_leaves_its_directory() {
        assert_eq!(component("v0.3.0"), "v0.3.0");
        assert_eq!(
            component("mculink-verge-v0.3.0.s19"),
            "mculink-verge-v0.3.0.s19"
        );
        assert_eq!(component("release/1.0"), "release%2F1.0");
        assert_eq!(component(".."), "%2E.");
        assert_eq!(component(".hidden"), "%2Ehidden");
        assert_eq!(component("a\\b"), "a%5Cb");
        assert_eq!(component("C:x"), "C%3Ax");
    }
}
