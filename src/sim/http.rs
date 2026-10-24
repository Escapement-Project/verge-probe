//! A simulated GitHub, serving releases the way the REST API does.
//!
//! The JSON carries the fields the API returns that the tool reads, plus a few it ignores. The
//! tests hold the parser to a recorded reply as well (`tests/fixtures/github-release-probe-rs.json`).
//!
//! Its releases are signed with a key pair it generates when it is made, so no secret key exists
//! anywhere but in memory. A [`GitHub`](crate::release::GitHub) reading it needs that key's
//! public half, from [`SimHttp::public_key`].

use std::sync::{Arc, Mutex};

use serde_json::json;
use sha2::{Digest, Sha256};

use super::{SimFirmware, srecord};
use crate::release::{Http, HttpError, Repo};

#[derive(Clone, Debug)]
pub struct SimAsset {
    pub name: String,
    pub bytes: Vec<u8>,
    /// What GitHub reports as the asset's digest; `None` leaves the field out.
    pub digest: Option<String>,
}

impl SimAsset {
    #[must_use]
    pub fn new(name: &str, bytes: Vec<u8>) -> SimAsset {
        let digest = Some(format!("sha256:{}", hex::encode(Sha256::digest(&bytes))));
        SimAsset {
            name: name.to_owned(),
            bytes,
            digest,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimRelease {
    pub tag: String,
    pub prerelease: bool,
    pub assets: Vec<SimAsset>,
}

struct Inner {
    repo: Repo,
    private: bool,
    releases: Vec<SimRelease>,
    requests: Vec<String>,
}

/// A minisign key pair made for a simulated GitHub or a test, and never written anywhere.
pub struct SimKey(minisign::KeyPair);

impl SimKey {
    #[must_use]
    pub fn generate() -> SimKey {
        SimKey(
            minisign::KeyPair::generate_unencrypted_keypair().expect("the system has randomness"),
        )
    }

    /// The public half, as the tool takes a key.
    #[must_use]
    pub fn public_key(&self) -> minisign_verify::PublicKey {
        minisign_verify::PublicKey::from_base64(&self.0.pk.to_base64())
            .expect("minisign writes keys minisign-verify reads")
    }

    /// A `.minisig` asset for `bytes`, with `trusted_comment` signed along with them.
    #[must_use]
    pub fn sign(&self, name: &str, bytes: &[u8], trusted_comment: &str) -> SimAsset {
        let signature = minisign::sign(
            Some(&self.0.pk),
            &self.0.sk,
            std::io::Cursor::new(bytes),
            Some(trusted_comment),
            None,
        )
        .expect("an unencrypted key signs");
        SimAsset::new(name, signature.into_string().into_bytes())
    }
}

#[derive(Clone)]
pub struct SimHttp {
    inner: Arc<Mutex<Inner>>,
    key: Arc<SimKey>,
}

impl SimHttp {
    #[must_use]
    pub fn new(repo: Repo) -> SimHttp {
        SimHttp {
            inner: Arc::new(Mutex::new(Inner {
                repo,
                private: false,
                releases: Vec::new(),
                requests: Vec::new(),
            })),
            key: Arc::new(SimKey::generate()),
        }
    }

    fn inner(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.inner
            .lock()
            .expect("the simulated GitHub lock is not poisoned")
    }

    /// The key this GitHub's releases are signed with.
    #[must_use]
    pub fn key(&self) -> &SimKey {
        &self.key
    }

    /// The public half of [`SimHttp::key`], for [`GitHub::with_key`](crate::release::GitHub::with_key).
    #[must_use]
    pub fn public_key(&self) -> minisign_verify::PublicKey {
        self.key.public_key()
    }

    /// The image, checksum and signature assets a release of ours carries, by the convention in
    /// [`crate::release`]. Its image declares no boards.
    #[must_use]
    pub fn ours_assets(&self, tag: &str) -> Vec<SimAsset> {
        self.ours_assets_declaring(tag, None)
    }

    /// [`SimHttp::ours_assets`], with an image that declares `boards` where they are given.
    #[must_use]
    pub fn ours_assets_declaring(&self, tag: &str, boards: Option<u32>) -> Vec<SimAsset> {
        let image = srecord(
            &SimFirmware::Ours {
                build: tag.to_owned(),
            },
            boards,
        )
        .into_bytes();
        let name = format!("mculink-verge-{tag}.s19");
        let sums = format!("{}  {name}\n", hex::encode(Sha256::digest(&image)));
        let signature = self.key.sign(
            &format!("{name}.minisig"),
            &image,
            &format!("file:{name} tag:{tag}"),
        );
        vec![
            SimAsset::new(&name, image),
            SimAsset::new(&format!("{name}.sha256"), sums.into_bytes()),
            signature,
        ]
    }

    /// A repository that answers only requests carrying a token.
    pub fn set_private(&self, private: bool) {
        self.inner().private = private;
    }

    pub fn publish(&self, release: SimRelease) {
        self.inner().releases.push(release);
    }

    /// Publishes a release of ours that follows the convention, and declares no boards.
    pub fn publish_ours(&self, tag: &str) {
        self.publish_ours_declaring(tag, None);
    }

    /// Publishes a release of ours whose image declares `boards` where they are given.
    pub fn publish_ours_declaring(&self, tag: &str, boards: Option<u32>) {
        self.publish(SimRelease {
            tag: tag.to_owned(),
            prerelease: false,
            assets: self.ours_assets_declaring(tag, boards),
        });
    }

    /// Every URL asked for, in order.
    #[must_use]
    pub fn requests(&self) -> Vec<String> {
        self.inner().requests.clone()
    }

    fn release_json(repo: &Repo, release: &SimRelease, index: usize) -> serde_json::Value {
        let base = format!("https://api.github.com/repos/{}/{}", repo.owner, repo.name);
        let assets: Vec<serde_json::Value> = release
            .assets
            .iter()
            .enumerate()
            .map(|(n, asset)| {
                json!({
                    "url": format!("{base}/releases/assets/{}", index * 100 + n),
                    "id": index * 100 + n,
                    "name": asset.name,
                    "label": "",
                    "content_type": "application/octet-stream",
                    "state": "uploaded",
                    "size": asset.bytes.len(),
                    "digest": asset.digest,
                    "browser_download_url": format!(
                        "https://github.com/{}/{}/releases/download/{}/{}",
                        repo.owner, repo.name, release.tag, asset.name
                    ),
                })
            })
            .collect();
        json!({
            "url": format!("{base}/releases/{index}"),
            "html_url": format!("https://github.com/{}/{}/releases/tag/{}", repo.owner, repo.name, release.tag),
            "id": index,
            "tag_name": release.tag,
            "name": format!("Release {}", release.tag),
            "draft": false,
            "prerelease": release.prerelease,
            "created_at": "2026-10-20T12:00:00Z",
            "published_at": "2026-10-20T12:00:00Z",
            "assets": assets,
        })
    }
}

impl Http for SimHttp {
    fn get(&self, url: &str, headers: &[(&str, &str)]) -> Result<Vec<u8>, HttpError> {
        let mut inner = self.inner();
        inner.requests.push(url.to_owned());
        let not_found = || HttpError::Status {
            status: 404,
            url: url.to_owned(),
        };
        let authorised = headers.iter().any(|(name, _)| *name == "Authorization");
        let repo = inner.repo.clone();
        let api = format!(
            "https://api.github.com/repos/{}/{}/releases/",
            repo.owner, repo.name
        );
        let download = format!(
            "https://github.com/{}/{}/releases/download/",
            repo.owner, repo.name
        );
        if let Some(rest) = url.strip_prefix(&api) {
            if inner.private && !authorised {
                return Err(not_found());
            }
            let found = if rest == "latest" {
                inner
                    .releases
                    .iter()
                    .enumerate()
                    .rev()
                    .find(|(_, r)| !r.prerelease)
            } else if let Some(tag) = rest.strip_prefix("tags/") {
                inner
                    .releases
                    .iter()
                    .enumerate()
                    .find(|(_, r)| r.tag == tag)
            } else if let Some(id) = rest.strip_prefix("assets/") {
                let id: usize = id.parse().map_err(|_| not_found())?;
                let asset = inner
                    .releases
                    .get(id / 100)
                    .and_then(|r| r.assets.get(id % 100))
                    .ok_or_else(not_found)?;
                return Ok(asset.bytes.clone());
            } else {
                None
            };
            let (index, release) = found.ok_or_else(not_found)?;
            return Ok(Self::release_json(&repo, release, index)
                .to_string()
                .into_bytes());
        }
        if let Some(rest) = url.strip_prefix(&download) {
            if inner.private {
                return Err(not_found());
            }
            let (tag, name) = rest.split_once('/').ok_or_else(not_found)?;
            return inner
                .releases
                .iter()
                .find(|r| r.tag == tag)
                .and_then(|r| r.assets.iter().find(|a| a.name == name))
                .map(|a| a.bytes.clone())
                .ok_or_else(not_found);
        }
        Err(not_found())
    }
}
