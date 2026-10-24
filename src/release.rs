//! Our firmware's GitHub releases: finding one, and downloading its image with its signature and
//! checksums checked before anything is flashed.
//!
//! The asset convention a release has to follow:
//!
//! - exactly one asset whose name ends in `.s19`: the image, as `objcopy -O srec` writes it;
//! - an asset named the same with `.minisig` appended: a minisign signature of the image, made
//!   with the key whose public half is [`RELEASE_KEY`], whose trusted comment names the image as
//!   `file:<name>` and the release as `tag:<tag>`. It has to be pre-hashed, as minisign signs by
//!   default; a legacy signature is refused;
//! - an asset named the same with `.sha256` appended, holding the image's SHA-256 in hex, as
//!   `sha256sum` writes it (`<hex>  <name>`). A file name after the hash, if present, must be the
//!   image's.
//!
//! The release's tag is the version: a probe built at that tag reports the same string. The
//! boards the image runs on are in its declaration ([`crate::image::Declaration`]), inside the
//! signed image.
//!
//! The signature is what makes an image ours. The trusted comment is signed with it. Naming the
//! file there stops a signed image being published under another name, and naming the tag stops
//! an old image being published again as a newer release, under its own name. The checksum file
//! is checked as well, and so is the SHA-256 GitHub keeps of the image (`digest`), where it
//! reports one.
//!
//! A release is downloaded only once the user agrees to it ([`Download`]), and kept in a
//! [`Cache`] for the next run. A cached copy is checked as a fresh download is, every time it is
//! used. Looking a release up always asks GitHub.

use std::fmt;
use std::str::FromStr;
use std::time::Duration;

use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::cache::Cache;
use crate::image::{Image, ImageError};
use crate::progress::{Progress, Step};

/// Where releases are looked up when nothing says otherwise. The organisation exists; the
/// repository does not yet, so until it is published every lookup finds nothing.
pub const DEFAULT_REPO: &str = "Escapement-Project/mculink-verge";

/// The minisign public key releases are signed with: the base64 line of the `.pub` file
/// `minisign -G` writes. `None` until that key pair exists; until then the tool can check no
/// release's signature, so it installs none.
pub const RELEASE_KEY: Option<&str> = None;

const API: &str = "https://api.github.com";
/// Generous for a 250 KB image, and small enough that a wrong URL cannot fill memory.
pub const DOWNLOAD_LIMIT: u64 = 8 << 20;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Repo {
    pub owner: String,
    pub name: String,
}

impl Default for Repo {
    fn default() -> Repo {
        DEFAULT_REPO.parse().expect("DEFAULT_REPO is owner/name")
    }
}

impl FromStr for Repo {
    type Err = String;
    fn from_str(text: &str) -> Result<Repo, String> {
        match text.split_once('/') {
            Some((owner, name)) if !owner.is_empty() && !name.is_empty() && !name.contains('/') => {
                Ok(Repo {
                    owner: owner.to_owned(),
                    name: name.to_owned(),
                })
            }
            _ => Err(format!("expected OWNER/NAME, got {text:?}")),
        }
    }
}

impl fmt::Display for Repo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.owner, self.name)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum HttpError {
    #[error("HTTP {status} from {url}")]
    Status { status: u16, url: String },
    #[error("{url} stalled: no answer in time to {phase}. Check the network and try again")]
    Timeout { url: String, phase: String },
    #[error("{0}")]
    Transport(String),
}

/// An HTTP GET, so that tests can answer from recorded replies.
pub trait Http {
    fn get(&self, url: &str, headers: &[(&str, &str)]) -> Result<Vec<u8>, HttpError>;
}

/// How long each part of a request may take before it fails as stalled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timeouts {
    /// Looking up the host, and connecting to it with TLS.
    pub connect: Duration,
    /// Sending the request, and receiving the reply's headers.
    pub response: Duration,
    /// Receiving the whole body.
    pub body: Duration,
}

impl Default for Timeouts {
    fn default() -> Timeouts {
        Timeouts {
            connect: Duration::from_secs(15),
            response: Duration::from_secs(30),
            // An image is about 250 KB, so this allows a link of a few KB/s.
            body: Duration::from_secs(60),
        }
    }
}

/// HTTPS through `ureq`.
#[derive(Clone)]
pub struct Ureq(ureq::Agent);

impl Ureq {
    #[must_use]
    pub fn new(timeouts: Timeouts) -> Ureq {
        let config = ureq::Agent::config_builder()
            .timeout_resolve(Some(timeouts.connect))
            .timeout_connect(Some(timeouts.connect))
            .timeout_send_request(Some(timeouts.response))
            .timeout_recv_response(Some(timeouts.response))
            .timeout_recv_body(Some(timeouts.body))
            .build();
        Ureq(config.into())
    }
}

impl Default for Ureq {
    fn default() -> Ureq {
        Ureq::new(Timeouts::default())
    }
}

impl Http for Ureq {
    fn get(&self, url: &str, headers: &[(&str, &str)]) -> Result<Vec<u8>, HttpError> {
        let failed = |error: ureq::Error| match error {
            ureq::Error::StatusCode(status) => HttpError::Status {
                status,
                url: url.to_owned(),
            },
            ureq::Error::Timeout(phase) => HttpError::Timeout {
                url: url.to_owned(),
                phase: phase.to_string(),
            },
            other => HttpError::Transport(other.to_string()),
        };
        let mut request = self.0.get(url);
        for (name, value) in headers {
            request = request.header(*name, *value);
        }
        request
            .call()
            .map_err(failed)?
            .body_mut()
            .with_config()
            .limit(DOWNLOAD_LIMIT)
            .read_to_vec()
            .map_err(failed)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ReleaseError {
    #[error("{repo} has no published release{}", tag.as_ref().map(|t| format!(" tagged {t}")).unwrap_or_default())]
    NotFound { repo: String, tag: Option<String> },
    #[error(
        "GitHub refused the request (HTTP {0}): the rate limit for unauthenticated use, or a private repository without a token"
    )]
    Refused(u16),
    #[error(transparent)]
    Http(HttpError),
    #[error(
        "{0}. A release is looked up on GitHub every time it is used, even one downloaded already"
    )]
    Unreachable(HttpError),
    #[error("the copy of release {tag} in the cache failed its check, {}: {reason}", deleted(.left))]
    CachedCopyFailed {
        tag: String,
        reason: String,
        /// Why the copy is still there, where it could not be deleted.
        left: Option<String>,
    },
    #[error(
        "release {tag} would have to be downloaded, and that was not asked. Nothing was downloaded or flashed"
    )]
    NotAgreed { tag: String },
    #[error("GitHub's reply did not parse: {0}")]
    Json(String),
    #[error("release {tag} has {found} .s19 assets; the convention is exactly one")]
    ImageAsset { tag: String, found: usize },
    #[error("release {tag} has no {name}, so its image cannot be checked")]
    NoChecksum { tag: String, name: String },
    #[error("{name} does not start with a SHA-256 in hex")]
    BadChecksumFile { name: String },
    #[error("{name} is for {named}, not {image}")]
    ChecksumNamesAnother {
        name: String,
        named: String,
        image: String,
    },
    #[error("{name} was {got} bytes, and the release says {expected}")]
    WrongSize {
        name: String,
        expected: u64,
        got: usize,
    },
    #[error(
        "{name} has SHA-256 {got}, and the release's checksum says {expected}. Nothing was flashed"
    )]
    ChecksumMismatch {
        name: String,
        expected: String,
        got: String,
    },
    #[error(
        "{name} has SHA-256 {got}, and GitHub's digest of it says {expected}. Nothing was flashed"
    )]
    DigestMismatch {
        name: String,
        expected: String,
        got: String,
    },
    #[error("{name}: {error}")]
    Image { name: String, error: ImageError },
    #[error("release {tag} has no {name}: it is not signed, so the tool will not install it")]
    Unsigned { tag: String, name: String },
    #[error(
        "this build of verge has no release key, so it cannot check a release's signature and installs none"
    )]
    NoKey,
    #[error("{name} is not a minisign signature ({reason}). Nothing was flashed")]
    BadSignatureFile { name: String, reason: String },
    #[error("{name} does not verify: {reason}. Nothing was flashed")]
    BadSignature { name: String, reason: String },
    #[error("{name} is a signature of {named}, not of {image}. Nothing was flashed")]
    SignatureNamesAnother {
        name: String,
        named: String,
        image: String,
    },
    #[error(
        "{name}'s trusted comment does not name the image it signs, as file:{image}. Nothing was flashed"
    )]
    SignatureNamesNothing { name: String, image: String },
    #[error("{name} was signed for release {signed}, not {tag}. Nothing was flashed")]
    SignatureForAnotherRelease {
        name: String,
        signed: String,
        tag: String,
    },
    #[error(
        "{name}'s trusted comment does not name the release it was signed for, as tag:{tag}. Nothing was flashed"
    )]
    SignatureNamesNoRelease { name: String, tag: String },
}

fn deleted(left: &Option<String>) -> String {
    match left {
        None => "so it was deleted".to_owned(),
        Some(error) => format!("and could not be deleted ({error})"),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Asset {
    pub name: String,
    pub size: u64,
    /// The public download URL.
    pub download_url: String,
    /// The API URL, which serves the bytes to a token for a private repository.
    pub api_url: String,
    /// GitHub's own SHA-256 of the asset, in hex, where the API reports one.
    pub sha256: Option<String>,
}

/// A release that follows the asset convention.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Release {
    pub tag: String,
    pub name: Option<String>,
    pub published: Option<String>,
    pub page: String,
    pub prerelease: bool,
    pub image: Asset,
    pub checksum: Asset,
    pub signature: Asset,
}

impl Release {
    /// The assets using the release takes: its signature, its checksum and its image, in the
    /// order [`GitHub::download`] fetches them.
    #[must_use]
    pub fn assets(&self) -> [&Asset; 3] {
        [&self.signature, &self.checksum, &self.image]
    }

    /// How many bytes downloading the release fetches.
    #[must_use]
    pub fn download_size(&self) -> u64 {
        self.assets().iter().map(|asset| asset.size).sum()
    }
}

/// Whether [`GitHub::download`] may fetch a release, or only take it from the cache.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Download {
    /// The user agreed to the download, in answer to a question that named it. The release is
    /// downloaded whatever the cache holds.
    Agreed,
    /// Nothing was asked: only a cached copy may be used.
    CachedOnly,
}

#[derive(Deserialize)]
struct ApiRelease {
    tag_name: String,
    name: Option<String>,
    published_at: Option<String>,
    html_url: String,
    #[serde(default)]
    prerelease: bool,
    assets: Vec<ApiAsset>,
}

#[derive(Deserialize)]
struct ApiAsset {
    name: String,
    size: u64,
    browser_download_url: String,
    url: String,
    #[serde(default)]
    digest: Option<String>,
}

impl From<ApiAsset> for Asset {
    fn from(asset: ApiAsset) -> Asset {
        Asset {
            name: asset.name,
            size: asset.size,
            download_url: asset.browser_download_url,
            api_url: asset.url,
            sha256: asset
                .digest
                .and_then(|d| d.strip_prefix("sha256:").map(str::to_ascii_lowercase)),
        }
    }
}

/// [`RELEASE_KEY`], parsed.
#[must_use]
pub fn release_key() -> Option<minisign_verify::PublicKey> {
    RELEASE_KEY.map(|key| {
        minisign_verify::PublicKey::from_base64(key).expect("RELEASE_KEY is a minisign public key")
    })
}

/// A repository's releases. Public releases need no token.
#[derive(Clone)]
pub struct GitHub<H> {
    http: H,
    repo: Repo,
    token: Option<String>,
    key: Option<minisign_verify::PublicKey>,
    cache: Option<Cache>,
}

impl<H: Http> GitHub<H> {
    /// Releases of `repo`, checked against the key built into the tool.
    pub fn new(http: H, repo: Repo, token: Option<String>) -> GitHub<H> {
        GitHub {
            http,
            repo,
            token,
            key: release_key(),
            cache: None,
        }
    }

    /// Keeps the releases it downloads in `cache`, and takes them from there next time.
    #[must_use]
    pub fn with_cache(mut self, cache: Option<Cache>) -> GitHub<H> {
        self.cache = cache;
        self
    }

    /// Checks signatures against `key` instead of the built-in one. For a simulated bench and the
    /// tests, whose releases are signed with a key made for them.
    #[must_use]
    pub fn with_key(mut self, key: Option<minisign_verify::PublicKey>) -> GitHub<H> {
        self.key = key;
        self
    }

    #[must_use]
    pub fn repo(&self) -> &Repo {
        &self.repo
    }

    fn get(&self, url: &str, accept: &str) -> Result<Vec<u8>, HttpError> {
        let auth = self.token.as_ref().map(|token| format!("Bearer {token}"));
        let mut headers = vec![
            ("Accept", accept),
            ("X-GitHub-Api-Version", "2022-11-28"),
            ("User-Agent", concat!("verge/", env!("CARGO_PKG_VERSION"))),
        ];
        if let Some(auth) = &auth {
            headers.push(("Authorization", auth));
        }
        self.http.get(url, &headers)
    }

    fn release(&self, url: &str, tag: Option<&str>) -> Result<Release, ReleaseError> {
        let body = self
            .get(url, "application/vnd.github+json")
            .map_err(|error| match error {
                HttpError::Status { status: 404, .. } => ReleaseError::NotFound {
                    repo: self.repo.to_string(),
                    tag: tag.map(str::to_owned),
                },
                HttpError::Status {
                    status: status @ (401 | 403 | 429),
                    ..
                } => ReleaseError::Refused(status),
                unreachable @ (HttpError::Transport(_) | HttpError::Timeout { .. }) => {
                    ReleaseError::Unreachable(unreachable)
                }
                other => ReleaseError::Http(other),
            })?;
        let release: ApiRelease =
            serde_json::from_slice(&body).map_err(|error| ReleaseError::Json(error.to_string()))?;
        conventional(release)
    }

    /// The latest release: GitHub's choice, which skips drafts and prereleases.
    pub fn latest(&self) -> Result<Release, ReleaseError> {
        let url = format!(
            "{API}/repos/{}/{}/releases/latest",
            self.repo.owner, self.repo.name
        );
        self.release(&url, None)
    }

    pub fn tagged(&self, tag: &str) -> Result<Release, ReleaseError> {
        let url = format!(
            "{API}/repos/{}/{}/releases/tags/{tag}",
            self.repo.owner, self.repo.name
        );
        self.release(&url, Some(tag))
    }

    fn fetch(&self, asset: &Asset) -> Result<Vec<u8>, ReleaseError> {
        let bytes = match &self.token {
            Some(_) => self.get(&asset.api_url, "application/octet-stream"),
            None => self.get(&asset.download_url, "application/octet-stream"),
        }
        .map_err(ReleaseError::Http)?;
        if bytes.len() as u64 != asset.size {
            return Err(ReleaseError::WrongSize {
                name: asset.name.clone(),
                expected: asset.size,
                got: bytes.len(),
            });
        }
        Ok(bytes)
    }

    /// The release's image, its signature and its checksum, with the image parsed only once all
    /// of them check out. `CachedOnly` takes them from the cache, where it holds them all; a copy
    /// that fails its check is deleted, and [`ReleaseError::CachedCopyFailed`] says so. `Agreed`
    /// downloads them whatever the cache holds, and keeps them there once they check out.
    pub fn download(
        &self,
        release: &Release,
        download: Download,
        progress: &mut dyn Progress,
    ) -> Result<Image, ReleaseError> {
        let key = self.key.as_ref().ok_or(ReleaseError::NoKey)?;
        let assets = release.assets();
        if download == Download::CachedOnly {
            let kept = self.cache.as_ref().and_then(|cache| {
                let [a, b, c] = assets.map(|asset| cache.read(&self.repo, &release.tag, asset));
                Some((cache, [a?, b?, c?]))
            });
            let Some((cache, kept)) = kept else {
                return Err(ReleaseError::NotAgreed {
                    tag: release.tag.clone(),
                });
            };
            let dir = cache.release_dir(&self.repo, &release.tag);
            tracing::info!(
                "release {} from the cache in {}",
                release.tag,
                dir.display()
            );
            return check(key, release, &kept).map_err(|error| ReleaseError::CachedCopyFailed {
                tag: release.tag.clone(),
                reason: error.to_string(),
                left: cache
                    .discard(&self.repo, &release.tag)
                    .err()
                    .map(|error| format!("{}: {error}", dir.display())),
            });
        }
        progress.start(Step::Download, release.download_size());
        let mut done = 0;
        let mut fetch = |asset: &Asset| {
            let bytes = self.fetch(asset)?;
            done += asset.size;
            progress.advance(done);
            Ok::<_, ReleaseError>(bytes)
        };
        let [signature, checksum, image] = assets;
        let fetched = [fetch(signature)?, fetch(checksum)?, fetch(image)?];
        let parsed = check(key, release, &fetched)?;
        if let Some(cache) = &self.cache {
            for (asset, bytes) in assets.iter().zip(&fetched) {
                if let Err(error) = cache.write(&self.repo, &release.tag, &asset.name, bytes) {
                    tracing::warn!("could not keep {} in the cache: {error}", asset.name);
                }
            }
        }
        Ok(parsed)
    }
}

/// Checks a release's image against its signature, its checksum file and GitHub's digest, and
/// parses it. `assets` are the contents of [`Release::assets`], in that order.
fn check(
    key: &minisign_verify::PublicKey,
    release: &Release,
    assets: &[Vec<u8>; 3],
) -> Result<Image, ReleaseError> {
    let [signature, sums, bytes] = assets;
    let expected = parse_checksum(sums, &release.checksum.name, &release.image.name)?;
    verify_signature(
        key,
        &release.signature.name,
        signature,
        &release.image.name,
        &release.tag,
        bytes,
    )?;
    let got = hex::encode(Sha256::digest(bytes));
    if got != expected {
        return Err(ReleaseError::ChecksumMismatch {
            name: release.image.name.clone(),
            expected,
            got,
        });
    }
    if let Some(digest) = &release.image.sha256
        && *digest != got
    {
        return Err(ReleaseError::DigestMismatch {
            name: release.image.name.clone(),
            expected: digest.clone(),
            got,
        });
    }
    let image = Image::parse(bytes).map_err(|error| ReleaseError::Image {
        name: release.image.name.clone(),
        error,
    })?;
    image
        .check_for_mculink()
        .map_err(|error| ReleaseError::Image {
            name: release.image.name.clone(),
            error,
        })?;
    Ok(image)
}

fn conventional(release: ApiRelease) -> Result<Release, ReleaseError> {
    let ApiRelease {
        tag_name,
        name,
        published_at,
        html_url,
        prerelease,
        assets,
    } = release;
    let (images, others): (Vec<ApiAsset>, Vec<ApiAsset>) = assets
        .into_iter()
        .partition(|asset| asset.name.ends_with(".s19"));
    let [image] = <[ApiAsset; 1]>::try_from(images).map_err(|images| ReleaseError::ImageAsset {
        tag: tag_name.clone(),
        found: images.len(),
    })?;
    let signature_name = format!("{}.minisig", image.name);
    let checksum_name = format!("{}.sha256", image.name);
    let mut signature = None;
    let mut checksum = None;
    for asset in others {
        if asset.name == signature_name {
            signature = Some(asset);
        } else if asset.name == checksum_name {
            checksum = Some(asset);
        }
    }
    let signature = signature.ok_or_else(|| ReleaseError::Unsigned {
        tag: tag_name.clone(),
        name: signature_name,
    })?;
    let checksum = checksum.ok_or_else(|| ReleaseError::NoChecksum {
        tag: tag_name.clone(),
        name: checksum_name,
    })?;
    Ok(Release {
        tag: tag_name,
        name,
        published: published_at,
        page: html_url,
        prerelease,
        image: image.into(),
        checksum: checksum.into(),
        signature: signature.into(),
    })
}

/// Checks that `signature`, the contents of the asset `name`, is `key`'s signature of `image`, and
/// that its trusted comment names the image as `file:<image_name>` and the release as `tag:<tag>`.
pub fn verify_signature(
    key: &minisign_verify::PublicKey,
    name: &str,
    signature: &[u8],
    image_name: &str,
    tag: &str,
    image: &[u8],
) -> Result<(), ReleaseError> {
    let bad_file = |reason: String| ReleaseError::BadSignatureFile {
        name: name.to_owned(),
        reason,
    };
    let text = std::str::from_utf8(signature).map_err(|_| bad_file("it is not text".into()))?;
    let decoded =
        minisign_verify::Signature::decode(text).map_err(|error| bad_file(error.to_string()))?;
    key.verify(image, &decoded, false)
        .map_err(|error| ReleaseError::BadSignature {
            name: name.to_owned(),
            reason: match error {
                minisign_verify::Error::UnexpectedKeyId => {
                    "it was made with a key other than the one built into verge".to_owned()
                }
                minisign_verify::Error::UnexpectedAlgorithm => {
                    "it is a legacy signature, of the image itself rather than its hash, and verge takes only the pre-hashed signatures minisign makes by default".to_owned()
                }
                minisign_verify::Error::InvalidSignature => {
                    "the image or its trusted comment was changed after it was signed".to_owned()
                }
                other => other.to_string(),
            },
        })?;
    // Verified above, so the comment is what the key's holder signed.
    let comment = decoded.trusted_comment();
    let field = |prefix: &str| {
        comment
            .split_whitespace()
            .find_map(|word| word.strip_prefix(prefix))
    };
    match field("file:") {
        Some(named) if named == image_name => {}
        Some(named) => {
            return Err(ReleaseError::SignatureNamesAnother {
                name: name.to_owned(),
                named: named.to_owned(),
                image: image_name.to_owned(),
            });
        }
        None => {
            return Err(ReleaseError::SignatureNamesNothing {
                name: name.to_owned(),
                image: image_name.to_owned(),
            });
        }
    }
    match field("tag:") {
        Some(signed) if signed == tag => Ok(()),
        Some(signed) => Err(ReleaseError::SignatureForAnotherRelease {
            name: name.to_owned(),
            signed: signed.to_owned(),
            tag: tag.to_owned(),
        }),
        None => Err(ReleaseError::SignatureNamesNoRelease {
            name: name.to_owned(),
            tag: tag.to_owned(),
        }),
    }
}

/// The hex digest from a `sha256sum`-style file, lowercased.
pub fn parse_checksum(bytes: &[u8], name: &str, image: &str) -> Result<String, ReleaseError> {
    let bad = || ReleaseError::BadChecksumFile {
        name: name.to_owned(),
    };
    let text = std::str::from_utf8(bytes).map_err(|_| bad())?;
    let mut words = text.split_whitespace();
    let digest = words.next().ok_or_else(bad)?;
    if digest.len() != 64 || !digest.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(bad());
    }
    if let Some(named) = words.next() {
        // `sha256sum -b` marks the name with '*'.
        let named = named.strip_prefix('*').unwrap_or(named);
        if named != image {
            return Err(ReleaseError::ChecksumNamesAnother {
                name: name.to_owned(),
                named: named.to_owned(),
                image: image.to_owned(),
            });
        }
    }
    Ok(digest.to_ascii_lowercase())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_client_bounds_every_wait() {
        let defaults = Timeouts::default();
        let set = Ureq::default().0.config().timeouts();
        assert_eq!(set.resolve, Some(defaults.connect));
        assert_eq!(set.connect, Some(defaults.connect));
        assert_eq!(set.send_request, Some(defaults.response));
        assert_eq!(set.recv_response, Some(defaults.response));
        assert_eq!(set.recv_body, Some(defaults.body));
    }
}
