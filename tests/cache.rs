//! Releases kept between runs: downloaded only when agreed, kept once they check out, checked
//! again every time they are used, and never a reason to skip looking a release up.

use escapement_verge_probe::cache::Cache;
use escapement_verge_probe::progress::Quiet;
use escapement_verge_probe::release::{
    Download, GitHub, Http, HttpError, Release, ReleaseError, Repo,
};
use escapement_verge_probe::sim::http::{SimHttp, SimKey, SimRelease};
use escapement_verge_probe::source::{self, LoadError, Source};

fn repo() -> Repo {
    "someone/mculink-verge".parse().unwrap()
}

/// An empty cache directory of the test's own.
fn cache(name: &str) -> Cache {
    let dir = std::env::temp_dir().join(format!("verge-cache-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    Cache::at(dir)
}

fn github(http: &SimHttp, cache: &Cache) -> GitHub<SimHttp> {
    GitHub::new(http.clone(), repo(), None)
        .with_key(Some(http.public_key()))
        .with_cache(Some(cache.clone()))
}

/// How many asset downloads `http` has served.
fn downloads(http: &SimHttp) -> usize {
    http.requests()
        .iter()
        .filter(|url| url.contains("/download/"))
        .count()
}

/// Changes one character of the file at `path` without changing its size: the first hex digit
/// at or after `from`, as a fraction of the file.
fn spoil(path: &std::path::Path, from: f64) {
    let mut bytes = std::fs::read(path).unwrap();
    let start = (bytes.len() as f64 * from) as usize;
    let at = (start..bytes.len())
        .find(|&i| bytes[i].is_ascii_hexdigit())
        .unwrap();
    bytes[at] = if bytes[at] == b'0' { b'1' } else { b'0' };
    std::fs::write(path, bytes).unwrap();
}

#[test]
fn a_release_is_downloaded_only_when_agreed_and_then_kept() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let cache = cache("kept");
    let github = github(&http, &cache);
    let release = github.latest().unwrap();
    assert!(!cache.holds(&repo(), &release));
    assert_eq!(
        github
            .download(&release, Download::CachedOnly, &mut Quiet)
            .unwrap_err(),
        ReleaseError::NotAgreed {
            tag: "v0.3.0".into()
        }
    );
    let loaded = source::load(
        &Source::Release(None),
        &github,
        None,
        true,
        Download::CachedOnly,
        &mut Quiet,
    );
    assert!(
        matches!(
            loaded,
            Err(LoadError::Release(ReleaseError::NotAgreed { .. }))
        ),
        "{:?}",
        loaded.map(|l| l.name)
    );
    assert_eq!(downloads(&http), 0);

    let image = github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    assert_eq!(downloads(&http), 3);
    for asset in release.assets() {
        let kept = cache
            .root()
            .join("someone/mculink-verge/v0.3.0")
            .join(&asset.name);
        assert!(kept.is_file(), "{}", kept.display());
    }

    // The next run, as the `verge isp flash` that `verge update` advises: the release is looked up
    // again, and its image is not downloaded again.
    let next = self::github(&http, &cache);
    let release = next.tagged("v0.3.0").unwrap();
    assert!(cache.holds(&repo(), &release));
    let again = next
        .download(&release, Download::CachedOnly, &mut Quiet)
        .unwrap();
    assert_eq!(again.chunks(), image.chunks());
    assert_eq!(downloads(&http), 3);
    std::fs::remove_dir_all(cache.root()).unwrap();
}

#[derive(Clone, Copy)]
enum Asset {
    Image,
    Checksum,
    Signature,
}

impl Asset {
    fn of(self, release: &Release) -> &escapement_verge_probe::release::Asset {
        match self {
            Asset::Image => &release.image,
            Asset::Checksum => &release.checksum,
            Asset::Signature => &release.signature,
        }
    }
}

/// A cached copy is checked as a fresh download is, against the release as it is looked up now.
/// One that fails is deleted, and downloaded again only when agreed.
#[test]
fn a_cached_copy_is_checked_every_time_it_is_used() {
    // Each case: what goes wrong with the kept copy, a fraction of the way into which file to
    // change it, and what the refusal names. `None` keeps the copy and changes GitHub's digest:
    // it comes from the lookup, so a copy kept from an asset since replaced fails it.
    let cases = [
        ("the image", Some((Asset::Image, 0.5)), "does not verify"),
        (
            "the checksum file",
            Some((Asset::Checksum, 0.0)),
            "the release's checksum says",
        ),
        ("the signature", Some((Asset::Signature, 0.4)), ".minisig"),
        ("GitHub's digest", None, "GitHub's digest of it says"),
    ];
    for (case, spoilt, reason) in cases {
        let http = SimHttp::new(repo());
        http.publish_ours("v0.3.0");
        let cache = cache("checked");
        let github = github(&http, &cache);
        let mut release = github.latest().unwrap();
        github
            .download(&release, Download::Agreed, &mut Quiet)
            .unwrap();
        match spoilt {
            Some((asset, from)) => {
                let name = &asset.of(&release).name;
                spoil(&cache.path(&repo(), "v0.3.0", name), from);
            }
            None => release.image.sha256 = Some("0".repeat(64)),
        }
        assert!(cache.holds(&repo(), &release), "{case}");
        let error = github
            .download(&release, Download::CachedOnly, &mut Quiet)
            .unwrap_err();
        let ReleaseError::CachedCopyFailed {
            tag,
            reason: got,
            left: None,
        } = &error
        else {
            panic!("{case}: {error:?}");
        };
        assert_eq!(tag, "v0.3.0", "{case}");
        assert!(got.contains(reason), "{case}: {got}");
        assert!(
            !cache.release_dir(&repo(), "v0.3.0").exists(),
            "{case}: the failed copy is still there"
        );
        assert!(!cache.holds(&repo(), &release), "{case}");
        assert!(
            matches!(
                github.download(&release, Download::CachedOnly, &mut Quiet),
                Err(ReleaseError::NotAgreed { .. })
            ),
            "{case}"
        );
        assert_eq!(downloads(&http), 3, "{case}");
        if spoilt.is_some() {
            github
                .download(&release, Download::Agreed, &mut Quiet)
                .unwrap();
            assert_eq!(downloads(&http), 6, "{case}");
            assert!(cache.holds(&repo(), &release), "{case}");
        }
        std::fs::remove_dir_all(cache.root()).unwrap();
    }
}

#[test]
fn a_release_that_fails_its_check_is_not_kept() {
    let http = SimHttp::new(repo());
    let mut assets = http.ours_assets("v0.3.0");
    let image = assets[0].clone();
    assets[2] = SimKey::generate().sign(
        &format!("{}.minisig", image.name),
        &image.bytes,
        &format!("file:{} tag:v0.3.0", image.name),
    );
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets,
    });
    let cache = cache("refused");
    let github = github(&http, &cache);
    let release = github.latest().unwrap();
    let error = github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap_err();
    assert!(
        matches!(error, ReleaseError::BadSignature { .. }),
        "{error:?}"
    );
    assert!(!cache.holds(&repo(), &release));
    assert!(!cache.release_dir(&repo(), "v0.3.0").exists());
}

/// Fails every request as a machine with no network does, or as a lookup that stalls.
#[derive(Clone)]
struct Unreachable(HttpError);

impl Http for Unreachable {
    fn get(&self, _: &str, _: &[(&str, &str)]) -> Result<Vec<u8>, HttpError> {
        Err(self.0.clone())
    }
}

/// Which tag is latest, and the digest a cached copy is checked against, come from GitHub. A
/// cached release is no fallback when it cannot be reached, with no network or a lookup that
/// stalls.
#[test]
fn looking_a_release_up_needs_github_even_when_it_is_cached() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let cache = cache("offline");
    let online = github(&http, &cache);
    let release = online.latest().unwrap();
    online
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();

    for failure in [
        HttpError::Transport("dns failed: no address for api.github.com".into()),
        HttpError::Timeout {
            url: "https://api.github.com/repos/someone/mculink-verge/releases/latest".into(),
            phase: "receive response".into(),
        },
    ] {
        let offline = GitHub::new(Unreachable(failure.clone()), repo(), None)
            .with_key(Some(http.public_key()))
            .with_cache(Some(cache.clone()));
        let error = offline.latest().unwrap_err();
        assert_eq!(error, ReleaseError::Unreachable(failure.clone()));
        assert!(
            error
                .to_string()
                .contains("looked up on GitHub every time it is used"),
            "{error}"
        );
        for tag in [None, Some("v0.3.0".to_owned())] {
            let loaded = source::load(
                &Source::Release(tag.clone()),
                &offline,
                None,
                true,
                Download::CachedOnly,
                &mut Quiet,
            );
            assert!(
                matches!(
                    loaded,
                    Err(LoadError::Release(ReleaseError::Unreachable(_)))
                ),
                "{failure:?}, {tag:?}: {:?}",
                loaded.map(|l| l.name)
            );
        }
    }
    std::fs::remove_dir_all(cache.root()).unwrap();
}

/// What the command line does: ask before a download, not before using a kept copy, and ask
/// again before replacing a kept copy that fails its check. A no downloads nothing.
#[test]
fn a_download_is_asked_for_and_a_kept_copy_is_not() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let cache = cache("asking");
    let github = github(&http, &cache);
    let mut asked = Vec::new();
    let mut load = |answer: Result<(), &'static str>| {
        source::load_asking(
            &Source::Release(None),
            &github,
            None,
            true,
            |what: &str| {
                asked.push(what.to_owned());
                answer
            },
            &mut Quiet,
        )
    };

    assert_eq!(load(Err("no")).unwrap_err(), "no");
    assert_eq!(downloads(&http), 0);
    load(Ok(())).unwrap().unwrap();
    assert_eq!(downloads(&http), 3);
    load(Ok(())).unwrap().unwrap();
    assert_eq!(downloads(&http), 3);
    let release = github.latest().unwrap();
    spoil(&cache.path(&repo(), "v0.3.0", &release.image.name), 0.5);
    assert_eq!(load(Err("no")).unwrap_err(), "no");
    assert_eq!(downloads(&http), 3);
    assert!(!cache.holds(&repo(), &release));
    load(Ok(())).unwrap().unwrap();
    assert_eq!(downloads(&http), 6);
    drop(load);

    let what = format!(
        "release v0.3.0 of someone/mculink-verge from GitHub ({:.1} KiB)",
        release.download_size() as f64 / 1024.0
    );
    assert_eq!(asked, vec![what; 4]);
    std::fs::remove_dir_all(cache.root()).unwrap();
}

/// An agreed download fetches the release whatever the cache holds, and replaces a bad copy. A
/// copy the tool could not delete would otherwise be read again in place of the download.
#[test]
fn an_agreed_download_fetches_whatever_the_cache_holds() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let cache = cache("agreed");
    let github = github(&http, &cache);
    let release = github.latest().unwrap();
    github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    spoil(&cache.path(&repo(), "v0.3.0", &release.image.name), 0.5);
    github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    assert_eq!(downloads(&http), 6);
    github
        .download(&release, Download::CachedOnly, &mut Quiet)
        .unwrap();
    assert_eq!(downloads(&http), 6);
    std::fs::remove_dir_all(cache.root()).unwrap();
}

/// Makes the file at `path` unreadable to this user, keeping its size where the permissions bind.
/// Where they do not, as for root, a directory in its place stands in, which has another size.
fn unreadable(path: &std::path::Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o0)).unwrap();
        if std::fs::read(path).is_err() {
            return;
        }
    }
    std::fs::remove_file(path).unwrap();
    std::fs::create_dir(path).unwrap();
}

/// A kept asset the tool cannot read, or one of another size than the release gives it, is not
/// used, and the release is asked about and downloaded as if nothing were kept.
#[test]
fn a_kept_copy_that_cannot_be_used_is_asked_about() {
    for case in ["unreadable", "another size"] {
        let http = SimHttp::new(repo());
        http.publish_ours("v0.3.0");
        let cache = cache("unusable");
        let github = github(&http, &cache);
        let release = github.latest().unwrap();
        github
            .download(&release, Download::Agreed, &mut Quiet)
            .unwrap();
        let image = cache.path(&repo(), "v0.3.0", &release.image.name);
        if case == "unreadable" {
            unreadable(&image);
        } else {
            let mut bytes = std::fs::read(&image).unwrap();
            bytes.push(b'\n');
            std::fs::write(&image, bytes).unwrap();
        }
        assert!(!cache.holds(&repo(), &release), "{case}");
        assert!(
            matches!(
                github.download(&release, Download::CachedOnly, &mut Quiet),
                Err(ReleaseError::NotAgreed { .. })
            ),
            "{case}"
        );
        let mut asked = 0;
        source::load_asking(
            &Source::Release(None),
            &github,
            None,
            true,
            |_: &str| {
                asked += 1;
                Ok::<(), ()>(())
            },
            &mut Quiet,
        )
        .unwrap()
        .unwrap();
        assert_eq!((asked, downloads(&http)), (1, 6), "{case}");
        std::fs::remove_dir_all(cache.root()).unwrap();
    }
}

/// A copy that fails its check and cannot be deleted, as in a cache another user wrote, is said
/// to be left, and an agreed download still replaces what it can. Needs a user the permissions
/// bind, so it passes without checking anything as root.
#[cfg(unix)]
#[test]
fn a_failed_copy_that_cannot_be_deleted_says_so() {
    use std::os::unix::fs::PermissionsExt;
    let chmod = |path: &std::path::Path, mode: u32| {
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
    };
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let cache = cache("undeletable");
    let github = github(&http, &cache);
    let release = github.latest().unwrap();
    github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    let dir = cache.release_dir(&repo(), "v0.3.0");
    spoil(&cache.path(&repo(), "v0.3.0", &release.image.name), 0.5);
    chmod(&dir, 0o555);
    let probe = dir.join("probe");
    if std::fs::write(&probe, b"").is_ok() {
        eprintln!("the permissions do not bind this user; nothing checked");
        chmod(&dir, 0o755);
        std::fs::remove_dir_all(cache.root()).unwrap();
        return;
    }
    let error = github
        .download(&release, Download::CachedOnly, &mut Quiet)
        .unwrap_err();
    assert!(
        matches!(&error, ReleaseError::CachedCopyFailed { left: Some(_), .. }),
        "{error:?}"
    );
    assert!(
        error.to_string().contains("could not be deleted"),
        "{error}"
    );
    let mut asked = 0;
    let loaded = source::load_asking(
        &Source::Release(None),
        &github,
        None,
        true,
        |_: &str| {
            asked += 1;
            Ok::<(), ()>(())
        },
        &mut Quiet,
    )
    .unwrap();
    chmod(&dir, 0o755);
    loaded.unwrap();
    assert_eq!((asked, downloads(&http)), (1, 6));
    std::fs::remove_dir_all(cache.root()).unwrap();
}
