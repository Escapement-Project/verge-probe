//! Release lookup: GitHub's real reply format, the asset convention, and the signature and
//! checksum checks.
//!
//! Each refusal test builds a release in which exactly one check can fail: an image changed after
//! signing gets fresh checksums, and an image whose checksum is wrong is signed properly.

use std::path::Path;

use serde_json::Value;
use sha2::{Digest, Sha256};
use verge_probe::progress::Quiet;
use verge_probe::release::{
    Download, GitHub, Http, HttpError, ReleaseError, Repo, Timeouts, Ureq, parse_checksum,
    verify_signature,
};
use verge_probe::sim::http::{SimAsset, SimHttp, SimKey, SimRelease};
use verge_probe::sim::{SimBus, SimFirmware, firmware_image, firmware_srecord};
use verge_probe::source::{self, LoadError, Source};
use verge_probe::update::{self, Outcome, Timing};
use verge_probe::version::Relation;
use verge_probe::{Cancel, Firmware, State, detect};

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name),
    )
    .unwrap()
}

/// Answers every request with one body.
struct Fixed(Vec<u8>);

impl Http for Fixed {
    fn get(&self, _: &str, _: &[(&str, &str)]) -> Result<Vec<u8>, HttpError> {
        Ok(self.0.clone())
    }
}

fn repo() -> Repo {
    "someone/mculink-verge".parse().unwrap()
}

const IMAGE: &str = "mculink-verge-v0.3.0.s19";

#[test]
fn a_recorded_github_reply_parses() {
    let recorded = fixture("github-release-probe-rs.json");
    // probe-rs's release has no .s19, so it parses and then fails the convention.
    let github = GitHub::new(Fixed(recorded.clone()), repo(), None);
    assert_eq!(
        github.latest(),
        Err(ReleaseError::ImageAsset {
            tag: "v0.32.0".into(),
            found: 0
        })
    );

    // The same reply with its assets renamed to follow the convention, and one asset's record
    // copied to stand for the signature.
    let mut json: Value = serde_json::from_slice(&recorded).unwrap();
    json["assets"][0]["name"] = "fw.s19.sha256".into();
    json["assets"][1]["name"] = "fw.s19".into();
    let mut signature = json["assets"][0].clone();
    signature["name"] = "fw.s19.minisig".into();
    json["assets"].as_array_mut().unwrap().push(signature);
    let github = GitHub::new(Fixed(json.to_string().into_bytes()), repo(), None);
    let release = github.latest().unwrap();
    let original: Value = serde_json::from_slice(&recorded).unwrap();
    let asset = &original["assets"][1];
    assert_eq!(release.tag, "v0.32.0");
    assert_eq!(release.page, original["html_url"].as_str().unwrap());
    assert_eq!(release.image.size, asset["size"].as_u64().unwrap());
    assert_eq!(
        release.image.download_url,
        asset["browser_download_url"].as_str().unwrap()
    );
    assert_eq!(release.image.api_url, asset["url"].as_str().unwrap());
    assert_eq!(
        release.image.sha256.as_deref(),
        asset["digest"].as_str().unwrap().strip_prefix("sha256:")
    );
    assert_eq!(release.signature.name, "fw.s19.minisig");
}

/// cargo-dist's checksum file for a probe-rs asset, against GitHub's own digest of that asset:
/// two sources that do not share a computation.
#[test]
fn a_recorded_checksum_file_matches_githubs_digest() {
    let name = "probe-rs-tools-aarch64-apple-darwin.tar.xz";
    let sums = fixture(&format!("{name}.sha256"));
    let digest = parse_checksum(&sums, &format!("{name}.sha256"), name).unwrap();
    let json: Value = serde_json::from_slice(&fixture("github-release-probe-rs.json")).unwrap();
    let asset = json["assets"]
        .as_array()
        .unwrap()
        .iter()
        .find(|a| a["name"] == name)
        .unwrap();
    assert_eq!(
        Some(format!("sha256:{digest}").as_str()),
        asset["digest"].as_str()
    );
    assert!(matches!(
        parse_checksum(&sums, "x.sha256", "another.tar.xz"),
        Err(ReleaseError::ChecksumNamesAnother { .. })
    ));
}

/// A signature the minisign command-line tool made, with its default trusted comment
/// (`timestamp:…\tfile:test\tprehashed`), from the documentation of minisign-verify 0.3.0
/// (`src/lib.rs`, lines 20 to 30). It is not made by the code under test.
///
/// It verifies, and its comment names the file, so the only check left to refuse it is the last:
/// a default comment names no release.
#[test]
fn a_signature_made_by_the_minisign_tool_verifies() {
    let key = minisign_verify::PublicKey::from_base64(
        "RWQf6LRCGA9i53mlYecO4IzT51TGPpvWucNSCh1CBM0QTaLn73Y7GFO3",
    )
    .unwrap();
    let signature = "untrusted comment: signature from minisign secret key
RUQf6LRCGA9i559r3g7V1qNyJDApGip8MfqcadIgT9CuhV3EMhHoN1mGTkUidF/z7SrlQgXdy8ofjb7bNJJylDOocrCo8KLzZwo=
trusted comment: timestamp:1633700835\tfile:test\tprehashed
wLMDjy9FLAuxZ3q4NlEvkgtyhrr0gtTu6KC4KBJdITbbOeAi1zBIYo0v4iTgt8jJpIidRJnp94ABQkJAgAooBQ==
";
    let check = |image_name: &str, image: &[u8]| {
        verify_signature(
            &key,
            "test.minisig",
            signature.as_bytes(),
            image_name,
            "v1.0.0",
            image,
        )
    };
    assert_eq!(
        check("test", b"test"),
        Err(ReleaseError::SignatureNamesNoRelease {
            name: "test.minisig".into(),
            tag: "v1.0.0".into()
        })
    );
    assert!(matches!(
        check("test", b"tesT"),
        Err(ReleaseError::BadSignature { .. })
    ));
    assert!(matches!(
        check("other", b"test"),
        Err(ReleaseError::SignatureNamesAnother { .. })
    ));
}

/// A legacy signature, of the image itself rather than its hash, from minisign-verify 0.3.0's
/// own test (`src/lib.rs`, lines 419 to 441), where it verifies with legacy signatures allowed.
/// verge allows only pre-hashed ones, so it is refused before its trusted comment is read.
#[test]
fn a_legacy_signature_is_refused() {
    let key = minisign_verify::PublicKey::from_base64(
        "RWQf6LRCGA9i53mlYecO4IzT51TGPpvWucNSCh1CBM0QTaLn73Y7GFO3",
    )
    .unwrap();
    let signature = "untrusted comment: signature from minisign secret key
RWQf6LRCGA9i59SLOFxz6NxvASXDJeRtuZykwQepbDEGt87ig1BNpWaVWuNrm73YiIiJbq71Wi+dP9eKL8OC351vwIasSSbXxwA=
trusted comment: timestamp:1555779966\tfile:test
QtKMXWyYcwdpZAlPF7tE2ENJkRd1ujvKjlj1m9RtHTBnZPa5WKU5uWRs5GoP5M/VqE81QFuMKI5k/SfNQUaOAA==
";
    let result = verify_signature(
        &key,
        "test.minisig",
        signature.as_bytes(),
        "test",
        "v1.0.0",
        b"test",
    );
    let Err(ReleaseError::BadSignature { reason, .. }) = &result else {
        panic!("{result:?}");
    };
    assert!(reason.contains("legacy signature"), "{reason}");
}

fn github(http: &SimHttp) -> GitHub<SimHttp> {
    GitHub::new(http.clone(), repo(), None).with_key(Some(http.public_key()))
}

fn image_bytes(build: &str) -> Vec<u8> {
    firmware_srecord(&SimFirmware::Ours {
        build: build.to_owned(),
    })
    .into_bytes()
}

fn sums(bytes: &[u8]) -> SimAsset {
    SimAsset::new(
        &format!("{IMAGE}.sha256"),
        format!("{}  {IMAGE}\n", hex::encode(Sha256::digest(bytes))).into_bytes(),
    )
}

fn signed(key: &SimKey, bytes: &[u8], comment: &str) -> SimAsset {
    key.sign(&format!("{IMAGE}.minisig"), bytes, comment)
}

/// The trusted comment the release workflow signs v0.3.0's image with.
fn comment() -> String {
    format!("file:{IMAGE} tag:v0.3.0")
}

/// Publishes v0.3.0 with these assets, and returns what downloading it gives.
fn download(http: &SimHttp, assets: Vec<SimAsset>) -> Result<(), ReleaseError> {
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets,
    });
    let github = github(http);
    let release = github.latest()?;
    github
        .download(&release, Download::Agreed, &mut Quiet)
        .map(|_| ())
}

#[test]
fn the_latest_release_downloads_and_checks_out() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.2.0");
    http.publish_ours("v0.3.0");
    http.publish(SimRelease {
        tag: "v0.4.0-rc1".into(),
        prerelease: true,
        assets: http.ours_assets("v0.4.0-rc1"),
    });
    let github = github(&http);
    let release = github.latest().unwrap();
    assert_eq!(release.tag, "v0.3.0");
    assert_eq!(release.image.name, IMAGE);
    let image = github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    assert_eq!(
        image.chunks(),
        firmware_image(&SimFirmware::Ours {
            build: "v0.3.0".into()
        })
        .chunks()
    );
    // Public downloads go to the public URLs and carry no token.
    assert!(http.requests().iter().all(|u| !u.contains("/assets/")));
    assert_eq!(github.tagged("v0.4.0-rc1").unwrap().tag, "v0.4.0-rc1");
}

#[test]
fn an_unsigned_release_is_refused_when_it_is_looked_up() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets: vec![SimAsset::new(IMAGE, bytes.clone()), sums(&bytes)],
    });
    let error = github(&http).latest().unwrap_err();
    assert_eq!(
        error,
        ReleaseError::Unsigned {
            tag: "v0.3.0".into(),
            name: format!("{IMAGE}.minisig")
        }
    );
    assert!(error.to_string().contains("it is not signed"), "{error}");
}

/// The case the signature exists for: the image, its checksum file and GitHub's digest all agree,
/// and only the signature knows the image is not what was signed.
#[test]
fn an_image_changed_after_signing_is_refused_though_its_checksums_agree() {
    let http = SimHttp::new(repo());
    let signed_bytes = image_bytes("v0.3.0");
    let other = image_bytes("v9.9.9");
    let signature = signed(http.key(), &signed_bytes, &comment());
    let error = download(
        &http,
        vec![SimAsset::new(IMAGE, other.clone()), sums(&other), signature],
    )
    .unwrap_err();
    let ReleaseError::BadSignature { reason, .. } = &error else {
        panic!("{error:?}");
    };
    assert!(reason.contains("changed after it was signed"), "{reason}");
}

#[test]
fn a_signature_by_another_key_is_refused() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let stranger = SimKey::generate();
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&bytes),
            signed(&stranger, &bytes, &comment()),
        ],
    )
    .unwrap_err();
    let ReleaseError::BadSignature { reason, .. } = &error else {
        panic!("{error:?}");
    };
    assert!(
        reason.contains("a key other than the one built into verge"),
        "{reason}"
    );
}

/// A validly signed image of one release, published again as another.
#[test]
fn a_signature_of_another_file_is_refused() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&bytes),
            signed(
                http.key(),
                &bytes,
                "file:mculink-verge-v0.2.0.s19 tag:v0.3.0",
            ),
        ],
    )
    .unwrap_err();
    assert_eq!(
        error,
        ReleaseError::SignatureNamesAnother {
            name: format!("{IMAGE}.minisig"),
            named: "mculink-verge-v0.2.0.s19".into(),
            image: IMAGE.into()
        }
    );

    let http = SimHttp::new(repo());
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&bytes),
            signed(http.key(), &bytes, "timestamp:1 tag:v0.3.0"),
        ],
    )
    .unwrap_err();
    assert!(
        matches!(error, ReleaseError::SignatureNamesNothing { .. }),
        "{error:?}"
    );
}

#[test]
fn a_signature_file_that_is_not_one_is_refused() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&bytes),
            SimAsset::new(&format!("{IMAGE}.minisig"), b"not a signature\n".to_vec()),
        ],
    )
    .unwrap_err();
    assert!(
        matches!(error, ReleaseError::BadSignatureFile { .. }),
        "{error:?}"
    );
}

#[test]
fn a_build_without_a_release_key_installs_no_release() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let github = GitHub::new(http.clone(), repo(), None).with_key(None);
    let release = github.latest().unwrap();
    assert_eq!(
        github
            .download(&release, Download::Agreed, &mut Quiet)
            .unwrap_err(),
        ReleaseError::NoKey
    );
    // Refused before any asset was fetched.
    assert!(http.requests().iter().all(|u| !u.contains("/download/")));
}

#[test]
fn an_image_that_does_not_match_its_checksum_is_refused() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&image_bytes("v0.2.0")),
            signed(http.key(), &bytes, &comment()),
        ],
    )
    .unwrap_err();
    assert!(
        matches!(error, ReleaseError::ChecksumMismatch { .. }),
        "{error:?}"
    );
}

#[test]
fn an_image_that_does_not_match_githubs_digest_is_refused() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let image = SimAsset {
        digest: SimAsset::new(IMAGE, image_bytes("v0.2.0")).digest,
        ..SimAsset::new(IMAGE, bytes.clone())
    };
    let error = download(
        &http,
        vec![image, sums(&bytes), signed(http.key(), &bytes, &comment())],
    )
    .unwrap_err();
    assert!(
        matches!(error, ReleaseError::DigestMismatch { .. }),
        "{error:?}"
    );
}

#[test]
fn releases_that_break_the_convention_are_refused() {
    let http = SimHttp::new(repo());
    let mut no_sums = http.ours_assets("v0.3.0");
    no_sums.retain(|a| !a.name.ends_with(".sha256"));
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets: no_sums,
    });
    assert!(matches!(
        github(&http).latest(),
        Err(ReleaseError::NoChecksum { .. })
    ));

    let http = SimHttp::new(repo());
    let mut two_images = http.ours_assets("v0.3.0");
    two_images.push(SimAsset::new("armwire.s19", b"S0".to_vec()));
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets: two_images,
    });
    assert_eq!(
        github(&http).latest(),
        Err(ReleaseError::ImageAsset {
            tag: "v0.3.0".into(),
            found: 2
        })
    );

    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    let digest = hex::encode(Sha256::digest(&bytes));
    let error = download(
        &http,
        vec![
            SimAsset::new(IMAGE, bytes.clone()),
            SimAsset::new(
                &format!("{IMAGE}.sha256"),
                format!("{digest}  something-else.s19\n").into_bytes(),
            ),
            signed(http.key(), &bytes, &comment()),
        ],
    )
    .unwrap_err();
    assert!(
        matches!(error, ReleaseError::ChecksumNamesAnother { .. }),
        "{error:?}"
    );
}

#[test]
fn a_missing_release_and_a_private_repository() {
    let http = SimHttp::new(repo());
    assert_eq!(
        github(&http).latest(),
        Err(ReleaseError::NotFound {
            repo: repo().to_string(),
            tag: None
        })
    );
    http.publish_ours("v0.3.0");
    http.set_private(true);
    assert!(matches!(
        github(&http).latest(),
        Err(ReleaseError::NotFound { .. })
    ));
    let with_token =
        GitHub::new(http.clone(), repo(), Some("t".into())).with_key(Some(http.public_key()));
    let release = with_token.latest().unwrap();
    with_token
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    assert!(
        http.requests()
            .iter()
            .any(|u| u.contains("/releases/assets/"))
    );
}

/// Nothing reaches the bus when the release fails its checks: loading the image comes first.
#[test]
fn a_bad_signature_flashes_nothing() {
    let http = SimHttp::new(repo());
    let bytes = image_bytes("v0.3.0");
    http.publish(SimRelease {
        tag: "v0.3.0".into(),
        prerelease: false,
        assets: vec![
            SimAsset::new(IMAGE, bytes.clone()),
            sums(&bytes),
            signed(&SimKey::generate(), &bytes, &comment()),
        ],
    });
    let bus = SimBus::scenario("ours").unwrap();
    let units = detect(&bus).unwrap();
    let before = bus.log().len();
    let result = source::load(
        &Source::Release(None),
        &github(&http),
        Some(&units[0]),
        false,
        Download::Agreed,
        &mut Quiet,
    );
    assert!(
        matches!(
            result,
            Err(LoadError::Release(ReleaseError::BadSignature { .. }))
        ),
        "{result:?}"
    );
    assert_eq!(bus.log().len(), before, "{:?}", &bus.log()[before..]);
}

#[test]
fn from_release_to_running() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let bus = SimBus::scenario("bench").unwrap();
    let units = detect(&bus).unwrap();
    let unit = update::select(&units, Some("CJ3E7A91C2")).unwrap();

    let github = github(&http);
    let release = github.latest().unwrap();
    assert_eq!(
        unit.build().unwrap().compare(&release.tag),
        Relation::Unknown
    );
    let image = github
        .download(&release, Download::Agreed, &mut Quiet)
        .unwrap();
    let version = source::ImageVersion::Ours(release.tag.clone());
    let outcome = update::update(
        &bus,
        unit,
        source::Payload::new(&image, &version),
        &Timing::default(),
        &mut Quiet,
        &Cancel::new(),
    )
    .unwrap();
    let Outcome::Running(back) = outcome else {
        panic!("{outcome:?}");
    };
    let State::Probe {
        firmware: Firmware::Ours(ours),
        ..
    } = &back.state
    else {
        panic!("{:?}", back.state);
    };
    assert_eq!(ours.build, release.tag);
    assert_eq!(
        back.build().unwrap().compare(&release.tag),
        Relation::UpToDate
    );
}

/// release.rs: "naming the file there stops a validly signed image being published again as
/// another release". Re-publish v0.2.0's three signed assets as release v0.9.0.
#[test]
fn an_old_signed_image_republished_under_a_newer_tag_is_refused() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.5.0");
    let old = http.ours_assets("v0.2.0");
    http.publish(SimRelease {
        tag: "v0.9.0".into(),
        prerelease: false,
        assets: old,
    });
    let bus = SimBus::default();
    bus.add_ours("v0.5.0");
    let unit = detect(&bus).unwrap().remove(0);
    let loaded = source::load(
        &Source::Release(None),
        &github(&http),
        Some(&unit),
        false,
        Download::Agreed,
        &mut Quiet,
    );
    match loaded {
        Ok(loaded) => {
            let is_old = loaded.image.chunks()
                == firmware_image(&SimFirmware::Ours {
                    build: "v0.2.0".into(),
                })
                .chunks();
            panic!(
                "installed as {:?}, {}; image is v0.2.0's: {is_old}",
                loaded.version, loaded.name
            );
        }
        Err(LoadError::Release(ReleaseError::SignatureForAnotherRelease {
            signed, tag, ..
        })) => {
            assert_eq!((signed.as_str(), tag.as_str()), ("v0.2.0", "v0.9.0"));
        }
        Err(other) => panic!("refused, but not for its tag: {other}"),
    }
}

/// The two ways a trusted comment can fail to name the release: another tag, and none.
#[test]
fn a_signature_for_another_release_or_none_is_refused() {
    let bytes = image_bytes("v0.3.0");
    for (signed_comment, expected) in [
        (
            format!("file:{IMAGE} tag:v0.2.0"),
            ReleaseError::SignatureForAnotherRelease {
                name: format!("{IMAGE}.minisig"),
                signed: "v0.2.0".into(),
                tag: "v0.3.0".into(),
            },
        ),
        (
            format!("file:{IMAGE}"),
            ReleaseError::SignatureNamesNoRelease {
                name: format!("{IMAGE}.minisig"),
                tag: "v0.3.0".into(),
            },
        ),
    ] {
        let http = SimHttp::new(repo());
        let error = download(
            &http,
            vec![
                SimAsset::new(IMAGE, bytes.clone()),
                sums(&bytes),
                signed(http.key(), &bytes, &signed_comment),
            ],
        )
        .unwrap_err();
        assert_eq!(error, expected, "{signed_comment}");
    }
}

/// A server on this machine that takes one request and stalls: before its reply, or partway
/// through the body. It holds the connection for five seconds, which bounds a test without
/// timeouts.
fn stalling_server(headers: bool) -> String {
    use std::io::{Read, Write};
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}/image.s19", listener.local_addr().unwrap());
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let mut request = [0; 1024];
        let _ = stream.read(&mut request);
        if headers {
            let _ =
                stream.write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 1000\r\n\r\nS0030000FC\n");
        }
        std::thread::sleep(std::time::Duration::from_secs(5));
    });
    url
}

/// HTTP timeouts are real time, so this test is bounded in real time.
#[test]
fn a_stalled_request_fails_with_a_message() {
    use std::time::{Duration, Instant};
    let http = Ureq::new(Timeouts {
        connect: Duration::from_secs(2),
        response: Duration::from_millis(300),
        body: Duration::from_millis(300),
    });
    for (headers, phase) in [(false, "receive response"), (true, "receive body")] {
        let url = stalling_server(headers);
        let started = Instant::now();
        let error = http.get(&url, &[]).unwrap_err();
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "{phase}: took {:?}",
            started.elapsed()
        );
        assert_eq!(
            error,
            HttpError::Timeout {
                url: url.clone(),
                phase: phase.into()
            }
        );
        assert!(error.to_string().contains("stalled"), "{error}");
    }
}

/// An image of ours goes on a board only where its declaration names it, and an image that
/// declares nothing names the full MCU-Link alone. A release says so only once it is downloaded.
/// Stock's own image goes on any board.
#[test]
fn an_image_of_ours_goes_only_on_a_board_it_declares() {
    const PLAIN: u32 = 1 << 0;
    const PRO: u32 = 1 << 1;
    let pro_bus = SimBus::scenario("pro").unwrap();
    let pro = detect(&pro_bus).unwrap().remove(0);
    let plain_bus = SimBus::scenario("stock").unwrap();
    let plain = detect(&plain_bus).unwrap().remove(0);
    let not_for = |board: &str, built_for: &str| LoadError::NotForBoard {
        board: board.into(),
        built_for: built_for.into(),
    };
    let cases: [(&str, &verge_probe::Unit, Option<u32>, Option<LoadError>); 7] = [
        (
            "pro, nothing declared",
            &pro,
            None,
            Some(not_for("an MCU-Link Pro", "the full MCU-Link only")),
        ),
        ("pro, the pro declared", &pro, Some(PRO), None),
        ("pro, both declared", &pro, Some(PLAIN | PRO), None),
        // A bit for a board this tool does not know names nothing.
        (
            "pro, board 5 declared",
            &pro,
            Some(1 << 5),
            Some(not_for("an MCU-Link Pro", "no board this tool knows")),
        ),
        ("plain, nothing declared", &plain, None, None),
        (
            "plain, the pro declared",
            &plain,
            Some(PRO),
            Some(not_for("a full MCU-Link", "the MCU-Link Pro only")),
        ),
        ("plain, both declared", &plain, Some(PLAIN | PRO), None),
    ];
    for (case, unit, boards, refused) in cases {
        let http = SimHttp::new(repo());
        http.publish_ours_declaring("v0.3.0", boards);
        let file = std::env::temp_dir().join(format!("verge-board-{}.s19", std::process::id()));
        let image = match boards {
            Some(boards) => verge_probe::sim::firmware_srecord_declaring(
                &SimFirmware::Ours {
                    build: "v0.3.0".into(),
                },
                boards,
            ),
            None => String::from_utf8(image_bytes("v0.3.0")).unwrap(),
        };
        std::fs::write(&file, image).unwrap();
        for source in [Source::Release(None), Source::File(file.clone())] {
            let result = source::load(
                &source,
                &github(&http),
                Some(unit),
                false,
                Download::Agreed,
                &mut Quiet,
            );
            match (&result, &refused) {
                (Ok(_), None) => {}
                (Err(error), Some(expected)) if error.to_string() == expected.to_string() => {}
                _ => panic!("{case}, {source:?}: {:?}", result.map(|l| l.name)),
            }
        }
        std::fs::remove_file(&file).unwrap();
    }
}

/// A board no image of ours can name is refused before anything is downloaded, and stock's own
/// image still goes on it.
#[test]
fn a_board_no_image_can_name_is_refused_before_the_download() {
    let http = SimHttp::new(repo());
    http.publish_ours_declaring("v0.3.0", Some(u32::MAX));
    let bus = SimBus::default();
    let unit = bus.add_ours("v0.2.0");
    bus.set_board(unit, verge_probe::dap::Ours::BOARD_ON_BOARD);
    let unit = detect(&bus).unwrap().remove(0);
    let result = source::load(
        &Source::Release(None),
        &github(&http),
        Some(&unit),
        false,
        Download::Agreed,
        &mut Quiet,
    );
    let Err(LoadError::OtherBoard { board }) = &result else {
        panic!("{:?}", result.map(|l| l.name));
    };
    assert_eq!(board, "an on-board MCU-Link");
    assert!(http.requests().is_empty(), "{:?}", http.requests());

    let dir = std::env::temp_dir().join(format!("verge-pro-stock-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("MCU-LINK_CMSIS-DAP_V3_172.s19"),
        firmware_srecord(&SimFirmware::Stock {
            version: verge_probe::version::StockVersion::parse("3.172").unwrap(),
        }),
    )
    .unwrap();
    for unit in [
        unit,
        detect(&SimBus::scenario("pro").unwrap()).unwrap().remove(0),
    ] {
        let stock = source::load(
            &Source::Stock(Some(dir.clone())),
            &github(&http),
            Some(&unit),
            false,
            Download::CachedOnly,
            &mut Quiet,
        );
        assert!(stock.is_ok(), "{:?}", stock.map(|l| l.name));
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

/// Nothing is downloaded until the user agrees: `CachedOnly` fetches nothing, a no to the question
/// fetches nothing, and a build without the release key refuses before it asks.
#[test]
fn a_release_is_downloaded_only_when_agreed() {
    let http = SimHttp::new(repo());
    http.publish_ours("v0.3.0");
    let github = github(&http);
    let release = github.latest().unwrap();
    let downloads = || {
        http.requests()
            .iter()
            .filter(|url| url.contains("/download/"))
            .count()
    };
    assert_eq!(
        github
            .download(&release, Download::CachedOnly, &mut Quiet)
            .unwrap_err(),
        ReleaseError::NotAgreed {
            tag: "v0.3.0".into()
        }
    );
    let load = |github: &GitHub<SimHttp>, answer: Result<(), &'static str>| {
        let mut asked = Vec::new();
        let loaded = source::load_asking(
            &Source::Release(None),
            github,
            None,
            true,
            |what: &str| {
                asked.push(what.to_owned());
                answer
            },
            &mut Quiet,
        );
        (loaded, asked)
    };

    let (loaded, asked) = load(&github, Err("no"));
    assert_eq!(loaded.unwrap_err(), "no");
    let what = format!(
        "release v0.3.0 of someone/mculink-verge from GitHub ({:.1} KiB)",
        release.download_size() as f64 / 1024.0
    );
    assert_eq!(asked, [what]);
    assert_eq!(downloads(), 0);
    let (loaded, _) = load(&github, Ok(()));
    loaded.unwrap().unwrap();
    assert_eq!(downloads(), 3);

    let (loaded, asked) = load(&github.clone().with_key(None), Ok(()));
    assert!(
        matches!(loaded, Ok(Err(LoadError::Release(ReleaseError::NoKey)))),
        "{:?}",
        loaded.map(|l| l.map(|l| l.name))
    );
    assert!(asked.is_empty());
    assert_eq!(downloads(), 3);
}
