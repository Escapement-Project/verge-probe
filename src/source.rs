//! Where an image comes from: a release of ours, a file, or the user's LinkServer install.

use std::path::PathBuf;

use crate::board;
use crate::detect::{Hardware, State, Unit};
use crate::image::{Image, ImageError};
use crate::progress::Progress;
use crate::release::{Download, GitHub, Http, Release, ReleaseError, Repo};
use crate::stock;
use crate::version::Relation;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// A release of ours: the latest, or the one with this tag.
    Release(Option<String>),
    /// An ELF or S-record file.
    File(PathBuf),
    /// The newest stock image in LinkServer's usual places, or in this directory.
    Stock(Option<PathBuf>),
}

/// What an image is, as far as telling a unit that runs it already, and whether it is stock's
/// own, which goes on any board.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ImageVersion {
    /// A release of ours, by its tag.
    Ours(String),
    Stock(crate::version::StockVersion),
    /// A file: nothing says which build it is. It counts as ours.
    Unknown,
}

impl ImageVersion {
    #[must_use]
    pub fn is_stock(&self) -> bool {
        matches!(self, ImageVersion::Stock(_))
    }
}

/// An image ready to flash, and how to name it to the user.
#[derive(Clone, Debug)]
pub struct Loaded {
    pub image: Image,
    pub name: String,
    pub release: Option<Release>,
    pub version: ImageVersion,
}

impl Loaded {
    #[must_use]
    pub fn payload(&self) -> Payload<'_> {
        Payload::new(&self.image, &self.version)
    }
}

/// An image to install, and what it is: stock's own, which goes on any board, or ours, which
/// goes only on the boards it declares.
#[derive(Clone, Copy, Debug)]
pub struct Payload<'a> {
    pub image: &'a Image,
    pub version: &'a ImageVersion,
}

impl<'a> Payload<'a> {
    #[must_use]
    pub fn new(image: &'a Image, version: &'a ImageVersion) -> Payload<'a> {
        Payload { image, version }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("{path}: {error}")]
    File { path: String, error: ImageError },
    #[error(transparent)]
    Release(#[from] ReleaseError),
    #[error(
        "no MCU-LINK_CMSIS-DAP_V*.s19 ({searched}). Install NXP's LinkServer, or name its probe_firmware directory"
    )]
    NoStock { searched: String },
    #[error("the unit already runs {tag}")]
    AlreadyRuns { tag: String },
    #[error("the unit runs {build}, which is newer than release {tag}")]
    Newer { build: String, tag: String },
    #[error("the unit is {board}, which no image of ours supports. NXP's stock image can go on it")]
    OtherBoard { board: String },
    #[error(
        "the unit is {board}, and this image is built for {built_for}. NXP's stock image can go on it"
    )]
    NotForBoard { board: String, built_for: String },
    #[error("{name}: {error}")]
    Image { name: String, error: ImageError },
}

fn from_file(path: &std::path::Path) -> Result<Image, LoadError> {
    Image::from_path(path).map_err(|error| LoadError::File {
        path: path.display().to_string(),
        error,
    })
}

/// Finds the release a [`Source::Release`] names, without downloading it.
pub fn release<H: Http>(github: &GitHub<H>, tag: Option<&str>) -> Result<Release, ReleaseError> {
    match tag {
        None | Some("latest") => github.latest(),
        Some(tag) => github.tagged(tag),
    }
}

/// Whether installing `release` on `unit` is an update. A unit not running ours always takes it.
pub fn check_release(unit: Option<&Unit>, release: &Release, force: bool) -> Result<(), LoadError> {
    let Some(build) = unit.and_then(Unit::build) else {
        return Ok(());
    };
    match build.compare(&release.tag) {
        Relation::UpToDate if !force => Err(LoadError::AlreadyRuns {
            tag: release.tag.clone(),
        }),
        Relation::Ahead if !force => Err(LoadError::Newer {
            build: build.raw,
            tag: release.tag.clone(),
        }),
        _ => Ok(()),
    }
}

/// Whether `source`'s image may go on `unit`, as far as can be told before it is loaded: stock's
/// own goes anywhere stock runs, and no image of ours goes on a board the tool cannot number.
pub fn check_unit(unit: Option<&Unit>, source: &Source) -> Result<(), LoadError> {
    match unit.map(Unit::hardware) {
        Some(Hardware::Other(board)) if !matches!(source, Source::Stock(_)) => {
            Err(LoadError::OtherBoard { board })
        }
        _ => Ok(()),
    }
}

/// Whether `image` may go on `unit`. Stock's own image, as `version` says it is, goes anywhere
/// stock runs. Anything else is ours, a release or a file, and goes only on a board its
/// declaration names, or the full MCU-Link where it declares none.
///
/// A unit that does not say which board it is passes: one in ISP mode, under
/// [`crate::paths::ISP_IDENTITY_WARNING`], and a running one under [`board_unknown`], which the
/// front end puts to the user.
pub fn check_board(
    unit: Option<&Unit>,
    image: &Image,
    version: &ImageVersion,
) -> Result<(), LoadError> {
    let Some(unit) = unit.filter(|_| !version.is_stock()) else {
        return Ok(());
    };
    match unit.hardware() {
        Hardware::Unknown => Ok(()),
        Hardware::Other(board) => Err(LoadError::OtherBoard { board }),
        Hardware::Board(on) => {
            let boards = image.boards().map_err(|error| LoadError::Image {
                name: "the image".into(),
                error,
            })?;
            if boards.contains(&on) {
                Ok(())
            } else {
                Err(LoadError::NotForBoard {
                    board: on.a().to_owned(),
                    built_for: board::built_for(&boards),
                })
            }
        }
    }
}

/// What the question before putting an image of ours on `unit` has to say when the unit runs a
/// firmware that does not say which board it is. `None` where the board is known, and for a unit
/// in ISP mode, whose question says [`crate::paths::ISP_IDENTITY_WARNING`] instead.
#[must_use]
pub fn board_unknown(unit: &Unit) -> Option<&'static str> {
    let running = matches!(
        unit.state,
        State::Probe { .. } | State::Unreadable { .. } | State::SwdJumper { .. }
    );
    (running && unit.hardware() == Hardware::Unknown).then_some(
        "This unit's firmware does not say which board it is, so the tool cannot check that the image is built for it.",
    )
}

/// Loads the image `source` names. Refuses an image of ours for a unit it is not built for. For a
/// release, refuses one `unit` already runs or is ahead of unless `force`, before anything is
/// downloaded; then takes it from the cache or downloads it, as `download` allows
/// ([`GitHub::download`]).
pub fn load<H: Http>(
    source: &Source,
    github: &GitHub<H>,
    unit: Option<&Unit>,
    force: bool,
    download: Download,
    progress: &mut dyn Progress,
) -> Result<Loaded, LoadError> {
    let loaded = match source {
        Source::Release(tag) => {
            let release = find_release(github, tag.as_deref(), unit, force)?;
            return load_release(github, release, unit, download, progress);
        }
        Source::File(path) => {
            check_unit(unit, source)?;
            load_file(path)?
        }
        Source::Stock(dir) => load_stock(dir.as_ref())?,
    };
    check_board(unit, &loaded.image, &loaded.version)?;
    Ok(loaded)
}

/// [`load`], asking `agree` before a release is downloaded, with what downloading it fetches
/// ([`download_what`]). What is refused without a download, such as a build with no release key,
/// is refused before the question. A release whose copy in the cache checks out needs no
/// question. One the cache cannot give, because it holds none, cannot read it, or holds a copy
/// that fails its check (deleted where it can be), is downloaded afresh after the question.
/// `agree`'s error is the outer one: nothing is downloaded once it says no.
pub fn load_asking<H: Http, E>(
    source: &Source,
    github: &GitHub<H>,
    unit: Option<&Unit>,
    force: bool,
    agree: impl FnOnce(&str) -> Result<(), E>,
    progress: &mut dyn Progress,
) -> Result<Result<Loaded, LoadError>, E> {
    let Source::Release(tag) = source else {
        return Ok(load(
            source,
            github,
            unit,
            force,
            Download::CachedOnly,
            progress,
        ));
    };
    let release = match find_release(github, tag.as_deref(), unit, force) {
        Ok(release) => release,
        Err(error) => return Ok(Err(error)),
    };
    match load_release(
        github,
        release.clone(),
        unit,
        Download::CachedOnly,
        progress,
    ) {
        Err(LoadError::Release(
            error @ (ReleaseError::NotAgreed { .. } | ReleaseError::CachedCopyFailed { .. }),
        )) => {
            if let ReleaseError::CachedCopyFailed { .. } = error {
                tracing::warn!("{error}");
            }
            agree(&download_what(github.repo(), &release))?;
            Ok(load_release(
                github,
                release,
                unit,
                Download::Agreed,
                progress,
            ))
        }
        loaded => Ok(loaded),
    }
}

/// The release `tag` names, the latest without one, looked up and judged for `unit` before
/// anything is downloaded: refused for a board no image of ours can name, and for a unit that
/// runs it already or a newer build unless `force`.
fn find_release<H: Http>(
    github: &GitHub<H>,
    tag: Option<&str>,
    unit: Option<&Unit>,
    force: bool,
) -> Result<Release, LoadError> {
    check_unit(unit, &Source::Release(tag.map(str::to_owned)))?;
    let release = release(github, tag)?;
    check_release(unit, &release, force)?;
    Ok(release)
}

/// `release`'s image, from the cache or downloaded where `download` agrees to it, with its
/// signature and checksums checked, and refused for a unit whose board it does not declare.
fn load_release<H: Http>(
    github: &GitHub<H>,
    release: Release,
    unit: Option<&Unit>,
    download: Download,
    progress: &mut dyn Progress,
) -> Result<Loaded, LoadError> {
    let image = github.download(&release, download, progress)?;
    let loaded = Loaded {
        image,
        name: release_name(&release),
        version: ImageVersion::Ours(release.tag.clone()),
        release: Some(release),
    };
    check_board(unit, &loaded.image, &loaded.version)?;
    Ok(loaded)
}

/// A release's image as the questions and reports name it.
fn release_name(release: &Release) -> String {
    format!("release {} ({})", release.tag, release.image.name)
}

/// What downloading `release` of `repo` fetches, for the question that asks first: the
/// repository, the tag and how much.
#[must_use]
pub fn download_what(repo: &Repo, release: &Release) -> String {
    format!(
        "release {} of {repo} from GitHub ({:.1} KiB)",
        release.tag,
        release.download_size() as f64 / 1024.0
    )
}

/// The command that flashes this image onto a unit in ISP mode, for advice: the release by its
/// tag, the file by its path, or stock from where it was found.
#[must_use]
pub fn flash_command(source: &Source, version: &ImageVersion) -> String {
    match (source, version) {
        (Source::Release(_), ImageVersion::Ours(tag)) => format!("verge isp flash --release {tag}"),
        (Source::Release(Some(tag)), _) => format!("verge isp flash --release {tag}"),
        (Source::Release(None), _) => "verge isp flash --release".to_owned(),
        (Source::File(path), _) => format!("verge isp flash --file {}", shell_word(path)),
        (Source::Stock(Some(dir)), _) => format!("verge isp flash --stock {}", shell_word(dir)),
        (Source::Stock(None), _) => "verge isp flash --stock".to_owned(),
    }
}

/// A path as one word for a shell, quoted where it has to be: in single quotes for a POSIX shell,
/// in double quotes on Windows, where a path cannot hold one.
fn shell_word(path: &std::path::Path) -> String {
    let text = path.display().to_string();
    let plain = |c: char| {
        c.is_ascii_alphanumeric() || "/._-+:=@,".contains(c) || (cfg!(windows) && c == '\\')
    };
    if !text.is_empty() && text.chars().all(plain) {
        text
    } else if cfg!(windows) {
        format!("\"{text}\"")
    } else {
        format!("'{}'", text.replace('\'', "'\\''"))
    }
}

fn load_file(path: &std::path::Path) -> Result<Loaded, LoadError> {
    let image = from_file(path)?;
    image.check_for_mculink().map_err(|error| LoadError::File {
        path: path.display().to_string(),
        error,
    })?;
    let name = format!("{} ({})", path.display(), image.format);
    Ok(Loaded {
        image,
        name,
        release: None,
        version: ImageVersion::Unknown,
    })
}

fn load_stock(dir: Option<&PathBuf>) -> Result<Loaded, LoadError> {
    let dirs = match dir {
        Some(dir) => vec![dir.clone()],
        None => stock::search_dirs(),
    };
    let Some(found) = stock::find(&dirs).into_iter().next() else {
        let searched = if dirs.is_empty() {
            "found no LinkServer install".to_owned()
        } else {
            let names: Vec<String> = dirs.iter().map(|d| d.display().to_string()).collect();
            format!("looked in {}", names.join(", "))
        };
        return Err(LoadError::NoStock { searched });
    };
    let image = from_file(&found.path)?;
    Ok(Loaded {
        image,
        name: format!("stock V{} from {}", found.version, found.path.display()),
        release: None,
        version: ImageVersion::Stock(found.version),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A path a shell would split or misread is quoted; a plain one is left as it is.
    #[test]
    fn a_flash_command_quotes_a_path_only_where_it_has_to() {
        let release = ImageVersion::Ours("v0.3.0".into());
        assert_eq!(
            flash_command(&Source::Release(None), &release),
            "verge isp flash --release v0.3.0"
        );
        let file = |path: &str| flash_command(&Source::File(path.into()), &ImageVersion::Unknown);
        if cfg!(windows) {
            assert_eq!(
                file(r"C:\NXP\mculink.s19"),
                r"verge isp flash --file C:\NXP\mculink.s19"
            );
            assert_eq!(
                file(r"C:\My images\mculink.s19"),
                r#"verge isp flash --file "C:\My images\mculink.s19""#
            );
        } else {
            assert_eq!(
                file("/tmp/build/mculink.s19"),
                "verge isp flash --file /tmp/build/mculink.s19"
            );
            assert_eq!(
                file("/tmp/my images/it's.s19"),
                r"verge isp flash --file '/tmp/my images/it'\''s.s19'"
            );
            assert_eq!(
                flash_command(
                    &Source::Stock(Some("/opt/Link Server/probe_firmware".into())),
                    &ImageVersion::Unknown
                ),
                "verge isp flash --stock '/opt/Link Server/probe_firmware'"
            );
        }
    }
}
