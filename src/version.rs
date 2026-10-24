//! Firmware versions: stock's release number, and the build string ours reports.

use std::cmp::Ordering;
use std::fmt;

/// A stock firmware release number, such as `3.172`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StockVersion {
    pub major: u16,
    pub minor: u16,
}

impl StockVersion {
    /// The first stock release NXP's tooling updates without the jumper.
    pub const JUMPER_FREE: StockVersion = StockVersion {
        major: 3,
        minor: 122,
    };

    /// Parses `3.172`, as stock reports in `DAP_Info` 0x09.
    #[must_use]
    pub fn parse(text: &str) -> Option<StockVersion> {
        let (major, minor) = text.trim().split_once('.')?;
        Some(StockVersion {
            major: major.parse().ok()?,
            minor: minor.parse().ok()?,
        })
    }

    /// Finds a `V3.172` token in a USB product string such as `MCU-LINK (r0FB) CMSIS-DAP V3.172`.
    #[must_use]
    pub fn from_product_string(product: &str) -> Option<StockVersion> {
        product
            .split_whitespace()
            .rev()
            .find_map(|word| word.strip_prefix(['V', 'v']).and_then(StockVersion::parse))
    }

    /// Whether this release takes the jumper-free entry into the bootloader.
    #[must_use]
    pub fn has_jumper_free_update(self) -> bool {
        self >= Self::JUMPER_FREE
    }
}

impl fmt::Display for StockVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// What `git describe --always --dirty --tags` said when the firmware was built.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Describe {
    /// Built exactly at a tag.
    Tag(String),
    /// Built `commits` commits after `tag`, at `hash`.
    AfterTag {
        tag: String,
        commits: u32,
        hash: String,
    },
    /// A bare abbreviated hash: no tag was reachable when it was built.
    Hash(String),
    /// The build could not run `git describe` and wrote `unknown`.
    Unknown,
}

/// The build string our firmware reports, parsed.
///
/// The firmware appends a variant to `git describe`, such as `+armwire` for the build with the
/// generated transfer paths off, so `v1.0.0-dirty+armwire` is a valid string.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Build {
    pub raw: String,
    pub describe: Describe,
    pub dirty: bool,
    pub variant: Option<String>,
}

impl Build {
    #[must_use]
    pub fn parse(raw: &str) -> Build {
        let raw = raw.trim();
        let (rest, variant) = match raw.rsplit_once('+') {
            Some((rest, variant)) if !variant.is_empty() => (rest, Some(variant.to_owned())),
            _ => (raw, None),
        };
        let (rest, dirty) = match rest.strip_suffix("-dirty") {
            Some(rest) => (rest, true),
            None => (rest, false),
        };
        Build {
            raw: raw.to_owned(),
            describe: parse_describe(rest),
            dirty,
            variant,
        }
    }

    /// The tag this build is at or after, if git found one.
    #[must_use]
    pub fn tag(&self) -> Option<&str> {
        match &self.describe {
            Describe::Tag(tag) | Describe::AfterTag { tag, .. } => Some(tag),
            Describe::Hash(_) | Describe::Unknown => None,
        }
    }

    /// How this build stands against a release tag.
    #[must_use]
    pub fn compare(&self, release_tag: &str) -> Relation {
        let Some(tag) = self.tag() else {
            return Relation::Unknown;
        };
        let exactly_at_tag = matches!(self.describe, Describe::Tag(_)) && !self.dirty;
        if tag == release_tag {
            return match (exactly_at_tag, &self.variant) {
                (true, None) => Relation::UpToDate,
                (true, Some(_)) => Relation::OtherVariant,
                (false, _) => Relation::Ahead,
            };
        }
        match compare_tags(tag, release_tag) {
            Some(Ordering::Less) => Relation::Behind,
            Some(Ordering::Greater) => Relation::Ahead,
            // Two different strings that parse to the same version, such as `v1.0` and `1.0.0`.
            Some(Ordering::Equal) | None => Relation::Unknown,
        }
    }
}

impl fmt::Display for Build {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.raw)
    }
}

fn parse_describe(text: &str) -> Describe {
    if text.is_empty() || text == "unknown" {
        return Describe::Unknown;
    }
    if let Some((front, hash)) = text.rsplit_once("-g")
        && is_hash(hash)
        && let Some((tag, commits)) = front.rsplit_once('-')
        && let Ok(commits) = commits.parse()
        && !tag.is_empty()
    {
        return Describe::AfterTag {
            tag: tag.to_owned(),
            commits,
            hash: hash.to_owned(),
        };
    }
    if is_hash(text) {
        return Describe::Hash(text.to_owned());
    }
    Describe::Tag(text.to_owned())
}

fn is_hash(text: &str) -> bool {
    (4..=40).contains(&text.len())
        && text
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn compare_tags(a: &str, b: &str) -> Option<Ordering> {
    let parse = |tag: &str| semver::Version::parse(tag.strip_prefix('v').unwrap_or(tag)).ok();
    Some(parse(a)?.cmp(&parse(b)?))
}

/// How the firmware on a probe stands against a release, from the probe's side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Relation {
    /// The probe runs exactly this release.
    UpToDate,
    /// The probe runs this release's source built another way, such as `+armwire`.
    OtherVariant,
    /// The release is newer than what the probe runs.
    Behind,
    /// The probe runs something newer than the release, or uncommitted work on top of it.
    Ahead,
    /// Nothing says how the two compare: the build carries no tag, or a tag is not a version.
    Unknown,
}

impl Relation {
    /// Whether installing the release is an update rather than a downgrade or a no-op.
    #[must_use]
    pub fn offers_update(self) -> bool {
        matches!(
            self,
            Relation::Behind | Relation::OtherVariant | Relation::Unknown
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stock_versions() {
        assert_eq!(
            StockVersion::parse("3.172"),
            Some(StockVersion {
                major: 3,
                minor: 172
            })
        );
        assert_eq!(StockVersion::parse("3"), None);
        assert_eq!(StockVersion::parse("x.1"), None);
        assert_eq!(
            StockVersion::from_product_string("MCU-LINK (r0FB) CMSIS-DAP V3.172"),
            Some(StockVersion {
                major: 3,
                minor: 172
            })
        );
        assert_eq!(StockVersion::from_product_string("Verge CMSIS-DAP"), None);
    }

    #[test]
    fn jumper_free_starts_at_3_122() {
        let at = |minor| StockVersion { major: 3, minor };
        assert!(!at(121).has_jumper_free_update());
        assert!(at(122).has_jumper_free_update());
        assert!(at(172).has_jumper_free_update());
        // Numeric, not lexical: 3.99 is older than 3.122 although "99" sorts after "122".
        assert!(!at(99).has_jumper_free_update());
        assert!(
            !StockVersion {
                major: 2,
                minor: 245
            }
            .has_jumper_free_update()
        );
    }

    #[test]
    fn builds() {
        let build = Build::parse("14d05d4");
        assert_eq!(build.describe, Describe::Hash("14d05d4".into()));
        assert!(!build.dirty);
        assert_eq!(build.variant, None);

        let build = Build::parse("14d05d4-dirty+armwire");
        assert_eq!(build.describe, Describe::Hash("14d05d4".into()));
        assert!(build.dirty);
        assert_eq!(build.variant.as_deref(), Some("armwire"));

        assert_eq!(
            Build::parse("v0.2.0").describe,
            Describe::Tag("v0.2.0".into())
        );
        assert_eq!(
            Build::parse("v0.2.0-3-g14d05d4-dirty").describe,
            Describe::AfterTag {
                tag: "v0.2.0".into(),
                commits: 3,
                hash: "14d05d4".into()
            }
        );
        // A tag with dashes of its own keeps them.
        assert_eq!(
            Build::parse("v1.0.0-rc1-12-gabcdef0").describe,
            Describe::AfterTag {
                tag: "v1.0.0-rc1".into(),
                commits: 12,
                hash: "abcdef0".into()
            }
        );
        assert_eq!(Build::parse("unknown").describe, Describe::Unknown);
    }

    #[test]
    fn relations() {
        let rel = |build: &str, tag: &str| Build::parse(build).compare(tag);
        assert_eq!(rel("v0.2.0", "v0.2.0"), Relation::UpToDate);
        assert_eq!(rel("v0.2.0+armwire", "v0.2.0"), Relation::OtherVariant);
        assert_eq!(rel("v0.2.0-dirty", "v0.2.0"), Relation::Ahead);
        assert_eq!(rel("v0.2.0-4-gabcdef0", "v0.2.0"), Relation::Ahead);
        assert_eq!(rel("v0.1.9", "v0.2.0"), Relation::Behind);
        assert_eq!(rel("v0.1.9-4-gabcdef0", "v0.2.0"), Relation::Behind);
        assert_eq!(rel("v0.10.0", "v0.9.0"), Relation::Ahead);
        assert_eq!(rel("14d05d4", "v0.2.0"), Relation::Unknown);
        assert_eq!(rel("nightly", "v0.2.0"), Relation::Unknown);
        assert!(Relation::Behind.offers_update());
        assert!(!Relation::UpToDate.offers_update());
        assert!(!Relation::Ahead.offers_update());
    }
}
