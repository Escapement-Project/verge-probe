//! The USB IDs this tool acts on, in one table.
//!
//! Ours shares stock's ID for now. When it moves to its own pid.codes ID (VID `0x1209`), add a
//! row here; nothing else in the tool names an ID.

/// What a USB ID says a unit is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Role {
    /// Running a CMSIS-DAP firmware, stock or ours.
    Probe,
    /// The LPC55S69 ROM bootloader: ISP mode.
    Bootloader,
    /// An MCU-Link mini in ISP mode. A different board and image, so the tool refuses it.
    MiniBootloader,
}

/// One row of [`KNOWN_IDS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KnownId {
    pub vid: u16,
    pub pid: u16,
    pub role: Role,
}

/// Every USB ID the tool recognises.
///
/// Every LPC55S69 ROM enumerates as `1fc9:0021`, which is why two units in ISP mode cannot be
/// told apart.
pub const KNOWN_IDS: &[KnownId] = &[
    KnownId {
        vid: 0x1fc9,
        pid: 0x0143,
        role: Role::Probe,
    },
    KnownId {
        vid: 0x1fc9,
        pid: 0x0021,
        role: Role::Bootloader,
    },
    KnownId {
        vid: 0x1fc9,
        pid: 0x0022,
        role: Role::MiniBootloader,
    },
];

/// The role of a USB ID, or `None` for a device this tool ignores.
#[must_use]
pub fn role(vid: u16, pid: u16) -> Option<Role> {
    KNOWN_IDS
        .iter()
        .find(|id| id.vid == vid && id.pid == pid)
        .map(|id| id.role)
}

/// The first ID with this role, for the code that has to name one, such as opening the ROM.
#[must_use]
pub fn id_for(role: Role) -> (u16, u16) {
    let id = KNOWN_IDS
        .iter()
        .find(|id| id.role == role)
        .expect("every role has a row in KNOWN_IDS");
    (id.vid, id.pid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roles_by_id() {
        assert_eq!(role(0x1fc9, 0x0143), Some(Role::Probe));
        assert_eq!(role(0x1fc9, 0x0021), Some(Role::Bootloader));
        assert_eq!(role(0x1fc9, 0x0022), Some(Role::MiniBootloader));
        assert_eq!(role(0x1fc9, 0x0023), None);
        assert_eq!(role(0x0d28, 0x0204), None);
    }

    #[test]
    fn every_role_has_an_id() {
        for role in [Role::Probe, Role::Bootloader, Role::MiniBootloader] {
            let (vid, pid) = id_for(role);
            assert_eq!(super::role(vid, pid), Some(role));
        }
    }
}
