# verge-probe

Host-side operations for probes running Verge firmware. The crate builds on probe-rs and sends Verge
commands through the same attached session used for target access. It includes typed helpers for
firmware info and cJTAG scan-format selection (`0x82`–`0x83`), J8 pins (`0x84`), reboot (`0x85`),
target sensing (`0x86`), SWD sampling and code lab (`0x87`–`0x88`), SWD/JTAG code generation and
diagnostics (`0x89`–`0x8D`), DIO (`0x8E`), target power (`0x8F`) and energy capture (`0x90`). Code-lab
calls require firmware built with `CODE_LAB`. The generic command method keeps newer or specialized
firmware commands reachable through the same session.

Install cJTAG support on the opened probe before selecting the cJTAG wire protocol and attaching.
Then wrap the attached session when you need Verge operations. The wrapper also returns access to
the ordinary probe-rs session:

```rust
use probe_rs::Permissions;
use verge_probe::{VergeSession, install_cjtag_support, set_cjtag_scan_formats};

set_cjtag_scan_formats(&mut probe, scan_format_mask)?;
install_cjtag_support(&mut probe)?;
let mut session = probe.attach(target, Permissions::default())?;
let mut verge = VergeSession::new(&mut session);
let info = verge.info()?;
let mut core = verge.session().core(0)?;
```

Check this crate from a checkout that also has the probe-rs working tree:

```sh
cargo check --manifest-path probe/Cargo.toml
```

The manifest points to that neighboring checkout because the session vendor-command API and
CMSIS-DAP cJTAG extension are not released yet. Replace the local path with the probe-rs revision or
release that contains `Session::send_vendor_command` and `CmsisDapCjtagExtension` before building
this crate elsewhere.
