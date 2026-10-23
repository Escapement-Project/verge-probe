# verge-probe

Host-side operations for probes running Verge firmware. The crate uses probe-rs's generic vendor
command API for firmware info (`0x82`), cJTAG framing (`0x83`) and energy capture (`0x90`).

Install cJTAG support before selecting the cJTAG wire protocol. Use `VergeProbe` for firmware info
and energy operations:

```rust
use verge_probe::{VergeProbe, install_cjtag_support};

install_cjtag_support(&mut probe)?;
let mut verge = VergeProbe::new(&mut probe);
let info = verge.info()?;
```

Check this crate from a checkout that also has the probe-rs working tree:

```sh
cargo check --manifest-path probe/Cargo.toml
```

The manifest points to that neighboring checkout because probe-rs's vendor-command API is not
released yet. Replace the local path with the probe-rs revision or release that contains
`Probe::send_vendor_command` and `CmsisDapCjtagExtension` before building this crate elsewhere.
