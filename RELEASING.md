# Releasing verge-probe

`verge-probe` has its own SemVer version and Git tags. Tag each release as `v<version>`.
The `verge` CLI and GUI should consume an exact tag or commit, not a floating branch.

Before the first release, make the probe-rs vendor-command and cJTAG API commits reachable from a
stable Git remote, then pin their exact commit in this crate. The current sibling-path dependency is
for local development and does not support a clean checkout on its own.
