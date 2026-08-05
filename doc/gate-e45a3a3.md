=== gate at e45a3a3 ===

Full pre-push scope, run from the primary checkout (not a caucus worktree,
so `third_party/` is present). Covers the three merges landed at
`f2e0b50` (p1-robotmodel), `ba9497c` (p3-acm) and the earlier `8b8b260`
(p9-ros) / `faeea3c` (p1-fixtures).

--- cargo fmt --all -- --check ---
rc=0

--- cargo clippy --workspace --all-targets -- -D warnings ---
rc=0, zero error/warning lines

--- cargo nextest run --workspace ---
1680 tests run: 1680 passed, 2 skipped

--- cargo test --doc --workspace ---
all test-result lines ok, 0 failed

--- tools/ci/verify-private-doc-links.sh ---
rc=0  "OK private-item doc links resolve"

--- tools/ci/verify-clean-checkout.sh ---
rc=0  "OK: every ci.yml step passes on a fresh checkout with no third_party/"

--- sg docker -c tools/ci/verify-ros-interop.sh ---
rc=0  "all gates passed"

`ros/moveit-ros` is a separate `[workspace]` (D5) that the root-workspace
gates cannot see. A host-side `cargo clippy` there fails with
`failed to open: ros/moveit-ros/target/debug/.cargo-build-lock:
Permission denied` -- that target dir is root-owned from the docker build,
so the docker gate above is the only way to run it, and it must be wrapped
in `sg docker -c` with absolute paths or a failure reads as success.

Not covered by any of the above: the 252 orphan sites
(`doc/assertion-discrimination-ledger-p3-acm.md`) are sites the assertion
scanner sees that no ledger has triaged. Gates passing says nothing about
them -- a blind assertion passes.
