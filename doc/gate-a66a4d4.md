=== gate at a66a4d4 ===

Full pre-push scope, run from the primary checkout (not a caucus worktree,
so `third_party/` is present). Covers the four merges that close the
assertion-discrimination sweep: `b68d545` (p1-fixtures), `b8d2bd1`
(p9-ros), `810487a` (p1-robotmodel), `b926252` (p1-joints), plus the three
doc/JSON commits above them (`eeddbb4`, `d721ec1`, `a66a4d4`).

--- cargo fmt --all -- --check ---
rc=0

--- cargo clippy --workspace --all-targets -- -D warnings ---
rc=0, zero error/warning lines

--- cargo nextest run --workspace ---
1683 tests run: 1683 passed, 2 skipped

--- cargo test --doc --workspace ---
all test-result lines ok, 0 failed

The four cargo gates ran at `eeddbb4`. The two commits after it touch only
`doc/*.md` and `tools/ci/assertion-ledger-equivalences.json`; no `.rs` file
changed, so they are not re-run.

--- sg docker -c tools/ci/verify-all.sh ---
rc=0  "OK all 11 verify script(s) passed", including
  verify-ros-interop.sh      "all gates passed", 171/171 unit tests
  verify-orphan-enumeration.sh  "OK ... 0 sites, commit a66a4d4723c9"
  verify-clean-checkout.sh   fresh checkout with no third_party/
  verify-private-doc-links.sh, verify-upstream-license-provenance.sh,
  verify-fixture-{provenance,replay}.sh, verify-vendored-fixture-tests.sh,
  verify-oracle-sweep.sh (5 model/group sweeps, 10000 cases each),
  verify-mpr-vs-epa.sh, verify-continuous-reseed-wrap.sh

`ros/moveit-ros` is a separate `[workspace]` (D5) that the root-workspace
gates cannot see. A host-side `cargo clippy` there fails with
`failed to open: ros/moveit-ros/target/debug/.cargo-build-lock:
Permission denied` -- that target dir is root-owned from the docker build,
so the docker gate above is the only way to run it, and it must be wrapped
in `sg docker -c` with absolute paths or a failure reads as success.

The orphan caveat carried by `doc/gate-e45a3a3.md` is now closed: the
reconciler reports 700 scanner sites (excl. `helper_body`), 700 matched,
0 orphans, and 0 unresolved ledger citations. What that does *not* say is
that every triaged site is discriminating -- it says every site has a
verdict recorded, and the ledgers name which verdicts rest on a read and
which on an isolating mutation.
