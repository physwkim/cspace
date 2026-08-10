#!/bin/bash
# `crates/cspace-collision/examples/probe_octree_leaf_contact_noise_floor.rs`
# had no caller anywhere in this repository before this script: `rg -n
# probe_octree_leaf_contact_noise_floor` outside the example's own file
# returns nothing, and its own header names only a hand-typed `cargo run`
# invocation for a person to run.
#
# # Why the example's own exit code is not enough
#
# The probe's `main()` panics (nonzero exit) only if it fails to reproduce
# the known octree "case 4" bit for bit (`CASE4_KNOWN =
# 4.129349354679189e-17`) -- a harness self-check, not the finding. The
# actual question the probe investigates -- whether some *other* reachable
# (resolution, box_half, axis) combination produces a residual outside the
# `~1e-14` noise floor, which `probe_gjk_positive_gap_boundary.rs` showed
# happens for curved/rotated shape pairs -- is answered by a printed
# "OUTLIERS FOUND" vs "NONE" line, not by the process exit status: `main()`
# returns normally either way (see its own tail: `over_floor.is_empty()`
# only selects which message to print). A future change that introduces a
# real outlier would exit 0 and this instrument would keep reporting
# nothing, forever, unless something reads its stdout. This script is that
# reader.
#
# # What this checks
#
#  - the process exits 0 (a nonzero exit is the case-4 harness-fidelity
#    panic itself -- treated as a hard failure, since nothing else the probe
#    measures can be trusted once that happens, per the probe's own panic
#    message)
#  - the main 288-configuration sweep reports "NONE" outliers, not
#    "OUTLIERS FOUND" -- the probe's own documented 2026 result
#    (`|dist| <= 1.07e-16` for all 288) is exactly this outcome
#  - that sweep's own None-count line reads "0" -- the documented result
#    reports every one of the 288 as a real `Some(dist)`, none of them the
#    `None` the probe can also legitimately produce (and does, deliberately,
#    in the separate leaf-origin sweep at extreme leaf coordinates the main
#    sweep never reaches)
#  - the swept-configuration count line still reads "288" -- catches a
#    silent change to the resolution/box_half/axis grids the probe's own
#    "Result" section describes without the description being updated
#
# This is a correctness invariant check, not a full-output transcript diff
# against a committed baseline (unlike `check-phase8-condition2-grid.sh`):
# no transcript could be captured for that pattern without first running the
# probe, which this session withheld (see commit message). A future run that
# wants tighter coverage of the full per-configuration table can add one.
#
# # State of this gate: UNEXECUTED
#
# Written and reviewed, never run -- same state `verify-phase3-penetration-
# extended.sh`, `verify-phase3-mesh-collision-bool.sh` and
# `verify-requirement-message-closure.sh` document for themselves.
#
# Needs neither docker nor the oracle -- pure Rust, this crate's own code --
# but a `cargo build`, so per `verify-sampler-self-validation.sh`'s own
# stated reasoning this is `verify-*.sh`, not `check-*.sh`: real time added
# per invocation is not part of the per-push `check-*` budget.
#
#   tools/ci/verify-octree-leaf-contact-noise-floor.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT
out="$OUT_DIR/probe.out"

echo
echo "=== octree leaf x robot-box contact, noise-floor probe ==="
echo

# Redirected to a file, never piped: a pipeline reports the filter's status,
# which is how a panic becomes a silent pass.
cargo run --release -p cspace-collision --example probe_octree_leaf_contact_noise_floor \
  >"$out" 2>&1
rc=$?

cat "$out"
echo

fail=0

if [ "$rc" -ne 0 ]; then
  echo "FAIL probe exited $rc -- see output above (likely the case-4 harness-fidelity panic)." >&2
  exit 1
fi

if ! grep -qE '^[0-9]+ configurations swept' "$out"; then
  echo "FAIL expected sweep-count line not found -- probe's output shape changed." >&2
  fail=1
elif ! grep -qE '^288 configurations swept' "$out"; then
  echo "FAIL sweep grid no longer covers 288 configurations -- see output above." >&2
  fail=1
else
  echo "ok   288 configurations swept, as documented"
fi

if ! grep -qE '^dist == None \(parry reported no contact at all\): 0$' "$out"; then
  echo "FAIL main sweep reported a None (no-contact) result -- the documented" >&2
  echo "  result is 0 for all 288 configurations; see output above." >&2
  fail=1
else
  echo "ok   main sweep: zero None (no-contact) results"
fi

if grep -q '^  -> OUTLIERS FOUND' "$out"; then
  echo "FAIL outliers found outside the ~1e-14 noise floor -- latent defect candidate:" >&2
  sed -n '/^  -> OUTLIERS FOUND/,/^$/p' "$out" >&2
  fail=1
elif grep -q '^  -> NONE\.' "$out"; then
  echo "ok   no configuration exceeded the noise floor"
else
  echo "FAIL neither the NONE nor the OUTLIERS FOUND marker was found -- probe's output shape changed." >&2
  fail=1
fi

echo
if [ "$fail" -ne 0 ]; then
  echo "FAIL octree leaf contact noise-floor probe did not confirm the documented result -- see above." >&2
  exit 1
fi

echo "OK octree leaf x robot-box contact stays within the documented ~1e-14 noise floor."
