#!/bin/bash
# Re-runs the libccd MPR vs. parry EPA comparison that closes deviation 6(b)
# and checks both numbers against the values `crates/moveit-collision/src/parry.rs`
# actually cites.
#
# PORTING-PLAN.md §200.1 recorded this as the open half of deviation 6: the
# closing argument rested on a number produced by an out-of-tree driver, so
# nothing in the repo could re-derive it and nothing would notice if it
# stopped holding. §201's rule is the reason this script exists at all --
# scaffolding may be deleted, but the evidence for a claim the tree depends
# on may not be.
#
# libccd has no system package here (`pkg-config --exists ccd` fails), so
# this gate needs a source checkout at tag v2.1. When there is none it
# SKIPs, loudly: a silent skip is indistinguishable from a pass, which is
# the failure `verify-vendored-fixture-tests.sh` documents at length.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

LIBCCD_SRC="${LIBCCD_SRC:-/home/stevek/work/libccd}"

# The two numbers `parry.rs`'s deviation-6(b) doc cites. Compared to a
# relative tolerance rather than exactly: libccd is built here from source
# by `build.sh`, and a different compiler or -O level can move the last
# bits of an iterative MPR refinement. `1e-9` is far tighter than the
# ~7.3ppm gap between libccd and the oracle that the doc's own argument
# turns on, so a drift large enough to matter still fails.
EXPECTED_MPR='7.47919999515277989e-02'
EXPECTED_EPA='-0.020869698793459224'
REL_TOL='1e-9'

if [[ ! -d "$LIBCCD_SRC" ]]; then
  skip_not_measured blocked \
    "LIBCCD_SRC=$LIBCCD_SRC not present -- the MPR side of deviation 6(b) is not re-derived." \
    "this is not a pass; check out https://github.com/danfis/libccd at tag v2.1 to cover it."
fi

tag="$(git -C "$LIBCCD_SRC" describe --tags --exact-match 2>/dev/null || true)"
if [[ "$tag" != "v2.1" ]]; then
  skip_not_measured blocked \
    "$LIBCCD_SRC is at '${tag:-<none>}', not tag v2.1 -- the harness is pinned to v2.1." \
    "this is not a pass; check out v2.1 to cover deviation 6(b)'s MPR side."
fi

LIBCCD_SRC="$LIBCCD_SRC" ./tools/mpr-vs-epa/build.sh >/dev/null

# The Rust side prints the reconstructed geometry on stdout and its own EPA
# depth on stderr; both are checked, because a reconstruction that silently
# changed would otherwise feed libccd different inputs and still "agree".
#
# `cargo run` as its own statement, not chained with `&&` inside the
# `epa_line=` substitution: chained, a `cargo run` that succeeds but whose
# stderr no longer contains "EPA depth=" makes `grep` the pipeline's exit
# status, which under this script's own `set -e`/`pipefail` aborts *at the
# assignment* -- before python's `one()` below, which exists specifically to
# print "FAIL could not read EPA depth from: ..." for exactly this case, ever
# runs. Round 18's sweep found it: the same `test_status=$?`-shape dead
# handler 48ef7ce closed in ros/verify-ros-interop.sh, here via `&&` instead
# of `$?`. Splitting the statements gives `cargo run` its own honest abort
# (a real build/run failure still stops the script, with cargo's own output
# already on screen) and lets a merely-absent "EPA depth=" line reach the
# handler that was written for it.
cargo run --release --example case104_mpr_input -p moveit-collision \
  2>"$REPO_ROOT/tools/mpr-vs-epa/build/epa.txt" \
  >"$REPO_ROOT/tools/mpr-vs-epa/build/geometry.txt"
epa_line="$(grep -F 'EPA depth=' "$REPO_ROOT/tools/mpr-vs-epa/build/epa.txt" || true)"

mpr_line="$(./tools/mpr-vs-epa/build/mpr_case104 <"$REPO_ROOT/tools/mpr-vs-epa/build/geometry.txt")"

python3 - "$epa_line" "$mpr_line" "$EXPECTED_EPA" "$EXPECTED_MPR" "$REL_TOL" <<'PY'
import re
import sys

epa_line, mpr_line, expected_epa, expected_mpr, rel_tol = sys.argv[1:6]
rel_tol = float(rel_tol)

def one(pattern, line, what):
    m = re.search(pattern, line)
    if not m:
        sys.exit(f"FAIL could not read {what} from: {line!r}")
    return float(m.group(1))

actual = {
    "EPA": (one(r"EPA depth=(-?[0-9.eE+-]+)", epa_line, "EPA depth"), float(expected_epa)),
    "MPR": (one(r"mpr_depth=(-?[0-9.eE+-]+)", mpr_line, "MPR depth"), float(expected_mpr)),
}

failed = False
for what, (got, want) in actual.items():
    drift = abs(got - want) / abs(want)
    if drift > rel_tol:
        print(f"FAIL {what} depth moved: {got!r} vs the cited {want!r} "
              f"(relative {drift:.3e} > {rel_tol:.0e})", file=sys.stderr)
        failed = True
    else:
        print(f"OK   {what} depth {got!r} matches the cited value (relative {drift:.3e})")

if failed:
    print("FAIL crates/moveit-collision/src/parry.rs's deviation 6(b) cites these numbers; "
          "update the doc and this gate together, or find what changed.", file=sys.stderr)
    sys.exit(1)
PY

echo "OK deviation 6(b)'s MPR-vs-EPA gap re-derived from libccd $tag"
