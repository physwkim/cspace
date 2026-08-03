#!/bin/bash
# Replays moveit-trajectory's 4 committed oracle request fixtures against
# the current oracle image and diffs the result against the committed
# response, byte-for-byte.
#
# PORTING-PLAN.md §40: the 21 `*_request.json` fixtures across this
# workspace cannot be mechanically replayed, because a fixture does not
# record which model it was captured against -- the `--urdf`/`--srdf` used
# to be knowledge that lived only in the consuming test's Rust source, and
# filename similarity is not provenance (`totg_request.json` looks like it
# belongs to `totg_synthetic_parity.rs`; it does not). Without that
# information, nothing in this repo compares *the merged oracle* against
# the committed responses -- the 94 parity tests in this crate only compare
# *Rust* against them, which cannot see an oracle-side regression a merge
# introduced.
#
# `oracle-fixtures-manifest.json` (beside the fixtures this script reads)
# closes that: it is the one place that records, per fixture stem, which
# `--urdf`/`--srdf` pair to replay against. This script is the mechanical
# consumer of that manifest, so the manifest cannot drift from what
# actually gets replayed.
#
# `totg_request.json`'s model entry (panda, matching its siblings) is
# arbitrary rather than load-bearing: `oracle.cpp`'s `totgCase` (the
# core-only branch, no top-level "group" key) never reads `model_` at all
# -- confirmed by grepping the function body -- so any loadable URDF/SRDF
# pair reproduces the identical result. It is still pinned to a concrete
# pair here rather than left unspecified, because "replayable" means one
# command reproduces the fixture, not "replayable modulo picking a model
# yourself."
#
# Needs docker and the moveit-rs/oracle image -- like
# verify-continuous-reseed-wrap.sh and verify-fixture-provenance.sh, this is
# deliberately not one of the `check-*.sh` scripts `.github/workflows/ci.yml`
# and the local gate loop glob and run on a runner with no docker and no
# oracle image.
#
#   tools/ci/verify-trajectory-oracle-replay.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FIXTURES="$REPO_ROOT/crates/moveit-trajectory/tests/fixtures"
MANIFEST="$FIXTURES/oracle-fixtures-manifest.json"
RUN_ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

# Canonical form matches check-fixture-format.sh's: 2-space indent, sorted
# keys. Applied to both sides of every diff below, so this only asserts
# semantic equality, never a formatting difference.
canon() {  # <file>
  python3 -c '
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
print(json.dumps(json.loads(p.read_text()), indent=2, sort_keys=True))
' "$1"
}

manifest_field() {  # <stem> <urdf|srdf>
  python3 -c '
import json, sys
manifest, stem, field = sys.argv[1:4]
print(json.load(open(manifest))[stem][field])
' "$MANIFEST" "$1" "$2"
}

overall=0
for stem in ruckig totg totg_robot_trajectory totg_synthetic; do
  request="$FIXTURES/${stem}_request.json"
  committed="$FIXTURES/${stem}_response.json"
  urdf_rel="$(manifest_field "$stem" urdf)"
  srdf_rel="$(manifest_field "$stem" srdf)"

  replayed="$(mktemp)"
  stderr_log="$(mktemp)"

  if jq -c '.[]' "$request" |
    "$RUN_ORACLE" --urdf "$REPO_ROOT/$urdf_rel" --srdf "$REPO_ROOT/$srdf_rel" 2>"$stderr_log" |
    jq -s '.' >"$replayed"; then
    if diff -q <(canon "$committed") <(canon "$replayed") >/dev/null; then
      echo "PASS $stem: replay matches committed response byte-for-byte"
    else
      echo "FAIL $stem: replay differs from committed response" >&2
      diff -u <(canon "$committed") <(canon "$replayed") >&2 || true
      overall=1
    fi
  else
    echo "FAIL $stem: oracle replay errored -- $(cat "$stderr_log")" >&2
    overall=1
  fi

  rm -f "$replayed" "$stderr_log"
done

exit "$overall"
