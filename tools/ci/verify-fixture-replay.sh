#!/bin/bash
# Every committed `*_request.json`/`*_response.json` oracle-fixture pair must
# still reproduce its committed response when replayed against the live
# oracle.
#
# The 925 parity tests in `cargo nextest run` compare *Rust against a
# committed response*; none of them compare *the current oracle against that
# same committed response*. `oracle.cpp` is shared across every crate's
# panel, so a merge that changes one op's answer leaves every *other* crate's
# committed fixture silently describing an oracle that no longer exists --
# and nothing in the normal test run can see that, because the normal test
# run never talks to the oracle at all.
#
# Replaying needs the exact `--urdf`/`--srdf` pair each fixture was captured
# against, which the request/response JSON itself does not record -- only
# the Rust test that consumes it knows, and only for fixtures a human
# happened to look at. `tests/fixtures/oracle-models.json` (one per crate
# that has fixtures) fixes that: a small manifest, keyed by fixture stem,
# naming the urdf/srdf pair relative to that crate's own `tests/fixtures/`.
# The information needed to replay lives with the fixture from now on, not
# buried in test source.
#
# The oracle's own protocol (see `main()` in `tools/moveit-oracle/src/
# oracle.cpp` and `capture-collision-fixtures.py`'s `Oracle` class) is
# newline-delimited JSON: one compact request object per line in, one
# compact response object per line out, in order. Committed fixtures are
# pretty-printed JSON *arrays* for reviewability, so replaying means
# flattening to NDJSON on the way in and re-assembling on the way out --
# `_replay_one.py` below does that.
#
# A manifest entry may also carry `ignore_result_fields_by_id`: some oracle
# ops read a field the C++ side never initializes on every code path (e.g.
# `collision_distance_field_types`'s `relative_cylinder_pose` for a
# Sphere-only body -- see `BodyDecomposition`'s doc comment in
# `collision_distance_field_types.rs` for the root cause, and
# `collision_distance_field_types_parity.rs`'s module doc for the same skip
# on the Rust-vs-oracle side). Replaying such an id twice in a row returns
# two different values, neither of them wrong -- there is no fixed value to
# match, so the field is excluded from the comparison rather than compared
# against a committed snapshot that just happened to be one random instance
# of the same garbage.
#
# Deliberately NOT named `check-*.sh`: replaying means running the oracle
# container, which needs docker and (like `verify-fixture-provenance.sh`)
# is unavailable to the CI runners that glob for `check-*.sh`. A script that
# always failed there would read as coverage while providing none.
#
#   tools/ci/verify-fixture-replay.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

REPLAY_ONE="$(mktemp)"
trap 'rm -f "$REPLAY_ONE"' EXIT
cat >"$REPLAY_ONE" <<'PYEOF'
# Replays one fixture pair against the live oracle.
#
# Usage: _replay_one.py <run_oracle.sh> <urdf> <srdf> <request.json> <response.json> <ignore_json>
# <ignore_json> is the fixture's `ignore_result_fields_by_id` manifest entry
# (a JSON object mapping string request id -> list of top-level `result`
# field names to exclude), or "{}" if the fixture has none.
# Prints "identical"/"DRIFTED <n diffs>"/"ORACLE-FAIL <reason>" to stdout,
# a unified diff to stderr on drift, and exits 0 only on an exact match.
import json
import subprocess
import sys

run_oracle, urdf, srdf, request_path, response_path, ignore_json = sys.argv[1:7]
ignore_result_fields_by_id = json.loads(ignore_json)

def as_list(parsed):
    # Most fixtures are a JSON array of request/response objects; a fixture
    # with exactly one case (e.g. distance_field_request.json) is committed
    # as a bare object instead. Normalizing here keeps the NDJSON round trip
    # below uniform rather than branching the whole script on fixture shape.
    return parsed if isinstance(parsed, list) else [parsed]

def strip_ignored(response_obj):
    fields = ignore_result_fields_by_id.get(str(response_obj.get("id")))
    if fields and isinstance(response_obj.get("result"), dict):
        for field in fields:
            response_obj["result"].pop(field, None)
    return response_obj

requests = as_list(json.loads(open(request_path).read()))
expected = [strip_ignored(r) for r in as_list(json.loads(open(response_path).read()))]

ndjson_in = "\n".join(json.dumps(r, sort_keys=True) for r in requests) + "\n"

try:
    proc = subprocess.run(
        [run_oracle, "--urdf", urdf, "--srdf", srdf],
        input=ndjson_in,
        capture_output=True,
        text=True,
        timeout=120,
    )
except subprocess.TimeoutExpired:
    print("ORACLE-FAIL replay timed out after 120s")
    sys.exit(1)

if proc.returncode != 0:
    reason = proc.stderr.strip().splitlines()[-1] if proc.stderr.strip() else f"exit {proc.returncode}"
    print(f"ORACLE-FAIL {reason}")
    sys.exit(1)

lines = [line for line in proc.stdout.splitlines() if line.strip()]
if len(lines) != len(requests):
    print(f"ORACLE-FAIL sent {len(requests)} requests, got {len(lines)} responses back")
    sys.exit(1)

try:
    actual = [strip_ignored(json.loads(line)) for line in lines]
except json.JSONDecodeError as e:
    print(f"ORACLE-FAIL response line is not valid JSON: {e}")
    sys.exit(1)

# Responses are matched by "id", not by position: the oracle answers each
# request it read in order, but this keeps the check honest about what it is
# actually asserting rather than trusting order to carry the correspondence.
actual_by_id = {r["id"]: r for r in actual}
expected_by_id = {r["id"]: r for r in expected}
if actual_by_id.keys() != expected_by_id.keys():
    print(f"ORACLE-FAIL response ids {sorted(actual_by_id)} != expected {sorted(expected_by_id)}")
    sys.exit(1)

actual_sorted = [actual_by_id[i] for i in sorted(actual_by_id)]
expected_sorted = [expected_by_id[i] for i in sorted(expected_by_id)]

if actual_sorted == expected_sorted:
    print("identical")
    sys.exit(0)

import difflib

a = json.dumps(expected_sorted, indent=2, sort_keys=True).splitlines()
b = json.dumps(actual_sorted, indent=2, sort_keys=True).splitlines()
diff = list(difflib.unified_diff(a, b, "committed", "replayed", lineterm=""))
print(f"DRIFTED {sum(1 for l in diff if l.startswith(('+', '-')) and not l.startswith(('+++', '---')))} line(s) differ")
sys.stderr.write("\n".join(diff[:40]) + "\n")
sys.exit(1)
PYEOF

status=0
found_manifest=0

shopt -s nullglob
# Absolute, not relative: `run-oracle.sh` mounts `$REPO_ROOT` into the
# container at the same absolute path, and the container's working
# directory is not `$REPO_ROOT`, so a relative path resolves to nothing
# inside it.
for manifest in "$REPO_ROOT"/crates/*/tests/fixtures/oracle-models.json; do
  found_manifest=1
  fixtures_dir="$(dirname "$manifest")"
  crate_dir="$(dirname "$(dirname "$fixtures_dir")")"
  crate="$(basename "$crate_dir")"

  stems="$(python3 -c '
import json, sys
for stem in sorted(json.load(open(sys.argv[1]))):
    print(stem)
' "$manifest")"

  while IFS= read -r stem; do
    [[ -z "$stem" ]] && continue
    read -r urdf_name srdf_name < <(python3 -c '
import json, sys
m = json.load(open(sys.argv[1]))[sys.argv[2]]
print(m["urdf"], m["srdf"])
' "$manifest" "$stem")
    ignore_json="$(python3 -c '
import json, sys
m = json.load(open(sys.argv[1]))[sys.argv[2]]
print(json.dumps(m.get("ignore_result_fields_by_id", {})))
' "$manifest" "$stem")"

    request="$fixtures_dir/${stem}_request.json"
    response="$fixtures_dir/${stem}_response.json"
    urdf="$fixtures_dir/$urdf_name"
    srdf="$fixtures_dir/$srdf_name"

    missing=0
    for f in "$request" "$response" "$urdf" "$srdf"; do
      if [[ ! -f "$f" ]]; then
        echo "MISSING    $crate/$stem -- $f does not exist" >&2
        missing=1
      fi
    done
    if [[ "$missing" -eq 1 ]]; then
      status=1
      continue
    fi

    result="$(python3 "$REPLAY_ONE" "$RUN_ORACLE" "$urdf" "$srdf" "$request" "$response" "$ignore_json")" || {
      echo "$result" | sed "s|^|$crate/$stem: |" >&2
      status=1
      continue
    }
    echo "$result    $crate/$stem"
  done <<<"$stems"
done
shopt -u nullglob

if [[ "$found_manifest" -eq 0 ]]; then
  echo "no crates/*/tests/fixtures/oracle-models.json found -- did the layout change?" >&2
  exit 1
fi

if [[ $status -ne 0 ]]; then
  echo "fixture replay check failed" >&2
fi
exit "$status"
