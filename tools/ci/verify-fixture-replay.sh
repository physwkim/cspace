#!/bin/bash
# Every committed `*_request.json`/`*_response.json` oracle-fixture pair must
# still reproduce its committed response when replayed against the live
# oracle -- twice: once in a process of its own, and once sharing a process
# with every other fixture captured against the same robot model. The second
# pass is documented where it runs, at the bottom of this file; it exists
# because a per-file replay cannot see one op leaving state behind for the
# next.
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

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
RUN_ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

REPLAY_ONE="$(mktemp)"
trap 'rm -f "$REPLAY_ONE"' EXIT
cat >"$REPLAY_ONE" <<'PYEOF'
# Replays one fixture pair against the live oracle.
#
# Usage: _replay_one.py <run_oracle.sh> <urdf> <srdf> <request.json> <response.json> <ignore_json> [timeout_s]
# <ignore_json> is the fixture's `ignore_result_fields_by_id` manifest entry
# (a JSON object mapping string request id -> list of top-level `result`
# field names to exclude), or "{}" if the fixture has none.
# Prints "identical"/"DRIFTED <n diffs>"/"ORACLE-FAIL <reason>" to stdout,
# a unified diff to stderr on drift, and exits 0 only on an exact match.
#
# The request file need not be a committed fixture: the combined pass below
# feeds it a concatenation of every fixture sharing one robot model, which is
# why the timeout is a parameter rather than the fixed 120s a single fixture
# needs.
import json
import subprocess
import sys

run_oracle, urdf, srdf, request_path, response_path, ignore_json = sys.argv[1:7]
timeout_s = int(sys.argv[7]) if len(sys.argv) > 7 else 120
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
        timeout=timeout_s,
    )
except subprocess.TimeoutExpired:
    print(f"ORACLE-FAIL replay timed out after {timeout_s}s")
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

# The loop below is driven by the manifest, but the invariant at the top of
# this file is over *committed pairs*: every `*_request.json`/`*_response.json`
# pair must still reproduce. Those two sets are not the same, and where they
# disagree the manifest wins silently -- a pair nobody listed is simply never
# replayed, and the run still ends with a row of `identical` lines and exit 0.
# That is indistinguishable from coverage. It happened:
# `metrics/panda_arm_5dof_kinematics_metrics` was committed in one
# round and went unreplayed until a later audit counted the two sets against
# each other (it replayed clean once registered -- the cost was the blind
# spot, not a drift).
#
# So the domain of the check is derived from the pairs, and an unregistered
# pair is a failure rather than a skip. Registering it is one manifest entry;
# the alternative is trusting every future fixture author to remember.
unregistered="$(python3 - "$REPO_ROOT" <<'PYEOF'
import glob, json, os, sys

repo_root = sys.argv[1]
for fixtures_dir in sorted(glob.glob(os.path.join(repo_root, "crates/*/tests/fixtures"))):
    crate = fixtures_dir.split(os.sep)[-3]
    manifest = os.path.join(fixtures_dir, "oracle-models.json")
    entries = set(json.load(open(manifest))) if os.path.exists(manifest) else set()
    for request in sorted(glob.glob(os.path.join(fixtures_dir, "*_request.json"))):
        stem = os.path.basename(request)[: -len("_request.json")]
        if not os.path.exists(os.path.join(fixtures_dir, f"{stem}_response.json")):
            continue
        if stem not in entries:
            print(f"{crate}/{stem}")
PYEOF
)"
if [[ -n "$unregistered" ]]; then
  while IFS= read -r pair; do
    crate="${pair%%/*}"
    echo "UNREGISTERED $pair -- committed fixture pair absent from" >&2
    echo "             crates/$crate/tests/fixtures/oracle-models.json, so it is" >&2
    echo "             never replayed against the live oracle" >&2
  done <<<"$unregistered"
  status=1
fi

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
      printf '%s\n' "$crate/$stem: ${result//$'\n'/$'\n'$crate/$stem: }" >&2
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

# Second pass: every fixture that shares a robot model, through ONE oracle
# process.
#
# The loop above starts a fresh container per fixture, so a request only ever
# sees the ops from its own file. But the oracle is a long-lived object holding
# mutable state -- `state_`, the RobotModel, the GroupStateRepresentation map
# `initialize()` pregenerates -- and `main()` reads requests in a loop from one
# stdin, so an op that leaves state behind changes the answer of whatever op
# runs after it. Within a fixture file that is exercised already. Across files
# it is structurally invisible: no run ever puts two files in one process
# (PORTING-PLAN.md §143.1). `oracle.cpp` is shared by every panel, which is
# where such a leak would come from in the first place.
#
# Grouping is by the *content* of the urdf/srdf pair, not the path: the same
# panda is copied under seven crates' fixture directories, and a leak crossing
# crates is the case worth covering. It also costs less than the pass above
# rather than more -- 43 fixtures are 6 models, so 6 container starts replace
# 43. A group of one adds nothing the per-file pass did not already do and is
# skipped rather than paid for.
#
# The comparison is not reimplemented here: request ids are renumbered to
# `fixture_index * 1000 + original_id` (ids are integers, max 12 across the
# corpus, so 1000 cannot collide), the `ignore_result_fields_by_id` maps are
# merged under the same renumbering, and _replay_one.py is handed the
# concatenation as if it were a single fixture. A drift is therefore compared,
# reported and diffed by exactly the code the per-file pass uses. The renumbered
# id ranges are printed on failure so a diff line points back at a fixture.
python3 - "$REPO_ROOT" "$RUN_ORACLE" "$REPLAY_ONE" <<'PYEOF' || status=1
import collections
import glob
import hashlib
import json
import os
import subprocess
import sys
import tempfile

repo_root, run_oracle, replay_one = sys.argv[1:4]

def digest(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()

def as_list(parsed):
    return parsed if isinstance(parsed, list) else [parsed]

fixtures = []
for manifest_path in sorted(
    glob.glob(os.path.join(repo_root, "crates/*/tests/fixtures/oracle-models.json"))
):
    fixtures_dir = os.path.dirname(manifest_path)
    crate = os.path.basename(os.path.dirname(os.path.dirname(fixtures_dir)))
    manifest = json.load(open(manifest_path))
    for stem in sorted(manifest):
        entry = manifest[stem]
        fixture = {
            "label": f"{crate}/{stem}",
            "crate": crate,
            "urdf": os.path.join(fixtures_dir, entry["urdf"]),
            "srdf": os.path.join(fixtures_dir, entry["srdf"]),
            "request": os.path.join(fixtures_dir, f"{stem}_request.json"),
            "response": os.path.join(fixtures_dir, f"{stem}_response.json"),
            "ignore": entry.get("ignore_result_fields_by_id", {}),
        }
        # A missing file is already reported as MISSING by the per-file pass,
        # which owns that failure; grouping it here would only turn the same
        # fact into a traceback.
        if all(os.path.isfile(fixture[k]) for k in ("urdf", "srdf", "request", "response")):
            fixtures.append(fixture)

groups = collections.OrderedDict()
for fixture in fixtures:
    groups.setdefault((digest(fixture["urdf"]), digest(fixture["srdf"])), []).append(fixture)

status = 0
for members in groups.values():
    model = os.path.basename(members[0]["urdf"])
    if len(members) < 2:
        print(f"single      {members[0]['label']} -- only fixture on {model}, per-file pass covers it")
        continue

    requests, expected, ignore, ranges = [], [], {}, []
    for index, member in enumerate(members):
        base = index * 1000
        member_requests = as_list(json.load(open(member["request"])))
        for request in member_requests:
            requests.append({**request, "id": base + request["id"]})
        for response in as_list(json.load(open(member["response"]))):
            expected.append({**response, "id": base + response["id"]})
        for request_id, fields in member["ignore"].items():
            ignore[str(base + int(request_id))] = fields
        ids = [base + r["id"] for r in member_requests]
        ranges.append(f"ids {min(ids)}-{max(ids)}  {member['label']}")

    crates = len({member["crate"] for member in members})
    label = f"{model}: {len(members)} fixtures, {len(requests)} requests, {crates} crate(s)"

    def replay(ordered):
        with tempfile.TemporaryDirectory() as scratch:
            request_path = os.path.join(scratch, "combined_request.json")
            response_path = os.path.join(scratch, "combined_response.json")
            with open(request_path, "w") as handle:
                json.dump(ordered, handle)
            with open(response_path, "w") as handle:
                json.dump(expected, handle)
            return subprocess.run(
                [
                    sys.executable, replay_one, run_oracle,
                    members[0]["urdf"], members[0]["srdf"],
                    request_path, response_path, json.dumps(ignore),
                    str(120 * len(members)),
                ],
                capture_output=True,
                text=True,
            )

    # Two orders, not one. A leak from request i into request j only shows up
    # when i precedes j, so a single walk exonerates exactly half the ordered
    # pairs and reports the other half as passing. Reversal is the one
    # permutation that inverts every pair at once, so the two runs together
    # put every ordered pair under test. `replay_one` matches by id rather
    # than position (see its own comment), so `expected` is order-free and
    # the same file serves both directions. This is not full coverage of
    # n! orders: it catches leaks between any two requests, not one that
    # needs a specific third request sitting between them.
    for order, ordered in (("combined", requests), ("reversed", requests[::-1])):
        proc = replay(ordered)
        if proc.returncode == 0:
            print(f"{order:<12}{label}")
            continue

        status = 1
        sys.stderr.write(f"{order.upper():<12}{label} -- {proc.stdout.strip()}\n")
        for line in ranges:
            sys.stderr.write(f"            {line}\n")
        sys.stderr.write(proc.stderr)

sys.exit(status)
PYEOF

if [[ $status -ne 0 ]]; then
  echo "fixture replay check failed" >&2
fi
exit "$status"
