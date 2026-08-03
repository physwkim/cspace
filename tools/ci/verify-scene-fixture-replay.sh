#!/bin/bash
# Replays crates/moveit-scene's committed oracle request fixtures against a
# live oracle and diffs the response against the committed one.
#
# `crates/moveit-scene/tests/fixtures/*_request.json` are the literal wire
# JSON sent to the oracle when a fixture was captured (`"op"`/`"id"` are
# already in the file), but until now the `--urdf`/`--srdf` needed to replay
# one lived only in the consuming test's `build_model()`/`srdf()` -- the
# fixture alone was not enough to re-run it. 931 parity tests compare Rust
# against the *committed response*; nothing compared the *current* oracle
# against that response, so a merge that silently changed an op's answer
# (oracle.cpp is seven panels' edits deep) had no check that could catch it.
# §40.
#
# The fix is a `"model"` field recorded in the request fixture itself (the
# short robot name -- `fixtures/<model>.urdf`/`.srdf`, the same convention
# run-oracle-sweep.sh's `CASES_TO_RUN` already uses), so this script needs
# nothing beyond the fixture pair to replay it. `panda_is_state_valid.json`
# and `pr2_attached_collision.json` (this crate's other two oracle fixtures)
# are a hand-built "cases" shape, not a literal captured request/response
# pair -- replaying them means reconstructing the wire request from summary
# fields rather than diffing one already on disk, a different job from the
# one this script does. Out of scope here.
#
# Needs docker and the moveit-rs/oracle image -- like run-oracle-sweep.sh,
# verify-fixture-provenance.sh and verify-continuous-reseed-wrap.sh, this is
# deliberately not one of the `check-*.sh` scripts `.github/workflows/ci.yml`
# and the local gate loop run, and is not a `cargo test`. The name is the
# whole mechanism: ci.yml globs `tools/ci/check-*.sh` rather than
# enumerating, precisely so a new check cannot be forgotten -- which means a
# docker-requiring script named `check-*` is picked up by a runner that has
# no docker and no oracle image, and fails there for a reason unrelated to
# what it tests.
#
#   sg docker -c 'tools/ci/verify-scene-fixture-replay.sh'
#
# Exits non-zero if any fixture's live response disagrees with the committed
# one, or if a request fixture has no "model" field to replay it with.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

FIXTURES="crates/moveit-scene/tests/fixtures"

# request file -> response file, both under $FIXTURES. Add a pair here when
# a new moveit-scene request/response fixture is committed.
declare -A CASES=(
  ["panda_frame_transform_request.json"]="panda_frame_transform_response.json"
)

LIVE_FILE="$(mktemp)"
trap 'rm -f "$LIVE_FILE"' EXIT

status=0
for request_file in "${!CASES[@]}"; do
  response_file="${CASES[$request_file]}"
  request_path="$FIXTURES/$request_file"
  response_path="$FIXTURES/$response_file"

  model="$(python3 -c "
import json, sys
print(json.load(open(sys.argv[1])).get('model', ''))
" "$request_path")"
  if [[ -z "$model" ]]; then
    echo "FAIL $request_file: no \"model\" field -- cannot tell which fixtures/<robot>.urdf/.srdf to replay against" >&2
    status=1
    continue
  fi

  urdf="$REPO_ROOT/fixtures/$model.urdf"
  srdf="$REPO_ROOT/fixtures/$model.srdf"

  # The oracle protocol is one JSON object per line (std::getline on stdin);
  # the committed fixture is pretty-printed across many lines for review, so
  # it has to be compacted to one line before it is a valid request.
  python3 -c "
import json, sys
print(json.dumps(json.load(open(sys.argv[1]))))
" "$request_path" | \
    "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" --urdf "$urdf" --srdf "$srdf" \
    2>/dev/null | tail -1 > "$LIVE_FILE"

  if python3 - "$LIVE_FILE" "$response_path" <<'PYEOF'
import json, sys

live_path, committed_path = sys.argv[1], sys.argv[2]
with open(live_path) as f:
    live = json.load(f)
with open(committed_path) as f:
    committed = json.load(f)

if live != committed:
    print("live:     " + json.dumps(live, sort_keys=True), file=sys.stderr)
    print("committed:" + json.dumps(committed, sort_keys=True), file=sys.stderr)
    sys.exit(1)
PYEOF
  then
    echo "PASS $request_file/$response_file: live oracle response matches the committed fixture field-for-field"
  else
    echo "FAIL $request_file/$response_file: live oracle response differs from the committed fixture" >&2
    status=1
  fi
done

exit "$status"
