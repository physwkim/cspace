#!/usr/bin/env bash
# Fails if `oracle_stamp`/`oracle_file_digest` (tools/moveit-oracle/src-
# digest.sh) can turn a real failure -- a directory that does not exist, a
# file that goes unreadable mid-`find` -- into a well-formed digest instead
# of a nonzero exit and no output.
#
# That used to be possible under a caller with `set -uo pipefail` and no
# `-e`, which is not a hypothetical posture: it is the exact one
# `verify-phase3-tangency-subset.sh` and `verify-phase3-penetration-
# subset.sh` deliberately choose, because their per-robot loop needs a
# comparison binary's nonzero exit to reach `run_verdict` rather than abort
# the script. `oracle_stamp` used to be `{ oracle_file_digest "$1";
# oracle_build_inputs; } | sha256sum | cut ...`: a brace group's exit status
# is its LAST command's, so a failing `oracle_file_digest` was discarded by
# the always-succeeding `oracle_build_inputs` printf that ran right after
# it, and `sha256sum` still hashed whatever partial bytes had been emitted
# (nothing, for a `cd` failure) -- producing a plausible 64-hex-char stamp
# for a directory that was never actually read. Under `set -uo pipefail`
# that reached `oracle_stamp_verdict` as a real, well-formed `want`, which
# is precisely the "well-formed stamp that was never built" failure this
# file's own header (:6-9, :44-49) says the stamp exists to prevent.
#
# Every case below is run three ways -- `set -uo pipefail` (the dangerous
# posture, must fail loud), `set -euo pipefail` (already safe before this
# fix, must stay safe), and no `set` at all (`tools/moveit-oracle/
# Dockerfile`'s `RUN ... && source ... && oracle_stamp ...` shape, which
# declares no posture of its own) -- because the fix is "correct regardless
# of caller posture", and a test run under only one posture cannot tell that
# apart from "correct under the posture I happened to test".
#
# Needs no docker: this only exercises the shell functions, not the oracle
# image.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$repo_root/tools/ci/gate-lib.sh"
require_caller_tree "$repo_root"
cd "$repo_root"

DIGEST_SH="$repo_root/tools/moveit-oracle/src-digest.sh"
HEX64='^[0-9a-f]{64}$'

failures=0
cases=0

# <label> <posture-set-command (may be empty)> <dir-under-test> <want: fail|ok>
run_case() {
  local label="$1" posture="$2" dir="$3" want="$4" out rc
  cases=$((cases + 1))
  # The command substitution is the condition of this `if`, not a plain
  # assignment -- under this script's own `set -e`, `out=$(failing_cmd)` as a
  # bare statement aborts the whole script right here before `rc=$?` is ever
  # reached, which is the exact defect this file exists to catch happening
  # to a DIFFERENT caller. `if out=$(...); then rc=0; else rc=$?; fi` is
  # exempt from `-e` by the same rule that exempts `&&`/`||`/`!` conditions.
  # shellcheck disable=SC2016
  if out="$(bash -c "$posture; source \"\$1\"; oracle_stamp \"\$2\"" _ "$DIGEST_SH" "$dir" 2>/dev/null)"; then
    rc=0
  else
    rc=$?
  fi
  if [ "$want" = fail ]; then
    if [ "$rc" -eq 0 ] || [ -n "$out" ]; then
      echo "FAIL $label: expected a nonzero exit and no output, got rc=$rc out=[$out]" >&2
      failures=$((failures + 1))
    fi
  else
    if [ "$rc" -ne 0 ] || ! [[ "$out" =~ $HEX64 ]]; then
      echo "FAIL $label: expected rc=0 and a 64-hex-char digest, got rc=$rc out=[$out]" >&2
      failures=$((failures + 1))
    fi
  fi
}

# --- boundary 1: the directory does not exist (cd fails) ---
for p in "set -uo pipefail" "set -euo pipefail" ""; do
  run_case "nonexistent dir, posture [$p]" "$p" "$repo_root/tools/moveit-oracle/no-such-dir-$$" fail
done

# --- boundary 2: the directory exists but a file inside is unreadable
# (cd succeeds, a later pipeline stage fails) ---
if [ "$(id -u)" -eq 0 ]; then
  echo "SKIP boundary 2 (unreadable file): running as root, chmod 000 does not restrict root -- 3 of ${cases} + 3 planned cases not run." >&2
else
  unreadable_dir="$(mktemp -d)"
  trap 'rm -rf "$unreadable_dir"' EXIT
  echo ok >"$unreadable_dir/readable.txt"
  echo secret >"$unreadable_dir/unreadable.txt"
  chmod 000 "$unreadable_dir/unreadable.txt"
  for p in "set -uo pipefail" "set -euo pipefail" ""; do
    run_case "unreadable file, posture [$p]" "$p" "$unreadable_dir" fail
  done
fi

# --- boundary 3: happy path -- a real, fully readable directory still
# produces a valid digest under the dangerous posture. Without this, the
# fix above could be trivially over-corrected into "oracle_stamp always
# fails" and every boundary-1/2 case would still pass. ---
run_case "real tools/moveit-oracle dir, posture [set -uo pipefail]" \
  "set -uo pipefail" "$repo_root/tools/moveit-oracle" ok

require_nonempty "$cases" "oracle_stamp boundary cases"

if [ "$failures" -ne 0 ]; then
  echo "FAIL $failures of $cases oracle_stamp boundary case(s) did not fail loud (or did not succeed) as expected." >&2
  exit 1
fi

echo "OK $cases oracle_stamp boundary case(s): every caller posture fails loud on a real failure, and the happy path still works."
