#!/bin/bash
# Runs every tools/ci/verify-*, by glob.
#
# The glob is on the PREFIX, not the extension, for the reason ci.yml's own
# now is: `ci.yml` globbed `check-*.sh` and so never once ran
# `check-citation-drift.py`, a gate that needs nothing but python3 and the
# tracked files. Every `verify-*` happens to be a `.sh` today, so widening
# this one changes nothing that runs -- it removes the way the next Python
# gate would leave the set without anyone noticing. A match without the
# executable bit is a hard failure here too.
#
# The `verify-*` scripts are deliberately not in `ci.yml`'s `check-*`
# glob -- each one's own header says why (docker, the gitignored
# `third_party/` tree, an upstream checkout a bare runner never has), and
# `verify-fixture-provenance.sh` states the principle: a script that always
# skips in CI reads as coverage while providing none. So they are run by
# hand, per merge round.
#
# "By hand" was the defect: the list of which ones to run was hand-typed
# prose in a round brief, and it drifted -- one such brief named three of
# the then-six scripts. A hand-maintained enumeration of a set the
# filesystem already holds is the same shape as the hand-typed `TESTS`
# array `verify-vendored-fixture-tests.sh` just removed, and it fails the
# same way: silently, in the direction of less coverage. This script is the
# enumeration, so there is nothing left to keep in sync.
#
# Run it the way the docker-dependent members need:
#
#   sg docker -c ./tools/ci/verify-all.sh
#
# Every script runs even after one fails; the failures are reported
# together at the end. A gate that could not run its measurement (docker
# absent, the oracle image unstamped, an opt-in env var left off, ...) exits
# `$NOT_MEASURED` (see `gate-lib.sh`'s `skip_not_measured`), not 0 -- so it is
# counted separately below, not folded into "passed". It used to exit 0 like
# a real pass, which is exactly what let `OK all $ran verify script(s)
# passed` be true while one of them had, by its own printed SKIP lines,
# measured nothing.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT" || exit 1  # no `set -e` here: a failed cd would
                           # otherwise run this gate in the caller's tree

self="$(basename "${BASH_SOURCE[0]}")"

shopt -s nullglob
scripts=(tools/ci/verify-*)
shopt -u nullglob

if [[ ${#scripts[@]} -eq 0 ]]; then
  echo "FAIL no tools/ci/verify-* found -- did the layout change?" >&2
  exit 1
fi

for script in "${scripts[@]}"; do
  if [[ ! -x "$script" ]]; then
    echo "FAIL $script matches the verify-* glob but is not executable." >&2
    exit 1
  fi
done

failed=()
not_measured=()
passed=0
ran=0
for script in "${scripts[@]}"; do
  [[ "$(basename "$script")" == "$self" ]] && continue
  ran=$((ran + 1))
  echo "=== $script"
  "$script"
  rc=$?
  if [[ "$rc" -eq 0 ]]; then
    passed=$((passed + 1))
  elif [[ "$rc" -eq "$NOT_MEASURED" ]]; then
    not_measured+=("$script")
  else
    failed+=("$script")
  fi
done

if [[ $ran -eq 0 ]]; then
  echo "FAIL the glob matched only this script -- nothing was verified." >&2
  exit 1
fi

echo
if [[ ${#not_measured[@]} -gt 0 ]]; then
  echo "${#not_measured[@]} of $ran verify script(s) did not measure anything this run" \
    "(see their own SKIP lines above) -- not counted as passed:"
  printf '  %s\n' "${not_measured[@]}"
fi
if [[ ${#failed[@]} -gt 0 ]]; then
  echo "FAIL ${#failed[@]} of $ran verify script(s) failed:" >&2
  printf '  %s\n' "${failed[@]}" >&2
  exit 1
fi
echo "OK $passed of $ran verify script(s) passed, ${#not_measured[@]} not measured"
