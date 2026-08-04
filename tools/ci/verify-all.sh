#!/bin/bash
# Runs every tools/ci/verify-*.sh, by glob.
#
# The `verify-*.sh` scripts are deliberately not in `ci.yml`'s `check-*.sh`
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
# together at the end. A `SKIP` line inside a script is that script's own
# business -- it is not a failure here, and each such script prints its own
# loud "this is not a pass" note.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

self="$(basename "${BASH_SOURCE[0]}")"

shopt -s nullglob
scripts=(tools/ci/verify-*.sh)
shopt -u nullglob

if [[ ${#scripts[@]} -eq 0 ]]; then
  echo "FAIL no tools/ci/verify-*.sh found -- did the layout change?" >&2
  exit 1
fi

failed=()
ran=0
for script in "${scripts[@]}"; do
  [[ "$(basename "$script")" == "$self" ]] && continue
  ran=$((ran + 1))
  echo "=== $script"
  if ! "$script"; then
    failed+=("$script")
  fi
done

if [[ $ran -eq 0 ]]; then
  echo "FAIL the glob matched only this script -- nothing was verified." >&2
  exit 1
fi

echo
if [[ ${#failed[@]} -gt 0 ]]; then
  echo "FAIL ${#failed[@]} of $ran verify script(s) failed:" >&2
  printf '  %s\n' "${failed[@]}" >&2
  exit 1
fi
echo "OK all $ran verify script(s) passed"
