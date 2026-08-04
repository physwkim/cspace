#!/usr/bin/env bash
# Fails if a workspace audit command has been copied back into a crate.
#
# `count-relative-eq.pl` was copied from `crates/moveit-geometry/audit/` into
# four more places, and the `tools/moveit-diff/` copy then missed the
# block-comment and string-literal fixes -- so the same audit command returned
# two different classifications depending on which crate a panel ran it from.
# The scripts now live once, in `tools/ci/`. This gate keeps them there:
# convention alone is what let the divergence happen.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

# Anything under a crate's or tool's `audit/` directory whose name starts with
# `count`. The canonical copies are `tools/ci/count-*`, which this does not
# match -- `tools/ci` has no `audit/` subdirectory.
mapfile -t copies < <(
  git ls-files -- '*/audit/count*' | sort
)

if [ "${#copies[@]}" -ne 0 ]; then
  echo "audit command copied back into a crate:" >&2
  printf '  %s\n' "${copies[@]}" >&2
  echo >&2
  echo "The workspace copies live in tools/ci/ and are run with an explicit" >&2
  echo "file list, e.g." >&2
  echo "  perl tools/ci/count-relative-eq.pl \$(find crates/<crate> -name '*.rs' | sort)" >&2
  exit 1
fi

echo "OK: no audit command copied into a crate (canonical copies in tools/ci/)"
