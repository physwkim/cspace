#!/usr/bin/env bash
# Fails if a workspace audit command has been copied back into a crate.
#
# `count-relative-eq.pl` was copied from `crates/cspace-core/audit/` into
# four more places, and the `tools/moveit-diff/` copy then missed the
# block-comment and string-literal fixes -- so the same audit command returned
# two different classifications depending on which crate a panel ran it from.
# The scripts now live once, in `tools/ci/`. This gate keeps them there:
# convention alone is what let the divergence happen.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$repo_root/tools/ci/gate-lib.sh"

require_caller_tree "$repo_root"
cd "$repo_root"

# Anything under a crate's or tool's `audit/` directory whose name starts with
# `count`. The canonical copies are `tools/ci/count-*`, which this does not
# match -- `tools/ci` has no `audit/` subdirectory.
#
# Checked command substitution, not `mapfile -t copies < <(git ls-files ...)`:
# process substitution discards the producer's exit status and `set -e` never
# sees it. The sibling gates survive that because they run `require_nonempty`
# and an empty list is their *failure* condition; here an empty list is the
# pass, so a broken `git ls-files` and a clean tree are the same state and
# nothing downstream can tell them apart. Reproduced by p1-fixtures against
# this script unmodified: `git archive HEAD` into a `.git`-less export prints
# `fatal: not a git repository`, then `OK: no audit command copied`, exit 0 --
# which is exactly how the oracle image builds its context (see
# `check-fixture-format.sh`'s header). Same mechanism as the
# `verify-clean-checkout.sh` bug and fixed the same way.
if ! copies_raw="$(git ls-files --deduplicate -- '*/audit/count*' | sort)"; then
  echo "FAIL git ls-files failed -- nothing was checked." >&2
  exit 1
fi
copies=()
[ -n "$copies_raw" ] && mapfile -t copies <<<"$copies_raw"

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
