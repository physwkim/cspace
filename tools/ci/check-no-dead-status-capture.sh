#!/usr/bin/env bash
# Fails if a line that closes a `$(...)` command substitution with no
# trailing `||` (or other operator) is immediately followed by a bare
# `var=$?` capture.
#
# Round 18's sweep found this exact shape in `ros/verify-ros-interop.sh`
# (fixed in 48ef7ce): `test_output="$(docker run ... 2>&1)"` followed on the
# next line by `test_status=$?`. Under `set -euo pipefail`, a failing
# command inside the substitution aborts the script *at the assignment* --
# the `$?` line, and every handler downstream of it, never runs. From then
# on `$test_status` can only ever be seen holding 0, which does not mean
# "the command succeeded", it means "the script would already be dead if it
# hadn't".
#
# This is a narrow syntactic pattern match, not a shell parser. It catches
# exactly one shape and is intentionally blind to everything else round
# 18's sweep found by hand, because those need semantic understanding a
# regex does not have:
#
#   - the `pipefail`-abort family (`x="$(cmd1 | grep pattern)"` aborting
#     before a downstream `-z` check runs when `grep` legitimately finds no
#     match) -- there is no `$?` token anywhere in this shape to anchor on
#   - the `&&`-chain variant of the same family
#     (`x="$(cmd1 && cmd2)"`, where `cmd2`'s legitimate nonzero exit aborts
#     the assignment before a handler written for `cmd2` failing ever runs)
#   - the process-substitution family (`done < <(producer)` /
#     `mapfile -t arr < <(producer)` silently discarding a producer's
#     failure as zero iterations) -- there is no assignment line at all
#   - `set -e` disabled inside a function or subshell the caller assumes
#     is still covered
#   - a `$?` read more than one line after the assignment (through an
#     intermediate variable, inside a different function, guarded by an
#     `if` in between)
#
# Those need a human re-read of each script. `doc/claim-audit/tools-ci-gates.md`
# records the anchors round 18 used to find them, so a future round can
# re-run the search instead of re-reading every script from scratch.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
cd "$repo_root"

mapfile -t scripts < <(
  git ls-files --deduplicate -- 'tools/ci/*.sh' 'tools/moveit-oracle/*.sh' 'ros/*.sh' 'tools/mpr-vs-epa/*.sh' | sort
)
require_nonempty "${#scripts[@]}" "shell script under tools/ or ros/"

status=0

for f in "${scripts[@]}"; do
  # awk's own exit status here is always 0 -- there is no `exit` call in the
  # program below, so it always runs to EOF and falls off the end. Nothing
  # here needs a `|| true` guard: the finding is carried in `$hits` being
  # non-empty, not in a nonzero exit code.
  hits="$(awk '
    function is_blank_or_comment(line) {
      return (line ~ /^[[:space:]]*$/) || (line ~ /^[[:space:]]*#/)
    }
    {
      if (is_blank_or_comment($0)) next
      if (pending != "" && $0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*=\$\?[[:space:]]*$/) {
        print pending_lineno ": " pending
        print FNR ": " $0
      }
      pending = ($0 ~ /\)"?[[:space:]]*$/ && $0 !~ /\|\|[[:space:]]*$/) ? $0 : ""
      pending_lineno = FNR
    }
  ' "$f")"
  if [ -n "$hits" ]; then
    echo "$f: closes a command substitution with no trailing '||', then captures" >&2
    echo "  \$? on the very next non-blank, non-comment line. A failure inside the" >&2
    echo "  substitution aborts the script at the assignment under set -e, so this" >&2
    echo "  capture -- and anything it guards -- is dead code (48ef7ce's shape):" >&2
    echo "$hits" | sed 's/^/    /' >&2
    status=1
  fi
done

if [ "$status" -ne 0 ]; then
  exit 1
fi

echo "OK: no bare \$? capture immediately follows an unguarded command-substitution close"
