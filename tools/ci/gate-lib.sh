# Shared guard for the gate scripts in this directory. Source it:
#
#   . "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
#
# Deliberately named neither `check-*` nor `verify-*`: it is a library, and
# both of those names are globs that run every match as a gate.

# Fails the calling gate when the set it was about to examine is empty.
#
# Every gate here ends with an unconditional `echo OK`, so a loop that runs
# zero times prints the same line as a loop that checked every file --
# "green" then means "nothing was examined", which is the one outcome a gate
# must never spell the same way as success. `verify-vendored-fixture-tests.sh`
# and `verify-all.sh` each grew this guard for their own list after the fact;
# this is that rule in one place, so a new gate gets it by calling one
# function rather than by its author remembering the failure mode.
#
# The current sets are all non-empty (the workspace has crates, the fixture
# directories have fixtures), so no call site below fires today. That is the
# point: the loops are driven off the filesystem precisely so they pick up new
# files without anyone updating a list, and the same property makes them go
# quietly empty if a path convention changes. The guard is what turns that
# into a failure instead of a pass.
#
# What this guard does NOT cover, and which gate needs what:
#
#   - An empty list because the *producer* failed. `mapfile -t x < <(cmd)`
#     throws away `cmd`'s exit status and `set -e` cannot see it, so a broken
#     `git ls-files` yields an empty array with no error anywhere. Where a
#     non-empty list is the failure condition, `require_nonempty` catches that
#     as a side effect -- the list is empty either way and the gate fails
#     either way. That is the only reason those sites are safe.
#   - Where an *empty* list is the pass condition, `require_nonempty` is not
#     applicable and there is nothing left to notice the producer's failure.
#     `check-audit-scripts-not-copied.sh` was the one such gate here and it
#     passed vacuously in a `.git`-less tree until it was rewritten to read the
#     list through a checked command substitution.
#
# So: `require_nonempty` asserts what the gate expected to find, not that the
# command that produced it worked. Read every list with `x="$(cmd)"` (under
# `set -e` and `pipefail`, that propagates) or an explicit `if ! x="$(cmd)"`,
# and use `require_nonempty` for the separate question of whether the result
# should have been non-empty.
require_nonempty() {
  local count="$1" what="$2"
  if [ "$count" -eq 0 ]; then
    echo "FAIL found no $what -- this gate would report OK having examined nothing." >&2
    echo "FAIL either the path convention changed or the tree is not what this gate assumes." >&2
    exit 1
  fi
}
