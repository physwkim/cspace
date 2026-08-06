#!/bin/bash
# Runs `classify-unported.py --phase-table-only`: every not-yet-MET row of
# PORTING-PLAN.md §5's status table must have an UNMET_BLOCKERS entry, and that
# entry must cite the same § the row cites.
#
# This wrapper exists because of the prefix convention, not in spite of it.
# `classify-unported.py` is named `classify-` and stays out of `ci.yml`'s
# `check-*` glob correctly: its main job classifies every unported file and
# needs the moveit2 checkout, which the glob's contract (python3 plus the
# tracked files, no docker, no cargo, no upstream) does not provide. But the §5
# cross-check inside it reads PORTING-PLAN.md and nothing else, and it is the
# part that keeps going stale -- it has caught four citation drifts, every one
# of them only because a human happened to run the script by hand. Two more sat
# in the tree until a merge tripped over them.
#
# So the upstream-free half gets its own entry point and the upstream half stays
# where it is. Whoever changes a §5 row's evidence column now hears about the
# copy in UNMET_BLOCKERS from CI rather than from the next person to run
# `--emit`.
#
# The full classification, which does need `--upstream`, still has to be run by
# hand after any change to the unported set:
#     tools/ci/classify-unported.py --emit doc/unported-classification.md
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

exec "$REPO_ROOT/tools/ci/classify-unported.py" \
  --phase-table-only --repo "$REPO_ROOT"
