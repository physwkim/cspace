#!/bin/bash
# Thin caller so tools/ci/verify-all.sh's `verify-*.sh` glob reaches
# reconcile-assertion-ledgers.py --verify.
#
# doc/assertion-discrimination-orphans.txt is a generated snapshot, not a
# hand-maintained document -- it goes stale the instant a merge changes the
# scanner corpus or any ledger's citations and the file isn't regenerated.
# A round did exactly that: it read as authoritative (self-dated header,
# 233 lines) while the live tree had already moved to 222 orphans. This
# script is the gate that catches that before it ships, instead of a
# reader trusting a header that no longer matches the tree.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 "$REPO_ROOT/tools/ci/reconcile-assertion-ledgers.py" --verify
