#!/bin/bash
# Every committed robot description in `fixtures/` must still match the vendored
# tree it was copied from.
#
# `fixtures/*.{urdf,srdf}` are copies rather than reads of
# `third_party/moveit_resources`, because that directory is a gitignored
# external checkout: absent from a fresh clone and from CI, so the workspace
# test run cannot read it. Every parity claim in this port -- FK, jacobian, ACM,
# dynamics, constraints -- is asserted against those copies. If a copy drifts
# from its source, nothing fails; the claims just quietly stop describing the
# robot they name. Several test files assert the byte-identity in a doc comment
# ("verified"), which is a one-time manual check that ages.
#
# Deliberately NOT named `check-*.sh`: that glob is what `.github/workflows/ci.yml`
# and the local gate loop run, and this check needs `third_party/`, which is
# exactly what those runners do not have. A script that always skips in CI reads
# as coverage while providing none. This one requires the vendored tree and
# fails when it is missing, and runs from `run-oracle-sweep.sh`, which already
# requires it.
#
#   tools/ci/verify-fixture-provenance.sh
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

VENDOR="third_party/moveit_resources"
if [[ ! -d "$VENDOR" ]]; then
  echo "$VENDOR is absent -- this check needs the vendored moveit_resources checkout" >&2
  exit 1
fi

# fixture -> vendored source. pr2's two files are both named `robot.xml`
# upstream, which is why the mapping is explicit rather than derived from the
# fixture's own name.
declare -A SOURCE_OF=(
  [fixtures/panda.urdf]="$VENDOR/panda_description/urdf/panda.urdf"
  [fixtures/panda.srdf]="$VENDOR/panda_moveit_config/config/panda.srdf"
  [fixtures/fanuc.urdf]="$VENDOR/fanuc_description/urdf/fanuc.urdf"
  [fixtures/fanuc.srdf]="$VENDOR/fanuc_moveit_config/config/fanuc.srdf"
  [fixtures/dual_arm_panda.srdf]="$VENDOR/dual_arm_panda_moveit_config/config/panda.srdf"
  [fixtures/pr2.urdf]="$VENDOR/pr2_description/urdf/robot.xml"
  [fixtures/pr2.srdf]="$VENDOR/pr2_description/srdf/robot.xml"
)

# Fixtures with no byte-for-byte source, each with the reason. Being on this
# list is a claim that needs its own evidence elsewhere, not an exemption to
# hand out freely.
declare -A GENERATED=(
  [fixtures/dual_arm_panda.urdf]="xacro expansion of dual_arm_panda_moveit_config/config/panda.urdf.xacro; regeneration command is in commit 2bcd7cb's body"
)

status=0

# Driven off the filesystem, not off the table: a fixture added without a
# mapping must fail rather than silently escape the check. That is the whole
# difference between a rule and a list of the files someone remembered.
shopt -s nullglob
for fixture in fixtures/*.urdf fixtures/*.srdf; do
  if [[ -n "${GENERATED[$fixture]:-}" ]]; then
    echo "generated  $fixture -- ${GENERATED[$fixture]}"
    continue
  fi

  source_path="${SOURCE_OF[$fixture]:-}"
  if [[ -z "$source_path" ]]; then
    echo "UNMAPPED   $fixture has no vendored source and is not listed as generated" >&2
    status=1
    continue
  fi

  if [[ ! -f "$source_path" ]]; then
    echo "MISSING    $source_path (source of $fixture) is not in the vendored tree" >&2
    status=1
    continue
  fi

  if cmp -s "$fixture" "$source_path"; then
    echo "identical  $fixture"
  else
    echo "DRIFTED    $fixture no longer matches $source_path" >&2
    diff "$fixture" "$source_path" | head -10 >&2
    status=1
  fi
done

if [[ $status -ne 0 ]]; then
  echo "fixture provenance check failed" >&2
fi
exit "$status"
