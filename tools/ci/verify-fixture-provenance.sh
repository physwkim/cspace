#!/bin/bash
# Every committed robot description must still match what it was copied from --
# `fixtures/*.{urdf,srdf}` against the vendored tree, and the crate-local copies
# under `crates/*/tests/fixtures/` against those.
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

check_fixture() {  # <fixture path> <vendored source path>
  local fixture="$1" source_path="$2"

  if [[ -n "${GENERATED[$fixture]:-}" ]]; then
    echo "generated  $fixture -- ${GENERATED[$fixture]}"
    return
  fi

  if [[ -z "$source_path" ]]; then
    echo "UNMAPPED   $fixture has no vendored source and is not listed as generated" >&2
    status=1
    return
  fi

  if [[ ! -f "$source_path" ]]; then
    echo "MISSING    $source_path (source of $fixture) is not in the vendored tree" >&2
    status=1
    return
  fi

  if cmp -s "$fixture" "$source_path"; then
    echo "identical  $fixture"
  else
    echo "DRIFTED    $fixture no longer matches $source_path" >&2
    diff "$fixture" "$source_path" | head -10 >&2
    status=1
  fi
}

# Driven off the filesystem, not off the table: a fixture added without a
# mapping must fail rather than silently escape the check. That is the whole
# difference between a rule and a list of the files someone remembered.
shopt -s nullglob
for fixture in fixtures/*.urdf fixtures/*.srdf; do
  check_fixture "$fixture" "${SOURCE_OF[$fixture]:-}"
done

# Collision meshes under fixtures/meshes/<package>/... mirror
# $VENDOR/<package>/... path-for-path (that is how they were copied in), so
# unlike the urdf/srdf table above -- where pr2's two files are both named
# `robot.xml` upstream and a derived mapping would collide -- the vendored
# path here is mechanically the fixture path with the leading
# `fixtures/meshes/` swapped for `$VENDOR/`. Still driven off the filesystem,
# not a table: a new file under fixtures/meshes/ is checked automatically,
# with no mapping entry to remember to add.
shopt -s globstar
for fixture in fixtures/meshes/**/*.stl; do
  check_fixture "$fixture" "$VENDOR/${fixture#fixtures/meshes/}"
done

# Crate-local robot descriptions under `crates/*/tests/fixtures/` are a second
# generation of copy: copies of the copies above, made so a crate's tests (and
# now `verify-fixture-replay.sh`'s manifests) can resolve a urdf/srdf relative
# to their own fixture directory. Twelve existed before this check did, and
# nothing compared them to anything -- the table above stops at `fixtures/`, so
# a crate-local copy could drift from the root copy, and the root copy stay
# provenance-clean, and every parity claim in that crate quietly describe a
# different robot than its name says.
#
# The rule is byte-identity with the root fixture of the same basename, because
# that is the one the table above ties to the vendored source. Chaining the two
# is what makes a crate-local copy provenance-checked at all.
#
# Two kinds of exception, both requiring a table entry rather than silence:
# DIVERGENT for a deliberate edit, SYNTHETIC for a robot that is not a copy of
# anything. A file that is neither identical nor listed fails, so a new
# crate-local description cannot escape by being forgotten.
declare -A DIVERGENT=(
  [crates/moveit-kinematics/tests/fixtures/pr2.srdf]="adds an l_gripper_finger_chain group (one active + one mimic joint in a single is_chain() group) that none of PR2's own upstream groups isolate; the joint types and l_gripper_l_finger_tip_joint's mimic multiplier/offset are the real PR2 URDF's, only the group boundary is new -- reason is restated in the file itself"
)

declare -A SYNTHETIC=(
  [crates/moveit-collision/tests/fixtures/octree_world_robot.urdf]="hand-written single-link robot for octree world-collision tests; not a copy of any vendored description"
  [crates/moveit-collision/tests/fixtures/octree_world_robot.srdf]="companion srdf for octree_world_robot.urdf"
  [crates/moveit-trajectory/tests/fixtures/totg_synthetic.urdf]="hand-written robot for the synthetic TOTG case; not a copy of any vendored description"
  [crates/moveit-trajectory/tests/fixtures/totg_synthetic.srdf]="companion srdf for totg_synthetic.urdf"
)

check_crate_local() {  # <crate-local fixture path>
  local fixture="$1"
  local root="fixtures/$(basename "$fixture")"

  if [[ -n "${SYNTHETIC[$fixture]:-}" ]]; then
    echo "synthetic  $fixture -- ${SYNTHETIC[$fixture]}"
    return
  fi

  if [[ -n "${DIVERGENT[$fixture]:-}" ]]; then
    if cmp -s "$fixture" "$root"; then
      echo "STALE      $fixture is listed as divergent from $root but is now identical to it" >&2
      status=1
    else
      echo "divergent  $fixture -- ${DIVERGENT[$fixture]}"
    fi
    return
  fi

  if [[ ! -f "$root" ]]; then
    echo "UNMAPPED   $fixture has no $root to match and is not listed as synthetic" >&2
    status=1
    return
  fi

  if cmp -s "$fixture" "$root"; then
    echo "identical  $fixture -> $root"
  else
    echo "DRIFTED    $fixture no longer matches $root" >&2
    diff "$fixture" "$root" | head -10 >&2
    status=1
  fi
}

for fixture in crates/*/tests/fixtures/*.urdf crates/*/tests/fixtures/*.srdf; do
  check_crate_local "$fixture"
done

if [[ $status -ne 0 ]]; then
  echo "fixture provenance check failed" >&2
fi
exit "$status"
