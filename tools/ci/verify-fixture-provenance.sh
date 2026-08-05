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
# fails when it is missing, and runs from `verify-oracle-sweep.sh`, which already
# requires it.
#
#   tools/ci/verify-fixture-provenance.sh
set -euo pipefail

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
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

# Fixtures with no byte-for-byte source: xacro expansions, whose "source" is
# a set of `.xacro` files plus the command that expanded them.
#
# `GENERATED_SOURCES` names that set, `GENERATED_DIGEST` pins its content,
# and `GENERATED_COMMAND` records how to reproduce the fixture from it. All
# three, not just the third: this table used to hold prose alone
# ("regeneration command is in commit 2bcd7cb's body"), which is a claim
# about provenance rather than a check of it -- the inputs could drift to a
# different robot and the entry would keep reading the same. A digest over
# the include closure is what makes the claim fail when it stops being true.
#
# What this does and does not establish. It pins the *inputs*: if any
# `.xacro` in the closure changes, the fixture is no longer the expansion of
# what this table says it is, and the check fails. It does not re-run the
# expansion, so it does not prove the committed output is what today's xacro
# would emit from those inputs -- that would need the toolchain, which lives
# in the oracle image (itself pinned by manifest digest, see
# `tools/moveit-oracle/src-digest.sh`) and which this script deliberately
# does not require. The regeneration command below is how that is checked by
# hand.
#
# Paths are relative to the roots resolved above/below: `$VENDOR` for
# fixtures generated out of the vendored `moveit_resources` checkout,
# `$MOVEIT2_SRC` for those generated out of the pinned upstream moveit2
# checkout (prbt ships inside moveit2's own `moveit_planners/test_configs/`,
# not in `moveit_resources`, which is why it needs the second root).
MOVEIT2_SRC="${MOVEIT2_SRC:-$HOME/work/moveit2}"

declare -A GENERATED_SOURCES=(
  [fixtures/dual_arm_panda.urdf]="\
$VENDOR/dual_arm_panda_moveit_config/config/panda.urdf.xacro
$VENDOR/dual_arm_panda_moveit_config/config/panda_arm_macro.xacro
$VENDOR/dual_arm_panda_moveit_config/config/panda.ros2_control.xacro
$VENDOR/dual_arm_panda_moveit_config/config/panda_hand.ros2_control.xacro"
  [fixtures/prbt.urdf]="\
$MOVEIT2_SRC/moveit_planners/test_configs/prbt_support/urdf/prbt.xacro
$MOVEIT2_SRC/moveit_planners/test_configs/prbt_support/urdf/prbt_macro.xacro"
  [fixtures/prbt.srdf]="\
$MOVEIT2_SRC/moveit_planners/test_configs/prbt_moveit_config/config/prbt.srdf.xacro
$MOVEIT2_SRC/moveit_planners/test_configs/prbt_moveit_config/config/prbt_manipulator.srdf.xacro"
)

# sha256 of each `GENERATED_SOURCES` file, in the order listed there.
declare -A GENERATED_DIGEST=(
  [fixtures/dual_arm_panda.urdf]="\
d3a2afc61ac7a7eef548fc792e004d9f653680991ed9eac0efbe85395cefc025
af9a18ddfba13cc1793b4d06d0875f9609fca429deac268cc6613b197aa3635a
5f557013d6948b23013b76f761926783aa85e50fed26143d39ced5231016e041
0d1dbc666f527d3dd60c0f7cc07af541fcd46dd44cdf49aab8edb94ac5a96510"
  [fixtures/prbt.urdf]="\
beb34073e5abd710351298c93865b62661b92b9e357b313f3e1ff16614a28778
31ba3d0be4da797408697446cbf0d5ebc9a1ba4d1ee36ca609aad96e08e9da47"
  [fixtures/prbt.srdf]="\
612170ee11201b7d958597fba1784434b5f8fffcf79efe3a6f0520ea38158995
4fd035b0841737f130206fdfbe82006573f10653a6d568fd800b59827957f69b"
)

declare -A GENERATED_COMMAND=(
  [fixtures/dual_arm_panda.urdf]="see commit 2bcd7cb's body (colcon-builds dual_arm_panda_moveit_config into a scratch workspace so \$(find ...) resolves, then xacro)"
  [fixtures/prbt.urdf]="sg docker -c 'docker run --rm --user \$(id -u):\$(id -g) -v <out>:<out>:rw --entrypoint bash <oracle image> -lc \"source /opt/ros/\\\$ROS_DISTRO/setup.bash && source /ws/install/setup.bash && xacro \\\$(ros2 pkg prefix moveit_resources_prbt_support)/share/moveit_resources_prbt_support/urdf/prbt.xacro\"' > fixtures/prbt.urdf"
  [fixtures/prbt.srdf]="same container/setup as fixtures/prbt.urdf, expanding \$(ros2 pkg prefix moveit_resources_prbt_moveit_config)/share/moveit_resources_prbt_moveit_config/config/prbt.srdf.xacro"
)

status=0

# A generated fixture's sources are pinned by digest; drift in any one of
# them means the fixture is no longer the expansion of what the table says.
check_generated() {  # <fixture path>
  local fixture="$1"
  local -a sources digests
  mapfile -t sources <<<"${GENERATED_SOURCES[$fixture]}"
  mapfile -t digests <<<"${GENERATED_DIGEST[$fixture]:-}"

  if [[ ${#sources[@]} -ne ${#digests[@]} ]]; then
    echo "UNMAPPED   $fixture lists ${#sources[@]} source(s) but ${#digests[@]} digest(s)" >&2
    status=1
    return
  fi
  require_nonempty "${#sources[@]}" "xacro source for $fixture"

  local i source want have
  for i in "${!sources[@]}"; do
    source="${sources[$i]}"
    want="${digests[$i]}"
    if [[ ! -f "$source" ]]; then
      echo "MISSING    $source (xacro source of $fixture) is not present" >&2
      status=1
      continue
    fi
    have="$(sha256sum "$source" | cut -d' ' -f1)"
    if [[ "$have" != "$want" ]]; then
      echo "DRIFTED    $source (xacro source of $fixture) changed" >&2
      echo "             recorded $want" >&2
      echo "             on disk  $have" >&2
      echo "           regenerate: ${GENERATED_COMMAND[$fixture]}" >&2
      status=1
    fi
  done
  echo "generated  $fixture -- xacro expansion, ${#sources[@]} pinned source(s)"
}

check_fixture() {  # <fixture path> <vendored source path>
  local fixture="$1" source_path="$2"

  if [[ -n "${GENERATED_SOURCES[$fixture]:-}" ]]; then
    check_generated "$fixture"
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
root_fixtures=(fixtures/*.urdf fixtures/*.srdf)
require_nonempty "${#root_fixtures[@]}" "urdf/srdf fixture under fixtures/"
for fixture in "${root_fixtures[@]}"; do
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
mesh_fixtures=(fixtures/meshes/**/*.stl)
require_nonempty "${#mesh_fixtures[@]}" "collision mesh under fixtures/meshes/"
for fixture in "${mesh_fixtures[@]}"; do
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
  [crates/moveit-metrics/tests/fixtures/panda.srdf]="adds panda_base (isolates virtual_joint, type=floating, for joint_limits_penalty's floating-joint skip) and panda_arm_5dof (panda_link0 to panda_link5, 5 active revolute joints, for manipulability_index/manipulability's columns < 6 SVD-product branch); every joint and link is the real panda URDF's, only the two group boundaries are new -- reason is restated in the file itself"
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

crate_local_fixtures=(crates/*/tests/fixtures/*.urdf crates/*/tests/fixtures/*.srdf)
require_nonempty "${#crate_local_fixtures[@]}" "crate-local fixture under crates/*/tests/fixtures/"
for fixture in "${crate_local_fixtures[@]}"; do
  check_crate_local "$fixture"
done

if [[ $status -ne 0 ]]; then
  echo "fixture provenance check failed" >&2
fi
exit "$status"
