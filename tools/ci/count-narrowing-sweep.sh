#!/usr/bin/env bash
# Usage: tools/ci/count-narrowing-sweep.sh <upstream.cpp|upstream.hpp> ...
#
# moveit-scene / moveit-metrics "§172 narrowing sweep" convention, made
# reproducible (see doc/claim-audit/moveit-scene.md and
# doc/claim-audit/moveit-metrics.md, "§172 narrowing sweep" section):
# mechanically list every candidate integer-narrowing site in a list of
# upstream C++ files -- either (a) a declaration of one of the integer
# types this convention watches (`int`, `unsigned`, `unsigned int`,
# `long`, `size_t`, `std::size_t`, `uint32_t`, `int32_t`), or (b) a
# `static_cast<...>` to one of those types. One line of output per hit,
# `file:line:matched-text`.
#
# Comments (`//` and `/* */`, including multi-line) and string-literal
# contents are stripped first, the same way count-public-declarations.sh
# does, so line numbers in the output still match the original file
# (block comments are replaced by an equal count of blank lines, not
# deleted, to keep numbering intact).
#
# This script only lists raw textual hits -- it is not a C++ parser.
# Two known false-positive shapes it cannot distinguish from a real
# declaration: a method whose *return type* is one of these integer
# types (`std::size_t size() const` reads as "declare `size`"), and a
# `new TYPE[...]` array-new expression (`new unsigned int[n]` reads as
# "declare a variable named `int`"). Likewise it cannot tell a bare
# function-signature parameter (no initializer) from a local variable.
# Classifying each hit -- real local declaration vs. parameter/field
# vs. false-positive text match vs. genuine float-to-int narrowing -- is
# a judgment call made in the doc prose, not by this script, matching
# count-public-declarations.sh's and count-relative-eq.pl's convention.
#
# Deliberately not wired into check-*.sh or verify-*.sh: there is no exit
# code to gate on -- the script always exits 0 and prints raw hits, and
# whether a given hit is real narrowing or a false-positive text match is
# the judgment call above, not something this script (or any script) can
# assert. Its inputs are files from the upstream checkout PORTING-PLAN.md
# pins at one fixed SHA (`e017c91e`, changed only by an explicit rebase
# round), so there is also nothing here that would drift between two CI
# runs for a gate to catch. Reproduce moveit-scene/moveit-metrics's own
# recorded totals (doc/claim-audit/moveit-scene.md,
# doc/claim-audit/moveit-metrics.md, "§172 narrowing sweep") with:
#
#   M2=/home/stevek/work/moveit2/moveit_core
#   tools/ci/count-narrowing-sweep.sh \
#     "$M2/planning_scene/src/planning_scene.cpp" \
#     "$M2/planning_scene/include/moveit/planning_scene/planning_scene.hpp" \
#     "$M2/robot_state/src/robot_state.cpp" \
#     "$M2/robot_state/include/moveit/robot_state/attached_body.hpp" \
#     "$M2/robot_state/src/attached_body.cpp" \
#     "$M2/collision_detection/src/world.cpp" \
#     "$M2/collision_detection/include/moveit/collision_detection/world.hpp" \
#     "$M2/kinematic_constraints/src/kinematic_constraint.cpp" \
#     | wc -l   # 140, matching moveit-scene.md
#   tools/ci/count-narrowing-sweep.sh \
#     "$M2/kinematics_metrics/src/kinematics_metrics.cpp" | wc -l   # 4, matching moveit-metrics.md
set -euo pipefail

TYPES='std::size_t|size_t|uint32_t|int32_t|unsigned int|unsigned|long|int'

for f in "$@"; do
  perl -0777 -pe '
    s{/\*.*?\*/}{"\n" x (($&) =~ tr/\n//)}gse;
    s{//[^\n]*}{}g;
    s{"(?:[^"\\]|\\.)*"}{""}gs;
  ' "$f" | grep -noE "\\b(${TYPES})\\s+[A-Za-z_][A-Za-z0-9_]*|static_cast<(${TYPES})>" \
    | sed "s#^#$f:#" || true
done
