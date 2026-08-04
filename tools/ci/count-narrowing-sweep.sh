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
