#!/usr/bin/env bash
# Usage: tools/ci/count-public-declarations.sh <header.h> <ClassName>
#
# Counts raw `public:` declarations belonging to <ClassName>'s own
# class body in an upstream C++ header, under this crate's audit
# counting convention (see tree.rs's "Symbol-by-symbol audit" module
# doc): one count per semicolon-terminated statement or complete
# `{ ... }` inline body, however many lines it spans; nested
# classes/structs (and their own `public:` sections) are excluded --
# only members at brace-depth 1 of the named class count. A
# forward-declared nested class (`class Foo;`, no body) still counts
# as one raw textual declaration here; whether the audit's stated
# collapsing rule then excludes it from the "expected" bullet tally
# is a judgment call made in the doc prose, not by this script.
#
# Comments (`//` and `/* */`, including multi-line) are stripped
# first so neither doc-comment prose nor `@code` examples are
# mistaken for declarations. A bare `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`
# macro invocation (no trailing `;`, geometric_shapes's `bodies.h`
# idiom for over-aligned Eigen members) is skipped the same way a
# preprocessor directive is -- it is not itself a member declaration.
#
# Round 19 item 1: a `"..."` string literal containing a bare `{` or `}`
# (confirmed absent from every header this script has actually been run
# against so far -- `grep -no '"[^"]*[{}][^"]*"'` on each finds nothing)
# would corrupt the brace-depth counter below, which counts every `{`/`}`
# character in a line textually. String literal contents are blanked for
# the same reason comments are stripped first, not because this repo's
# headers have hit it yet.
#
# The class head may carry one visibility-macro token between `class` and the
# name -- `class MOVEIT_MOVE_GROUP_INTERFACE_EXPORT MoveGroupInterface`. Before
# that was allowed the script answered `0` for such a class, which reads as "no
# public declarations" and not as "this class was never found"; a caller with
# no independent count would have believed it. Widening only adds matches:
# `bodies.h`'s six classes answer 28/16/16/16/20/12 either way.
#
# Deliberately not wired into check-*.sh or verify-*.sh: it prints a raw
# integer with no pass/fail sense of its own -- whether that count matches
# the Rust port's own public surface is a per-class judgment call made in
# each crate's doc comment (see bodies.rs, shapes.rs, tree.rs), not
# something this script asserts. There is also no fixed <header, class>
# pair to run it against in CI: each new class ported gets a new
# invocation the day it is audited, chosen by the person doing the audit,
# not by this script. Self-check with no docker required (matches
# moveit-geometry/src/lib.rs's recorded `0`, since a bash file has no
# `class` to match):
#
#   bash tools/ci/count-public-declarations.sh \
#     tools/ci/count-public-declarations.sh count_public_declarations
#
# Reproduce a real, oracle-backed count (matches bodies.rs's recorded
# `Body: 28`):
#
#   sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 \
#     -c 'cat /opt/ros/rolling/include/geometric_shapes/geometric_shapes/bodies.h'" > /tmp/bodies.h
#   tools/ci/count-public-declarations.sh /tmp/bodies.h Body   # 28
set -euo pipefail
header="$1"
cls="$2"
perl -0777 -pe 's{/\*.*?\*/}{}gs; s{//[^\n]*}{}g; s{"(?:[^"\\]|\\.)*"}{""}g' "$header" | awk -v cls="$cls" '
BEGIN { depth=-1; in_target=0; access="private"; pending=0; entered_brace=0; count=0 }
{
  line = $0
  if (line ~ /^[ \t]*$/) next
  if (!pending && line ~ /^[ \t]*#/) next
  if (!pending && line ~ /^[ \t]*EIGEN_MAKE_ALIGNED_OPERATOR_NEW[ \t]*$/) next
  if (!in_target) {
    if (line ~ ("class[ \t]+([A-Z_][A-Z_0-9]*[ \t]+)?" cls "([ \t]|:|\\{|$)")) { in_target=1; depth=0; access="private" }
    else next
  }
  n_open = gsub(/\{/, "{", line)
  n_close = gsub(/\}/, "}", line)
  if (depth==0) { depth += n_open - n_close; next }
  if (depth==1 && !pending && n_close>0 && (depth+n_open-n_close)<=0) {
    depth += n_open - n_close; in_target=0; next  # class-closing `};`, not a declaration
  }
  if (depth==1 && !pending) {
    if (line ~ /^[ \t]*public:[ \t]*$/) { access="public"; depth+=n_open-n_close; next }
    if (line ~ /^[ \t]*(protected|private):[ \t]*$/) { access="other"; depth+=n_open-n_close; next }
  }
  if (access=="public") {
    if (!pending) {
      depth_after = depth + n_open - n_close
      if (n_open>0) { entered_brace=1; if (depth_after==1) count++; else pending=1 }
      else { if (line ~ /;[ \t]*$/) count++; else { pending=1; entered_brace=0 } }
    } else {
      if (!entered_brace && n_open>0) entered_brace=1
      depth_after = depth + n_open - n_close
      if (entered_brace) { if (depth_after==1) { count++; pending=0; entered_brace=0 } }
      else { if (line ~ /;[ \t]*$/) { count++; pending=0 } }
    }
  }
  depth += n_open - n_close
  if (depth<=0) in_target=0
}
END { print count+0 }'
