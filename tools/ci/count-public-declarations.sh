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
    if (line ~ ("class[ \t]+" cls "([ \t]|:|\\{|$)")) { in_target=1; depth=0; access="private" }
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
