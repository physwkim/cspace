#!/bin/bash
# Re-derives the 49-cell measurement behind `doc/upstream-bugs.md`'s
# `shape-intersect-tangency-follows-libccd-dispatch`: at a gap of exactly
# zero, `fcl::collide` reports a contact iff the shape pair has a non-libccd
# specialisation registered.
#
# §201's rule is why this exists rather than a scratch directory. The entry's
# central claim ("49 of 49, no exception") came from a probe compiled inside
# the oracle image; without the source and a runner in the tree, nothing could
# re-derive it and nothing would notice if a later fcl stopped behaving that
# way. `tools/mpr-vs-epa/` is the precedent.
#
# Three things are checked (no args), and they are independent:
#
#   1. the measured tangency answer per pair, against the table below;
#   2. that same answer against the specialised set *parsed out of fcl's own
#      header inside the image* -- so the mechanism claim is re-derived, not
#      restated. A future fcl that registers a new pair moves both sides and
#      the run still agrees; one that changes the boundary convention without
#      changing the registrations fails check 2 while check 1 tells you which
#      cell moved.
#   3. the committed `Box`/`Sphere`/`Cylinder`/`Cone` dispatch table
#      `crates/moveit-collision/src/fcl_tangency_table.rs` -- the one
#      `accumulate_collision` actually reads at `dist == 0.0` -- against that
#      same freshly-parsed specialised set. Checks 1/2 compare the *probe's*
#      own answer; this compares the file a `cargo build` reads, which is a
#      separate artifact a hand-edit or a stale `--emit` could drift from.
#
# `--emit <path>` writes that table (regenerated from a fresh image parse,
# never hand-edited) to `<path>` and exits, skipping checks 1-3.
#
# Needs docker (via `sg`, per this repo's wrapper rule) and the digest-gated
# oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

GENERATED="$REPO_ROOT/crates/moveit-collision/src/fcl_tangency_table.rs"

EMIT_PATH=""
if [[ "${1:-}" == "--emit" ]]; then
  EMIT_PATH="${2:?usage: $0 --emit <path>}"
fi

PROBE="$REPO_ROOT/tools/fcl-tangency-probe/probe.cpp"

# The pinned answer at delta == 0, rows = upper shape, cols = lower shape,
# both in the order `probe.cpp` declares its shapes -- not the alphabetical or
# by-family order the prose tables use, which is how the first draft of this
# block was wrong in four cells until check 1 said so.
# `T` = `fcl::collide` reported a collision.
#         box sph cyl con cap ell cvx
EXPECTED="\
box       T T F F F F F
sphere    T T T F T F F
cylinder  F T F F F F F
cone      F F F F F F F
capsule   F T F F F F F
ellipsoid F F F F F F F
convex    F F F F F F F"

if ! command -v docker >/dev/null 2>&1; then
  skip_not_measured blocked \
    "docker not on PATH -- the 49-cell fcl dispatch measurement is not re-derived." \
    "this is not a pass."
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"

stamp="$(oracle_stamp_verdict "$IMAGE" "$want")"
if [ "$stamp" != ok ]; then
  # A docker this shell cannot reach is not a skip -- nothing was measured.
  # `oracle_stamp_explain` returns nonzero for exactly that cause, because
  # `verify-all.sh` reads each gate's exit status and not these lines, so
  # exiting 0 would report it as a pass.
  oracle_stamp_explain "$stamp" "$IMAGE" "$want" "SKIP " || exit 1
  skip_not_measured blocked "this is not a pass -- the oracle was never consulted."
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
cp "$PROBE" "$work/probe.cpp"

# The image ships fcl as headers plus libfcl.so; the probe is a single
# translation unit, so g++ inside the image is the whole build.
start=$SECONDS
docker run --rm -v "$work:$work" -w "$work" --entrypoint bash "$IMAGE" -c '
  set -euo pipefail
  # fcl/common/types.h includes <Eigen/Dense> unqualified, so Eigen'"'"'s own
  # include dir has to be on the path -- the image has it under eigen3/.
  g++ -O2 -std=c++17 $(pkg-config --cflags eigen3) -o probe probe.cpp -lfcl -lccd 2>&1
  ./probe > cells.csv
  # fcl draws the non-libccd set itself; parse the registrations rather than
  # trusting a copy of the table.
  # Drop the #define lines first: the macro definition itself reads
  # FCL_GJK_LIBCCD_SHAPE_INTERSECT(SHAPE, ALG) and would otherwise enter the
  # set as a "shape/shape" pair, inflating the count this script prints.
  grep -v "^#define" /usr/include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h |
    grep -oE "FCL_GJK_LIBCCD_SHAPE(_SHAPE)?_INTERSECT\([A-Za-z]+(, *[A-Za-z]+)?," \
    > registrations.txt
  # The macros above are ONE of fcl'"'"'s two non-libccd registration mechanisms, not
  # the only one: gjk_solver_libccd-inl.h also hand-writes
  # `ShapeIntersectLibccdImpl<S, Shape1<S>, Shape2<S>>` partial specializations
  # (Halfspace/Plane today) entirely outside these macros. Every line opening a
  # `struct ShapeIntersectLibccdImpl` declaration -- specialized or not -- goes to
  # specializations.txt so the parser below can tell the primary template (no `<...>`
  # argument list) from a partial specialization (has one) by C++ grammar, not by
  # which shapes happen to be specialized today.
  grep -nE "^struct ShapeIntersectLibccdImpl" \
    /usr/include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h \
    > specializations.txt
  dpkg-query -W libfcl-dev > fclver.txt 2>/dev/null || echo "libfcl-dev <unknown>" > fclver.txt
' >"$work/build.log" 2>&1 || { echo "FAIL probe build/run failed inside $IMAGE:"; sed 's/^/  | /' "$work/build.log"; exit 1; }
elapsed=$((SECONDS - start))

echo "== $(cat "$work/fclver.txt") in $IMAGE, ${elapsed}s"

if [[ -n "$EMIT_PATH" ]]; then
  EMIT_PATH="$EMIT_PATH" python3 - "$work/registrations.txt" "$work/specializations.txt" <<'PY'
import os, re, sys

regs_path, specs_path = sys.argv[1], sys.argv[2]
emit_path = os.environ["EMIT_PATH"]

# Same parse as check 2 below -- kept in sync by hand since this runs in a
# separate mode (`--emit` skips checks 1-3 entirely, see this script's own
# header).
text = open(regs_path).read()
spec = set()
for a in re.findall(r"FCL_GJK_LIBCCD_SHAPE_INTERSECT\((\w+),", text):
    spec.add((a.lower(), a.lower()))
for a, b in re.findall(r"FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT\((\w+), *(\w+),", text):
    spec.add((a.lower(), b.lower()))
    spec.add((b.lower(), a.lower()))
if not spec:
    sys.exit("FAIL parsed no registrations out of gjk_solver_libccd-inl.h -- the macro names moved")

# Second registration mechanism: hand-written `ShapeIntersectLibccdImpl<S, Shape1<S>,
# Shape2<S>>` partial specializations (Halfspace/Plane today), invisible to the macro
# regexes above. `struct_lines` is every `struct ShapeIntersectLibccdImpl` declaration
# found by the loosest possible anchor -- the class name alone, no assumption about
# which shapes are specialized -- so a future specialization this parse can't read
# still shows up as "found but unexplained" instead of silently missing `spec`.
struct_lines = open(specs_path).read().splitlines()
# The primary (unspecialised) template names the class with no explicit `<...>`
# argument list -- that C++ grammar distinction, not gjk_solver_libccd-inl.h:112's line
# number, is what excludes it here.
specialization_lines = [ln for ln in struct_lines if re.match(r"^\d+:struct ShapeIntersectLibccdImpl<", ln)]
SPEC_ARGS_RE = re.compile(r"^\d+:struct ShapeIntersectLibccdImpl<S,\s*(\w+)<S>,\s*(\w+)<S>>\s*$")
unexplained = []
for ln in specialization_lines:
    m = SPEC_ARGS_RE.match(ln)
    if not m:
        unexplained.append(ln)
        continue
    a, b = m.group(1).lower(), m.group(2).lower()
    spec.add((a, b))
    spec.add((b, a))
if unexplained:
    sys.exit(
        f"FAIL {len(unexplained)} hand-written ShapeIntersectLibccdImpl specialization(s) "
        "in gjk_solver_libccd-inl.h are not explained by the parsed specialised set:\n"
        + "\n".join(f"  {ln}" for ln in unexplained)
    )

# Only the four kinds `crates/moveit-collision/src/parry.rs`'s `TangencyKind`
# classifies a shape into -- `capsule`/`ellipsoid`/`convex` have no
# `moveit_geometry::Shape` variant, and `mesh` is a third path (BVHModel),
# not a libccd specialisation, so it is not in this set at all; `parry.rs`
# hard-codes mesh's own "always true" rule instead of reading it from here.
KINDS = ["box", "sphere", "cylinder", "cone"]
rows = [[(a, b) in spec for b in KINDS] for a in KINDS]

def rust_row(kind, row):
    cells = ", ".join("true" if v else "false" for v in row)
    return f"    /* {kind:<8} */ [{cells}],"

lines = [
    "// Copyright (c) 2026, moveit-rs contributors",
    "// SPDX-License-Identifier: BSD-3-Clause",
    "",
    "// GENERATED by tools/ci/verify-fcl-tangency-dispatch.sh --emit",
    "//   crates/moveit-collision/src/fcl_tangency_table.rs",
    "// Do not hand-edit: running tools/ci/verify-fcl-tangency-dispatch.sh with no",
    "// arguments fails if this drifts from a fresh parse of the oracle image's own",
    "// `gjk_solver_libccd-inl.h` -- see that script's module doc for what its",
    "// third check verifies.",
    "",
    "//! fcl's non-libccd (specialised) shape-intersect registrations, restricted",
    "//! to the four kinds `crates/moveit-collision/src/parry.rs`'s `TangencyKind`",
    "//! classifies a shape into (`Box`/`Sphere`/`Cylinder`/`Cone` -- `Mesh` is not",
    "//! in this table; `parry.rs` hard-codes mesh's own \"always true\" rule",
    "//! instead, since fcl maps it to a `BVHModel` traversal this header does not",
    "//! register). Row/column order matches the array below; the table is",
    "//! symmetric because `FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT` registers both",
    "//! directions.",
    "//!",
    "//! Provenance, independent of the oracle image this file is machine",
    "//! generated from (that mechanism is docker-gated --",
    "//! `tools/ci/verify-fcl-tangency-dispatch.sh`'s own doc -- and its cited",
    "//! `doc/upstream-bugs.md` entry no longer exists, deleted by `f7186386` when",
    "//! the porting-doc tree was replaced by `GOALS.md`, so that particular trail",
    "//! cannot be followed further): the same four registrations, re-derived by",
    "//! hand against a live `/home/stevek/work/fcl` checkout at",
    "//! `0.7.0-17-ge5efcc4`, `rg`-searched rather than parsed by script, but",
    "//! against the identical macros `verify-fcl-tangency-dispatch.sh --emit`",
    "//! reads. `gjk_solver_libccd-inl.h:245,246,250,252` --",
    "//! `FCL_GJK_LIBCCD_SHAPE_INTERSECT(Sphere, detail::sphereSphereIntersect)`,",
    "//! `FCL_GJK_LIBCCD_SHAPE_INTERSECT(Box, detail::boxBoxIntersect)`,",
    "//! `FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Box,",
    "//! detail::sphereBoxIntersect)`,",
    "//! `FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT(Sphere, Cylinder,",
    "//! detail::sphereCylinderIntersect)` -- are exactly the four `true` cells",
    "//! below, no more and no fewer. Every other",
    "//! `FCL_GJK_LIBCCD_SHAPE(_SHAPE)?_INTERSECT` line in that header",
    "//! (`:248,254-260,262-267`) names `Capsule`, `Halfspace`, `Plane`,",
    "//! `Ellipsoid` or `Convex`, none of which is one of these four kinds; in",
    "//! particular `Cone` never appears as either argument to either macro",
    "//! against `Box`, `Sphere` or `Cylinder`, or against itself, in this header",
    "//! at all. `gjk_solver_indep-inl.h` (fcl's alternate, non-libccd GJK",
    "//! backend) registers the identical four pairs against the identical",
    "//! algorithm functions (`:249,250,254,256`), so which solver `fcl::collide`",
    "//! picks does not change this table; `GST_LIBCCD` is fcl's own default",
    "//! regardless (`collision_request.h:102`).",
    "//!",
    "//! The `false` cells are not the same strength of claim as the `true` ones.",
    "//! A `true` cell is a single closed-form routine, proven non-strict at the",
    "//! boundary by reading its body -- see `accumulate_collision`'s own doc,",
    "//! just above its `fcl_tangency_verdict(...) == Some(true)` branch, for all",
    "//! four routines and their exact comparison operators. A `false` cell means",
    "//! no such routine is registered for that pair at all: `cone` has zero",
    "//! registrations against `box`, `sphere`, `cylinder` or itself in either",
    "//! header, so every cell in its row and column falls through to fcl's",
    "//! generic libccd MPR path, which the oracle image's probe measured `false`",
    "//! at the one tie configuration it constructed. That is consistent with,",
    "//! but weaker than, an algebraic guarantee: `discoverPortal`",
    "//! (`/home/stevek/work/libccd`, checkout `7931e76`, `mpr.c:189,209,232`)",
    "//! rejects on `ccdIsZero(dot) || dot < 0`, a test over the direction MPR's",
    "//! portal discovery happens to pick, unlike the closed-form routines'",
    "//! orientation-independent `s2 > 0`. A `false` cell reaching",
    "//! `fcl_tangency_verdict` is `Some(false)`, not `None` --",
    "//! `touches_at_tie`'s `.unwrap_or(true)` never applies to it, so a cone",
    "//! pair sitting inside the rounding band reads as the positive \"not",
    "//! touching\" this table asserts, not an unknown the fallback would",
    "//! otherwise resolve to \"touching\". This crate has not measured whether",
    "//! that holds at every cone-pair orientation, only the one the oracle probe",
    "//! constructed.",
    "",
    "pub(crate) const SPECIALISED: [[bool; 4]; 4] = [",
    "    //             box    sphere cylinder cone",
] + [rust_row(k, r) for k, r in zip(KINDS, rows)] + [
    "];",
    "",
]
with open(emit_path, "w") as f:
    f.write("\n".join(lines) + "\n")
true_count = sum(sum(r) for r in rows)
print(f"OK emitted {emit_path} ({true_count} of {len(KINDS) ** 2} cells true)")
PY
  exit 0
fi

EXPECTED="$EXPECTED" python3 - "$work/cells.csv" "$work/registrations.txt" "$work/specializations.txt" "$GENERATED" <<'PY'
import csv, os, re, sys

cells_path, regs_path, specs_path, generated_path = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]

expected = {}
order = []
for line in os.environ["EXPECTED"].strip().splitlines():
    parts = line.split()
    order.append(parts[0])
for line in os.environ["EXPECTED"].strip().splitlines():
    parts = line.split()
    row = parts[0]
    for col, v in zip(order, parts[1:]):
        expected[(row, col)] = v == "T"

rows = [r for r in csv.DictReader(open(cells_path)) if r["delta"] == "+0e+00"]
if len(rows) != len(order) ** 2:
    sys.exit(f"FAIL probe produced {len(rows)} tangent cells, expected {len(order) ** 2}")

# The specialised set, parsed from fcl's own registration macros.
text = open(regs_path).read()
spec = set()
for a in re.findall(r"FCL_GJK_LIBCCD_SHAPE_INTERSECT\((\w+),", text):
    spec.add((a.lower(), a.lower()))
for a, b in re.findall(r"FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT\((\w+), *(\w+),", text):
    spec.add((a.lower(), b.lower()))
    spec.add((b.lower(), a.lower()))
if not spec:
    sys.exit("FAIL parsed no registrations out of gjk_solver_libccd-inl.h -- the macro names moved")

# Second registration mechanism: hand-written `ShapeIntersectLibccdImpl<S, Shape1<S>,
# Shape2<S>>` partial specializations (Halfspace/Plane today), invisible to the macro
# regexes above -- same parse as --emit's, kept in sync by hand for the same reason
# the macro parse above is. `struct_lines` is every `struct ShapeIntersectLibccdImpl`
# declaration found by the loosest possible anchor -- the class name alone, no
# assumption about which shapes are specialized -- so a future specialization this
# parse can't read still shows up as "found but unexplained" instead of silently
# missing `spec`.
struct_lines = open(specs_path).read().splitlines()
# The primary (unspecialised) template names the class with no explicit `<...>`
# argument list -- that C++ grammar distinction, not gjk_solver_libccd-inl.h:112's line
# number, is what excludes it here.
specialization_lines = [ln for ln in struct_lines if re.match(r"^\d+:struct ShapeIntersectLibccdImpl<", ln)]
SPEC_ARGS_RE = re.compile(r"^\d+:struct ShapeIntersectLibccdImpl<S,\s*(\w+)<S>,\s*(\w+)<S>>\s*$")
unexplained = []
for ln in specialization_lines:
    m = SPEC_ARGS_RE.match(ln)
    if not m:
        unexplained.append(ln)
        continue
    a, b = m.group(1).lower(), m.group(2).lower()
    spec.add((a, b))
    spec.add((b, a))
if unexplained:
    sys.exit(
        f"FAIL {len(unexplained)} hand-written ShapeIntersectLibccdImpl specialization(s) "
        "in gjk_solver_libccd-inl.h are not explained by the parsed specialised set:\n"
        + "\n".join(f"  {ln}" for ln in unexplained)
    )

drift, mechanism = [], []
for r in rows:
    key = (r["upper"], r["lower"])
    got = r["collision"] == "1"
    if key not in expected:
        sys.exit(f"FAIL probe reported a pair the pin does not know: {key}")
    if got != expected[key]:
        drift.append(f"{key[0]} x {key[1]}: pinned {expected[key]}, measured {got}")
    if got != (key in spec):
        mechanism.append(
            f"{key[0]} x {key[1]}: collision={got} but "
            f"{'is' if key in spec else 'is NOT'} registered as non-libccd"
        )

if drift:
    print(f"FAIL {len(drift)} of {len(rows)} tangency cell(s) moved:")
    for d in drift:
        print(f"  {d}")
if mechanism:
    print(f"FAIL {len(mechanism)} cell(s) break 'specialised iff collides at tangency':")
    for m in mechanism:
        print(f"  {m}")

# Check 3: the committed `Box`/`Sphere`/`Cylinder`/`Cone` table
# `accumulate_collision` actually reads, parsed back out of the tracked file
# and compared to the same `spec` set restricted to those four kinds --
# independent of checks 1/2 above, which compare the *probe's* own answer,
# not the file a `cargo build` reads.
KINDS4 = ["box", "sphere", "cylinder", "cone"]
gen_text = open(generated_path).read()
marker = "SPECIALISED: [[bool; 4]; 4] = ["
if marker not in gen_text:
    sys.exit(f"FAIL {generated_path} has no `{marker}` -- regenerate with --emit")
body = gen_text.split(marker, 1)[1]
gen_rows = re.findall(
    r"\[\s*(true|false)\s*,\s*(true|false)\s*,\s*(true|false)\s*,\s*(true|false)\s*\]", body
)
table_drift = []
if len(gen_rows) != 4:
    table_drift.append(
        f"{generated_path}: found {len(gen_rows)} row(s) under `{marker}`, expected 4"
    )
else:
    for a, row in zip(KINDS4, gen_rows):
        for b, cell in zip(KINDS4, row):
            got = cell == "true"
            want = (a, b) in spec
            if got != want:
                table_drift.append(f"{a} x {b}: file says {got}, image parse says {want}")
if table_drift:
    print(f"FAIL {len(table_drift)} committed-table cell(s) disagree with a fresh image parse:")
    for d in table_drift:
        print(f"  {d}")

if drift or mechanism or table_drift:
    sys.exit(1)

print(f"OK {len(rows)} of {len(rows)} tangency cells match the pin, and all {len(rows)} agree")
print(f"OK with the {len(spec)} non-libccd pairs parsed out of fcl's own header")
print(f"OK {generated_path}'s committed SPECIALISED table matches that same parse")
PY
