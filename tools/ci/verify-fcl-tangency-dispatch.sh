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
# Two things are checked, and they are independent:
#
#   1. the measured tangency answer per pair, against the table below;
#   2. that same answer against the specialised set *parsed out of fcl's own
#      header inside the image* -- so the mechanism claim is re-derived, not
#      restated. A future fcl that registers a new pair moves both sides and
#      the run still agrees; one that changes the boundary convention without
#      changing the registrations fails check 2 while check 1 tells you which
#      cell moved.
#
# Needs docker (via `sg`, per this repo's wrapper rule) and the digest-gated
# oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

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
  echo "SKIP docker not on PATH -- the 49-cell fcl dispatch measurement is not re-derived."
  echo "SKIP this is not a pass."
  exit 0
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"

have="$(docker run --rm --entrypoint cat "$IMAGE" /usr/local/share/oracle-src.sha256 2>/dev/null || true)"
if [[ "$have" != "$want" ]]; then
  echo "SKIP $IMAGE is missing or built from different oracle sources"
  echo "SKIP   image: ${have:-<missing or unstamped>}"
  echo "SKIP   tree:  $want"
  echo "SKIP this is not a pass; rebuild with tools/moveit-oracle/build.sh, and remember"
  echo "SKIP that an unwrapped docker call here reports failure as success -- use sg docker."
  exit 0
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
  dpkg-query -W libfcl-dev > fclver.txt 2>/dev/null || echo "libfcl-dev <unknown>" > fclver.txt
' >"$work/build.log" 2>&1 || { echo "FAIL probe build/run failed inside $IMAGE:"; sed 's/^/  | /' "$work/build.log"; exit 1; }
elapsed=$((SECONDS - start))

echo "== $(cat "$work/fclver.txt") in $IMAGE, ${elapsed}s"

EXPECTED="$EXPECTED" python3 - "$work/cells.csv" "$work/registrations.txt" <<'PY'
import csv, os, re, sys

cells_path, regs_path = sys.argv[1], sys.argv[2]

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
if drift or mechanism:
    sys.exit(1)

print(f"OK {len(rows)} of {len(rows)} tangency cells match the pin, and all {len(rows)} agree")
print(f"OK with the {len(spec)} non-libccd pairs parsed out of fcl's own header")
PY
