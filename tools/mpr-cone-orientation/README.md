# mpr-cone-orientation

Tests whether `crates/cspace-collision/src/fcl_tangency_table.rs`'s
`SPECIALISED[cone][*] = false` (and, as a control, the same generic-MPR
`false` cells for cylinder) holds at every relative ORIENTATION of an
exact tangency, or only at the one axis-aligned pose
`tools/fcl-tangency-probe/probe.cpp` measured it at.

## Why this exists

`SPECIALISED`'s `false` cells mean "no closed-form fcl routine
registered; falls through to fcl's generic libccd MPR path" — a
qualitatively weaker claim than a `true` cell's closed-form,
orientation-independent proof (see that file's own module doc). The pin
in `tools/ci/verify-fcl-tangency-dispatch.sh`'s `EXPECTED` table is a
single measurement at `probe.cpp`'s one configuration (both shapes
identity rotation, stacked along z). This directory asks whether that
single measurement generalizes.

## Building

```sh
./build.sh
```

Same pin as `tools/mpr-vs-epa/build.sh`: libccd from
`/home/stevek/work/libccd` (or `LIBCCD_SRC`), checked out exactly at tag
`v2.1`, built with `CCD_DOUBLE`.

## Running

```sh
./build/cone_orientation_witness
```

No stdin, no fixture — every configuration is constructed in the program
itself (see `cone_orientation_witness.c`'s own header for the
construction and why it uses fcl's own ported support functions rather
than libccd's `testsuites/support.c`).

## Finding

The controls (fcl's literal probe.cpp configuration, identity rotation,
delta=0) match the pin: `separated` for both box×cone and box×cylinder.

Tilting the cone's axis by as little as 5 degrees off that one
axis-aligned pose — while keeping the apex pinned at the exact same
touching point, confirmed by direct numeric sampling of the base rim,
never by trusting the algebra alone — flips `ccdMPRIntersect` /
`ccdMPRPenetration` from `separated` to `INTERSECT` (depth exactly 0),
and it stays `INTERSECT` at every tested angle up to the geometric limit
of the single-point-tangency family (63.4 degrees, where the cone's own
half-angle plus the tilt reaches 90 degrees and the contact stops being
a single point). Only the one orientation `probe.cpp` happened to
construct agrees with the pinned `false`; every other orientation of the
identical exact-zero-gap configuration reads the opposite verdict.

The cylinder control shows the same instability is not cone-specific: at
exact tangency, the tilt family reads `INTERSECT` at theta = 0, 5, 10,
20, 30 degrees, flips to `separated` at 45 degrees, then back to
`INTERSECT` at 60 and 80 degrees — non-monotonic, not a single clean
flip. This is a property of fcl's generic libccd-MPR fallback path
(`GJKCollide` → `ccdMPRIntersect`/`ccdMPRPenetration` →
`discoverPortal`) at exact-zero-gap inputs in general, not a
cone-specific defect: `discoverPortal`'s three reject tests
(`/home/stevek/work/libccd`, `mpr.c:189,209,232`,
`ccdIsZero(dot) || dot < CCD_ZERO`) are fed by support-point-derived dot
products that depend on which boundary point each shape's support
function returns for the portal's chosen probe directions — a function
of orientation, not just the exact-zero gap the closed-form routines
test in their `s2 > 0`-style boundary checks.

An earlier revision of the cone construction used a box with 100×100
half-extent (meant to approximate "an infinite flat face") and found
every orientation, including theta=0, reading `INTERSECT` — contradicting
even the control case. `discoverPortal`'s outcome is not scale-free: it
depends on the actual center-to-center metric geometry (`findOrigin`'s
`v0`), not purely on directions, so a witness has to match the touching
shape's own scale, not dwarf it. The box below is a 0.5 half-extent cube,
matching `probe.cpp`'s own `Box<S>(1,1,1)` exactly. Kept as a
cautionary note in this file rather than silently fixed and forgotten:
see `cone_orientation_witness.c`'s own header comment for the same note
attached to the code.
