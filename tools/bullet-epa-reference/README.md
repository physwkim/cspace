# bullet-epa-reference

Runs the real `btGjkEpaSolver2::Penetration` and `::Distance` from
bullet3 @ `7dee3436` (tag 3.24) on twenty shape pairs and prints one line
each. `crates/cspace-bullet/src/epa.rs` embeds that output verbatim as
`BULLET_REFERENCE` and asserts the port reproduces every field.

## Why this exists

EPA's answer is not the geometric answer, so a hand-derived expectation
is an assertion *against* Bullet rather than about it. The first row,
`box_box_deep_x`, is two unit boxes overlapping 0.5 m along x: the depth
is plainly 0.5 along x, and Bullet reports 0.288675129 along a corner
diagonal. `EncloseOrigin` builds a tetrahedron with the origin *on* one
of its edges; the silhouette walk then reaches a face already marked with
the current pass, `expand` returns false, and `Evaluate` breaks out with
`InvalidHull` leaving `outer` at whichever face was best when it gave up.
A port that answered 0.5 there would be wrong.

The rows are also the only evidence that the port's arithmetic agrees
bit for bit rather than approximately. `probe.cpp` prints with `%.9g`,
which round-trips a `float` exactly, and the Rust side compares
`f32::to_bits` -- so a single rounding difference anywhere in GJK, EPA,
the support functions or `btVector3`'s reciprocal-multiply division shows
up as a failing row rather than disappearing into a tolerance.

## Building

```sh
./build.sh
```

`build.sh` compiles `probe.cpp` against `third_party/bullet3` and runs
it. It refuses to run against a checkout that is not exactly at the pin
`7dee3436e747958e7088dfdcea0e4ae031ce619e`, because the fixtures are that
revision's output; set `BULLET_SRC` to point elsewhere if needed.

The compile flags are what make the two sides comparable at all: no
`-march`, so GCC has no FMA to contract into, and no
`BT_USE_DOUBLE_PRECISION`, so `btScalar` stays `float`. On non-Apple
Linux `btScalar.h:216-244` leaves `BT_USE_SSE`, `BT_USE_SIMD_VECTOR3` and
`BT_USE_SSE_IN_API` undefined, so `btVector3` is the scalar struct
`crates/cspace-bullet/src/linear_math.rs` ports.

## Regenerating the fixtures

Paste the whole of `build.sh`'s stdout over `BULLET_REFERENCE`'s body.
Paste it whole rather than editing rows: a row transcribed field by field
picks up the transcriber's idea of the field order, and the parser in
`epa.rs` checks only that each line has thirteen fields.
