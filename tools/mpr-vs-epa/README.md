# mpr-vs-epa

Compares libccd's real, unmodified `ccdMPRPenetration` against this
port's own `parry3d_f64::query::contact` (EPA) on `cspace-collision`'s
`visibility_cone` case 104 (`bl_caster_l_wheel_link` vs the cone-mesh
triangle it touches) — the pair `crates/cspace-collision/src/parry.rs`'s
deviation-6(b) doc cites as evidence that libccd's MPR is deeper than this
backend's EPA by construction, not by a fixable defect in the port.

## Why this exists

`ccdMPRPenetration` is upstream's own comparison algorithm, not something
this port can call without depending on libccd — the only way to
corroborate a claim about it is to actually run it, from source, against
the same inputs this port's own reconstruction produces. See
`mpr_case104.c`'s own header comment for the full reasoning and why the
triangle/cylinder inputs are never re-derived in C.

## Building

```sh
./build.sh
```

libccd has no system package here (`pkg-config --exists ccd` fails).
`build.sh` builds it from source, pinned to tag `v2.1` with `CCD_DOUBLE`
(the same build round 16/17/21's own investigation measured against).
Set `LIBCCD_SRC` to point at a different checkout if
`/home/stevek/work/libccd`'s default is not available; `build.sh` refuses
to run against a checkout that is not exactly at tag `v2.1`.

## Running

```sh
cargo run --release --example case104_mpr_input -p cspace-collision \
    | ./build/mpr_case104
```

The Rust side reconstructs case 104 (forward kinematics +
`VisibilityConstraint::cone_mesh`'s formula + parry's own deepest-triangle
search — see that example's own module doc) and prints the winning
triangle plus cylinder geometry on stdout; this backend's own EPA depth
goes to stderr. `mpr_case104` reads the stdout numbers and prints libccd's
own MPR depth for the identical pair.

Expect `mpr_depth` near `7.47919999515277989e-2` (libccd) against
`~-2.087e-2` (this backend's own EPA, stderr) — the oracle's own captured
case-104 depth is `7.47914550966356367e-2`, within `~7.3ppm` of libccd's
number and nowhere near this backend's own.

## Generalizing past case 104: `visibility_cone_mpr_sweep`

`case104_mpr_input.rs` is one case. `crates/cspace-collision/examples/
visibility_cone_mpr_sweep.rs` (round 27) drives the same comparison over
every `visibility_cone` mismatch in a real oracle sweep, and found 9 of
945 cases where MPR is *shallower* than EPA — see that file's own module
doc and `parry.rs`'s deviation-6(b) doc for the full numbers. Its
`--dump-case <idx>` flag prints one specific case's exact fed geometry
(the same bytes the sweep would otherwise pipe to `mpr_case104`) to
stdout, so a specific historical case can be re-fed to `mpr_case104`
without re-running the whole sweep or the oracle:

```sh
cargo run --release --example visibility_cone_mpr_sweep -p cspace-collision -- \
    --urdf <abs>/fixtures/pr2.urdf --srdf <abs>/fixtures/pr2.srdf \
    --seed 4 --cases <idx+1> --dump-case <idx> \
    | ./build/mpr_case104
```

## Reproducing round 28's mechanism finding (not committed — see why below)

Round 28 traced *why* those 9 cases plateau at `length/2` by building
libccd with `-DMPR_DIAG` and tracing `mpr.c`'s `findPenetr` loop directly
— iteration count, the portal's own outward direction, and all four
candidate points, printed every iteration to stderr. This is not
committed here: `mpr_case104.c`'s whole design principle is driving the
*real, unmodified* `ccdMPRPenetration` through its public API, never
vendoring or patching libccd's own source (see this file's own "Why this
exists" above) — an in-tree copy of `mpr.c`, instrumented or not, is
exactly the kind of second copy that drifts silently from the pinned
`v2.1` source and nobody notices. The finding's own reproducibility
instead rests on two things that *are* in the tree: `parry.rs`'s
deviation-6(b) doc cites the exact upstream line numbers the mechanism
runs through (`mpr.c`'s `portalDir`/`portalReachTolerance`/`findPenetr`,
`testsuites/support.c`'s `cylSupport`), and `--dump-case` (above)
reproduces the exact plateau *output* the mechanism predicts without
needing the instrumented build at all. To re-derive the internal trace
directly:

```sh
# from a clean checkout at $LIBCCD_SRC, tag v2.1
cmake -S "$LIBCCD_SRC" -B /tmp/mpr_diag/libccd -DCCD_DOUBLE=ON \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=-DMPR_DIAG
cmake --build /tmp/mpr_diag/libccd --target ccd
# add, inside src/mpr.c's findPenetr loop, an `#ifdef MPR_DIAG` block that
# fprintf(stderr, ...)s: iterations, portalDir's own dir, v1/v2/v3, v4, and
# portalReachTolerance's dv1..dv4/dot1..dot3 — every value that function
# already computes, nothing re-derived. Rebuild, then link mpr_case104.c
# against the resulting libccd.so exactly as build.sh does, and pipe a
# --dump-case geometry dump into the resulting binary with stderr kept.
git -C "$LIBCCD_SRC" diff --stat  # must be empty when done -- revert before reusing
```

## Round 29: `--dump-contacts` and `--max-contacts-per-pair`

Round 28 left one case (623) explained mechanically but not yet measured
on the oracle side: its own winning triangle plateaus exactly like the
other 8, but the oracle's own `collision` op reported a normal deep value
for the same pair, at the `max_contacts_per_pair = 1` default that can
only ever return one candidate triangle per pair. Round 29 uses the
`max_contacts_per_pair` field the coordinator added to that op
(`47a271c`, oracle image `700e7be54cb0a61f`) to see every contact FCL
actually found, not just the first:

```sh
cargo run --release --example visibility_cone_mpr_sweep -p cspace-collision -- \
    --urdf <abs>/fixtures/pr2.urdf --srdf <abs>/fixtures/pr2.srdf \
    --seed 4 --cases 624 --dump-contacts 623 --max-contacts-per-pair 32 \
    --oracle <abs>/tools/moveit-oracle/run-oracle.sh
```

`--max-contacts-per-pair <N>` threads straight through to the `collision`
op's own request field (omitted, not sent, when absent — every other
invocation of this binary is byte-unaffected). `--dump-contacts <idx>`
prints every contact the oracle returned for the touched pair at that
case index and exits, instead of running the usual EPA/MPR comparison.
See `doc/claim-audit/cspace-collision.md`'s round 29 for the result.

## Round 30: plateau histograms

Round 29 found the sweep's own `oracle=`/`mpr=` readings plateau on the
touched cylinder's own `radius` or `length/2` in a majority of cases,
counted by hand from raw output. `visibility_cone_mpr_sweep.rs` now
classifies every oracle/EPA/MPR reading itself, against its own case's
cylinder dimensions at a `1e-6` relative tolerance, and prints the count —
every run of the sweep (no new flags needed) ends with:

```
oracle plateau histogram (n=1000): axial(length/2)=619 (61.9%)  radial(radius)=0 (0.0%)  other=381 (38.1%)
epa plateau histogram (n=1000): axial(length/2)=0 (0.0%)  radial(radius)=0 (0.0%)  other=1000 (100.0%)
mpr plateau histogram (n=945): axial(length/2)=9 (1.0%)  radial(radius)=153 (16.2%)  other=783 (82.9%)
```

plus `(n=...)` appended to the existing Pearson-correlation line. This is a
number to read off the run from here on, not one to recount from raw
`println!` output by hand. See `doc/claim-audit/cspace-collision.md`'s
round 30 for the full population recount and the mechanism traced for the
`radial(radius)` bucket.
