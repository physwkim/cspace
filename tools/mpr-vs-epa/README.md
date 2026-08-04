# mpr-vs-epa

Compares libccd's real, unmodified `ccdMPRPenetration` against this
port's own `parry3d_f64::query::contact` (EPA) on `moveit-collision`'s
`visibility_cone` case 104 (`bl_caster_l_wheel_link` vs the cone-mesh
triangle it touches) — the pair `crates/moveit-collision/src/parry.rs`'s
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
cargo run --release --example case104_mpr_input -p moveit-collision \
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
