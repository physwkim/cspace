# fcl-mesh-orientation-probe

The fcl-side counterpart of
`crates/moveit-collision/examples/mesh_orientation_probe.rs`. That probe
measured `check_robot_collision` reporting `false` for 6,083 of 24,970
rotated-mesh-at-exact-tangency configurations and left one question open:
does fcl -- the library this port's dispatch decisions are supposed to track
-- have a single stable answer at those same tilted orientations, or is fcl
itself unstable there (as `evidence-retention-1e69c0a3-1`'s
`tools/mpr-cone-orientation` already measured for cone and cylinder under
generic MPR)? `PORTING-PLAN.md`'s successor `GOALS.md:46` requires matching
upstream on pairs where the two dispatch tables overlap, not matching a
uniform rule -- so "the port misses" is only a fixable defect where fcl has
one stable answer to converge to.

`probe.cpp` runs the identical 497 systematic orientations (7 axes: `x`,
`y`, `z`, the three face diagonals, the body diagonal; 5-degree resolution
from 5 to 355 degrees) against the same 5 other kinds and the same 2
argument-order roles (`mesh=upper/attached`, `mesh=lower/world`) the Rust
probe sweeps, using the identical construction algorithm -- the mesh's own
extremal rotated vertex translated exactly onto `TOUCH = (5, 0, 0)`, the
other shape's own `HALF`-extent touching feature translated onto the same
point from the opposite side -- against `fcl::BVHModel<fcl::OBBRSSd>`, the
same instantiation moveit2 actually uses
(`collision_detection_fcl/src/collision_common.cpp:949,955,960`). It is not
bit-identical to the Rust construction: Eigen's `AngleAxis` and nalgebra's
`UnitQuaternion::from_axis_angle` are different implementations of the same
standard formula, so the two rotate the same 8 vertices to floating-point
values that agree to double precision but not necessarily to the last bit --
exactly the same order of magnitude as the phenomenon under study. This is
named, not elided: it means a `false` this probe reports right at the
`1e-16` scale carries that same uncertainty, which is why `MISS_DEPTH_PROBES`
(mirroring the Rust probe's own array) also runs here, and why the
comparison below reads pose-level *stability* rather than any single row in
isolation.

## Run it

Run the way every docker-touching script in this repo must be run, `sg
docker -c ...`. From the repo root:

```
source tools/moveit-oracle/src-digest.sh
want="$(oracle_stamp tools/moveit-oracle)"
IMAGE="$(oracle_image_tag "$want")"
work="$(mktemp -d)"
cp tools/fcl-mesh-orientation-probe/probe.cpp "$work/probe.cpp"
sg docker -c "docker run --rm -v $work:$work -w $work --entrypoint bash $IMAGE -c '
  g++ -O2 -std=c++17 \$(pkg-config --cflags eigen3) -o probe probe.cpp -lfcl -lccd
  ./probe
'"
```

Each line is `CSV,<other>,<role>,axis=<name>,angle=<deg>deg,<true|false>,<depth|NA>`,
directly joinable on the first five fields against the Rust probe's own
`CSV,...` lines (`cargo run -p moveit-collision --release --example
mesh_orientation_probe 2>&1 | grep '^CSV,'`, filtered to the systematic rows,
`label.starts_with("axis=")`, 4,970 of them). Takes a few seconds, almost all
`g++`.

## Measured (2026-08-08, `libfcl-dev 0.7.0-3build2` in the pinned oracle image, `moveit-rs/oracle:ccc22ff0287a603f`)

4,970 rows joined 1:1 against the Rust probe's own 4,970 systematic rows (0
missing on either side). Raw row-level confusion matrix (port, fcl):

| port \ fcl | fcl=false | fcl=true |
|---|---|---|
| **port=false** | 272 | 502 |
| **port=true**  | 424 | 3,772 |

696 fcl=false (14.0%), 774 port=false (15.6%) among these 4,970 -- close but
not the same population, and the per-kind, pose-level table below is what
actually separates a real divergence from an artifact of picking one
argument order:

For each of the 497 `(other, axis, angle)` poses, both roles' fcl answers
were compared to each other (fcl's own order-stability) and to both roles'
port answers:

| kind | agree | fcl=T, port=F (both roles) | fcl=F, port=T (both roles) | port itself splits by role | fcl itself splits by role |
|---|---|---|---|---|---|
| box      | 481 | 3   | 8  | 0   | 5   |
| sphere   | 352 | 145 | 0  | 0   | 0   |
| cylinder | 460 | 0   | 11 | 13  | 13  |
| cone     | 23  | 1   | 18 | 47  | 408 |
| mesh     | 265 | 21  | 8  | 109 | 94  |

Reading each column: "agree" is both libraries giving the same answer at
both roles. "fcl=T, port=F (both roles)" is a real, stable, unrescued miss --
fcl says touching regardless of which side the mesh is attached to, the port
says not touching regardless either, this is "outcome 1" (a real, stable divergence) with a genuine
target. "fcl=F, port=T (both roles)" is the *other* direction -- the port
over-reports a collision fcl, stably, does not see; the same shape of risk
Task 1's `cylinder x box` uniformization would have introduced, found here
without anyone proposing a fix. "port itself splits by role" is fcl stable
but the *port's own* answer depends on which side the mesh is attached to --
a port-side argument-order sensitivity independent of anything fcl does,
un-investigated this round. "fcl itself splits by role" is
"outcome 3" (no stable fcl target): no single fcl answer exists at that pose at all.

By kind:

- **sphere**: clean "outcome 1" (a real, stable divergence). 145/497 (29.2%) poses are a real,
  unrescued miss, fcl says `true` at *every one* of them regardless of
  argument order, zero fcl-instability. Matches the closed-form
  `Sphere`-triangle specialisation
  (`gjk_solver_libccd-inl.h:403-419,480-498`) already known to pad its
  boundary inclusive (`sphere_triangle-inl.h:146,156`) -- orientation should
  not matter to a specialisation keyed on the mesh's nearest point to a
  sphere's centre, and measured, it does not.
- **box**, **cylinder**: mostly agree (481/497, 460/497). Small residual
  divergence both directions (box: 3 miss / 8 over-report; cylinder: 0 miss
  / 11 over-report), plus a handful of instability (box 5, cylinder 13 fcl,
  cylinder 13 port-side).
- **cone**: dominated by "outcome 3" (no stable fcl target). 408/497 (82.1%) poses have no single
  fcl answer -- generalises `evidence-retention-1e69c0a3-1`'s own
  cone-under-tilt instability finding from the poses it sampled to the large
  majority of this probe's own 497. Only 23 poses (4.6%) even reach a
  stable-agree comparison at all.
- **mesh x mesh**: the most mixed cell. 94/497 (18.9%) fcl-unstable, a real
  but smaller stable divergence in both directions (21 miss, 8 over-report),
  and the largest port-side role-split of any kind (109/497, 21.9%) -- the
  port's own mesh-vs-mesh answer depends on which side is attached
  independent of fcl, which this probe did not set out to explain and does
  not.

## What this does *not* settle

No fix is applied here. `sphere`'s divergence has an unambiguous, stable
target; `cone`'s does not, matching how
`exact_tangency_is_decided_per_shape_pair.rs` already leaves `mesh x cone`
unresolved rather than picking a value neither of fcl's own two answers
agrees with. `box`/`cylinder`'s residuals and `mesh x mesh`'s port-side
role-split are measured, not diagnosed to a root cause, in this round.
