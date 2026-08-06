# fcl-cylinder-box-distance-probe

`probe.cpp` scores `fcl::distance` on a `cylinder`/`box` pair against the
*exact* answer, over 2,000 poses, four ways per pose: MoveIt's default GJK
stopping threshold in each operand order, then the threshold tightened, then
tightened under a different solver.

The sibling `tools/fcl-distance-tolerance-probe/` established that fcl's
answer on a box/cylinder pair moves when only `distance_tolerance` changes.
That is enough to call the reference imprecise and not enough to arbitrate a
disagreement, because both of its columns are fcl's. This probe removes that
limit by choosing a configuration whose correct separation is known with no
narrowphase at all: `tools/moveit-diff`'s floor box at `--floor-top-z -0.5`
has its top face on the plane `z = -0.5`, and while the cylinder floats above
that face with its silhouette strictly inside the 4x4 footprint — both
enforced by `face_is_nearest` — the distance is exactly
`c_z - z0 - h*|a_z| - r*sqrt(1 - a_z^2)`.

Pose 0 is not random. It is `prbt_flange`'s world pose at case 8148 of the
seed-1 10,000-state prbt sweep, composed from the *oracle's own* `fk` answer
rather than from this port's kinematics, so the row that pins the divergence
owes nothing to the port. The remaining poses are an xorshift64-drawn band
around it.

`tools/ci/verify-fcl-cylinder-box-distance.sh` runs it inside the digest-gated
oracle image and checks four things that fail in different directions — the
default-threshold answer misses the closed form by more than Phase 3's own
`1e-4`; tightening the threshold alone brings the sample back inside `1e-8`;
`GST_INDEP` at that threshold agrees with the same closed form to `5e-11`; and
swapping the two operands moves the default answer by more than `1e-4`, which
an exact distance cannot do. The last one is what makes the defect readable as
a defect rather than as noise. Plus one pin: pose 0's box-first column must
still reproduce, bit for bit, the value the oracle published for case 8148.

Run it the way every docker-touching script in this repo must be run:

```
sg docker -c ./tools/ci/verify-fcl-cylinder-box-distance.sh
```

It takes about 3 seconds, nearly all of it `g++`, and SKIPs loudly (never
silently passes) when docker or the stamped image is absent. `verify-all.sh`
picks it up by glob.
