# fcl-distance-tolerance-probe

`probe.cpp` calls `fcl::distance` on `fixtures/prbt.urdf`'s own box/cylinder
pair over 2,000 separated poses, three times per pose: once the way MoveIt
asks for it, once with only the GJK stopping threshold tightened, and once with
a different solver at the same tight threshold.

It exists because `PORTING-PLAN.md` §5 Phase 3's `distance: f64` clause spends
its `1e-4` against a reference that is not accurate to `1e-4`.
`distanceCallback` builds `fcl::DistanceRequestd(cdata->req->enable_nearest_points)`
(`moveit_core/collision_detection_fcl/src/collision_common.cpp:603`), leaving
`distance_tolerance` at its `1e-6` default — a field fcl documents as "the
threshold used in GJK algorithm to stop distance iteration", which is a
progress threshold, not a bound on the error. Neither ordering of box/cylinder
has a non-libccd distance specialisation, so both go through GJK and the
threshold is visible in the answer.

The shape pair is not a convenient one: it is the pair the prbt sweep's own
worst separated-branch disagreement lands on (`prbt_link_4` against
`prbt_base_link`), so the measurement is about the number that clause actually
reports rather than about a constructed worst case.

`tools/ci/verify-fcl-distance-tolerance.sh` runs it inside the digest-gated
oracle image and checks two independent things — that the default-tolerance
answer moves at all when the threshold alone is tightened, and that the two
tightened solvers agree far more closely than that movement. The second is what
makes the first readable as the default's error rather than as two algorithms
disagreeing; without it a solver quietly wandering would look identical.

Run it the way every docker-touching script in this repo must be run:

```
sg docker -c ./tools/ci/verify-fcl-distance-tolerance.sh
```

It takes about 3 seconds, nearly all of it `g++`, and SKIPs loudly (never
silently passes) when docker or the stamped image is absent. `verify-all.sh`
picks it up by glob. `tools/fcl-tangency-probe/` is the sibling of this layout
and carries the same §201 argument for why it is in the tree at all.
