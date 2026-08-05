# fcl-tangency-probe

`probe.cpp` calls `fcl::collide` directly, on every ordered pair of the seven
bounded shape kinds fcl and this port both have, at three vertical offsets:
`+1e-9` of clear air, a gap of **exactly** zero, and `-1e-9` of overlap. Every
shape is sized so its extremity along `z` is at exactly `±0.5`, a value that is
exact in binary, so the middle offset is a true tie rather than a very small
gap.

It exists because `doc/upstream-bugs.md`'s
`shape-intersect-tangency-follows-libccd-dispatch` rests on what that middle
offset does: `fcl::collide` reports a contact of depth 0 for a pair that has a
non-libccd specialisation registered and no contact at all for a pair that does
not — 49 cells out of 49, including `box`/`box` `true` against `convex`/`convex`
`false` on the same eight vertices. That claim was first measured from a scratch
directory, which `PORTING-PLAN.md` §201's rule does not allow to stand:
scaffolding may be deleted, but the evidence for a claim the tree depends on may
not be. `tools/mpr-vs-epa/` is the same situation and the precedent for this
layout.

`tools/ci/verify-fcl-tangency-dispatch.sh` compiles and runs it inside the
digest-gated oracle image and checks two independent things — the measured
table against a pin, and that same table against the specialised set *parsed out
of fcl's own header inside the image*. The second is what makes it a mechanism
check rather than a restatement: a future fcl that registers a new pair moves
both sides and still agrees, while one that changes the boundary convention
without touching the registrations fails it.

Run it the way every docker-touching script in this repo must be run:

```
sg docker -c ./tools/ci/verify-fcl-tangency-dispatch.sh
```

It takes about 3 seconds, nearly all of it `g++`, and SKIPs loudly (never
silently passes) when docker or the stamped image is absent. `verify-all.sh`
picks it up by glob.
