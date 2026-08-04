// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Behaviorally derived from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_fcl/include/moveit/collision_detection_fcl/collision_common.hpp
//   moveit_core/collision_detection_fcl/src/collision_common.cpp
//   moveit_core/collision_detection_fcl/src/collision_env_fcl.cpp
//
// This is not a line-by-line port of any one upstream file: there is no
// `parry` backend upstream to port from, so this reproduces the FCL
// backend's *observable behavior* (the `collisionCallback`/`distanceCallback`
// algorithms, and which upstream request fields those two functions actually
// read) on top of `parry3d-f64` instead of FCL's own narrow phase.

//! A [`CollisionEnv`] backend for `moveit_state::RobotState`, built on
//! `parry3d-f64`.
//!
//! # Design: one globally-posed part per shape
//!
//! Upstream's `FCLObject` holds `std::vector<FCLCollisionObjectPtr>
//! collision_objects_` — one FCL `CollisionObject` **per shape**, each
//! carrying that shape's own global pose. `constructFCLObjectWorld` pushes
//! one per `Object::shapes_[i]` at `global_shape_poses_[i]`;
//! `constructFCLObjectRobot` pushes one per robot geometry at
//! `getCollisionBodyTransform(link, shape_index)`; and both
//! `checkRobotCollisionHelper` and `distanceRobotHelper` loop over that
//! vector, invoking the broadphase once per collision object. Nothing is
//! combined anywhere.
//!
//! [`PosedBody`] reproduces exactly that: [`PosedBody::parts`] is one
//! `(global pose, shape)` per shape, and a body-vs-body check is the cross
//! product of the two bodies' parts.
//!
//! Combining a body's shapes into a single `parry` shape instead would be
//! both a deviation and unsound: `parry` treats
//! [`parry3d_f64::shape::TriMesh`] as a composite shape, and
//! `Compound::new` panics (`"Nested composite shapes are not allowed"`) as
//! soon as one part is a mesh — which [`World::add_shapes_to_object`] makes
//! reachable from public API for any scene object carrying a mesh alongside
//! anything else.
//!
//! # Deviations from upstream
//!
//! 1. **`group_name` is inert.** Verified by reading `collision_env_fcl.cpp`
//!    in full: `checkSelfCollision`/`checkRobotCollision`/`distanceSelf`/
//!    `distanceRobot` never call `enableGroup`/read `active_components_only_`
//!    at all — that machinery is wired up only by the RobotModel-needing
//!    convenience overloads this crate's `env` module already declines to
//!    port (`distanceSelf(state)`, `distanceRobot(state, verbose)`, ...). So
//!    this backend does not filter by group either, matching upstream's real
//!    (if surprising) behavior rather than the narrower one a fresh
//!    implementation might guess at.
//! 2. **World objects are never padded or scaled.** Verified from
//!    `constructFCLObjectWorld` (calls the two-argument
//!    `createCollisionGeometry(shape, obj)` overload, no scale/padding) versus
//!    `constructFCLObjectRobot` (uses the padding/scale-taking overload via
//!    the cached `robot_geoms_`). [`LinkPaddingScale`] is consulted only when
//!    converting a [`moveit_model::LinkModel`]'s shapes, never a
//!    [`crate::World`] object's.
//! 3. **`CollisionRequest::pad_environment_collisions`/`pad_self_collisions`
//!    are not read.** Grepped all of `moveit_core/`: both fields are read
//!    only by `planning_scene.cpp` (out of this crate's scope), which
//!    switches between two whole `CollisionEnv` instances — one padded, one
//!    not — rather than either field ever reaching a `CollisionEnv` backend.
//!    Neither `collision_env_fcl.cpp` nor `collision_common.cpp` reference
//!    either field, so this backend always applies whatever
//!    [`LinkPaddingScale`] it was constructed with, matching the real FCL
//!    backend.
//! 4. **At most one [`Contact`] per *part* pair.**
//!    `parry3d_f64::query::contact` returns a single closest/deepest point
//!    per shape pair, where FCL's narrow phase can report several contact
//!    points for one object pair (e.g. mesh-mesh, or a box corner against a
//!    face). A body pair therefore yields at most `a.parts.len() *
//!    b.parts.len()` contacts here, against upstream's unbounded-per-pair
//!    narrow phase. [`CollisionRequest::max_contacts_per_pair`] is applied
//!    (see [`accumulate_collision`]) and does bind whenever either body
//!    carries several shapes.
//! 5. **Always a full contact query, never FCL's cheap boolean-only path.**
//!    `collisionCallback`'s `NEVER`/no-entry branch runs a cheap,
//!    contact-data-free `fcl::collide` once the storage budget
//!    (`max_contacts`) is exhausted, since only the `collision` flag is still
//!    needed at that point. This backend always calls
//!    `parry3d_f64::query::contact` (prediction `0.0`), which yields the
//!    collision flag *and* contact data in one call; the extra data is simply
//!    discarded once the budget is spent. Observably identical output, no
//!    optimization pass ported.
//! 6. **FCL's non-convex penetration depth is an approximation, not exact
//!    EPA, and this backend computes an independent approximation of the
//!    same ill-posed quantity — the two need not agree, and disagreement
//!    takes three distinct presentations below.** Exact penetration depth
//!    between two non-convex shapes has no closed form; upstream's
//!    `distanceCallback` approximates it by re-running `fcl::collide` (up to
//!    200 contacts) and taking the *maximum* penetration depth found across
//!    every contact discovered, once `enable_signed_distance` and the pair
//!    turns out to be touching or penetrating. This backend instead calls
//!    `parry3d_f64::query::contact` once per pair (see deviation 4: at most
//!    one contact exists here anyway), and reads `Contact::dist` directly as
//!    the signed distance, clamping it to `>= 0` when `enable_signed_distance`
//!    was not requested. `nearest_points` and `normal` are likewise read
//!    from that same call's `point1`/`point2`/`normal1` rather than a second
//!    FCL-specific query. "One call" is not "one triangle" when either shape
//!    is a mesh: `parry3d_f64`'s own `contact_composite_shape_shape` (the
//!    dispatch a `TriMesh` pair goes through) already visits every sub-shape
//!    whose AABB overlaps the other's and keeps the single deepest across
//!    all of them — this backend's one call is already the
//!    maximum-over-the-contact-set reduction upstream's 200-contact
//!    re-collide performs, for whichever contact set that single
//!    narrow-phase pass considers.
//!
//!    Two independent approximations of an ill-posed quantity can disagree
//!    in more than one way. A reader who hits a fourth presentation should
//!    be able to recognize it from the paragraph above, not fail to find it
//!    in the list below.
//!
//!    **(a) Pair-ranking disagreement**: several pairs interpenetrate
//!    simultaneously and the two backends pick a different one as globally
//!    deepest, because each pair's own independent EPA disagrees with the
//!    other's independently of every other pair's — not systematically
//!    one-sided. Confirmed, not assumed:
//!    `moveit-collision/tests/collision_parity.rs`'s
//!    `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact` takes the
//!    live panda sweep's single worst `robot_distance` disagreement and
//!    shows the oracle's own answer there exceeds the colliding link's mesh
//!    bounding radius by nearly an order of magnitude — geometrically
//!    impossible for that pair, on either backend, however exhaustively its
//!    contacts are searched. That case's gap is not this backend missing a
//!    deeper real contact; it is upstream FCL/libccd's own penetration-depth
//!    computation producing an implausible number under deep, arbitrarily-
//!    rotated interpenetration. Whether the same holds for every other
//!    interpenetrating disagreement this backend reports is not established
//!    either way — but at least one other case is the ordinary, expected
//!    shape of this presentation rather than panda's impossible-number
//!    failure mode: pr2's case 7552
//!    (`pr2_case_7552_depth_disagreement_ranks_a_different_pair`, same test
//!    file) has this backend and the oracle each ranking a *different* mesh
//!    pair as globally deepest among several simultaneously-interpenetrating
//!    candidates, `|d|` two orders of magnitude smaller than panda's outlier
//!    and well inside each pair's own bounding radius. The robot-vs-world
//!    side shows the same presentation at scale, and — cross-robot — at a
//!    far larger one too. Round 15 re-ran this crate's own `--collision` op
//!    directly against today's tree (`PORTING-PLAN.md` §60/§67's own
//!    command, which predates this module's own opening doc's pr2
//!    `<mesh>` collision geometry even being compared): a seed-20260804,
//!    3000-case pr2 `right_arm` sweep finds `robot_distance` pair-
//!    disagreeing with the oracle on 2644 of 3000 cases (a second,
//!    independent seed-999002 sweep: 2642/3000) — this presentation, not
//!    (c)'s same-pair-and-diverges population below, is what actually
//!    carries pr2's robot-vs-world side: of the *same-pair* population
//!    disjoint from this flip count, only 2 of 356 (seed 20260804) / 0 of
//!    358 (seed 999002) exceed `PORTING-PLAN.md` §5's `1e-4` policy
//!    tolerance, the rest floating-point-scale noise (median `~3e-13`) on
//!    an already-agreed positive clearance. A `panda_arm` cross-robot sweep
//!    (seed 20260804, 2000 cases) shows the far-larger-magnitude instance
//!    of this same presentation this doc already cited as one example
//!    (`panda_link0`/`floor`, oracle ≈ `-1.9m` vs this backend ≈ `-0.1m`)
//!    is in fact typical for that pair, not an outlier: it disagrees (any
//!    magnitude) on 1360 of 2000 cases and exceeds the policy tolerance on
//!    1261 of those (63%), median `|d| = 1.84m`, oracle consistently the
//!    deeper side.
//!
//!    **(b) A magnitude plateau, for one pair both backends already agree
//!    is deepest, across a range of states**: this backend's own answer for
//!    that pair holds constant while the true depth is changing, because a
//!    second, state-invariant candidate happens to be shallower there.
//!    Confirmed for a convex-primitive-vs-mesh pair (this deviation is not
//!    limited to mesh-vs-mesh):
//!    `pr2_torso_lift_bellow_pair_crossover_confirms_min_of_two_candidates`
//!    (same test file) — `base_bellow_link` (a `<box>`) against
//!    `torso_lift_link` (a `<mesh>`), this backend's own dominant
//!    self-collision constant across a 10,000-case pr2 sweep. This
//!    backend's answer for that pair is `min(candidate_x, candidate_z(t))`
//!    over `torso_lift_joint` (the pair's only degree of freedom):
//!    `candidate_x` is a `torso_lift_joint`-invariant `-x`-face-vs-mesh
//!    planar contact (the same local box face and mesh feature are found
//!    across the whole plateau; only the mesh's own FK-correct z-position
//!    moves, which that contact does not depend on), and `candidate_z(t)`
//!    is a genuinely `t`-dependent z-direction candidate, linear in `t`
//!    with slope `1` (a rigid z-translation of the mesh shifts a
//!    z-direction separating distance by exactly the translation). Below
//!    the crossover `candidate_x` is shallower and this backend reports the
//!    plateau; the oracle's own sweep over the same range instead decreases
//!    smoothly and monotonically at very nearly `1:1` with the joint
//!    travel — a real z-direction overlap, not EPA noise settling
//!    differently each state, so "the oracle's own answer fails to hold
//!    constant" is not itself the argument for this being deviation 6
//!    rather than a bug (a claim that the true depth *cannot* change over
//!    that span would be circular, assuming the very thing in question).
//!    Past the crossover `candidate_z(t)` shallows further and wins, and
//!    this backend's answer then matches the oracle's own to `~1e-9` — the
//!    falsifiable signature the test asserts, not a plateau it merely
//!    describes: agreement exactly where `min(...)` predicts it, not
//!    resemblance throughout. Confirmed the same way as (a)'s two cases
//!    (bounding-radius plausibility, not merely argued) — and, unlike
//!    those two, `collision_parity.rs`'s own `assert_full_parity_matches_oracle`
//!    now runs that same bounding-radius check on every colliding case in
//!    every fixture, not just these three hand-picked ones: the sign-only
//!    branch documented above now pairs with a real assertion that fails
//!    if a colliding pair's own reported depth ever exceeds twice its own
//!    bounding radius, catching a future regression toward
//!    panda-worst-case-style impossible numbers even though exact magnitude
//!    parity is not required.
//!
//!    What looked like three of this backend's own frozen self-distance
//!    constants across that 10,000-case sweep are actually two pair
//!    *families*, not three distinct geometric coincidences, and only one
//!    of them is this presentation: `base_link`/each of the eight
//!    `*_caster_*_wheel_link`s collapses to one value within `3.98e-14`
//!    across the seed-1 sweep's own sampled states. Round 13 found this is
//!    not the wheel's own roll-axis symmetry a previous version of this doc
//!    claimed — that could not explain the collapse surviving a swap to a
//!    *different* wheel, and a dense sweep of `*_caster_rotation_joint`
//!    (whose axis is vertical, nowhere near the wheel's own horizontal
//!    roll axis) shows the true picture is not even a global invariant:
//!    [`query::contact`] against a [`TriMesh`] visits every triangle
//!    overlapping the other shape and keeps the deepest
//!    (`contact_composite_shape_shape`, see
//!    `panda_worst_sweep_deviation_is_not_a_missed_deeper_contact`'s own doc
//!    for how that was read from the vendored source), and the winner is a
//!    `min` of *several* `base_link` triangles, not one. One of them is a
//!    near-planar face whose three vertices share one `z` in `base_link`'s
//!    own frame (`pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`);
//!    a rotation about `*_caster_rotation_joint`'s own (vertical) axis and
//!    a translation to a different corner both leave that face's `z` — and
//!    so the wheel's perpendicular clearance to it — unchanged, so it wins
//!    unchanged across roughly 80% of the joint's own range and at every
//!    corner sampled. The other ~20% (two *different*, genuinely
//!    `theta`-varying triangles, one on each side) is shallower there and
//!    wins instead
//!    (`pr2_self_wheel_same_pair_frozen_constant_is_a_plateau_not_a_global_invariant`).
//!    This is the exact same "one constant candidate, one that actually
//!    moves, `min` of the two" shape this doc's own (b), directly below,
//!    already describes for `base_bellow_link`/`torso_lift_link`'s
//!    `candidate_x`/`candidate_z(t)`, not a distinct symmetry — eight
//!    geometrically distinct pairs sharing (b)'s own plateau-then-ramp
//!    mechanism against the same planar feature, and not an instance of
//!    this deviation at all over the range where that feature wins;
//!    `base_bellow_link`/`torso_lift_link` is not a true constant either,
//!    only (b)'s own plateau, spanning `5.536e-3` across 177 sampled states
//!    before the ramp above takes over. So the apparent "three frozen
//!    constants" were one real instance of (b) plus one second instance of
//!    the *same* plateau-then-ramp mechanism as (b), not three separate
//!    phenomena — and the seed-1 sweep's own sampled states simply never
//!    landed in the wheel pair's ~20% ramp region. All of the above
//!    establishes only that *this backend's* `-0.046592m` is the correct
//!    answer to the question its own [`TriMesh`] narrow phase asks
//!    (`pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`
//!    confirms this by calling `parry3d_f64::query::contact` directly, not
//!    through this backend's own pipeline). It says nothing about whether
//!    the oracle's FCL/libccd search misses a deeper self-side contact for
//!    this pair family: unlike (a) and (b)'s cases, a self-side pair has no
//!    independent geometric reference the way `floor_env`'s box gives (c),
//!    below, so that question is still open and is not resolved by anything
//!    in this doc.
//!
//!    Round 15 re-measured the caster-wheel-vs-`base_link` split this
//!    question was originally raised against (`PORTING-PLAN.md`'s round-12
//!    62-case sweep, never committed as a fixture and so not revisitable by
//!    index) from a fresh, independently reproduced two-seed sample: of the
//!    21 caster-wheel self-pair same-pair cases across both pr2 sweeps
//!    above, 5 have this backend's depth exceeding the smaller of the two
//!    contacting links' own bounding radius (`link_bounding_radius`) by
//!    more than 2x — the same "geometrically implausible" bound
//!    `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible` already
//!    tests for the oracle side — versus 16 within that bound; the
//!    round-12 sweep's own 3-of-10/7-of-10 split is the same shape at a
//!    different sample size, not a coincidence this round's independent
//!    reproduction should be read as contradicting. Round 15 left the
//!    16-within-bound remainder as a blocker: confirming whether FCL's own
//!    EPA search *within a triangle pair it does examine* converges to a
//!    shallower-than-true answer needed FCL/libccd's own penetration-depth
//!    source, not present locally at the time.
//!
//!    Round 16 got that source and closed the question — refuted, not
//!    confirmed. Versions matter here, so they are recorded once: FCL git
//!    tag `0.7.0` (the oracle image's `libfcl-dev 0.7.0-3build2`'s Debian
//!    changelog shows only a sparc64 patch and no-change rebuilds on top of
//!    upstream 0.7.0, so the oracle runs plain upstream 0.7.0, not the
//!    17-commits-later tree a plain checkout lands on by default — that
//!    later tree rewrites 447 lines of
//!    `include/fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h`,
//!    so reading it instead would risk documenting an algorithm the oracle
//!    does not run) and libccd git tag `v2.1` (the oracle image's
//!    `libccd2 2.1-2`'s changelog is packaging-only on top of upstream
//!    v2.1, and a plain checkout lands there directly), both built with
//!    `CCD_DOUBLE` (the image's own `/usr/include/ccd/config.h` defines it,
//!    `CCD_SINGLE` undefined). Diffed the 0.7.0 tag against the
//!    17-commits-later tree first, to know whether the gap mattered at all:
//!    `include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h` (owns
//!    `ShapeTransformedTriangleIntersectLibccdImpl::run`, the mesh-triangle-
//!    vs-shape entry point) is byte-identical between the two, and within
//!    `gjk_libccd-inl.h` the specific functions that entry point reaches —
//!    `supportTriangle`, `supportCyl`, `GJKCollide` — are unchanged too; the
//!    447-line rewrite (`3c2b993`/`da430b1`) is confined to the *distance*-
//!    query code (`ccdVec3PointTriDist2`, `doSimplex3`/`doSimplex4`,
//!    `GJKDistance`/`GJKSignedDistance`), a different query type from the
//!    *collision* query (`checkCollision`/`getCostSources`, hence
//!    self-distance's own depth) this question is about. So the 17-commit
//!    gap does not affect this answer either way.
//!
//!    `gjk_solver_libccd-inl.h` (confirmed identical across both FCL trees)
//!    shows `ShapeTransformedTriangleIntersectLibccdImpl::run` builds a real
//!    3-vertex `ccd_triangle_t` via `triCreateGJKObject(P1, P2, P3, tf2)`
//!    and calls `detail::GJKCollide<S>`, which (`gjk_libccd-inl.h`, line
//!    2670 in the 17-commits-later tree, line 2575 in the 0.7.0 tag — line
//!    number differs, body confirmed byte-identical) is a thin wrapper:
//!    `CCD_INIT`, set `support1`/`support2`/`center1`/`center2`/
//!    `max_iterations`/`mpr_tolerance`, then call the *unmodified upstream*
//!    `ccdMPRPenetration` — libccd's own Minkowski Portal Refinement, from
//!    `mpr.c`, not an FCL reimplementation. `fcl/narrowphase/collision_request.h`
//!    confirms `GJKSolverType::GST_LIBCCD` is `CollisionRequest`'s default
//!    `gjk_solver_type_`, and grepping all of `moveit2` finds no override —
//!    so this *is* the code path `checkCollision` runs. `supportTriangle`
//!    (same file, line 2615/2520 respectively, body byte-identical) is
//!    exactly the farthest of the 3 fixed vertices along `dir`, no
//!    reimplementation ambiguity to guess at.
//!
//!    Built libccd v2.1 from source (`cmake -DCCD_DOUBLE=ON`) and drove
//!    `ccdMPRPenetration` directly, outside this crate (a C library this
//!    crate does not and should not depend on, so this is a described
//!    external repro, not a committed test) — against `base_link`'s own
//!    real winning triangle (vertex indices `[14, 12, 15]`, the same
//!    triangle `pr2_self_wheel_same_pair_frozen_constant_is_a_planar_base_link_face`
//!    names) and `bl_caster_l_wheel_link`'s own real cylinder
//!    (`radius=0.074792`, `length=0.034`), both dumped live from the model
//!    this round. Swept 560 poses (5 tilt axes × 7 angles × 4 planar
//!    offsets × 4 target penetration depths) and compared every one against
//!    `parry3d_f64::query::contact`'s own independent EPA answer for the
//!    identical triangle/cylinder pose — the same cross-implementation
//!    standard `native_deepest_triangle_vs_cylinder` already uses as ground
//!    truth in this file's own test suite, not a new methodology invented
//!    for this question.
//!
//!    **Result: refuted.** Of 560 poses, 409 had both algorithms agree the
//!    shapes touch; of those, 215 disagreed on depth by more than float
//!    noise (max 0.158m, median-over-all-409 8.5e-4m), and in every one of
//!    those 215, `ccdMPRPenetration`'s depth was *larger* than parry's,
//!    never smaller (`mpr>parry: 215, mpr<parry: 0`). At the exact
//!    tangency boundary (target depth 0) the two disagree on whether the
//!    shapes touch at all in 63/140 poses (62 parry-yes/mpr-no, 1 the
//!    reverse) — expected floating-point sign noise astride a knife-edge
//!    boundary, not a magnitude bias, since it does not recur once the true
//!    depth is bounded away from zero. **`ccdMPRPenetration` does not
//!    converge to a shallower-than-true depth for this shape pair; if
//!    anything the opposite.** So the 16-within-bound remainder's magnitude
//!    disagreement is not explained by narrow-phase precision loss inside a
//!    triangle pair FCL already examines. What was still open after round
//!    16 was whether FCL's broad-phase/BVH traversal ever hands the
//!    narrow-phase a *different, shallower* candidate triangle than this
//!    backend's own exhaustive per-triangle search finds — a
//!    candidate-*selection* question, not a numerical-precision one.
//!
//!    Round 17 closed it, and not in the direction the question assumed.
//!    `fcl::Contact` carries primitive indices `b1`/`b2` (the triangle id,
//!    for a mesh) confirmed present at FCL tag `0.7.0`
//!    (`include/fcl/narrowphase/contact.h:48-68`), but
//!    `collision_detection::Contact`
//!    (`moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp:73-102`
//!    at the pinned moveit2 commit) does not — `pos`/`normal`/`depth`/
//!    `body_name_{1,2}`/`body_type_{1,2}`/`percent_interpolation` only — and
//!    `collision_common.cpp` has zero references to `b1`/`b2` (`rg '\bb1\b|\bb2\b'
//!    moveit_core/collision_detection_fcl/src/collision_common.cpp`, 0
//!    hits), so which triangle FCL itself names is unreachable through any
//!    MoveIt-mediated oracle op. Closing it needed the same move as round
//!    16's libccd repro, aimed at FCL this time: a one-off C++ program,
//!    compiled inside the oracle container against the real
//!    `libfcl-dev 0.7.0-3build2` (`pkg-config --cflags --libs fcl` there
//!    resolves to `-I/usr/include/eigen3 -lfcl -lccd`, confirming the
//!    package the running image actually links, not this host's checkout),
//!    calling `fcl::collide` directly — bypassing `CollisionEnvFCL`
//!    entirely, so BVH traversal, candidate selection and the narrow-phase
//!    all run for real — and reading `CollisionResult::getContact(i).b1`
//!    back.
//!
//!    The request built there is the exact one `distanceCallback` itself
//!    builds for a penetrating pair with `enable_signed_distance` set
//!    (`collision_common.cpp:636-663` at the pinned commit — the fallback
//!    that runs for every one of this deviation's self-collision cases,
//!    read in full this round to confirm it's the right target before
//!    reproducing it): `enable_contact = true`, `num_max_contacts = 200`,
//!    `gjk_solver_type` left at `CollisionRequest`'s own default
//!    `GST_LIBCCD` (`include/fcl/narrowphase/collision_request.h:102`, the
//!    constructor's default argument, at FCL tag `0.7.0` — `distanceCallback`
//!    never overrides it, matching round 16's own finding). `base_link`'s
//!    geometry is FCL's real
//!    `BVHModel<fcl::OBBRSSd>`, moveit's own BV choice
//!    (`createCollisionGeometry<fcl::OBBRSSd, ...>`,
//!    `collision_common.cpp:900-922,949`; `fcl::OBBRSSd` at
//!    `include/fcl/math/bv/OBBRSS.h:107`), built via the same
//!    `beginModel`/`addSubModel`/`endModel` sequence
//!    (`include/fcl/geometry/bvh/BVH_model.h:97,106,112`) from `base_link`'s
//!    real mesh vertices/triangles, dumped live from this crate's own
//!    `RobotModel`.
//!
//!    The 16-within-bound population itself is a fresh, independent
//!    reproduction, not round 15's original 16 (never committed as a
//!    fixture, not revisitable by index): a standalone scratch client
//!    (not `tools/moveit-diff`, not committed — `tools/moveit-diff` never
//!    surfaces [`Op::RandomStates`]'s own per-case joint values, only
//!    aggregate pass/fail) spoke the oracle's wire protocol directly for
//!    one whole-model `random_states` sweep (seed `20260817`, `8000`
//!    cases) followed by one `collision` request per case, both ops
//!    already used elsewhere in this repo, landing on 467 self same-pair
//!    `base_link`/`*_caster_*_wheel_link` cases, of which 16 sit within the
//!    same `2x` smaller-link-bounding-radius plausibility bound
//!    [`pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`] already
//!    tests for — 16 by this sweep's own count, not forced to match round
//!    15's.
//!
//!    Every case's `base_link`/wheel poses came from this port's own FK
//!    (the same `RobotState`/`Posed` [`distance_self`] itself uses), fed
//!    identically to both the C++ repro and this backend's own exhaustive
//!    per-triangle search. That identical construction is the actual basis
//!    for the result below, not merely supporting color: whatever this
//!    port's FK gets wrong relative to the oracle's own FK is a constant
//!    inside this one comparison, since it lands on both the repro and the
//!    exhaustive search alike — it cannot manufacture a one-directional gap
//!    between FCL's own reported depth and parry's independent EPA depth
//!    for the *same* triangle at the *same* pose. "FCL over-reports vs.
//!    parry, 16/16, never the reverse" below holds regardless of how far
//!    this port's FK sits from the oracle's real one.
//!
//!    Two secondary checks corroborate without being load-bearing. First,
//!    that the repro is genuinely running the oracle's own code path, not
//!    a lookalike: 13 of 16 cases reproduce the oracle's actual
//!    `self_distance` magnitude to under `1e-6` (most under `1e-15`, pure
//!    float noise). Second, a magnitude-separation argument that only
//!    *narrows*, not closes, how much the remaining 3 could matter even if
//!    they were FK-caused: the worst of the 3 (`1.231e-3`) is 25x smaller
//!    than the smallest of the 16 narrow-phase biases measured below
//!    (`0.0312`) — a size/majority argument alone was already shown
//!    insufficient to close a residual once in this workspace
//!    (`PORTING-PLAN.md` §119.1/120.1: a 91.3%-dominant-cause argument for
//!    pr2's `visibility_cone` mismatches left 10/115 `touching >= 2` cases
//!    unclosed until a direct 285-case cross-tab settled which population
//!    they belonged to).
//!
//!    So the 3-case gap was measured directly this round rather than left
//!    on the size argument. The FK-divergence hypothesis is **refuted**:
//!    querying the oracle's own `frame_transform` op for `base_link` and
//!    the wheel link at the exact joint values of all 3 mismatched states
//!    (`br_caster_l_wheel_link`, `fr_caster_l_wheel_link` and
//!    `fr_caster_r_wheel_link` — one case each, not `fr_caster_l_wheel_link`
//!    twice as an earlier pass through this same data mis-stated) and
//!    comparing component-wise (translation as Euclidean distance, rotation
//!    as the largest single 3x3-matrix-entry difference, reported
//!    separately so a rotation error scaled by link length would be visible
//!    as translation rather than hidden in it) against this port's own
//!    `Posed::global_link_transform` for the same states gives translation
//!    error `0`-`7.9e-17m` and rotation error `0`-`3.3e-16` — both within a
//!    small integer multiple of `f64::EPSILON` (`2.22e-16`), i.e.
//!    floating-point noise, not a measurable gap. A control group of 4 of
//!    the 13 well-reproduced cases run through the identical comparison
//!    (`br_caster_l_wheel_link`, `fr_caster_l_wheel_link`,
//!    `br_caster_r_wheel_link`, `fl_caster_r_wheel_link`) shows the same
//!    noise level, not a smaller one — this port's FK does not measurably
//!    diverge from the oracle's anywhere sampled in this population,
//!    mismatched and matched cases alike. `7.9e-17m` sits roughly 12 orders
//!    of magnitude below even the smallest of the 3 mismatches
//!    (`2.417e-5`), so FK divergence is categorically not a candidate cause
//!    of them.
//!
//!    Round 19 measured the padding half of that next candidate rather than
//!    leaving it as a hypothesis. `CollisionEnv::CollisionEnv(model, padding
//!    = 0.0, scale = 1.0)` (`collision_env.hpp:61`) fills `link_padding_`/
//!    `link_scale_` with that same uniform `0.0`/`1.0` pair for every link
//!    `getLinkModelsWithCollisionGeometry()` returns
//!    (`collision_env.cpp:83-96`) — not per-link, one constant for the whole
//!    robot. The oracle's `collision` op constructs
//!    `collision_detection::CollisionEnvFCL env(model_, world);`
//!    (`tools/moveit-oracle/src/oracle.cpp:2097`), the 2-argument
//!    constructor, and never calls `setPadding`/`setLinkPadding` anywhere on
//!    that op's path — so every pr2 link's padding stays at the ctor
//!    default. That default genuinely reaches FCL's geometry construction,
//!    not a dead field: `CollisionEnvFCL`'s own `model, world, padding,
//!    scale` constructor (the one `CollisionEnvFCL env(model_, world)` at
//!    `oracle.cpp:2097` resolves to) calls
//!    `createCollisionGeometry(shape, getLinkScale(name), getLinkPadding(name),
//!    link, j)` per link per shape, the padding/scale-taking overload
//!    (`collision_detection_fcl/src/collision_env_fcl.cpp:135`, mirrored at
//!    `:96` for the world-less constructor and `:503` for
//!    `updatedPaddingOrScaling`), not the no-padding one — it is applied, at
//!    value `0.0`/`1.0`, which scales/pads nothing. `moveit_core/srdf`
//!    (checked via `rg -n padding`) has no padding concept at all, and the
//!    pr2 SRDF fixture has no such key either, so there is no config-file
//!    source of a nonzero default to miss.
//!
//!    This port's own construction is `LinkPaddingScale::default()`
//!    (`crates/moveit-collision/tests/collision_parity.rs:354`) — an empty
//!    map, whose [`LinkPaddingScale::link_padding`]/
//!    [`LinkPaddingScale::link_scale`] getters return `LinkAdjustment`'s
//!    `Default` (`env.rs`: `padding: 0.0, scale: 1.0`) for any untracked
//!    link, the identical numeric pair. Treatment group (the 3 mismatched
//!    links: `br_caster_l_wheel_link`, `fr_caster_l_wheel_link`,
//!    `fr_caster_r_wheel_link`) and control group (3 arbitrary other pr2
//!    links with collision geometry: `base_link`, `torso_lift_link`,
//!    `fl_caster_r_wheel_link`) report the same `0.0`/`1.0` pair on both
//!    sides for every link checked — expected, since neither code path ever
//!    branches on link name when deciding padding; the oracle's ctor default
//!    and this port's untracked-link default are the same two literals by
//!    construction, not by sampling luck. The padding/scale hypothesis is
//!    **refuted**: it cannot explain any of the 3 mismatches, because both
//!    backends apply exactly zero padding and unit scale to every pr2 link on
//!    this code path, full stop.
//!
//!    The mesh half of the original candidate is still untraced and is the
//!    actual next candidate: whether the C++ repro's hand-built `BVHModel`
//!    (constructed directly from this port's own `RobotModel` mesh data for
//!    the repro) loads/converts pr2's `base_link`/wheel-link collision STLs
//!    into the same vertices and triangle winding that
//!    `CollisionEnvFCL::getCollisionGeometry`'s own
//!    `createCollisionGeometry`/`FCLShapeCache` path builds inside the oracle
//!    before `distanceCallback` runs. Padding/scale being ruled out narrows
//!    what "different BVH" could mean for the 3 residual cases to mesh
//!    construction itself — not traced this round.
//!
//!    The 4 different-triangle cases and these 3 `self_distance` mismatches
//!    overlap in exactly 1 of the 16 (`br_caster_l_wheel_link`, case index
//!    4 of the within-bound population) — not a subset relationship either
//!    way, so the two anomalies are mostly independent populations and this
//!    paragraph's refutation does not reach back into the different-triangle
//!    interpretation two paragraphs below.
//!
//!    Round 20 traced the BVH-construction half of the mesh candidate above
//!    structurally rather than by sampling: whether FCL's and this backend's
//!    own tree-split heuristics can make either miss the true deepest
//!    triangle. FCL's `BVHModel` constructor installs
//!    `BVSplitter<BV>(detail::SPLIT_METHOD_MEAN)` unconditionally
//!    (`include/fcl/geometry/bvh/BVH_model-inl.h:68` at FCL tag `0.7.0` — the
//!    same tag round 16 established the oracle actually links; `git diff
//!    0.7.0..e5efcc4` (the 17-commits-later tree a plain checkout lands on)
//!    is empty for this file and for `BV_splitter-inl.h`, so the gap round
//!    16 already characterized does not touch this question either).
//!    `moveit_core` never overrides it: `rg -n
//!    'BVSplitter|setBVSplitMethod|SPLIT_METHOD'` against the pinned
//!    `e017c91e` checkout finds 0 hits. For `fcl::OBBRSSd`, `SPLIT_METHOD_MEAN`
//!    picks the OBB's own principal axis
//!    (`split_vector = bv.obb.axis.col(0)`,
//!    `include/fcl/geometry/bvh/detail/BV_splitter-inl.h:540-546`) and splits
//!    at the mean of every candidate triangle's centroid projected onto it
//!    (`computeSplitValue_mean`, `:556-587`). This backend's own `TriMesh`
//!    instead rebuilds an 8-bin surface-area-heuristic `Bvh`
//!    (`BvhBuildStrategy::Binned`, `parry3d-f64` 0.30.0
//!    `src/shape/trimesh.rs:1157`; the bin/cost/split logic at
//!    `src/partitioning/bvh/bvh_binned_build.rs:39-95`) — a materially
//!    different heuristic, exactly what D4.5 expects and not itself a
//!    defect.
//!
//!    What the split heuristic cannot do, on either side, is make a truly
//!    overlapping triangle unreachable. FCL fits each node's bound from its
//!    own actual primitive range before applying the split
//!    (`bv = bv_fitter->fit(cur_primitive_indices, num_primitives)` ahead of
//!    `bv_splitter->computeRule`/`apply` on the same primitives,
//!    `BVH_model-inl.h:875-925`), and this backend's `Bvh` computes every
//!    internal node's bound as the exact merge of its children's
//!    (`.merged(...)`, `bvh_binned_build.rs:159,178`) — both are
//!    conservative, exact-fit hierarchies regardless of which heuristic chose
//!    the partition, so an overlap query at either tree visits every leaf
//!    that could possibly touch the query shape, independent of split
//!    method. This backend's own mesh-vs-shape contact query confirms it
//!    does exactly that, with no cap: `CompositeShapeRef::contact_with_shape`
//!    iterates `self.0.bvh().intersect_aabb(&ls_aabb2)` over every
//!    AABB-overlapping leaf and keeps the global minimum `dist`
//!    (`src/query/contact/contact_composite_shape_shape.rs:24-39`) — unlike
//!    upstream's own capped re-collide (`num_max_contacts = 200`, cited
//!    above), but that cap only matters if a pair has more than 200 truly
//!    overlapping candidate triangles. It does not here, measured rather
//!    than assumed: `base_link`'s real collision mesh
//!    (`pr2.urdf`'s `<collision>` block for `base_link` names
//!    `base_v0/base_L.stl`) is a binary STL whose own 4-byte triangle-count
//!    header reads `96`, and the file is exactly `84 + 96*50 = 4884` bytes —
//!    the binary-STL record size confirming the header, not merely echoing
//!    it. 96 is below the 200-contact cap by construction, so the cap cannot
//!    truncate this pair's candidate set regardless of traversal order,
//!    whatever that order turns out to be. All 3 residual links carry the
//!    identical collision geometry round 16/17's cylinder repro already
//!    validated against (`<cylinder radius="0.074792" length="0.034">`,
//!    `pr2.urdf`, all three of `br_caster_l_wheel_link`/
//!    `fr_caster_l_wheel_link`/`fr_caster_r_wheel_link`) — the same pair
//!    family, not a different one needing its own geometric argument.
//!
//!    So BVH split method/traversal order is **structurally ruled out**, not
//!    merely unmeasured, as a cause of the 3 residual `self_distance`
//!    mismatches: neither tree can hide a genuinely deepest triangle from
//!    its own narrow phase for this mesh, and FCL's contact cap cannot bind
//!    on a 96-triangle mesh. This is a stronger claim than round 17's
//!    finding for the 16-within-bound population above, which showed no
//!    missed candidate empirically, for a sample; this one holds for every
//!    state of this pair family, by construction. It does not, by itself,
//!    determine whether these 3 cases' actual cause is the same
//!    narrow-phase magnitude bias round 16/17 characterized for that
//!    population (only 1 of the 3 — `br_caster_l_wheel_link` — was even part
//!    of that 16-case sample, and which side of its same-triangle/
//!    different-triangle split it fell on was not recorded) or the
//!    still-open mesh-construction-fidelity question above: vertex order and
//!    triangle indices. Ruling out the BVH question narrows, but does not
//!    close, deviation 6(b)'s remaining candidates to those two.
//!
//!    **Result, falsifiable and measured on all 16, not a sample: FCL names
//!    the *same* triangle this backend's own exhaustive search names in 12
//!    of 16 cases, a *different* one in the remaining 4 — and in every one
//!    of the 16, `fcl::Contactd::penetration_depth` for FCL's own chosen
//!    triangle exceeds `parry3d_f64::query::contact`'s independent EPA
//!    answer for that *identical* triangle/cylinder pair, by `0.031m` to
//!    `0.104m` (min/median/max `0.0312/0.0619/0.1042`), never the reverse
//!    (16/16, not 12/16 — the 4 different-triangle cases show the same
//!    one-directional gap too).** For the 12 same-triangle cases this is
//!    the whole story: FCL agrees with this backend on which triangle is
//!    deepest, and diverges only in its own narrow-phase's reported depth
//!    for it — pure narrow-phase-magnitude bias, the same sign round 16
//!    already established (`ccdMPRPenetration ≥ parry`, never `<`), now
//!    measured at real magnitude on real `base_link` mesh triangles instead
//!    of round 16's synthetic sweep. For the 4 different-triangle cases,
//!    parry's own independent depth for FCL's chosen triangle (`0.031`-
//!    `0.046m`) is *not* larger than this backend's own winner (`0.046592m`
//!    constant) — the alternate triangle is not genuinely deeper by any
//!    measure both algorithms agree on. It only wins FCL's own
//!    `max`-over-200-contacts selection because FCL's *own* inflation
//!    happens to be larger for that triangle than for the true winner, an
//!    uneven application of the same one-directional bias, not a
//!    broad-phase/BVH traversal handing the narrow-phase a shallower true
//!    candidate. **So candidate selection is not an independent cause of
//!    this deviation: it is a downstream symptom of the same narrow-phase
//!    magnitude bias round 16 already characterized, in the 4/16 cases
//!    large enough (and unevenly enough distributed across candidate
//!    triangles) to also flip the `max`-contact selection, not a defect in
//!    BVH traversal order or a genuinely missed deeper candidate.** This
//!    closes round 16's UNFIXED: nothing further to read in FCL's
//!    broad-phase for this question, and no oracle extension is needed —
//!    the whole 16-within-bound remainder is explained by narrow-phase
//!    magnitude bias alone.
//!
//!    **(c) A magnitude disagreement on a pair both backends already agree
//!    is deepest, at a single state — no ranking flip, no plateau, just two
//!    different depths for the one pair.** The same seed-20260804 pr2
//!    `right_arm` sweep that supplies (a)'s pair-flip count has 2 cases of
//!    this presentation instead, past `PORTING-PLAN.md` §5's `1e-4` policy
//!    tolerance (`tools/moveit-diff`'s `robot_same_pair_and_value_diverges`,
//!    structurally disjoint from the pair-flip tally by construction — a
//!    pair either matches or it does not; round 15 reproduces this exactly
//!    on today's tree, and an independent seed-999002 sweep of the same
//!    size finds 0 — this presentation is a small, seed-sensitive
//!    population on pr2, not a stable count):
//!    `l_gripper_l_finger_tip_link`/`floor` and
//!    `l_gripper_r_finger_tip_link`/`floor`, both with this backend
//!    *deeper* (oracle -0.011274/-0.009943 vs this backend
//!    -0.015686/-0.012375). Confirmed deeper-and-correct, not merely
//!    deeper: `pr2_world_object_same_pair_deeper_depth_is_a_real_vertex_not_a_spurious_direction`
//!    (same test file) independently measures the lowest global-frame z
//!    among the contacting fingertip mesh's own vertices — a computation
//!    that touches neither backend's collision pipeline — and finds it
//!    matches this backend's own deeper magnitude, not the oracle's
//!    shallower one, with the deepest vertex confirmed (not assumed)
//!    inside `floor_env`'s 4×4m footprint, where straight up is the only
//!    cheap escape. So here the oracle's own re-collide-and-take-max-depth
//!    search still misses this mesh's true deepest point; this backend's
//!    independent EPA does not.
//!
//!    **Measured, not merely described, across three robots (round 15).**
//!    Two independent pr2 `right_arm` sweeps (seeds 20260804/999002, 3000
//!    cases each), one `panda_arm` sweep (2000 cases) and one fanuc
//!    `manipulator` sweep (1500 cases), all via `tools/moveit-diff
//!    --collision --tol-distance 0.0` (reports every case with any nonzero
//!    disagreement, not only ones past a chosen tolerance), give this
//!    deviation's actual shape on same-pair magnitude divergence (ranking
//!    flips like (a) excluded, since those are a different presentation):
//!    - **Never a `collision: bool` flip.** `self_collision`/
//!      `robot_collision` matched the oracle in all 9500 cases sampled
//!      across all four sweeps — this deviation only ever changes *how
//!      deep*, never *whether*.
//!    - **Never a sign flip either.** Zero of 108 (pr2 self-side, both
//!      seeds, past the `1e-4` policy tolerance) / 179 (panda self-side) /
//!      513 (fanuc self-side) same-pair-diverging cases disagree on the
//!      sign of the reported distance — both backends always agree a
//!      penetrating pair is penetrating.
//!    - **Magnitude spans multiple orders and has no floor.** pr2 self-side
//!      (both seeds combined): max `1.374e-1`, median `~5.0e-2`; panda
//!      self-side: max `8.13e-2`, median `9.35e-3`; fanuc self-side: max
//!      `2.15e-1`, median `1.77e-2` — the module doc's opening claim ("the
//!      two numbers do not converge under any tolerance") with actual
//!      medians and maxima, not only the hand-picked example pairs cited
//!      above.
//!    - **Oracle depth does not generally match the contacting link's own
//!      radius.** The one documented instance of that (a different sweep,
//!      `PORTING-PLAN.md` §119.1's case 104, `bl_caster_l_wheel_link`
//!      within 7ppm of its own cylinder radius) does not generalize: of
//!      this crate's own 65 pr2 self-side same-pair cases (seed 20260804),
//!      0 have an oracle `|depth|` within even 40% of either contacting
//!      link's own bounding radius (`link_bounding_radius`, same helper
//!      `collision_parity.rs` already uses) — the wheel-family cases
//!      (10/65) are all `>12%` off (`5.9e5`–`1.4e6` ppm), the
//!      `base_bellow_link`/`torso_lift_link` family (55/65) all `>40%`
//!      off. Case 104's near-exact match is that state's own coincidence,
//!      not a signature of this deviation.
//!
//!    None of the three presentations comes from a different code path on
//!    the robot-vs-world side: [`distance_self`]/[`distance_robot`] both
//!    call the same [`accumulate_distance`] over the same per-part
//!    [`query::contact`] call and threshold logic, differing only in which
//!    [`PosedBody`] list supplies the pairs ([`self_pairs`] permutes one
//!    robot body list against itself; [`cross_pairs`] takes the robot list
//!    against [`world_bodies`]'s). The one structural asymmetry between the
//!    two sides — world objects are never padded or scaled (deviation 2,
//!    above) — does not apply to (c)'s two cases either, since `floor`
//!    carries no padding on either side of the comparison by construction.
//! 7. **No early exit on `distanceSelf`/`distanceRobot`.** Upstream's
//!    `distanceCallback` sets `cdata->done = true` (stopping the broadphase
//!    traversal) as soon as a collision is confirmed and
//!    `enable_signed_distance` was not requested — which pairs end up in
//!    `DistanceResult::distances` after that point depends on FCL's
//!    broadphase (AABB tree) traversal order, which this port does not
//!    reproduce (there is no broadphase here at all; every ACM-permitted
//!    pair is evaluated in link/object order every time). This backend
//!    therefore always evaluates every pair exhaustively: `distances` here
//!    can be a superset of what a given upstream run would report, but
//!    `minimum_distance` and `collision` — the two fields every real caller
//!    actually reads — are order-independent and match either way.
//! 8. **Cylinder/Cone axis convention.** `moveit_geometry::Cylinder`/`Cone`
//!    are z-aligned (a cone's tip at `+z`); `parry3d_f64::shape::Cylinder`/
//!    `Cone` are always y-aligned (a cone's apex at `+y`, verified by reading
//!    `parry3d-f64`'s own `cone.rs`). [`axis_fix`] is the fixed +90°
//!    rotation about x that maps parry's `+y` onto moveit's `+z`, correcting
//!    both the axis and (for [`moveit_geometry::Cone`]) the apex direction in
//!    one rotation.
//! 9. **A degenerate [`moveit_geometry::Plane`] (`a = b = c = 0`) converts to
//!    no shape at all.** [`moveit_geometry::Plane::new`] does not validate
//!    its coefficients (an infinite plane has no notion of a negative
//!    dimension to reject), so this case is reachable; a plane with no normal
//!    has no well-defined half-space to build, so [`convert_shape`] excludes
//!    it from collision geometry rather than construct a `HalfSpace` with a
//!    zero-length (and therefore un-normalizable) normal.
//! 10. **`check_robot_collision_continuous` returns [`Error`].** See
//!     [`crate::CollisionEnv::check_robot_collision_continuous`]'s own doc:
//!     upstream's FCL backend does not implement this case either, silently
//!     leaving `res` untouched; this backend has no swept/conservative-
//!     advancement query wired up, and returns an explicit error rather than
//!     an approximation that would misreport a real path collision as clear.
//! 11. **[`Shape::OcTree`] converts to a `Cuboid`-per-occupied-leaf
//!     [`Compound`], not a native octree.** `parry3d-f64` has no equivalent of
//!     FCL's `fcl::OcTreed` (`PORTING-PLAN.md` records no mature Rust octree-
//!     collision-shape crate was found); [`compound_from_octree`] builds one
//!     `Cuboid` per occupied leaf instead. Measured, not merely described:
//!     `crates/moveit-collision/tests/octree_leaf_count_scaling_parity.rs`
//!     runs the same scene and query against the real oracle (a genuine
//!     `fcl::OcTreed` inside `CollisionEnvFCL`) and this backend at occupied-
//!     leaf counts from 1 to 217 (one leaf at a fixed, known distance from
//!     the robot, plus 0 to 216 decoy leaves placed where they can never be
//!     the nearest one). `robot_distance` is bit-for-bit identical to the
//!     oracle's at every leaf count tested — this approximation does not
//!     measurably diverge from a native octree on `robot_distance`, at least
//!     up to the leaf counts exercised there.
//!
//! # Attached-body geometry
//!
//! `attached_body_body` builds one [`PosedBody`] per
//! [`AttachedBodyGeometry`], scaled/padded by its *attached link's*
//! [`LinkPaddingScale`] entry (matching upstream's `getAttachedBodyObjects`,
//! which scales/pads by `getLinkScale(ab->getAttachedLinkName())` — not a
//! padding of the attached body's own, which does not exist) and posed at
//! `link_pose * shape_pose` for each shape. [`robot_bodies`] appends these
//! after every robot link, so they participate in every one of self-
//! collision, robot-vs-world collision and both distance queries exactly
//! where upstream's own `constructFCLObjectRobot` puts them (module doc,
//! above).
//!
//! [`accumulate_collision`]/[`accumulate_distance`] additionally reproduce
//! `collisionCallback`/`distanceCallback`'s touch-links special-casing
//! (`collision_common.cpp`), evaluated *after* the ACM lookup but *before*
//! any geometry query, and able to force a pair "always allowed" regardless
//! of what the ACM said (even `Never`, even no entry at all):
//!
//! - a [`BodyType::RobotLink`]/[`BodyType::RobotAttached`] pair where the
//!   attached body's `touch_links` names the link — both callbacks;
//! - two [`BodyType::RobotAttached`] bodies on the *same* attached link, or
//!   where either's `touch_links` names the other's id — `collisionCallback`
//!   only; verified absent from `distanceCallback` by reading it in full
//!   (`collision_common.cpp:471-560`), so [`accumulate_distance`] does not
//!   apply this rule.
//!
//! "Same object" pairs (upstream's `cd1->sameObject(*cd2)`, the first check
//! in both callbacks) need no code here: [`self_pairs`]/[`cross_pairs`]
//! never produce `(x, x)`, and every [`PosedBody`] is already one body's
//! *entire* geometry combined into a single [`Compound`] (this module's own
//! design, above), so there is no second, distinct `PosedBody` sharing an
//! identity with the first for that check to ever need to catch.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::{Arc, Mutex, PoisonError, Weak};

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Shape, Vector3, compound_from_octree};
use moveit_model::LinkModel;
use moveit_state::Posed;

use parry3d_f64::bounding_volume::Aabb;
use parry3d_f64::math::{Pose, Vector as ParryVector};
use parry3d_f64::query::{self, Contact as ParryContact};
use parry3d_f64::shape::{
    Ball, Cone as ParryCone, Cuboid as ParryCuboid, Cylinder as ParryCylinder, HalfSpace,
    Shape as ParryShape, SharedShape, TriMesh, Triangle as ParryTriangle,
};

use crate::common::{
    AttachedBodyGeometry, BodyType, CollisionRequest, CollisionResult, Contact, ContactData,
    CostSource, DistanceRequest, DistanceRequestType, DistanceResult, DistanceResultsData,
};
use crate::env::{CollisionEnv, LinkPaddingScale};
use crate::matrix::{AllowedCollision, AllowedCollisionMatrix};
use crate::world::{Object, World};

/// A practical, effectively-unbounded search distance for `parry`'s own
/// prediction-margin arithmetic. Passing `f64::MAX` directly (upstream's own
/// default [`DistanceRequest::distance_threshold`]) risks overflowing to
/// `+inf` inside `parry`'s internal AABB-inflation math, since a finite AABB
/// corner offset by `f64::MAX` overflows under IEEE 754 addition. No real
/// robot geometry is within a million metres of itself, so clamping the
/// value actually sent to `parry` here changes no reachable behavior; the
/// logical threshold used for accumulation and the strict-`<` boundary check
/// is never clamped, only the query's own prediction argument.
const EFFECTIVELY_UNBOUNDED: f64 = 1.0e6;

/// Clamps a logical threshold down to a prediction margin `parry` will
/// accept. Upstream's threshold is a *signed* value under
/// [`DistanceRequest::enable_signed_distance`] and
/// [`DistanceRequestType::Global`]'s running `minimum_distance` accumulator
/// (see [`accumulate_distance`]): once one penetrating pair has been found,
/// every later pair's threshold argument is that penetration's negative
/// depth, not a search radius. `parry3d_f64::bounding_volume::Aabb::loosened`
/// panics on a negative margin ("The loosening margin must be
/// non-negative"), so the lower bound must be clamped here too, not only the
/// upper one -- a negative margin is otherwise reachable the first time a
/// deeply-penetrating pair updates the accumulator before every pair has been
/// visited. Clamping to `0.0` (rather than leaving the query out entirely)
/// still finds any pair at least as penetrating, matching
/// [`accumulate_collision`]'s own prediction-`0.0` convention for a
/// touching-or-penetrating-only query.
fn bounded_prediction(threshold: f64) -> f64 {
    threshold.clamp(0.0, EFFECTIVELY_UNBOUNDED)
}

/// The fixed rotation that maps `parry3d_f64`'s y-aligned `Cylinder`/`Cone`
/// convention onto `moveit_geometry`'s z-aligned one: +90° about x sends
/// local `(0, 1, 0)` to `(0, 0, 1)`, fixing the axis for both shapes and (for
/// `Cone`) the apex direction (parry: apex at `+y`; moveit: tip at `+z`) in
/// the same rotation. See the module doc, deviation 8.
fn axis_fix() -> Isometry3 {
    Isometry3::rotation(Vector3::x() * std::f64::consts::FRAC_PI_2)
}

fn to_pose(iso: Isometry3) -> Pose {
    iso.into()
}

fn from_parry_vector(v: ParryVector) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

/// Cache of [`Shape::OcTree`] → [`SharedShape`] conversions, keyed by the
/// wrapped tree's `Arc` pointer identity — the same identity
/// [`moveit_geometry::OcTree`]'s own [`PartialEq`] compares by, per its type
/// doc's deviation note.
///
/// [`convert_shape`] is called fresh for every shape of every body on every
/// [`CollisionEnv::check_robot_collision`]/[`CollisionEnv::distance_robot`]
/// call ([`robot_bodies`]/[`world_bodies`] rebuild every [`PosedBody`] from
/// scratch each time; see the module doc's design section). Without this
/// cache, an unmoved sensor octree already in the [`World`] would pay
/// [`compound_from_octree`]'s full leaf-to-`Cuboid` cost — measured at 130ms
/// for a 0.02m leaf size over a room-scale scene, PORTING-PLAN.md §4.8 — on
/// every single query against it, not once per octree update.
///
/// Each entry carries a [`Weak`] to the tree it was keyed on, purely to keep
/// that heap block alive. An address is only a valid identity while the
/// allocation behind it exists: with nothing pinning it, dropping the last
/// `Arc` to an octree frees the block, and the allocator is free to hand the
/// same address to the next, unrelated octree — which then reads the first
/// tree's cached conversion. That is not hypothetical here; on this
/// allocator a freed tree's address was reused by the very next allocation,
/// so a non-empty tree silently received an empty one's cached `None` and
/// dropped out of collision checking altogether. `moveit-distance-field`'s
/// `get_body_decomposition_cache_entry` hit the same defect and fixed it the
/// same way; see this file's `octree_cache_survives_shape_churn`. The `Weak`
/// is never upgraded — the `HashMap` key stays the bare address, and holding
/// a `Weak` rather than an `Arc` keeps the pinned block to the control word
/// instead of the whole tree.
///
/// An entry is pruned the moment nothing holds its tree anymore: every
/// [`Self::get_or_compute`] call first drops every entry whose [`Weak`] has
/// gone dead (`strong_count() == 0`), before doing anything else. That count
/// is the actual fact this cache needs — not a size cap or a timer standing
/// in for it — because [`World::remove_object`]/[`World::clear_objects`]
/// dropping the last [`Arc`] to an octree *is* what "this tree is gone" means
/// here; nothing else needs telling. A caller that rebuilds-and-replaces one
/// `World` object's octree every sensor scan therefore holds at most one
/// stale entry at a time (the just-replaced tree, until the very next
/// [`CollisionEnv`](crate::CollisionEnv) call prunes it), not one accumulated
/// per scan. The entry [`Self::get_or_compute`] is about to look up can never
/// be the one pruned: its caller is holding the `Arc` being queried, so that
/// entry's `strong_count()` is at least 1 at the moment of the prune.
/// Verified against a real `World`/[`ParryCollisionEnv`] rebuild-and-replace
/// loop, not merely argued from the prune's placement, in this file's
/// `octree_cache_stays_bounded_across_a_real_rebuild_and_replace_loop`.
///
/// Shared, not reset, across [`ParryCollisionEnv::clone`] — matching
/// [`Shape::OcTree`]'s own shallow-clone semantics (cloning a [`World`] that
/// carries an octree bumps the wrapped `Arc`'s refcount rather than
/// rebuilding the tree), a clone that still points at the same octree `Arc`
/// still benefits from a cache hit here.
#[derive(Clone, Default)]
struct OctreeCache(Arc<Mutex<HashMap<usize, OctreeCacheEntry>>>);

/// The pin plus the cached conversion. The [`Weak`] is dead weight by design
/// — see [`OctreeCache`] for why it has to exist anyway.
type OctreeCacheEntry = (Weak<moveit_octomap::OcTree>, Option<SharedShape>);

impl std::fmt::Debug for OctreeCache {
    /// `SharedShape` (`Arc<dyn parry3d_f64::shape::Shape>`) has no `Debug`
    /// impl — the trait only requires `RayCast + PointQuery + Any + Send +
    /// Sync` — so this reports the cache's size rather than its contents.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let len = self.0.lock().map_or(0, |cache| cache.len());
        f.debug_struct("OctreeCache")
            .field("entries", &len)
            .finish()
    }
}

impl OctreeCache {
    /// Returns the cached conversion for `tree`, or computes it with `build`
    /// and stores the result (`Some` or `None`) so a later call for the same
    /// tree never re-invokes `build`.
    ///
    /// Takes the `Arc` rather than a caller-computed address so the key and
    /// the [`Weak`] that pins it cannot be derived from two different trees:
    /// there is no signature here that lets a caller pass an address without
    /// also handing over the thing that keeps it valid.
    ///
    /// Prunes every dead entry first (see [`OctreeCache`]'s own doc for why
    /// `strong_count() == 0` is the right — and only needed — signal), so a
    /// tree no longer reachable from any live [`World`] does not hold its
    /// cache slot past the next call.
    fn get_or_compute(
        &self,
        tree: &Arc<moveit_octomap::OcTree>,
        build: impl FnOnce() -> Option<SharedShape>,
    ) -> Option<SharedShape> {
        let key = Arc::as_ptr(tree) as *const () as usize;
        let mut cache = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        cache.retain(|_, (weak, _)| weak.strong_count() > 0);
        if let Some((_, cached)) = cache.get(&key) {
            return cached.clone();
        }
        let value = build();
        cache.insert(key, (Arc::downgrade(tree), value.clone()));
        value
    }

    /// How many entries the map actually holds right now — a pure observer,
    /// deliberately. Pruning here as well would make the growth-bound test
    /// unable to fail: it would report the pruned count whether or not
    /// [`Self::get_or_compute`] ever pruned anything, which is a test that
    /// measures its own helper. Dead entries are counted, because "an entry
    /// nobody can use is still occupying the map" is exactly the fact that
    /// test exists to catch.
    #[cfg(test)]
    fn len(&self) -> usize {
        self.0.lock().unwrap_or_else(PoisonError::into_inner).len()
    }
}

/// Convert one [`Shape`] into a `parry` shape, plus the extra local
/// transform (identity, [`axis_fix`], or a plane offset) needed to align
/// `parry`'s axis convention with upstream's.
///
/// `None` for a degenerate [`Shape::Plane`] (module doc, deviation 9) and for
/// [`Shape::OcTree`] in two distinct cases that both happen to convert to
/// nothing: no tree attached at all (`Shape::OcTree(OcTree { octree: None
/// })`, upstream's default-constructed null `shared_ptr` state) has no
/// geometry to build, ever; a tree that *is* attached but has no occupied
/// leaves (a brand new tree, or one entirely marked free) has geometry that
/// happens to be empty for this particular tree — [`compound_from_octree`]
/// guards `Compound::new`'s empty-list panic the same way for that case.
/// Both convert to "no collision geometry", but they are not the same fact:
/// the first is a structural absence (no octree value exists here at all,
/// matching upstream's null `shared_ptr`), the second is a data-dependent
/// property of a real octree value (this particular immutable tree happens
/// to have no occupied leaves; a differently-built tree of the same shape
/// would not be empty).
///
/// A [`Shape::OcTree`] with an occupied leaf converts through
/// [`compound_from_octree`], memoized by [`OctreeCache`] (see its own doc) so
/// that repeated calls for the same tree — one per
/// [`CollisionEnv`](crate::CollisionEnv) query — do not rebuild the same
/// `Compound` from scratch every time. Oracle-verified end-to-end against a
/// real `CollisionEnvFCL` in
/// `crates/moveit-collision/tests/octree_world_collision_parity.rs`; this is
/// no longer a deviation from upstream, so it does not carry an entry in the
/// module doc's numbered list above.
fn convert_shape(shape: &Shape, octree_cache: &OctreeCache) -> Option<(SharedShape, Isometry3)> {
    match shape {
        Shape::Sphere(s) => Some((SharedShape::new(Ball::new(s.radius)), Isometry3::identity())),
        Shape::Cylinder(c) => Some((
            SharedShape::new(ParryCylinder::new(c.length * 0.5, c.radius)),
            axis_fix(),
        )),
        Shape::Cone(c) => Some((
            SharedShape::new(ParryCone::new(c.length * 0.5, c.radius)),
            axis_fix(),
        )),
        Shape::Cuboid(b) => Some((
            SharedShape::new(ParryCuboid::new(ParryVector::new(
                b.size[0] * 0.5,
                b.size[1] * 0.5,
                b.size[2] * 0.5,
            ))),
            Isometry3::identity(),
        )),
        Shape::Plane(p) => {
            let magnitude = (p.a * p.a + p.b * p.b + p.c * p.c).sqrt();
            if magnitude == 0.0 {
                return None;
            }
            let normal = ParryVector::new(p.a / magnitude, p.b / magnitude, p.c / magnitude);
            // p*n_hat, the plane's signed offset along its own unit normal:
            // p = -d/|n|, n_hat = n/|n|, so p*n_hat = -d*n/|n|^2.
            let offset = -p.d / (magnitude * magnitude);
            let translation = Vector3::new(p.a, p.b, p.c) * offset;
            Some((
                SharedShape::new(HalfSpace::new(normal)),
                Isometry3::translation(translation.x, translation.y, translation.z),
            ))
        }
        Shape::Mesh(m) => {
            let vertices = m
                .vertices
                .iter()
                .map(|v| ParryVector::new(v.x, v.y, v.z))
                .collect();
            TriMesh::new(vertices, m.triangles.clone())
                .ok()
                .map(|mesh| (SharedShape::new(mesh), Isometry3::identity()))
        }
        Shape::OcTree(o) => {
            let tree = o.octree.as_ref()?;
            octree_cache
                .get_or_compute(tree, || compound_from_octree(tree).map(SharedShape::new))
                .map(|shape| (shape, Isometry3::identity()))
        }
    }
}

/// [`Shape::scale_and_padd`] on a clone, for a robot link's own collision
/// shape (never a world object's — see the module doc, deviation 2).
///
/// # Panics
///
/// Never, in practice: every non-mesh shape variant's dimensions are already
/// validated non-negative at construction, so scaling by a
/// validated-positive [`LinkPaddingScale::link_scale`] and adding a
/// validated-non-negative [`LinkPaddingScale::link_padding`] can never make
/// them negative. [`Shape::Mesh`] can appear in [`LinkModel::shapes`] — the
/// URDF loader does resolve `<mesh>` collision geometry, through
/// `moveit_model::MeshSearchPaths` — but `moveit_geometry::stl::mesh_from_bytes`
/// calls `compute_vertex_normals` unconditionally at load time, so
/// [`Shape::scale_and_padd`]'s one documented mesh failure mode
/// (`vertex_normals: None`) is unreachable for any mesh that reached
/// `LinkModel::shapes` through that loader.
fn scaled_padded_shape(shape: &Shape, scale: f64, padding: f64) -> Shape {
    let mut shape = shape.clone();
    shape.scale_and_padd(scale, padding).expect(
        "every robot link collision shape is either non-negative by construction or a mesh \
         with vertex normals already computed at load time, so scale_and_padd cannot fail here",
    );
    shape
}

/// One named collision body — a robot link, an attached body, or a world
/// object — as the list of globally-posed shapes upstream's
/// `FCLObject::collision_objects_` holds for it. See the module doc.
struct PosedBody {
    name: String,
    body_type: BodyType,
    /// One `(global pose, shape)` per shape, upstream's
    /// `global_shape_poses_[i]` / `getCollisionBodyTransform(link, i)`.
    /// Never empty: [`pose_parts`] returns `None` rather than build a body
    /// with nothing to check.
    parts: Vec<(Pose, SharedShape)>,
    /// The link this body is rigidly attached to. `Some` only for
    /// [`BodyType::RobotAttached`] — needed by the touch-links same-link
    /// rule (module doc, "Attached-body geometry").
    attached_link: Option<String>,
    /// Links/attached-body ids this body is allowed to touch without that
    /// counting as a collision. Empty for [`BodyType::RobotLink`]/
    /// [`BodyType::WorldObject`], which upstream's touch-links rule never
    /// reads from that side of a pair.
    touch_links: BTreeSet<String>,
}

/// Compose each shape part's body-relative pose with the body's own `pose`,
/// yielding the global poses [`PosedBody::parts`] stores. `None` if `parts`
/// is empty, so a body with no convertible geometry is dropped rather than
/// carried as an empty one.
fn pose_parts(
    parts: Vec<(Pose, SharedShape)>,
    pose: Isometry3,
) -> Option<Vec<(Pose, SharedShape)>> {
    if parts.is_empty() {
        return None;
    }
    let body_pose = to_pose(pose);
    Some(
        parts
            .into_iter()
            .map(|(part_pose, shape)| (body_pose * part_pose, shape))
            .collect(),
    )
}

/// One robot link's [`PosedBody`], scaled and padded per `padding_scale`.
/// `None` if the link has no (convertible) collision geometry at all —
/// matching upstream's `getLinkModelsWithCollisionGeometry()` filter, which
/// this crate's [`LinkPaddingScale`] doc already reproduces for the
/// padding/scale bookkeeping itself.
fn link_body(
    link: &LinkModel,
    pose: Isometry3,
    padding_scale: &LinkPaddingScale,
    octree_cache: &OctreeCache,
) -> Option<PosedBody> {
    let scale = padding_scale.link_scale(link.name());
    let padding = padding_scale.link_padding(link.name());
    let parts: Vec<(Pose, SharedShape)> = link
        .shapes()
        .iter()
        .filter_map(|link_shape| {
            let shape = scaled_padded_shape(&link_shape.shape, scale, padding);
            let (parry_shape, extra) = convert_shape(&shape, octree_cache)?;
            Some((to_pose(link_shape.origin_transform * extra), parry_shape))
        })
        .collect();
    Some(PosedBody {
        name: link.name().to_owned(),
        body_type: BodyType::RobotLink,
        parts: pose_parts(parts, pose)?,
        attached_link: None,
        touch_links: BTreeSet::new(),
    })
}

/// One attached body's [`PosedBody`] (module doc, "Attached-body
/// geometry"), scaled and padded by its *attached link's* entry in
/// `padding_scale` — matching upstream's `getAttachedBodyObjects`, which
/// scales/pads by the attached link's own padding, not a padding of the
/// attached body. `None` if the body has no (convertible) shapes.
fn attached_body_body(
    geometry: &AttachedBodyGeometry<'_>,
    link_pose: Isometry3,
    padding_scale: &LinkPaddingScale,
    octree_cache: &OctreeCache,
) -> Option<PosedBody> {
    let scale = padding_scale.link_scale(geometry.link_name);
    let padding = padding_scale.link_padding(geometry.link_name);
    let parts: Vec<(Pose, SharedShape)> = geometry
        .shapes
        .iter()
        .zip(geometry.shape_poses)
        .filter_map(|(shape, shape_pose)| {
            let scaled = scaled_padded_shape(shape, scale, padding);
            let (parry_shape, extra) = convert_shape(&scaled, octree_cache)?;
            Some((to_pose(*shape_pose * extra), parry_shape))
        })
        .collect();
    Some(PosedBody {
        name: geometry.id.to_owned(),
        body_type: BodyType::RobotAttached,
        parts: pose_parts(parts, link_pose)?,
        attached_link: Some(geometry.link_name.to_owned()),
        touch_links: geometry.touch_links.clone(),
    })
}

/// One world object's [`PosedBody`], unscaled and unpadded (module doc,
/// deviation 2). `None` if the object has no (convertible) shapes.
fn object_body(id: &str, object: &Object, octree_cache: &OctreeCache) -> Option<PosedBody> {
    let parts: Vec<(Pose, SharedShape)> = object
        .shapes()
        .iter()
        .filter_map(|entry| {
            let (parry_shape, extra) = convert_shape(entry.shape(), octree_cache)?;
            Some((to_pose(entry.pose() * extra), parry_shape))
        })
        .collect();
    Some(PosedBody {
        name: id.to_owned(),
        body_type: BodyType::WorldObject,
        parts: pose_parts(parts, object.pose())?,
        attached_link: None,
        touch_links: BTreeSet::new(),
    })
}

fn robot_bodies(
    state: &Posed<'_, '_>,
    attached_bodies: &[AttachedBodyGeometry<'_>],
    padding_scale: &LinkPaddingScale,
    octree_cache: &OctreeCache,
) -> Vec<PosedBody> {
    let links = state.model().link_models().iter().filter_map(|link| {
        let pose = state.global_link_transform_at(link.link_index());
        link_body(link, pose, padding_scale, octree_cache)
    });
    // `global_link_transform` only fails for a link name absent from
    // `state`'s own model; a caller building `AttachedBodyGeometry` from a
    // `PlanningScene` over the same `RobotModel` an attach already validated
    // (`PlanningScene::attach`/`attach_new` both reject an unknown link name
    // up front) cannot hit this. Skipping rather than panicking here treats
    // a mismatched caller the same way an unconvertible shape is already
    // treated above: dropped, not fatal.
    let attached = attached_bodies.iter().filter_map(|geometry| {
        let link_pose = state.global_link_transform(geometry.link_name).ok()?;
        attached_body_body(geometry, link_pose, padding_scale, octree_cache)
    });
    links.chain(attached).collect()
}

fn world_bodies(world: &World, octree_cache: &OctreeCache) -> Vec<PosedBody> {
    world
        .iter()
        .filter_map(|(id, object)| object_body(id, object, octree_cache))
        .collect()
}

/// Every unordered pair among `bodies` (`i < j`), for self-collision.
fn self_pairs(bodies: &[PosedBody]) -> impl Iterator<Item = (&PosedBody, &PosedBody)> {
    (0..bodies.len())
        .flat_map(move |i| (i + 1..bodies.len()).map(move |j| (i, j)))
        .map(move |(i, j)| (&bodies[i], &bodies[j]))
}

/// The full cross product of `a` and `b`, for robot-vs-world checks.
fn cross_pairs<'a>(
    a: &'a [PosedBody],
    b: &'a [PosedBody],
) -> impl Iterator<Item = (&'a PosedBody, &'a PosedBody)> {
    a.iter().flat_map(move |x| b.iter().map(move |y| (x, y)))
}

/// Every `(pose, shape)` combination of two bodies' parts — the cross product
/// upstream gets for free by registering each body's collision objects
/// individually with the broadphase manager (`FCLObject::registerTo`) and
/// then calling `collide`/`distance` once per collision object.
fn part_pairs<'a>(
    a: &'a PosedBody,
    b: &'a PosedBody,
) -> impl Iterator<
    Item = (
        &'a Pose,
        &'a dyn parry3d_f64::shape::Shape,
        &'a Pose,
        &'a dyn parry3d_f64::shape::Shape,
    ),
> {
    a.parts.iter().flat_map(move |(a_pose, a_shape)| {
        b.parts
            .iter()
            .map(move |(b_pose, b_shape)| (a_pose, a_shape.as_ref(), b_pose, b_shape.as_ref()))
    })
}

/// `fcl2contact`, adapted from `parry3d_f64::query::Contact`'s fields:
/// `pos` is the midpoint of the two surface points (upstream's own `pos`
/// comes from FCL-internal contact geometry with no documented meaning
/// beyond "contact position"; a midpoint is a reasoned, defensible stand-in),
/// `normal` is `normal1` ("points from shape 1 toward shape 2", matching
/// upstream's own convention of a normal pointing from the first body to the
/// second), `depth` is `-dist` clamped to `>= 0`, and `nearest_points` is
/// `[point1, point2]` — populated here even though upstream's own
/// `fcl2contact` leaves `Contact::nearest_points` untouched for a narrow-
/// phase contact (only `DistanceResultsData::nearest_points` is ever set
/// upstream); `parry` gives us both points for free from the same query, so
/// this is a strict improvement over upstream's indeterminate field, not a
/// behavior this crate's tests can observe upstream ever relying on.
fn to_contact(
    pc: &ParryContact,
    name1: &str,
    type1: BodyType,
    name2: &str,
    type2: BodyType,
) -> Contact {
    let point1 = from_parry_vector(pc.point1);
    let point2 = from_parry_vector(pc.point2);
    Contact {
        pos: (point1 + point2) * 0.5,
        normal: from_parry_vector(pc.normal1),
        depth: (-pc.dist).max(0.0),
        body_name_1: name1.to_owned(),
        body_type_1: type1,
        body_name_2: name2.to_owned(),
        body_type_2: type2,
        percent_interpolation: 0.0,
        nearest_points: [point1, point2],
    }
}

fn cost_source_from_aabb(aabb: Aabb) -> CostSource {
    CostSource {
        aabb_min: [aabb.mins.x, aabb.mins.y, aabb.mins.z],
        aabb_max: [aabb.maxs.x, aabb.maxs.y, aabb.maxs.z],
        // `fcl2costsource`: `cs.cost = fcs.cost_density`. `cost_density` is a
        // plain `CollisionGeometry` member FCL default-initializes to `1`
        // (`collision_geometry-inl.h:56`) and nothing in `moveit_core` ever
        // writes (`rg cost_density moveit_core` finds only the read side,
        // `collision_tools.cpp:275`), so every real cost source this crate's
        // oracle can produce carries that same constant. This is not a
        // simplification of a computed estimate: FCL never computes a
        // density estimate either, so there is nothing else to reproduce.
        cost: 1.0,
    }
}

/// The world-space AABB of `tri` (given in `pose`'s local frame), built from
/// its three transformed vertices directly — matching
/// `mesh_collision_traversal_node-inl.h`'s `AABB<S>(p1, p2, p3)`, computed
/// there from vertices already baked into world coordinates (see
/// `initialize`'s upfront `tf1`/`tf2` bake-in in the same file). This is
/// tighter than [`Aabb::transform_by`] on the triangle's local AABB, which
/// would loosen to bound the *rotated box*, not the *rotated triangle* — the
/// two agree only when the rotation is axis-aligned.
fn triangle_world_aabb(pose: &Pose, tri: &ParryTriangle) -> Aabb {
    let a = pose * tri.a;
    let b = pose * tri.b;
    let c = pose * tri.c;
    Aabb::new(a.min(b).min(c), a.max(b).max(c))
}

/// Upstream's per-narrowphase-call cost-source production, independent of
/// [`AllowedCollision::Conditional`]'s own accept/reject decision: reading
/// `collision_common.cpp` in full shows `if (enable_cost) { ... }` running
/// unconditionally after *every* `fcl::collide` call this file makes — inside
/// the `dcf` branch (after the per-contact accept/reject loop, not gated by
/// its outcome), and in both halves of the no-`dcf` branch (whether or not
/// `want_contact_count` left room to store a [`Contact`]). So this is called
/// from [`accumulate_collision`] for every part pair that already produced a
/// real geometric [`ParryContact`], before that pair's
/// [`AllowedCollision::Conditional`] predicate (if any) is even consulted —
/// a pair the predicate goes on to silently accept still contributes cost
/// sources, matching upstream exactly.
///
/// Three shape-kind combinations, matching the three FCL traversal-node
/// files that call `addCostSource` (`detail/traversal/collision/`):
/// - **mesh vs mesh** (`mesh_collision_traversal_node-inl.h`): one
///   [`CostSource`] per pair of triangles confirmed to intersect —
///   [`mesh_mesh_cost_sources`].
/// - **mesh vs anything else** (`mesh_shape_collision_traversal_node-inl.h`):
///   one per `mesh`-side triangle confirmed to intersect the other side,
///   overlapped against that side's own *whole-shape* AABB, not a
///   triangle-vs-triangle box — [`mesh_shape_cost_sources`].
/// - **neither is a mesh** (`shape_collision_traversal_node-inl.h`): at most
///   one, over both shapes' own whole-shape AABBs from
///   [`ParryShape::compute_aabb`] — the same call already named as the
///   non-mesh half of this fill-in by `moveit-scene`'s own doc audit.
///
/// Every case does its candidate search BVH-pruned on the mesh side(s)
/// (`TriMesh::bvh`/[`parry3d_f64::partitioning::Bvh::intersect_aabb`]) and
/// then confirms with an exact geometric test
/// ([`query::intersection_test`]) before emitting anything — the same
/// broad/narrow two-stage structure FCL's own BVH traversal performs, not an
/// AABB-overlap approximation of it. A [`Compound`](parry3d_f64::shape::Compound)
/// built from an octree (deviation 11) is not a [`TriMesh`], so it always
/// takes the whole-shape-AABB path above — one cost source per colliding
/// octree pair, not per occupied leaf. FCL's own octree narrowphase
/// (`octree_solver-inl.h`) *does* cost per leaf, so this is a real, if minor,
/// further deviation on top of deviation 11's own Cuboid-per-leaf
/// [`Compound`] choice, not a new one this backend didn't already carry:
/// once occupied leaves are flattened into one compound shape, no
/// finer-than-the-compound granularity survives to cost separately.
fn cost_sources_for_part_pair(
    a_pose: &Pose,
    a_shape: &dyn ParryShape,
    b_pose: &Pose,
    b_shape: &dyn ParryShape,
) -> Vec<CostSource> {
    match (a_shape.as_trimesh(), b_shape.as_trimesh()) {
        (Some(mesh_a), Some(mesh_b)) => mesh_mesh_cost_sources(a_pose, mesh_a, b_pose, mesh_b),
        (Some(mesh_a), None) => mesh_shape_cost_sources(a_pose, mesh_a, b_pose, b_shape),
        (None, Some(mesh_b)) => mesh_shape_cost_sources(b_pose, mesh_b, a_pose, a_shape),
        (None, None) => {
            let aabb_a = a_shape.compute_aabb(a_pose);
            let aabb_b = b_shape.compute_aabb(b_pose);
            aabb_a
                .intersection(&aabb_b)
                .map(|overlap| vec![cost_source_from_aabb(overlap)])
                .unwrap_or_default()
        }
    }
}

/// One [`CostSource`] per pair of triangles (one from `mesh_a`, one from
/// `mesh_b`) confirmed to actually intersect. `mesh_a`'s triangles are
/// visited exhaustively — `TriMesh::bvh()` exposes no node-to-node walk that
/// would let the *outer* search skip whole subtrees the way
/// [`Bvh::intersect_aabb`](parry3d_f64::partitioning::Bvh::intersect_aabb)
/// does for the inner one — but each visited triangle's candidate partners
/// in `mesh_b` are BVH-pruned by its own local AABB (transformed into
/// `mesh_b`'s local frame, where its BVH lives) before the exact
/// [`query::intersection_test`] runs.
fn mesh_mesh_cost_sources(
    a_pose: &Pose,
    mesh_a: &TriMesh,
    b_pose: &Pose,
    mesh_b: &TriMesh,
) -> Vec<CostSource> {
    let a_to_b = b_pose.inv_mul(a_pose);
    let mut sources = Vec::new();
    for ia in 0..mesh_a.num_triangles() as u32 {
        let tri_a = mesh_a.triangle(ia);
        let local_aabb_a = Aabb::new(
            tri_a.a.min(tri_a.b).min(tri_a.c),
            tri_a.a.max(tri_a.b).max(tri_a.c),
        );
        let query_aabb_in_b = local_aabb_a.transform_by(&a_to_b);
        for ib in mesh_b.bvh().intersect_aabb(&query_aabb_in_b) {
            let tri_b = mesh_b.triangle(ib);
            if !query::intersection_test(a_pose, &tri_a, b_pose, &tri_b).unwrap_or(false) {
                continue;
            }
            let world_a = triangle_world_aabb(a_pose, &tri_a);
            let world_b = triangle_world_aabb(b_pose, &tri_b);
            if let Some(overlap) = world_a.intersection(&world_b) {
                sources.push(cost_source_from_aabb(overlap));
            }
        }
    }
    sources
}

/// One [`CostSource`] per `mesh`-side triangle confirmed to intersect
/// `other` (a non-mesh shape), the triangle's own world AABB overlapped
/// against `other`'s whole-shape world AABB — matching
/// `mesh_shape_collision_traversal_node-inl.h`'s `AABB<S>(p1, p2,
/// p3).overlap(shape_aabb, ...)`, `shape_aabb` there being `other`'s own
/// `computeBV` result, not a per-feature box of `other`.
fn mesh_shape_cost_sources(
    mesh_pose: &Pose,
    mesh: &TriMesh,
    other_pose: &Pose,
    other: &dyn ParryShape,
) -> Vec<CostSource> {
    let other_world_aabb = other.compute_aabb(other_pose);
    let query_aabb_in_mesh = other_world_aabb.transform_by(&mesh_pose.inverse());
    let mut sources = Vec::new();
    for i in mesh.bvh().intersect_aabb(&query_aabb_in_mesh) {
        let tri = mesh.triangle(i);
        if !query::intersection_test(mesh_pose, &tri, other_pose, other).unwrap_or(false) {
            continue;
        }
        let tri_world_aabb = triangle_world_aabb(mesh_pose, &tri);
        if let Some(overlap) = tri_world_aabb.intersection(&other_world_aabb) {
            sources.push(cost_source_from_aabb(overlap));
        }
    }
    sources
}

/// The `ROBOT_LINK`/`ROBOT_ATTACHED` half of `collisionCallback`'s/
/// `distanceCallback`'s touch-links rule (module doc, "Attached-body
/// geometry"): true if one side is a link the other side's attached body is
/// allowed to touch.
fn link_touches_attached(a: &PosedBody, b: &PosedBody) -> bool {
    match (a.body_type, b.body_type) {
        (BodyType::RobotLink, BodyType::RobotAttached) => b.touch_links.contains(&a.name),
        (BodyType::RobotAttached, BodyType::RobotLink) => a.touch_links.contains(&b.name),
        _ => false,
    }
}

/// The `ROBOT_ATTACHED`/`ROBOT_ATTACHED` half of `collisionCallback`'s
/// touch-links rule — `collisionCallback` only, see the module doc for why
/// [`accumulate_distance`] does not call this: two attached bodies on the
/// same link never collide; two on different links may still be allowed via
/// either one's `touch_links` naming the other's id.
fn attached_pair_allowed(a: &PosedBody, b: &PosedBody) -> bool {
    if a.body_type != BodyType::RobotAttached || b.body_type != BodyType::RobotAttached {
        return false;
    }
    if a.attached_link == b.attached_link {
        return true;
    }
    a.touch_links.contains(&b.name) || b.touch_links.contains(&a.name)
}

/// `collisionCallback`'s per-pair algorithm (see the module doc, deviations
/// 4 and 5, and "Attached-body geometry"), folded over every candidate pair:
///
/// - [`AllowedCollision::Always`], or the touch-links rule
///   ([`link_touches_attached`]/[`attached_pair_allowed`]): skip the pair, no
///   query at all. The touch-links rule is independent of `acm` and checked
///   after it — it can override any other ACM outcome, including `Never` or
///   no entry, matching upstream's own evaluation order.
/// - Real contact found (`parry3d_f64::query::contact`, prediction `0.0`,
///   so only touching/penetrating pairs yield `Some`) and
///   [`AllowedCollision::Conditional`]: the predicate decides — rejected
///   (`false`) is a collision, accepted (`true`) is silently not.
/// - Real contact found and [`AllowedCollision::Never`] or no entry:
///   unconditionally a collision.
/// - `Err`/`None` from the query (no contact, or an unsupported shape-pair
///   combination — unreachable for the `Ball`/`Cuboid`/`Cylinder`/`Cone`/
///   `HalfSpace`/`TriMesh` compounds this backend builds, all pairwise-
///   supported by `parry`'s default query dispatcher): not a collision.
///
/// The `collision` flag is set independent of the storage budget
/// (`request.max_contacts`, `request.max_contacts_per_pair`) for every pair,
/// matching upstream's own invariant.
fn accumulate_collision<'a>(
    pairs: impl Iterator<Item = (&'a PosedBody, &'a PosedBody)>,
    request: &CollisionRequest,
    acm: Option<&AllowedCollisionMatrix>,
) -> CollisionResult {
    let mut collision = false;
    let mut by_pair: BTreeMap<(String, String), Vec<Contact>> = BTreeMap::new();
    let mut stored_total = 0usize;
    // Most-costly-first, matching upstream's `std::set<CostSource,
    // CostSource::operator<>` (see `CostSource`'s own `Ord` doc); trimmed to
    // `request.max_cost_sources` after every insertion below, exactly as
    // `cdata->res_->cost_sources.insert(cs); while (... > max_cost_sources)
    // erase(--end());` does in `collision_common.cpp`.
    let mut cost_sources: BTreeSet<CostSource> = BTreeSet::new();
    for (a, b) in pairs {
        let allowed = acm.and_then(|m| m.allowed_collision(&a.name, &b.name));
        if matches!(allowed, Some(AllowedCollision::Always)) {
            continue;
        }
        if link_touches_attached(a, b) || attached_pair_allowed(a, b) {
            continue;
        }
        for (a_pose, a_shape, b_pose, b_shape) in part_pairs(a, b) {
            let Ok(Some(contact)) = query::contact(a_pose, a_shape, b_pose, b_shape, 0.0) else {
                continue;
            };
            // Independent of `is_collision`, below: `collision_common.cpp`
            // computes cost sources unconditionally once a real contact is
            // found, whether or not `AllowedCollision::Conditional`'s own
            // predicate goes on to accept it (see `cost_sources_for_part_pair`'s
            // own doc).
            if request.cost {
                for source in cost_sources_for_part_pair(a_pose, a_shape, b_pose, b_shape) {
                    cost_sources.insert(source);
                    while cost_sources.len() > request.max_cost_sources {
                        cost_sources.pop_last();
                    }
                }
            }
            let mut c = to_contact(&contact, &a.name, a.body_type, &b.name, b.body_type);
            let is_collision = match allowed {
                Some(AllowedCollision::Conditional(ref predicate)) => !predicate(&mut c),
                Some(AllowedCollision::Never) | None => true,
                Some(AllowedCollision::Always) => unreachable!("filtered out above"),
            };
            if !is_collision {
                continue;
            }
            collision = true;
            if request.contacts && stored_total < request.max_contacts {
                let bucket = by_pair.entry((a.name.clone(), b.name.clone())).or_default();
                if bucket.len() < request.max_contacts_per_pair {
                    bucket.push(c);
                    stored_total += 1;
                }
            }
        }
    }
    CollisionResult {
        collision,
        distance: None,
        contacts: request.contacts.then_some(ContactData { by_pair }),
        cost_sources: request.cost.then(|| cost_sources.into_iter().collect()),
    }
}

/// `distanceCallback`'s per-pair algorithm (see the module doc, deviations 6
/// and 7, and "Attached-body geometry"): [`AllowedCollision::Always`] or the
/// [`link_touches_attached`] touch-links rule skips the pair (upstream:
/// `always_allow_collision`, which the distance callback only ever sets from
/// `AllowedCollision::Always` or that one touch-links rule — `Never`/
/// `Conditional` are not special-cased for a distance query, and neither is
/// the attached-attached same-link rule [`accumulate_collision`] applies:
/// verified absent from `distanceCallback` by reading it in full); otherwise
/// the pair's distance is computed and folded into `minimum_distance` and
/// (for every [`DistanceRequestType`] but `Global`) `distances`, per
/// upstream's exact accumulation rule for that type.
fn accumulate_distance<'a>(
    pairs: impl Iterator<Item = (&'a PosedBody, &'a PosedBody)>,
    request: &DistanceRequest<'_>,
) -> DistanceResult {
    let mut result = DistanceResult::default();
    for (a, b) in pairs {
        if let Some(acm) = request.acm
            && matches!(
                acm.allowed_collision(&a.name, &b.name),
                Some(AllowedCollision::Always)
            )
        {
            continue;
        }
        if link_touches_attached(a, b) {
            continue;
        }
        let key = (a.name.clone(), b.name.clone());
        // Per *part* pair, not per body pair: each of upstream's collision
        // objects reaches `distanceCallback` as its own invocation, which
        // re-reads the running `minimum_distance`/`distances` state to pick
        // its threshold. Recomputing inside the loop is what reproduces that.
        for (a_pose, a_shape, b_pose, b_shape) in part_pairs(a, b) {
            let threshold = match request.request_type {
                DistanceRequestType::Global => result.minimum_distance.distance,
                DistanceRequestType::Limited => {
                    if result
                        .distances
                        .get(&key)
                        .is_some_and(|existing| existing.len() >= request.max_contacts_per_body)
                    {
                        continue;
                    }
                    request.distance_threshold
                }
                DistanceRequestType::Single => result
                    .distances
                    .get(&key)
                    .map_or(request.distance_threshold, |existing| existing[0].distance),
                DistanceRequestType::All => request.distance_threshold,
            };
            let Ok(Some(contact)) = query::contact(
                a_pose,
                a_shape,
                b_pose,
                b_shape,
                bounded_prediction(threshold),
            ) else {
                continue;
            };
            if contact.dist >= threshold {
                continue;
            }
            let distance_value = if request.enable_signed_distance {
                contact.dist
            } else {
                contact.dist.max(0.0)
            };
            let mut data = DistanceResultsData {
                distance: distance_value,
                nearest_points: [Vector3::zeros(); 2],
                link_names: [a.name.clone(), b.name.clone()],
                body_types: [a.body_type, b.body_type],
                normal: Vector3::zeros(),
            };
            if request.enable_nearest_points {
                if distance_value <= 0.0 {
                    let p = from_parry_vector(contact.point1);
                    data.nearest_points = [p, p];
                } else {
                    let p1 = from_parry_vector(contact.point1);
                    let p2 = from_parry_vector(contact.point2);
                    data.normal = (p2 - p1).normalize();
                    data.nearest_points = [p1, p2];
                }
            }
            if data.distance < result.minimum_distance.distance {
                result.minimum_distance = data.clone();
            }
            if data.distance <= 0.0 {
                result.collision = true;
            }
            match request.request_type {
                DistanceRequestType::Global => {}
                DistanceRequestType::All | DistanceRequestType::Limited => {
                    result.distances.entry(key.clone()).or_default().push(data);
                }
                DistanceRequestType::Single => {
                    let bucket = result.distances.entry(key.clone()).or_default();
                    if bucket.is_empty() {
                        bucket.push(data);
                    } else if data.distance < bucket[0].distance {
                        bucket[0] = data;
                    }
                }
            }
        }
    }
    result
}

/// A [`CollisionEnv`] backend for `moveit_state::RobotState`
/// (`Posed<'s, 'm>`), over `parry3d-f64`. See the module doc for scope and
/// deviations from upstream's FCL backend.
#[derive(Debug, Clone, Default)]
pub struct ParryCollisionEnv {
    world: World,
    padding_scale: LinkPaddingScale,
    /// See [`OctreeCache`]'s own doc: avoids re-running
    /// [`compound_from_octree`] for the same octree on every call.
    octree_cache: OctreeCache,
}

impl ParryCollisionEnv {
    /// Build a backend over `world`, applying `padding_scale` to robot links
    /// only (module doc, deviation 2).
    pub fn new(world: World, padding_scale: LinkPaddingScale) -> Self {
        Self {
            world,
            padding_scale,
            octree_cache: OctreeCache::default(),
        }
    }

    /// The collision world this backend checks the robot against.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Mutable access to the collision world.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// The per-link padding/scale this backend applies to robot geometry.
    pub fn padding_scale(&self) -> &LinkPaddingScale {
        &self.padding_scale
    }

    /// Mutable access to the per-link padding/scale.
    pub fn padding_scale_mut(&mut self) -> &mut LinkPaddingScale {
        &mut self.padding_scale
    }
}

impl<'s, 'm> CollisionEnv<Posed<'s, 'm>> for ParryCollisionEnv {
    fn check_self_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        let bodies = robot_bodies(
            state,
            attached_bodies,
            &self.padding_scale,
            &self.octree_cache,
        );
        accumulate_collision(self_pairs(&bodies), request, acm)
    }

    fn check_robot_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        let robot = robot_bodies(
            state,
            attached_bodies,
            &self.padding_scale,
            &self.octree_cache,
        );
        let world = world_bodies(&self.world, &self.octree_cache);
        accumulate_collision(cross_pairs(&robot, &world), request, acm)
    }

    fn check_robot_collision_continuous(
        &self,
        _request: &CollisionRequest,
        _state1: &Posed<'s, 'm>,
        _state2: &Posed<'s, 'm>,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
        _acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<CollisionResult> {
        Err(Error::other(
            "continuous robot-collision checking is not implemented by ParryCollisionEnv: no \
             swept/conservative-advancement query is wired up, and approximating it (e.g. \
             sampling the path, or only checking the end state) would silently misreport a real \
             path collision as clear",
        ))
    }

    fn distance_self(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        let bodies = robot_bodies(
            state,
            attached_bodies,
            &self.padding_scale,
            &self.octree_cache,
        );
        accumulate_distance(self_pairs(&bodies), request)
    }

    fn distance_robot(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        let robot = robot_bodies(
            state,
            attached_bodies,
            &self.padding_scale,
            &self.octree_cache,
        );
        let world = world_bodies(&self.world, &self.octree_cache);
        accumulate_distance(cross_pairs(&robot, &world), request)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_geometry::{Cuboid, OcTree, Plane, Shape, Sphere};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;

    // Geometry-level tests: `convert_shape`, `axis_fix`, `to_contact`.

    #[test]
    fn convert_shape_sphere_is_a_ball_at_the_origin() {
        let (_shape, extra) = convert_shape(
            &Shape::Sphere(Sphere::new(2.0).unwrap()),
            &OctreeCache::default(),
        )
        .unwrap();
        assert_eq!(extra, Isometry3::identity());
    }

    #[test]
    fn convert_shape_degenerate_plane_is_excluded() {
        let plane = Shape::Plane(Plane::new(0.0, 0.0, 0.0, 1.0));
        assert!(convert_shape(&plane, &OctreeCache::default()).is_none());
    }

    #[test]
    fn convert_shape_plane_offset_matches_hesse_normal_form() {
        // x = 3 (a=1, b=0, c=0, d=-3): signed offset from the origin along
        // the unit normal (1, 0, 0) is 3.
        let plane = Shape::Plane(Plane::new(1.0, 0.0, 0.0, -3.0));
        let (_shape, extra) = convert_shape(&plane, &OctreeCache::default()).unwrap();
        // The normal (1, 0, 0) is already axis-aligned and unit length, so the
        // offset is `-d` with no division or trig involved -- exact.
        assert_eq!(extra.translation.vector.x, 3.0);
        assert_eq!(extra.translation.vector.y, 0.0);
        assert_eq!(extra.translation.vector.z, 0.0);
    }

    // --- Shape::OcTree: the two distinct "no tree" cases (convert_shape's
    // own doc, above) must both convert to None, but for different reasons. ---

    #[test]
    fn convert_shape_octree_with_no_tree_attached_is_excluded() {
        // OcTree::new(): upstream's default-constructed null shared_ptr. No
        // octree value exists at all, structurally, regardless of leaves.
        assert!(convert_shape(&Shape::OcTree(OcTree::new()), &OctreeCache::default()).is_none());
    }

    #[test]
    fn convert_shape_octree_with_a_tree_but_no_occupied_leaves_is_also_excluded() {
        // A tree *is* attached here -- this is the data-dependent "empty"
        // case, not the structural "absent" case above -- but it still has
        // no occupied leaves, so it still converts to nothing.
        let tree = Arc::new(moveit_octomap::OcTree::new(0.1));
        let shape = Shape::OcTree(OcTree::from_tree(tree));
        assert!(convert_shape(&shape, &OctreeCache::default()).is_none());
    }

    /// Regression test for a defect this port introduced: [`OctreeCache`]
    /// keyed on a bare `Arc::as_ptr(tree) as usize` with nothing pinning that
    /// address. Dropping the last `Arc` to an octree freed the block, and the
    /// allocator handed the same address straight back to the next octree --
    /// on this allocator, on the very first attempt -- so a tree with an
    /// occupied leaf received an empty tree's cached `None` and vanished from
    /// collision checking entirely. That is a silent false negative: no
    /// error, no missing shape, just an obstacle that stops being an
    /// obstacle.
    ///
    /// The churn below is deliberately the shape that triggered it -- build,
    /// convert, drop, repeat -- alternating empty and occupied trees so a
    /// stale hit lands on a differing answer rather than a matching one.
    /// Every result is checked against what that tree must convert to
    /// independently, so the test fails on a mixed-up entry whether or not
    /// this particular run happens to reuse an address.
    #[test]
    fn octree_cache_survives_shape_churn() {
        let cache = OctreeCache::default();

        for i in 0..200 {
            let occupied = i % 2 == 0;
            let mut t = moveit_octomap::OcTree::new(0.1);
            if occupied {
                t.update_node(nalgebra::Point3::new(0.05, 0.05, 0.05), true, false);
            }
            let shape = Shape::OcTree(OcTree::from_tree(Arc::new(t)));
            let got = convert_shape(&shape, &cache);

            assert_eq!(
                got.is_some(),
                occupied,
                "iteration {i}: a tree with occupied={occupied} got the wrong cached conversion"
            );
            if let Some((parry_shape, _)) = got {
                assert_eq!(
                    parry_shape
                        .as_compound()
                        .expect("an occupied tree converts to a Compound")
                        .shapes()
                        .len(),
                    1,
                    "iteration {i}: wrong leaf count, i.e. another tree's Compound"
                );
            }
        }
    }

    #[test]
    fn convert_shape_octree_with_an_occupied_leaf_converts_to_a_compound() {
        let mut tree = moveit_octomap::OcTree::new(0.1);
        tree.update_node(nalgebra::Point3::new(0.05, 0.05, 0.05), true, false);
        let shape = Shape::OcTree(OcTree::from_tree(Arc::new(tree)));

        let (parry_shape, extra) = convert_shape(&shape, &OctreeCache::default())
            .expect("an occupied leaf must convert to a Compound");

        assert_eq!(extra, Isometry3::identity());
        let compound = parry_shape
            .as_compound()
            .expect("Shape::OcTree converts to a parry3d_f64::shape::Compound");
        assert_eq!(compound.shapes().len(), 1);
    }

    #[test]
    fn convert_shape_octree_two_adjacent_leaves_of_different_occupancy_are_not_merged() {
        // Two finest-resolution leaves sharing a face, one occupied and one
        // free: octomap cannot prune them into a shared parent (their
        // occupancy differs), so the Compound must carry two Cuboids, not
        // one coarse box straddling the boundary.
        let mut tree = moveit_octomap::OcTree::new(0.1);
        tree.update_node(nalgebra::Point3::new(0.05, 0.05, 0.05), true, true);
        tree.update_node(nalgebra::Point3::new(0.15, 0.05, 0.05), false, true);
        tree.update_inner_occupancy();
        tree.prune();
        let shape = Shape::OcTree(OcTree::from_tree(Arc::new(tree)));

        let (parry_shape, _) = convert_shape(&shape, &OctreeCache::default())
            .expect("one occupied leaf out of the two must still convert");
        let compound = parry_shape.as_compound().expect("shape is a Compound");
        assert_eq!(
            compound.shapes().len(),
            1,
            "only the occupied leaf becomes a Cuboid; its free neighbor contributes none, \
             and the two are never merged into one box"
        );
    }

    #[test]
    fn convert_shape_octree_caches_and_does_not_rebuild_on_a_second_call() {
        let mut tree = moveit_octomap::OcTree::new(0.1);
        tree.update_node(nalgebra::Point3::new(0.05, 0.05, 0.05), true, false);
        let shape = Shape::OcTree(OcTree::from_tree(Arc::new(tree)));
        let cache = OctreeCache::default();

        let (first_shape, _) = convert_shape(&shape, &cache).expect("first conversion succeeds");
        let (second_shape, _) = convert_shape(&shape, &cache).expect("second conversion succeeds");

        assert!(
            Arc::ptr_eq(&first_shape.0, &second_shape.0),
            "a second convert_shape call for the same tree must return the cached SharedShape, \
             not a freshly rebuilt Compound"
        );
    }

    #[test]
    fn octree_cache_get_or_compute_invokes_build_only_once_per_key() {
        let cache = OctreeCache::default();
        let tree = Arc::new(moveit_octomap::OcTree::new(0.1));
        let calls = std::cell::Cell::new(0);
        let build = || {
            calls.set(calls.get() + 1);
            Some(SharedShape::new(Ball::new(1.0)))
        };

        assert!(cache.get_or_compute(&tree, build).is_some());
        assert!(cache.get_or_compute(&tree, build).is_some());

        assert_eq!(
            calls.get(),
            1,
            "the second call for the same key must hit the cache"
        );
    }

    /// The growth-bound half of the cache-key fix (`8d5f4ee`): a `Weak`-only
    /// pin stops the address-reuse defect, but by itself would grow one entry
    /// per octree ever converted, forever -- exactly the sensor-driven
    /// rebuild-and-replace pattern this port is being built for. Dropping the
    /// last `Arc` to a tree (simulating `World::remove_object` discarding it)
    /// must free that entry by the *next* `get_or_compute` call, without
    /// [`OctreeCache`] ever being told to do so explicitly.
    #[test]
    fn octree_cache_prunes_an_entry_once_nothing_holds_its_tree() {
        let cache = OctreeCache::default();

        {
            let tree = Arc::new(moveit_octomap::OcTree::new(0.1));
            assert!(cache.get_or_compute(&tree, || None).is_none());
            assert_eq!(cache.len(), 1);
        }
        // `tree` just went out of scope: nothing holds that octree anymore,
        // matching a World object's octree being removed/replaced.

        let other = Arc::new(moveit_octomap::OcTree::new(0.1));
        assert!(cache.get_or_compute(&other, || None).is_none());

        assert_eq!(
            cache.len(),
            1,
            "the dead tree's entry must be pruned, not accumulated alongside the live one"
        );
    }

    #[test]
    fn axis_fix_maps_parry_y_up_onto_moveit_z_up() {
        let fixed = axis_fix() * Vector3::new(0.0, 1.0, 0.0);
        assert_eq!(fixed.x, 0.0);
        // `axis_fix()`'s 90-degree rotation is built from `f64::consts::FRAC_PI_2`'s
        // sin/cos, not exact literals, so this component carries a 1-ULP
        // (`f64::EPSILON`) residue rather than landing on 0.0 exactly.
        assert_relative_eq!(fixed.y, 0.0, epsilon = 1e-15, max_relative = 0.0);
        assert_eq!(fixed.z, 1.0);
    }

    #[test]
    fn to_contact_maps_fields_per_fcl2contact_convention() {
        let pc = ParryContact::new(
            ParryVector::new(0.0, 0.0, 0.0),
            ParryVector::new(1.0, 0.0, 0.0),
            ParryVector::new(1.0, 0.0, 0.0),
            ParryVector::new(-1.0, 0.0, 0.0),
            -0.5,
        );
        let c = to_contact(&pc, "a", BodyType::RobotLink, "b", BodyType::WorldObject);
        // Every input here is exactly representable and every step (midpoint,
        // copy, negate-and-clamp) is exact under IEEE 754, so this is a
        // structural identity, not a value measured for this input alone.
        assert_eq!(c.pos, Vector3::new(0.5, 0.0, 0.0));
        assert_eq!(c.normal, Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(c.depth, 0.5);
        assert_eq!(c.body_name_1, "a");
        assert_eq!(c.body_type_1, BodyType::RobotLink);
        assert_eq!(c.body_name_2, "b");
        assert_eq!(c.body_type_2, BodyType::WorldObject);
        assert_eq!(c.percent_interpolation, 0.0);
        assert_eq!(c.nearest_points[0], Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(c.nearest_points[1], Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn scaled_padded_shape_grows_a_cuboid_by_scale_then_padding() {
        // Upstream applies scale before padding; a unit half-extent scaled
        // by 2 then padded by 0.5 is 2.5, not (1 + 0.5) * 2 = 3.
        let shape = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).unwrap());
        let scaled = scaled_padded_shape(&shape, 2.0, 0.5);
        match scaled {
            // `size[0] * scale + padding * 2.0` on exactly-representable
            // operands (2.0 * 2.0 + 0.5 * 2.0 = 5.0) is exact under IEEE
            // 754, not a value measured for this input alone.
            Shape::Cuboid(c) => assert_eq!(c.size[0], 5.0),
            other => panic!("expected Cuboid, got {other:?}"),
        }
    }

    // Fixture: a fixed-base robot with a shapeless `base` link and two
    // independent floating-joint children `p`/`q`, each a 1x1x1 box, so each
    // can be posed to an arbitrary independent global transform via
    // `RobotState::set_joint_transform`. Mirrors the fixture pattern already
    // established in `moveit_model::robot_model`'s own test module.

    const FIXED_BASE_SRDF: &str = r#"<robot name="test">
        <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    </robot>"#;

    fn box_link(name: &str) -> String {
        format!(
            r#"<link name="{name}">
                <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>"#
        )
    }

    fn floating_joint(name: &str, parent: &str, child: &str) -> String {
        format!(
            r#"<joint name="{name}" type="floating">
                <parent link="{parent}"/>
                <child link="{child}"/>
            </joint>"#
        )
    }

    fn build_model(link_names: &[&str]) -> RobotModel {
        let links_and_joints: String = link_names
            .iter()
            .map(|name| {
                format!(
                    "{}{}",
                    box_link(name),
                    floating_joint(&format!("joint_{name}"), "base", name)
                )
            })
            .collect();
        let urdf_xml =
            format!(r#"<robot name="test"><link name="base"/>{links_and_joints}</robot>"#);
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(FIXED_BASE_SRDF).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("test fixture model must build")
    }

    fn state_with_links_at<'m>(
        model: &'m RobotModel,
        poses: &[(&str, Isometry3)],
    ) -> RobotState<'m> {
        let mut state = RobotState::new(model);
        for (link, pose) in poses {
            state
                .set_joint_transform(&format!("joint_{link}"), pose)
                .expect("floating joint transform must set");
        }
        state
    }

    #[test]
    fn check_self_collision_detects_overlapping_boxes() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_reports_free_when_boxes_are_apart() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(5.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_always_entry_suppresses_an_otherwise_colliding_pair() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);

        let result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_conditional_entry_predicate_decides() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("p", "q", Arc::new(|_: &mut Contact| true));

        let result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));

        assert!(!result.collision);
    }

    #[test]
    fn check_self_collision_conditional_entry_predicate_can_still_report_collision() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("p", "q", Arc::new(|_: &mut Contact| false));

        let result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_remove_entry_restores_default_behavior() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);
        acm.remove_entry("p", "q");

        let result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_max_contacts_budget_caps_stored_contacts_across_pairs() {
        // p, q, r all mutually overlapping at the origin: 3 colliding pairs.
        let model = build_model(&["p", "q", "r"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::identity()),
                ("r", Isometry3::identity()),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 2,
            max_contacts_per_pair: 1,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(result.collision);
        let stored: usize = result.contacts.expect("contacts requested").count();
        assert_eq!(stored, 2);
    }

    #[test]
    fn check_self_collision_still_reports_collision_with_a_spent_contact_budget() {
        // `max_contacts: 0` is not a hypothetical. `CollisionEnv::check_collision`
        // subtracts the self-check's contact count from the request before
        // calling the robot check (PORTING-PLAN.md 10.5), saturating at zero, so
        // a self-check that fills the budget hands the robot check exactly this
        // request. The budget governs how many contacts are *stored*; it must
        // never govern whether a collision is *found*. A backend that folded the
        // two together would report a clear scene for the overlapping pair here,
        // and the caller has no way to tell that apart from a genuinely clear one.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[("p", Isometry3::identity()), ("q", Isometry3::identity())],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 0,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(
            result.collision,
            "a spent contact budget must not suppress the collision flag"
        );
        assert_eq!(
            result.contacts.expect("contacts requested").count(),
            0,
            "a spent contact budget must store nothing"
        );
    }

    #[test]
    fn check_robot_collision_detects_overlap_with_a_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(result.collision);
    }

    #[test]
    fn check_robot_collision_reports_free_when_world_object_is_far_away() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(10.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(!result.collision);
    }

    /// One occupied leaf, wrapped as a [`Shape::OcTree`] world object, at
    /// `leaf_center` in a tree of `resolution`.
    fn octree_world_with_one_leaf(resolution: f64, leaf_center: nalgebra::Point3<f64>) -> World {
        let mut tree = moveit_octomap::OcTree::new(resolution);
        tree.update_node(leaf_center, true, false);
        let mut world = World::new();
        world.add_shape(
            "octree_obstacle",
            Arc::new(Shape::OcTree(OcTree::from_tree(Arc::new(tree)))),
            Isometry3::identity(),
        );
        world
    }

    #[test]
    fn check_robot_collision_detects_overlap_with_an_octree_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        // A leaf at the origin sits well inside the 1x1x1 robot box at "p".
        let world = octree_world_with_one_leaf(0.1, nalgebra::Point3::new(0.0, 0.0, 0.0));
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(result.collision);
    }

    #[test]
    fn check_robot_collision_reports_free_when_octree_leaf_is_far_away() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let world = octree_world_with_one_leaf(0.1, nalgebra::Point3::new(10.0, 0.0, 0.0));
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(!result.collision);
    }

    #[test]
    fn check_robot_collision_touching_exactly_on_an_octree_leaf_face_is_detected() {
        // The 1x1x1 robot box at "p" has its +x face at x=0.5. A leaf of
        // size 0.1 centered at x=0.55 spans [0.5, 0.6]: its -x face lands
        // exactly on the robot's +x face, a touching (zero-gap, not
        // overlapping) contact -- the same "prediction 0.0 counts touching
        // as collision" convention this backend already applies to every
        // other shape pair (module doc, deviation 5).
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let world = octree_world_with_one_leaf(0.1, nalgebra::Point3::new(0.55, 0.0, 0.0));
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(
            result.collision,
            "a leaf face exactly touching the robot's face must count as a collision"
        );
    }

    /// Round 7 item 3: the [`OctreeCache`] doc's claim that a caller who
    /// rebuilds-and-replaces one `World` object's octree every scan "holds
    /// at most one stale entry at a time" is tested here end-to-end -- a
    /// real `World`, a real `ParryCollisionEnv`, and a real
    /// `check_robot_collision` call every iteration -- not merely asserted
    /// in prose. `World::add_shape` on an id that already exists *appends*
    /// a shape rather than replacing it (`World::add_to_object`'s own doc),
    /// so a genuine replace has to drop the old object first via
    /// `World::remove_object`; that drop is what makes the previous
    /// iteration's tree unreachable and lets `OctreeCache::get_or_compute`'s
    /// prune -- run as the very first action of every call -- collect it on
    /// the very next query.
    #[test]
    fn octree_cache_stays_bounded_across_a_real_rebuild_and_replace_loop() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();

        let mut env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());

        for i in 0..50 {
            let mut tree = moveit_octomap::OcTree::new(0.1);
            // Far from "p": the collision result itself is not what this
            // test checks, only the cache's entry count.
            tree.update_node(
                nalgebra::Point3::new(10.0 + i as f64, 0.0, 0.0),
                true,
                false,
            );

            env.world_mut().remove_object("octree_obstacle");
            env.world_mut().add_shape(
                "octree_obstacle",
                Arc::new(Shape::OcTree(OcTree::from_tree(Arc::new(tree)))),
                Isometry3::identity(),
            );

            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);

            assert!(
                env.octree_cache.len() <= 1,
                "iteration {i}: cache held {} entries for a single rebuild-and-replace \
                 object, expected at most 1",
                env.octree_cache.len()
            );
        }

        assert_eq!(
            env.octree_cache.len(),
            1,
            "after the loop, exactly the last scan's tree should still be cached"
        );
    }

    #[test]
    fn distance_self_reports_the_gap_between_separated_boxes() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(2.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.distance_self(&DistanceRequest::default(), &posed, &[]);

        assert!(!result.collision);
        // Axis-aligned box distance at an exact-literal translation reduces
        // to subtraction of exact literals -- exact under IEEE 754. The same
        // reasoning applies to the other minimum-distance assertions below.
        assert_eq!(result.minimum_distance.distance, 1.0);
    }

    #[test]
    fn distance_self_clamps_to_zero_without_signed_distance() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: false,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[]);

        assert!(result.collision);
        assert_eq!(result.minimum_distance.distance, 0.0);
    }

    #[test]
    fn distance_self_reports_negative_penetration_with_signed_distance_enabled() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[]);

        assert!(result.collision);
        assert_eq!(result.minimum_distance.distance, -0.5);
    }

    #[test]
    fn distance_self_does_not_panic_when_an_earlier_pair_deeply_penetrates() {
        // p, q, r all identically posed: three mutually, deeply overlapping
        // pairs. `DistanceRequestType::Global` (the default) folds every
        // pair's threshold into the running `minimum_distance`, so whichever
        // pair is visited first drives it deeply negative; every later pair
        // must still be queryable rather than handed that negative value as
        // `parry`'s prediction margin (`bounded_prediction` used to pass it
        // through unclamped on the low end, and `parry` panics on a negative
        // margin).
        let model = build_model(&["p", "q", "r"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::identity()),
                ("r", Isometry3::identity()),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[]);

        assert!(result.collision);
        assert_eq!(result.minimum_distance.distance, -1.0);
    }

    #[test]
    fn distance_self_always_entry_skips_the_pair_entirely() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);
        let request = DistanceRequest {
            acm: Some(&acm),
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[]);

        assert!(!result.collision);
        assert_eq!(result.minimum_distance.distance, f64::MAX);
    }

    #[test]
    fn distance_self_never_entry_has_no_effect_unlike_collision_checking() {
        // Unlike `check_self_collision`, `Never`/`Conditional` ACM entries
        // have no effect on a distance query at all (module doc, deviation
        // 6): only `Always` skips a pair.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", false);
        let request = DistanceRequest {
            acm: Some(&acm),
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[]);

        assert!(result.collision);
        assert_eq!(result.minimum_distance.distance, -0.5);
    }

    #[test]
    fn distance_robot_reports_the_gap_to_a_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.distance_robot(&DistanceRequest::default(), &posed, &[]);

        assert!(!result.collision);
        assert_eq!(result.minimum_distance.distance, 1.0);
    }

    #[test]
    fn distance_robot_reports_the_gap_to_an_octree_world_object() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        // Robot box's +x face is at x=0.5; a leaf of size 0.1 centered at
        // x=1.55 has its -x face at x=1.5, a 1.0m gap -- deliberately the
        // same gap `distance_robot_reports_the_gap_to_a_world_object` checks
        // for a Cuboid world object, so the two are directly comparable.
        let world = octree_world_with_one_leaf(0.1, nalgebra::Point3::new(1.55, 0.0, 0.0));
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let result = env.distance_robot(&DistanceRequest::default(), &posed, &[]);

        assert!(!result.collision);
        assert_eq!(result.minimum_distance.distance, 1.0);
    }

    #[test]
    fn check_robot_collision_continuous_returns_an_error_rather_than_approximating() {
        let model = build_model(&["p"]);
        let mut state1 = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let mut state2 =
            state_with_links_at(&model, &[("p", Isometry3::translation(1.0, 0.0, 0.0))]);
        let posed1 = state1.update();
        let posed2 = state2.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_robot_collision_continuous(
            &CollisionRequest::default(),
            &posed1,
            &posed2,
            &[],
            None,
        );

        assert!(result.is_err());
    }

    // Attached-body geometry: module doc, "Attached-body geometry".

    #[test]
    fn check_robot_collision_detects_overlap_via_attached_body_geometry() {
        // `p` itself sits well clear of the obstacle; only geometry reached
        // through an attached body's `shape_poses` overlaps it -- this is the
        // gap `robot_bodies` iterating only `link_models()` used to leave.
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(3.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let without_attached =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);
        assert!(
            !without_attached.collision,
            "p's own geometry must not reach the obstacle"
        );

        let shapes = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let shape_poses = vec![Isometry3::translation(2.8, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let with_attached =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[attached], None);
        assert!(
            with_attached.collision,
            "attached body geometry must be checked against the world too"
        );
    }

    #[test]
    fn distance_robot_reports_the_gap_to_a_world_object_via_attached_body_geometry() {
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(5.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let shapes = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let shape_poses = vec![Isometry3::translation(3.0, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let result = env.distance_robot(&DistanceRequest::default(), &posed, &[attached]);

        assert!(!result.collision);
        assert_eq!(result.minimum_distance.distance, 1.0);
    }

    #[test]
    fn check_self_collision_two_attached_bodies_on_the_same_link_never_collide() {
        // `p` itself sits clear of both attached bodies; `a`/`b` are attached
        // to the same link and placed at the identical pose, so only the
        // same-attached-link touch-links rule (module doc) can be why this
        // reports clear.
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let shapes_a = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_a = vec![Isometry3::translation(5.0, 0.0, 0.0)];
        let touch_a = BTreeSet::new();
        let a = AttachedBodyGeometry {
            id: "a",
            link_name: "p",
            shapes: &shapes_a,
            shape_poses: &poses_a,
            touch_links: &touch_a,
        };
        let shapes_b = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_b = vec![Isometry3::translation(5.0, 0.0, 0.0)];
        let touch_b = BTreeSet::new();
        let b = AttachedBodyGeometry {
            id: "b",
            link_name: "p",
            shapes: &shapes_b,
            shape_poses: &poses_b,
            touch_links: &touch_b,
        };

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[a, b], None);

        assert!(
            !result.collision,
            "two attached bodies on the same link must never collide with each other"
        );
    }

    #[test]
    fn distance_self_two_attached_bodies_on_the_same_link_still_reports_penetration() {
        // The mirror of the test above, over `distance_self`: `collisionCallback`'s
        // same-attached-link rule is verified absent from `distanceCallback`
        // (module doc), so this pair must NOT be skipped here, unlike
        // `check_self_collision` on identical geometry.
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let shapes_a = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_a = vec![Isometry3::translation(5.0, 0.0, 0.0)];
        let touch_a = BTreeSet::new();
        let a = AttachedBodyGeometry {
            id: "a",
            link_name: "p",
            shapes: &shapes_a,
            shape_poses: &poses_a,
            touch_links: &touch_a,
        };
        let shapes_b = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_b = vec![Isometry3::translation(5.0, 0.0, 0.0)];
        let touch_b = BTreeSet::new();
        let b = AttachedBodyGeometry {
            id: "b",
            link_name: "p",
            shapes: &shapes_b,
            shape_poses: &poses_b,
            touch_links: &touch_b,
        };
        let request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };

        let result = env.distance_self(&request, &posed, &[a, b]);

        assert!(result.collision);
        assert_eq!(result.minimum_distance.distance, -1.0);
    }

    #[test]
    fn check_self_collision_touch_links_allows_a_link_and_an_attached_body_to_overlap() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(5.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let shapes = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let shape_poses = vec![Isometry3::translation(5.0, 0.0, 0.0)];

        let no_touch = BTreeSet::new();
        let without_touch_links = AttachedBodyGeometry {
            id: "pad",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &no_touch,
        };
        let baseline = env.check_self_collision(
            &CollisionRequest::default(),
            &posed,
            &[without_touch_links],
            None,
        );
        assert!(
            baseline.collision,
            "the attached body's geometry must actually overlap q for this test to prove anything"
        );

        let mut touch = BTreeSet::new();
        touch.insert("q".to_string());
        let with_touch_links = AttachedBodyGeometry {
            id: "pad",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch,
        };
        let result = env.check_self_collision(
            &CollisionRequest::default(),
            &posed,
            &[with_touch_links],
            None,
        );
        assert!(
            !result.collision,
            "an attached body's touch_links must allow overlap with the link it names"
        );
    }

    #[test]
    fn check_self_collision_touch_links_allows_two_attached_bodies_on_different_links_to_overlap() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.0, 5.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let shapes_a = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_a = vec![Isometry3::translation(10.0, 0.0, 0.0)];
        let shapes_b = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let poses_b = vec![Isometry3::translation(10.0, -5.0, 0.0)];

        let no_touch = BTreeSet::new();
        let baseline_a = AttachedBodyGeometry {
            id: "body_a",
            link_name: "p",
            shapes: &shapes_a,
            shape_poses: &poses_a,
            touch_links: &no_touch,
        };
        let baseline_b = AttachedBodyGeometry {
            id: "body_b",
            link_name: "q",
            shapes: &shapes_b,
            shape_poses: &poses_b,
            touch_links: &no_touch,
        };
        let baseline = env.check_self_collision(
            &CollisionRequest::default(),
            &posed,
            &[baseline_a, baseline_b],
            None,
        );
        assert!(
            baseline.collision,
            "body_a and body_b must actually overlap for this test to prove anything"
        );

        let mut touch_a = BTreeSet::new();
        touch_a.insert("body_b".to_string());
        let a = AttachedBodyGeometry {
            id: "body_a",
            link_name: "p",
            shapes: &shapes_a,
            shape_poses: &poses_a,
            touch_links: &touch_a,
        };
        let b = AttachedBodyGeometry {
            id: "body_b",
            link_name: "q",
            shapes: &shapes_b,
            shape_poses: &poses_b,
            touch_links: &no_touch,
        };
        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[a, b], None);
        assert!(
            !result.collision,
            "one attached body naming the other in touch_links must allow their overlap, \
             regardless of which side names which"
        );
    }

    // Cost sources: `cost_sources_for_part_pair` and its two mesh helpers,
    // tested directly against hand-computed AABBs so the exact numbers are
    // checked, not just "something non-empty came out" -- plus the
    // `accumulate_collision`-level invariants (None-vs-empty-Vec, the
    // `Conditional`-accept case, and `max_cost_sources` trimming) that no
    // amount of geometry-level testing alone would cover.

    fn unit_cuboid() -> ParryCuboid {
        ParryCuboid::new(ParryVector::new(0.5, 0.5, 0.5))
    }

    /// `approx` has no blanket `RelativeEq` for `[f64; 3]`; this compares
    /// component-wise instead of stringifying both sides into a slice.
    fn assert_point_close(actual: [f64; 3], expected: [f64; 3]) {
        for i in 0..3 {
            // Hand-computed AABB coordinates against exact-literal cuboid
            // geometry -- exact, not merely close.
            assert_eq!(actual[i], expected[i]);
        }
    }

    #[test]
    fn cost_sources_for_part_pair_shape_shape_is_the_overlap_of_both_whole_aabbs() {
        let a_pose = to_pose(Isometry3::identity());
        let b_pose = to_pose(Isometry3::translation(0.5, 0.0, 0.0));
        let a = unit_cuboid();
        let b = unit_cuboid();

        let sources = cost_sources_for_part_pair(&a_pose, &a, &b_pose, &b);

        assert_eq!(sources.len(), 1);
        assert_point_close(sources[0].aabb_min, [0.0, -0.5, -0.5]);
        assert_point_close(sources[0].aabb_max, [0.5, 0.5, 0.5]);
        assert_eq!(sources[0].cost, 1.0);
    }

    #[test]
    fn cost_sources_for_part_pair_shape_shape_disjoint_is_empty() {
        let a_pose = to_pose(Isometry3::identity());
        let b_pose = to_pose(Isometry3::translation(50.0, 0.0, 0.0));
        let a = unit_cuboid();
        let b = unit_cuboid();

        assert!(cost_sources_for_part_pair(&a_pose, &a, &b_pose, &b).is_empty());
    }

    /// One large flat triangle in the z=0 plane, base `y = -5` to apex
    /// `(0, 5)`: at `y = -2` its footprint spans `x ∈ [-3.5, 3.5]` (linear
    /// interpolation along both slanted edges), comfortably containing every
    /// query shape the tests below place near `(0, -2, 0)` — chosen so which
    /// triangle a query point falls into is never ambiguous, unlike a
    /// two-triangle square split along a diagonal that could pass through
    /// the query point itself.
    fn big_flat_triangle() -> TriMesh {
        TriMesh::new(
            vec![
                ParryVector::new(-5.0, -5.0, 0.0),
                ParryVector::new(5.0, -5.0, 0.0),
                ParryVector::new(0.0, 5.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap()
    }

    #[test]
    fn mesh_shape_cost_sources_is_one_triangle_aabb_overlapped_with_the_whole_shape_aabb() {
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = big_flat_triangle();
        let other_pose = to_pose(Isometry3::translation(0.0, -2.0, 0.0));
        let other = ParryCuboid::new(ParryVector::new(0.1, 0.1, 0.1));

        let sources = mesh_shape_cost_sources(&mesh_pose, &mesh, &other_pose, &other);

        assert_eq!(sources.len(), 1);
        assert_point_close(sources[0].aabb_min, [-0.1, -2.1, 0.0]);
        assert_point_close(sources[0].aabb_max, [0.1, -1.9, 0.0]);
        assert_eq!(sources[0].cost, 1.0);
    }

    #[test]
    fn mesh_shape_cost_sources_no_intersection_is_empty() {
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = big_flat_triangle();
        // Well outside the triangle's footprint, even though its AABB (a
        // huge flat square, z ranging only over [0, 0]) still overlaps the
        // cuboid's own AABB in x/y -- this exercises the exact geometric
        // `query::intersection_test` gate, not merely a bounding-box check.
        let other_pose = to_pose(Isometry3::translation(-4.9, 4.9, 5.0));
        let other = ParryCuboid::new(ParryVector::new(0.05, 0.05, 0.05));

        assert!(mesh_shape_cost_sources(&mesh_pose, &mesh, &other_pose, &other).is_empty());
    }

    #[test]
    fn mesh_mesh_cost_sources_is_the_intersecting_triangle_pairs_overlap() {
        let a_pose = to_pose(Isometry3::identity());
        let mesh_a = big_flat_triangle();
        // A thin vertical triangle through the flat one at (0, -2, 0):
        // base at z=-0.5 spanning x in [-0.1, 0.1], apex at (0, -2, 0.5).
        // Linearly interpolating to z=0 (halfway) narrows the cross-section
        // to x in [-0.05, 0.05] -- still inside mesh_a's x in [-3.5, 3.5]
        // footprint at y=-2, so the two triangles genuinely cross rather
        // than merely sharing an AABB.
        let mesh_b = TriMesh::new(
            vec![
                ParryVector::new(-0.1, -2.0, -0.5),
                ParryVector::new(0.1, -2.0, -0.5),
                ParryVector::new(0.0, -2.0, 0.5),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap();
        let b_pose = to_pose(Isometry3::identity());

        let sources = mesh_mesh_cost_sources(&a_pose, &mesh_a, &b_pose, &mesh_b);

        assert_eq!(sources.len(), 1);
        assert_point_close(sources[0].aabb_min, [-0.1, -2.0, 0.0]);
        assert_point_close(sources[0].aabb_max, [0.1, -2.0, 0.0]);
        assert_eq!(sources[0].cost, 1.0);
    }

    #[test]
    fn mesh_mesh_cost_sources_no_intersection_is_empty() {
        let a_pose = to_pose(Isometry3::identity());
        let mesh_a = big_flat_triangle();
        let mesh_b = TriMesh::new(
            vec![
                ParryVector::new(-0.1, 100.0, -0.5),
                ParryVector::new(0.1, 100.0, -0.5),
                ParryVector::new(0.0, 100.0, 0.5),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap();
        let b_pose = to_pose(Isometry3::identity());

        assert!(mesh_mesh_cost_sources(&a_pose, &mesh_a, &b_pose, &mesh_b).is_empty());
    }

    #[test]
    fn check_self_collision_cost_sources_is_none_when_not_requested() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(result.cost_sources.is_none());
    }

    #[test]
    fn check_self_collision_cost_sources_is_some_empty_when_requested_but_nothing_collides() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(5.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            cost: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(!result.collision);
        assert_eq!(result.cost_sources, Some(Vec::new()));
    }

    #[test]
    fn check_self_collision_cost_sources_populated_for_a_colliding_pair() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            cost: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        let sources = result.cost_sources.expect("cost requested");
        assert_eq!(sources.len(), 1);
        assert_point_close(sources[0].aabb_min, [0.0, -0.5, -0.5]);
        assert_point_close(sources[0].aabb_max, [0.5, 0.5, 0.5]);
        assert_eq!(sources[0].cost, 1.0);
    }

    #[test]
    fn check_self_collision_cost_sources_computed_even_when_conditional_predicate_accepts() {
        // `AllowedCollision::Conditional`'s predicate deciding "not a
        // collision" must not suppress cost-source computation: reading
        // `collision_common.cpp` in full shows `if (enable_cost) {...}`
        // running unconditionally after fcl::collide, independent of what
        // the pair's own `dcf` predicate goes on to decide per contact.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_conditional_entry("p", "q", Arc::new(|_: &mut Contact| true));
        let request = CollisionRequest {
            cost: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], Some(&acm));

        assert!(!result.collision, "predicate accepted the contact");
        let sources = result.cost_sources.expect("cost requested");
        assert_eq!(
            sources.len(),
            1,
            "cost is computed independently of the predicate's accept/reject decision"
        );
    }

    #[test]
    fn check_self_collision_cost_sources_always_entry_skips_cost_too() {
        // The `AllowedCollision::Always`/touch-links skip happens *before*
        // any geometry query at all -- no `parry3d_f64::query::contact` call
        // means no cost source either, matching upstream's `if
        // (always_allow_collision) return false;` guarding the entire rest
        // of `collisionCallback`, cost included.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "q", true);
        let request = CollisionRequest {
            cost: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], Some(&acm));

        assert!(!result.collision);
        assert_eq!(result.cost_sources, Some(Vec::new()));
    }

    #[test]
    fn check_self_collision_max_cost_sources_keeps_only_the_most_costly() {
        // p, q, r pairwise overlapping by different amounts along x, so
        // each pair's overlap volume (cost is a constant `1.0`, so ranking
        // is purely by volume) differs: p-q overlaps by 0.1, p-r by 0.8,
        // q-r by 0.3. `max_cost_sources: 2` must keep the two largest
        // (p-r, q-r) and drop the smallest (p-q) -- exercising the same
        // trim upstream's `while (cost_sources.size() > max_cost_sources)
        // erase(--end())` performs, most-costly-first.
        let model = build_model(&["p", "q", "r"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(0.9, 0.0, 0.0)),
                ("r", Isometry3::translation(0.2, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            cost: true,
            max_cost_sources: 2,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        let sources = result.cost_sources.expect("cost requested");
        assert_eq!(sources.len(), 2, "trimmed to max_cost_sources");
        let volumes: Vec<f64> = sources.iter().map(CostSource::volume).collect();
        // p-r's overlap is `1.0 - 0.2`, exact.
        assert_eq!(volumes[0], 0.8);
        // q-r's overlap is `1.0 - (0.9 - 0.2)`: `0.9 - 0.2` is not exactly
        // representable in binary floating point (it rounds to
        // 0.7000000000000001), so the result carries a 1-ULP residue.
        assert_relative_eq!(volumes[1], 0.3, epsilon = 1e-15, max_relative = 0.0);
    }
}
