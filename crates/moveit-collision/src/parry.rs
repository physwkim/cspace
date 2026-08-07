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
//! 1. **`group_name` restricts a pair to links [`moveit_model::JointModelGroup`] updates,
//!    OR'd across the pair, not ANDed.** `checkSelfCollisionHelper`/
//!    `checkRobotCollisionHelper` (`collision_env_fcl.cpp:274-298`/
//!    `328-355`) both call `cd.enableGroup(getRobotModel())` unconditionally,
//!    which resolves `req_->group_name` to
//!    `JointModelGroup::getUpdatedLinkModelsSet()` (every link a joint in
//!    that group moves, including fixed-joint descendants —
//!    [`moveit_model::JointModelGroup::updated_link_names`] is the same set, already
//!    ported) and `collisionCallback` (`collision_detection_fcl/collision_common.cpp:79-94`) then
//!    skips a pair only when *neither* side resolves to an active link — a
//!    world object never resolves to one on its own, so a robot-vs-world
//!    pair is kept exactly when the robot link is active, and a self-pair is
//!    kept when *either* link is. `distanceCallback` reads the same
//!    `DistanceRequest::active_components_only` (`collision_detection_fcl/collision_common.cpp:483-500`);
//!    `distanceSelf`/`distanceRobot` themselves never call `enableGroup` (an
//!    earlier draft of this doc took that as proof group filtering was
//!    unwired everywhere — it is not: the caller of `distanceSelf`/
//!    `distanceRobot` is expected to have already populated
//!    `active_components_only`, exactly as `checkSelfCollisionHelper`'s own
//!    `dreq.group_name = req.group_name; dreq.enableGroup(...)` does before
//!    calling `distanceSelf`). [`active_group_links`] reproduces the
//!    `enableGroup` resolution and [`pair_in_active_group`] the callbacks'
//!    filter predicate, applied in [`ParryCollisionEnv::check_self_collision`]/
//!    [`CollisionEnv::check_robot_collision`]/[`CollisionEnv::distance_self`]/[`CollisionEnv::distance_robot`].
//! 2. **World objects are never padded or scaled.** Verified from
//!    `constructFCLObjectWorld` (calls the two-argument
//!    `createCollisionGeometry(shape, obj)` overload, no scale/padding) versus
//!    `constructFCLObjectRobot` (uses the padding/scale-taking overload via
//!    the cached `robot_geoms_`). [`LinkPaddingScale`] is consulted only when
//!    converting a [`moveit_model::LinkModel`]'s shapes, never a
//!    [`crate::World`] object's.
//! 3. **This backend always applies the [`LinkPaddingScale`] it was built
//!    with, to the self check and the robot-vs-world check alike** — as
//!    `CollisionEnvFCL` does: neither `collision_env_fcl.cpp` nor
//!    `collision_common.cpp` mentions `pad_environment_collisions`/
//!    `pad_self_collisions`, and both checks read the one padded
//!    `robot_geoms_` the constructor built. Those two flags are a
//!    `PlanningScene`-level choice between two whole environments
//!    (`planning_scene.cpp:442`, `:453`, `:558`) and are not ported here;
//!    see [`CollisionRequest`]'s own doc.
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
//!    [`fn@query::contact`] against a [`TriMesh`] visits every triangle
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
//!    (`collision_detection_fcl/collision_common.cpp:636-663` at the pinned commit — the fallback
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
//!    `collision_detection_fcl/collision_common.cpp:900-922,949`; `fcl::OBBRSSd` at
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
//!    surfaces `Op::RandomStates`'s own per-case joint values, only
//!    aggregate pass/fail) spoke the oracle's wire protocol directly for
//!    one whole-model `random_states` sweep (seed `20260817`, `8000`
//!    cases) followed by one `collision` request per case, both ops
//!    already used elsewhere in this repo, landing on 467 self same-pair
//!    `base_link`/`*_caster_*_wheel_link` cases, of which 16 sit within the
//!    same `2x` smaller-link-bounding-radius plausibility bound
//!    `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible` already
//!    tests for — 16 by this sweep's own count, not forced to match round
//!    15's.
//!
//!    Every case's `base_link`/wheel poses came from this port's own FK
//!    (the same `RobotState`/`Posed` [`CollisionEnv::distance_self`] itself uses), fed
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
//!    (`tools/moveit-oracle/src/oracle.cpp:2464`), the 2-argument
//!    constructor, and never calls `setPadding`/`setLinkPadding` anywhere on
//!    that op's path — so every pr2 link's padding stays at the ctor
//!    default. That default genuinely reaches FCL's geometry construction,
//!    not a dead field: `CollisionEnvFCL`'s own `model, world, padding,
//!    scale` constructor (the one `CollisionEnvFCL env(model_, world)` at
//!    `oracle.cpp:2464` resolves to) calls
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
//!    `e017c91e` checkout finds 0 hits — an absence, so this expires the
//!    moment the moveit2 pin moves past a commit where `moveit_core` itself
//!    starts specifying a split method. For `fcl::OBBRSSd`, `SPLIT_METHOD_MEAN`
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
//!    (`parry3d-f64-0.30.0/src/query/contact/contact_composite_shape_shape.rs:24-39`) — unlike
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
//!    whatever that order turns out to be — and this half expires too, if
//!    `base_link`'s own collision mesh is ever replaced with one whose
//!    triangle count exceeds 200. All 3 residual links carry the
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
//!    different-triangle split it fell on was not recorded). The
//!    mesh-construction-fidelity question above — vertex order and triangle
//!    indices — is not a second open candidate: both halves are closed
//!    (`PORTING-PLAN.md` §168), 36/36 exact matches on the fixture's own
//!    oracle-emission order and on the oracle's own `triangles` field,
//!    confirmed non-vacuous by mutation (flipping one triangle's winding or
//!    swapping two vertices each fails its own assertion). Ruling out the
//!    BVH question narrows deviation 6(b)'s remaining candidate to
//!    exactly one: narrow-phase magnitude bias, not yet measured directly
//!    for these 3 states specifically.
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
//!    Round 21 extended this same magnitude-bias mechanism to a second,
//!    independent mesh: `moveit-constraints`' visibility-cone check (a
//!    `cone_sides`-gon mesh, not `base_link`'s STL) against the identical
//!    `bl_caster_l_wheel_link` cylinder. `tools/moveit-diff`'s own captured
//!    mismatch (`main.rs`'s `a_real_mismatching_case_touches_exactly_one_link`,
//!    "case 104": oracle `7.47914550966356367e-2`, this backend
//!    `2.08696987934593702e-2`) gave a full, reproducible joint-state/cone
//!    spec to work from. Reconstructing it (this crate's own `RobotModel`/
//!    `RobotState` FK for the wheel, the cone mesh built by the exact same
//!    vertex/triangle formula `VisibilityConstraint::cone_mesh` uses) and
//!    calling `parry3d_f64::query::contact` per candidate triangle the same
//!    way `native_deepest_triangle_vs_cylinder` does for `base_link`
//!    reproduces this backend's own `2.08696987934592244e-2` (matching the
//!    captured reference to float noise, `~1.5e-14` relative) and names the
//!    winning triangle: cone vertices `[5, 1, 6]`, where vertex `1` (the
//!    cone's own base-center point) lands within `1.1e-16` of the wheel
//!    cylinder's own local origin — the visibility-cone generator anchors a
//!    "near" case's target pose exactly at the touched link's own shape
//!    center (`tools/moveit-diff`'s `build_constraint_case`/
//!    `crates/moveit-constraints/examples/visibility_cone_depth_sweep.rs`'s
//!    `build_case`, `Some(link_name)` arm), so every such case interpenetrates
//!    through the link's own centroid by construction, not a coincidence of
//!    this one case.
//!
//!    Driving the real, unmodified `ccdMPRPenetration` (round 16/17's own
//!    build: libccd git tag `v2.1`, `CCD_DOUBLE`) directly on that exact
//!    triangle (in the cylinder's own local, Z-native frame — `ccd_cyl_t`'s
//!    own support function (`testsuites/support.c`) reads `ccdVec3Z(&dir)`
//!    directly, so unlike `parry3d_f64::shape::Cylinder` this needs no
//!    Y-onto-Z [`axis_fix`]; applying it anyway was this round's own
//!    first-pass error, caught by re-reading `testsuites/support.c` and
//!    round 16's own `gen_cases.py` — which already used the plain Z
//!    convention — before trusting the resulting number) against the same
//!    real cylinder radius/length gives `7.47919999515277989e-2` — the
//!    oracle's own reported depth (`7.47914550966356367e-2`) to within
//!    `5.4e-7` absolute, `~7.3ppm` relative, the same order as round 17's own
//!    "under `1e-6`" corroboration bound for a genuinely-reproduced code
//!    path. Round 26 committed this as a reproducible harness rather than
//!    leaving it as prose (`tools/mpr-vs-epa/`, fed by this crate's own
//!    `examples/case104_mpr_input.rs`) — see `doc/claim-audit/moveit-collision.md`'s
//!    own round-26 section for the end-to-end confirmation. **This is the
//!    same one-directional bias, not a new mechanism**:
//!    libccd's MPR overestimates relative to this backend's own EPA for this
//!    triangle too (`0.0748` vs `0.0209`, MPR deeper), consistent with the
//!    16/16 base_link sample's own sign and inside its `0.0312`-`0.1042m`
//!    magnitude band. So the `visibility_cone` population's mismatches are
//!    not a distinct open question: they are governed by the same
//!    already-characterized deviation 6(b) — libccd's MPR is a portal-
//!    refinement algorithm, not exact EPA, and round 16's 560-pose sweep
//!    already established it is not guaranteed to converge to the true
//!    minimum penetration-depth witness the way this backend's own EPA does
//!    — made unusually large in this population specifically because the
//!    generator's own near-placement always drives the interpenetration
//!    through the target link's centroid, not because the underlying
//!    mechanism differs. Not a fixable defect in this port for case 104
//!    itself: matching libccd's own number here would mean re-implementing
//!    libccd's specific, non-exact MPR early-termination behavior rather
//!    than computing the true minimum-translation separating distance, the
//!    wrong direction for this backend's independent EPA to go on this one
//!    case. Whether "MPR deeper" is the *only* direction this early
//!    termination ever produces, for this pair shape in general, is a
//!    separate question this single case cannot answer by itself — round
//!    27 below measures it on 945 cases, not one, and round 28 explains the
//!    exception mechanically rather than leaving it as a correlation. Same
//!    expiry condition as round 16/17's own finding: re-open only if a
//!    future moveit2 pin changes FCL/libccd's own narrow-phase algorithm.
//!
//!    Round 25's `collision_parity.rs` test
//!    `visibility_cone_near_placement_interpenetrates_through_the_touched_links_own_centroid`
//!    checks whether case 104's own mechanism generalizes across 15
//!    `(joint_state, target_radius, cone_sides)` combinations, not just the
//!    one measured above, and splits it into two claims: every sampled
//!    case's cone vertices do stay inside the touched cylinder's own
//!    inscribed sphere and interpenetrate through its centroid by
//!    construction (confirmed, all 15), but the *winning* triangle
//!    specifically containing the target-center vertex the way case 104's
//!    own `[5, 1, 6]` did is **not** a general rule — measured true in only
//!    4 of the 15. Case 104 was this mechanism's most visible instance, not
//!    its typical shape; the rest win through a triangle sharing the sensor
//!    vertex instead. The interpenetration claim above (why the whole
//!    population is unusually large) does not depend on which vertex the
//!    winning triangle happens to share.
//!
//!    **Round 27 falsified the "always deeper" half of the paragraph above
//!    ("libccd's MPR overestimates relative to this backend's own EPA for
//!    this triangle too") on a real sample, not the single case 104 that
//!    sentence generalized from.** `examples/visibility_cone_mpr_sweep.rs`
//!    extends case 104's own single-case comparison to every
//!    `visibility_cone` case, in one 1000-case pr2 sweep (seed 4), where
//!    this backend's own EPA depth disagrees with the oracle's own reported
//!    depth by more than `1e-4`: 945 such mismatches got a real
//!    `ccdMPRPenetration` reading on their own winning triangle. **853
//!    (90.3%) are deeper, as round 21's single case predicted; 83 (8.8%)
//!    match EPA within float noise (`<1e-9`); but 9 (0.95%) are genuinely
//!    *shallower*, by `0.0088`–`0.0147m`, four orders of magnitude past the
//!    noise floor — MPR is not deeper than EPA "by construction" for this
//!    pair shape.** `pearson(gap, epa_depth) = 0.107`, `pearson(gap,
//!    triangle_size) = -0.182` — the gap magnitude is not strongly
//!    explained by penetration depth or triangle size alone, and (round 28
//!    below) neither correlation is the mechanism, only a symptom that
//!    something else was going on.
//!
//!    **Round 28 gets inside `ccdMPRPenetration` itself (libccd `v2.1`,
//!    `src/mpr.c`) rather than stopping at the correlation.** All 9 shallow
//!    cases read `mpr_depth = 1.700000e-2` to six significant figures — pr2
//!    wheel collision cylinders are `<cylinder length="0.034"
//!    radius="0.074792"/>` (`fixtures/pr2.urdf`), `0.034 / 2 == 0.017`
//!    exactly, the cylinder's own half-length — and `mpr_case104` is fed
//!    the exact winning triangle this backend's own EPA search already
//!    names as deepest, so this is not a wrong-pair or wrong-triangle
//!    artifact. Building libccd from the same pinned source with
//!    `-DMPR_DIAG` (a scratch, uncommitted instrumentation pass — see below
//!    for why nothing from this build is in the tree) added `fprintf`
//!    tracing to `findPenetr`'s own refinement loop (`mpr.c:317-352`,
//!    unmodified except for the added prints) and ran it, unmodified
//!    otherwise, on all 9 shallow cases' own winning triangles plus 3
//!    sampled deep cases (41, 4, 10) as contrast, using
//!    `visibility_cone_mpr_sweep.rs`'s own new `--dump-case <idx>` flag to
//!    extract each case's *exact* fed geometry (the same bytes the
//!    committed sweep already sends to `mpr_case104`, not a fresh
//!    reconstruction) rather than re-deriving it.
//!
//!    **Result, mechanical and line-cited, not argued from timing:** all
//!    9/9 shallow cases stop at `findPenetr`'s very first iteration
//!    (`iterations=0`), every time, with the portal face's outward normal
//!    (`portalDir`, `mpr.c:491-501`) landing at *exactly* `(0,0,±1)` — the
//!    cylinder's own local Z axis — and all three existing portal vertices
//!    already sitting at `z = length/2`. `__ccdSupport` dispatches to the
//!    cylinder's own support function (`ccd_cyl_t`'s case in
//!    `testsuites/support.c:54-69`), and that function has a documented
//!    degenerate branch: `zdist = sqrt(dir.x² + dir.y²)`, and when
//!    `ccdIsZero(zdist)` — direction exactly parallel to the axis — the
//!    support point returned is `(0, 0, sign(dir.z) * height/2)`
//!    (`support.c:60-62`), the cap's own *center*, not a point on the rim.
//!    Every measured shallow case's own `v4` is exactly that point. Because
//!    `v1`/`v2`/`v3` are already at `z = length/2` too (the portal Phase-1/2
//!    discovery, `discoverPortal`/`refinePortal`, already converged there
//!    before `findPenetr` runs at all) and `v4` cannot go past `z =
//!    length/2` either — the cylinder does not extend further along its own
//!    axis — `portalReachTolerance` (`mpr.c:511-534`) measures the
//!    candidate's *improvement strictly along `dir`* (`dv4 - dv{1,2,3}`,
//!    the dot products with the portal's own outward normal), which comes
//!    out exactly `0.0` in every measured case: the new point is no
//!    "further out" along the frozen axis-aligned `dir` than the three it
//!    already has, even though none of the four have converged in the
//!    plane *perpendicular* to `dir` at all. `0.0 <= mpr_tolerance
//!    (1e-10)`, so refinement reports convergence on the very first check.
//!    `findPenetr` then computes `depth` as the point-to-triangle distance
//!    from the Minkowski-difference origin to that frozen `v1`-`v2`-`v3`
//!    face (`mpr.c:325-330`) — a triangle lying entirely in the
//!    `z = length/2` plane — giving exactly the cylinder's own half-length,
//!    regardless of where within that plane the true minimum-distance
//!    witness actually sits. The 3 sampled deep cases (41, 4, 10) show the
//!    opposite on every point of this chain: `portalDir`'s own `dir` at
//!    iteration 0 is a genuine 3D direction, not axis-locked
//!    (`(0.600,-0.572,0.560)`, `(0.707,0.135,-0.695)`,
//!    `(0.026,-0.654,0.756)`), the tolerance gap starts at `2.3e-2`-`2.6e-2`
//!    and shrinks geometrically over `16`-`24` real iterations before
//!    reaching `1e-10`. This is not a coincidence of one case generalized
//!    again: it is the same mechanism, measured 9 times independently on 9
//!    different triangle/state pairs sharing only the cylinder's own
//!    geometry, with a 3-case contrast sample showing the mechanism does
//!    not fire when `dir` is not axis-locked.
//!
//!    **The half-length is not the only plateau, and the contrast sample
//!    was too small to see the other one.** This block used to end the
//!    sentence above with "and the reported depths (`0.0677`, `0.0748`,
//!    `0.0734`) bear no simple relationship to the cylinder's own
//!    dimensions." Counted over the sweep's own printed readings rather
//!    than three sampled cases, that is false. In a seed-4 run through case
//!    623 (586 lines carrying an `mpr=` reading, 623 carrying an `oracle=`
//!    one):
//!
//!    - `mpr == 7.479200e-2` in **101 of 586** — exactly the cylinder's own
//!      `radius="0.074792"`, to every printed digit, and by far the modal
//!      value (the next most common appears 3 times).
//!    - `oracle == 1.700000e-2` in **381 of 623** — the `length/2` plateau
//!      this block traces, produced by FCL on the oracle side in 61% of
//!      sampled cases, not the rare libccd artifact the "9 of 945" framing
//!      suggests.
//!    - `epa` has no modal value at all: its top bucket appears twice.
//!
//!    Contrast case 4's own reported depth *is* `7.479200e-2` — the radius
//!    — so one of the three cases chosen to show the mechanism not firing
//!    is itself sitting on a dimension plateau, just a different one. What
//!    the instrumented `findPenetr` trace establishes about the axis-locked
//!    `length/2` case stands as measured; what does not follow from it is
//!    that a non-axis-locked `dir` implies a genuine depth. Whether the
//!    radius plateau has its own degenerate branch in `support.c` (the
//!    `zdist` computation's other side) was not traced this round — round
//!    30 traces it directly, after the case-623 paragraph below.
//!
//!    **What is still unexplained after this**, stated plainly rather than
//!    folded into a tidier-sounding claim: *why* the portal Phase-1/2
//!    discovery (`discoverPortal`/`refinePortal`, both upstream of
//!    `findPenetr` and not instrumented this round) converges to an
//!    exactly-axis-locked portal for these particular 9 triangle/cylinder
//!    configurations out of 945, and not the other 936, was not traced.
//!    The mechanism above explains why an axis-locked portal *produces* the
//!    `length/2` plateau and stops refining immediately once it occurs; it
//!    does not explain what makes `dir` land exactly on the axis for only
//!    some near-placement cases. A geometric conjecture (the winning
//!    triangle's own plane happening to be close to perpendicular to the
//!    cylinder axis for these 9) is consistent with the numbers above but
//!    was not measured this round and is not asserted here.
//!
//!    One case (of the 1000) does not fit even the confirmed mechanism:
//!    case 623 has the oracle's own `collision` op reporting `7.479e-2` (a
//!    normal deep MPR value) for the same (cone, link) pair where this
//!    backend's own direct `mpr_case104` call on its own winning triangle
//!    — instrumented the same way as the other 8 — shows the *identical*
//!    `iterations=0`, axis-locked-`dir`, `length/2`-plateau signature. So
//!    case 623's own winning triangle is not a different mechanism; the
//!    oracle's own reported `7.479e-2` must come from a *different*
//!    triangle than the one this backend's own exhaustive search names as
//!    deepest — and this is now measured directly, not inferred. Oracle
//!    image `moveit-rs/oracle:700e7be54cb0a61f` added a settable
//!    `max_contacts_per_pair` on the `collision` op (this crate's own
//!    `doc/oracle-request-collision-max-contacts-per-pair.md` requested it;
//!    the request shipped before this doc could even finish citing it —
//!    see that file's own note). Re-running case 623 with
//!    `visibility_cone_mpr_sweep --dump-contacts 623
//!    --max-contacts-per-pair 32` against that image returns *16* contacts
//!    for the `(cone, fr_caster_r_wheel_link)` pair in one response, not
//!    one: thirteen cluster at `~1.700e-2` (the plateau, matching this
//!    backend's own EPA/MPR reading of its winning triangle to 4
//!    significant figures) and three cluster at `~7.36e-2`-`7.479e-2` (the
//!    oracle's originally-reported deep value, `0.07479153841188203` among
//!    them, matching the `7.479e-2` figure this doc recorded earlier in
//!    this same section to the 4 significant figures that figure was
//!    recorded at). Both this backend's own value and the oracle's own
//!    originally-reported value are present, simultaneously, in FCL's own
//!    per-pair contact list for the identical robot state — confirming,
//!    not just making plausible, that `max_contacts_per_pair = 1`
//!    (`collision_detection/collision_common.hpp:176`) was reporting whichever one of at least
//!    two genuinely-touching triangles FCL's own narrow-phase traversal
//!    happened to place first, and that triangle was not the one this
//!    backend's own exhaustive deepest-first search names as deepest. Case
//!    623 is closed: it is the same 9/9 MPR mechanism as the other 8,
//!    observed through an oracle response that, at the old default, could
//!    only ever show one of the several candidate triangles actually in
//!    contact.
//!
//!    **Round 30 recounts the whole population exactly instead of through
//!    case 623 alone (round 29's own range was `n=624`, `mpr` readings
//!    `n=587`).** Re-run at `--cases 1000` (same seed 4, same oracle image),
//!    `examples/visibility_cone_mpr_sweep.rs` now classifies every reading
//!    against its own case's cylinder dimensions and prints exact counts
//!    (`print_plateau_histogram`, one call per channel; `(n=...)` appended to
//!    the Pearson line) instead of leaving that count to be done by hand from
//!    raw output:
//!
//!    - oracle: `axial(length/2)` in **619 of 1000** (61.9%), `radial
//!      (radius)` in **0 of 1000**, `other` in **381 of 1000** — FCL's own
//!      reported depth is exactly the cylinder's own half-length in a
//!      majority of cases, never exactly the radius.
//!    - epa (this backend's own): `axial`/`radial` in **0 of 1000** each,
//!      `other` in **1000 of 1000** — this backend's own EPA never lands on
//!      either plateau, in any of the 1000 sampled cases, a stronger and
//!      more exact claim than round 29's "no modal value."
//!    - mpr (the 945 mismatches with a real reading): `axial(length/2)` in
//!      **9 of 945** (1.0%, exactly the 9 sign-flip cases above), `radial
//!      (radius)` in **153 of 945** (16.2%), `other` in **783 of 945**
//!      (82.9%).
//!    - `pearson(gap, epa_depth) = 0.1066`, `pearson(gap, triangle_size) =
//!      -0.1823` (`n=945`, re-derived from the full population — round 27
//!      first reported these figures without stating `n`).
//!
//!    The 624-case range round 29 counted by hand is consistent with this
//!    fuller run at the same point (`101 of 586` there vs `101 of 587` /
//!    `381 of 623` vs `381 of 624` here, off by round 29's own 0-vs-1
//!    indexing) — it was not wrong, only smaller than the population the
//!    same tool already covers.
//!
//!    **Is there a second degenerate branch in `support.c` producing the
//!    radial(radius) plateau the way `ccdIsZero(zdist)` produces the
//!    axial(length/2) one? Measured on 20 of the 153 radial-plateau cases
//!    (first 20 by index — 4, 5, 17, 28, 51, 60, 71, 98, 110, 116, 134, 141,
//!    147, 161, 162, 166, 167, 173, 181, 182, including case 4, round 29's
//!    own flagged contrast case) plus a 10-case contrast sample (first 10
//!    `other`-bucket mismatches by index — 0, 1, 2, 6, 7, 8, 9, 10, 11, 12),
//!    instrumented the same `-DMPR_DIAG` way as round 28: no, not the same
//!    mechanism.** All 20/20 radial-plateau traces show genuine
//!    multi-iteration convergence (`iterations` 15 or 16 in every one, never
//!    0), the tolerance gap (`dv4 - dv{1,2,3}`, `portalReachTolerance`'s own
//!    computation, `mpr.c:511-534`) shrinking geometrically across
//!    iterations of the `while(1)` loop at `mpr.c:317` (`portalDir`/
//!    `__ccdSupport` per iteration, `:319-320`) to below `mpr_tolerance`
//!    (case 4: `2.545e-2` at iteration 0, `6.852e-10` at iteration 13,
//!    `1.713e-10` at 14, `4.283e-11` at the iteration-15 stop, crossing
//!    `1e-10` naturally) rather than hitting the exact `0.0` a degenerate
//!    branch returning the same point twice produces — that exact-`0.0`
//!    value never appears in any of the 20 traces. `dir`'s own `z` component
//!    converges toward, but never reaches, exactly `0`: case 4 goes from
//!    `-0.6946` at iteration 0 to `-3.97e-9` at iteration 13 to `-8.30e-10`
//!    at the iteration-15 stop — `ccdSign`'s own `val < CCD_EPS` tie
//!    (`vec3.h:206-219`, the literal `ccdSign(0)==0` case) never fires;
//!    `zdist` (`support.c:58-59`) stays well clear of `ccdIsZero`'s
//!    threshold throughout.
//!
//!    What actually produces the radial(radius) value is a structural
//!    property of `ccdSupport`'s cylinder branch, not a bug in it. A solid
//!    capped cylinder is a disc x interval product domain, so the exact
//!    analytic support point along any direction `d=(dx,dy,dz)` maximizes
//!    the radial and axial terms of the linear functional independently —
//!    `support.c:54-69`'s two branches both compute this correctly, and
//!    *both* set `z = ccdSign(dz) * height/2` (`:62` in the `ccdIsZero
//!    (zdist)` branch, `:68` in the general one): whenever `dz != 0`,
//!    however small, the true maximizing `z` is exactly one rim, never an
//!    intermediate point on the smooth curved belt between the caps. The
//!    belt is *never* a reachable MPR support point except at the exact tie
//!    `dz==0`, and every one of the 20 sampled traces' final portal vertices
//!    sit almost exactly on the two rim circles (radius match to 9-10
//!    significant figures; `z` exactly `+height/2` on some vertices and
//!    exactly `-height/2` on others within the *same* converged triangle —
//!    case 4's final `v1.z=-0.017, v2.z=+0.017, v3.z=-0.017`).
//!
//!    That converged triangle exists because of how the near-placement
//!    generator builds the cone mesh, not because of anything in libccd:
//!    `visibility_cone_mpr_sweep.rs:766` sets `anchor =
//!    cyl_frame.translation.vector` (the touched cylinder's own local-frame
//!    origin in world coordinates) and `:776` builds the cone's own "target"
//!    vertex (mesh vertex index 1) at exactly that point
//!    (`Isometry3::from_parts(anchor.into(), ...)`); transformed into the
//!    cylinder's own local frame for the MPR feed (`:391-392`'s `to_cyl =
//!    cyl_frame.inverse()`), that vertex lands at exactly `(0.0, 0.0, 0.0)` —
//!    confirmed in the dump, not asserted: all 20/20 sampled radial-plateau
//!    cases' fed geometry has one vertex at bit-exact `(0,0,0)`; the 10-case
//!    contrast sample splits 4/10 with that vertex present and 6/10 without.
//!    When the winning triangle has a vertex exactly on the cylinder's own
//!    axis, the true nearest boundary feature for that vertex is symmetric
//!    between the two rims and independent of height — the portal's
//!    refinement genuinely (not degenerately) converges its outward normal
//!    toward purely radial (`dir.z -> 0`) to represent that symmetry using
//!    only the rim points `ccdSupport` can ever return, and the resulting
//!    depth converges to the radius because that genuinely is the distance
//!    from an on-axis point to the cylinder's own curved boundary.
//!
//!    The 4/10 contrast cases (0, 6, 7, 9) that also have the axis vertex
//!    but land in the `other` bucket show a *different*, already-
//!    characterized mechanism instead: `dir` locks to *exactly*
//!    `(0,0,+/-1)` (the discrete `ccdIsZero(zdist)` freeze round 28 traced
//!    for the axial plateau) but *after* 3-5 real iterations rather than at
//!    iteration 0 (cases 0, 6, 7, 9: `iterations` 4, 3, 3, 5), so the frozen
//!    triangle carries real pre-freeze structure and the resulting depth
//!    (`6.286e-2`, `3.677e-2`, `3.196e-2`, `6.585e-2`) is neither plateau —
//!    the same freeze mechanism, a different stopping point, a value that
//!    lands nowhere near either named dimension. The remaining 6/10 contrast
//!    cases (no axis vertex) show a continuum rather than a hard boundary: 4
//!    of them (cases 2, 8, 10, 12) converge over 20-24 real iterations to
//!    depths `0.0703`-`0.0734`, close to but outside the plateau's `1e-6`
//!    relative tolerance around `0.074792` — near-but-not-exact axis
//!    proximity produces a near-but-not-exact radius reading, consistent
//!    with a genuine geometric limit rather than a discrete branch; the
//!    other 2 (cases 1, 11) converge in 2 iterations to depths (`0.0189`,
//!    `0.0337`) unrelated to either dimension.
//!
//!    This mechanism is distinct in kind from the axial(length/2) one: no
//!    degenerate branch fires, no iteration-0 freeze, no exact tie — it is
//!    `ccdSupport`'s own exact, correct, rim-only range for a cylinder
//!    support function, combined with the generator's own construction
//!    always placing one cone vertex exactly on the touched cylinder's own
//!    axis, converging MPR toward a real but misleading number: that one
//!    vertex's own radial escape distance, not the true minimum-translation
//!    depth of the whole triangle, which this backend's own EPA (operating
//!    on all three vertices together) reports correctly and almost always
//!    smaller — see the 853/945 "MPR deeper" figure above.
//!
//!    **Deviation 6(b), re-scoped to what the counts actually show.** This
//!    block's framing through round 29 was "libccd's MPR reads deeper than
//!    this backend's own EPA for the `visibility_cone` pair, by construction
//!    except for 9 shallow outliers." That framing survives for the
//!    majority of the 945-case mismatch population (853/945 deeper, the
//!    round-16/17/21 magnitude-bias mechanism), but it is not a complete
//!    description of the *plateau* fraction: on the oracle side, 619 of
//!    every 1000 `visibility_cone` cases (61.9%) report a depth that is
//!    exactly the touched cylinder's own `length/2`, not a real narrow-phase
//!    penetration measurement, and on the MPR side 162 of 945 mismatches (9
//!    axial + 153 radial, 17.1%) plateau on one of the cylinder's own two
//!    dimensions rather than converging to a real depth — this backend's own
//!    EPA never plateaus on either dimension, in 1000 sampled cases. So for
//!    a majority of oracle readings and a material minority of MPR readings,
//!    this is not "two algorithms disagreeing about one shape pair's
//!    depth" — it is "FCL's own narrow-phase and libccd's own
//!    `ccdMPRPenetration`, independently, sometimes degenerate to reporting
//!    one of the cylinder's own dimensions instead of a depth, on different
//!    fractions of the population and via mechanisms that are not the same
//!    mechanism (FCL's own internal cause was not traced this round — FCL's
//!    own source is out of this crate's scope to instrument), while this
//!    backend's own EPA is the one of the three channels that never does."
//!    The "by construction, MPR deeper" claim is not false; it is
//!    incomplete: true for the 82.9%-of-mismatches `other` fraction where a
//!    real MPR convergence happened, silent about the ~17% where neither
//!    side computed a depth at all.
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
//!    the robot-vs-world side: [`CollisionEnv::distance_self`]/[`CollisionEnv::distance_robot`] both
//!    call the same [`accumulate_distance`] over the same per-part
//!    [`fn@query::contact`] call and threshold logic, differing only in which
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
//!     [`parry3d_f64::shape::Compound`], not a native octree.** `parry3d-f64` has no equivalent of
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
//!   (`collision_detection_fcl/collision_common.cpp:471-560`), so [`accumulate_distance`] does not
//!   apply this rule.
//!
//! "Same object" pairs (upstream's `cd1->sameObject(*cd2)`, the first check
//! in both callbacks) need no code here: [`self_pairs`]/[`cross_pairs`]
//! never produce `(x, x)`, and every [`PosedBody`] is already one body's
//! *entire* geometry combined into a single [`parry3d_f64::shape::Compound`] (this module's own
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
    Shape as ParryShape, ShapeType, SharedShape, TriMesh, Triangle as ParryTriangle,
};

use crate::common::{
    AttachedBodyGeometry, BodyType, CollisionDistance, CollisionRequest, CollisionResult, Contact,
    ContactData, CostSource, DistanceRequest, DistanceRequestType, DistanceResult,
    DistanceResultsData,
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
/// still finds any pair at least as penetrating, which is all a
/// touching-or-penetrating-only prediction can be asked to do; this is not,
/// and does not need to be, the same value [`accumulate_collision`] passes
/// -- that function has no running accumulator to protect and always queries
/// at a literal `0.0`, while this one's argument tracks whatever the current
/// sweep has found so far. The two functions' *tie verdict* is unified by
/// [`touches_at_tie`], which both now call on the `Contact::dist` this
/// clamp only shapes the search radius for -- not by their prediction
/// arguments matching, which they do not and are not meant to.
///
/// Neither clamp is dead code: every [`DistanceRequestType`] variant's
/// threshold traces back to [`DistanceRequest::distance_threshold`]'s
/// `f64::MAX` default at least once (`Global` and `Single` on a pair whose
/// running bound/bucket is not yet set; `Limited`/`All` on every pair, since
/// they read it directly), so the upper clamp fires on every real query this
/// crate has ever run (`rg -n distance_threshold` outside this file finds no
/// caller that overrides the default). The lower clamp fires whenever
/// `Global`'s running minimum, or `Single`'s per-key bucket, has already
/// gone negative from an earlier pair in the same sweep -- ordinary, not
/// hypothetical, the moment a query touches more than one penetrating pair.
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
/// [`CollisionEnv`] call prunes it), not one accumulated
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
/// [`CollisionEnv`] query — do not rebuild the same
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
/// shape or an attached body's (never a world object's — see the module doc,
/// deviation 2).
///
/// A [`Shape::Mesh`] still carrying `vertex_normals: None` has them computed
/// on the clone first, because [`Shape::scale_and_padd`] needs them for the
/// per-vertex padding direction and errors without them. Upstream never
/// reaches `Mesh::scaleAndPadd` without them either, but it establishes that
/// differently: every `geometric_shapes` mesh-creation entry point ends with
/// `computeTriangleNormals(); computeVertexNormals();`
/// (`third_party/geometric_shapes/src/mesh_operations.cpp:124-125`, `:200-201`,
/// `:436-437`, and `shape_operations.cpp:534-535`), so a mesh that came from a
/// `shape_msgs::Mesh` — which is how an attached body's geometry arrives —
/// already has them. This port computes them in
/// `moveit_geometry::stl::mesh_from_bytes` alone, which covers
/// [`LinkModel::shapes`] and nothing else; an [`AttachedBodyGeometry`] carrying
/// a mesh the caller built with `moveit_geometry::Mesh::new` is a supported
/// public input that arrives here with `None`. Computing them here is what
/// makes the `expect` below true for *both* callers rather than only the link
/// one.
///
/// # Panics
///
/// Never: every non-mesh shape variant's dimensions are already validated
/// non-negative at construction, so scaling by a validated-positive
/// [`LinkPaddingScale::link_scale`] and adding a validated-non-negative
/// [`LinkPaddingScale::link_padding`] can never make them negative, and
/// [`Shape::scale_and_padd`]'s one documented mesh failure mode
/// (`vertex_normals: None`) has just been removed above.
fn scaled_padded_shape(shape: &Shape, scale: f64, padding: f64) -> Shape {
    let mut shape = shape.clone();
    if let Shape::Mesh(mesh) = &mut shape {
        if mesh.vertex_normals.is_none() {
            mesh.compute_vertex_normals();
        }
    }
    shape.scale_and_padd(scale, padding).expect(
        "every dimension is non-negative by construction and any mesh has had its vertex \
         normals computed just above, so scale_and_padd cannot fail here",
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

/// `CollisionData::enableGroup`/`DistanceRequest::enableGroup`
/// (`collision_detection_fcl/collision_common.cpp:1012-1022`, `collision_detection/collision_common.hpp:206-216`): the
/// set of link names a `group_name` resolves to, or `None` for "no active
/// group" (module doc, deviation 1) — either `group_name` is `None`, or it
/// names a group the model does not have, matching upstream's
/// `hasJointModelGroup` guard around both `enableGroup` overloads.
fn active_group_links<'m>(
    state: &Posed<'_, 'm>,
    group_name: Option<&str>,
) -> Option<BTreeSet<&'m str>> {
    let group = state.model().joint_model_group(group_name?).ok()?;
    Some(
        group
            .updated_link_names()
            .iter()
            .map(String::as_str)
            .collect(),
    )
}

/// The link a body counts as for group-filtering purposes: its own name for
/// a robot link, its attached link for an attached body, and no link at all
/// for a world object — the same three-way split `collisionCallback`'s
/// `cd1->type ==`/`cd2->type ==` ternary makes to get `l1`/`l2`
/// (`collision_detection_fcl/collision_common.cpp:80-87`).
fn robot_link_name(body: &PosedBody) -> Option<&str> {
    match body.body_type {
        BodyType::RobotLink => Some(&body.name),
        BodyType::RobotAttached => body.attached_link.as_deref(),
        BodyType::WorldObject => None,
    }
}

/// `collisionCallback`/`distanceCallback`'s active-group predicate
/// (`collision_detection_fcl/collision_common.cpp:79-94`/`482-500`), inverted from "skip when
/// neither side is active" to "keep when either side is": a world object
/// never resolves to a link of its own, so a robot-vs-world pair is kept
/// exactly when the robot link is active, and a self-pair is kept when
/// *either* link is (matching upstream's OR, not an AND-of-both-links
/// filter a fresh implementation might guess at).
fn pair_in_active_group(active: &BTreeSet<&str>, a: &PosedBody, b: &PosedBody) -> bool {
    let side_active = |body: &PosedBody| robot_link_name(body).is_some_and(|l| active.contains(l));
    side_active(a) || side_active(b)
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
/// The granularity of that cost is not decided by the traversal node alone
/// — it is gated by `fcl::CollisionRequest`'s `use_approximate_cost` flag,
/// whose constructor default is `true`
/// (`fcl/include/fcl/narrowphase/collision_request.h:101`, byte-identical
/// between FCL tag `0.7.0` and the checkout's current HEAD for this file)
/// and which `moveit_core` never overrides: all three
/// `fcl::CollisionRequestd(...)` call sites pass exactly 4 positional
/// arguments, never a 5th
/// (`collision_detection_fcl/src/collision_common.cpp:227,303,364`, moveit2
/// pin `e017c91e`). So every request this crate's oracle actually issues has
/// `use_approximate_cost == true`. `collision_func_matrix-inl.h` reads that
/// flag in exactly four places — OcTree↔BVH (`:184`), BVH↔OcTree (`:237`),
/// `BVHShapeCollider::collide` (`:330`), and `orientedBVHShapeCollide`
/// (`:391`) — and all four share one shape: a cost-disabled traversal
/// computes contacts first, then a *single* `Box` built from the mesh
/// side's BV(0) root (`constructBox(obj->getBV(0).bv, ...)`) stands in for
/// the whole mesh in a second, cost-only pass against the other side.
/// `MeshShapeCollisionTraversalNode::leafTesting`'s per-triangle
/// `addCostSource` (`mesh_shape_collision_traversal_node-inl.h:112,123`) is
/// unreached code under moveit — it only runs when `use_approximate_cost ==
/// false`, which `moveit_core` never requests. Expires (§153.1) the moment
/// any `fcl::CollisionRequestd(...)` call in `moveit_core` gains a 5th
/// positional argument — anchor: `rg -n 'CollisionRequestd\(' moveit_core/`.
///
/// Three shape-kind combinations, matching what upstream's dispatch actually
/// does once `use_approximate_cost == true` is accounted for:
/// - **mesh vs mesh** (`mesh_collision_traversal_node-inl.h`, reached via
///   `BVHCollide`/`orientedMeshCollide`, which never read
///   `use_approximate_cost` at all): one [`CostSource`] per pair of
///   triangles confirmed to intersect — [`mesh_mesh_cost_sources`].
/// - **mesh vs anything else** (`BVHShapeCollider`/`orientedBVHShapeCollide`):
///   one [`CostSource`] — the world-space axis-aligned bound of the mesh's
///   own *oriented* root box overlapped against the other side's
///   whole-shape world AABB — [`mesh_shape_cost_sources`]. This is not the
///   mesh's plain `Bvh` root AABB (`TriMesh::aabb`/`compute_aabb`): FCL's
///   `constructBox` reduces the always-`OBBRSS` root bound to its oriented
///   `OBB` component, discarding the RSS radius
///   (`fcl/include/fcl/geometry/shape/utility-inl.h:1083-1088`), so this
///   port fits and axis-aligns that same oriented box instead —
///   [`mesh_world_obb_aabb`], see its own doc and
///   [`mesh_shape_cost_sources`]'s for the fitting algorithm and its
///   measured accuracy.
/// - **neither is a mesh** (`shape_collision_traversal_node-inl.h`, no
///   `use_approximate_cost` branch either): at most one, over both shapes'
///   own whole-shape AABBs from [`ParryShape::compute_aabb`] — the same
///   call already named as the non-mesh half of this fill-in by
///   `moveit-scene`'s own doc audit.
///
/// The mesh-mesh case still does its candidate search BVH-pruned
/// (`TriMesh::bvh`/[`parry3d_f64::partitioning::Bvh::intersect_aabb`]) and
/// confirms with an exact geometric test ([`query::intersection_test`])
/// before emitting anything, matching the one traversal-node path it
/// actually takes upstream.
///
/// A [`parry3d_f64::shape::Compound`] built from an octree
/// (deviation 11) is not a [`TriMesh`], so it always takes the
/// whole-shape-AABB path — one cost source per colliding octree pair, not
/// per occupied leaf. This is a real deviation, and a stricter one than
/// previously documented here: `octree_solver-inl.h` never reads
/// `use_approximate_cost` anywhere in the file (confirmed absent by reading
/// it in full — not merely `rg`-absent), so FCL's octree-side leaf
/// recursion (`OcTreeSolver::OcTreeShapeIntersectRecurse`, `:332-354`) calls
/// `addCostSource` once per occupied leaf unconditionally. That holds even
/// in the mesh↔octree case: once the *mesh* side has been collapsed to one
/// box above, the *octree* side still walks down to its individual occupied
/// leaves against that box, because the second, cost-only pass in
/// `OcTreeBVHCollide`/`BVHOcTreeCollide`
/// (`collision_func_matrix-inl.h:189-206`/`242-259`) is itself
/// `OcTreeShapeCollide`/`ShapeOcTreeCollide` — the same per-leaf octree
/// solver, called with the mesh's box as its shape argument, not a
/// whole-octree AABB test. Once this port's occupied leaves are flattened
/// into one [`parry3d_f64::shape::Compound`] at construction time, no finer-than-the-compound
/// granularity survives to cost separately — this closes the measurement
/// §171.4 left open, confirming the deviation rather than retracting it.
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

/// At most one [`CostSource`]: the world-space AABB of `mesh`'s own root
/// *oriented* bounding box ([`mesh_world_obb_aabb`]) overlapped against
/// `other`'s whole-shape world AABB — matching
/// `BVHShapeCollider`/`orientedBVHShapeCollide`'s second, cost-only pass
/// (`collision_func_matrix-inl.h:330-355`), which replaces the whole mesh
/// with `constructBox(obj1->getBV(0).bv, ...)` before colliding it against
/// `other`, then bounds that oriented box axis-aligned before intersecting
/// (`shape_collision_traversal_node-inl.h:116-121`, `computeBV`/`overlap`).
/// The `cost_density` this carries (`cost_source_from_aabb`'s constant
/// `1.0`) is already the mesh's own: every `CollisionGeometry` in this
/// crate's oracle-reachable population carries that same default, so there
/// is no second value to pick between.
///
/// An earlier version of this function used `mesh.aabb(mesh_pose)` — the
/// mesh's plain, axis-aligned `Bvh` root — reasoning that it was at least
/// the right *box*, if not necessarily the right *bounds*: id 8's single
/// pair matched the oracle to `1e-13`, but ids 2/3/4/6's nine mesh-vs-floor
/// pairs landed 0.003-0.07m off, real and not noise. That framing was
/// itself wrong, not just imprecise: `moveit_core` always instantiates
/// `fcl::BVHModel<fcl::OBBRSSd>` (`collision_detection_fcl/collision_common.cpp:949-1006`, every
/// `createCollisionGeometry` call), so `getBV(0).bv` is never an AABB at
/// all — [`mesh_world_obb_aabb`] fits the *oriented* box FCL actually
/// builds, and id 8's coincidental ULP-level match was a mesh whose
/// principal axes happen to already line up with its local frame, not
/// evidence the AABB approach was structurally right.
///
/// [`mesh_world_obb_aabb`] does not reproduce FCL's OBB fit bit-for-bit —
/// both sides fit via a covariance-matrix eigendecomposition
/// (`parry3d_f64::utils::cov`+`nalgebra::SymmetricEigen` here,
/// `getCovariance`+`eigen_old` in `fcl/include/fcl/math/geometry-inl.h`),
/// two independent implementations of the same numerically-approximate
/// technique with no shared tie-breaking convention for near-symmetric
/// covariance matrices. What closes the gap is matching FCL's *input* to
/// that decomposition: `getCovariance` sums over each triangle's three
/// vertices individually (`geometry-inl.h:1349-1379`), not the mesh's
/// deduplicated vertex list, so a vertex shared by five triangles is
/// weighted five times — [`mesh_world_obb_aabb`] reproduces that by
/// flattening `mesh.triangles()` into one corner per `(triangle, vertex)`
/// pair before fitting, rather than calling `mesh.vertices()` directly.
///
/// Measured, not assumed, against every mesh-vs-shape pair in
/// `moveit-scene/tests/fixtures/panda_{cost_sources,path_cost_sources}_response.json`
/// (oracle `moveit-rs/oracle:3537df47121b8c7f`, ids 2-6/8 state-op, ids
/// 3-6 path-op — 22 total non-empty pairs across both fixtures, borrowing
/// `moveit-scene`'s own committed fixture read-only via a temporary,
/// fully-reverted debug instrumentation of its ignored parity tests, not
/// this crate's own test data): every returned box's `aabb_min`/`aabb_max`
/// matches its oracle counterpart to `9e-14`..`4e-13` per component — same
/// order of magnitude as id 1's mesh-mesh ULP noise floor
/// (`cost_sources_parity.rs`'s own `COST_SOURCE_EPSILON = 1e-9` documents
/// `1.11e-16` there), not the 0.003-0.07m the AABB approach left open. The
/// state-op survivor *counts* already matched post-§171 (this function's
/// granularity fix alone); the path-op survivor counts did not (id 3: 1
/// actual vs 5 expected, id 4: 1 vs 4, id 5: 1 vs 0, id 6: 5 vs 6, measured
/// this round before this fix) — `remove_cost_sources`'/`remove_overlapping`'s
/// (`crates/moveit-collision/src/tools.rs`) overlap-fraction threshold
/// comparisons are sensitive to exact box bounds, so the AABB approach's
/// bounds error was cascading into wrong split/removal decisions even
/// though those functions are themselves byte-faithful to
/// `collision_tools.cpp:188-271` (compared directly, line by line, this
/// round). With this fix, every one of those four path-op ids now matches
/// the oracle's survivor count exactly.
fn mesh_shape_cost_sources(
    mesh_pose: &Pose,
    mesh: &TriMesh,
    other_pose: &Pose,
    other: &dyn ParryShape,
) -> Vec<CostSource> {
    let mesh_world_aabb = mesh_world_obb_aabb(mesh_pose, mesh);
    let other_world_aabb = other.compute_aabb(other_pose);
    mesh_world_aabb
        .intersection(&other_world_aabb)
        .map(|overlap| vec![cost_source_from_aabb(overlap)])
        .unwrap_or_default()
}

/// The world-space axis-aligned bound of `mesh`'s own oriented root
/// bounding box, matching `constructBox(obj->getBV(0).bv, tf, ...)` +
/// `computeBV` for `moveit_core`'s always-`OBBRSS` `BVHModel`
/// (`fcl/include/fcl/geometry/shape/utility-inl.h:1083-1088`: `constructBox`
/// for `OBBRSS` uses only its inner `obb` field, ignoring the RSS radius
/// entirely). See [`mesh_shape_cost_sources`]'s doc for the fitting
/// algorithm and its measured accuracy.
fn mesh_world_obb_aabb(mesh_pose: &Pose, mesh: &TriMesh) -> Aabb {
    let corners: Vec<ParryVector> = mesh.triangles().flat_map(|t| [t.a, t.b, t.c]).collect();
    let (obb_pose, obb_cuboid) = parry3d_f64::utils::obb(&corners);
    let world_obb_pose = mesh_pose * obb_pose;
    obb_cuboid.compute_aabb(&world_obb_pose)
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

/// The key both of this backend's per-pair maps are filed under
/// ([`CollisionResult::contacts`]'s `by_pair` and [`DistanceResult::distances`]),
/// **lexicographically smaller name first**.
///
/// Upstream sorts at both sites and does it inline:
/// `cd1->getID() < cd2->getID() ? make_pair(cd1->getID(), cd2->getID()) :
/// make_pair(cd2->getID(), cd1->getID())` -- `collision_detection_fcl/collision_common.cpp:240-242` in
/// `collisionCallback` and `:564-567` in `distanceCallback`, at `e017c91ee`.
/// The ordering is not cosmetic: these are `BTreeMap`s (upstream `std::map`s),
/// so it decides both what a caller must look a pair up by and what
/// `contacts.begin()` yields -- upstream's own `ContactReporting` reads that
/// first element.
///
/// A single constructor rather than the sort written at each site: the two
/// sites are what let this backend file `distance_robot`'s pairs under
/// `(robot_link, world_object)` -- iteration order, since `cross_pairs` puts
/// the robot first -- while upstream filed them the other way for every world
/// object whose name sorts before the link's.
///
/// Note the distance-field backend is *not* part of this family and must not
/// be "fixed" to match: upstream's `collision_env_distance_field.cpp:329`,
/// `:621` and `:1618` file contacts under `(con.body_name_1, con.body_name_2)`
/// unsorted, and `crates/moveit-distance-field` reproduces that.
fn pair_key(a: &str, b: &str) -> (String, String) {
    if a < b {
        (a.to_string(), b.to_string())
    } else {
        (b.to_string(), a.to_string())
    }
}

/// The workspace-buildable shape kinds fcl's tangency dispatch splits on
/// (`fcl_tangency_table`'s own module doc: "Shape intersect algorithms not
/// using libccd"). `Mesh` is deliberately absent — fcl maps it to a
/// `BVHModel` traversal, a third path that is neither a libccd
/// specialisation nor generic libccd MPR, so [`is_mesh_pair`] carries its
/// rule separately rather than through this table.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TangencyKind {
    Box = 0,
    Sphere = 1,
    Cylinder = 2,
    Cone = 3,
}

/// Classifies a `parry` shape into the [`TangencyKind`]
/// `fcl_tangency_table::SPECIALISED` is indexed by. `None` for a kind the
/// table has no measurement for (`HalfSpace`/`Compound`, i.e. this crate's
/// `Plane` and `OcTree`) — [`fcl_tangency_verdict`]'s callers keep their own
/// pre-existing behaviour for those rather than guessing at one.
fn tangency_kind(shape: &dyn parry3d_f64::shape::Shape) -> Option<TangencyKind> {
    match shape.shape_type() {
        ShapeType::Cuboid => Some(TangencyKind::Box),
        ShapeType::Ball => Some(TangencyKind::Sphere),
        ShapeType::Cylinder => Some(TangencyKind::Cylinder),
        ShapeType::Cone => Some(TangencyKind::Cone),
        _ => None,
    }
}

/// Whether either shape in the pair is a mesh. Consulted only from
/// [`touches_at_tie`]'s rounding band, alongside [`fcl_tangency_verdict`] —
/// not in [`accumulate_collision`]'s `None`-but-touching branch, whose
/// [`query::intersection_test`] fallback exists specifically for a
/// *specialised closed-form* routine excluding a boundary its own geometric
/// test admits (`contact_ball_ball`'s strict `<`; see
/// `parry_boolean_queries_disagree_in_both_directions_at_the_tie`). Mesh
/// pairs go through `contact_composite_shape_shape`/
/// `contact_shape_composite_shape` — the same generic per-triangle traversal
/// as any other non-ball pair, not a closed-form specialisation — and
/// already return `Some` at exact tangency today
/// (`exact_tangency_is_decided_per_shape_pair.rs`'s `mesh` row), so
/// extending the `None` branch to mesh would add an `intersection_test` call
/// to every non-touching mesh pair in a scene for a case that, unlike
/// `sphere x sphere`, has no measurement showing it is ever reached.
fn is_mesh_pair(a: &dyn parry3d_f64::shape::Shape, b: &dyn parry3d_f64::shape::Shape) -> bool {
    a.shape_type() == ShapeType::TriMesh || b.shape_type() == ShapeType::TriMesh
}

/// fcl's tangency-dispatch table (`fcl_tangency_table`, generated from the
/// pinned oracle image's own registration macros — see
/// `tools/ci/verify-fcl-tangency-dispatch.sh`), for a shape pair that
/// classifies into [`TangencyKind`] on both sides. `None` when either shape
/// doesn't — the caller falls back to its own pre-existing behaviour, mesh
/// included ([`is_mesh_pair`] is checked separately, ahead of this).
fn fcl_tangency_verdict(
    a: &dyn parry3d_f64::shape::Shape,
    b: &dyn parry3d_f64::shape::Shape,
) -> Option<bool> {
    Some(
        crate::fcl_tangency_table::SPECIALISED[tangency_kind(a)? as usize]
            [tangency_kind(b)? as usize],
    )
}

/// The magnitude of the largest world coordinate this pair's narrow phase
/// works in: the larger of the two shapes' AABB half-extent (the largest
/// single component of either half-extent vector, so an anisotropic shape's
/// scale follows its longest axis rather than its shortest), and the larger
/// of the two AABB centres' distance from the origin along any one axis. The
/// same `compute_aabb` call [`cost_sources_for_part_pair`] already makes for
/// its own overlap test, reused here rather than a second, independent notion
/// of "how big is this pair" ([`touches_at_tie`]'s own doc has the reason it
/// needs one at all).
///
/// The centre term is not decoration, and the reason is not that GJK's
/// rounding grows with distance — it does not. Placed on exactly
/// representable coordinates, a pair measures the same `dist` at every
/// offset. What grows is the error in *placing* the tangency at all:
/// `100.0 + 0.1` is not a representable sum, so a pair that is tangent in
/// exact arithmetic is stored up to half a coordinate ULP away from
/// tangency. At 100 m that is `1.4e-14`, while the spacing of the shapes'
/// own 5 cm half-extents is `1.1e-17` — a factor of 1280. A separation
/// below the spacing of the coordinates a pair is expressed in cannot be
/// told from an exact tie by any arithmetic on those poses, so the band
/// that decides which comparisons *are* ties has to be measured in that
/// spacing, not in the shapes' own.
///
/// [`TIE_ROUNDING_MARGIN`]'s sweep never separated the two, because its
/// `pair_at` puts the lower shape at `translation(0, 0, 0)` — world
/// magnitude and shape size are the same number in every case it measures,
/// so a half-extent-only scale fit its data exactly and was wrong off it.
/// Measured with 5 cm shapes, a tangency placed 10 m out lands `-3.6e-16`
/// from zero, which is `0.20` coordinate ULP but `32.5`
/// `EPS * half_extent` units; 100 m out it is `-5.7e-15`, `0.40` ULP but
/// `512` units. (Pairs differ in the last digit — the curved-cap ones read
/// `-3.7e-16` and `33.8` at 10 m — so these are the `box`/`cylinder` row,
/// the one the regression test reports first.) Both are outside the `16.0`
/// band, so `touches_at_tie` fell
/// through to `dist`'s sign, which rounds negative and answers "touching"
/// for every pair the dispatch table calls apart — all ten of the table's
/// `false` cells, at both offsets. With the centre folded in the same
/// cases read `0.16` and `0.26` units and reach the table, which is the
/// answer they have at the origin.
/// `an_exact_tie_is_decided_by_the_dispatch_table_however_far_from_the_origin`
/// is the regression case.
fn tie_scale(
    a_pose: &Pose,
    a_shape: &dyn ParryShape,
    b_pose: &Pose,
    b_shape: &dyn ParryShape,
) -> f64 {
    let half_extent_max = |aabb: &Aabb| aabb.half_extents().max_element();
    let centre_max = |aabb: &Aabb| {
        let c = aabb.center();
        c.x.abs().max(c.y.abs()).max(c.z.abs())
    };
    let reach = |aabb: &Aabb| half_extent_max(aabb).max(centre_max(aabb));
    reach(&a_shape.compute_aabb(a_pose)).max(reach(&b_shape.compute_aabb(b_pose)))
}

/// How many multiples of `f64::EPSILON * `[`tie_scale`]` a measured `dist`
/// may sit away from zero and still be [`touches_at_tie`]'s "exact tie"
/// case, rather than a real signed distance this crate can trust the sign
/// of directly.
///
/// Measured, not chosen — `tie_rounding_margin_clears_measured_ties_and_does_not_swallow_a_real_delta`
/// (below) is the derivation, run fresh on every test invocation rather than
/// trusted as a one-time reading: the largest rounding this crate has found
/// at a genuine exact tie, across every [`TangencyKind`] pair and five
/// orders of magnitude of scale, is `1.92` `EPS * scale` units
/// (`cylinder x cylinder`); `16.0` sits comfortably above that without
/// reaching anywhere close to `query::contact`'s own reach before it starts
/// answering `None` (on the order of `1e6`-`1e7` `EPS * scale`, matching
/// `parry3d_f64::query::gjk::gjk::eps_tol`'s `10.0 * f64::EPSILON`
/// convergence tolerance) — the pinning test's own doc has the two-sided
/// reasoning and the numbers.
const TIE_ROUNDING_MARGIN: f64 = 16.0;

/// The single rule [`accumulate_collision`] and [`accumulate_distance`] both
/// consult for "does this pair, at this measured `dist`, count as touching":
/// within [`TIE_ROUNDING_MARGIN`] `* f64::EPSILON * `[`tie_scale`]` of zero,
/// `dist`'s own sign is GJK's rounding on what is geometrically an exact
/// tie, not a real answer, so [`is_mesh_pair`]/[`fcl_tangency_verdict`]
/// decide instead; outside that band, `dist`'s sign is trusted directly.
///
/// This replaces two independent rules that happened to agree until they
/// did not: `accumulate_collision` special-cased literal `dist == 0.0`
/// (unreachable for most of the pairs that are geometrically tangent — GJK's
/// own iteration rounds a true `0.0` to a nonzero value far more often than
/// not, see the constant's own doc) and treated every other `Some` as a
/// touch regardless of sign; `accumulate_distance` used `dist <= 0.0`
/// directly, with no tie case at all. Two owners of the same question is an
/// edge factory by construction — this crate's own `octree_world_collision_
/// parity.rs` case 4 and `exact_tangency_boundary.rs`'s prbt fixture already
/// disagree with each other on which side of `0.0` an exactly-touching pair
/// lands, so a rule keyed on the literal bit pattern was never going to
/// generalise. One helper, one answer, called from both places.
fn touches_at_tie(
    dist: f64,
    a_pose: &Pose,
    a_shape: &dyn ParryShape,
    b_pose: &Pose,
    b_shape: &dyn ParryShape,
) -> bool {
    let scale = tie_scale(a_pose, a_shape, b_pose, b_shape);
    if dist.abs() <= TIE_ROUNDING_MARGIN * f64::EPSILON * scale {
        is_mesh_pair(a_shape, b_shape) || fcl_tangency_verdict(a_shape, b_shape).unwrap_or(true)
    } else {
        dist <= 0.0
    }
}

/// The outcome of asking whether a part pair touches, keeping "parry has no
/// dispatch algorithm for this shape-kind pair" ([`query::contact`]'s own
/// `Err(Unsupported)`) distinct from "parry computed a definite answer: they
/// do not touch" (`Ok(None)`). Every call site used to collapse the two --
/// `if let Ok(Some(_)) = query::contact(...)` (and, in `accumulate_distance`,
/// the equivalent `let Ok(Some(_)) = ... else { continue }`) treats a query
/// parry could not even attempt exactly like one it ran and found nothing,
/// silently reporting "not colliding" for a pair whose verdict was never
/// computed.
///
/// `default_query_dispatcher.rs:305-359` (`parry3d-f64` 0.30.0) has no arm
/// at all for `HalfSpace` paired with `HalfSpace`: neither side implements
/// `as_support_map` or `as_composite_shape`, so every arm (ball, halfspace x
/// support-map, ball x convex, support-map x support-map, composite x
/// anything) falls through to `Err(Unsupported)`. That is the only pair
/// among this crate's seven constructible `ShapeType`s (`Ball`, `Cuboid`,
/// `Cylinder`, `Cone`, `HalfSpace`, `TriMesh`, `Compound`) with no dispatch
/// arm at all -- verified by reading every arm each of the 49 ordered pairs
/// over those seven kinds can reach, not assumed from the one cited pair.
#[derive(Debug)]
enum PartContactOutcome {
    /// A real contact, at or within the query's `prediction` distance.
    Touching(ParryContact),
    /// parry computed a definite answer: this pair does not touch.
    NotTouching,
    /// parry has no dispatch arm for this shape-kind pair. Every caller
    /// must name this arm explicitly -- seeing this type's own doc for why
    /// an `Option`/`bool` collapse is exactly the defect this replaces.
    Unsupported,
}

/// [`query::intersection_test`]'s equivalent of [`PartContactOutcome`], for
/// [`accumulate_collision`]'s `query::contact`-missed rescue branch. See
/// [`part_intersects`]'s own doc for why that call site is the only one.
enum PartIntersectOutcome {
    Touching,
    NotTouching,
    Unsupported,
}

/// The one place [`query::contact`] is called from this file's collision and
/// distance accumulation ([`accumulate_collision`], [`accumulate_distance`])
/// -- collapsing `Result<Option<Contact>, Unsupported>` into
/// [`PartContactOutcome`] here, once, so a new call site cannot reintroduce
/// the silent swallow that type's own doc describes.
///
/// `mesh_mesh_cost_sources`'s own `query::intersection_test` call is
/// deliberately not routed through the equivalent [`part_intersects`]: it
/// only gates whether a `CostSource` is appended for the optional
/// `request.cost` diagnostic, never `collision`, and its own pair (`Triangle`
/// x `Triangle`, always support-mapped on both sides) cannot itself return
/// `Err` (`default_query_dispatcher.rs:218-221`, the generic
/// support-map-support-map arm) -- confirmed, not assumed exempt because it
/// looked unrelated.
fn part_contact(
    a_pose: &Pose,
    a_shape: &dyn ParryShape,
    b_pose: &Pose,
    b_shape: &dyn ParryShape,
    prediction: f64,
) -> PartContactOutcome {
    match query::contact(a_pose, a_shape, b_pose, b_shape, prediction) {
        Ok(Some(contact)) => PartContactOutcome::Touching(contact),
        Ok(None) => PartContactOutcome::NotTouching,
        Err(_) => PartContactOutcome::Unsupported,
    }
}

/// [`part_contact`]'s equivalent for [`query::intersection_test`]. Only ever
/// called from [`accumulate_collision`]'s rescue branch, and only after
/// [`fcl_tangency_verdict`] has already restricted the pair to `{Box,
/// Sphere, Cylinder, Cone}` on both sides (`Some(true)` is impossible
/// otherwise) -- none of which can return `Err` there
/// (`default_query_dispatcher.rs:177-244`, every arm those four kinds reach
/// is `Ok`). Routed through the same three-way outcome anyway: that
/// restriction holds only as long as [`tangency_kind`] classifies exactly
/// those four kinds, and a `.unwrap_or(false)` here would silently swallow
/// the day that stops being true -- exactly as it did for `HalfSpace` x
/// `HalfSpace` before [`PartContactOutcome`] existed.
fn part_intersects(
    a_pose: &Pose,
    a_shape: &dyn ParryShape,
    b_pose: &Pose,
    b_shape: &dyn ParryShape,
) -> PartIntersectOutcome {
    match query::intersection_test(a_pose, a_shape, b_pose, b_shape) {
        Ok(true) => PartIntersectOutcome::Touching,
        Ok(false) => PartIntersectOutcome::NotTouching,
        Err(_) => PartIntersectOutcome::Unsupported,
    }
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
///   so only touching/penetrating pairs yield `Some`), [`touches_at_tie`]
///   says the pair touches, and [`AllowedCollision::Conditional`]: the
///   predicate decides — rejected (`false`) is a collision, accepted
///   (`true`) is silently not.
/// - Real contact found, `touches_at_tie` says touching, and
///   [`AllowedCollision::Never`] or no entry: unconditionally a collision.
/// - Real contact found but `touches_at_tie` says not touching (a real
///   clearance `query::contact`'s own margin still reported, or a rounding
///   tie the dispatch table says this shape pair does not collide at):
///   no collision, regardless of the ACM entry.
/// - No contact, but the pair classifies into [`TangencyKind`] as one the
///   table says collides, is not [`AllowedCollision::Conditional`], and
///   [`query::intersection_test`] independently confirms they touch:
///   unconditionally a collision, with no `Contact` recorded — see the
///   branch's own comment below for why.
/// - No contact, and none of the above: not a collision.
/// - [`PartContactOutcome::Unsupported`] (parry has no dispatch arm for this
///   shape-kind pair at all — today, only `HalfSpace` x `HalfSpace`, see that
///   type's own doc): a collision, unless [`AllowedCollision::Conditional`]
///   (same exclusion as the rescue branch above, and for the same reason —
///   its predicate needs a `Contact` this outcome cannot produce either).
///   This is a fail-safe for a verdict that was never computed, not a
///   computed answer — see [`PartContactOutcome`]'s own doc.
///
/// The `collision` flag is set independent of the storage budget
/// (`request.max_contacts`, `request.max_contacts_per_pair`) for every pair,
/// matching upstream's own invariant.
///
/// The sweep also stops where upstream's does — see [`sweep_is_done`] for the
/// rule and for why the stop is observable rather than a pure saving.
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
    let mut done = false;
    for (a, b) in pairs {
        // Upstream's `if (cdata->done_) return true;`
        // (`collision_detection_fcl/collision_common.cpp:70-71`). One collision-object pair there is
        // one *part* pair here, so the inner loop carries the same guard; this
        // one only stops the outer sweep once the inner one has broken out.
        if done {
            break;
        }
        let allowed = acm.and_then(|m| m.allowed_collision(&a.name, &b.name));
        if matches!(allowed, Some(AllowedCollision::Always)) {
            continue;
        }
        if link_touches_attached(a, b) || attached_pair_allowed(a, b) {
            continue;
        }
        for (a_pose, a_shape, b_pose, b_shape) in part_pairs(a, b) {
            if done {
                break;
            }
            match part_contact(a_pose, a_shape, b_pose, b_shape, 0.0) {
                PartContactOutcome::Touching(contact) => {
                    // NOT gated on `contact.dist <= 0.0`, though the wording
                    // above ("prediction `0.0`, so only touching/penetrating
                    // pairs yield `Some`") reads as though it were. `parry`
                    // returns a contact across a small positive gap too:
                    // measured on prbt's base cylinder against a `4x4x0.1`
                    // box, a `3e-8 m` gap yields `Some` and a `1e-7 m` gap
                    // yields `None`, so the effective boundary sits near
                    // `5e-8 m` of clear air rather than at zero -- far above
                    // `touches_at_tie`'s own band, so this margin is
                    // untouched by it.
                    //
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
                    // `touches_at_tie` decides this, not a literal `dist == 0.0`
                    // check: `fcl::collide` dispatches per shape pair --
                    // `octree_world_collision_response.json` case 4 (an octree
                    // leaf whose `-x` face lands exactly on the robot box's `+x`
                    // face) comes back `robot_collision: true, robot_distance:
                    // -0.0`, while prbt's cylinder resting exactly on a box comes
                    // back `false` with the `-1.0` sentinel (`doc/upstream-bugs.md`,
                    // `fcl-distance-sentinel-survives-zero-contacts`) -- and
                    // `parry` does not land on `0.0` exactly for most of these
                    // ties either (`crate::TIE_ROUNDING_MARGIN`'s own doc has the
                    // measurement); `parry` puts the octree pair a hair *above*
                    // zero and prbt's at `-2.775558e-17`, both well inside
                    // `touches_at_tie`'s rounding band, so both still reach the
                    // table rather than falling through to a raw sign check that
                    // would get one of them wrong.
                    // `crates/moveit-collision/tests/exact_tangency_boundary.rs` pins
                    // both ends of this as measurements rather than intentions.
                    let touches = touches_at_tie(contact.dist, a_pose, a_shape, b_pose, b_shape);
                    let is_collision = touches
                        && match allowed {
                            Some(AllowedCollision::Conditional(ref predicate)) => {
                                !predicate(&mut c)
                            }
                            Some(AllowedCollision::Never) | None => true,
                            Some(AllowedCollision::Always) => unreachable!("filtered out above"),
                        };
                    if is_collision {
                        collision = true;
                        if request.contacts && stored_total < request.max_contacts {
                            let bucket = by_pair.entry(pair_key(&a.name, &b.name)).or_default();
                            if bucket.len() < request.max_contacts_per_pair {
                                bucket.push(c);
                                stored_total += 1;
                            }
                        }
                    }
                }
                PartContactOutcome::NotTouching => {
                    if !matches!(allowed, Some(AllowedCollision::Conditional(_)))
                        && fcl_tangency_verdict(a_shape, b_shape) == Some(true)
                    {
                        // `query::contact` missed this pair (`None`) even
                        // though they may be geometrically touching or
                        // overlapping: the only way that happens is a
                        // specialised closed-form routine excluding a
                        // boundary its own geometric test admits --
                        // `contact_ball_ball`'s strict `<` vs
                        // `intersection_test_ball_ball`'s `<=` is the one
                        // this crate has measured
                        // (`parry_boolean_queries_disagree_in_both_directions_at_the_tie`).
                        // `fcl_tangency_verdict` gates this to the pairs the
                        // table says collide, so a pair it says does *not*
                        // (e.g. `box x cone`) never reaches
                        // `part_intersects` here at all -- this branch
                        // cannot make one of those `true`.
                        let intersects = match part_intersects(a_pose, a_shape, b_pose, b_shape) {
                            PartIntersectOutcome::Touching => true,
                            PartIntersectOutcome::NotTouching => false,
                            // Unreachable today (see `part_intersects`'s own
                            // doc) but still named explicitly rather than
                            // folded into `false` by a wildcard or
                            // `.unwrap_or` -- the whole point of this type.
                            PartIntersectOutcome::Unsupported => true,
                        };
                        if intersects {
                            // There is no `Contact` to build for this pair,
                            // so unlike every other branch this sets
                            // `collision` alone: no `by_pair` entry, no cost
                            // sources, and `AllowedCollision::Conditional`
                            // pairs are excluded above rather than guessed
                            // at, since its predicate needs a `Contact` this
                            // branch cannot produce.
                            collision = true;
                        }
                    }
                }
                PartContactOutcome::Unsupported => {
                    // Fail-safe, not a computed verdict: parry has no
                    // dispatch arm for this shape-kind pair (today, only
                    // `HalfSpace` x `HalfSpace` -- see `PartContactOutcome`'s
                    // own doc). Same `Conditional` exclusion as the branch
                    // above and for the same reason: its predicate needs a
                    // `Contact` this outcome cannot produce either. No
                    // `Contact` to build or store either way.
                    if !matches!(allowed, Some(AllowedCollision::Conditional(_))) {
                        collision = true;
                    }
                }
            }
            // Reached whether or not the query found anything, exactly as
            // upstream's termination block is: `fcl::collide` returning zero
            // contacts still falls through to `collision_detection_fcl/collision_common.cpp:395`. Only
            // the skip rules above bypass it, and upstream's counterparts
            // `return false` at `:184-185` before ever reaching it.
            done = sweep_is_done(request, collision, stored_total, &by_pair, &cost_sources);
        }
    }
    sweep_result(request, collision, &by_pair, &cost_sources)
}

/// The sweep state of [`accumulate_collision`] in the shape a caller sees it —
/// used both for the function's own return value and for the argument
/// [`CollisionRequest::is_done`] is handed mid-sweep, so a callback observes
/// exactly the result it would have got had the sweep ended there.
///
/// `distance` is always `None`: upstream fills `res.distance` only after the
/// sweep returns (`collision_env_fcl.cpp:283-297` and `:340-354`), so the
/// result its own `is_done` sees never carries one either.
///
/// The clone is bounded by `request.max_contacts` and
/// `request.max_cost_sources`, not by the number of pairs swept.
fn sweep_result(
    request: &CollisionRequest,
    collision: bool,
    by_pair: &BTreeMap<(String, String), Vec<Contact>>,
    cost_sources: &BTreeSet<CostSource>,
) -> CollisionResult {
    CollisionResult {
        collision,
        distance: None,
        contacts: request.contacts.then(|| ContactData {
            by_pair: by_pair.clone(),
        }),
        cost_sources: request.cost.then(|| cost_sources.iter().cloned().collect()),
    }
}

/// Upstream's two termination sources, evaluated once per part pair at
/// `collision_detection_fcl/collision_common.cpp:395-424` and answering its `done_`:
///
/// - implicit (`:395-407`) — a collision is on record, the contact budget is
///   either unwanted or already full, and no cost is being accumulated. Every
///   field is therefore already at its final value, so this one only decides
///   how much work the sweep still does; the one way it shows is that an
///   [`AllowedCollision::Conditional`] predicate on a later pair stops being
///   called, which is upstream's behaviour too.
/// - explicit (`:411-413`) — [`CollisionRequest::is_done`], consulted only
///   when the implicit rule did not already fire, and given
///   [`sweep_result`]'s view of the state. This one *is* observable: a
///   callback can stop the sweep with `collision` still `false`, or with
///   fewer contacts stored than `request.max_contacts` allows.
///
/// Note `request.cost` suppresses only the implicit source, matching
/// upstream's nesting — `is_done` is still consulted while costs accumulate.
fn sweep_is_done(
    request: &CollisionRequest,
    collision: bool,
    stored_total: usize,
    by_pair: &BTreeMap<(String, String), Vec<Contact>>,
    cost_sources: &BTreeSet<CostSource>,
) -> bool {
    if collision && (!request.contacts || stored_total >= request.max_contacts) && !request.cost {
        return true;
    }
    match &request.is_done {
        Some(is_done) => is_done(&sweep_result(request, collision, by_pair, cost_sources)),
        None => false,
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
///
/// [`PartContactOutcome::Unsupported`] (parry has no dispatch arm for this
/// shape-kind pair at all) sets `result.collision` alone, with no distance or
/// nearest points recorded — there is no `Contact` to compute them from, and
/// unlike `accumulate_collision`, upstream's `distanceCallback` does not
/// special-case `AllowedCollision::Conditional` at all, so this fail-safe
/// applies unconditionally rather than excluding it.
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
        let key = pair_key(&a.name, &b.name);
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
            let contact = match part_contact(
                a_pose,
                a_shape,
                b_pose,
                b_shape,
                bounded_prediction(threshold),
            ) {
                PartContactOutcome::Touching(contact) => contact,
                PartContactOutcome::NotTouching => continue,
                PartContactOutcome::Unsupported => {
                    // Fail-safe, not a computed verdict -- see
                    // `PartContactOutcome`'s own doc (today, only `HalfSpace`
                    // x `HalfSpace`). No `Contact` to derive a distance or
                    // nearest points from, but `result.collision` must not
                    // silently stay `false` for a pair whose verdict was
                    // never computed.
                    result.collision = true;
                    continue;
                }
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
            // `touches_at_tie` on the raw `contact.dist`, not `data.distance`:
            // `enable_signed_distance == false` clamps `data.distance` to
            // `contact.dist.max(0.0)` above, which would otherwise turn every
            // negative rounding tie into an unconditional `<= 0.0` match here
            // regardless of the margin -- the same two-owners problem
            // `touches_at_tie`'s own doc describes, reappearing between this
            // clamp and the check rather than between the two functions.
            if touches_at_tie(contact.dist, a_pose, a_shape, b_pose, b_shape) {
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

/// The trailing `if (req.distance)` block that both of upstream's collision
/// helpers carry (`collision_env_fcl.cpp:283-297` for self, `:340-354` for
/// robot, at `e017c91ee`): a second, separate distance query whose
/// `minimum_distance` lands in `CollisionResult::distance`, and whose full
/// result lands there instead when `detailed_distance` is also set.
///
/// Factored rather than written twice on purpose. The two helpers upstream
/// are line-for-line identical here apart from which query they call, and
/// duplicating it is how one of them ends up implementing `request.distance`
/// while the other silently ignores it -- which is what this backend did on
/// both paths until upstream's own `DistanceSelf` / `DistanceWorld` cases
/// (`test_collision_common_panda.hpp:237-267`) were ported and asked for the
/// field.
///
/// The follow-up request is upstream's, field for field: `group_name` and
/// `acm` carried over from the collision request, everything else left at
/// `DistanceRequest`'s defaults -- notably `enable_signed_distance: false`,
/// so a penetrating pair reports `0.0` here rather than a signed depth.
fn attach_requested_distance(
    result: &mut CollisionResult,
    request: &CollisionRequest,
    acm: Option<&AllowedCollisionMatrix>,
    query: impl FnOnce(&DistanceRequest<'_>) -> DistanceResult,
) {
    if !request.distance {
        return;
    }
    let distance_request = DistanceRequest {
        group_name: request.group_name.as_deref(),
        acm,
        ..DistanceRequest::default()
    };
    let distance_result = query(&distance_request);
    result.distance = Some(if request.detailed_distance {
        CollisionDistance::Detailed(distance_result)
    } else {
        CollisionDistance::Closest(distance_result.minimum_distance.distance)
    });
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
        let active = active_group_links(state, request.group_name.as_deref());
        let pairs = self_pairs(&bodies).filter(|(a, b)| {
            active
                .as_ref()
                .is_none_or(|g| pair_in_active_group(g, a, b))
        });
        let mut result = accumulate_collision(pairs, request, acm);
        attach_requested_distance(&mut result, request, acm, |distance_request| {
            self.distance_self(distance_request, state, attached_bodies)
        });
        result
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
        let active = active_group_links(state, request.group_name.as_deref());
        let pairs = cross_pairs(&robot, &world).filter(|(a, b)| {
            active
                .as_ref()
                .is_none_or(|g| pair_in_active_group(g, a, b))
        });
        let mut result = accumulate_collision(pairs, request, acm);
        attach_requested_distance(&mut result, request, acm, |distance_request| {
            self.distance_robot(distance_request, state, attached_bodies)
        });
        result
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
        let active = active_group_links(state, request.group_name);
        let pairs = self_pairs(&bodies).filter(|(a, b)| {
            active
                .as_ref()
                .is_none_or(|g| pair_in_active_group(g, a, b))
        });
        accumulate_distance(pairs, request)
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
        let active = active_group_links(state, request.group_name);
        let pairs = cross_pairs(&robot, &world).filter(|(a, b)| {
            active
                .as_ref()
                .is_none_or(|g| pair_in_active_group(g, a, b))
        });
        accumulate_distance(pairs, request)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_geometry::{Cuboid, Mesh, OcTree, Plane, Shape, Sphere};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;
    use crate::common::IsDoneFn;

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

    #[test]
    fn scaled_padded_shape_pads_a_mesh_that_arrived_without_vertex_normals() {
        // `Mesh::new` is public and leaves `vertex_normals: None`, and it is
        // how an `AttachedBodyGeometry`'s mesh is built on this side; only
        // `moveit_geometry::stl::mesh_from_bytes` computes them. Upstream
        // cannot reach this state at all -- every `geometric_shapes`
        // creation entry point ends with `computeVertexNormals()` -- so it
        // has no equivalent guard to port. `Mesh::scale_and_padd` reads the
        // normal array for *any* scale and padding, including this
        // identity pair, so the whole call panicked before, not just a
        // padded one.
        let mesh = Mesh::new(
            vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap();
        assert!(
            mesh.vertex_normals.is_none(),
            "Mesh::new must still leave the normals uncomputed, or this pins nothing"
        );
        let padded = scaled_padded_shape(&Shape::Mesh(mesh), 1.0, 0.0);
        match padded {
            Shape::Mesh(m) => assert_eq!(m.vertices.len(), 3),
            other => panic!("expected Mesh, got {other:?}"),
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

    /// [`build_model`], plus one SRDF `<group>` naming a subset of
    /// `link_names`' floating joints — for exercising [`active_group_links`]
    /// (module doc, deviation 1), which [`build_model`]'s group-less SRDF
    /// cannot reach.
    fn build_model_with_group(
        link_names: &[&str],
        group_name: &str,
        group_links: &[&str],
    ) -> RobotModel {
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
        let joints: String = group_links
            .iter()
            .map(|name| format!(r#"<joint name="joint_{name}"/>"#))
            .collect();
        let srdf_xml = format!(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
                <group name="{group_name}">{joints}</group>
            </robot>"#
        );
        let srdf = SrdfModel::parse_str(&srdf_xml).expect("test SRDF must parse");
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

    // Termination rule (`sweep_is_done`). All of these drive
    // `check_robot_collision` with a single robot link, because `cross_pairs`
    // walks the world in `World`'s `BTreeMap` order: naming the objects fixes
    // the order in which the sweep meets them, which is what makes "stopped
    // before the second pair" a statement about the rule and not about
    // whichever pair happened to come first.

    /// One robot link `p`, a unit box at the origin.
    fn one_link_at_origin(model: &RobotModel) -> RobotState<'_> {
        state_with_links_at(model, &[("p", Isometry3::identity())])
    }

    /// A world of unit boxes at the given ids and poses.
    fn world_with_boxes(boxes: &[(&str, Isometry3)]) -> World {
        let mut world = World::new();
        for (id, pose) in boxes {
            world.add_shape(
                id,
                Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
                *pose,
            );
        }
        world
    }

    /// What [`recording_is_done`] noted down: one `(collision, contact
    /// count)` per call, in call order.
    type IsDoneLog = Arc<Mutex<Vec<(bool, usize)>>>;

    /// An `is_done` that answers `verdict` and records `(collision, contact
    /// count)` from every result it is handed, so a test can assert both how
    /// often the sweep consulted it and what it saw.
    fn recording_is_done(verdict: bool) -> (IsDoneLog, IsDoneFn) {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        let callback: IsDoneFn = Arc::new(move |result: &CollisionResult| {
            sink.lock().unwrap_or_else(PoisonError::into_inner).push((
                result.collision,
                result.contacts.as_ref().map_or(0, ContactData::count),
            ));
            verdict
        });
        (seen, callback)
    }

    fn recorded(seen: &IsDoneLog) -> Vec<(bool, usize)> {
        seen.lock().unwrap_or_else(PoisonError::into_inner).clone()
    }

    #[test]
    fn is_dones_answer_decides_whether_a_later_colliding_pair_is_reached() {
        // The whole point of `is_done`: the caller, not the backend, decides
        // the sweep is over. `a_far` is out of contact, so the implicit rule
        // cannot have fired at the pair that answers; the sweep goes on to
        // `b_near`, which overlaps `p`, only if the answer was no. Both
        // answers run against one scene because the boundary is the answer —
        // asserting either alone leaves "the answer is read at all" untested.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let scene = || {
            world_with_boxes(&[
                ("a_far", Isometry3::translation(10.0, 0.0, 0.0)),
                ("b_near", Isometry3::translation(0.5, 0.0, 0.0)),
            ])
        };
        let request = |verdict: bool| CollisionRequest {
            is_done: Some(Arc::new(move |_: &CollisionResult| verdict)),
            ..CollisionRequest::default()
        };

        let stopped = ParryCollisionEnv::new(scene(), LinkPaddingScale::default())
            .check_robot_collision(&request(true), &posed, &[], None);
        let ran_on = ParryCollisionEnv::new(scene(), LinkPaddingScale::default())
            .check_robot_collision(&request(false), &posed, &[], None);

        assert!(
            !stopped.collision,
            "is_done answering yes must end the sweep at the pair that asked"
        );
        assert!(
            ran_on.collision,
            "is_done answering no must leave the sweep running"
        );
    }

    #[test]
    fn a_pair_with_no_contact_still_consults_is_done() {
        // Upstream's termination block sits after the `fcl::collide` call, not
        // inside its "found something" branch, so a pair that came back empty
        // still gets to end the sweep. A port that folds the check into the
        // contact-found arm never offers the callback a clear pair at all.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[("a_far", Isometry3::translation(10.0, 0.0, 0.0))]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        env.check_robot_collision(&request, &posed, &[], None);

        assert_eq!(
            recorded(&seen),
            vec![(false, 0)],
            "the one clear pair must reach is_done exactly once"
        );
    }

    #[test]
    fn a_pair_the_acm_always_allows_never_consults_is_done() {
        // Upstream's `if (always_allow_collision) return false;`
        // (`collision_detection_fcl/collision_common.cpp:184-185`) returns before the termination
        // block, so an allowed pair is invisible to the callback however
        // deeply it overlaps. Both objects here overlap `p`, so the one
        // recorded call can only be `b_checked`; `cost` is what keeps the
        // implicit rule from claiming that call first.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[
            ("a_allowed", Isometry3::translation(0.5, 0.0, 0.0)),
            ("b_checked", Isometry3::translation(-0.5, 0.0, 0.0)),
        ]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let mut acm = AllowedCollisionMatrix::new();
        acm.set_entry("p", "a_allowed", true);
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            cost: true,
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        env.check_robot_collision(&request, &posed, &[], Some(&acm));

        assert_eq!(
            recorded(&seen),
            vec![(true, 0)],
            "an ACM-allowed pair must not be offered to is_done"
        );
    }

    #[test]
    fn the_implicit_stop_pre_empts_is_done() {
        // `collision_detection_fcl/collision_common.cpp:411` guards the callback with `!cdata->done_`.
        // With contacts and cost both off, the first collision satisfies the
        // implicit rule outright, so the callback is never asked — and the
        // second overlapping object is never looked at either.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[
            ("a_near", Isometry3::translation(0.5, 0.0, 0.0)),
            ("b_near", Isometry3::translation(-0.5, 0.0, 0.0)),
        ]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert!(result.collision);
        assert_eq!(
            recorded(&seen),
            Vec::new(),
            "the implicit stop must fire without consulting is_done"
        );
    }

    #[test]
    fn cost_accumulation_suppresses_the_implicit_stop_but_not_is_done() {
        // Upstream nests `if (!req.cost) done_ = true;` inside the implicit
        // rule alone: costs keep the sweep alive because a later pair can
        // still displace a cheaper source, but they say nothing about the
        // caller's own callback, which is consulted for both pairs.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[
            ("a_near", Isometry3::translation(0.5, 0.0, 0.0)),
            ("b_near", Isometry3::translation(-0.5, 0.0, 0.0)),
        ]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            cost: true,
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        env.check_robot_collision(&request, &posed, &[], None);

        assert_eq!(
            recorded(&seen),
            vec![(true, 0), (true, 0)],
            "with cost on, both pairs must be offered to is_done"
        );
    }

    #[test]
    fn is_done_sees_the_contacts_stored_so_far() {
        // The callback's argument is the running result, not a fresh one:
        // upstream hands it `*cdata->res_`. The second call must therefore
        // see the contact the first pair stored. A budget of 5 keeps the
        // implicit rule from firing at either pair.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[
            ("a_near", Isometry3::translation(0.5, 0.0, 0.0)),
            ("b_near", Isometry3::translation(-0.5, 0.0, 0.0)),
        ]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 5,
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        env.check_robot_collision(&request, &posed, &[], None);

        assert_eq!(
            recorded(&seen),
            vec![(true, 1), (true, 2)],
            "is_done must see the sweep's running result, not a fresh one"
        );
    }

    #[test]
    fn a_full_contact_budget_stops_the_sweep_at_the_pair_that_filled_it() {
        // `res.contact_count >= req.max_contacts` is the other side of the
        // boundary above: one contact against a budget of one satisfies the
        // implicit rule, so `b_near` is never reached and the second contact
        // never stored. Nothing is lost by stopping — the budget could not
        // have taken it — which is why upstream stops.
        let model = build_model(&["p"]);
        let mut state = one_link_at_origin(&model);
        let posed = state.update();
        let world = world_with_boxes(&[
            ("a_near", Isometry3::translation(0.5, 0.0, 0.0)),
            ("b_near", Isometry3::translation(-0.5, 0.0, 0.0)),
        ]);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let (seen, is_done) = recording_is_done(false);
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 1,
            is_done: Some(is_done),
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert_eq!(result.contacts.expect("contacts requested").count(), 1);
        assert_eq!(
            recorded(&seen),
            Vec::new(),
            "a filled budget must stop the sweep without consulting is_done"
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

    /// Module doc, deviation 1: a robot-vs-world pair is kept exactly when
    /// the robot link is in `group_name`'s active set. `q` overlaps the
    /// world object but is not in `"p_only"` (whose sole member is `p`,
    /// which does not overlap), so the pair must be dropped entirely.
    #[test]
    fn check_robot_collision_group_name_drops_a_pair_whose_link_is_outside_the_group() {
        let model = build_model_with_group(&["p", "q"], "p_only", &["p"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::translation(10.0, 0.0, 0.0)),
                ("q", Isometry3::identity()),
            ],
        );
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = CollisionRequest {
            group_name: Some("p_only".to_string()),
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert!(!result.collision);
    }

    /// The mirror of the case above: `p` is both in the active group and
    /// overlapping the world object, so the pair must be kept.
    #[test]
    fn check_robot_collision_group_name_keeps_a_pair_whose_link_is_inside_the_group() {
        let model = build_model_with_group(&["p", "q"], "p_only", &["p"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(10.0, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = CollisionRequest {
            group_name: Some("p_only".to_string()),
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert!(result.collision);
    }

    /// Module doc, deviation 1: a self-collision pair is kept when *either*
    /// side is active, not only when both are — `q` is outside `"p_only"`
    /// but `p` is inside it, and the two overlap, so the pair must still be
    /// reported.
    #[test]
    fn check_self_collision_group_name_keeps_a_pair_with_only_one_side_active() {
        let model = build_model_with_group(&["p", "q"], "p_only", &["p"]);
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
            group_name: Some("p_only".to_string()),
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(result.collision);
    }

    /// `RobotModel::joint_model_group` returning `Err` for an unknown name
    /// falls back to "no active group" (`enableGroup`'s own
    /// `hasJointModelGroup` guard, module doc deviation 1), matching
    /// `group_name: None` rather than filtering out everything.
    #[test]
    fn check_robot_collision_unknown_group_name_falls_back_to_unfiltered() {
        let model = build_model_with_group(&["p"], "p_only", &["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = CollisionRequest {
            group_name: Some("does_not_exist".to_string()),
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert!(result.collision);
    }

    /// [`distance_robot`] reads the same `group_name`/`active_components_only`
    /// filter as [`check_robot_collision`] (module doc, deviation 1): `q` is
    /// nearer the world object (gap `0.5`) but outside `"p_only"`, so the
    /// reported minimum distance must be `p`'s (`10.0`), not `q`'s.
    #[test]
    fn distance_robot_group_name_ignores_a_nearer_link_outside_the_group() {
        let model = build_model_with_group(&["p", "q"], "p_only", &["p"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::translation(-11.0, 0.0, 0.0)),
                ("q", Isometry3::translation(1.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = DistanceRequest {
            group_name: Some("p_only"),
            ..DistanceRequest::default()
        };

        let result = env.distance_robot(&request, &posed, &[]);

        assert_eq!(result.minimum_distance.distance, 10.0);
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

    /// An axis-aligned box's boundary as 12 triangles (2 per face). None of
    /// `moveit_geometry`'s mesh loaders emit a bare box like this (they
    /// parse STL/OBJ files), so a test that wants a `Shape::Mesh` sized
    /// differently from [`box_link`]'s hardcoded `1x1x1` collision box has
    /// to build one by hand.
    fn box_shell_mesh(half_extent: f64) -> Mesh {
        let h = half_extent;
        let vertices = vec![
            Vector3::new(-h, -h, -h), // 0
            Vector3::new(h, -h, -h),  // 1
            Vector3::new(h, h, -h),   // 2
            Vector3::new(-h, h, -h),  // 3
            Vector3::new(-h, -h, h),  // 4
            Vector3::new(h, -h, h),   // 5
            Vector3::new(h, h, h),    // 6
            Vector3::new(-h, h, h),   // 7
        ];
        let triangles = vec![
            [0, 1, 2],
            [0, 2, 3], // z = -h
            [4, 6, 5],
            [4, 7, 6], // z = h
            [0, 5, 1],
            [0, 4, 5], // y = -h
            [3, 2, 6],
            [3, 6, 7], // y = h
            [0, 3, 7],
            [0, 7, 4], // x = -h
            [1, 5, 6],
            [1, 6, 2], // x = h
        ];
        Mesh::new(vertices, triangles).unwrap()
    }

    #[test]
    fn mesh_engulfment_is_reported_as_no_collision_and_a_positive_gap_not_penetration() {
        // §56.4's residual: parry's `TriMesh` contact is a per-triangle
        // max (module doc, "Composite shapes"), so a mesh's boundary-only
        // representation can miss a real overlap that never shows up as
        // any single triangle pair intersecting. The sharpest instance of
        // that is not a shallow crossing -- it is full containment, where
        // no triangle pair from either mesh ever comes closer than the two
        // meshes' nearest parallel faces, even though the two *solids* the
        // meshes represent overlap completely.
        //
        // Two concentric, axis-aligned box shells centered on the origin:
        // `outer` half-extent `1.0`, `inner` half-extent `0.1`. `inner`
        // never crosses `outer`'s boundary anywhere -- the closest any
        // pair of triangles gets is the two meshes' parallel faces,
        // `1.0 - 0.1 = 0.9` apart on every axis -- so a purely
        // surface-vs-surface narrow phase (min `dist` over every candidate
        // triangle pair) has nothing closer than `0.9` to report. The true
        // minimum-translation distance to separate the two *solids* the
        // shells represent is `1.0 + 0.1 = 1.1` (translate `inner` until
        // its near face clears `outer`'s far face on the same axis) -- the
        // opposite sign and a different number entirely from the `0.9` a
        // boundary-only narrow phase can see.
        let outer = box_shell_mesh(1.0);
        let inner = box_shell_mesh(0.1);

        let model = build_model(&["p"]);
        // `p` itself carries its own default `1x1x1` box collision
        // (`box_link`) -- parked far from the origin so only the attached
        // `inner` mesh (translated back to the origin by its own
        // `shape_pose`) can reach `outer`.
        let mut state =
            state_with_links_at(&model, &[("p", Isometry3::translation(100.0, 0.0, 0.0))]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape("outer", Arc::new(Shape::Mesh(outer)), Isometry3::identity());
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let shapes = vec![Arc::new(Shape::Mesh(inner))];
        let shape_poses = vec![Isometry3::translation(-100.0, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "inner",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let collision =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[attached], None);
        assert!(
            !collision.collision,
            "a fully-engulfed mesh must read as no collision under a per-triangle narrow \
             phase, or this test no longer demonstrates §56.4's mechanism"
        );

        let distance = env.distance_robot(&DistanceRequest::default(), &posed, &[attached]);
        assert!(!distance.collision);
        // The closed-form gap is `1.0 - 0.1 = 0.9`; parry's actual answer
        // comes from a vector closest-points computation, not a scalar
        // subtraction, and lands one ULP below that (`0.8999999999999999`)
        // -- `1e-12` absorbs that rounding path without absorbing the
        // `0.9`-scale mechanism this test exists to pin.
        assert!(
            (distance.minimum_distance.distance - 0.9).abs() < 1e-12,
            "boundary-only narrow phase must report the parallel-face gap (~0.9), not the \
             solids' true overlap: got {}",
            distance.minimum_distance.distance
        );
    }

    #[test]
    fn check_robot_collision_halfspace_pair_fails_safe_to_collision() {
        // `HalfSpace` x `HalfSpace` has no dispatch arm at all in
        // `default_query_dispatcher.rs`, for any pose (`PartContactOutcome`'s
        // own doc) -- `query::contact` returns `Err(Unsupported)` for any two
        // planes. Reachability: ordinary URDF parsing can never produce
        // `Shape::Plane` for a robot link (`urdf-rs`'s `Geometry` enum has no
        // `Plane` variant), and `cross_pairs`/`self_pairs` never pair
        // world-vs-world -- so the only way two `HalfSpace`s reach
        // `accumulate_collision` together is one on the robot side (via
        // `AttachedBodyGeometry`, used here) and one on the world side (via
        // `World::add_shape`), through `check_robot_collision`'s own
        // `cross_pairs`.
        //
        // This test makes no claim about these two planes' true geometric
        // relationship -- the closed-form `HalfSpace` x `HalfSpace` predicate
        // is a separate, unresolved question. It pins only that parry cannot
        // compute *any* verdict for this pair, and that "never computed" must
        // read as a collision, not silently as "computed: no". Before
        // `PartContactOutcome` existed, `if let Ok(Some(contact)) =
        // query::contact(...)` collapsed that `Err` into the same `false` a
        // real negative would have produced.
        let model = build_model(&["p"]);
        // `p` itself carries its own default `1x1x1` box collision
        // (`box_link`) -- parked far from the origin so only the attached
        // plane can reach the world plane.
        let mut state =
            state_with_links_at(&model, &[("p", Isometry3::translation(100.0, 0.0, 0.0))]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "world_plane",
            Arc::new(Shape::Plane(Plane::new(0.0, 0.0, -1.0, -10.0))),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let shapes = vec![Arc::new(Shape::Plane(Plane::new(0.0, 0.0, 1.0, 0.0)))];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "attached_plane",
            link_name: "p",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let result =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[attached], None);
        assert!(
            result.collision,
            "HalfSpace x HalfSpace has no query::contact dispatch arm at all -- the fail-safe \
             must report a collision rather than silently swallow the Err as \"not colliding\""
        );
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

    /// Adversarial check for `doc/upstream-bugs.md`'s
    /// `distance-callback-max-contact-depth` entry, at the one level
    /// `penetration_depth_scale_invariance.rs` cannot reach: that reproducer
    /// runs the whole `panda_link0`-vs-floor pipeline, so an artifact hiding
    /// in a single triangle's own EPA result could in principle be masked by
    /// the `min` this backend takes across many triangles
    /// (`accumulate_distance`'s `data.distance < result.minimum_distance.distance`,
    /// and, inside `parry3d_f64`'s own `TriMesh` narrow phase, the
    /// per-triangle `min` this module's own doc describes at
    /// "keeps the deepest"). This test isolates a *single* degenerate
    /// (zero-volume) triangle, embedded in a box exactly the way the entry's
    /// mechanism requires -- "a triangle lying entirely inside a large box
    /// has no separating axis" -- and calls `parry3d_f64::query::contact`
    /// directly, bypassing every reduction this crate or `parry3d_f64`'s
    /// mesh narrow phase performs.
    ///
    /// If EPA on a fully-embedded degenerate triangle ever resolves to a
    /// lateral escape (the mechanism the entry attributes to FCL/libccd),
    /// `dist` would grow with the box's half-width here. It does not: held
    /// fixed to nine decimal places across a 50x width sweep, this backend's
    /// `contact.dist` reports the true (vertical) shortest exit at every
    /// width. That does not establish FCL/libccd's own libccd/EPA behaves
    /// differently on the same input -- this crate has no FCL binding to
    /// query for comparison -- only that the claimed artifact is not a
    /// property embedded-degenerate-triangle-vs-box queries have in general;
    /// it is specific to whichever narrow-phase implementation exhibits it.
    #[test]
    fn parry_epa_on_a_degenerate_embedded_triangle_does_not_scale_with_box_width() {
        let triangle = ParryTriangle::new(
            ParryVector::new(-0.01, -0.01, 0.0),
            ParryVector::new(0.01, -0.01, 0.0),
            ParryVector::new(0.0, 0.01, 0.0),
        );
        let tri_pose = to_pose(Isometry3::identity());
        let mut dists = Vec::new();
        for &width in &[0.4_f64, 1.0, 4.0, 20.0] {
            let half = width / 2.0;
            let thickness = 0.1;
            let cuboid = ParryCuboid::new(ParryVector::new(half, half, thickness / 2.0));
            // Box top face at z = 0.02, so the z=0 triangle sits 0.02m below
            // the top and 0.03m above the bottom -- embedded, nearest exit is
            // vertical regardless of width.
            let box_pose = to_pose(Isometry3::translation(0.0, 0.0, -0.03));
            let result = query::contact(&tri_pose, &triangle, &box_pose, &cuboid, 100.0)
                .expect("contact query must not error")
                .expect("triangle must be within prediction distance of the box");
            dists.push((width, result.dist));
        }
        let values: Vec<f64> = dists.iter().map(|&(_, d)| d).collect();
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max - min < 1e-9,
            "a degenerate triangle embedded in a box must report the same escape distance \
             regardless of the box's width, but it spread by {:.6e}: {dists:?}",
            max - min
        );
        assert!(
            (values[0] - -0.02).abs() < 1e-9,
            "expected the vertical exit -0.02, got {dists:?}"
        );
    }

    /// `approx` has no blanket `RelativeEq` for `[f64; 3]`; this compares
    /// component-wise instead of stringifying both sides into a slice.
    /// Tolerance `1e-12`: most callers hand-compute exact-literal geometry
    /// and would pass at `0.0`, but a mesh whose points lie exactly on a
    /// degenerate (zero-extent) principal axis drives
    /// `parry3d_f64::utils::obb`'s covariance eigendecomposition to a
    /// near-singular matrix, which lands that axis at noise level
    /// (measured up to `1.5e-15`, not exactly `0.0`) rather than a bug.
    fn assert_point_close(actual: [f64; 3], expected: [f64; 3]) {
        for i in 0..3 {
            assert!(
                (actual[i] - expected[i]).abs() < 1e-12,
                "component {i}: actual {} vs expected {}",
                actual[i],
                expected[i]
            );
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

    /// Two triangles far apart, each with a small local AABB, so the mesh's
    /// combined root box is much larger than either one alone: a query
    /// shape can sit inside the combined box while missing both individual
    /// triangle AABBs entirely. This is the case that distinguishes coarse
    /// (root-box) granularity from per-triangle granularity — the two
    /// would disagree on it (empty vs. one source). Both triangles are
    /// right triangles with legs on the x/y axes and the point set is
    /// symmetric under swapping x and y, so this mesh does not exercise
    /// whether [`mesh_world_obb_aabb`] actually applies a rotation — see
    /// the off-axis mesh in
    /// `mesh_world_obb_aabb_tracks_the_mesh_rotation_not_just_its_aabb` for
    /// that.
    fn two_far_apart_triangles() -> TriMesh {
        TriMesh::new(
            vec![
                ParryVector::new(0.0, 0.0, 0.0),
                ParryVector::new(1.0, 0.0, 0.0),
                ParryVector::new(0.0, 1.0, 0.0),
                ParryVector::new(9.0, 9.0, 0.0),
                ParryVector::new(10.0, 9.0, 0.0),
                ParryVector::new(9.0, 10.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        )
        .unwrap()
    }

    #[test]
    fn mesh_shape_cost_sources_is_the_whole_mesh_obb_overlapped_with_the_whole_shape_aabb() {
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = two_far_apart_triangles();
        // Centered in the gap between the two triangles: outside both of
        // their individual AABBs (x in [0,1] and x in [9,10]), but inside
        // the mesh's combined AABB (x in [0,10]).
        let other_pose = to_pose(Isometry3::translation(5.0, 5.0, 0.0));
        let other = ParryCuboid::new(ParryVector::new(2.0, 2.0, 2.0));

        let sources = mesh_shape_cost_sources(&mesh_pose, &mesh, &other_pose, &other);

        assert_eq!(
            sources.len(),
            1,
            "one coarse box, not zero (per-triangle would find no intersection here)"
        );
        assert_point_close(sources[0].aabb_min, [3.0, 3.0, 0.0]);
        assert_point_close(sources[0].aabb_max, [7.0, 7.0, 0.0]);
        assert_eq!(sources[0].cost, 1.0);
    }

    #[test]
    fn mesh_world_obb_aabb_tracks_the_mesh_rotation_not_just_its_points_aabb() {
        // A thin triangle running along the world (1,1) diagonal: two
        // vertices far out on the diagonal, one just off it, so the mesh's
        // own tightest-fitting box is not axis-aligned. If this function
        // silently degenerated to the mesh's plain axis-aligned points-AABB
        // (the bug this fix replaces), it would equal that AABB exactly;
        // a genuinely oriented fit cannot ever be tighter than that AABB
        // (the direct AABB of a point set is already the tightest possible
        // axis-aligned bound of those points, so any other box containing
        // them, axis-aligned afterward, bounds at least as much volume),
        // and for a box tilted away from the world axes it is strictly
        // more, which is what actually finding the tilt should produce.
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = TriMesh::new(
            vec![
                ParryVector::new(-5.0, -5.0, 0.0),
                ParryVector::new(5.0, 5.0, 0.0),
                ParryVector::new(0.0, 0.1, 0.0),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap();

        let points_aabb = mesh.aabb(&mesh_pose);
        let obb_aabb = mesh_world_obb_aabb(&mesh_pose, &mesh);

        assert!(
            obb_aabb.mins.x < points_aabb.mins.x
                || obb_aabb.mins.y < points_aabb.mins.y
                || obb_aabb.maxs.x > points_aabb.maxs.x
                || obb_aabb.maxs.y > points_aabb.maxs.y,
            "obb_aabb {obb_aabb:?} should extend past points_aabb {points_aabb:?} on at least \
             one axis if the fit is actually oriented, not just re-deriving the points' own AABB"
        );
    }

    /// This function's own doc claims [`mesh_world_obb_aabb`] fits `mesh`'s
    /// oriented box from `mesh.triangles()`'s flattened per-*corner* points
    /// (FCL's `getCovariance`, `geometry-inl.h:1349-1379`), weighting a
    /// shared vertex once per incident triangle rather than once per unique
    /// vertex. Every other test of this function uses a mesh where each
    /// vertex is incident to exactly one triangle, so per-corner and
    /// per-vertex weighting coincide there and neither this claim nor a
    /// regression to `mesh.vertices()` (deduplicated) would show up.
    ///
    /// This test would catch that regression: `v0` is the apex of a
    /// 5-triangle fan, so it carries 5 corner copies against 2 apiece for
    /// `v1..v5` -- a weighting only the per-corner scheme produces. Both
    /// candidate fits are computed independently here, via the exact same
    /// `parry3d_f64::utils::obb` this function calls, once on the real
    /// per-corner list and once on the deduplicated vertex list, so this
    /// does not just re-derive [`mesh_world_obb_aabb`]'s own computation.
    ///
    /// What this pins: the weighting scheme (per-corner vs. per-vertex).
    /// What it would not catch: a change to a *different* per-corner-weighted
    /// fit (e.g. a different eigensolver or point-projection convention)
    /// that still weights `v0` 5x -- the claim-audit entry for
    /// `parry3d_f64::utils::obb` itself already covers that the algorithm
    /// used is PCA-via-covariance, not bit-identical to FCL's own.
    #[test]
    fn mesh_world_obb_aabb_weights_by_triangle_corner_not_deduplicated_vertex() {
        let verts = vec![
            ParryVector::new(0.0, 0.0, 0.0),
            ParryVector::new(10.0, 0.0, 0.0),
            ParryVector::new(10.0, 8.0, 0.0),
            ParryVector::new(2.0, 10.0, 0.0),
            ParryVector::new(-8.0, 6.0, 0.0),
            ParryVector::new(-6.0, -6.0, 0.0),
        ];
        let tris: [[u32; 3]; 5] = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5], [0, 5, 1]];
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = TriMesh::new(verts.clone(), tris.to_vec()).unwrap();

        let weighted_corners: Vec<ParryVector> = tris
            .iter()
            .flat_map(|t| t.iter().map(|&i| verts[i as usize]))
            .collect();
        let (weighted_pose, weighted_cuboid) = parry3d_f64::utils::obb(&weighted_corners);
        let weighted_aabb = weighted_cuboid.compute_aabb(&(mesh_pose * weighted_pose));

        let (dedup_pose, dedup_cuboid) = parry3d_f64::utils::obb(&verts);
        let dedup_aabb = dedup_cuboid.compute_aabb(&(mesh_pose * dedup_pose));

        let actual = mesh_world_obb_aabb(&mesh_pose, &mesh);

        assert_point_close(
            [actual.mins.x, actual.mins.y, actual.mins.z],
            [
                weighted_aabb.mins.x,
                weighted_aabb.mins.y,
                weighted_aabb.mins.z,
            ],
        );
        assert_point_close(
            [actual.maxs.x, actual.maxs.y, actual.maxs.z],
            [
                weighted_aabb.maxs.x,
                weighted_aabb.maxs.y,
                weighted_aabb.maxs.z,
            ],
        );
        assert!(
            (actual.mins.x - dedup_aabb.mins.x).abs() > 0.1
                || (actual.mins.y - dedup_aabb.mins.y).abs() > 0.1
                || (actual.maxs.x - dedup_aabb.maxs.x).abs() > 0.1
                || (actual.maxs.y - dedup_aabb.maxs.y).abs() > 0.1,
            "actual {actual:?} should differ from the deduplicated-vertex fit {dedup_aabb:?} by \
             more than fitting noise -- if it did not, this mesh would fail to distinguish the \
             two weighting schemes and this test would not be pinning anything"
        );
    }

    #[test]
    fn mesh_shape_cost_sources_no_intersection_is_empty() {
        let mesh_pose = to_pose(Isometry3::identity());
        let mesh = big_flat_triangle();
        // The mesh's whole AABB is a flat square (z ranging only over
        // [0, 0]); placed far enough along z that even the coarse whole-mesh
        // AABB does not reach the cuboid's AABB.
        let other_pose = to_pose(Isometry3::translation(0.0, -2.0, 5.0));
        let other = ParryCuboid::new(ParryVector::new(0.1, 0.1, 0.1));

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

    // `pair_key` -- both per-pair maps file under the lexicographically
    // smaller name (`collision_detection_fcl/collision_common.cpp:240-242`, `:564-567`). Both cases
    // below are built so iteration order is the *reverse* of sorted order,
    // since a pair that is already sorted cannot tell the two apart.

    #[test]
    fn self_collision_contacts_are_keyed_smaller_name_first() {
        // Two *links* cannot reverse the order: `robot_bodies` walks
        // `link_models()`, which is built from joints sorted by joint name
        // (`robot_model.rs:196-197`), and `build_model` derives `joint_<link>`
        // -- so link order always agrees with name order. An attached body
        // can, because `robot_bodies` chains links first and attached bodies
        // after: `a` on link `z` reaches `self_pairs` as `(z, a)`.
        let model = build_model(&["z"]);
        let mut state = state_with_links_at(&model, &[("z", Isometry3::identity())]);
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let shapes = vec![Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))];
        let shape_poses = vec![Isometry3::translation(0.5, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "a",
            link_name: "z",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 10,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[attached], None);

        assert!(result.collision, "the attached box overlaps its own link");
        let contacts = result.contacts.expect("contacts requested");
        let keys: Vec<_> = contacts.by_pair.keys().cloned().collect();
        assert_eq!(keys, vec![("a".to_string(), "z".to_string())]);
    }

    #[test]
    fn robot_world_distances_are_keyed_smaller_name_first() {
        // `cross_pairs(robot, world)` yields `(z, a)`; upstream files `(a, z)`.
        let model = build_model(&["z"]);
        let mut state = state_with_links_at(&model, &[("z", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "a",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(1.5, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = DistanceRequest {
            request_type: DistanceRequestType::Single,
            ..DistanceRequest::default()
        };

        let result = env.distance_robot(&request, &posed, &[]);

        let keys: Vec<_> = result.distances.keys().cloned().collect();
        assert_eq!(keys, vec![("a".to_string(), "z".to_string())]);
    }

    // `CollisionRequest::distance` -- upstream's trailing `if (req.distance)`
    // block, factored into [`attach_requested_distance`]. Both entry points
    // carry it upstream (`collision_env_fcl.cpp:283-297` and `:340-354`), and
    // both are asserted here: this backend previously honoured it on neither,
    // and a per-entry-point implementation is how one of them regains it while
    // the other stays silent.

    /// The `distance` field, asserting it is the undetailed variant on the
    /// way past -- `CollisionDistance::distance()` reads through both, so a
    /// test using it could not tell `detailed_distance` was ignored.
    fn closest_distance(result: &CollisionResult) -> f64 {
        match result.distance {
            Some(CollisionDistance::Closest(distance)) => distance,
            ref other => panic!("expected CollisionDistance::Closest, got {other:?}"),
        }
    }

    #[test]
    fn check_self_collision_distance_is_none_when_not_requested() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(1.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();

        let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], None);

        assert!(result.distance.is_none());
    }

    #[test]
    fn check_self_collision_distance_reports_the_closest_separation() {
        // Two unit cubes centred 1.5 apart along x: faces at x = 0.5 and
        // x = 1.0, so the gap is exactly 0.5.
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(1.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            distance: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(!result.collision);
        assert_eq!(closest_distance(&result), 0.5);
    }

    #[test]
    fn check_robot_collision_distance_reports_the_closest_separation() {
        // The world-side entry point, separately: it is a distinct upstream
        // block calling `distanceRobot` rather than `distanceSelf`.
        let model = build_model(&["p"]);
        let mut state = state_with_links_at(&model, &[("p", Isometry3::identity())]);
        let posed = state.update();
        let mut world = World::new();
        world.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(1.5, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let request = CollisionRequest {
            distance: true,
            ..CollisionRequest::default()
        };

        let result = env.check_robot_collision(&request, &posed, &[], None);

        assert!(!result.collision);
        assert_eq!(closest_distance(&result), 0.5);
    }

    #[test]
    fn check_self_collision_detailed_distance_reports_the_whole_result() {
        let model = build_model(&["p", "q"]);
        let mut state = state_with_links_at(
            &model,
            &[
                ("p", Isometry3::identity()),
                ("q", Isometry3::translation(1.5, 0.0, 0.0)),
            ],
        );
        let posed = state.update();
        let env = ParryCollisionEnv::default();
        let request = CollisionRequest {
            distance: true,
            detailed_distance: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        let Some(CollisionDistance::Detailed(detailed)) = result.distance else {
            panic!(
                "detailed_distance must select the Detailed variant, got {:?}",
                result.distance
            );
        };
        assert_eq!(detailed.minimum_distance.distance, 0.5);
        let mut names = detailed.minimum_distance.link_names.clone();
        names.sort();
        assert_eq!(names, ["p".to_string(), "q".to_string()]);
    }

    #[test]
    fn check_self_collision_distance_of_a_penetrating_pair_is_unsigned() {
        // The follow-up request carries `DistanceRequest`'s defaults, which
        // include `enable_signed_distance: false` -- so a penetrating pair
        // reports `0.0` here, not a depth. Upstream's block builds the same
        // default-constructed `DistanceRequest`, so this is its answer too;
        // a caller wanting the signed depth must call `distance_self`
        // directly with the flag set, as `distance_robot`'s own tests do.
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
            distance: true,
            ..CollisionRequest::default()
        };

        let result = env.check_self_collision(&request, &posed, &[], None);

        assert!(result.collision, "the cubes overlap by 0.5");
        assert_eq!(closest_distance(&result), 0.0);
    }

    /// Why [`accumulate_collision`]'s `None`-but-touching branch is gated the
    /// way it is, measured on the two configurations that pull in opposite
    /// directions.
    ///
    /// `parry` answers "do these two touch?" with two different queries, and
    /// they disagree in *both* directions -- each one is the only correct
    /// answer for a case this crate already pins, so neither can replace the
    /// other outright:
    ///
    /// | pair | gap | `contact(_, 0.0)` | `intersection_test` |
    /// |---|---|---|---|
    /// | `sphere × sphere`, exact tangency | exactly `0` | `None` | `true` |
    /// | octree leaf face on the robot box | `+4.129349354679189e-17` | `Some` | `false` |
    ///
    /// Row 1: `contact_ball_ball` takes the boundary with a strict `<`, so
    /// `query::contact` alone answers `false` for `sphere × sphere` at exact
    /// tangency. [`query::intersection_test`] has no such gap -- it is `true`
    /// for all 25 pairs [`crate`]'s own tangency table covers, because
    /// `intersection_test_ball_ball` takes the same boundary with `<=`.
    /// `accumulate_collision`'s `None` branch is exactly this row's fix:
    /// gated on [`fcl_tangency_verdict`] saying the pair's [`TangencyKind`]
    /// collides (`sphere`/`sphere` does) and on `intersection_test`
    /// confirming the tie, it sets `collision = true` with no `Contact`
    /// recorded (see that branch's own comment for why none exists to
    /// record).
    ///
    /// Row 2 is why that branch is gated rather than a blanket
    /// `contact(_, 0.0).is_some() || intersection_test`. The octree leaf's
    /// `-x` face and the box's `+x` face are one leaf-quantisation ulp apart
    /// rather than exactly equal, so the pair sits in strictly positive air.
    /// [`query::intersection_test`] is exactly "gap `<= 0`" and rejects it;
    /// only the positive margin `query::contact` carries at prediction `0.0`
    /// admits it, and upstream calls that pair a collision
    /// (`octree_world_collision_response.json` case 4, `robot_collision: true`).
    /// A blanket union would still answer this row correctly (`Some`, so the
    /// union takes the `Some` branch and never even evaluates
    /// `intersection_test`) -- row 2 is not why the union is *wrong*; it is
    /// why substituting `intersection_test` for `query::contact` everywhere
    /// is: that substitution fails this fixture with `id 4: robot_collision
    /// mismatch`, alongside
    /// `check_robot_collision_touching_exactly_on_an_octree_leaf_face_is_detected`
    /// and `exact_tangency_boundary`'s `the_collision_boundary_sits_in_a_positive_gap`
    /// -- the same case §229.2's `contact.dist <= 0.0` gate broke, and for the
    /// same reason: both remove the margin.
    ///
    /// The real reason the `None` branch is *gated* rather than an
    /// unconditional union: upstream can never report a collision with zero
    /// contacts (`res_->collision = true` sits inside `if (num_contacts > 0)`
    /// in all three branches of `collisionCallback`
    /// (`moveit_core/collision_detection_fcl/src/collision_common.cpp:269`,
    /// `moveit_core/collision_detection_fcl/src/collision_common.cpp:332`,
    /// `moveit_core/collision_detection_fcl/src/collision_common.cpp:367`),
    /// and the first of those reaches it only through a `DecideContactFn`
    /// call made once per contact
    /// (`moveit_core/collision_detection_fcl/src/collision_common.cpp:243-247`)
    /// -- so an unconditional union would bypass
    /// `AllowedCollision::Conditional` wholesale, turning any predicate's
    /// `true` (allowed) into an unreviewable `true` (collision) the moment a
    /// pair's own boolean queries happened to disagree this way. Restricting
    /// the branch to kind pairs [`fcl_tangency_verdict`] says collide, and
    /// excluding `Conditional` outright, keeps that bypass from reaching any
    /// pair this table has no measurement calling a collision.
    #[test]
    fn parry_boolean_queries_disagree_in_both_directions_at_the_tie() {
        let cache = OctreeCache::default();

        // Two balls of radius `0.5` whose centres are exactly `2 * 0.5` apart:
        // the tie is exact in binary, not an approximation of one.
        let sphere = Shape::Sphere(Sphere::new(0.5).expect("sphere"));
        let (upper, upper_fix) = convert_shape(&sphere, &cache).expect("sphere converts");
        let (lower, lower_fix) = convert_shape(&sphere, &cache).expect("sphere converts");
        let upper_pose = to_pose(Isometry3::translation(0.0, 0.0, 0.5) * upper_fix);
        let lower_pose = to_pose(Isometry3::translation(0.0, 0.0, -0.5) * lower_fix);
        if let Some(found) = query::contact(&upper_pose, &*upper, &lower_pose, &*lower, 0.0)
            .expect("ball/ball is dispatched")
        {
            panic!(
                "contact_ball_ball's strict `<` must still exclude the exact tie, \
                 but it yielded a contact at dist {:e}",
                found.dist
            );
        }
        assert!(
            query::intersection_test(&upper_pose, &*upper, &lower_pose, &*lower)
                .expect("ball/ball is dispatched"),
            "intersection_test_ball_ball's `<=` must still include the exact tie"
        );

        // The same configuration as
        // `check_robot_collision_touching_exactly_on_an_octree_leaf_face_is_detected`,
        // read one level down: a 0.1 leaf centred at x=0.55 against the 1x1x1
        // box at the origin.
        let mut tree = moveit_octomap::OcTree::new(0.1);
        tree.update_node(nalgebra::Point3::new(0.55, 0.0, 0.0), true, false);
        let leaf = Shape::OcTree(OcTree::from_tree(Arc::new(tree)));
        let (leaf_shape, leaf_fix) = convert_shape(&leaf, &cache).expect("leaf converts");
        let robot_box = Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).expect("cuboid"));
        let (box_shape, box_fix) = convert_shape(&robot_box, &cache).expect("cuboid converts");
        let leaf_pose = to_pose(leaf_fix);
        let box_pose = to_pose(box_fix);
        let gap = query::contact(&box_pose, &*box_shape, &leaf_pose, &*leaf_shape, 0.0)
            .expect("cuboid/compound is dispatched")
            .expect("the positive margin admits this pair")
            .dist;
        // Measured `4.129349354679189e-17`. The load-bearing half is `> 0.0`:
        // it is what makes every "gap `<= 0`" predicate reject this pair. The
        // upper bound states the magnitude with a ~24x margin over the measured
        // value, so a real change in the leaf's placement is not absorbed.
        assert!(
            gap > 0.0 && gap < 1e-15,
            "the leaf face must sit in strictly positive air, near 4.13e-17, not {gap:e}"
        );
        assert!(
            !query::intersection_test(&box_pose, &*box_shape, &leaf_pose, &*leaf_shape)
                .expect("cuboid/compound is dispatched"),
            "intersection_test is exactly `gap <= 0` and must reject this pair"
        );
    }

    /// [`TIE_ROUNDING_MARGIN`]'s own evidence: reproduces
    /// `exact_tangency_is_decided_per_shape_pair.rs`'s construction (every
    /// [`TangencyKind`] pair, both orders, tangent at `delta == 0.0`) at five
    /// scales spanning five orders of magnitude, and pins both margins the
    /// constant has to clear.
    ///
    /// Upper margin: every one of those genuine exact ties -- GJK's own
    /// rounding on a `dist` whose true value is `0.0` -- must measure under
    /// the constant. The largest observed here is `1.92` (`cylinder x
    /// cylinder`, scale `50.0`); four pairs this crate has documented before
    /// (`box x cylinder`, `cylinder x box`, `box x cone`, `cone x box`) also
    /// show it, all under `1`. `TIE_ROUNDING_MARGIN`'s `16.0` is set above
    /// that measurement, not equal to it, so a marginally worse rounding
    /// case on a shape or scale this sweep does not cover still clears the
    /// bound rather than silently reclassifying as a real distance.
    ///
    /// Lower margin: a delta an order of magnitude above the constant must
    /// still measure as *itself*, not get rounded away into the tie bucket.
    /// `160.0 * f64::EPSILON * scale` on `box x cylinder` (one of the four
    /// pairs with nonzero rounding at `delta == 0.0`, so the hardest case to
    /// keep distinct) is confirmed close to its injected value at every
    /// scale tried -- proving the margin has not silently drifted wide
    /// enough to swallow a real, if tiny, clearance.
    ///
    /// What this file's own measurement found and the margin does *not* try
    /// to encode: there is no wide, empty gap between "rounding noise" and
    /// "a real clearance report" the way a first read of this constant's
    /// name might suggest. `dist` tracks an injected delta accurately from
    /// a few `EPS * scale` up through roughly `1e6`-`1e7` `EPS * scale`
    /// (`query::contact`'s own reach before it starts returning `None`,
    /// matching the order of `parry3d_f64::query::gjk::gjk::eps_tol`'s
    /// `10.0 * f64::EPSILON` convergence tolerance) -- a continuum, not two
    /// separated clusters. `TIE_ROUNDING_MARGIN` only has to sit above the
    /// rounding ceiling and below anything this crate treats as a
    /// meaningfully different clearance (§229.2's own `~5e-8 m` positive
    /// margin, itself far above `16.0 * EPS * scale` for any shape size this
    /// workspace builds), not at the edge of a cliff either side of it.
    ///
    /// Separately and not part of this bound at all: an isolated,
    /// reproducible `query::contact` anomaly exists for `cylinder x box`/
    /// `cone x box` at `scale == 500.0`, `delta` in roughly `[2.6, 3.55] *
    /// EPS * scale` -- `dist` comes back `-4.08`, seven-plus orders of
    /// magnitude too large, instead of the near-zero value every
    /// neighbouring `delta` in that sweep produces. No `TIE_ROUNDING_MARGIN`
    /// choice can route around it (the reported magnitude is nowhere near
    /// any sane margin, in either direction), and it does not regress
    /// anything this change touches: `dist <= 0.0` misjudges that pair
    /// identically before and after, since old code took *any* `Some` as a
    /// touch regardless of `dist`'s value. Filed as a `parry3d_f64`
    /// numerical-robustness gap, not this crate's dispatch logic, and out of
    /// this change's scope.
    #[test]
    fn tie_rounding_margin_clears_measured_ties_and_does_not_swallow_a_real_delta() {
        #[derive(Clone, Copy, Debug)]
        enum K {
            Box,
            Sphere,
            Cylinder,
            Cone,
        }
        impl K {
            fn shape(self, half: f64) -> Shape {
                match self {
                    K::Box => {
                        Shape::Cuboid(Cuboid::new(2.0 * half, 2.0 * half, 2.0 * half).unwrap())
                    }
                    K::Sphere => Shape::Sphere(Sphere::new(half).unwrap()),
                    K::Cylinder => {
                        Shape::Cylinder(moveit_geometry::Cylinder::new(half, 2.0 * half).unwrap())
                    }
                    K::Cone => Shape::Cone(moveit_geometry::Cone::new(half, 2.0 * half).unwrap()),
                }
            }
            fn name(self) -> &'static str {
                match self {
                    K::Box => "box",
                    K::Sphere => "sphere",
                    K::Cylinder => "cylinder",
                    K::Cone => "cone",
                }
            }
        }
        const KINDS: [K; 4] = [K::Box, K::Sphere, K::Cylinder, K::Cone];
        // Five orders of magnitude, matching this constant's own doc.
        const SCALES: [f64; 5] = [0.005, 0.05, 0.5, 50.0, 500.0];
        let cache = OctreeCache::default();

        let pair_at = |half: f64, upper: K, lower: K, delta: f64| {
            let (u, u_fix) = convert_shape(&upper.shape(half), &cache).expect("converts");
            let (l, l_fix) = convert_shape(&lower.shape(half), &cache).expect("converts");
            let u_pose = to_pose(Isometry3::translation(0.0, 0.0, 2.0 * half + delta) * u_fix);
            let l_pose = to_pose(Isometry3::translation(0.0, 0.0, 0.0) * l_fix);
            let scale = tie_scale(&u_pose, &*u, &l_pose, &*l);
            let dist = query::contact(&u_pose, &*u, &l_pose, &*l, 0.0)
                .expect("dispatched")
                .map(|c| c.dist);
            (dist, scale)
        };

        // Upper margin: no genuine exact tie may reach the constant.
        let mut worst: Option<(f64, K, K, f64)> = None;
        for &half in &SCALES {
            for upper in KINDS {
                for lower in KINDS {
                    let (dist, scale) = pair_at(half, upper, lower, 0.0);
                    let Some(dist) = dist else { continue }; // sphere x sphere: strict `<`, no Some
                    let units = dist.abs() / (f64::EPSILON * scale);
                    if worst.is_none_or(|(w, ..)| units > w) {
                        worst = Some((units, upper, lower, half));
                    }
                }
            }
        }
        let (worst_units, upper, lower, half) = worst.expect("at least one pair measured");
        assert!(
            worst_units < TIE_ROUNDING_MARGIN,
            "{} x {} at scale {half:e} measured {worst_units:e} EPS*scale of rounding at an \
             exact tie, at or above TIE_ROUNDING_MARGIN ({TIE_ROUNDING_MARGIN:e}) -- widen the \
             margin (with a new doc citation) or investigate why rounding got worse",
            upper.name(),
            lower.name(),
        );

        // Lower margin: a real delta an order of magnitude above the margin
        // must not be rounded away into the tie bucket, on the pair hardest
        // to tell apart from one (it already has nonzero rounding at
        // delta == 0.0).
        for &half in &SCALES {
            let delta = 10.0 * TIE_ROUNDING_MARGIN * f64::EPSILON * half;
            let (dist, scale) = pair_at(half, K::Box, K::Cylinder, delta);
            let dist = dist.expect("a real positive gap this small still returns Some (§229.2)");
            let relative_error = ((dist - delta) / delta).abs();
            assert!(
                relative_error < 0.05,
                "box x cylinder at scale {half:e}: injected delta {delta:e}, measured dist \
                 {dist:e} (scale {scale:e}) -- a real delta ten margins above \
                 TIE_ROUNDING_MARGIN must be reported close to itself, not rounded toward zero"
            );
        }
    }

    /// An exact tie stays an exact tie when the pair is not at the origin.
    ///
    /// The sweep above never varies this: its `pair_at` puts the lower shape
    /// at `translation(0, 0, 0)`, so world-coordinate magnitude and shape size
    /// are the same number in every case it measures, and a [`tie_scale`]
    /// built from half-extents alone fit that data exactly while being wrong
    /// everywhere off it.
    ///
    /// What goes wrong off the origin is not that GJK's rounding grows -- it
    /// does not; placed on exactly-representable coordinates the same pair
    /// measures the same `dist` at every offset. It is that a tangency
    /// *cannot be placed* far from the origin: `100.0 + 0.1` is not a
    /// representable sum, so the stored pair misses tangency by up to half a
    /// coordinate ULP, which at 100 m is 1.4e-14 while the shapes' own
    /// half-extent spacing is 1.1e-17. A separation below the spacing of the
    /// coordinates the pair is expressed in is not distinguishable from an
    /// exact tie by any arithmetic on those poses, so it is one, and the band
    /// that decides which comparisons are ties has to be measured in that
    /// spacing.
    ///
    /// So this asserts the invariant directly: **where the pair sits within
    /// one coordinate ULP of tangency, the verdict is the dispatch table's**,
    /// at every offset. The expectation is [`fcl_tangency_verdict`] rather
    /// than a transcribed table so it cannot drift from the generated one.
    ///
    /// On the half-extent-only `tie_scale` this reaches 59 tie placements
    /// (14 at the origin, 45 away from it) and 20 of them disagree: all ten
    /// of the table's `false` cells, at 10 m and at 100 m, each answered
    /// "touching" because the sign of a `-3.7e-16` / `-5.7e-15` miss rounds
    /// negative, `-3.6e-16` being `32.5` half-extent units against a `16.0`
    /// band and `0.20` of a coordinate ULP. The `true` cells are unaffected
    /// — the sign happens to agree with them — so a test that only checked
    /// pairs the table calls touching would pass on the defect.
    ///
    /// `None` from the production query (`prediction = 0.0`) is a verdict, not
    /// a skip: it is the "apart" answer, and is scored as `false`.
    #[test]
    fn an_exact_tie_is_decided_by_the_dispatch_table_however_far_from_the_origin() {
        #[derive(Clone, Copy, Debug, PartialEq)]
        enum K {
            Box,
            Sphere,
            Cylinder,
            Cone,
        }
        impl K {
            fn shape(self, half: f64) -> Shape {
                match self {
                    K::Box => {
                        Shape::Cuboid(Cuboid::new(2.0 * half, 2.0 * half, 2.0 * half).unwrap())
                    }
                    K::Sphere => Shape::Sphere(Sphere::new(half).unwrap()),
                    K::Cylinder => {
                        Shape::Cylinder(moveit_geometry::Cylinder::new(half, 2.0 * half).unwrap())
                    }
                    K::Cone => Shape::Cone(moveit_geometry::Cone::new(half, 2.0 * half).unwrap()),
                }
            }
            fn name(self) -> &'static str {
                match self {
                    K::Box => "box",
                    K::Sphere => "sphere",
                    K::Cylinder => "cylinder",
                    K::Cone => "cone",
                }
            }
        }
        const KINDS: [K; 4] = [K::Box, K::Sphere, K::Cylinder, K::Cone];
        // 0.0 reproduces the sweep above; the rest are ordinary placements --
        // 10 m is well inside a normal octomap extent, not a stress value.
        const OFFSETS: [f64; 4] = [0.0, 0.5, 10.0, 100.0];
        const HALF: f64 = 0.05;
        let cache = OctreeCache::default();

        // Cases that reached the assertion, split by whether the offset was
        // the one the sweep above already covers. A band bug that made every
        // placement read as a resolvable gap would otherwise pass this test
        // having tested nothing.
        let mut ties_at_origin = 0usize;
        let mut ties_off_origin = 0usize;

        for upper in KINDS {
            for lower in KINDS {
                // parry's `contact_ball_ball` compares the gap strictly
                // against the prediction, so an exact sphere/sphere tangency
                // is `None` at production's `prediction = 0.0` and never
                // reaches `touches_at_tie` at all, while the table says
                // `true`. That disagreement is a distinct defect in a
                // different function, present at the origin and unchanged by
                // any band, so it is excluded here rather than left to fail
                // this test for a reason this test is not about. The
                // assertion below the loop is what stops the exclusion from
                // outliving the defect.
                if upper == K::Sphere && lower == K::Sphere {
                    continue;
                }
                let (u, u_fix) = convert_shape(&upper.shape(HALF), &cache).expect("converts");
                let (l, l_fix) = convert_shape(&lower.shape(HALF), &cache).expect("converts");
                let table = fcl_tangency_verdict(&*u, &*l).expect("both are specialised shapes");
                for off in OFFSETS {
                    let base = Isometry3::translation(off, off, off);
                    let u_pose =
                        to_pose(base * Isometry3::translation(0.0, 0.0, 2.0 * HALF) * u_fix);
                    let l_pose = to_pose(base * l_fix);

                    // The spacing of the coordinates the poses are stored in.
                    let coord = off.abs().max(2.0 * HALF);
                    let ulp = coord.next_up() - coord;

                    // `prediction = 0.0` hides a positive gap behind `None`,
                    // so classify with a wider one: is this placement within a
                    // ULP of tangency, or a gap the arithmetic can resolve?
                    let gap = query::contact(&u_pose, &*u, &l_pose, &*l, 4.0 * ulp)
                        .expect("dispatched")
                        .map(|c| c.dist);
                    let Some(gap) = gap else { continue };
                    if gap.abs() > ulp {
                        continue; // a real, resolvable gap: not a tie
                    }
                    if off == 0.0 {
                        ties_at_origin += 1;
                    } else {
                        ties_off_origin += 1;
                    }

                    // What production answers, through the query it really makes.
                    let got = match query::contact(&u_pose, &*u, &l_pose, &*l, 0.0)
                        .expect("dispatched")
                    {
                        Some(c) => touches_at_tie(c.dist, &u_pose, &*u, &l_pose, &*l),
                        None => false,
                    };
                    assert_eq!(
                        got,
                        table,
                        "{} x {} sits {gap:e} from tangency at offset {off}: {:.2} of the \
                         {ulp:e} spacing of its own coordinates, but {:.1} EPS * half-extent \
                         units, which is what a half-extent-only tie_scale would compare \
                         against the {TIE_ROUNDING_MARGIN:.1} band. Inside one coordinate ULP \
                         the dispatch table decides it; tie_scale was {:e}",
                        upper.name(),
                        lower.name(),
                        gap.abs() / ulp,
                        gap.abs() / (f64::EPSILON * HALF),
                        tie_scale(&u_pose, &*u, &l_pose, &*l),
                    );
                }
            }
        }
        assert!(
            ties_at_origin > 0 && ties_off_origin > 0,
            "this test asserted on {ties_at_origin} tie(s) at the origin and \
             {ties_off_origin} away from it -- it must exercise both or it is not \
             comparing the two"
        );

        // The sphere/sphere exclusion above, kept honest: if parry starts
        // reporting that contact, delete the exclusion rather than this.
        let (s, s_fix) = convert_shape(&K::Sphere.shape(HALF), &cache).expect("converts");
        let up = to_pose(Isometry3::translation(0.0, 0.0, 2.0 * HALF) * s_fix);
        let lo = to_pose(s_fix);
        assert!(
            query::contact(&up, &*s, &lo, &*s, 0.0)
                .expect("dispatched")
                .is_none(),
            "parry now reports the sphere x sphere tangency -- drop the exclusion in this \
             test and let the pair be asserted like every other"
        );
    }
}
