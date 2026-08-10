// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The bridge from a sampled [`JointModelGroupSpace`] state to a real
//! collision/constraint check: [`PlanningSceneValidityChecker`] writes a
//! candidate sample into a [`PlanningScene`]'s current state and asks
//! [`PlanningScene::is_state_valid`] — the collision-then-constraints
//! composition `p1-fixtures` ported this round — rather than re-deriving
//! that composition here.

use std::cell::RefCell;

use cspace_collision::{CollisionEnv, CollisionRequest};
use cspace_constraints::KinematicConstraintSet;
use cspace_core::state::Posed;
use cspace_scene::PlanningScene;

use crate::compound::CompoundValue;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::space::StateSpace;
use crate::validity::StateValidityChecker;

/// Checks a [`JointModelGroupSpace`] sample against a real
/// [`PlanningScene`]: collision (via `env`) and, if given, `constraints`.
///
/// # Why `RefCell`
///
/// [`PlanningScene::is_state_valid`] takes `&mut self` —
/// `current_state_mut` materializes an inherited state and
/// `check_collision` calls [`cspace_core::state::RobotState::update`], both
/// mutating operations — but [`StateValidityChecker::is_valid`] takes
/// `&self`: every existing implementor ([`crate::validity::DiscreteMotionValidator`]
/// and the blanket `Fn` impl) is immutable, and [`crate::rrt_connect::rrt_connect`]
/// holds its checker as a shared `&C` across the whole search, alongside a
/// nearest-neighbour index it also only ever borrows immutably — there is
/// no `&mut C` to thread through that loop without restructuring it for
/// this one caller. `RefCell` adapts the mutable scene to that
/// shared-reference contract instead. Nothing here calls
/// [`StateValidityChecker::is_valid`] from more than one thread or
/// re-enters it while a borrow is outstanding (a single sampling-planner
/// search is sequential), so the runtime check `RefCell` adds never
/// actually panics — this module's tests exercise exactly that call
/// pattern end to end, through [`crate::rrt_connect::rrt_connect`] itself,
/// not just a single direct call.
///
/// # No state pooling
///
/// Every [`PlanningSceneValidityChecker::is_valid`] call writes into the
/// scene's *existing* current [`cspace_core::state::RobotState`]
/// ([`JointModelGroupSpace::write_robot_state`]) rather than constructing a
/// new one — there is no `RobotState::new` anywhere in this type's hot
/// path. The cost this type does *not* avoid is
/// [`cspace_core::state::RobotState::set_variable_positions`]'s own
/// `positions().to_vec()` (one `Vec<f64>` clone of the model's full
/// variable count, not just this group's, on every call) plus whatever
/// forward-kinematics work [`PlanningScene::is_state_valid`]'s
/// `RobotState::update` performs the first time this sample's transforms
/// are actually read. Measured by
/// `cargo run --example planning_scene_validity_bench -p cspace-planners-sbp`
/// (panda_arm, 7 DoF, real mesh-loaded collision geometry via
/// `fixture_mesh_search_paths`, empty world, no constraints, 50 calls;
/// add `--release` for the optimized figure): **dev mean 2.27 ms/call
/// (min 1.05, max 6.26), release mean 1.89 ms/call (min 0.95, max 5.16)**.
/// This is a total, not a breakdown — no profiling was done to say how
/// much is the `Vec<f64>` clone versus FK versus the real self-collision
/// mesh checks [`cspace_collision::ParryCollisionEnv`] now actually
/// performs.
///
/// Those two figures used to be `debug ~8-15 ms` against `release ~1-6 ms`,
/// and the argument below was built on the gap between them. `e733f19` gave
/// the workspace a `[profile.dev]` (`opt-level = 1`, `2` for dependencies),
/// which closed it: dev is now 1.20x release, not 6x, and the numbers above
/// were re-measured under that profile rather than scaled from the old ones.
/// (The old debug figure was itself three orders of magnitude past an even
/// earlier no-mesh-geometry measurement this comment once cited, whose scene
/// had no collision shapes loaded at all — see this crate's commit history.)
///
/// The collapse strengthens the conclusion rather than retiring it. There is
/// now effectively one regime, so a single `Termination::Iterations(20_000)`
/// RRT-Connect query against real panda geometry costs on the order of
/// 20,000 x ~2 ms ≈ 40 s at the mean and 20,000 x ~5-6 ms ≈ 2 min at the
/// max-observed per-call cost, in *either* profile — a real planning-latency
/// concern for real-geometry queries specifically, and now measured not to
/// be a debug-profile artifact at all, since removing the debug tax left it
/// where it was. No pooling scheme is built against it yet — pooling would not
/// address mesh collision cost at all (it only avoids the `Vec<f64>`
/// clone, a small fraction of this total), and nothing here has measured
/// what would. This module's own
/// `is_valid_does_not_regress_by_orders_of_magnitude` test guards only
/// against a catastrophic regression in this cost, not against exceeding
/// it — see that test's doc comment for why the bound it asserts is loose
/// on purpose.
pub struct PlanningSceneValidityChecker<'a, 'm, E> {
    scene: RefCell<&'a mut PlanningScene<'m>>,
    env: &'a E,
    request: CollisionRequest,
    constraints: Option<&'a KinematicConstraintSet>,
    space: &'a JointModelGroupSpace,
}

impl<'a, 'm, E> PlanningSceneValidityChecker<'a, 'm, E> {
    /// Checks each sample by writing it into `scene`'s current state (via
    /// `space`) and calling `scene.is_state_valid(env, &request,
    /// constraints)`. `constraints` mirrors upstream's `path_constraints`
    /// (`None` means unconstrained, matching
    /// [`PlanningScene::is_state_valid`]'s own `constraints` parameter).
    pub fn new(
        scene: &'a mut PlanningScene<'m>,
        env: &'a E,
        request: CollisionRequest,
        constraints: Option<&'a KinematicConstraintSet>,
        space: &'a JointModelGroupSpace,
    ) -> Self {
        Self {
            scene: RefCell::new(scene),
            env,
            request,
            constraints,
            space,
        }
    }
}

impl<'a, 'm, E> StateValidityChecker<JointModelGroupSpace>
    for PlanningSceneValidityChecker<'a, 'm, E>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    /// # Side effect
    ///
    /// Leaves the scene's current state at whatever `state` was last
    /// checked — it is not restored afterward, unlike
    /// [`PlanningScene::is_path_valid`]'s own per-waypoint save/restore.
    /// Restoring here would cost a second full-state clone on every one of
    /// the hundreds of thousands of calls a single planning query makes
    /// ([`PlanningSceneValidityChecker`]'s own doc comment measures the
    /// unavoidable per-call cost already), for a property
    /// ([`PlanningScene::current_state`] reading back whatever it held
    /// before planning started) nothing in this crate's planning path
    /// relies on: a caller that needs the pre-planning state preserved
    /// clones it once, itself, before handing the scene to this type.
    fn is_valid(&self, state: &Vec<CompoundValue>) -> bool {
        // Bounds first, matching OMPL's own `StateValidityChecker` ordering:
        // `write_robot_state` writes `state` into the scene's `RobotState`
        // with no bounds enforcement of its own (mirroring
        // `RobotState::set_variable_position`), and
        // `PlanningScene::is_state_valid`'s collision-then-constraints
        // composition has nothing that checks joint limits either -- a
        // grossly out-of-limit sample that happens to be collision-free and
        // constraint-satisfying would otherwise read as valid.
        if !self.space.satisfies_bounds(state) {
            return false;
        }
        let mut scene = self.scene.borrow_mut();
        self.space
            .write_robot_state(state, scene.current_state_mut());
        scene.is_state_valid(self.env, &self.request, self.constraints)
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use cspace_collision::{LinkPaddingScale, ParryCollisionEnv};
    use cspace_constraints::{
        Constraint, JointConstraint, KinematicConstraintSet, PositionConstraint,
    };
    use cspace_core::geometry::Shape;
    use cspace_core::geometry::shapes::Sphere;
    use cspace_core::geometry::{Isometry3, Vector3};
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_scene::PlanningScene;

    use super::*;
    use crate::joint_model_group_space::JointModelGroupSpace;

    /// The `moveit_resources_panda_description` package committed under
    /// `fixtures/meshes/` (see `tools/ci/verify-fixture-provenance.sh` and
    /// `cspace-collision`'s `collision_parity` integration test, which
    /// established this exact pattern) — lets [`load_panda`] resolve
    /// panda's real `<mesh>` collision geometry instead of skipping it.
    fn fixture_mesh_search_paths() -> MeshSearchPaths {
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )])
    }

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// panda.srdf's own `"ready"` named `<group_state>` for `panda_arm`
    /// (`panda_joint2 = -0.785`, `panda_joint4 = -2.356`,
    /// `panda_joint6 = 1.571`, `panda_joint7 = 0.785`, the rest 0) — unlike
    /// the all-default (every joint 0) state, which is a genuinely
    /// self-colliding configuration for panda's real collision meshes (the
    /// oracle-verified `panda_collision.json` fixture's `joint_values: {}`
    /// case records `self_collision: true`), `"ready"` is moveit's own
    /// designed non-self-colliding demo pose, so it isolates
    /// environment-collision behaviour from self-collision.
    fn ready_state(model: &RobotModel) -> cspace_core::state::RobotState<'_> {
        let mut state = cspace_core::state::RobotState::new(model);
        state.set_to_default_values();
        for (name, value) in [
            ("panda_joint1", 0.0),
            ("panda_joint2", -0.785),
            ("panda_joint3", 0.0),
            ("panda_joint4", -2.356),
            ("panda_joint5", 0.0),
            ("panda_joint6", 1.571),
            ("panda_joint7", 0.785),
        ] {
            state.set_variable_position(name, value).unwrap();
        }
        state
    }

    /// A world containing a sphere at `panda_link4`'s FK pose for
    /// [`ready_state`], offset by `offset` — a sphere big enough that
    /// `offset = identity` collides with panda_link4's own real,
    /// mesh-loaded collision geometry (via [`fixture_mesh_search_paths`]),
    /// while a large `offset` places it nowhere near the robot.
    fn world_with_obstacle_at_ready_link4_pose(
        model: &RobotModel,
        offset: cspace_core::geometry::Isometry3,
    ) -> cspace_collision::World {
        let mut state = ready_state(model);
        let pose = offset * state.update().global_link_transform("panda_link4").unwrap();
        let mut world = cspace_collision::World::new();
        world.add_shape(
            "blocker",
            std::sync::Arc::new(Shape::Sphere(Sphere::new(0.3).unwrap())),
            pose,
        );
        world
    }

    #[test]
    fn a_colliding_sample_is_rejected_and_a_clear_one_is_accepted() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let ready = space.read_robot_state(&ready_state(&model));

        // "ready" must collide with a sphere centered on its own
        // panda_link4 pose.
        let mut colliding_scene = PlanningScene::new(&model, &srdf);
        let colliding_world = world_with_obstacle_at_ready_link4_pose(
            &model,
            cspace_core::geometry::Isometry3::identity(),
        );
        let colliding_env = ParryCollisionEnv::new(colliding_world, LinkPaddingScale::default());
        let colliding_checker = PlanningSceneValidityChecker::new(
            &mut colliding_scene,
            &colliding_env,
            cspace_collision::CollisionRequest::default(),
            None,
            &space,
        );
        assert!(
            !colliding_checker.is_valid(&ready),
            "\"ready\" must collide with a sphere centered on its own panda_link4 pose"
        );

        // The same "ready" state, with the same sphere translated 10m away,
        // must be collision-free.
        let mut clear_scene = PlanningScene::new(&model, &srdf);
        let clear_world = world_with_obstacle_at_ready_link4_pose(
            &model,
            cspace_core::geometry::Isometry3::translation(10.0, 0.0, 0.0),
        );
        let clear_env = ParryCollisionEnv::new(clear_world, LinkPaddingScale::default());
        let clear_checker = PlanningSceneValidityChecker::new(
            &mut clear_scene,
            &clear_env,
            cspace_collision::CollisionRequest::default(),
            None,
            &space,
        );
        assert!(
            clear_checker.is_valid(&ready),
            "the same state must be collision-free once the obstacle is moved 10m away"
        );
    }

    #[test]
    fn a_state_grossly_outside_joint_limits_is_rejected_even_with_no_collision() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());

        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            cspace_collision::CollisionRequest::default(),
            None,
            &space,
        );

        // panda_joint1's real limit is +/-2.9671 rad (panda.urdf); 100.0 rad
        // is nowhere near reachable. Nothing in the collision/constraint
        // path this checker calls can see that -- only
        // `StateSpace::satisfies_bounds` can, and `RobotState::set_variable_position`
        // (unlike this crate's own request adapters) does not enforce
        // bounds on write.
        let mut out_of_bounds = ready_state(&model);
        out_of_bounds
            .set_variable_position("panda_joint1", 100.0)
            .unwrap();
        let sample = space.read_robot_state(&out_of_bounds);

        assert!(
            !checker.is_valid(&sample),
            "a state 100.0 rad past panda_joint1's real limit of +/-2.9671 rad must not be \
             reported valid"
        );
    }

    #[test]
    fn a_path_constraint_rejects_a_state_the_constraint_forbids_even_with_no_collision() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());

        let constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.01, 0.01, 1.0)
            .expect("valid joint constraint");
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            cspace_collision::CollisionRequest::default(),
            Some(&set),
            &space,
        );

        // "ready" (not the all-default, all-zero state -- see
        // `ready_state`'s doc comment for why that self-collides with
        // panda's real collision meshes) already has `panda_joint1 == 0.0`.
        let ready = ready_state(&model);
        assert!(
            checker.is_valid(&space.read_robot_state(&ready)),
            "panda_joint1 == 0.0 must satisfy a constraint centered on 0.0"
        );

        let mut far = ready_state(&model);
        far.set_variable_position("panda_joint1", 1.0).unwrap();
        assert!(
            !checker.is_valid(&space.read_robot_state(&far)),
            "panda_joint1 == 1.0 must violate a +/-0.01 constraint centered on 0.0, with no collision to blame"
        );
    }

    /// Not a timing measurement (see `examples/planning_scene_validity_bench.rs`
    /// for that) -- a regression guard. The bound is deliberately huge
    /// relative to the measured cost (this type's own doc comment: dev mean
    /// 2.27 ms/call, max observed 6.26 ms) so it never fails from ordinary
    /// machine-speed variance, but still catches an accidental
    /// orders-of-magnitude blowup -- e.g. a change that makes this call
    /// quadratic in variable count, or drops an early-out FCL/`parry` relies
    /// on -- in a normal `cargo nextest run`, without needing anyone to
    /// remember to run the example.
    ///
    /// The 2s bound is left where it was rather than retightened to track
    /// the measurement. What it has to be is far above the worst legitimate
    /// call and far below a pathological one, and it is both; tying it to a
    /// ratio against the current mean would just make it move every time the
    /// build profile does, which is what happened to the stale "~100x the
    /// ~15 ms debug-profile maximum" this comment and the assertion message
    /// used to quote (`e733f19` closed the dev/release gap and made that
    /// figure ~320x overnight, with nothing about the code changed).
    #[test]
    fn is_valid_does_not_regress_by_orders_of_magnitude() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            cspace_collision::CollisionRequest::default(),
            None,
            &space,
        );

        let ready = ready_state(&model);
        let sample = space.read_robot_state(&ready);
        let start = std::time::Instant::now();
        std::hint::black_box(checker.is_valid(&sample));
        let elapsed = start.elapsed();
        assert!(
            elapsed < std::time::Duration::from_secs(2),
            "a single is_valid call took {elapsed:?}, orders of magnitude past the \
             ~6.3 ms worst case this type's doc comment measures -- see \
             examples/planning_scene_validity_bench.rs for a precise re-measurement"
        );
    }

    /// This is a proof, not a wiring: it demonstrates the constructor-side
    /// half of `PlanningScene::transforms_with_world_objects`'s own doc
    /// comment on the one `PositionConstraint::new` call this test builds
    /// itself, but there is no production call site to wire yet. Every
    /// production path that builds a `Transforms` for a goal constraint
    /// (e.g. `construct_goal_pose_constraints` in `cspace-constraints`)
    /// takes `tf: &Transforms` from its own caller rather than deriving it
    /// from a `PlanningScene`; nothing in this workspace yet builds goal
    /// constraints directly from a `PlanningScene`. `cspace-constraints`
    /// cannot depend on `cspace-scene` itself
    /// (`tools/ci/check-dep-direction.sh` would reject the cycle -- collision
    /// checking already flows `cspace-scene -> cspace-constraints`), so
    /// when that production path exists, its call site threading a
    /// `PlanningScene`-derived [`cspace_core::geometry::Transforms`] into
    /// [`cspace_constraints::PositionConstraint::new`] has to live here, in
    /// this crate, the one place that depends on both. Until then, this
    /// test is the only evidence in the tree that `scene.transforms()`
    /// alone fails to resolve a world-object reference frame (upstream's
    /// base, non-scene-aware `Transforms::isFixedFrame` half) while
    /// `scene.transforms_with_world_objects()` reaches the scene-aware
    /// object-frame half `SceneTransforms::isFixedFrame` overrides in.
    #[test]
    fn a_position_constraint_against_a_world_object_only_resolves_through_transforms_with_world_objects()
     {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene.add_shape(
            "table",
            std::sync::Arc::new(Shape::Sphere(Sphere::new(0.1).unwrap())),
            Isometry3::translation(1.0, 0.0, 0.0),
        );

        let bare = scene.transforms();
        let err = PositionConstraint::new(
            &model,
            bare,
            "panda_link8",
            "table",
            Vector3::zeros(),
            &[(
                Shape::Sphere(Sphere::new(0.05).unwrap()),
                Isometry3::identity(),
            )],
            1.0,
        )
        .expect_err(
            "scene.transforms() alone does not know about world objects, so \"table\" must not \
             resolve as a reference frame",
        );
        // PositionConstraint::new has two Error::UnknownName sites (an
        // unknown link_name, an unknown frame_id); matching only the variant
        // cannot tell them apart, so this checks `kind` too -- message-swap
        // bite-checked against the link_name site.
        assert!(
            matches!(
                err,
                cspace_core::error::Error::UnknownName {
                    kind: "frame",
                    ref name
                } if name == "table"
            ),
            "expected UnknownName{{kind: \"frame\", name: \"table\"}}, got {err:?}"
        );

        let with_objects = scene.transforms_with_world_objects();
        let constraint = PositionConstraint::new(
            &model,
            &with_objects,
            "panda_link8",
            "table",
            Vector3::zeros(),
            &[(
                Shape::Sphere(Sphere::new(0.05).unwrap()),
                Isometry3::identity(),
            )],
            1.0,
        )
        .expect("transforms_with_world_objects() must resolve the \"table\" world object");
        assert!(
            !constraint.mobile_reference_frame(),
            "a world-object reference frame must resolve Fixed, matching upstream \
             SceneTransforms::isFixedFrame's object-frame half"
        );
    }
}
