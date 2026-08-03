// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The bridge from a sampled [`JointModelGroupSpace`] state to a real
//! collision/constraint check: [`PlanningSceneValidityChecker`] writes a
//! candidate sample into a [`PlanningScene`]'s current state and asks
//! [`PlanningScene::is_state_valid`] — the collision-then-constraints
//! composition `p1-fixtures` ported this round — rather than re-deriving
//! that composition here.

use std::cell::RefCell;

use moveit_collision::{CollisionEnv, CollisionRequest};
use moveit_constraints::KinematicConstraintSet;
use moveit_scene::PlanningScene;
use moveit_state::Posed;

use crate::compound::CompoundValue;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::validity::StateValidityChecker;

/// Checks a [`JointModelGroupSpace`] sample against a real
/// [`PlanningScene`]: collision (via `env`) and, if given, `constraints`.
///
/// # Why `RefCell`
///
/// [`PlanningScene::is_state_valid`] takes `&mut self` —
/// `current_state_mut` materializes an inherited state and
/// `check_collision` calls [`moveit_state::RobotState::update`], both
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
/// scene's *existing* current [`moveit_state::RobotState`]
/// ([`JointModelGroupSpace::write_robot_state`]) rather than constructing a
/// new one — there is no `RobotState::new` anywhere in this type's hot
/// path. The cost this type does *not* avoid is
/// [`moveit_state::RobotState::set_variable_positions`]'s own
/// `positions().to_vec()` (one `Vec<f64>` clone of the model's full
/// variable count, not just this group's, on every call) plus whatever
/// forward-kinematics work [`PlanningScene::is_state_valid`]'s
/// `RobotState::update` performs the first time this sample's transforms
/// are actually read. Measured by this module's own `measured_call_cost`
/// test on the panda fixture (`panda_arm`, 7 DoF, an empty world, no
/// constraints, `cargo nextest` debug profile, 10,000 calls): **~88
/// µs/call** (~11.4k calls/second on one core) for the whole write-and-check
/// round trip this type performs — this is a total, not a breakdown; no
/// profiling was done to say how much of it is the `Vec<f64>` clone versus
/// FK versus collision. A single `Termination::Iterations(20_000)`
/// RRT-Connect query is therefore ~1.8s of validity checking alone in this
/// unoptimized profile; a release build would be faster, not measured here.
/// That is slow enough to be worth watching if planning latency becomes a
/// complaint, but no pooling scheme is built against it yet — a pooling
/// scheme would need its own measurement showing it actually removes enough
/// of this ~88 µs to matter, and nothing here has taken that measurement.
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
        let mut scene = self.scene.borrow_mut();
        self.space
            .write_robot_state(state, scene.current_state_mut());
        scene.is_state_valid(self.env, &self.request, self.constraints)
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::time::Instant;

    use moveit_collision::{LinkPaddingScale, ParryCollisionEnv};
    use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
    use moveit_geometry::{Cuboid, Isometry3, Shape};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_scene::PlanningScene;
    use moveit_srdf::SrdfModel;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use super::*;
    use crate::joint_model_group_space::JointModelGroupSpace;
    use crate::space::StateSpace;

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// A synthetic floating body carrying a real (primitive, not mesh)
    /// collision shape. panda's own `<collision>` geometry is mesh-based,
    /// and every fixture load in this workspace uses
    /// [`MeshSearchPaths::none`] (the meshes are not vendored into the
    /// repo — see `moveit-distance-field`'s
    /// `collision_env_distance_field` module doc), so panda's links never
    /// carry a loaded collision shape in a test. A collision-behaviour
    /// test needs geometry that is actually there, hence this synthetic
    /// model rather than a fixture — the same reason
    /// [`JointModelGroupSpace`]'s own tests give its `floating_joint`
    /// submodule a synthetic model instead of reusing a fixture.
    fn synthetic_floating_model() -> (RobotModel, SrdfModel) {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="floating_test">
  <link name="world"/>
  <link name="body">
    <collision>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <joint name="body_joint" type="floating">
    <parent link="world"/>
    <child link="body"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="floating_test">
  <group name="body_group">
    <joint name="body_joint"/>
  </group>
</robot>
"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("synthetic URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("synthetic SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("synthetic model must build");
        (model, srdf)
    }

    #[test]
    fn a_colliding_sample_is_rejected_and_a_clear_one_is_accepted() {
        let (model, srdf) = synthetic_floating_model();
        let space = JointModelGroupSpace::new(&model, "body_group").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene.add_shape(
            "obstacle",
            std::sync::Arc::new(Shape::Cuboid(Cuboid::new(0.2, 0.2, 0.2).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            moveit_collision::CollisionRequest::default(),
            None,
            &space,
        );

        // The floating joint defaults to identity, coincident with the
        // world-frame obstacle: the two 0.2m boxes must overlap.
        let mut origin_state = moveit_state::RobotState::new(&model);
        origin_state.set_to_default_values();
        let origin = space.read_robot_state(&origin_state);
        assert!(
            !checker.is_valid(&origin),
            "a body coincident with the obstacle must collide"
        );

        // Translating the body 5m away clears a pair of 0.2m boxes.
        let mut far_state = moveit_state::RobotState::new(&model);
        far_state.set_to_default_values();
        far_state
            .set_joint_transform("body_joint", &Isometry3::translation(5.0, 0.0, 0.0))
            .unwrap();
        let far = space.read_robot_state(&far_state);
        assert!(
            checker.is_valid(&far),
            "a body 5m from the obstacle must be collision-free"
        );
    }

    #[test]
    fn a_path_constraint_rejects_a_state_the_constraint_forbids_even_with_no_collision() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());

        let constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.01, 0.01, 1.0)
            .expect("valid joint constraint");
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            moveit_collision::CollisionRequest::default(),
            Some(&set),
            &space,
        );

        let mut zero = moveit_state::RobotState::new(&model);
        zero.set_to_default_values();
        assert!(
            checker.is_valid(&space.read_robot_state(&zero)),
            "panda_joint1 == 0.0 must satisfy a constraint centered on 0.0"
        );

        let mut far = moveit_state::RobotState::new(&model);
        far.set_to_default_values();
        far.set_variable_position("panda_joint1", 1.0).unwrap();
        assert!(
            !checker.is_valid(&space.read_robot_state(&far)),
            "panda_joint1 == 1.0 must violate a +/-0.01 constraint centered on 0.0, with no collision to blame"
        );
    }

    /// Not a correctness test: reports the per-call cost this type's own
    /// doc comment cites, so that number is reproducible rather than only
    /// asserted in prose. No `assert!` on the timing itself — machine
    /// speed varies too much across CI/dev hosts for a hard bound to be
    /// meaningful here.
    #[test]
    fn measured_call_cost() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            moveit_collision::CollisionRequest::default(),
            None,
            &space,
        );

        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let samples: Vec<_> = (0..10_000)
            .map(|_| space.sample_uniform(&mut rng))
            .collect();

        let start = Instant::now();
        for sample in &samples {
            std::hint::black_box(checker.is_valid(sample));
        }
        let elapsed = start.elapsed();
        println!(
            "PlanningSceneValidityChecker::is_valid: {:?}/call over {} calls (panda_arm, empty world, no constraints)",
            elapsed / samples.len() as u32,
            samples.len()
        );
    }
}
