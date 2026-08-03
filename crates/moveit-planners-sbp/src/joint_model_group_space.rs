// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The bridge from a [`RobotModel`]'s [`moveit_model::JointModelGroup`] to a
//! [`StateSpace`]: [`JointModelGroupSpace`] dispatches each of a group's
//! active joints to the right subspace and composes them with
//! [`CompoundSpace`].

use std::f64::consts::PI;

use moveit_model::RobotModel;
use moveit_model::joint::{JointKind, VariableBounds};
use moveit_state::RobotState;
use rand::Rng;

use crate::compound::{CompoundSpace, CompoundValue};
use crate::error::SbpError;
use crate::se3::{Se3Space, Se3State};
use crate::so2::So2Space;
use crate::space::{RealVectorSpace, StateSpace};

/// Half-extent (metres) substituted for a translation axis whose own
/// `VariableBounds` are non-finite.
///
/// A floating or planar joint's translation is unbounded (`[-inf, inf]`) in
/// `RobotModel` by design: upstream MoveIt leaves it that way too and gets a
/// workspace box from the planner's own configuration (OMPL's
/// `workspace_bounds` parameter) at plan time, not from the robot
/// description — see [`VariableBounds`]'s doc comment. `JointModelGroupSpace`
/// has no such parameter channel (its constructor takes only a `&RobotModel`
/// and a group name), so a non-finite bound
/// falls back to this fixed half-extent instead of being passed through to
/// [`RealVectorSpace::new`], which rejects infinities outright. Matches the
/// translation bounds this crate's own `Se3Space`/`CompoundSpace` tests
/// already use as a representative workspace scale.
const UNBOUNDED_TRANSLATION_HALF_EXTENT: f64 = 10.0;

/// `(min, max)` for a translation-like axis: the joint's own bounds if both
/// finite, otherwise [`UNBOUNDED_TRANSLATION_HALF_EXTENT`] either side of
/// zero. See that constant's doc comment for why a substitution is needed at
/// all.
fn bounded_axis(bounds: &VariableBounds) -> (f64, f64) {
    if bounds.min_position.is_finite() && bounds.max_position.is_finite() {
        (bounds.min_position, bounds.max_position)
    } else {
        (
            -UNBOUNDED_TRANSLATION_HALF_EXTENT,
            UNBOUNDED_TRANSLATION_HALF_EXTENT,
        )
    }
}

/// Where one [`CompoundSpace`] subspace's scalar value(s) live in a
/// [`RobotState`]'s flat variable vector, and how to convert between them.
#[derive(Debug, Clone, Copy)]
enum SubspaceSlot {
    /// A [`RealVectorSpace`] axis of dimension 1 (bounded revolute,
    /// prismatic, or one of a planar joint's `x`/`y`): one global variable
    /// index.
    RealVector(usize),
    /// A [`So2Space`] (continuous revolute, or a planar joint's `theta`):
    /// one global variable index.
    So2(usize),
    /// An [`Se3Space`] (a floating joint): the global variable indices of
    /// `trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w`, in that
    /// (upstream `FloatingJointModel`) order.
    Se3([usize; 7]),
}

/// A [`StateSpace`] built from one [`RobotModel`] joint model group.
///
/// Dispatches each of the group's active (non-fixed, non-mimic) joints:
///
/// - bounded revolute or prismatic -> a one-axis [`RealVectorSpace`] with
///   the joint's own `VariableBounds`
/// - continuous revolute -> [`So2Space`]
/// - planar -> `x`, `y` as two one-axis `RealVectorSpace`s plus `theta` as
///   `So2Space`
/// - floating -> [`Se3Space`]
///
/// composed with [`CompoundSpace`], which already handles a heterogeneous
/// weighted product — see [`CompoundSpace`]'s own doc comment for why this
/// type does not reimplement that.
///
/// # Weighting
///
/// Each subspace's `CompoundSpace` weight is `1 / extent`, where `extent` is
/// that subspace's own range: `max - min` for a `RealVectorSpace` axis,
/// `2 * PI` for `So2Space` (the whole circle), and, for `Se3Space`,
/// `sqrt(dx^2 + dy^2 + dz^2) + PI/2 * angular_distance_weight` (mirroring
/// upstream `FloatingJointModel::getMaximumExtent`, applied to this
/// constructor's own — possibly substituted, see this module's
/// `UNBOUNDED_TRANSLATION_HALF_EXTENT` — translation bounds). A planar
/// joint's `theta` subspace additionally scales its `1 / (2 * PI)` weight by
/// that joint's own `angular_distance_weight`, the same SRDF-configurable
/// knob `Se3Space::new`'s `rotation_weight` argument carries for a floating
/// joint — both are `PlanarJoint`/`FloatingJointModel`'s own
/// rotation-vs-translation weight upstream, so this keeps a configured value
/// from being silently dropped just because both joint kinds are split into
/// independent subspaces here rather than measured by one call to
/// `PlanarJoint::distance`/`FloatingJoint::distance`.
///
/// This normalizes every subspace to comparable scale before
/// [`CompoundSpace`] sums their weighted distances, so a joint with a wide
/// range and one with a narrow range contribute comparably to "nearest
/// neighbour" rather than the widest-range joint dominating the metric by
/// raw magnitude. It is *not* upstream `JointModelGroup::distance`'s own
/// rule (that method weights by `JointModel::getDistanceFactor`, which is
/// each joint's *variable count*, not its extent) — that rule cannot express
/// this type's finer split of a planar joint into three independent
/// subspaces, and MoveIt's own OMPL bridge (`ModelBasedStateSpace`, see
/// `moveit_planners/ompl/ompl_interface/src/parameterization/model_based_state_space.cpp`)
/// does not build a true weighted `CompoundStateSpace` at all — it delegates
/// distance to `JointModelGroup::distance` directly over a flat array. The
/// extent-normalization rule here instead follows the general OMPL
/// convention for composing a `CompoundStateSpace` from heterogeneous
/// subspaces of otherwise-incomparable units (radians vs. metres): weight
/// each by the reciprocal of its own extent.
pub struct JointModelGroupSpace {
    compound: CompoundSpace,
    layout: Vec<SubspaceSlot>,
}

impl JointModelGroupSpace {
    /// Builds a space for `model`'s joint model group named `group_name`.
    ///
    /// # Errors
    /// [`SbpError::UnknownGroup`] if `model` has no such group.
    pub fn new(model: &RobotModel, group_name: &str) -> Result<Self, SbpError> {
        let group = model
            .joint_model_group(group_name)
            .map_err(|_| SbpError::UnknownGroup {
                name: group_name.to_string(),
            })?;

        let mut subspaces = Vec::new();
        let mut layout = Vec::new();

        for &joint_index in group.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let first = model
                .variable_index(joint.name())
                .expect("an active joint always has a variable index");

            match joint.kind() {
                JointKind::Revolute(revolute) => {
                    if revolute.is_continuous() {
                        subspaces.push((CompoundSpace::so2(So2Space::new()), 1.0 / (2.0 * PI)));
                        layout.push(SubspaceSlot::So2(first));
                    } else {
                        let (min, max) = bounded_axis(&joint.variable_bounds()[0]);
                        let space = RealVectorSpace::new(vec![(min, max)])?;
                        subspaces.push((CompoundSpace::real_vector(space), 1.0 / (max - min)));
                        layout.push(SubspaceSlot::RealVector(first));
                    }
                }
                JointKind::Prismatic(_) => {
                    let (min, max) = bounded_axis(&joint.variable_bounds()[0]);
                    let space = RealVectorSpace::new(vec![(min, max)])?;
                    subspaces.push((CompoundSpace::real_vector(space), 1.0 / (max - min)));
                    layout.push(SubspaceSlot::RealVector(first));
                }
                JointKind::Planar(planar) => {
                    let bounds = joint.variable_bounds();
                    let (x_min, x_max) = bounded_axis(&bounds[0]);
                    let (y_min, y_max) = bounded_axis(&bounds[1]);

                    let x_space = RealVectorSpace::new(vec![(x_min, x_max)])?;
                    subspaces.push((CompoundSpace::real_vector(x_space), 1.0 / (x_max - x_min)));
                    layout.push(SubspaceSlot::RealVector(first));

                    let y_space = RealVectorSpace::new(vec![(y_min, y_max)])?;
                    subspaces.push((CompoundSpace::real_vector(y_space), 1.0 / (y_max - y_min)));
                    layout.push(SubspaceSlot::RealVector(first + 1));

                    // `angular_distance_weight` (SRDF `<joint_property
                    // name="angular_distance_weight">`, default `1.0`) is
                    // the same per-joint rotation-vs-translation knob used
                    // for the floating case below (there, directly as
                    // `Se3Space::new`'s `rotation_weight`): folding it into
                    // theta's extent-normalized weight here keeps a
                    // configured value from being silently dropped just
                    // because this type splits a planar joint into
                    // independent subspaces instead of calling
                    // `PlanarJoint::distance` directly.
                    let theta_weight = planar.angular_distance_weight() / (2.0 * PI);
                    subspaces.push((CompoundSpace::so2(So2Space::new()), theta_weight));
                    layout.push(SubspaceSlot::So2(first + 2));
                }
                JointKind::Floating(floating) => {
                    let bounds = joint.variable_bounds();
                    let translation_bounds = [
                        bounded_axis(&bounds[0]),
                        bounded_axis(&bounds[1]),
                        bounded_axis(&bounds[2]),
                    ];
                    let angular_weight = floating.angular_distance_weight();
                    let extent = translation_bounds
                        .iter()
                        .map(|&(min, max)| (max - min).powi(2))
                        .sum::<f64>()
                        .sqrt()
                        + PI * 0.5 * angular_weight;

                    let space = Se3Space::new(translation_bounds, angular_weight)?;
                    subspaces.push((CompoundSpace::se3(space), 1.0 / extent));
                    layout.push(SubspaceSlot::Se3([
                        first,
                        first + 1,
                        first + 2,
                        first + 3,
                        first + 4,
                        first + 5,
                        first + 6,
                    ]));
                }
                JointKind::Fixed => {
                    unreachable!(
                        "JointModelGroup::active_joint_indices excludes fixed joints, got '{}'",
                        joint.name()
                    )
                }
            }
        }

        let compound = CompoundSpace::new(subspaces)?;
        Ok(Self { compound, layout })
    }

    /// Reads this group's active-joint values out of `robot_state` into a
    /// [`StateSpace::State`].
    pub fn read_robot_state(&self, robot_state: &RobotState) -> <Self as StateSpace>::State {
        self.read_variables(robot_state.positions())
    }

    /// Writes `state`'s values into `robot_state`'s variables at this
    /// group's indices, leaving every other variable untouched.
    pub fn write_robot_state(
        &self,
        state: &<Self as StateSpace>::State,
        robot_state: &mut RobotState,
    ) {
        let mut variables = robot_state.positions().to_vec();
        self.write_variables(state, &mut variables);
        robot_state.set_variable_positions(&variables);
    }

    /// Reads this group's active-joint values out of `variables` — shaped
    /// like [`RobotState::positions`], one entry per model variable — into a
    /// [`StateSpace::State`].
    fn read_variables(&self, variables: &[f64]) -> <Self as StateSpace>::State {
        self.layout
            .iter()
            .map(|slot| match *slot {
                SubspaceSlot::RealVector(index) => {
                    CompoundValue::RealVector(vec![variables[index]])
                }
                SubspaceSlot::So2(index) => CompoundValue::So2(variables[index]),
                SubspaceSlot::Se3(indices) => CompoundValue::Se3(Se3State {
                    translation: [
                        variables[indices[0]],
                        variables[indices[1]],
                        variables[indices[2]],
                    ],
                    // `Se3State::rotation` is `(w, x, y, z)`; a floating
                    // joint's local variables are `rot_x, rot_y, rot_z,
                    // rot_w` — a pure reorder, no arithmetic, so this stays
                    // bit-exact.
                    rotation: [
                        variables[indices[6]],
                        variables[indices[3]],
                        variables[indices[4]],
                        variables[indices[5]],
                    ],
                }),
            })
            .collect()
    }

    /// Writes `state`'s values into `variables` — shaped like
    /// [`RobotState::positions`] — at this group's indices, leaving every
    /// other entry untouched.
    fn write_variables(&self, state: &<Self as StateSpace>::State, variables: &mut [f64]) {
        debug_assert_eq!(state.len(), self.layout.len());
        for (slot, value) in self.layout.iter().zip(state) {
            match (*slot, value) {
                (SubspaceSlot::RealVector(index), CompoundValue::RealVector(v)) => {
                    variables[index] = v[0];
                }
                (SubspaceSlot::So2(index), CompoundValue::So2(v)) => {
                    variables[index] = *v;
                }
                (SubspaceSlot::Se3(indices), CompoundValue::Se3(se3)) => {
                    variables[indices[0]] = se3.translation[0];
                    variables[indices[1]] = se3.translation[1];
                    variables[indices[2]] = se3.translation[2];
                    variables[indices[3]] = se3.rotation[1];
                    variables[indices[4]] = se3.rotation[2];
                    variables[indices[5]] = se3.rotation[3];
                    variables[indices[6]] = se3.rotation[0];
                }
                (slot, value) => panic!(
                    "JointModelGroupSpace: subspace slot {slot:?} does not match state value {value:?}"
                ),
            }
        }
    }
}

impl StateSpace for JointModelGroupSpace {
    type State = Vec<CompoundValue>;

    fn dimension(&self) -> usize {
        self.compound.dimension()
    }

    fn distance(&self, a: &Self::State, b: &Self::State) -> f64 {
        self.compound.distance(a, b)
    }

    fn interpolate(&self, from: &Self::State, to: &Self::State, t: f64) -> Self::State {
        self.compound.interpolate(from, to, t)
    }

    fn enforce_bounds(&self, state: &mut Self::State) {
        self.compound.enforce_bounds(state);
    }

    fn satisfies_bounds(&self, state: &Self::State) -> bool {
        self.compound.satisfies_bounds(state)
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> Self::State {
        self.compound.sample_uniform(rng)
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &Self::State, radius: f64) -> Self::State {
        self.compound.sample_near(rng, center, radius)
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use moveit_model::MeshSearchPaths;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use super::*;
    use crate::rrt_connect::{RrtConnectParams, Termination, rrt_connect};
    use crate::test_support::assert_metric_and_interpolation_axioms;
    use crate::validity::{DiscreteMotionValidator, MotionValidator};

    /// Every group this module's tests exercise, one per row: `(urdf file,
    /// srdf file, group name)`, all under the repo-root `fixtures/`
    /// directory (the same fixtures `moveit-model`'s parity tests use).
    ///
    /// Coverage by joint kind: `panda_arm`/`manipulator`/
    /// `{left,right}_panda_arm` are bounded-revolute-only; PR2's `right_arm`
    /// adds two continuous joints (`r_forearm_roll_joint`,
    /// `r_wrist_roll_joint`); `base` is PR2's planar virtual joint alone;
    /// `arms_and_torso` adds a prismatic joint (`torso_lift_joint`) on top
    /// of both arms' revolute and continuous joints. No fixture's group
    /// includes a floating joint (panda's floating virtual joint is not a
    /// member of any SRDF group) — the floating branch has its own
    /// synthetic-model unit tests below instead.
    const GROUPS: &[(&str, &str, &str)] = &[
        ("panda.urdf", "panda.srdf", "panda_arm"),
        ("fanuc.urdf", "fanuc.srdf", "manipulator"),
        (
            "dual_arm_panda.urdf",
            "dual_arm_panda.srdf",
            "left_panda_arm",
        ),
        (
            "dual_arm_panda.urdf",
            "dual_arm_panda.srdf",
            "right_panda_arm",
        ),
        ("pr2.urdf", "pr2.srdf", "right_arm"),
        ("pr2.urdf", "pr2.srdf", "base"),
        ("pr2.urdf", "pr2.srdf", "arms_and_torso"),
    ];

    fn load_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
        let urdf_path = format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            urdf_file
        );
        let srdf_path = format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            srdf_file
        );
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = moveit_srdf::SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    #[test]
    fn every_fixture_group_builds() {
        for &(urdf_file, srdf_file, group_name) in GROUPS {
            let model = load_model(urdf_file, srdf_file);
            JointModelGroupSpace::new(&model, group_name).unwrap_or_else(|e| {
                panic!("{urdf_file}/{group_name}: JointModelGroupSpace::new failed: {e}")
            });
        }
    }

    #[test]
    fn unknown_group_is_rejected() {
        // Not assert_eq!: JointModelGroupSpace holds a CompoundSpace, which
        // (like CompoundSpace itself) cannot implement Debug/PartialEq
        // because it boxes `dyn StateSpace`.
        let model = load_model("panda.urdf", "panda.srdf");
        match JointModelGroupSpace::new(&model, "no_such_group") {
            Err(e) => assert_eq!(
                e,
                SbpError::UnknownGroup {
                    name: "no_such_group".to_string()
                }
            ),
            Ok(_) => panic!("expected Err(UnknownGroup)"),
        }
    }

    /// `read_robot_state` then `write_robot_state` must return the
    /// identical `f64` bits at every one of this group's variables:
    /// everything downstream (planning, then applying the result back to a
    /// `RobotState`) depends on this round trip losing nothing.
    #[test]
    fn round_trip_through_robot_state_is_bit_exact() {
        for &(urdf_file, srdf_file, group_name) in GROUPS {
            let model = load_model(urdf_file, srdf_file);
            let space = JointModelGroupSpace::new(&model, group_name).unwrap();
            let group = model.joint_model_group(group_name).unwrap();
            let mut rng = ChaCha8Rng::seed_from_u64(1);

            for _ in 0..200 {
                let mut original = RobotState::new(&model);
                original.set_to_random_positions_with(&mut rng);

                let group_state = space.read_robot_state(&original);

                // Start from a *different* RobotState (all default
                // positions, not `original`'s) so a passing assertion below
                // proves `write_robot_state` actually wrote these values,
                // rather than them having already been there.
                let mut round_tripped = RobotState::new(&model);
                space.write_robot_state(&group_state, &mut round_tripped);

                for &joint_index in group.active_joint_indices() {
                    let joint = model.joint_model_at(joint_index);
                    for name in joint.variable_names() {
                        let index = model.variable_index(name).unwrap();
                        assert_eq!(
                            original.positions()[index],
                            round_tripped.positions()[index],
                            "{urdf_file}/{group_name}: bit mismatch at variable '{name}'"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn metric_and_interpolation_axioms_hold_for_every_fixture_group() {
        for &(urdf_file, srdf_file, group_name) in GROUPS {
            let model = load_model(urdf_file, srdf_file);
            let space = JointModelGroupSpace::new(&model, group_name).unwrap();
            let mut rng = ChaCha8Rng::seed_from_u64(2);
            assert_metric_and_interpolation_axioms(
                &space,
                &mut rng,
                |rng| space.sample_uniform(rng),
                2000,
                1e-6,
            );
        }
    }

    #[test]
    fn rrt_connect_runs_end_to_end_on_panda_arm() {
        let model = load_model("panda.urdf", "panda.srdf");
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(3);

        let start = space.sample_uniform(&mut rng);
        let goal = space.sample_uniform(&mut rng);

        let always_valid = |_: &Vec<CompoundValue>| true;
        let resolution = 0.05;
        let mv = DiscreteMotionValidator::new(&always_valid, resolution);
        let params = RrtConnectParams {
            step_size: 0.5,
            goal_bias: 0.05,
            termination: Termination::Iterations(20_000),
            nn_degree: 8,
        };

        let path = rrt_connect(
            &space,
            &always_valid,
            &mv,
            start.clone(),
            goal.clone(),
            &mut rng,
            &params,
        )
        .expect("an always-valid checker over panda_arm's bounded box must be solvable");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        assert!(path.len() >= 2);
        for pair in path.windows(2) {
            assert!(
                mv.is_motion_valid(&space, &pair[0], &pair[1]),
                "invalid segment {:?} -> {:?}",
                pair[0],
                pair[1]
            );
        }
    }

    /// No fixture group includes a floating joint, so the floating branch
    /// (and the [`UNBOUNDED_TRANSLATION_HALF_EXTENT`] fallback it needs,
    /// since a floating joint's translation is unbounded by default — see
    /// that constant's doc comment) gets its own minimal synthetic model
    /// instead of fixture coverage.
    mod floating_joint {
        use super::*;

        fn synthetic_floating_model() -> RobotModel {
            let urdf_xml = r#"<?xml version="1.0"?>
<robot name="floating_test">
  <link name="world"/>
  <link name="body"/>
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
            let srdf =
                moveit_srdf::SrdfModel::parse_str(srdf_xml).expect("synthetic SRDF must parse");
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("synthetic model must build")
        }

        #[test]
        fn builds_and_holds_metric_axioms() {
            let model = synthetic_floating_model();
            let space = JointModelGroupSpace::new(&model, "body_group").unwrap();
            assert_eq!(space.dimension(), 6);
            let mut rng = ChaCha8Rng::seed_from_u64(4);
            assert_metric_and_interpolation_axioms(
                &space,
                &mut rng,
                |rng| space.sample_uniform(rng),
                2000,
                1e-6,
            );
        }

        #[test]
        fn round_trip_through_robot_state_is_bit_exact() {
            let model = synthetic_floating_model();
            let space = JointModelGroupSpace::new(&model, "body_group").unwrap();
            let mut rng = ChaCha8Rng::seed_from_u64(5);

            for _ in 0..200 {
                let mut original = RobotState::new(&model);
                original.set_to_random_positions_with(&mut rng);

                let group_state = space.read_robot_state(&original);

                let mut round_tripped = RobotState::new(&model);
                space.write_robot_state(&group_state, &mut round_tripped);

                assert_eq!(original.positions(), round_tripped.positions());
            }
        }
    }
}
