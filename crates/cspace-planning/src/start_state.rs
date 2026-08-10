// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2019, Universitaet Hamburg.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/src/planning_scene.cpp (`getCurrentStateUpdated`)
//   moveit_core/robot_state/src/conversions.cpp (`jointStateToRobotStateImpl`,
//     `robotStateMsgToRobotStateHelper`)
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//     (`setVariableValues(const sensor_msgs::msg::JointState&)`)
//   moveit_core/utils/src/message_checks.cpp (`isEmpty(const moveit_msgs::msg::RobotState&)`)

//! [`StartState`] replaces `moveit_msgs::msg::MotionPlanRequest::start_state`:
//! the state a query is to be planned *from*.
//!
//! # What the field means upstream, measured rather than assumed
//!
//! No upstream reader treats `req.start_state` as a complete robot state.
//! Every one of them composes the same two lines — the scene's current state,
//! then the message written over it:
//!
//! ```text
//! moveit::core::RobotState start_state = planning_scene->getCurrentState();
//! moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);
//! ```
//!
//! verbatim at `check_start_state_bounds.cpp:87-88` and
//! `check_start_state_collision.cpp:74-75`, and through the identical
//! `PlanningScene::getCurrentStateUpdated` helper (`planning_scene.cpp:636-641`,
//! which is those same two lines) at `planning_context_manager.cpp:586`
//! (OMPL), `stomp_moveit_planning_context.cpp:226` (STOMP) and, spelled out
//! again, `chomp_planner.cpp:77-78` (CHOMP).
//!
//! So `start_state` is an **overlay**, not a state: it names some variables
//! and gives them values, and every variable it does not name keeps whatever
//! the scene's current state holds. That is the convention
//! `third_party/moveit_msgs/dox/definePlanningRequest.dox` states in prose
//! ("Joint values that are not set in this message are assumed to be the same
//! as the robot's *current* state"), and it is what the code does:
//! `robotStateMsgToRobotStateHelper` (`conversions.cpp:371-397`) reaches
//! `setVariableValues` (`robot_state.hpp:1125-1131`), which calls
//! `setVariablePositions(msg.name, msg.position)` — a loop over the *named*
//! variables only (`robot_state.cpp:395-406`).
//!
//! # Why this is one sum type and not a value plus a flag
//!
//! The wire message carries an `is_diff` boolean next to the values, and the
//! obvious transcription would be a `bool` field beside a state. That pairing
//! is exactly the illegal-combination shape this port avoids, and upstream's
//! own code says the flag is not what selects the overlay behaviour:
//!
//! - Positions and velocities are applied by name whether or not `is_diff`
//!   is set — `setVariableValues` never reads it.
//! - `robotStateMsgToRobotStateHelper` reads `is_diff` in exactly two places
//!   (`conversions.cpp:377`, `:389`): to log "Found empty JointState message"
//!   and bail on a message that is neither a diff nor names anything, and to
//!   `clearAttachedBodies()` before re-applying `attached_collision_objects`.
//!   Both are attached-body concerns, and this port keeps attached bodies on
//!   [`cspace_scene::PlanningScene`], not on a state — so the msg->core
//!   conversion in `cspace-ros` rejects `attached_collision_objects` on its
//!   own and there is nothing left for `is_diff` to select.
//!
//! What upstream *does* branch on is `moveit::core::isEmpty(req.start_state)`
//! (`message_checks.cpp:54-64`; used at `move_action_capability.cpp:159`), a
//! predicate over the whole message rather than a stored flag. That predicate
//! is this type's [`StartState::CurrentState`] variant, and it is reached by
//! construction: [`StartState::new`] returns `CurrentState` for an assignment
//! that names nothing, so [`StartState::Overriding`] never holds an empty
//! override and the two variants can never describe the same request.
//!
//! # The invariants `StartStateOverride` holds by construction
//!
//! `sensor_msgs/JointState`'s own comment is "All arrays in this message
//! should have the same size, or be empty" — a convention upstream *assumes*
//! in two places and checks in one:
//!
//! - checked: `jointStateToRobotStateImpl` (`conversions.cpp:62-74`) rejects
//!   `name.size() != position.size()`, which also means a `name` list with an
//!   empty `position` list applies nothing at all upstream, not "velocities
//!   only".
//! - assumed: `setVariableVelocities(names, values)` (`robot_state.cpp:422-429`)
//!   guards the pairing with a bare `assert`, so a shorter `velocity` array
//!   reads past the end of the vector in any build with `NDEBUG` set.
//!
//! [`StartStateOverride`] makes all three unrepresentable instead: its fields
//! are private and [`StartStateOverride::new`] is the only way in, so a value
//! of this type always has one position per name, and either no velocities or
//! one per name.

use cspace_core::error::{Error, Result};
use cspace_core::state::RobotState;

/// The state a [`crate::PlanningRequest`] is planned from — replaces
/// `moveit_msgs::msg::MotionPlanRequest::start_state`.
///
/// See this module's doc for the upstream measurement behind the shape: the
/// wire field is an overlay on the scene's current state, and the empty
/// overlay is upstream's own "plan from wherever the robot is".
#[derive(Debug, Clone, Default, PartialEq)]
pub enum StartState {
    /// Plan from [`cspace_scene::PlanningScene::current_state`] exactly, with
    /// nothing written over it.
    ///
    /// This is `moveit::core::isEmpty(req.start_state)`
    /// (`message_checks.cpp:54-64`) and the `Default`, matching an unset
    /// `moveit_msgs::msg::MotionPlanRequest::start_state` — the same
    /// unset-means-default reading [`crate::WorkspaceBounds::default`]
    /// documents for [`crate::PlanningRequest::workspace_bounds`].
    #[default]
    CurrentState,
    /// Plan from the scene's current state with the named variables written
    /// over it.
    ///
    /// Never empty: [`StartState::new`] answers `CurrentState` when there is
    /// nothing to write, so this variant naming no variable is unconstructible
    /// rather than merely unusual.
    Overriding(StartStateOverride),
}

impl StartState {
    /// The only constructor, taking `sensor_msgs/JointState`'s three arrays as
    /// they arrive on the wire.
    ///
    /// An assignment that names nothing is upstream's empty `start_state` and
    /// becomes [`StartState::CurrentState`]; anything else becomes
    /// [`StartState::Overriding`].
    ///
    /// # Errors
    ///
    /// The two length rules `sensor_msgs/JointState`'s own comment states and
    /// upstream half-enforces (see this module's doc): `positions` must have
    /// one entry per name, and `velocities` must be empty or have one entry
    /// per name.
    pub fn new(names: Vec<String>, positions: Vec<f64>, velocities: Vec<f64>) -> Result<Self> {
        if names.is_empty() {
            // Upstream's `jointStateToRobotStateImpl` rejects a name/position
            // length mismatch in *both* directions, so positions without names
            // is as much a violation as names without positions -- reported
            // here rather than silently collapsed to `CurrentState`.
            if !positions.is_empty() || !velocities.is_empty() {
                return Err(Error::construct(format!(
                    "start_state.joint_state names no variable but carries {} position(s) and \
                     {} velocity(ies); the wire's own convention is \"All arrays in this \
                     message should have the same size, or be empty\"",
                    positions.len(),
                    velocities.len()
                )));
            }
            return Ok(Self::CurrentState);
        }
        Ok(Self::Overriding(StartStateOverride::new(
            names, positions, velocities,
        )?))
    }

    /// Ports `PlanningScene::getCurrentStateUpdated` (`planning_scene.cpp:636-641`)
    /// applied in place: writes this overlay onto `state`, leaving every
    /// variable it does not name alone.
    ///
    /// [`StartState::CurrentState`] is a no-op, which is what makes the
    /// "unset means current state" convention hold without a caller-side
    /// branch — the same call site serves both variants.
    ///
    /// # Errors
    ///
    /// [`cspace_core::error::Error::UnknownName`] if the overlay names a variable
    /// `state`'s model does not have. Upstream reaches the same condition
    /// through `RobotModel::getVariableIndex` throwing inside
    /// `setVariablePositions` (`robot_state.cpp:395-406`).
    pub fn apply_to(&self, state: &mut RobotState<'_>) -> Result<()> {
        let Self::Overriding(over) = self else {
            return Ok(());
        };
        // One iteration per *name*, not one pass per array, so the index that
        // pairs a name with its value appears once instead of twice: a
        // reversed or mis-zipped array is then a single visible defect rather
        // than two that can disagree with each other.
        for (index, name) in over.names().iter().enumerate() {
            let position = over.positions()[index];
            state.set_variable_position(name, position).map_err(|e| {
                Error::construct(format!(
                    "start_state.joint_state[{index}] position {position} for {name:?}: {e}"
                ))
            })?;
            let Some(&velocity) = over.velocities().get(index) else {
                continue;
            };
            // `set_variable_velocity` resolves `name` through the same
            // `RobotModel::variable_index` the position write above just
            // succeeded on, so this `?` reports nothing the line above did not
            // already report -- it is propagation, not a second check.
            state.set_variable_velocity(name, velocity)?;
        }
        Ok(())
    }
}

/// A non-empty overlay: one position per named variable, and either no
/// velocities or one per named variable.
///
/// Fields are private and [`StartStateOverride::new`] is the only constructor
/// precisely so those two rules cannot be violated by a value of this type —
/// see this module's doc, "The invariants `StartStateOverride` holds by
/// construction".
#[derive(Debug, Clone, PartialEq)]
pub struct StartStateOverride {
    names: Vec<String>,
    positions: Vec<f64>,
    velocities: Vec<f64>,
}

impl StartStateOverride {
    /// Build an overlay from `sensor_msgs/JointState`'s three arrays.
    ///
    /// Callers normally reach this through [`StartState::new`], which routes
    /// the empty case to [`StartState::CurrentState`] instead of building an
    /// override that names nothing.
    ///
    /// # Errors
    ///
    /// [`cspace_core::error::Error::Construct`] if `names` is empty, if `positions`
    /// does not have one entry per name, or if `velocities` is neither empty
    /// nor one entry per name.
    pub fn new(names: Vec<String>, positions: Vec<f64>, velocities: Vec<f64>) -> Result<Self> {
        if names.is_empty() {
            return Err(Error::construct(
                "start_state.joint_state override names no variable; an overlay that writes \
                 nothing is StartState::CurrentState, not an empty override",
            ));
        }
        if positions.len() != names.len() {
            return Err(Error::construct(format!(
                "start_state.joint_state has {} name(s) but {} position(s); upstream's \
                 jointStateToRobotStateImpl (conversions.cpp:62-74) rejects this message \
                 outright, applying neither the positions nor the velocities",
                names.len(),
                positions.len()
            )));
        }
        if !velocities.is_empty() && velocities.len() != names.len() {
            return Err(Error::construct(format!(
                "start_state.joint_state has {} name(s) but {} velocity(ies); the wire's own \
                 convention is \"All arrays in this message should have the same size, or be \
                 empty\", and upstream's setVariableVelocities (robot_state.cpp:422-429) \
                 guards the pairing with a bare assert",
                names.len(),
                velocities.len()
            )));
        }
        Ok(Self {
            names,
            positions,
            velocities,
        })
    }

    /// The variables this overlay writes, in the order the wire listed them.
    /// Never empty.
    pub fn names(&self) -> &[String] {
        &self.names
    }

    /// One position per [`StartStateOverride::names`] entry, same order.
    pub fn positions(&self) -> &[f64] {
        &self.positions
    }

    /// Either empty (the message carried no velocities) or one per
    /// [`StartStateOverride::names`] entry, same order.
    pub fn velocities(&self) -> &[f64] {
        &self.velocities
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;

    use super::*;

    fn panda() -> RobotModel {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    #[track_caller]
    fn assert_err_mentions<T: std::fmt::Debug>(result: Result<T>, needle: &str) {
        let rendered = result
            .expect_err("expected this call to be rejected")
            .to_string();
        assert!(
            rendered.contains(needle),
            "expected the rejection to come from the branch that reports {needle:?}, got: {rendered}"
        );
    }

    #[test]
    fn naming_nothing_is_current_state_not_an_empty_override() {
        assert_eq!(
            StartState::new(vec![], vec![], vec![]).unwrap(),
            StartState::CurrentState
        );
        assert_eq!(StartState::default(), StartState::CurrentState);
    }

    #[test]
    fn an_empty_override_is_unconstructible_through_the_override_constructor_too() {
        // `StartState::new` routes the empty case away from `Overriding`; this
        // is the other door into the same variant, and it has to be shut as
        // well or the "Overriding is never empty" invariant is a convention
        // rather than a guarantee.
        assert_err_mentions(
            StartStateOverride::new(vec![], vec![], vec![]),
            "names no variable",
        );
    }

    #[test]
    fn positions_without_names_is_rejected_not_read_as_current_state() {
        assert_err_mentions(
            StartState::new(vec![], vec![0.1], vec![]),
            "names no variable but carries 1 position(s) and 0 velocity(ies)",
        );
    }

    #[test]
    fn velocities_without_names_is_rejected_not_read_as_current_state() {
        // Sibling of the case above through the same guard's other operand:
        // `!positions.is_empty() || !velocities.is_empty()`. Without this one
        // the `||`'s right half is a blind operand.
        assert_err_mentions(
            StartState::new(vec![], vec![], vec![0.1]),
            "names no variable but carries 0 position(s) and 1 velocity(ies)",
        );
    }

    #[test]
    fn a_name_without_a_position_is_rejected_not_read_as_a_velocity_only_overlay() {
        // Upstream's `jointStateToRobotStateImpl` rejects `name.size() !=
        // position.size()` before `setVariableValues` runs, so this message
        // applies nothing at all upstream -- including its velocities.
        assert_err_mentions(
            StartState::new(vec!["panda_joint1".to_string()], vec![], vec![1.0]),
            "has 1 name(s) but 0 position(s)",
        );
    }

    #[test]
    fn a_short_velocity_array_is_rejected_rather_than_read_past_its_end() {
        assert_err_mentions(
            StartState::new(
                vec!["panda_joint1".to_string(), "panda_joint2".to_string()],
                vec![0.1, 0.2],
                vec![1.0],
            ),
            "has 2 name(s) but 1 velocity(ies)",
        );
    }

    #[test]
    fn applying_current_state_changes_nothing() {
        let model = panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let before = state.clone();
        StartState::CurrentState.apply_to(&mut state).unwrap();
        assert_eq!(state, before);
    }

    #[test]
    fn an_overlay_writes_the_named_variables_and_leaves_the_rest_at_the_current_state() {
        // The whole point of the upstream semantics this type ports: a
        // partially-specified start state is *not* a complete state, and the
        // variables it omits keep the scene's value rather than resetting.
        let model = panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position("panda_joint2", -0.75).unwrap();

        StartState::new(vec!["panda_joint1".to_string()], vec![0.25], vec![])
            .unwrap()
            .apply_to(&mut state)
            .unwrap();

        assert_eq!(state.variable_position("panda_joint1").unwrap(), 0.25);
        assert_eq!(
            state.variable_position("panda_joint2").unwrap(),
            -0.75,
            "a variable the overlay does not name must keep the value it already had"
        );
    }

    #[test]
    fn an_overlay_pairs_each_value_with_its_own_name() {
        // Two names and two *different* values, asserted individually: a
        // conversion or an `apply_to` that zipped the arrays in the wrong
        // order, or reversed one of them, keeps every name and every value and
        // is invisible to any test that checks only "the overlay landed".
        let model = panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();

        StartState::new(
            vec!["panda_joint1".to_string(), "panda_joint2".to_string()],
            vec![0.25, -0.5],
            vec![],
        )
        .unwrap()
        .apply_to(&mut state)
        .unwrap();

        assert_eq!(state.variable_position("panda_joint1").unwrap(), 0.25);
        assert_eq!(state.variable_position("panda_joint2").unwrap(), -0.5);
    }

    #[test]
    fn an_overlay_writes_velocities_when_the_message_carried_them() {
        let model = panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        assert!(!state.has_velocities());

        StartState::new(
            vec!["panda_joint1".to_string(), "panda_joint2".to_string()],
            vec![0.25, -0.5],
            vec![1.5, -2.5],
        )
        .unwrap()
        .apply_to(&mut state)
        .unwrap();

        assert!(state.has_velocities());
        assert_eq!(state.variable_velocity("panda_joint1").unwrap(), 1.5);
        assert_eq!(state.variable_velocity("panda_joint2").unwrap(), -2.5);
    }

    #[test]
    fn an_overlay_naming_a_variable_the_model_lacks_is_rejected_at_apply_time() {
        // `StartState` carries names, not indices, so it can outlive the model
        // it was built against -- the same reason upstream resolves the name at
        // apply time (and throws out of `getVariableIndex`) rather than at
        // message-decode time.
        let model = panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();

        let result = StartState::new(
            vec!["panda_joint1".to_string(), "no_such_joint".to_string()],
            vec![0.1, 0.2],
            vec![],
        )
        .unwrap()
        .apply_to(&mut state);
        // Index *and* value, not just the name: the message is the only place
        // a wire-side mis-pairing (a reversed value array, a zip in the wrong
        // order) becomes visible to a caller that never gets to read the state
        // back -- which is every caller of this port's `/move_action` today,
        // since no planner runs and no trajectory comes back.
        assert_err_mentions(
            result,
            "start_state.joint_state[1] position 0.2 for \"no_such_joint\"",
        );
    }
}
