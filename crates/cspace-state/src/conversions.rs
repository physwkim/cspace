// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2011-2013, Willow Garage, Inc.
// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/conversions.hpp
//   moveit_core/robot_state/src/conversions.cpp
// (`robotStateToStream`, both overloads, cpp:498 and cpp:526, and
// `streamToRobotState`, cpp:563. The remaining functions in those two files
// convert `moveit_msgs`/`trajectory_msgs` types; see "What is not here"
// below for where each of them lives instead.)

//! CSV serialization of a [`RobotState`]'s variable positions.
//!
//! Upstream's `robotStateToStream`/`streamToRobotState` exist so a state can
//! be written one line at a time and read back — a trajectory recorded as
//! CSV, a regression corpus, a hand-edited seed. This module is those three
//! functions and nothing else.
//!
//! # What is not here
//!
//! `conversions.cpp`'s other functions all convert to or from a ROS message
//! type, and D1/D6 (`PORTING-PLAN.md` §0, §129.3) put every `moveit_msgs`
//! conversion in `ros/cspace-ros` as a `TryFrom`, never on a core type.
//! They are ported there, against `doc/message-mapping.md`, rather than
//! transcribed from this file:
//!
//! - `jointStateToRobotState`, both `robotStateMsgToRobotState`,
//!   `robotStateToRobotStateMsg`, `robotStateToJointStateMsg` and the
//!   `multiDofJointsToRobotState`/`robotStateToMultiDofJointState` helpers →
//!   `moveit_ros::state`'s `TryFrom` pair (§9), which rejects rather than
//!   silently drops the message fields no core type can represent.
//! - `attachedBodyToMsg`, `msgToAttachedBody` and
//!   `attachedBodiesToAttachedCollisionObjectMsgs` →
//!   `moveit_ros::scene::apply_attached_collision_object` (§11).
//! - `jointTrajPointToRobotState` takes a `trajectory_msgs::JointTrajectory`,
//!   so D1 places it beside `moveit_ros::trajectory` (§10) rather than here.
//!
//! # Deviations
//!
//! **The separator is a `char`, not a string.** Upstream takes
//! `const std::string& separator` and its two directions then disagree about
//! what that means: the writers emit the whole string (`cpp:509`, `:542`),
//! while `streamToRobotState` splits on `separator[0]` alone (`cpp:572`). A
//! `", "` separator therefore writes files upstream's own reader cannot
//! read. One `char` for both directions removes the disagreement by type
//! instead of documenting it.
//!
//! **[`robot_state_to_csv_by_groups`] returns [`Result`].** Upstream
//! dereferences the `getJointModelGroup(joint_group_id)` result with no null
//! check (`cpp:535`, then `:542`) — `doc/upstream-bugs.md`,
//! `robot-state-to-stream-group-lookup-unchecked`. Here the lookup is
//! [`RobotModel::joint_model_group`](cspace_model::RobotModel::joint_model_group),
//! which is already fallible, so the defect cannot be written.
//!
//! **[`csv_to_robot_state`] returns [`Result`], and writes through
//! [`RobotState::set_variable_positions`].** Upstream logs
//! `"Missing variable"` and then runs `std::stod` on the cell it just
//! reported missing (`cpp:573-574`; `doc/upstream-bugs.md`,
//! `stream-to-robot-state-missing-variable-falls-through`), and it assigns
//! through the raw non-dirtying `getVariablePositions()` pointer, leaving
//! forward kinematics answering from the previous state
//! (`stream-to-robot-state-bypasses-dirty-flags`). Here a short or
//! unparsable line is an `Err`, and the write marks the transforms dirty.
//!
//! **Values are written at full `f64` precision.** `std::ostream`'s default
//! precision is six significant digits, so upstream's CSV cannot round-trip
//! a joint value at all (`doc/upstream-bugs.md`,
//! `robot-state-to-stream-default-ostream-precision`). Rust's `Display` for
//! `f64` emits the shortest decimal that reads back bit-for-bit.
//!
//! What is deliberately *not* changed: [`robot_state_to_csv_by_groups`]
//! keeps upstream's separator after every value including the last
//! (`cpp:542`, `:553`), where [`robot_state_to_csv`] omits it at the end
//! (`cpp:508`, `:520`). The two overloads really do emit different dialects.
//! That is observable output shape rather than a defect, and
//! [`csv_to_robot_state`] reads exactly
//! [`RobotModel::variable_count`](cspace_model::RobotModel::variable_count)
//! cells — upstream's own loop bound (`cpp:569`) — so the trailing empty
//! field is ignored by both implementations alike.

use cspace_error::{Error, Result};

use crate::RobotState;

/// `robotStateToStream(state, out, include_header, separator)`: every
/// variable of `state`, in the robot model's own variable order, as one CSV
/// line — optionally preceded by a header line of variable names.
///
/// The returned `String` ends in a newline, as upstream's stream does.
/// Upstream writes into a `std::ostream`; here the caller writes the
/// returned text wherever it wants, which is what makes this testable
/// without a file.
pub fn robot_state_to_csv(state: &RobotState<'_>, include_header: bool, separator: char) -> String {
    let mut out = String::new();
    if include_header {
        push_line(
            &mut out,
            state.model().variable_names().iter().map(String::as_str),
            separator,
        );
    }
    push_line(
        &mut out,
        state.positions().iter().map(f64::to_string),
        separator,
    );
    out
}

/// `robotStateToStream(state, out, joint_groups_ordering, include_header,
/// separator)`: the same, but emitting each named group's variables in the
/// order the groups are given, rather than in the model's variable order.
///
/// A variable belonging to two of the named groups is emitted once per
/// group, and one belonging to none is not emitted at all: upstream
/// concatenates the groups without deduplicating, and so does this. Within
/// a group the order is
/// [`JointModelGroup::variable_names`](cspace_model::JointModelGroup::variable_names),
/// which is upstream's `copyJointGroupPositions` order — `variable_names_`
/// and `variable_index_list_` are filled in the same loop
/// (`joint_model_group.cpp:158`, `:165`).
///
/// # Errors
///
/// [`Error::UnknownName`] if any entry of `group_ordering` does not name a
/// joint model group of this state's robot model. Upstream dereferences the
/// null its lookup returned instead; see the module doc.
pub fn robot_state_to_csv_by_groups(
    state: &RobotState<'_>,
    group_ordering: &[&str],
    include_header: bool,
    separator: char,
) -> Result<String> {
    let mut headers = String::new();
    let mut values = String::new();

    for group_name in group_ordering {
        for name in state
            .model()
            .joint_model_group(group_name)?
            .variable_names()
        {
            if include_header {
                headers.push_str(name);
                headers.push(separator);
            }
            values.push_str(&state.variable_position(name)?.to_string());
            values.push(separator);
        }
    }

    let mut out = String::new();
    if include_header {
        out.push_str(&headers);
        out.push('\n');
    }
    out.push_str(&values);
    out.push('\n');
    Ok(out)
}

/// `streamToRobotState(state, line, separator)`: read one CSV line's worth
/// of variable positions into `state`, in the robot model's own variable
/// order.
///
/// The line is taken as a full state, mimic variables included, so no mimic
/// propagation runs — the same assumption
/// [`RobotState::set_variable_positions`] documents and upstream's raw
/// assignment makes.
///
/// Cells past the model's variable count are ignored, matching upstream's
/// loop bound: a line from [`robot_state_to_csv_by_groups`], whose trailing
/// separator leaves an empty final cell, therefore reads back without
/// special-casing.
///
/// # Errors
///
/// [`Error::Parse`] if `line` holds fewer cells than the model has
/// variables, or if a cell within that count is not an `f64`. Upstream logs
/// the first case and then parses the missing cell anyway; see the module
/// doc.
pub fn csv_to_robot_state(state: &mut RobotState<'_>, line: &str, separator: char) -> Result<()> {
    let variable_count = state.model().variable_count();
    let mut cells = line.split(separator);
    let mut positions = Vec::with_capacity(variable_count);

    for index in 0..variable_count {
        let name = &state.model().variable_names()[index];
        let cell = cells.next().ok_or_else(|| Error::Parse {
            source_kind: "CSV",
            message: format!(
                "line holds {index} cells, but this robot model has {variable_count} \
                 variables; missing variable {name:?}"
            ),
        })?;
        let value = cell.trim().parse::<f64>().map_err(|error| Error::Parse {
            source_kind: "CSV",
            message: format!("variable {name:?} holds {cell:?}, which is not a number: {error}"),
        })?;
        positions.push(value);
    }

    state.set_variable_positions(&positions);
    Ok(())
}

/// One `separator`-joined line plus its newline, with no separator after the
/// last item — upstream's `if (i < count - 1)` guard, hoisted so the header
/// and the value line of [`robot_state_to_csv`] are built by one rule and
/// cannot drift apart.
fn push_line(out: &mut String, items: impl Iterator<Item = impl AsRef<str>>, separator: char) {
    for (index, item) in items.enumerate() {
        if index > 0 {
            out.push(separator);
        }
        out.push_str(item.as_ref());
    }
    out.push('\n');
}
