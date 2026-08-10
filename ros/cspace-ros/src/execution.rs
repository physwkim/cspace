// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp
//     (EXECUTION_EVENT_TOPIC:50, processEvent:343, receiveEvent:355,
//      stopExecutionInternal:1193, stopExecution:1209)

//! The `trajectory_execution_event` topic: upstream's
//! `TrajectoryExecutionManager` stop event, and the state it transitions.
//!
//! `MoveGroupInterface::stop()` publishes a `std_msgs/msg/String` carrying
//! `"stop"` on this topic (`move_group_interface.cpp:179` builds the
//! publisher from `TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC`). That
//! Upstream names the topic once, as
//! `TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC`
//! (`trajectory_execution_manager.cpp:50`). This port does **not** mirror
//! that as a Rust `const`: `tools/ci/measure-client-endpoint-surface.py`
//! derives the port's endpoint surface by matching a string literal inside
//! the r2r factory call (its `PORT_OPENER` regex), so a registration that
//! names the topic through a constant measures as *absent* while being
//! bound. The name therefore appears exactly once in this crate -- the
//! literal in `bin/move_group.rs`'s `subscribe` call, which is how the other
//! three endpoints are registered too.
//!
//! That is the entire wire contract: one topic, one string, and upstream's
//! `processEvent` (`trajectory_execution_manager.cpp:343`) recognises exactly
//! one value and warns by name on anything else.
//!
//! # `execution_complete_` is a flag with a second meaning; this is a type
//!
//! Upstream tracks execution with a `bool execution_complete_` beside a
//! separate `last_execution_status_`, and `stopExecution` (`:1209`) reads the
//! flag twice -- once unlocked, once under the lock -- before writing both
//! fields. The pair is the classic value-plus-flag: "complete" is true both
//! before anything has ever executed and after something was preempted, and
//! only `last_execution_status_` tells those apart. [`ExecutionState`] makes
//! them one value, so "was this preempted" is not a question about two
//! fields that could disagree.
//!
//! # One owner for the stop transition
//!
//! [`TrajectoryExecution::stop`] is the only function that leaves
//! [`ExecutionState::Executing`], and it performs the whole transition in one
//! statement: there is no window in which execution is marked stopped but the
//! finalizing work has not run. Upstream cannot make that claim -- it sets
//! `execution_complete_ = true` (`:1220`) and *then* calls
//! `stopExecutionInternal` (`:1221`), whose per-handle `cancelExecution()` can
//! throw (which is why upstream wraps each handle in its own try/catch at
//! `:1197-1205`, so one throwing handle does not skip the rest).
//!
//! The reason this port has no such window is worth stating exactly, because
//! it is a property of what is missing rather than of the design: **there are
//! no controller handles to cancel.** No `moveit_controller_manager` is
//! ported, nothing in this workspace executes a trajectory, and so
//! `stopExecutionInternal`'s loop has an empty body here. If a controller
//! manager ever lands, cancelling becomes fallible and the transition stops
//! being a single statement -- at which point the finalizer must move behind
//! a guard that runs on every exit path, and this note is the marker for it.
//!
//! # What is reachable from the wire today
//!
//! [`ExecutionState::Executing`] has no producer in the node: `/move_action`
//! answers with a typed error long before it could execute anything
//! (PORTING-PLAN.md §5 Phase 9 -- there is no `cspace_planning::pipeline::
//! Planner` to call). So a `"stop"` arriving from a real client always takes
//! the [`StopOutcome::NothingToStop`] arm. That arm is not a placeholder: it
//! is upstream's own `if (!execution_complete_)` falling through (`:1211`),
//! and it is the behaviour a client's `stop()` gets from upstream too when
//! nothing is running. [`TrajectoryExecution::begin`] exists so the preempting
//! arm is reachable to a test rather than being an arm nothing can enter.

use cspace_core::error::{Error, Result};
use r2r::std_msgs::msg as std_msgs;

/// The one event upstream's `processEvent` recognises
/// (`trajectory_execution_manager.cpp:345`).
///
/// An enum rather than a `&str` compared at the call site: the wire
/// vocabulary is closed, and a closed vocabulary decoded once at the boundary
/// is what stops "did this string mean stop?" from being re-answered by every
/// later reader.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionEvent {
    /// `"stop"` -- upstream calls `stopExecution(true)` (`:347`).
    Stop,
}

/// Wraps the topic's `std_msgs/msg/String` for the msg->core direction.
pub struct ExecutionEventMsg(pub std_msgs::String);

impl TryFrom<ExecutionEventMsg> for ExecutionEvent {
    type Error = Error;

    /// Upstream's `else` branch is `RCLCPP_WARN_STREAM("Unknown event type:
    /// '" << event << '\'')` (`:351`) -- it names the payload, which is the
    /// only way an operator can tell a typo from a version skew. This returns
    /// the same information as an error and lets the caller decide the
    /// severity, rather than deciding it here.
    fn try_from(msg: ExecutionEventMsg) -> Result<Self> {
        match msg.0.data.as_str() {
            "stop" => Ok(Self::Stop),
            other => Err(Error::other(format!(
                "unknown trajectory_execution_event type: '{other}' \
                 (upstream recognises only 'stop', trajectory_execution_manager.cpp:345)"
            ))),
        }
    }
}

/// Upstream's `last_execution_status_` values that this port can reach.
///
/// Only `PREEMPTED` is reachable: it is the one status
/// [`TrajectoryExecution::stop`] writes (`:1223`), and every other value
/// upstream sets is written by the execution thread, which is not ported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionStatus {
    /// Upstream `moveit_controller_manager::ExecutionStatus::PREEMPTED`.
    Preempted,
}

/// Upstream's `execution_complete_` and `last_execution_status_` as a single
/// value. See the module doc for why they are not a `bool` and a field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionState {
    /// Nothing is executing (upstream `execution_complete_ == true`).
    /// `last` is `None` before anything has ever executed and `Some` after a
    /// completed or preempted execution -- the distinction upstream needs two
    /// fields to express.
    Idle {
        /// `None` until something has executed, `Some` from then on.
        last: Option<ExecutionStatus>,
    },
    /// A trajectory is executing (upstream `execution_complete_ == false`).
    Executing,
}

/// What [`TrajectoryExecution::stop`] actually did.
///
/// Returned rather than inferred by the caller re-reading the state: the
/// caller's log line differs between the two, and re-deriving "was something
/// running?" after the transition has already erased it is the shape that
/// makes a stop look successful when it was a no-op.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopOutcome {
    /// Execution was in flight and is now preempted.
    Preempted,
    /// Nothing was executing. Upstream's `if (!execution_complete_)` simply
    /// not entering (`:1211`) -- a defined no-op, not a failure.
    NothingToStop,
}

/// The execution state a node keeps, and the only thing allowed to change it.
#[derive(Debug, Default)]
pub struct TrajectoryExecution {
    state: ExecutionState,
}

impl Default for ExecutionState {
    fn default() -> Self {
        Self::Idle { last: None }
    }
}

impl TrajectoryExecution {
    /// Nothing has executed yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// The current state. The field behind it is private, so this type's own
    /// methods are the only thing that can transition it.
    pub fn state(&self) -> ExecutionState {
        self.state
    }

    /// Enter [`ExecutionState::Executing`].
    ///
    /// The node never calls this -- see the module doc's last section. It is
    /// the only producer of `Executing`, so that the preempting arm of
    /// [`stop`](Self::stop) has exactly one way to be reached and cannot be
    /// entered by a field assignment somewhere else.
    pub fn begin(&mut self) {
        self.state = ExecutionState::Executing;
    }

    /// Upstream `stopExecution` (`trajectory_execution_manager.cpp:1209`).
    /// **The only transition out of [`ExecutionState::Executing`].**
    ///
    /// Upstream's `auto_clear` argument is not ported: it gates `clear()`,
    /// which frees the queued `TrajectoryExecutionContext`s, and no queue is
    /// ported. The thread join (`:1230-1235`) is likewise absent -- there is
    /// no execution thread.
    pub fn stop(&mut self) -> StopOutcome {
        match self.state {
            ExecutionState::Executing => {
                // Upstream marks complete, then cancels each active handle
                // (`:1220-1221`). There are no handles here, so the mark and
                // the finalize are one statement and no exit path can sit
                // between them. See the module doc.
                self.state = ExecutionState::Idle {
                    last: Some(ExecutionStatus::Preempted),
                };
                StopOutcome::Preempted
            }
            ExecutionState::Idle { .. } => StopOutcome::NothingToStop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(data: &str) -> ExecutionEventMsg {
        ExecutionEventMsg(std_msgs::String {
            data: data.to_string(),
        })
    }

    #[test]
    fn stop_is_the_one_recognised_event() {
        assert_eq!(
            ExecutionEvent::try_from(event("stop")).unwrap(),
            ExecutionEvent::Stop
        );
    }

    #[test]
    fn an_unknown_event_is_rejected_and_names_the_payload() {
        let err = ExecutionEvent::try_from(event("wobble"))
            .unwrap_err()
            .to_string();
        // Naming the payload is the assertion, not merely that it failed:
        // upstream's warning exists so an operator can tell a typo from a
        // version skew, and an error that only said "unknown event" would
        // lose exactly that.
        assert!(
            err.contains("'wobble'"),
            "the rejection must quote the payload, got: {err}"
        );
    }

    #[test]
    fn an_empty_payload_is_not_stop() {
        // The wire's default-constructed String. A `starts_with`/`contains`
        // decoder would let this through; equality does not.
        ExecutionEvent::try_from(event("")).unwrap_err();
    }

    #[test]
    fn stop_is_matched_exactly_not_by_prefix() {
        ExecutionEvent::try_from(event("stopped")).unwrap_err();
        ExecutionEvent::try_from(event(" stop")).unwrap_err();
        ExecutionEvent::try_from(event("STOP")).unwrap_err();
    }

    #[test]
    fn a_stop_while_idle_is_a_defined_no_op() {
        let mut execution = TrajectoryExecution::new();
        assert_eq!(execution.stop(), StopOutcome::NothingToStop);
        // And it must not manufacture a status: an idle stop that recorded
        // PREEMPTED would claim something was preempted that never ran.
        assert_eq!(execution.state(), ExecutionState::Idle { last: None });
    }

    #[test]
    fn a_stop_while_executing_preempts_and_lands_idle() {
        let mut execution = TrajectoryExecution::new();
        execution.begin();
        assert_eq!(execution.state(), ExecutionState::Executing);
        assert_eq!(execution.stop(), StopOutcome::Preempted);
        assert_eq!(
            execution.state(),
            ExecutionState::Idle {
                last: Some(ExecutionStatus::Preempted)
            }
        );
    }

    #[test]
    fn a_second_stop_after_a_preempt_is_a_no_op_and_keeps_the_preempted_status() {
        let mut execution = TrajectoryExecution::new();
        execution.begin();
        execution.stop();
        // Upstream's second `if (!execution_complete_)` under the lock
        // (`:1214`) is what stops a re-entrant stop from re-running the
        // transition. The status must survive it: overwriting `last` here
        // would erase the only record that a preempt happened.
        assert_eq!(execution.stop(), StopOutcome::NothingToStop);
        assert_eq!(
            execution.state(),
            ExecutionState::Idle {
                last: Some(ExecutionStatus::Preempted)
            }
        );
    }
}
