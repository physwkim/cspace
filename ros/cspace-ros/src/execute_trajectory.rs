// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The `execute_trajectory` action server -- upstream
//! `move_group::MoveGroupExecuteTrajectoryAction`
//! (`moveit_ros/move_group/src/default_capabilities/
//! execute_trajectory_action_capability.cpp`).
//!
//! This endpoint is opened by `MoveGroupInterface`'s own constructor
//! (`move_group_interface.cpp:191-193`), not by a client that asked to
//! execute anything, which is why it is worth binding even though nothing
//! here can execute. What its absence costs is measured in
//! `PORTING-PLAN.md` §273.5 and re-measured by
//! `ros/verify-execute-trajectory-interop.sh`: the constructor discards
//! `wait_for_action_server`'s return, so a missing server costs one silent
//! `wait_for_servers` timeout and nothing else. Binding it does not unblock
//! `plan()` -- `plan()` looks at `move_action_client_` alone
//! (`move_group_interface.cpp:659`) -- it removes that timeout and gives
//! `execute`/`asyncExecute` (`move_group_interface.hpp:732,741,750,759`) a
//! server that answers.
//!
//! # What upstream answers, and with which code
//!
//! Read off `executePathCallback`
//! (`execute_trajectory_action_capability.cpp:87-115`) and `executePath`
//! (`execute_trajectory_action_capability.cpp:117-149`). Every row is a
//! terminal transition; upstream has no arm that leaves a goal live.
//!
//! | condition | `error_code.val` | terminal |
//! | --- | --- | --- |
//! | no `trajectory_execution_manager_` (`~allow_trajectory_execution` false) | `CONTROL_FAILED` (-4) | `abort` |
//! | `TrajectoryExecutionManager::push` returned false | `CONTROL_FAILED` (-4) | `abort` |
//! | execution finished `SUCCEEDED` | `SUCCESS` (1) | `succeed` |
//! | execution finished `PREEMPTED` | `PREEMPTED` (-7) | `abort` |
//! | execution finished `TIMED_OUT` | `TIMED_OUT` (-6) | `abort` |
//! | execution finished anything else | `CONTROL_FAILED` (-4) | `abort` |
//!
//! The terminal column is one rule and not six: `executePathCallback`
//! succeeds on `error_code.val == SUCCESS` and aborts otherwise
//! (`execute_trajectory_action_capability.cpp:103-112`). `PREEMPTED`
//! aborting rather than cancelling is upstream's, and is named here because
//! it is the row a reader is most likely to guess wrong.
//!
//! # Which of the two servers this is
//!
//! This is the **no-execution-backend** server, not a server that reports
//! `SUCCESS` having executed nothing.
//!
//! This workspace has no trajectory execution backend.
//! `rg -l -i 'trajectory_execution|TrajectoryExecutionManager|
//! controller_manager|FollowJointTrajectory' crates/ ros/` matches only
//! `pr2.urdf` fixtures (whose `<gazebo>` tags name controllers), the port's
//! own gates, and prose in this crate saying there is none -- no type, no
//! crate, no dependency. There is nothing for the last four rows above to
//! describe.
//!
//! So the port lands on the **first** row, which is not a stub: it is a
//! configuration upstream itself ships and reaches, `~allow_trajectory_
//! execution` set false. [`no_execution_backend`] is the one
//! `ExecuteTrajectory::Result` this module constructs, it always carries
//! `CONTROL_FAILED`, and [`serve`] always aborts. A `SUCCESS` reply is
//! therefore not something a caller of this module can produce -- which is
//! the point. The alternative port, a server answering `SUCCESS` because it
//! has nothing to fail at, is a lie no client can detect: `execute()`
//! returns `SUCCESS`, the robot has not moved, and the next
//! `getCurrentState()` is the first hint.
//!
//! The goal's `trajectory` and `controller_names` are not inspected, and
//! that is deliberate rather than unfinished. Upstream's first row returns
//! before either field is read, so a port that rejected, say, an empty
//! `trajectory` with some other code would be answering a question upstream
//! never asks on this path. One condition, one code, no boundary where the
//! rule changes.
//!
//! # Deviation: upstream builds the explanation and drops it
//!
//! `executePathCallback` assigns
//! `const std::string response = "Cannot execute trajectory since
//! ~allow_trajectory_execution was set to false"`
//! (`execute_trajectory_action_capability.cpp:92`) and never reads it:
//! `action_res->error_code.message` is left empty, so the sentence never
//! reaches the client and it sees a bare `-4`. This port puts its own
//! sentence in `message` and names itself in `source`, which is what
//! `/move_action` and `/plan_kinematic_path` already do here. The deviation
//! is additive -- a client branching on `val` sees upstream's value -- but
//! it is a deviation, and the dead store is recorded in
//! `doc/upstream-bugs.md`.
//!
//! # The cancel path
//!
//! Upstream registers two constant callbacks
//! (`execute_trajectory_action_capability.cpp:79-83`): the goal callback
//! returns `GoalResponse::ACCEPT_AND_EXECUTE` and the cancel callback
//! returns `CancelResponse::ACCEPT`, both ignoring their arguments. It also
//! defines `preemptExecuteTrajectoryCallback`
//! (`execute_trajectory_action_capability.cpp:151-154`), whose body is
//! `stopExecution(true)` -- and **nothing calls it**. `rg -n
//! preemptExecuteTrajectoryCallback` over the whole upstream tree returns
//! two hits, the declaration
//! (`execute_trajectory_action_capability.hpp:66`) and that definition. An
//! upstream cancel is therefore accepted, the client is told the goal is
//! cancelling, and the trajectory runs to completion.
//!
//! That is the behaviour this port must not reproduce, and it is why the
//! answer here is `reject`: nothing is executing, so there is nothing to
//! stop, and a rejection is the only answer a client can act on. Accepting
//! would be upstream-faithful and wrong in the one way the client can
//! observe.
//!
//! Rejecting is also the only answer that is safe under r2r's shape. A
//! cancel request arrives on a per-goal channel handed out by
//! `ActionServerGoalRequest::accept`, and there are three ways to get it
//! wrong, each costing the client differently (read from
//! `r2r/src/action_servers.rs` at the pinned rev):
//!
//! * Drop the receiver. r2r's `try_send` fails, it logs, and that goal
//!   contributes no channel -- so the response is never filtered, the client
//!   is told `ERROR_NONE` with the goal still listed under
//!   `goals_canceling`, and nothing cancels it. This is upstream's own
//!   failure, reproduced by omission.
//! * Take the request and drop it unanswered. The oneshot resolves
//!   `Canceled`, r2r drops the request and **no `CancelGoal` response is
//!   ever sent** -- the client's call hangs until its own timeout.
//! * Accept it and never reach a terminal transition. The goal moves to
//!   CANCELING and `get_result` never resolves.
//!
//! [`serve`] takes the first off the table by keeping the receiver, and the
//! third by terminating every accepted goal before it yields. It cannot take
//! the second: `reject_pending_cancels` answers every request it takes, and
//! that is the property the `CancelRequest` seam exists to make testable
//! rather than merely stated. Both are private, and that follows from
//! [`serve`] being the single owner of an accepted goal handle: a cancel
//! stream exists only as the second half of `accept`'s return, [`serve`] is
//! the only thing in this crate that calls `accept`, so a caller outside this
//! module cannot hold one to drain. A public drain would be an API for a
//! value no caller can obtain.
//!
//! Today no cancel request can reach that drain, and the reason is
//! structural rather than lucky: [`serve`] terminates the goal in the same
//! poll in which it accepts it, with no `await` in between, so the goal is
//! never in rcl's cancelable set at a moment when `spin_once` could process
//! a cancel request against it. The drain runs anyway, because
//! "unreachable" is a property of this handler and not of the wire -- a
//! future handler that awaits anything between accept and terminate
//! re-opens it.

use futures::FutureExt;
use futures::stream::{Stream, StreamExt};
use r2r::moveit_msgs::action::ExecuteTrajectory;
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::{ActionServerCancelRequest, ActionServerGoalRequest};

/// `MoveItErrorCodes::source` on every reply from this endpoint.
///
/// The endpoint, not the binary, for the reason `src/bin/move_group.rs`
/// records about its own two: a `source` built from the binary name goes
/// stale on the wire the moment the binary is renamed, which has already
/// happened once (PORTING-PLAN.md §255).
pub const SOURCE: &str = "cspace-ros/execute_trajectory";

/// The sentence this port sends where upstream sends an empty `message`.
///
/// It names the upstream branch rather than describing the port, so a client
/// that logs `message` records *which of upstream's six rows* it landed on
/// -- the thing the bare `-4` cannot say, since three of those rows carry
/// it.
pub const NO_EXECUTION_BACKEND: &str = "cspace-ros has no trajectory execution backend, so this is upstream's own \
     `!trajectory_execution_manager_` branch (~allow_trajectory_execution false), \
     which aborts with CONTROL_FAILED before reading the goal";

/// The one `ExecuteTrajectory::Result` this module constructs.
///
/// Always `CONTROL_FAILED`, because the condition it reports is a property
/// of this workspace and not of the goal -- see this module's doc. Nothing
/// here builds a `SUCCESS` result, which is what makes [`serve`]'s
/// unconditional `abort` a total rule rather than the reachable half of a
/// branch.
pub fn no_execution_backend() -> ExecuteTrajectory::Result {
    ExecuteTrajectory::Result {
        error_code: MoveItErrorCodes {
            val: MoveItErrorCodes::CONTROL_FAILED as i32,
            message: NO_EXECUTION_BACKEND.to_string(),
            source: SOURCE.to_string(),
        },
    }
}

/// A cancel request that can be answered.
///
/// One method, one production implementor, and it exists for one reason:
/// `ActionServerCancelRequest`'s fields are private to `r2r` and its
/// `reject` consumes a `oneshot::Sender` no test can observe, so without
/// this seam [`reject_pending_cancels`] could only ever be tested on streams
/// that yield *nothing*. Those tests cannot tell "answers every request"
/// from "drops every request", which is the one property that matters here:
/// a taken-and-dropped request is the failure mode that hangs the client
/// (see this module's doc). The seam is what lets a test hold a real
/// request, watch it be answered, and fail when it is not.
///
/// Private for the reason this module's doc gives: the only source of a
/// cancel stream is [`serve`]'s own `accept`, so there is no caller outside
/// this module that could have one.
trait CancelRequest {
    /// Refuse the cancellation, sending the client a `CancelGoal` response
    /// that says so.
    fn reject(self);
}

impl CancelRequest for ActionServerCancelRequest {
    fn reject(self) {
        ActionServerCancelRequest::reject(self)
    }
}

/// Answers every cancel request currently queued on `cancels`, and returns
/// how many it answered.
///
/// `now_or_never` rather than `.await`: this runs after the goal it belongs
/// to is already terminal, so awaiting would park [`serve`]'s loop forever
/// on a stream that can never yield again, and no later goal would be
/// served. Draining what is there and returning is the whole contract.
///
/// Every request taken is answered -- there is no path from `next()` to the
/// end of the loop body that skips `reject`. See this module's doc for what
/// a taken-and-unanswered request costs the client.
fn reject_pending_cancels<C: CancelRequest>(
    cancels: &mut (impl Stream<Item = C> + Unpin),
) -> usize {
    let mut answered = 0;
    // `Some(Some(_))` and not `Some(_)`: `now_or_never` reports "the stream
    // is finished" as `Some(None)`, which a looser pattern would retry
    // forever.
    while let Some(Some(request)) = cancels.next().now_or_never() {
        request.reject();
        answered += 1;
    }
    answered
}

/// Serves `execute_trajectory` goals until `goal_requests` ends.
///
/// The single owner of an accepted goal handle: the handle is created and
/// terminated inside one loop iteration with no `await` between the two, and
/// no path out of the iteration skips the terminal transition. A caller
/// never holds an `ActionServerGoal`, so it cannot leave one live -- which
/// is the only way a client of this endpoint can be made to wait forever.
///
/// Upstream's goal callback is a constant `ACCEPT_AND_EXECUTE` inspecting
/// neither the UUID nor the goal
/// (`execute_trajectory_action_capability.cpp:79-81`), so there is no
/// rejection branch to port.
///
/// No feedback is published, matching the branch this port lands on:
/// upstream's first row aborts at
/// `execute_trajectory_action_capability.cpp:94` without publishing any.
/// `setExecuteTrajectoryState` is reached only on the `push` success path
/// (`execute_trajectory_action_capability.cpp:124`) and after the goal is
/// already terminal (`execute_trajectory_action_capability.cpp:114`) -- and
/// that second call is one rclcpp_action logs an error for and publishes
/// nothing from, the same call `/move_action` here already declines to port
/// for the same reason.
pub async fn serve(
    mut goal_requests: impl Stream<Item = ActionServerGoalRequest<ExecuteTrajectory::Action>> + Unpin,
) {
    while let Some(request) = goal_requests.next().await {
        let (mut goal, mut cancels) = match request.accept() {
            Ok(accepted) => accepted,
            Err(e) => {
                eprintln!("accepting execute_trajectory goal: {e}");
                continue;
            }
        };
        if let Err(e) = goal.abort(no_execution_backend()) {
            eprintln!("terminating execute_trajectory goal: {e}");
        }
        let answered = reject_pending_cancels(&mut cancels);
        if answered > 0 {
            eprintln!(
                "execute_trajectory: rejected {answered} cancel request(s) -- nothing is \
                 executing, so there is nothing to stop"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    /// A [`CancelRequest`] that records having been answered, so a drop
    /// instead of a `reject` is visible to a test.
    struct RecordingCancel {
        id: u32,
        answered: Rc<RefCell<Vec<u32>>>,
    }

    impl CancelRequest for RecordingCancel {
        fn reject(self) {
            self.answered.borrow_mut().push(self.id);
        }
    }

    /// The value a client branches on. `-4` is spelled out rather than
    /// written as `MoveItErrorCodes::CONTROL_FAILED`, because the constant
    /// and the assertion would then be the same symbol and the test could
    /// not tell a wrong constant from a right one.
    #[test]
    fn the_only_result_this_port_builds_is_upstreams_control_failed() {
        assert_eq!(no_execution_backend().error_code.val, -4);
    }

    /// Not `!= SUCCESS`: `SUCCESS` is `1` and every other code upstream can
    /// answer here is negative, so a mutation returning any of them would
    /// pass a `!= SUCCESS` check. These are the codes upstream's own table
    /// can produce, named apart, plus the one this same node's planning
    /// endpoints answer with.
    #[test]
    fn the_result_is_none_of_the_codes_a_real_backend_would_report() {
        let val = no_execution_backend().error_code.val;
        assert_ne!(val, MoveItErrorCodes::SUCCESS as i32);
        assert_ne!(val, MoveItErrorCodes::PREEMPTED as i32);
        assert_ne!(val, MoveItErrorCodes::TIMED_OUT as i32);
        assert_ne!(
            val,
            MoveItErrorCodes::FAILURE as i32,
            "FAILURE is what this node's planning endpoints answer for a missing pipeline; \
             a missing executor is a different condition with its own upstream code"
        );
    }

    /// `source` separates an answer built by this endpoint from one built by
    /// `/move_action` beside it, or by a client that never reached this
    /// node. Both halves are asserted -- the value, and that it is not the
    /// neighbouring endpoint's.
    #[test]
    fn the_result_names_this_endpoint_as_its_source() {
        let result = no_execution_backend();
        assert_eq!(result.error_code.source, "cspace-ros/execute_trajectory");
        assert_ne!(result.error_code.source, "cspace-ros/move_action");
    }

    /// The assertion is on the condition the sentence names, not on its
    /// wording: a client reading `message` learns which of upstream's six
    /// rows it landed on, which the bare `-4` cannot say.
    #[test]
    fn the_message_names_the_upstream_branch_this_port_lands_on() {
        let message = no_execution_backend().error_code.message;
        assert!(
            message.contains("trajectory_execution_manager_"),
            "the message must name the upstream branch, got: {message}"
        );
        assert!(
            message.contains("CONTROL_FAILED"),
            "the message must name the code it carries, got: {message}"
        );
    }

    /// The property the client depends on: every request the drain takes off
    /// the stream is answered. A `reject` replaced by a drop leaves
    /// `answered` empty here while the returned count is unchanged, which is
    /// exactly the r2r failure mode that hangs a client's `CancelGoal` call.
    #[test]
    fn the_drain_answers_every_request_it_takes() {
        let answered = Rc::new(RefCell::new(Vec::new()));
        let mut cancels = futures::stream::iter((0..3).map(|id| RecordingCancel {
            id,
            answered: Rc::clone(&answered),
        }));
        assert_eq!(reject_pending_cancels(&mut cancels), 3);
        assert_eq!(*answered.borrow(), vec![0, 1, 2]);
    }

    /// A finished stream must not spin the drain forever: `next()` on it
    /// resolves to `None`, which `now_or_never` reports as `Some(None)` -- a
    /// shape the loop pattern has to reject rather than retry.
    #[test]
    fn draining_a_finished_cancel_stream_answers_nothing_and_returns() {
        let mut cancels = futures::stream::empty::<RecordingCancel>();
        assert_eq!(reject_pending_cancels(&mut cancels), 0);
    }

    /// A stream that is merely *pending* must also return rather than park.
    /// This is the case the drain exists for: after the goal is terminal its
    /// cancel channel stays open and empty forever, and an `.await` there
    /// would stop [`serve`] taking the next goal.
    #[test]
    fn draining_a_pending_cancel_stream_returns_instead_of_parking() {
        let mut cancels = futures::stream::pending::<RecordingCancel>();
        assert_eq!(reject_pending_cancels(&mut cancels), 0);
    }
}
