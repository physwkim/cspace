// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2021, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/exceptions/include/moveit/exceptions/exceptions.hpp
//   moveit_core/utils/include/moveit/utils/moveit_error_code.hpp
//   moveit_msgs/msg/MoveItErrorCodes.msg   (numeric values are wire-exact)

//! Error types for moveit-rs.
//!
//! Two distinct things live here, matching the upstream split:
//!
//! - [`MoveItErrorCode`] — the *result* enum that planning, kinematics and
//!   execution report. Upstream this is `moveit_msgs::msg::MoveItErrorCodes`,
//!   an `int32` field wrapped by `moveit::core::MoveItErrorCode`.
//!   [`MoveItErrorCode::as_i32`] is wire-exact with the `.msg`, so the optional
//!   `moveit-ros` crate converts without a lookup table of its own.
//! - [`Error`] — the *exceptional* type, replacing upstream's
//!   `moveit::Exception` and `moveit::ConstructException`.
//!
//! # Deviation from upstream
//!
//! Upstream throws `ConstructException` from constructors and `Exception` from
//! unrecoverable paths. This port returns `Result<_, Error>` instead; nothing
//! panics on malformed robot descriptions. [`Error::Construct`] is the variant
//! that corresponds to `ConstructException`.
//!
//! # Why [`Error::Construct`]/[`Error::Other`] are not split per failure mode
//!
//! Asked and decided deliberately, not left undone. Tests that need to pin
//! *which* branch of a call rejected them currently match on the rendered
//! message (`assert_err_mentions` in `moveit-constraints/tests/decide.rs`),
//! because two sibling branches of one call can both produce
//! [`Error::Construct`] and a bare `.is_err()` cannot tell them apart. The
//! obvious structural answer — give each branch its own variant — is the
//! wrong one here, on counts (`rg`, `crates/` + `ros/`, construction calls
//! plus match/pattern sites, counted separately so neither hides in the
//! other): [`Error::Construct`] has 121 sites (89 `Error::construct(...)`
//! calls, 32 `Error::Construct(...)` matches) and [`Error::Other`] 149 (104
//! calls, 45 matches), spread across every crate as deliberate
//! message-carrying catch-alls for exactly the two upstream types named
//! above. Per-branch variants would mean hundreds of them, and the enum
//! would stop describing a hierarchy and start listing call sites.
//!
//! The structured form already exists where the discriminating information
//! is itself structured rather than prose: [`Error::UnknownName`] carries
//! `{ kind, name }` and is used at 42 sites (20 `Error::unknown_name(...)`
//! calls, 22 `Error::UnknownName { .. }` matches), including
//! `constraint_sampler_manager.rs`'s own branch discrimination, which needs
//! no string matching at all. That is the rule to apply when adding a
//! variant — structure the error when the thing that distinguishes it is
//! data a caller would want to read, not merely when a test would find it
//! convenient. Where the distinguishing fact is only "which sentence
//! explains this", the sentence is the right carrier and a test asserting
//! on it is matching the real contract.

use std::fmt;

/// Result alias used across moveit-rs crates.
pub type Result<T> = std::result::Result<T, Error>;

/// An unrecoverable error, replacing upstream's `moveit::Exception` hierarchy.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// Corresponds to upstream `moveit::ConstructException`: an object could
    /// not be built from the description it was given.
    #[error("construction failed: {0}")]
    Construct(String),

    /// Corresponds to upstream `moveit::Exception`.
    #[error("{0}")]
    Other(String),

    /// A name was looked up in the robot model and not found.
    ///
    /// Upstream reports this through `MoveItErrorCodes::INVALID_LINK_NAME`,
    /// `INVALID_GROUP_NAME` and friends; here the lookup failure is separate
    /// from the result code so callers can distinguish "the caller asked for a
    /// link that does not exist" from "planning failed".
    #[error("no {kind} named {name:?}")]
    UnknownName {
        /// What was being looked up: `"link"`, `"joint"`, `"group"`, ...
        kind: &'static str,
        /// The name that was not found.
        name: String,
    },

    /// A parse error while reading URDF or SRDF.
    #[error("{source_kind} parse error: {message}")]
    Parse {
        /// `"URDF"` or `"SRDF"`.
        source_kind: &'static str,
        /// Human-readable detail.
        message: String,
    },

    /// An operation carried a result code that is not success.
    #[error("operation failed: {0}")]
    Code(MoveItErrorCode),
}

impl Error {
    /// Build an [`Error::Construct`].
    pub fn construct(message: impl Into<String>) -> Self {
        Self::Construct(message.into())
    }

    /// Build an [`Error::Other`].
    pub fn other(message: impl Into<String>) -> Self {
        Self::Other(message.into())
    }

    /// Build an [`Error::UnknownName`].
    pub fn unknown_name(kind: &'static str, name: impl Into<String>) -> Self {
        Self::UnknownName {
            kind,
            name: name.into(),
        }
    }
}

/// Result code reported by planning, kinematics and execution.
///
/// Each variant's doc comment names its `moveit_msgs/msg/MoveItErrorCodes.msg`
/// constant at the pinned upstream SHA; [`MoveItErrorCode::as_i32`] and
/// [`From<i32>`](MoveItErrorCode::from) carry the mapping, and the test module
/// pins it against a table transcribed from that `.msg`.
///
/// [`MoveItErrorCode::Unknown`] carries any value the upstream message gains
/// later, so a newer peer cannot make conversion fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum MoveItErrorCode {
    /// `UNDEFINED = 0`
    Undefined,
    /// `SUCCESS = 1`
    Success,
    /// `FAILURE = 99999`
    Failure,

    /// `PLANNING_FAILED = -1`
    PlanningFailed,
    /// `INVALID_MOTION_PLAN = -2`
    InvalidMotionPlan,
    /// `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3`
    MotionPlanInvalidatedByEnvironmentChange,
    /// `CONTROL_FAILED = -4`
    ControlFailed,
    /// `UNABLE_TO_AQUIRE_SENSOR_DATA = -5` (upstream spelling retained)
    UnableToAquireSensorData,
    /// `TIMED_OUT = -6`
    TimedOut,
    /// `PREEMPTED = -7`
    Preempted,

    /// `START_STATE_IN_COLLISION = -10`
    StartStateInCollision,
    /// `START_STATE_VIOLATES_PATH_CONSTRAINTS = -11`
    StartStateViolatesPathConstraints,
    /// `START_STATE_INVALID = -26`
    StartStateInvalid,

    /// `GOAL_IN_COLLISION = -12`
    GoalInCollision,
    /// `GOAL_VIOLATES_PATH_CONSTRAINTS = -13`
    GoalViolatesPathConstraints,
    /// `GOAL_CONSTRAINTS_VIOLATED = -14`
    GoalConstraintsViolated,
    /// `GOAL_STATE_INVALID = -27`
    GoalStateInvalid,
    /// `UNRECOGNIZED_GOAL_TYPE = -28`
    UnrecognizedGoalType,

    /// `INVALID_GROUP_NAME = -15`
    InvalidGroupName,
    /// `INVALID_GOAL_CONSTRAINTS = -16`
    InvalidGoalConstraints,
    /// `INVALID_ROBOT_STATE = -17`
    InvalidRobotState,
    /// `INVALID_LINK_NAME = -18`
    InvalidLinkName,
    /// `INVALID_OBJECT_NAME = -19`
    InvalidObjectName,

    /// `FRAME_TRANSFORM_FAILURE = -21`
    FrameTransformFailure,
    /// `COLLISION_CHECKING_UNAVAILABLE = -22`
    CollisionCheckingUnavailable,
    /// `ROBOT_STATE_STALE = -23`
    RobotStateStale,
    /// `SENSOR_INFO_STALE = -24`
    SensorInfoStale,
    /// `COMMUNICATION_FAILURE = -25`
    CommunicationFailure,
    /// `CRASH = -29`
    Crash,
    /// `ABORT = -30`
    Abort,

    /// `NO_IK_SOLUTION = -31`
    NoIkSolution,

    /// A value this build does not know. Preserves the raw `int32`.
    Unknown(i32),
}

impl MoveItErrorCode {
    /// The raw `int32` this code serializes to.
    pub const fn as_i32(self) -> i32 {
        match self {
            Self::Unknown(v) => v,
            Self::Undefined => 0,
            Self::Success => 1,
            Self::Failure => 99999,
            Self::PlanningFailed => -1,
            Self::InvalidMotionPlan => -2,
            Self::MotionPlanInvalidatedByEnvironmentChange => -3,
            Self::ControlFailed => -4,
            Self::UnableToAquireSensorData => -5,
            Self::TimedOut => -6,
            Self::Preempted => -7,
            Self::StartStateInCollision => -10,
            Self::StartStateViolatesPathConstraints => -11,
            Self::StartStateInvalid => -26,
            Self::GoalInCollision => -12,
            Self::GoalViolatesPathConstraints => -13,
            Self::GoalConstraintsViolated => -14,
            Self::GoalStateInvalid => -27,
            Self::UnrecognizedGoalType => -28,
            Self::InvalidGroupName => -15,
            Self::InvalidGoalConstraints => -16,
            Self::InvalidRobotState => -17,
            Self::InvalidLinkName => -18,
            Self::InvalidObjectName => -19,
            Self::FrameTransformFailure => -21,
            Self::CollisionCheckingUnavailable => -22,
            Self::RobotStateStale => -23,
            Self::SensorInfoStale => -24,
            Self::CommunicationFailure => -25,
            Self::Crash => -29,
            Self::Abort => -30,
            Self::NoIkSolution => -31,
        }
    }

    /// Whether this is `SUCCESS`.
    ///
    /// Upstream `MoveItErrorCode` has `operator bool()` returning
    /// `val == SUCCESS` (`moveit_error_code.hpp`); this is that predicate.
    pub const fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }
}

impl From<i32> for MoveItErrorCode {
    fn from(v: i32) -> Self {
        match v {
            0 => Self::Undefined,
            1 => Self::Success,
            99999 => Self::Failure,
            -1 => Self::PlanningFailed,
            -2 => Self::InvalidMotionPlan,
            -3 => Self::MotionPlanInvalidatedByEnvironmentChange,
            -4 => Self::ControlFailed,
            -5 => Self::UnableToAquireSensorData,
            -6 => Self::TimedOut,
            -7 => Self::Preempted,
            -10 => Self::StartStateInCollision,
            -11 => Self::StartStateViolatesPathConstraints,
            -26 => Self::StartStateInvalid,
            -12 => Self::GoalInCollision,
            -13 => Self::GoalViolatesPathConstraints,
            -14 => Self::GoalConstraintsViolated,
            -27 => Self::GoalStateInvalid,
            -28 => Self::UnrecognizedGoalType,
            -15 => Self::InvalidGroupName,
            -16 => Self::InvalidGoalConstraints,
            -17 => Self::InvalidRobotState,
            -18 => Self::InvalidLinkName,
            -19 => Self::InvalidObjectName,
            -21 => Self::FrameTransformFailure,
            -22 => Self::CollisionCheckingUnavailable,
            -23 => Self::RobotStateStale,
            -24 => Self::SensorInfoStale,
            -25 => Self::CommunicationFailure,
            -29 => Self::Crash,
            -30 => Self::Abort,
            -31 => Self::NoIkSolution,
            other => Self::Unknown(other),
        }
    }
}

impl From<MoveItErrorCode> for i32 {
    fn from(c: MoveItErrorCode) -> Self {
        c.as_i32()
    }
}

impl fmt::Display for MoveItErrorCode {
    /// Matches the strings produced by upstream `errorCodeToString`, an
    /// `inline` free function in
    /// `moveit_core/utils/include/moveit/utils/moveit_error_code.hpp:82`.
    ///
    /// The citation this replaces named
    /// `moveit_core/utils/src/moveit_error_code.cpp` and a member
    /// `MoveItErrorCode::toString()`. Neither exists: that directory's only
    /// `toString` is `moveit::core::toString(double)` in `lexical_casts.cpp`,
    /// an unrelated float formatter, and there is no `.cpp` for this header at
    /// all.
    ///
    /// # Deviation from upstream
    ///
    /// All 31 named codes render identically. [`MoveItErrorCode::Unknown`]
    /// does not: upstream's `switch` has no `default`, so any value outside
    /// the 31 falls past it to
    /// `"Unrecognized MoveItErrorCode. This should never happen!"` — one
    /// string for every unrecognized value, discarding which value it was.
    /// This renders `UNKNOWN(v)` instead, keeping `v`.
    ///
    /// The deviation is deliberate and matches what [`MoveItErrorCode`]
    /// already does elsewhere: `Unknown(v)` exists precisely so a code this
    /// port does not know survives a round trip through `as_i32`, and a
    /// `Display` that dropped `v` would make the one case the variant exists
    /// for the one case you cannot diagnose. Upstream can afford to discard it
    /// because its own comment says the case should never happen; for a port
    /// reading messages produced by a *newer* upstream, it is the expected
    /// case, not the impossible one.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::Undefined => "UNDEFINED",
            Self::Success => "SUCCESS",
            Self::Failure => "FAILURE",
            Self::PlanningFailed => "PLANNING_FAILED",
            Self::InvalidMotionPlan => "INVALID_MOTION_PLAN",
            Self::MotionPlanInvalidatedByEnvironmentChange => {
                "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"
            }
            Self::ControlFailed => "CONTROL_FAILED",
            Self::UnableToAquireSensorData => "UNABLE_TO_AQUIRE_SENSOR_DATA",
            Self::TimedOut => "TIMED_OUT",
            Self::Preempted => "PREEMPTED",
            Self::StartStateInCollision => "START_STATE_IN_COLLISION",
            Self::StartStateViolatesPathConstraints => "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            Self::StartStateInvalid => "START_STATE_INVALID",
            Self::GoalInCollision => "GOAL_IN_COLLISION",
            Self::GoalViolatesPathConstraints => "GOAL_VIOLATES_PATH_CONSTRAINTS",
            Self::GoalConstraintsViolated => "GOAL_CONSTRAINTS_VIOLATED",
            Self::GoalStateInvalid => "GOAL_STATE_INVALID",
            Self::UnrecognizedGoalType => "UNRECOGNIZED_GOAL_TYPE",
            Self::InvalidGroupName => "INVALID_GROUP_NAME",
            Self::InvalidGoalConstraints => "INVALID_GOAL_CONSTRAINTS",
            Self::InvalidRobotState => "INVALID_ROBOT_STATE",
            Self::InvalidLinkName => "INVALID_LINK_NAME",
            Self::InvalidObjectName => "INVALID_OBJECT_NAME",
            Self::FrameTransformFailure => "FRAME_TRANSFORM_FAILURE",
            Self::CollisionCheckingUnavailable => "COLLISION_CHECKING_UNAVAILABLE",
            Self::RobotStateStale => "ROBOT_STATE_STALE",
            Self::SensorInfoStale => "SENSOR_INFO_STALE",
            Self::CommunicationFailure => "COMMUNICATION_FAILURE",
            Self::Crash => "CRASH",
            Self::Abort => "ABORT",
            Self::NoIkSolution => "NO_IK_SOLUTION",
            Self::Unknown(v) => return write!(f, "UNKNOWN({v})"),
        };
        f.write_str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every value listed in the pinned `MoveItErrorCodes.msg`, transcribed
    /// from the file rather than from memory.
    const UPSTREAM: &[(i32, MoveItErrorCode)] = &[
        (1, MoveItErrorCode::Success),
        (0, MoveItErrorCode::Undefined),
        (99999, MoveItErrorCode::Failure),
        (-1, MoveItErrorCode::PlanningFailed),
        (-2, MoveItErrorCode::InvalidMotionPlan),
        (
            -3,
            MoveItErrorCode::MotionPlanInvalidatedByEnvironmentChange,
        ),
        (-4, MoveItErrorCode::ControlFailed),
        (-5, MoveItErrorCode::UnableToAquireSensorData),
        (-6, MoveItErrorCode::TimedOut),
        (-7, MoveItErrorCode::Preempted),
        (-10, MoveItErrorCode::StartStateInCollision),
        (-11, MoveItErrorCode::StartStateViolatesPathConstraints),
        (-26, MoveItErrorCode::StartStateInvalid),
        (-12, MoveItErrorCode::GoalInCollision),
        (-13, MoveItErrorCode::GoalViolatesPathConstraints),
        (-14, MoveItErrorCode::GoalConstraintsViolated),
        (-27, MoveItErrorCode::GoalStateInvalid),
        (-28, MoveItErrorCode::UnrecognizedGoalType),
        (-15, MoveItErrorCode::InvalidGroupName),
        (-16, MoveItErrorCode::InvalidGoalConstraints),
        (-17, MoveItErrorCode::InvalidRobotState),
        (-18, MoveItErrorCode::InvalidLinkName),
        (-19, MoveItErrorCode::InvalidObjectName),
        (-21, MoveItErrorCode::FrameTransformFailure),
        (-22, MoveItErrorCode::CollisionCheckingUnavailable),
        (-23, MoveItErrorCode::RobotStateStale),
        (-24, MoveItErrorCode::SensorInfoStale),
        (-25, MoveItErrorCode::CommunicationFailure),
        (-29, MoveItErrorCode::Crash),
        (-30, MoveItErrorCode::Abort),
        (-31, MoveItErrorCode::NoIkSolution),
    ];

    #[test]
    fn discriminants_match_upstream_msg() {
        for &(raw, code) in UPSTREAM {
            assert_eq!(code.as_i32(), raw, "as_i32 for {code}");
            assert_eq!(MoveItErrorCode::from(raw), code, "from({raw})");
        }
    }

    /// Every string upstream's `errorCodeToString` returns, keyed by the code
    /// it returns it for, transcribed from
    /// `moveit_core/utils/include/moveit/utils/moveit_error_code.hpp:86-147`.
    ///
    /// Kept as its own table rather than derived from [`UPSTREAM`]: deriving
    /// the expected string from the variant name would test the derivation,
    /// not the port. `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE` and
    /// `UNABLE_TO_AQUIRE_SENSOR_DATA` (upstream's spelling of "acquire") are
    /// exactly the entries such a derivation would get wrong.
    const UPSTREAM_STRINGS: &[(i32, &str)] = &[
        (1, "SUCCESS"),
        (0, "UNDEFINED"),
        (99999, "FAILURE"),
        (-1, "PLANNING_FAILED"),
        (-2, "INVALID_MOTION_PLAN"),
        (-3, "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"),
        (-4, "CONTROL_FAILED"),
        (-5, "UNABLE_TO_AQUIRE_SENSOR_DATA"),
        (-6, "TIMED_OUT"),
        (-7, "PREEMPTED"),
        (-10, "START_STATE_IN_COLLISION"),
        (-11, "START_STATE_VIOLATES_PATH_CONSTRAINTS"),
        (-26, "START_STATE_INVALID"),
        (-12, "GOAL_IN_COLLISION"),
        (-13, "GOAL_VIOLATES_PATH_CONSTRAINTS"),
        (-14, "GOAL_CONSTRAINTS_VIOLATED"),
        (-27, "GOAL_STATE_INVALID"),
        (-28, "UNRECOGNIZED_GOAL_TYPE"),
        (-15, "INVALID_GROUP_NAME"),
        (-16, "INVALID_GOAL_CONSTRAINTS"),
        (-17, "INVALID_ROBOT_STATE"),
        (-18, "INVALID_LINK_NAME"),
        (-19, "INVALID_OBJECT_NAME"),
        (-21, "FRAME_TRANSFORM_FAILURE"),
        (-22, "COLLISION_CHECKING_UNAVAILABLE"),
        (-23, "ROBOT_STATE_STALE"),
        (-24, "SENSOR_INFO_STALE"),
        (-25, "COMMUNICATION_FAILURE"),
        (-29, "CRASH"),
        (-30, "ABORT"),
        (-31, "NO_IK_SOLUTION"),
    ];

    #[test]
    fn display_matches_upstream_error_code_to_string() {
        for &(raw, expected) in UPSTREAM_STRINGS {
            assert_eq!(
                MoveItErrorCode::from(raw).to_string(),
                expected,
                "Display for code {raw}"
            );
        }
        // The two tables must cover the same codes, or one of them is stale
        // and the coverage this test claims is smaller than it looks.
        assert_eq!(UPSTREAM_STRINGS.len(), UPSTREAM.len());
        for &(raw, _) in UPSTREAM {
            assert!(
                UPSTREAM_STRINGS.iter().any(|&(other, _)| other == raw),
                "code {raw} has a discriminant pinned but no string pinned"
            );
        }
    }

    #[test]
    fn round_trips_through_i32() {
        for &(raw, _) in UPSTREAM {
            assert_eq!(i32::from(MoveItErrorCode::from(raw)), raw);
        }
    }

    #[test]
    fn unknown_preserves_raw_value() {
        // -20 is a gap in the upstream msg; -32 is past its end.
        for raw in [-20, -32, 7, 123_456] {
            let code = MoveItErrorCode::from(raw);
            assert_eq!(code, MoveItErrorCode::Unknown(raw));
            assert_eq!(code.as_i32(), raw);
            assert!(!code.is_success());
        }
    }

    #[test]
    fn only_success_is_success() {
        for &(_, code) in UPSTREAM {
            assert_eq!(
                code.is_success(),
                code == MoveItErrorCode::Success,
                "{code}"
            );
        }
    }
}
