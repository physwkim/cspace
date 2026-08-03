// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Wire protocol shared with the C++ oracle (`tools/moveit-oracle`).
//!
//! One JSON object per line in each direction. The oracle is launched once
//! with a URDF/SRDF pair and then answers requests until stdin closes, so the
//! cost of parsing the robot description is paid once per run rather than once
//! per case.
//!
//! Keep this file and `tools/moveit-oracle/src/oracle.cpp` in step — the C++
//! side hand-rolls the same shapes.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// A request sent to the oracle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Request {
    /// Correlates the response. Monotonic within a run.
    pub id: u64,
    /// What to compute.
    #[serde(flatten)]
    pub op: Op,
}

/// The operations the oracle understands.
///
/// Phase 1 needs [`Op::ModelInfo`]; Phase 2 needs [`Op::Fk`] and
/// [`Op::Jacobian`]. Later phases extend this enum; the oracle answers
/// `ok: false` for an op it does not implement, which is what lets a newer
/// runner talk to an older oracle binary without a version handshake.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Op {
    /// Structural facts about the loaded `RobotModel`.
    ModelInfo,
    /// Forward kinematics for the named links at the given joint values.
    Fk {
        /// Joint name to position. Joints omitted keep their default value.
        joint_values: BTreeMap<String, f64>,
        /// Links to report. Empty means every link in the model.
        #[serde(default)]
        links: Vec<String>,
    },
    /// Draw `count` whole-model random states from the oracle's own sampler.
    ///
    /// The oracle owns randomness so that floating-joint quaternions come out
    /// normalized, bounds are respected per joint type and mimic values are
    /// derived — none of which a variable-by-variable sampler here would get
    /// right. A run is reproducible from `seed`.
    RandomStates {
        /// How many states to draw.
        count: usize,
        /// Seed for `random_numbers::RandomNumberGenerator`.
        seed: i32,
    },
    /// Geometric Jacobian of `group` at the given joint values.
    Jacobian {
        /// Joint model group name.
        group: String,
        /// Joint name to position.
        joint_values: BTreeMap<String, f64>,
    },
}

/// A response from the oracle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Response {
    /// Echoes [`Request::id`].
    pub id: u64,
    /// Whether `result` is present.
    pub ok: bool,
    /// Present when `ok`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<OracleResult>,
    /// Present when `!ok`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// The payload of a successful [`Response`].
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum OracleResult {
    /// Answer to [`Op::ModelInfo`].
    ModelInfo(ModelInfo),
    /// Answer to [`Op::Fk`].
    Fk(FkResult),
    /// Answer to [`Op::RandomStates`].
    RandomStates(RandomStatesResult),
    /// Answer to [`Op::Jacobian`].
    Jacobian(JacobianResult),
}

/// Structural facts about a `RobotModel`, used by the Phase 1 completion check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelInfo {
    /// `RobotModel::getName()`.
    pub name: String,
    /// `RobotModel::getModelFrame()`.
    pub model_frame: String,
    /// `RobotModel::getRootLinkName()`.
    pub root_link: String,
    /// Every link name, in the model's own order.
    pub links: Vec<String>,
    /// Every joint name, in the model's own order.
    pub joints: Vec<String>,
    /// Per-joint facts.
    pub joint_details: Vec<JointDetail>,
    /// Group name to the joint names it contains.
    pub groups: BTreeMap<String, Vec<String>>,
}

/// Per-joint facts compared in the Phase 1 completion check.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointDetail {
    /// Joint name.
    pub name: String,
    /// `JointModel::getTypeName()`: `Revolute`, `Prismatic`, `Planar`,
    /// `Floating`, `Fixed`, `Unknown`. Capitalized exactly so — upstream
    /// returns these strings verbatim from a switch on its type enum, not the
    /// enumerator spelling.
    pub type_name: String,
    /// `JointModel::getVariableNames()`, in the model's own order.
    ///
    /// The oracle reports these rather than the runner deriving them: MoveIt
    /// names a multi-DOF joint's variables `<joint>/trans_x`, `<joint>/rot_w`
    /// and so on, and any convention invented here would silently disagree.
    pub variable_names: Vec<String>,
    /// Per-variable `(min, max)` position bounds, parallel to
    /// `variable_names`. `None` on a side means unbounded: JSON has no
    /// infinity, and a floating joint's translation limits are infinite while
    /// `position_bounded` still reads true.
    pub bounds: Vec<(Option<f64>, Option<f64>)>,
    /// Per-variable `VariableBounds::position_bounded_`, parallel to
    /// `variable_names`.
    pub position_bounded: Vec<bool>,
    /// The joint this one mimics, if any, with its multiplier and offset.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mimic: Option<Mimic>,
}

/// A mimic relationship.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mimic {
    /// Name of the mimicked joint.
    pub joint: String,
    /// `value = multiplier * mimicked + offset`.
    pub multiplier: f64,
    /// See `multiplier`.
    pub offset: f64,
}

/// Answer to [`Op::Fk`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FkResult {
    /// Link name to its global transform, row-major 4x4.
    pub link_transforms: BTreeMap<String, [f64; 16]>,
}

/// Answer to [`Op::RandomStates`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RandomStatesResult {
    /// One map of variable name to position per state.
    pub states: Vec<BTreeMap<String, f64>>,
}

/// Answer to [`Op::Jacobian`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JacobianResult {
    /// Row count (6).
    pub rows: usize,
    /// Column count (group DOF).
    pub cols: usize,
    /// Row-major `rows * cols` entries.
    pub data: Vec<f64>,
}
