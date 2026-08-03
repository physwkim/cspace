// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_parameters.yaml

use std::collections::HashMap;

/// Search parameters for a [`crate::KinematicsSolver`].
///
/// Upstream reads these from a ROS parameter server, one
/// `kdl_kinematics.<field>` entry per field of
/// `kdl_kinematics_parameters.yaml`, loaded through a generated
/// `ParamListener` inside `initialize(node, ...)`. This port has no node to
/// read from (see this crate's "do not port the ROS surface" doc), so the
/// same defaults become a plain [`Default`] impl the caller can override
/// before constructing a solver.
#[derive(Debug, Clone, PartialEq)]
pub struct SolverParams {
    /// `max_solver_iterations`: the iteration cap inside one `CartToJnt`
    /// call, not a wall-clock timeout — see this crate's doc comment on
    /// why a wall-clock `searchPositionIK` timeout is not ported.
    pub max_solver_iterations: usize,
    /// `epsilon`: `CartToJnt` accepts a step once
    /// `max(position_error, orientation_error)` drops to this or below.
    pub epsilon: f64,
    /// `orientation_vs_position`: relative weight of the three orientation
    /// error rows against the three position error rows.
    /// [`SolverParams::position_only`] overrides this to `0.0` exactly as
    /// upstream's `position_only_ik` overrides `orientation_vs_position`.
    pub orientation_vs_position: f64,
    /// `position_only_ik`.
    pub position_only: bool,
    /// `joints`/`__map_joints.weight`: per-active-joint weight, by joint
    /// name. A name absent here defaults to `1.0`, matching
    /// `KDLKinematicsPlugin::getJointWeights`'s own default-then-override.
    pub joint_weights: HashMap<String, f64>,
    /// `ChainIkSolverVelMimicSVD`'s `threshold` constructor argument: a
    /// singular value below `threshold * (largest singular value)` is
    /// treated as zero. Used by [`crate::NewtonRaphsonSolver`], the solver
    /// that ports `ChainIkSolverVelMimicSVD` as-is.
    pub svd_threshold: f64,
    /// Tikhonov damping constant for [`crate::LevenbergMarquardtSolver`]'s
    /// velocity step: `s / (s^2 + lambda^2)` in place of
    /// [`SolverParams::svd_threshold`]'s hard singular-value cutoff. Not an
    /// upstream field — `kdl_kinematics_plugin` only ever ships the
    /// truncated-SVD solver; see [`crate::LevenbergMarquardtSolver`]'s doc
    /// comment for why a fixed damping constant is still a faithful
    /// Levenberg-Marquardt / damped-least-squares IK step.
    pub lma_lambda: f64,
    /// Not an upstream field: replaces `searchPositionIK`'s wall-clock
    /// `timeout` with a bounded retry count, so a run is reproducible from
    /// a seed rather than from wall-clock timing. Upstream re-seeds
    /// randomly and retries `CartToJnt` until `timedOut`; this port retries
    /// up to this many times instead. `0` matches `getPositionIK`'s own
    /// "single attempt, no re-seeding" behaviour (it calls
    /// `searchPositionIK` with `timeout = 0.0`).
    pub max_restarts: usize,
}

impl Default for SolverParams {
    fn default() -> Self {
        Self {
            max_solver_iterations: 500,
            epsilon: 0.00001,
            orientation_vs_position: 1.0,
            position_only: false,
            joint_weights: HashMap::new(),
            svd_threshold: 0.001,
            lma_lambda: 0.01,
            max_restarts: 20,
        }
    }
}

impl SolverParams {
    /// `orientation_vs_position_weight` at `searchPositionIK`'s call site:
    /// `position_only_ik ? 0.0 : orientation_vs_position`.
    pub(crate) fn orientation_weight(&self) -> f64 {
        if self.position_only {
            0.0
        } else {
            self.orientation_vs_position
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_weight_is_zero_exactly_when_position_only_is_set() {
        let mut params = SolverParams {
            orientation_vs_position: 3.5,
            ..Default::default()
        };
        assert_eq!(params.orientation_weight(), 3.5);

        params.position_only = true;
        assert_eq!(params.orientation_weight(), 0.0);
    }
}
