// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Not a direct upstream port — see this type's doc comment.

use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::error::Result;
use crate::geometry::Isometry3;
use crate::model::RobotModel;
use crate::state::RobotState;

use crate::kinematics::cart_to_jnt::{SolveContext, search_position_ik};
use crate::kinematics::chain::ChainInfo;
use crate::kinematics::params::SolverParams;
use crate::kinematics::registry::{
    KINEMATICS_SOLVERS, KinematicsSolver, SolveOptions, SolverRegistration,
};

/// A Levenberg-Marquardt (Tikhonov-damped least squares) alternative to
/// [`crate::kinematics::NewtonRaphsonSolver`], sharing every part of the Newton
/// iteration (`cart_to_jnt`) and the mimic-aware SVD fold
/// (`velocity::solve_velocity`) except the pseudo-inverse's
/// singular-value transfer function.
///
/// # Deviation from upstream: not a port of any single upstream type
///
/// `kdl_kinematics_plugin` ships exactly one velocity solver,
/// `ChainIkSolverVelMimicSVD`, and it is truncated-SVD
/// ([`crate::kinematics::NewtonRaphsonSolver`] ports it directly). Upstream has no
/// second, damped solver to port. This type exists because damped least
/// squares — `f(s) = s / (s^2 + lambda^2)` in place of truncated-SVD's hard
/// `f(s) = 1/s` cutoff — is the standard, well-documented
/// "Levenberg-Marquardt inverse kinematics" formulation (Wampler 1986;
/// Sugihara 2011), and the task this crate serves asks for both a
/// Newton-Raphson and an LMA solver over the same Jacobian. Unlike a hard
/// cutoff, `f(s)` here is smooth and never exactly zero, which trades a
/// small residual velocity in the truncated singular directions for no
/// discontinuity in the step size right at the truncation boundary —
/// [`SolverParams::lma_lambda`] controls how much.
pub struct LevenbergMarquardtSolver {
    model: RobotModel,
    chain: ChainInfo,
    params: SolverParams,
    joint_weights: Vec<f64>,
    rng: ChaCha8Rng,
}

/// See `newton_raphson::DEFAULT_SEED` — the same rationale applies here.
const DEFAULT_SEED: u64 = 0;

impl LevenbergMarquardtSolver {
    /// See [`crate::kinematics::NewtonRaphsonSolver::new`], including its `# Deviation`
    /// on why there is no OS-entropy default.
    ///
    /// # Errors
    ///
    /// `SolverParams::validate`, or see `ChainInfo::build`'s `# Errors`.
    pub fn new(model: &RobotModel, group_name: &str, params: &SolverParams) -> Result<Self> {
        Self::new_with_seed(model, group_name, params, DEFAULT_SEED)
    }

    /// See [`crate::kinematics::NewtonRaphsonSolver::new_with_seed`].
    ///
    /// # Errors
    ///
    /// `SolverParams::validate`, or see `ChainInfo::build`'s `# Errors`.
    pub fn new_with_seed(
        model: &RobotModel,
        group_name: &str,
        params: &SolverParams,
        seed: u64,
    ) -> Result<Self> {
        params.validate()?;
        let chain = ChainInfo::build(model, group_name)?;
        let joint_weights = chain.resolve_joint_weights(params)?;
        Ok(Self {
            model: model.clone(),
            chain,
            params: params.clone(),
            joint_weights,
            rng: ChaCha8Rng::seed_from_u64(seed),
        })
    }
}

impl KinematicsSolver for LevenbergMarquardtSolver {
    fn group_name(&self) -> &str {
        &self.chain.group_name
    }

    fn joint_names(&self) -> &[String] {
        self.chain.solver_joint_names()
    }

    fn base_frame(&self) -> &str {
        self.chain.base_frame()
    }

    fn tip_frame(&self) -> &str {
        self.chain.tip_frame()
    }

    fn solve_with_options(
        &mut self,
        seed: &[f64],
        target: &Isometry3,
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        let lambda = self.params.lma_lambda;
        let pinv = |s: f64, _smax: f64| s / (s * s + lambda * lambda);
        // See `NewtonRaphsonSolver::solve`'s comment on this same call: a
        // bare `RobotState::new` leaves any non-chain variable (e.g. a
        // floating virtual joint's quaternion) at raw `0.0`, not identity.
        let mut state = RobotState::new(&self.model);
        state.set_to_default_values();
        let ctx = SolveContext {
            chain: &self.chain,
            params: &self.params,
            joint_weights: &self.joint_weights,
        };
        search_position_ik(
            &ctx,
            &mut state,
            seed,
            target,
            &pinv,
            &mut self.rng,
            options,
        )
    }
}

// See `registry::KINEMATICS_SOLVERS`'s doc comment: this crate's
// `unsafe_code` lint is `allow`, so the `distributed_slice` macro's
// generated static needs no source-level suppression here.
#[linkme::distributed_slice(KINEMATICS_SOLVERS)]
static LMA: SolverRegistration = SolverRegistration {
    name: "lma",
    construct: |model, group_name, params| {
        LevenbergMarquardtSolver::new(model, group_name, params)
            .map(|solver| Box::new(solver) as Box<dyn KinematicsSolver>)
    },
};
