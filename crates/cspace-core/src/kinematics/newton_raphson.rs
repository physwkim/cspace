// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp
// (`KDLKinematicsPlugin::initialize`'s solver-construction tail: resolved
// joint weights, RNG). This file's `pinv` truncation rule is *not* ported
// from `chainiksolver_vel_mimic_svd.{hpp,cpp}` — see [`NewtonRaphsonSolver`]'s
// own doc comment for why citing that LGPL-2.1-or-later file would be wrong
// even as an interface-fact pointer.

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

/// The singular-value pseudo-inverse this solver's velocity step uses is
/// *truncated*, not damped — a singular value at or below
/// `params.svd_threshold * largest_singular_value` is treated as exactly
/// zero. This is Eigen's own public, documented `JacobiSVD::setThreshold`
/// RELATIVE contract: singular values above `threshold * |largest|` are the
/// nonzero ones.
///
/// The LGPL-2.1-or-later `chainiksolver_vel_mimic_svd.hpp` does describe a
/// truncation rule in its own text, at `:59` — but an *absolute* one
/// ("`@param threshold` if a singular value is below this value, its inverse
/// is set to zero"), which is not what its own `.cpp:62` `setThreshold` call
/// does. So this port does not restate that file: it implements the relative
/// contract Eigen documents, which that file's prose gets wrong. The rule
/// itself lives inside Eigen's (MPL2-licensed) `JacobiSVD`, which this crate
/// has never read.
///
/// Register this solver under `"newton_raphson"` in [`KINEMATICS_SOLVERS`],
/// or construct it directly with [`NewtonRaphsonSolver::new`].
pub struct NewtonRaphsonSolver {
    model: RobotModel,
    chain: ChainInfo,
    params: SolverParams,
    joint_weights: Vec<f64>,
    rng: ChaCha8Rng,
}

/// The seed [`NewtonRaphsonSolver::new`] falls back to — see that method's
/// doc comment.
const DEFAULT_SEED: u64 = 0;

impl NewtonRaphsonSolver {
    /// `KDLKinematicsPlugin::initialize`, seeded from
    /// `DEFAULT_SEED` — see [`NewtonRaphsonSolver::new_with_seed`] for
    /// caller-controlled seeding, and this method's `# Deviation` for why
    /// there is no OS-entropy default.
    ///
    /// # Deviation from upstream: no OS-entropy default
    ///
    /// Upstream's own `random_numbers::RandomNumberGenerator` seeds itself
    /// from system time. This workspace's `rand` dependency is pinned
    /// `default-features = false, features = ["std"]` (no
    /// `os_rng`/`thread_rng` feature), matching
    /// [`crate::state::RobotState`]'s own established convention of never
    /// owning a self-seeded RNG — every randomized method there takes
    /// `&mut impl Rng` from the caller instead.
    /// [`crate::kinematics::registry::SolverRegistration::construct`]'s fixed
    /// three-argument function-pointer signature has no room for a
    /// caller-supplied seed either, so a solver built through
    /// [`KINEMATICS_SOLVERS`] gets `DEFAULT_SEED` rather than entropy; a
    /// caller who needs their own seeding calls
    /// [`NewtonRaphsonSolver::new_with_seed`] directly instead of going
    /// through the registry.
    ///
    /// # Errors
    ///
    /// `SolverParams::validate`, or see `ChainInfo::build`'s `# Errors`.
    pub fn new(model: &RobotModel, group_name: &str, params: &SolverParams) -> Result<Self> {
        Self::new_with_seed(model, group_name, params, DEFAULT_SEED)
    }

    /// [`NewtonRaphsonSolver::new`], seeded deterministically — every
    /// `searchPositionIK`-equivalent reseed and stuck-wiggle draw becomes
    /// reproducible from `seed`.
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

impl KinematicsSolver for NewtonRaphsonSolver {
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
        let svd_threshold = self.params.svd_threshold;
        let pinv = |s: f64, smax: f64| {
            if s > svd_threshold * smax {
                1.0 / s
            } else {
                0.0
            }
        };
        // `RobotState::new` alone leaves every model variable at raw `0.0`
        // (see that method's doc comment) -- valid for `chain`'s own joints,
        // which `cart_to_jnt` overwrites every iteration, but not
        // necessarily for a variable outside the chain: a floating virtual
        // joint's quaternion at `(0, 0, 0, 0)` is not a unit rotation, and
        // every downstream global link transform is degenerate until it is.
        // `set_to_default_values` gives every non-chain variable a real
        // default (identity for a floating joint) before the loop starts;
        // `apply_full` never touches a variable outside `chain`, so this one
        // call keeps them valid for the whole solve.
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
static NEWTON_RAPHSON: SolverRegistration = SolverRegistration {
    name: "newton_raphson",
    construct: |model, group_name, params| {
        NewtonRaphsonSolver::new(model, group_name, params)
            .map(|solver| Box::new(solver) as Box<dyn KinematicsSolver>)
    },
};
