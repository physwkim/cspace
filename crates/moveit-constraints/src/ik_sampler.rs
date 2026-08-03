// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.hpp
//   (struct IKSamplingPose, class IKConstraintSampler)
//   moveit_core/constraint_samplers/src/default_constraint_samplers.cpp
//   (IKSamplingPose ctors, IKConstraintSampler::{configure(IKSamplingPose),
//    loadIKSolver, getSamplingVolume, getLinkName, samplePose, sample,
//    sampleHelper, validate, callIK})

use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Transforms, UnitQuaternion, Vector3};
use moveit_kinematics::KinematicsSolver;
use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::{Posed, RobotState};
use rand::{Rng, RngExt};

use crate::{ConstraintSampler, OrientationConstraint, OrientationTolerance, PositionConstraint};

/// A position constraint, an orientation constraint, or both, sharing one
/// link — the target [`IkConstraintSampler::sample_pose`] samples within.
///
/// Upstream `IKSamplingPose` has six constructor overloads (empty; either
/// constraint alone; both; and a `Ptr`-sharing variant of each) whose only
/// job is copying one or two already-configured constraints into two
/// fields. A plain two-field struct with public fields is the direct Rust
/// equivalent — matching this crate's own [`crate::ConstraintRegion`], which
/// carries no constructor for the same reason — and a struct literal
/// (`IkSamplingPose { position_constraint: Some(pc), orientation_constraint:
/// None }`) replaces each overload.
#[derive(Debug, Clone, PartialEq)]
pub struct IkSamplingPose {
    /// `position_constraint_`.
    pub position_constraint: Option<PositionConstraint>,
    /// `orientation_constraint_`.
    pub orientation_constraint: Option<OrientationConstraint>,
}

/// Samples poses that satisfy an [`IkSamplingPose`] and solves IK for them.
///
/// Upstream `constraint_samplers::IKConstraintSampler`.
///
/// # Deviation from upstream: does not implement [`crate::ConstraintSampler`]
///
/// Every upstream `ConstraintSampler` subclass owns its `kb_` (the IK
/// solver) for the sampler's whole lifetime, obtained once via
/// `jmg_->getSolverInstance()` at `configure()` time. `PORTING-PLAN.md`
/// §68.4 already excludes that group-to-solver lookup (D4: no runtime
/// plugin-by-string dispatch); the caller passes a solver in instead. A
/// solver argument does not fit [`crate::ConstraintSampler::sample`]'s
/// `(&self, state, rng)` shape — there is no parameter for it, and adding
/// one (or an owned solver field, which would force `&mut self` onto every
/// implementer, including [`crate::JointConstraintSampler`] and
/// [`crate::UnionConstraintSampler`], which need no such thing) reopens the
/// question `PORTING-PLAN.md` §68.4 just closed. `IkConstraintSampler`
/// instead has its own inherent [`IkConstraintSampler::sample`], matching
/// the `rng: &mut dyn Rng` precedent already established on the trait
/// itself: an external mutable resource is a call parameter, not a stored
/// field. One consequence: this type on its own cannot be placed inside a
/// [`crate::UnionConstraintSampler`] (whose element type is `Box<dyn
/// ConstraintSampler>`) — [`IkConstraintSamplerAdapter`], round 10's
/// composition decision, is what goes there instead. See that type's own
/// doc comment for why an adapter, not a wider trait, is the fix.
///
/// # Deviation from upstream: no fixed-link bridging
///
/// `loadIKSolver` falls back to `LinkModel::getAssociatedFixedTransforms()`
/// when the constrained link is not the solver's own tip link but is
/// rigidly attached to it, storing `eef_to_ik_tip_transform_` to bridge the
/// two. `moveit-model`'s `LinkModel` does not carry that accessor at all
/// (`link_model.rs`'s own doc comment: it "serve\[s\] the collision backend
/// ... rather than anything this phase's done-criteria read", and
/// `moveit-model` is out of scope this round). [`IkConstraintSampler::new`]
/// requires an exact match between the solver's tip frame and the
/// constrained link instead of silently falling back, and reports the
/// narrower case as [`Error::Construct`] rather than a bridgeable gap.
///
/// # Deviation from upstream: no `ik_timeout_`
///
/// `getIKTimeout`/`setIKTimeout`/`ik_timeout_` are dropped entirely, not
/// renamed: `moveit-kinematics::KinematicsSolver::solve_with_options` takes
/// no timeout at all (see its own doc comment) — the timeout concept is
/// fully replaced by `SolverParams::max_restarts`, which is configured once
/// on the solver itself, outside this sampler's control.
///
/// # Deviation from upstream: no "no enabled constraints" check
///
/// Upstream's `configure(IKSamplingPose)` rejects a sampling pose whose
/// only present constraint(s) are disabled. [`PositionConstraint`] and
/// [`OrientationConstraint`] have no disabled representation in this port
/// (`crate`'s own Round 6 symbol audit: "always satisfied-by-construction
/// ... illegal/disabled states are prevented at `new()`"), so the check has
/// no case left to catch.
#[derive(Debug, Clone, PartialEq)]
pub struct IkConstraintSampler {
    sampling_pose: IkSamplingPose,
    /// `ik_frame_`, leading `/` already stripped.
    ik_frame: String,
    /// `transform_ik_`.
    transform_ik: bool,
}

impl IkConstraintSampler {
    /// Build and validate an IK constraint sampler against `model` and
    /// `solver`. Upstream's `configure(const IKSamplingPose&)`, combined
    /// with `loadIKSolver` (this port has no separate `kb_ =
    /// jmg_->getSolverInstance()` step to fail independently, since `solver`
    /// is already a live reference).
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `sampling_pose` has neither constraint, if
    /// both are present but target different links, or if `solver`'s tip
    /// frame does not exactly match the constrained link (see this type's
    /// doc comment on fixed-link bridging).
    /// [`Error::UnknownName`] if `solver.base_frame()` differs from
    /// `model.model_frame()` and is not a link of `model` either (mirrors
    /// [`PositionConstraint::new`]'s mobile-frame check; upstream instead
    /// warns and silently disables the transform, which this port's
    /// established "an unresolvable frame is an error, not a warning"
    /// convention — see [`OrientationConstraint::new`]'s doc comment —
    /// rejects at construction instead).
    pub fn new(
        model: &RobotModel,
        solver: &dyn KinematicsSolver,
        sampling_pose: IkSamplingPose,
    ) -> Result<Self> {
        if sampling_pose.position_constraint.is_none()
            && sampling_pose.orientation_constraint.is_none()
        {
            return Err(Error::construct(
                "IkConstraintSampler needs a position constraint, an orientation constraint, or both",
            ));
        }
        if let (Some(pc), Some(oc)) = (
            &sampling_pose.position_constraint,
            &sampling_pose.orientation_constraint,
        ) {
            if pc.link_name() != oc.link_name() {
                return Err(Error::construct(format!(
                    "position and orientation constraints target different links ('{}' vs '{}'); IK-based sampling needs one link",
                    pc.link_name(),
                    oc.link_name()
                )));
            }
        }

        let mut ik_frame = solver.base_frame().to_string();
        if ik_frame.starts_with('/') {
            ik_frame.remove(0);
        }
        let transform_ik = !Transforms::same_frame(&ik_frame, model.model_frame());
        if transform_ik && !model.has_link_model(&ik_frame) {
            return Err(Error::unknown_name("frame", &ik_frame));
        }

        let tip = solver.tip_frame();
        for link_name in [
            sampling_pose
                .position_constraint
                .as_ref()
                .map(PositionConstraint::link_name),
            sampling_pose
                .orientation_constraint
                .as_ref()
                .map(OrientationConstraint::link_name),
        ]
        .into_iter()
        .flatten()
        {
            if !Transforms::same_frame(tip, link_name) {
                return Err(Error::construct(format!(
                    "IK solver's tip frame is '{tip}', not the constrained link '{link_name}' — \
                     IkConstraintSampler ports no fixed-link bridging (see this type's doc comment)"
                )));
            }
        }

        Ok(Self {
            sampling_pose,
            ik_frame,
            transform_ik,
        })
    }

    /// `getSamplingVolume`.
    pub fn sampling_volume(&self) -> f64 {
        let mut v = 1.0;
        if let Some(pc) = &self.sampling_pose.position_constraint {
            let regions = pc.constraint_regions();
            if !regions.is_empty() {
                let vol: f64 = regions.iter().map(|r| r.body.compute_volume()).sum();
                v *= vol;
            }
        }
        if let Some(oc) = &self.sampling_pose.orientation_constraint {
            let (x, y, z) = match oc.tolerance() {
                OrientationTolerance::XyzEuler { x, y, z } => (x, y, z),
                OrientationTolerance::RotationVector { x, y, z } => (x, y, z),
            };
            v *= x * y * z;
        }
        v
    }

    /// `getLinkName`: the orientation constraint's link if one is present,
    /// otherwise the position constraint's (matching upstream's own
    /// preference order); [`IkConstraintSampler::new`] already guarantees
    /// the two agree whenever both are present.
    pub fn link_name(&self) -> &str {
        match (
            &self.sampling_pose.orientation_constraint,
            &self.sampling_pose.position_constraint,
        ) {
            (Some(oc), _) => oc.link_name(),
            (None, Some(pc)) => pc.link_name(),
            (None, None) => {
                unreachable!("IkConstraintSampler::new requires at least one constraint")
            }
        }
    }

    /// `samplePose`: a candidate `(position, orientation)` in
    /// `reference`'s frame, or `None` if no constraint-satisfying position
    /// could be sampled within `max_attempts` (upstream's `false` return —
    /// this is the one failure this port surfaces as `Option::None` rather
    /// than propagating further, matching upstream's own "sampling ran out
    /// of attempts" semantics, not an error).
    ///
    /// `reference` supplies frame transforms only (upstream's
    /// `const RobotState& ks`) — it is never written to. Upstream's own
    /// `dirtyLinkTransforms()` guard has no case left to check here: a
    /// [`Posed`] proves its transforms are current by construction (see
    /// that type's own doc comment), unlike upstream's `RobotState`, which
    /// can be dirty at any call site.
    pub fn sample_pose(
        &self,
        reference: &Posed<'_, '_>,
        mut rng: &mut dyn Rng,
        max_attempts: u32,
    ) -> Option<(Vector3, UnitQuaternion)> {
        let mut pos = if let Some(pc) = &self.sampling_pose.position_constraint {
            let regions = pc.constraint_regions();
            let k = rng.random_range(0..regions.len());
            let mut sampled = None;
            for i in 0..regions.len() {
                let region = &regions[(i + k) % regions.len()];
                let body = region.body.clone_at(region.pose);
                if let Some(pt) =
                    body.sample_point_inside(max_attempts, &mut |lo, hi| rng.random_range(lo..hi))
                {
                    sampled = Some(pt);
                    break;
                }
            }
            let pt = sampled?;
            if pc.mobile_reference_frame() {
                let frame_tf = reference
                    .frame_transform(pc.reference_frame())
                    .expect("mobile reference frame was validated resolvable at construction");
                (frame_tf * nalgebra::Point3::from(pt)).coords
            } else {
                pt
            }
        } else {
            // No position constraint: FK a randomized state and read off
            // the orientation constraint's link position (upstream: `temp_state.setToRandomPositions(jmg_)`,
            // whole-model scoped here rather than group-scoped for the
            // same reason `RobotState::set_to_random_positions_with` already
            // is — see that method's own doc comment).
            let oc = self.sampling_pose.orientation_constraint.as_ref().expect(
                "IkConstraintSampler::new requires at least one of position or orientation",
            );
            let mut scratch = RobotState::new(reference.model());
            scratch.set_variable_positions(reference.positions());
            scratch.set_to_random_positions_with(&mut rng);
            let scratch_posed = scratch.update();
            scratch_posed
                .global_link_transform(oc.link_name())
                .expect("link name was resolved against the model at construction")
                .translation
                .vector
        };

        let mut quat = if let Some(oc) = &self.sampling_pose.orientation_constraint {
            let eps = f64::EPSILON;
            let (x_tol, y_tol, z_tol) = match oc.tolerance() {
                OrientationTolerance::XyzEuler { x, y, z } => (x, y, z),
                OrientationTolerance::RotationVector { x, y, z } => (x, y, z),
            };
            let angle_x = 2.0 * (rng.random::<f64>() - 0.5) * (x_tol - eps);
            let angle_y = 2.0 * (rng.random::<f64>() - 0.5) * (y_tol - eps);
            let angle_z = 2.0 * (rng.random::<f64>() - 0.5) * (z_tol - eps);

            let diff = match oc.tolerance() {
                OrientationTolerance::XyzEuler { .. } => {
                    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle_x)
                        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle_y)
                        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle_z)
                }
                OrientationTolerance::RotationVector { .. } => {
                    // Convert the rotation-vector delta from the
                    // constraint's own header frame into the target frame,
                    // matching upstream's comment on this exact line.
                    let rotation_vector = oc.desired_rotation_matrix_in_ref_frame().transpose()
                        * Vector3::new(angle_x, angle_y, angle_z);
                    UnitQuaternion::from_axis_angle(
                        &nalgebra::Unit::new_normalize(rotation_vector),
                        rotation_vector.norm(),
                    )
                }
            };

            UnitQuaternion::from_rotation_matrix(&oc.desired_rotation_matrix()) * diff
        } else {
            let (x, y, z, w) = sample_unit_quaternion(rng);
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z))
        };

        if let Some(oc) = &self.sampling_pose.orientation_constraint {
            if oc.mobile_reference_frame() {
                let frame_rot = reference
                    .frame_transform(oc.reference_frame())
                    .expect("mobile reference frame was validated resolvable at construction")
                    .rotation;
                quat = frame_rot * quat;
            }
        }

        if let Some(pc) = &self.sampling_pose.position_constraint {
            if pc.has_link_offset() {
                pos -= quat * pc.link_offset();
            }
        }

        Some((pos, quat))
    }

    /// `sample` (upstream also names this `sampleHelper` — a one-line
    /// wrapper upstream, folded together here since nothing else calls
    /// `sampleHelper` directly).
    ///
    /// `state` supplies the IK seed on the first attempt and receives the
    /// solution on success; `solver` performs the actual IK search.
    /// Upstream's `sample(state, reference_state, max_attempts)` keeps
    /// `reference_state` as a second, separate, never-written parameter for
    /// pose sampling's frame lookups. This port has no such second
    /// parameter (matching the same collapse [`crate::ConstraintSampler::sample`]
    /// already made) — instead `state` is copied once, up front, into an
    /// owned reference before any IK attempt can write into `state` itself,
    /// giving `sample_pose` the same "fixed throughout the loop" reference
    /// upstream's separate parameter does.
    ///
    /// No bijection array: upstream indexes `seed`/`ik_sol` by position
    /// through `jmg_->getKinematicsSolverJointBijection()`, a
    /// separately-maintained `Vec<size_t>` whose only job is keeping two
    /// parallel index spaces in agreement. This port reads and writes each
    /// entry by name instead, via [`KinematicsSolver::joint_names`] and
    /// [`RobotState::set_variable_positions_named`] — the ordering the
    /// bijection exists to keep honest is enforced by the name lookup
    /// itself rather than by two arrays staying in sync.
    ///
    /// Matches upstream's one further wart: on a converged-but-invalid IK
    /// solution (fails `validate`), `state` is left
    /// holding that solution rather than being reverted before the next
    /// attempt — upstream's `callIK` writes via `setJointGroupPositions`
    /// *before* calling `validate()` and never undoes it either.
    pub fn sample(
        &self,
        state: &mut RobotState<'_>,
        solver: &mut dyn KinematicsSolver,
        mut rng: &mut dyn Rng,
        max_attempts: u32,
    ) -> bool {
        let mut reference = RobotState::new(state.model());
        reference.set_variable_positions(state.positions());
        let reference_posed = reference.update();

        for attempt in 0..max_attempts {
            let Some((mut point, mut quat)) = self.sample_pose(&reference_posed, rng, max_attempts)
            else {
                return false;
            };

            if self.transform_ik {
                let ik_frame_tf = reference_posed
                    .frame_transform(&self.ik_frame)
                    .expect("ik_frame was validated resolvable at construction");
                let world = Isometry3::from_parts(nalgebra::Translation3::from(point), quat);
                let local = ik_frame_tf.inverse() * world;
                point = local.translation.vector;
                quat = local.rotation;
            }

            let seed: Vec<f64> = if attempt == 0 {
                solver
                    .joint_names()
                    .iter()
                    .map(|name| {
                        state
                            .variable_position(name)
                            .expect("solver joint name must be a variable of this model")
                    })
                    .collect()
            } else {
                let mut scratch = RobotState::new(state.model());
                scratch.set_to_random_positions_with(&mut rng);
                solver
                    .joint_names()
                    .iter()
                    .map(|name| {
                        scratch
                            .variable_position(name)
                            .expect("solver joint name must be a variable of this model")
                    })
                    .collect()
            };

            let target = Isometry3::from_parts(nalgebra::Translation3::from(point), quat);
            let Some(solution) = solver.solve(&seed, &target) else {
                continue;
            };

            let names: Vec<&str> = solver.joint_names().iter().map(String::as_str).collect();
            state
                .set_variable_positions_named(&names, &solution)
                .expect("solver joint names are a subset of this model's variables");

            if self.validate(state) {
                return true;
            }
        }
        false
    }

    /// `validate`.
    fn validate(&self, state: &mut RobotState<'_>) -> bool {
        let posed = state.update();
        let orientation_ok = match &self.sampling_pose.orientation_constraint {
            Some(oc) => oc.decide(&posed).satisfied,
            None => true,
        };
        let position_ok = match &self.sampling_pose.position_constraint {
            Some(pc) => pc.decide(&posed).satisfied,
            None => true,
        };
        orientation_ok && position_ok
    }
}

/// A uniformly random unit quaternion as `(x, y, z, w)`, via Shoemake's
/// algorithm. Upstream's no-orientation-constraint fallback in `samplePose`
/// calls `random_numbers::RandomNumberGenerator::quaternion(q)`; that
/// package's implementation is genuinely absent from both this host (only
/// `moveit_core` is vendored locally) and the oracle container (its
/// `random_numbers.h` declares the method with no body, and no
/// `random_numbers.cpp` exists anywhere in the image either — confirmed by
/// searching both per this repo's mandatory "search the oracle before
/// concluding a reference is absent" rule). `moveit-state::RobotState`'s
/// private `sample_unit_quaternion` (used for floating-joint random
/// positions) already ports the identical upstream call the same way, for
/// the same reason; this is an independent transcription of the same
/// well-known algorithm rather than a reach into that crate's private
/// internals, matching that function's own doc comment: this produces a
/// uniform distribution over the unit sphere in `R^4`, which is what
/// upstream's own contract guarantees and what this crate's tests check —
/// not bit-exact agreement with the C++ RNG.
fn sample_unit_quaternion(rng: &mut dyn Rng) -> (f64, f64, f64, f64) {
    let u1: f64 = rng.random();
    let u2: f64 = rng.random();
    let u3: f64 = rng.random();
    let sqrt_1_u1 = (1.0 - u1).sqrt();
    let sqrt_u1 = u1.sqrt();
    let x = sqrt_1_u1 * (2.0 * PI * u2).sin();
    let y = sqrt_1_u1 * (2.0 * PI * u2).cos();
    let z = sqrt_u1 * (2.0 * PI * u3).sin();
    let w = sqrt_u1 * (2.0 * PI * u3).cos();
    (x, y, z, w)
}

/// Round 10's composition decision: wraps an [`IkConstraintSampler`] plus
/// the solver and retry budget it needs so the pair together implement
/// [`ConstraintSampler`] and can sit inside a [`crate::UnionConstraintSampler`].
///
/// # Why an adapter, not a wider trait
///
/// [`IkConstraintSampler`]'s own doc comment already ruled out widening
/// [`ConstraintSampler::sample`]'s signature to carry a solver: every other
/// implementer would grow a parameter it never reads. The remaining
/// question is where the solver instance itself lives. [`ConstraintSampler::sample`]
/// takes `&self`, not `&mut self` (upstream's own `sample` is `const`-ish in
/// spirit — the point of an immutable interface for a union of samplers to
/// call through in sequence), but [`KinematicsSolver::solve`] needs `&mut
/// dyn KinematicsSolver`. `Rc<RefCell<Box<dyn KinematicsSolver>>>` resolves
/// that the same way upstream's own `kb_` does: a `kinematics::KinematicsBaseConstPtr`
/// is a `shared_ptr`, one solver instance shared by every
/// `IKConstraintSampler` built against the same group (upstream's
/// `getSolverInstance()` returns the same pointer every time it is called on
/// one `JointModelGroup`) — `Rc::clone` is this port's equivalent of copying
/// that `shared_ptr`, and `RefCell` supplies the interior mutability `&self`
/// needs to still reach `solve`. Single-threaded by construction, matching
/// this crate's other D4 decisions: nothing here is `Send`.
pub struct IkConstraintSamplerAdapter {
    group: JointModelGroup,
    ik: IkConstraintSampler,
    solver: Rc<RefCell<Box<dyn KinematicsSolver>>>,
    /// Baked in at construction, not read per-call: `ConstraintSampler::sample`
    /// takes no `max_attempts` parameter (see `sampler.rs`'s own doc comment
    /// on why), so this is where upstream's `sample(..., max_attempts)`
    /// argument has to live instead.
    max_attempts: u32,
    /// `frame_depends_`, computed once here rather than recomputed on every
    /// [`ConstraintSampler::frame_dependency`] call, matching
    /// [`crate::UnionConstraintSampler`]'s own precomputed `frame_depends`.
    frame_depends: Vec<String>,
}

impl IkConstraintSamplerAdapter {
    /// Build an adapter around a fresh [`IkConstraintSampler`], taking
    /// shared ownership of `solver` (see this type's doc comment on why
    /// `Rc<RefCell<_>>`).
    ///
    /// # Errors
    ///
    /// Whatever [`IkConstraintSampler::new`] can return.
    pub fn new(
        model: &RobotModel,
        group: &JointModelGroup,
        solver: Rc<RefCell<Box<dyn KinematicsSolver>>>,
        sampling_pose: IkSamplingPose,
        max_attempts: u32,
    ) -> Result<Self> {
        let mut frame_depends = Vec::new();
        if let Some(pc) = &sampling_pose.position_constraint {
            if pc.mobile_reference_frame() {
                frame_depends.push(pc.reference_frame().to_string());
            }
        }
        if let Some(oc) = &sampling_pose.orientation_constraint {
            if oc.mobile_reference_frame() {
                frame_depends.push(oc.reference_frame().to_string());
            }
        }

        let ik = {
            let solver_ref = solver.borrow();
            IkConstraintSampler::new(model, &**solver_ref, sampling_pose)?
        };

        Ok(Self {
            group: group.clone(),
            ik,
            solver,
            max_attempts,
            frame_depends,
        })
    }

    /// `getSamplingVolume`, forwarded from the wrapped [`IkConstraintSampler`]
    /// — the sampling-volume tie-break `crate::constraint_sampler_manager`
    /// needs is not part of the [`ConstraintSampler`] trait itself (no other
    /// implementer has a meaningful volume to compare).
    pub fn sampling_volume(&self) -> f64 {
        self.ik.sampling_volume()
    }
}

impl ConstraintSampler for IkConstraintSamplerAdapter {
    fn joint_model_group(&self) -> &JointModelGroup {
        &self.group
    }

    fn frame_dependency(&self) -> &[String] {
        &self.frame_depends
    }

    fn sample(&self, state: &mut RobotState<'_>, rng: &mut dyn Rng) -> bool {
        let mut solver_ref = self.solver.borrow_mut();
        self.ik
            .sample(state, &mut **solver_ref, rng, self.max_attempts)
    }
}
