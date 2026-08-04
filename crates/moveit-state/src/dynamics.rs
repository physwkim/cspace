// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2009, Ruben Smits
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/dynamics_solver/include/moveit/dynamics_solver/dynamics_solver.hpp
//   moveit_core/dynamics_solver/src/dynamics_solver.cpp
// and orocos_kinematics_dynamics @ v1.5.1:
//   orocos_kdl/src/chainidsolver_recursive_newton_euler.{hpp,cpp}
//   orocos_kdl/src/{frames.hpp,frames.inl,rigidbodyinertia.{hpp,cpp},
//                   rotationalinertia.hpp,segment.{hpp,cpp},joint.{hpp,cpp},
//                   chain.{hpp,cpp}}
//
// Every operator below was diffed against the headers the oracle's own
// image ships under `/usr/include/kdl` before being trusted (`.cpp` files
// have no image-side counterpart — KDL ships there as a compiled `.so` —
// so those were read from the pinned `v1.5.1` checkout directly).

//! Inverse dynamics (`DynamicsSolver`) for moveit-rs: joint torques from a
//! chain's inertial properties, velocity and acceleration, via the
//! Recursive Newton-Euler algorithm (`KDL::ChainIdSolver_RNE::CartToJnt`).
//!
//! # Deviation from upstream: no `kdl_parser`, no `RobotState`
//!
//! Upstream builds a `KDL::Chain` from the raw URDF via `kdl_parser` and
//! walks it independently of `moveit::core::RobotModel`. `kdl_parser` has
//! no available source anywhere local to this repo (host or oracle image;
//! only a compiled `.so` and header declarations) to verify its exact
//! `Joint`/`Segment::f_tip` split against, so this port does not attempt to
//! reproduce that split. Instead it uses two facts that hold regardless of
//! that split:
//!
//! - **`X[i]` (`Segment::pose(q)`) is exactly [`LinkModel::joint_origin_transform`]
//!   composed with [`JointModel::compute_transform`].** Both describe the
//!   same physical quantity — link i's pose relative to link i-1 as a
//!   function of the joint variable — and there is only one physically
//!   correct answer for a URDF revolute/prismatic joint. This port's own
//!   `X[i]` is already the formula [`crate::state`]'s forward kinematics
//!   uses and has verified against the oracle's `fk` op.
//! - **`S[i]` (the joint's unit motion subspace, expressed in link i's own
//!   final frame) is exactly the joint's local axis** (`rot` for revolute,
//!   `vel` for prismatic), with no reference-point offset. URDF's own
//!   convention places a joint's rotation/translation axis through the
//!   child frame's own origin, so the instantaneous unit twist about that
//!   axis, expressed at link i's own origin in link i's own frame, has no
//!   linear (for revolute) or angular (for prismatic) component to begin
//!   with — matching `ChainIdSolver_RNE`'s own comment that `S[i]` is time
//!   constant. This is the same axis-invariance fact
//!   [`crate::state`]'s Jacobian relies on.
//!
//! Together these mean the sweep below needs only a [`RobotModel`] handle
//! (no `RobotState`, no FK query) — including for `getMaxPayload`'s and
//! `getPayloadTorques`'s base→tip wrench transform, which is exactly the
//! inverse of the forward sweep's own accumulated `X[0]*X[1]*...*X[ns-1]`
//! product, a world-frame-independent identity.
//!
//! # Deviation from upstream: mass/inertia via `LinkModel`, not `kdl_parser`
//!
//! Upstream's `DynamicsSolver` reads mass/inertia from the raw URDF a
//! second time via `kdl_parser`, bypassing `RobotModel`/`LinkModel`
//! entirely (`moveit::core::LinkModel` has no such field). This port reads
//! [`LinkModel::mass`]/[`LinkModel::center_of_mass`]/[`LinkModel::inertia`]
//! instead — see that type's doc comment, deviation 5.
//!
//! # Deviation from upstream: `max_torques` is an explicit parameter
//!
//! Upstream's constructor reads `urdf_model->getJoint(name)->limits->effort`
//! directly from the raw URDF for every joint in the group (fixed
//! included), bypassing `RobotModel` exactly as it does for mass/inertia —
//! `moveit::core::VariableBounds` carries no effort field either, and this
//! port's own [`moveit_model::joint::VariableBounds`] deliberately doesn't
//! add one just for this (see that type's module for the parity
//! reasoning). [`DynamicsSolver::new`] therefore takes `max_torques` as an
//! explicit `Vec<f64>`, indexed exactly like upstream's `max_torques_`:
//! one entry per [`JointModelGroup::joint_indices`] entry (fixed joints
//! included, effort `0.0`), not per active joint. A caller with a raw URDF
//! (as this crate's own tests do, via the `urdf-rs` dev-dependency) builds
//! this the same way upstream's constructor loop does; production
//! `moveit-state` code carries no `urdf-rs` dependency for it.
//!
//! # Deviation from upstream: `getMaxPayload`'s indexing bug is replicated
//!
//! Upstream's `getMaxPayload` saturation check reads `max_torques_[i]` for
//! `i` in the *active*-joint index space, even though `max_torques_` was
//! built over the *full* (fixed-joint-inclusive) space — for a chain with
//! a fixed joint before its last active joint, this compares a real joint's
//! torque against a different (often always-`0.0`) joint's limit. See
//! `capture-dynamics-fixtures.py`'s module doc comment and
//! `oracle.cpp`'s `dynamics()` doc comment for the confirmed mechanism on
//! `pr2`'s `right_arm` group. This port's [`DynamicsSolver::max_payload`]
//! reproduces the bug rather than fixing it: the only ground truth
//! available to verify against (`pr2_dynamics.json`, captured from the real
//! oracle) reflects the buggy behavior, and there is no ground truth to
//! verify a "fixed" version against. A caller building `max_torques` from
//! [`JointModelGroup::active_joint_indices`] instead of
//! [`JointModelGroup::joint_indices`] sidesteps the bug entirely (no fixed
//! joints in the index space to misalign against); this port does not
//! force that choice on a caller, matching upstream's own constructor
//! signature exactly.
//!
//! # Omitted: the joint's own scalar reflected inertia
//!
//! `CartToJnt`'s backward sweep adds
//! `chain.getSegment(i).getJoint().getInertia()*q_dotdot(j)` to each
//! torque — a scalar "rotor"/reflected inertia carried on `KDL::Joint`
//! itself, distinct from the link's [`RigidBodyInertia`] above. URDF has no
//! such concept (only `<link><inertial>`), and every `KDL::Joint`
//! constructor takes this value as a required, explicitly-passed
//! parameter with no data source in a URDF to read it from — so any
//! URDF-driven `kdl_parser` construction leaves it at `0.0`. This term is
//! therefore always zero for every fixture this port has ground truth for,
//! and is omitted rather than coded as a permanent no-op.

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Vector3};
use moveit_model::joint::JointKind;
use moveit_model::{JointModelGroup, RobotModel};
use nalgebra::Matrix3;

/// A spatial velocity: linear (`vel`) and angular (`rot`) parts, both
/// expressed about the same reference point and in the same frame. `KDL::Twist`.
#[derive(Debug, Clone, Copy)]
struct Twist {
    vel: Vector3,
    rot: Vector3,
}

impl Twist {
    fn zero() -> Self {
        Twist {
            vel: Vector3::zeros(),
            rot: Vector3::zeros(),
        }
    }

    fn scale(self, s: f64) -> Self {
        Twist {
            vel: self.vel * s,
            rot: self.rot * s,
        }
    }

    fn add(self, other: Self) -> Self {
        Twist {
            vel: self.vel + other.vel,
            rot: self.rot + other.rot,
        }
    }
}

/// A spatial force: linear force and torque, both expressed about the same
/// reference point and in the same frame. `KDL::Wrench`.
#[derive(Debug, Clone, Copy)]
struct Wrench {
    force: Vector3,
    torque: Vector3,
}

impl Wrench {
    fn zero() -> Self {
        Wrench {
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
        }
    }

    fn add(self, other: Self) -> Self {
        Wrench {
            force: self.force + other.force,
            torque: self.torque + other.torque,
        }
    }

    fn sub(self, other: Self) -> Self {
        Wrench {
            force: self.force - other.force,
            torque: self.torque - other.torque,
        }
    }
}

/// A rigid body's mass distribution, about its own segment origin.
/// `KDL::RigidBodyInertia`.
#[derive(Debug, Clone, Copy)]
struct RigidBodyInertia {
    mass: f64,
    /// First mass moment about the segment origin (`m*center_of_mass`).
    h: Vector3,
    /// Rotational inertia about the segment origin (already parallel-axis
    /// shifted from the link's own center-of-mass-relative tensor).
    inertia: Matrix3<f64>,
}

/// The rotational inertia of `link` about its own segment origin, built
/// from its center-of-mass-relative inertia tensor and mass distribution.
///
/// # Not transcribed from `RigidBodyInertia`'s two-argument-plus-tensor constructor
///
/// The Huygens-Steiner (parallel-axis) theorem for a rigid body's
/// rotational inertia tensor: about a reference point `O`, `I_O` is
/// defined as the integral over the body of `(|r|^2 * Id - r*r^T) dm`,
/// where `r` is each mass element's position relative to `O`. Splitting
/// `r = r' + c` (`r'` relative to the center of mass, `c` the center of
/// mass's own position relative to `O`, both fixed vectors pulled out of
/// the integral) expands the square and cross terms; every term linear in
/// `r'` vanishes under integration by the defining property of the center
/// of mass (`integral of r' dm = 0` — that is what "center of mass" means),
/// leaving `I_O = I_cm + m*(|c|^2*Id - c*c^T)`, the standard tensor form of
/// the theorem. `link.inertia()` is already `I_cm` (about the link's own
/// center of mass — URDF's `<inertial><inertia>` convention), so
/// `ic - m*(c*c^T - c.dot(c)*Id)` is exactly `I_cm - m*(c*c^T - |c|^2*Id)`,
/// the same expression with the subtraction sign carrying the `+m*(|c|^2*Id
/// - ...)` term instead of a separate addition.
fn rigid_body_inertia_from_link(link: &moveit_model::LinkModel) -> RigidBodyInertia {
    let m = link.mass();
    let c = link.center_of_mass();
    let ic = link.inertia();
    let inertia = ic - m * (c * c.transpose() - c.dot(&c) * Matrix3::identity());
    RigidBodyInertia {
        mass: m,
        h: m * c,
        inertia,
    }
}

/// The spatial velocity `t`, referred to the reference frame's origin and
/// expressed in the reference frame's basis, re-expressed about the local
/// origin of the frame `x` places relative to that reference, in the
/// local frame's own basis — `x`'s inverse.
///
/// # Not transcribed from `Frame::Inverse(const Twist&)`
///
/// The rigid-body velocity transfer formula: a twist `(v, w)` referred to
/// a point `P` describes a rigid body's spatial velocity field, so the
/// velocity of any other body-fixed point `Q` is `v + w x (Q - P)` — this
/// is the definition of angular velocity (elementary rigid-body
/// kinematics, e.g. any statics/dynamics text's rigid-body velocity
/// field). Take `P` as the reference origin (where `t` is referred) and
/// `Q` as the local origin: by `x`'s own definition (a point with local
/// coordinates `q` has reference-frame coordinates
/// `x.translation + x.rotation*q`), `Q` expressed in the reference frame
/// is exactly `x.translation`. Substituting gives the local origin's
/// velocity, still in reference-frame components, as
/// `t.vel + t.rot x x.translation`. Rotating that — and `t.rot` itself,
/// re-expressed but otherwise unchanged — into the local frame's own
/// basis via `x.rotation^-1` (the inverse of the rotation that maps local
/// components to reference components) produces this function's
/// `vel`/`rot`. `t.rot x x.translation` and `-(x.translation x t.rot)`
/// are the same vector (cross-product antisymmetry), which is why the
/// code reads as a subtraction.
fn frame_inverse_twist(x: &Isometry3, t: Twist) -> Twist {
    let r_inv = x.rotation.inverse();
    Twist {
        rot: r_inv * t.rot,
        vel: r_inv * (t.vel - x.translation.vector.cross(&t.rot)),
    }
}

/// The spatial force `w`, referred to the local origin of the frame `x`
/// places relative to a reference frame and expressed in that local
/// frame's basis, re-expressed about the reference origin, in the
/// reference frame's own basis — the forward direction of `x`, dual to
/// `frame_inverse_twist`'s inverse direction.
///
/// # Not transcribed from `Frame::operator*(const Wrench&)`
///
/// The moment transfer theorem from statics: a force is a free vector, so
/// re-expressing it in the reference frame is a plain rotation
/// (`force = R*w.force`, `R` the rotation that maps local components to
/// reference components — `x.rotation`'s own definition). Moving a
/// wrench's point of application from the local origin to the reference
/// origin leaves the force unchanged but adds `offset x force` to the
/// torque (moving where a force is applied changes the moment it produces
/// about a fixed point by exactly that cross product — the textbook
/// moment-transfer/Varignon rule), where `offset` is the local origin's
/// position expressed in the reference frame — `x.translation` itself, by
/// `x`'s own definition (see `frame_inverse_twist`'s doc comment).
/// Substituting gives `torque = R*w.torque + x.translation x (R*w.force)`
/// — the already-rotated force, so the offset multiplies the force as
/// seen in the reference frame, not the local one.
fn frame_mul_wrench(x: &Isometry3, w: Wrench) -> Wrench {
    let force = x.rotation * w.force;
    let torque = x.rotation * w.torque + x.translation.vector.cross(&force);
    Wrench { force, torque }
}

/// The spatial (motion) cross product of two twists: the rate of change of
/// `rhs` as seen by an observer moving with `lhs`.
///
/// # Not transcribed from `operator*(const Twist&, const Twist&)`
///
/// The transport theorem: for a vector quantity `u` expressed in a frame
/// rotating at angular velocity `w`, its rate of change in a fixed frame
/// is `du/dt (in the rotating frame) + w x u` (the standard rule for
/// differentiating a quantity carried by a rotating frame — the same rule
/// that produces Coriolis/centripetal terms anywhere in rigid-body
/// mechanics). Applying it to both the linear and angular parts of a
/// twist `rhs` as seen from a frame instantaneously moving with `lhs`
/// gives the rotational part `lhs.rot x rhs.rot` (angular velocities
/// compose the same way any rotating vector does) and, for the linear
/// part, two contributions by the product rule — `lhs.rot`'s rotation
/// sweeping `rhs.vel` around (`lhs.rot x rhs.vel`), plus `lhs.vel`'s own
/// translation coupling into `rhs`'s rotational motion
/// (`lhs.vel x rhs.rot`) — summing to `lhs.rot x rhs.vel + lhs.vel x
/// rhs.rot`. This is why `sweep` needs it for its acceleration
/// recurrence: a joint's own velocity contribution is defined in that
/// joint's moving frame, so differentiating it a second time picks up
/// exactly this correction.
fn twist_cross(lhs: Twist, rhs: Twist) -> Twist {
    Twist {
        vel: lhs.rot.cross(&rhs.vel) + lhs.vel.cross(&rhs.rot),
        rot: lhs.rot.cross(&rhs.rot),
    }
}

/// The spatial (force) cross product: the unique bilinear map dual to
/// `twist_cross` under the twist-wrench power pairing (`dot_twist_wrench`).
///
/// # Not transcribed from `operator*(const Twist&, const Wrench&)`
///
/// Rather than an independently-guessed formula, this is *defined* by the
/// identity `dot_twist_wrench(twist_cross(lhs, w), rhs) ==
/// -dot_twist_wrench(w, twist_cross_wrench(lhs, rhs))` holding for every
/// twist `w` — the standard construction of a dual/adjoint operator from a
/// bilinear pairing (here, mechanical power), used throughout spatial
/// vector algebra so that a velocity-dependent force term and a
/// velocity-dependent acceleration term stay power-consistent with each
/// other. Expanding both sides with the scalar triple product identity
/// `a.(b x c) == (a x b).c` and collecting the coefficients of `w.vel` and
/// `w.rot` pins down `force = lhs.rot x rhs.force` and
/// `torque = lhs.rot x rhs.torque + lhs.vel x rhs.force` as the only
/// solution — this file's `dot_twist_wrench` and `twist_cross` together
/// already fix every term in that identity, so the result isn't a
/// separate assumption.
fn twist_cross_wrench(lhs: Twist, rhs: Wrench) -> Wrench {
    Wrench {
        force: lhs.rot.cross(&rhs.force),
        torque: lhs.rot.cross(&rhs.torque) + lhs.vel.cross(&rhs.force),
    }
}

/// The mechanical power a wrench `w` delivers through a twist `t`:
/// force-dot-linear-velocity plus torque-dot-angular-velocity — the
/// standard definition of power (equivalently, virtual work per unit
/// time) for a wrench acting through a rigid body's motion.
fn dot_twist_wrench(t: Twist, w: Wrench) -> f64 {
    t.vel.dot(&w.force) + t.rot.dot(&w.torque)
}

/// The spatial momentum (linear force part, angular torque part) of a
/// rigid body with inertia `inertia`, moving with spatial velocity `t`
/// referred to `inertia`'s own reference point.
///
/// # Not transcribed from `operator*(const RigidBodyInertia&, const Twist&)`
///
/// Linear momentum is mass times the center of mass's own velocity: by
/// the same rigid-body velocity field used to derive `frame_inverse_twist`
/// (`v_cm = t.vel + t.rot x c`), `p = m*(t.vel + t.rot x c) = m*t.vel -
/// (m*c) x t.rot = m*t.vel - inertia.h x t.rot`. Angular momentum about
/// the reference point is, by definition, `L = integral of r x v_r dm`
/// over the body (`r` each mass element's position relative to the
/// reference point, `v_r` its velocity there); substituting the same
/// rigid-body velocity field `v_r = t.vel + t.rot x r` and splitting the
/// integral gives `L = (integral of r dm) x t.vel + (integral of r x (t.rot
/// x r) dm) = (m*c) x t.vel + I*t.rot` (the first integral is `m` times
/// the center of mass by definition; the second is exactly
/// `rigid_body_inertia_from_link`'s own inertia tensor acting on `t.rot`,
/// by that function's own derivation) — `(m*c) x t.vel` is `inertia.h x
/// t.vel` directly (`inertia.h` is already `m*c`), so `L` is exactly
/// `inertia.inertia*t.rot + inertia.h.cross(t.vel)`, this function's
/// `torque`.
fn inertia_mul_twist(inertia: &RigidBodyInertia, t: Twist) -> Wrench {
    Wrench {
        force: inertia.mass * t.vel - inertia.h.cross(&t.rot),
        torque: inertia.inertia * t.rot + inertia.h.cross(&t.vel),
    }
}

/// The result of [`DynamicsSolver::max_payload`]: the heaviest tip payload
/// (kilograms) this configuration can carry before some joint's torque
/// limit is exceeded, and which joint (by *active*-joint index, matching
/// the index space of [`DynamicsSolver::torques`]'s input/output) is the
/// limiting one. `DynamicsSolver::getMaxPayload`'s `double& payload,
/// unsigned int& joint_saturated` out-parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaxPayload {
    /// The maximum additional tip payload mass, in kilograms.
    pub payload: f64,
    /// The active-joint index whose torque limit bounds `payload`.
    pub joint_saturated: usize,
}

/// Inverse dynamics for one chain group. `moveit::core::DynamicsSolver`.
///
/// Built once per `(group, gravity, max_torques)` and reused across calls,
/// same as upstream (`kdl_chain_`/`max_torques_` are constructed once in
/// upstream's own constructor too).
pub struct DynamicsSolver<'m> {
    model: &'m RobotModel,
    group: &'m JointModelGroup,
    gravity: Vector3,
    gravity_norm: f64,
    max_torques: Vec<f64>,
}

impl<'m> DynamicsSolver<'m> {
    /// Build a solver for `group_name` in `model`. `max_torques` must have
    /// one entry per [`JointModelGroup::joint_indices`] entry (fixed joints
    /// included) — see this module's doc comment for why this is an
    /// explicit parameter rather than something read from `model`.
    ///
    /// Errors (matching upstream's own constructor preconditions, checked
    /// via `RCLCPP_ERROR`+leaving the solver default-constructed/unusable
    /// upstream, returned here instead): `group_name` does not name a
    /// group; the group is not a chain; the group has a mimic joint; the
    /// group's root link has no parent (i.e. is the model's root link);
    /// `max_torques.len()` does not match the group's joint count.
    pub fn new(
        model: &'m RobotModel,
        group_name: &str,
        gravity: Vector3,
        max_torques: Vec<f64>,
    ) -> Result<Self> {
        let group = model.joint_model_group(group_name)?;
        if !group.is_chain() {
            return Err(Error::construct(format!(
                "group {group_name:?} is not a chain"
            )));
        }
        if !group.mimic_joint_indices().is_empty() {
            return Err(Error::construct(format!(
                "group {group_name:?} has a mimic joint, which DynamicsSolver does not support"
            )));
        }
        let root_link_index = *group
            .link_indices()
            .first()
            .ok_or_else(|| Error::construct(format!("group {group_name:?} has no links")))?;
        if model
            .link_model_at(root_link_index)
            .parent_link_index()
            .is_none()
        {
            return Err(Error::construct(format!(
                "group {group_name:?}'s root link has no parent link"
            )));
        }
        if max_torques.len() != group.joint_indices().len() {
            return Err(Error::construct(format!(
                "max_torques has {} entries, group {group_name:?} has {}",
                max_torques.len(),
                group.joint_indices().len()
            )));
        }
        Ok(Self {
            model,
            group,
            gravity,
            gravity_norm: gravity.norm(),
            max_torques,
        })
    }

    /// `max_torques_`, verbatim: one entry per group joint including fixed
    /// joints, in [`JointModelGroup::joint_indices`] order.
    pub fn max_torques(&self) -> &[f64] {
        &self.max_torques
    }

    fn num_active(&self) -> usize {
        self.group.active_joint_indices().len()
    }

    fn num_segments(&self) -> usize {
        self.group.joint_indices().len()
    }

    /// `getTorques`: joint torques for this configuration, with no external
    /// wrench applied at the tip. `angles`/`velocities`/`accelerations` are
    /// indexed by active joint, matching [`JointModelGroup::active_joint_indices`].
    pub fn torques(
        &self,
        angles: &[f64],
        velocities: &[f64],
        accelerations: &[f64],
    ) -> Result<Vec<f64>> {
        let zero_wrenches = vec![Wrench::zero(); self.num_segments()];
        let (torques, _base_to_tip) =
            self.sweep(angles, velocities, accelerations, &zero_wrenches)?;
        Ok(torques)
    }

    /// `getMaxPayload`: the heaviest tip payload this configuration can
    /// carry (aligned with gravity, applied at the last segment's tip)
    /// before some joint's torque limit is exceeded. See this module's doc
    /// comment for the upstream indexing bug this reproduces.
    pub fn max_payload(&self, angles: &[f64]) -> Result<MaxPayload> {
        let num_active = self.num_active();
        let ns = self.num_segments();
        let zeros = vec![0.0; num_active];
        let zero_wrenches = vec![Wrench::zero(); ns];
        let (zero_torques, base_to_tip) = self.sweep(angles, &zeros, &zeros, &zero_wrenches)?;

        for (i, (&zero_torque, &max_torque)) in
            zero_torques.iter().zip(self.max_torques.iter()).enumerate()
        {
            if zero_torque.abs() >= max_torque {
                return Ok(MaxPayload {
                    payload: 0.0,
                    joint_saturated: i,
                });
            }
        }

        let rotation = base_to_tip.inverse().rotation;
        let mut wrenches = vec![Wrench::zero(); ns];
        wrenches[ns - 1] = Wrench {
            force: rotation * Vector3::new(0.0, 0.0, 1.0),
            torque: Vector3::zeros(),
        };
        let (probe_torques, _) = self.sweep(angles, &zeros, &zeros, &wrenches)?;

        let mut min_payload = f64::INFINITY;
        let mut joint_saturated = 0usize;
        for (i, ((&zero_torque, &probe_torque), &max_torque)) in zero_torques
            .iter()
            .zip(probe_torques.iter())
            .zip(self.max_torques.iter())
            .enumerate()
        {
            let delta = probe_torque - zero_torque;
            let candidate =
                ((max_torque - zero_torque) / delta).max((-max_torque - zero_torque) / delta);
            if candidate < min_payload {
                min_payload = candidate;
                joint_saturated = i;
            }
        }
        Ok(MaxPayload {
            payload: min_payload / self.gravity_norm,
            joint_saturated,
        })
    }

    /// `getPayloadTorques`: joint torques with `payload` kilograms applied
    /// at the last segment's tip, aligned with gravity.
    pub fn payload_torques(&self, angles: &[f64], payload: f64) -> Result<Vec<f64>> {
        let num_active = self.num_active();
        let ns = self.num_segments();
        let zeros = vec![0.0; num_active];
        let (_, base_to_tip) = self.sweep(angles, &zeros, &zeros, &vec![Wrench::zero(); ns])?;

        let rotation = base_to_tip.inverse().rotation;
        let mut wrenches = vec![Wrench::zero(); ns];
        wrenches[ns - 1] = Wrench {
            force: rotation * Vector3::new(0.0, 0.0, payload * self.gravity_norm),
            torque: Vector3::zeros(),
        };
        let (torques, _) = self.sweep(angles, &zeros, &zeros, &wrenches)?;
        Ok(torques)
    }

    /// `CartToJnt`: the two-sweep Recursive Newton-Euler algorithm. Returns
    /// per-active-joint torques and the accumulated base-to-tip transform
    /// (`X[0]*X[1]*...*X[ns-1]`), which `max_payload`/`payload_torques` also
    /// need and would otherwise have to recompute in a second pass.
    fn sweep(
        &self,
        angles: &[f64],
        velocities: &[f64],
        accelerations: &[f64],
        external_wrenches: &[Wrench],
    ) -> Result<(Vec<f64>, Isometry3)> {
        let num_active = self.num_active();
        if angles.len() != num_active
            || velocities.len() != num_active
            || accelerations.len() != num_active
        {
            return Err(Error::other(format!(
                "expected {num_active} active joint values for group {:?}, got {}/{}/{}",
                self.group.name(),
                angles.len(),
                velocities.len(),
                accelerations.len()
            )));
        }
        let joint_indices = self.group.joint_indices();
        let link_indices = self.group.link_indices();
        let ns = joint_indices.len();
        if external_wrenches.len() != ns {
            return Err(Error::other(format!(
                "expected {ns} external wrenches for group {:?}, got {}",
                self.group.name(),
                external_wrenches.len()
            )));
        }

        // Gravity folded into a virtual base acceleration: a rigid body
        // sitting in a uniform gravitational field g experiences the same
        // relative dynamics as one with no gravity but whose base is
        // instead given a rigid acceleration of -g (d'Alembert's
        // principle -- a uniform field is indistinguishable, force-wise,
        // from an accelerating, non-inertial reference frame). Injecting
        // this as the root link's own acceleration in the forward sweep
        // below (its `k == 0` special case) makes every downstream link's
        // ordinary inertial coupling automatically produce the correct
        // `-m*g` gravitational force, without a separate gravity term
        // anywhere else in this function.
        let ag = Twist {
            vel: -self.gravity,
            rot: Vector3::zeros(),
        };

        let mut next_active = 0usize;
        let mut x = Vec::with_capacity(ns);
        let mut s = Vec::with_capacity(ns);
        let mut f = Vec::with_capacity(ns);
        let mut base_to_tip = Isometry3::identity();
        let mut prev_v = Twist::zero();
        let mut prev_a = Twist::zero();

        for k in 0..ns {
            let joint = self.model.joint_model_at(joint_indices[k]);
            let link = self.model.link_model_at(link_indices[k]);

            let sk = match joint.kind() {
                JointKind::Fixed => Twist::zero(),
                JointKind::Revolute(r) => Twist {
                    rot: r.axis(),
                    vel: Vector3::zeros(),
                },
                JointKind::Prismatic(p) => Twist {
                    vel: p.axis(),
                    rot: Vector3::zeros(),
                },
                JointKind::Planar(_) | JointKind::Floating(_) => {
                    return Err(Error::other(format!(
                        "joint {:?} has type {}, which DynamicsSolver does not support",
                        joint.name(),
                        joint.type_name()
                    )));
                }
            };
            let is_active = joint.joint_type() != moveit_model::joint::JointType::Fixed;

            let q = if is_active { angles[next_active] } else { 0.0 };
            let (qdot, qddot) = if is_active {
                (velocities[next_active], accelerations[next_active])
            } else {
                (0.0, 0.0)
            };
            if is_active {
                next_active += 1;
            }

            let local_q = [q];
            let joint_transform = if is_active {
                joint.compute_transform(&local_q)
            } else {
                joint.compute_transform(&[])
            };
            let xk = *link.joint_origin_transform() * joint_transform;

            // Forward velocity/acceleration recursion across the chain:
            // each link's spatial velocity/acceleration is the parent's
            // own velocity/acceleration transported into this link's
            // frame (`frame_inverse_twist`, the rigid-body transfer
            // formula across a fixed offset) plus this joint's own local
            // contribution. `vj = S_k*qdot` is the velocity contributed
            // by this joint's own motion (this file's own `S[i]`/`vj`
            // derivation, module doc). For the root link (`k == 0`) there
            // is no parent to transport from; the velocity recursion
            // simply starts at `vj` (a base not itself moving relative to
            // the world), and the acceleration recursion starts from
            // gravity's virtual base acceleration `ag` instead.
            //
            // The acceleration recursion additionally needs
            // `twist_cross(v, vj)`: `vj` is defined in this joint's own
            // (generally rotating) frame, so differentiating it a second
            // time to get an acceleration contribution picks up exactly
            // the transport-theorem correction `twist_cross` computes
            // (see that function's own doc comment) -- the standard
            // Coriolis/centripetal term any serial-chain acceleration
            // recursion needs when a joint's velocity is expressed in its
            // own moving frame.
            let vj = sk.scale(qdot);
            let v = if k == 0 {
                vj
            } else {
                frame_inverse_twist(&xk, prev_v).add(vj)
            };
            let a = if k == 0 {
                frame_inverse_twist(&xk, ag)
                    .add(sk.scale(qddot))
                    .add(twist_cross(v, vj))
            } else {
                frame_inverse_twist(&xk, prev_a)
                    .add(sk.scale(qddot))
                    .add(twist_cross(v, vj))
            };

            // Newton-Euler's equation in spatial-vector form: net spatial
            // force = inertia*acceleration, plus a velocity-product
            // "gyroscopic" term, minus any externally applied force.
            // `twist_cross_wrench(v, inertia_mul_twist(&inertia, v))` is
            // the spatial generalization of the classical rigid-body
            // (Euler's) equation `torque = I*alpha + omega x (I*omega)`:
            // the inertia's own spatial momentum
            // `inertia_mul_twist(&inertia, v)` is being differentiated
            // while expressed in this link's own moving frame, so --
            // exactly as with the velocity recursion's `twist_cross` term
            // above -- its true rate of change picks up a transport-
            // theorem correction, which is what `twist_cross_wrench`
            // computes for a momentum quantity rather than a plain
            // vector.
            let inertia = rigid_body_inertia_from_link(link);
            let fk = inertia_mul_twist(&inertia, a)
                .add(twist_cross_wrench(v, inertia_mul_twist(&inertia, v)))
                .sub(external_wrenches[k]);

            base_to_tip *= xk;
            x.push(xk);
            s.push(sk);
            f.push(fk);
            prev_v = v;
            prev_a = a;
        }

        let mut torques = vec![0.0; num_active];
        let mut j = num_active;
        for k in (0..ns).rev() {
            let joint = self.model.joint_model_at(joint_indices[k]);
            if joint.joint_type() != moveit_model::joint::JointType::Fixed {
                j -= 1;
                // Generalized force by virtual work: a 1-DOF joint's
                // torque is exactly the component of the joint's net
                // spatial force doing work against that joint's one
                // allowed direction of motion -- the projection of
                // `f[k]` onto the joint's own motion subspace `s[k]`,
                // i.e. the power pairing `dot_twist_wrench` already
                // defines (standard generalized-force-by-virtual-work
                // reasoning, not specific to this algorithm).
                torques[j] = dot_twist_wrench(s[k], f[k]);
            }
            if k != 0 {
                // Force propagation up the chain: by Newton's third law,
                // the force a child link exerts back on its parent
                // through their shared joint equals -- transported into
                // the parent's own frame via `frame_mul_wrench` -- the
                // net spatial force the child itself needs to satisfy its
                // own Newton-Euler equation above. Summing every child's
                // back-reaction into the parent's own `f` is how a
                // backward sweep accumulates a chain's forces from the
                // tip toward the root, one rigid joint connection at a
                // time.
                f[k - 1] = f[k - 1].add(frame_mul_wrench(&x[k], f[k]));
            }
        }

        Ok((torques, base_to_tip))
    }
}
