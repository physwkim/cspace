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

/// `KDL::RigidBodyInertia(m, c, Ic)`: build about the segment origin from
/// mass `m`, center of mass `c` (relative to the segment origin) and
/// rotational inertia `ic` about `c` — `rigidbodyinertia.cpp`'s
/// two-argument-plus-tensor constructor: `I = Ic - m*(c*c^T - (c.c)*Id)`.
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

/// `Frame::Inverse(const Twist&)`: `rot=M.Inverse(arg.rot);
/// vel=M.Inverse(arg.vel-p*arg.rot)`.
fn frame_inverse_twist(x: &Isometry3, t: Twist) -> Twist {
    let r_inv = x.rotation.inverse();
    Twist {
        rot: r_inv * t.rot,
        vel: r_inv * (t.vel - x.translation.vector.cross(&t.rot)),
    }
}

/// `Frame::operator*(const Wrench&)`: `force=M*arg.force;
/// torque=M*arg.torque+p*force` (using the already-rotated force).
fn frame_mul_wrench(x: &Isometry3, w: Wrench) -> Wrench {
    let force = x.rotation * w.force;
    let torque = x.rotation * w.torque + x.translation.vector.cross(&force);
    Wrench { force, torque }
}

/// `operator*(const Twist&, const Twist&)`, the spatial (motion) cross
/// product: `rot=lhs.rot x rhs.rot; vel=lhs.rot x rhs.vel + lhs.vel x rhs.rot`.
fn twist_cross(lhs: Twist, rhs: Twist) -> Twist {
    Twist {
        vel: lhs.rot.cross(&rhs.vel) + lhs.vel.cross(&rhs.rot),
        rot: lhs.rot.cross(&rhs.rot),
    }
}

/// `operator*(const Twist&, const Wrench&)`, the spatial (force) cross
/// product: `force=lhs.rot x rhs.force;
/// torque=lhs.rot x rhs.torque + lhs.vel x rhs.force`.
fn twist_cross_wrench(lhs: Twist, rhs: Wrench) -> Wrench {
    Wrench {
        force: lhs.rot.cross(&rhs.force),
        torque: lhs.rot.cross(&rhs.torque) + lhs.vel.cross(&rhs.force),
    }
}

/// `dot(const Twist&, const Wrench&)`: `dot(vel,force)+dot(rot,torque)`.
fn dot_twist_wrench(t: Twist, w: Wrench) -> f64 {
    t.vel.dot(&w.force) + t.rot.dot(&w.torque)
}

/// `operator*(const RigidBodyInertia&, const Twist&)`:
/// `force=m*t.vel-h x t.rot; torque=I*t.rot+h x t.vel`.
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

        // ag = -Twist(gravity, 0): ChainIdSolver_RNE's constructor.
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
                torques[j] = dot_twist_wrench(s[k], f[k]);
            }
            if k != 0 {
                f[k - 1] = f[k - 1].add(frame_mul_wrench(&x[k], f[k]));
            }
        }

        Ok((torques, base_to_tip))
    }
}
