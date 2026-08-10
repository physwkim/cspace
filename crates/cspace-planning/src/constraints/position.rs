// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class PositionConstraint)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (PositionConstraint::configure, PositionConstraint::decide)

use cspace_core::error::{Error, Result};
use cspace_core::geometry::bodies::Body;
use cspace_core::geometry::{Isometry3, Shape, Transforms, Vector3};
use cspace_core::model::RobotModel;
use cspace_core::state::Posed;

use crate::constraints::ConstraintEvaluationResult;

const EPS: f64 = f64::EPSILON;

/// One region of a [`PositionConstraint`]'s allowed volume: a body shape at
/// a pose.
///
/// # Deviation from upstream: one `Vec` of a sum type, not three parallel
/// arrays
///
/// `moveit_msgs::msg::PositionConstraint` carries `constraint_region.primitives`
/// (a `Vec` of primitive-shape messages), `constraint_region.meshes` (a
/// second `Vec`, of mesh messages) and `constraint_region.primitive_poses`/
/// `mesh_poses` (two more, one pose per entry in each of the first two) —
/// four vectors whose correctness rests entirely on the caller keeping
/// `primitives.len() == primitive_poses.len()` and
/// `meshes.len() == mesh_poses.len()` by convention; nothing stops them
/// drifting apart (upstream's own `configure()` has to defend against
/// exactly that with an `if (primitive_poses.size() <= i) { warn; continue; }`
/// guard per loop). [`ConstraintRegion`] pairs one [`Body`] (itself already a
/// sum type over sphere/cylinder/cuboid/mesh — see `cspace_core::geometry::bodies`)
/// with its one pose, so [`PositionConstraint`]'s regions are a single
/// `Vec<ConstraintRegion>` and the length-agreement invariant does not need
/// to be checked because it cannot fail to hold.
#[derive(Debug, Clone, PartialEq)]
pub struct ConstraintRegion {
    /// The region's shape.
    pub body: Body,
    /// The region's pose. Meaning depends on which
    /// [`PositionConstraint`] reference-frame variant holds this region —
    /// see that type's doc comment.
    pub pose: Isometry3,
}

/// Where a [`PositionConstraint`]'s regions are expressed, and what that
/// implies about revisiting their poses on every `decide()` call.
///
/// # Deviation from upstream: the mobile/fixed split owns the region list
///
/// Upstream keeps one `bool mobile_frame_` beside one
/// `EigenSTL::vector_Isometry3d constraint_region_pose_` whose *meaning*
/// flips with the flag: when fixed, every pose was already transformed into
/// the planning frame at `configure()` time (`decide()` reads them as-is);
/// when mobile, every pose is still relative to `constraint_frame_id_` and
/// must be re-transformed through `state.getFrameTransform()` on every call.
/// Same field, two meanings picked by a sibling flag — the same shape
/// `PORTING-PLAN.md` §4.1 already flagged for `RobotState`'s dirty flags.
/// Here each region list lives *inside* the variant whose meaning applies to
/// it, so a `Vec<ConstraintRegion>` obtained from a `PositionConstraint`
/// always carries one fixed meaning by construction.
#[derive(Debug, Clone, PartialEq)]
enum ReferenceFrame {
    /// `regions`' poses are already expressed in `frame` (upstream's
    /// `tf.getTargetFrame()`), resolved once at construction.
    Fixed {
        frame: String,
        regions: Vec<ConstraintRegion>,
    },
    /// `regions`' poses are relative to `frame` and must be re-resolved via
    /// [`Posed::frame_transform`] on every [`PositionConstraint::decide`]
    /// call.
    Mobile {
        frame: String,
        regions: Vec<ConstraintRegion>,
    },
}

/// Resolves `frame_id` to a fixed or mobile [`ReferenceFrame`] for `regions`,
/// exactly as [`PositionConstraint::new`] does — factored out so
/// [`PositionConstraint::with_updated_position`] (built for
/// `crate::constraints::utils::update_position_constraint`) can re-resolve a new frame
/// without duplicating the fixed/mobile decision.
fn resolve_frame(
    model: &RobotModel,
    tf: &Transforms,
    frame_id: &str,
    mut regions: Vec<ConstraintRegion>,
) -> Result<ReferenceFrame> {
    if tf.can_transform(frame_id) {
        for region in &mut regions {
            region.pose = tf.transform_pose(frame_id, &region.pose)?;
        }
        Ok(ReferenceFrame::Fixed {
            frame: tf.target_frame().to_string(),
            regions,
        })
    } else {
        // A mobile frame is resolved fresh on every `decide()` call via
        // `Posed::frame_transform`, which only ever names a link or the
        // model frame — checked now so `decide()` never has to handle a
        // frame that cannot resolve.
        if !model.has_link_model(frame_id) && frame_id != model.model_frame() {
            return Err(Error::unknown_name("frame", frame_id));
        }
        Ok(ReferenceFrame::Mobile {
            frame: frame_id.to_string(),
            regions,
        })
    }
}

/// Constrains a link's position (with an optional local offset) to lie
/// within one or more [`ConstraintRegion`]s.
///
/// Upstream `kinematic_constraints::PositionConstraint`.
#[derive(Debug, Clone, PartialEq)]
pub struct PositionConstraint {
    link_index: usize,
    link_name: String,
    offset: Vector3,
    frame: ReferenceFrame,
    weight: f64,
}

impl PositionConstraint {
    /// Build and resolve a position constraint against `model`.
    ///
    /// `shapes` is the region list as `(shape, pose)` pairs, `pose` being
    /// each region's pose relative to `frame_id`. `tf` decides, exactly as
    /// upstream's `configure(pc, tf)` does, whether `frame_id` is fixed
    /// (poses get transformed into `tf.target_frame()` once, here) or
    /// mobile (poses are kept relative to `frame_id` and resolved fresh on
    /// every `decide()`).
    ///
    /// `weight <= f64::EPSILON` (including negative weights) normalizes to
    /// `1.0` rather than erroring — see
    /// [`crate::constraints::JointConstraint::new`]'s "Weight normalization" doc section
    /// for the full rationale and the D6/D14 boundary this is not D6.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `link_name` is not in `model`.
    /// [`Error::Construct`] if `frame_id` is empty, if a shape fails to
    /// build a [`Body`] (a [`Shape::Cone`], [`Shape::Plane`] or
    /// [`Shape::OcTree`] region, or a malformed mesh — upstream instead logs
    /// a warning and drops just that region, keeping the constraint valid
    /// as long as one region remains; this port treats a region the caller
    /// cannot even construct as an input error rather than silently
    /// narrowing the constraint), or if no regions were given at all.
    pub fn new(
        model: &RobotModel,
        tf: &Transforms,
        link_name: &str,
        frame_id: &str,
        offset: Vector3,
        shapes: &[(Shape, Isometry3)],
        weight: f64,
    ) -> Result<Self> {
        let link = model.link_model(link_name)?;
        if frame_id.trim().is_empty() {
            return Err(Error::construct(
                "no frame specified for position constraint",
            ));
        }
        let weight = if weight <= EPS { 1.0 } else { weight };
        if shapes.is_empty() {
            return Err(Error::construct(
                "PositionConstraint needs at least one constraint region",
            ));
        }

        let mut regions = Vec::with_capacity(shapes.len());
        for (shape, pose) in shapes {
            let body = Body::from_shape(shape)?.ok_or_else(|| {
                Error::construct(format!(
                    "shape {shape:?} has no bodies:: counterpart to build a constraint region from"
                ))
            })?;
            regions.push(ConstraintRegion { body, pose: *pose });
        }

        let frame = resolve_frame(model, tf, frame_id, regions)?;

        Ok(Self {
            link_index: link.link_index(),
            link_name: link_name.to_string(),
            offset,
            frame,
            weight,
        })
    }

    /// `getLinkModel` (name only — this crate resolves indices privately).
    pub fn link_name(&self) -> &str {
        &self.link_name
    }

    /// `getLinkOffset`
    pub fn link_offset(&self) -> Vector3 {
        self.offset
    }

    /// `hasLinkOffset`: whether the offset is more than negligibly nonzero.
    /// Upstream caches this as `has_offset_`, computed once from `offset_`
    /// at `configure()` time and never touched again — a pure function of a
    /// field this type already stores, so this port recomputes it instead of
    /// storing a second copy of the same fact (see the crate's other
    /// `_bounded`-style caches for the cases where upstream's flag really is
    /// independent state, which are kept as fields).
    pub fn has_link_offset(&self) -> bool {
        self.offset.norm_squared() > EPS
    }

    /// `getReferenceFrame`
    pub fn reference_frame(&self) -> &str {
        match &self.frame {
            ReferenceFrame::Fixed { frame, .. } | ReferenceFrame::Mobile { frame, .. } => frame,
        }
    }

    /// `mobileReferenceFrame`
    pub fn mobile_reference_frame(&self) -> bool {
        matches!(self.frame, ReferenceFrame::Mobile { .. })
    }

    /// `getConstraintRegions`
    pub fn constraint_regions(&self) -> &[ConstraintRegion] {
        match &self.frame {
            ReferenceFrame::Fixed { regions, .. } | ReferenceFrame::Mobile { regions, .. } => {
                regions
            }
        }
    }

    /// Not an upstream accessor: needed by
    /// `crate::constraints::utils::update_position_constraint`, which reconstructs a
    /// `PositionConstraint` from an existing one's fields rather than
    /// mutating a stored `moveit_msgs` field in place (see this crate's
    /// introducing doc comment on why `new()` replaces `configure()`).
    pub fn weight(&self) -> f64 {
        self.weight
    }

    /// `updatePositionConstraint`'s core: a new constraint with the same
    /// link, offset, weight and single region's shape/orientation, but that
    /// region re-centered at `position` and re-resolved against `frame_id`
    /// (which — matching upstream's `constraint.header = goal_point.header`
    /// — may differ from this constraint's current reference frame).
    ///
    /// `Ok(None)` when this constraint has other than exactly one region:
    /// upstream's update only ever touches `primitive_poses.at(0)`, which
    /// would leave any further region's numeric pose stale under a changed
    /// frame_id — a pre-existing upstream wart no caller in `utils.cpp`
    /// actually exercises, since every `constructGoalConstraints` overload
    /// builds exactly one region. Rather than reproduce that latent
    /// inconsistency, this port supports only the case every real caller
    /// produces and reports the rest as "not updatable" instead of silently
    /// leaving other regions wrong.
    pub(crate) fn with_updated_position(
        &self,
        model: &RobotModel,
        tf: &Transforms,
        frame_id: &str,
        position: Vector3,
    ) -> Result<Option<Self>> {
        let regions = self.constraint_regions();
        if regions.len() != 1 {
            return Ok(None);
        }
        let mut pose = regions[0].pose;
        pose.translation.vector = position;
        let region = ConstraintRegion {
            body: regions[0].body.clone(),
            pose,
        };
        let frame = resolve_frame(model, tf, frame_id, vec![region])?;
        Ok(Some(Self {
            link_index: self.link_index,
            link_name: self.link_name.clone(),
            offset: self.offset,
            frame,
            weight: self.weight,
        }))
    }

    /// `PositionConstraint::decide`.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        // `self.offset` is a point in the link's local frame, not a free
        // vector; `nalgebra::Isometry3: Mul<Vector3>` gives vector semantics
        // (rotation only, no translation), unlike upstream's
        // `Eigen::Isometry3d * Eigen::Vector3d`, which treats a plain vector
        // as a point and applies the full transform. Going through
        // `nalgebra::Point3` is the same fix `cspace_core::geometry::bodies`'
        // private `transform_point` helper already applies for this exact
        // defect shape.
        let pt = (state.global_link_transform_at(self.link_index)
            * nalgebra::Point3::from(self.offset))
        .coords;

        match &self.frame {
            ReferenceFrame::Fixed { regions, .. } => {
                for (i, region) in regions.iter().enumerate() {
                    let ok = region.body.clone_at(region.pose).contains_point(&pt);
                    if ok || i + 1 == regions.len() {
                        return finish(pt, region.pose.translation.vector, self.weight, ok);
                    }
                }
            }
            ReferenceFrame::Mobile { frame, regions } => {
                // Constructed only when `frame` names a link or the model
                // frame (see `new`'s `Mobile` branch), so this cannot fail.
                let frame_tf = state
                    .frame_transform(frame)
                    .expect("mobile reference frame was validated resolvable at construction");
                for (i, region) in regions.iter().enumerate() {
                    let world_pose = frame_tf * region.pose;
                    let ok = region.body.clone_at(world_pose).contains_point(&pt);
                    if ok || i + 1 == regions.len() {
                        return finish(pt, world_pose.translation.vector, self.weight, ok);
                    }
                }
            }
        }
        ConstraintEvaluationResult::new(false, 0.0)
    }
}

fn finish(
    pt: Vector3,
    desired: Vector3,
    weight: f64,
    satisfied: bool,
) -> ConstraintEvaluationResult {
    let d = desired - pt;
    ConstraintEvaluationResult::new(satisfied, weight * d.norm())
}
