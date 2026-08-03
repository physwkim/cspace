// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//     (KDLKinematicsPlugin::initialize, lines 126-251)

use nalgebra::DMatrix;

use moveit_error::{Error, Result};
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;
use moveit_model::joint::{JointKind, JointType};
use moveit_state::Posed;

/// The per-(model, group) setup `KDLKinematicsPlugin::initialize` computes
/// once and every `searchPositionIK` call reuses: which joints are in the
/// chain, which of those are active vs. mimic (and each mimic's master and
/// factor), and each one's position bounds.
///
/// Every field here is indexed in the group's own depth-first joint order,
/// restricted to non-fixed joints — the same order
/// [`moveit_state::Posed::jacobian`]'s columns come back in, since both this
/// type and that method read it off the same
/// [`moveit_model::JointModelGroup::variable_names`]. Call this the *full*
/// space (upstream's `dimension_`: active-joint count plus mimic-joint
/// count). A second, shorter space — the *reduced* space, upstream's
/// mimic-folded Jacobian columns — has one entry per **active** joint only;
/// [`ChainInfo::map_index`] and [`ChainInfo::multiplier`] are what maps a
/// full-space entry onto its reduced-space column.
///
/// # Deviation from upstream: the public seed/solution vectors are
/// reduced-space, not full-space
///
/// `KDLKinematicsPlugin`'s own public `ik_seed_state`/`solution` vectors are
/// full-space (size `dimension_`, one entry per mimic joint too), because
/// upstream's `KDL::JntArray q_out` is a raw buffer with no notion that a
/// mimic joint's value is derived. This port's [`moveit_state::RobotState`]
/// already derives a mimic joint's position from its master whenever the
/// master is set ([`moveit_state::RobotState::set_variable_position`]), so
/// carrying redundant mimic entries in the public API would let a caller
/// pass a self-contradictory seed with no way to detect it. Every solver in
/// this crate takes/returns [`ChainInfo::active_joint_names`]-ordered
/// (reduced-space) vectors instead; only the internal iteration loop
/// ([`crate::cart_to_jnt`]) ever sees full-space values, exactly as
/// `KDL::JntArray q_out` does upstream.
#[derive(Debug, Clone)]
pub(crate) struct ChainInfo {
    pub(crate) group_name: String,
    /// This chain's tip link: `group->getLinkModels().back()`.
    pub(crate) tip_link_index: usize,
    /// The link the chain's root joint hangs off. `None` for the model's
    /// absolute root joint, matching
    /// `moveit_state::RobotState`'s own `parent_link_of_joint`.
    pub(crate) root_link_index: Option<usize>,
    /// `KinematicsBase::base_frame_`: [`ChainInfo::root_link_index`]'s link
    /// name, or [`moveit_model::RobotModel::root_link_name`] when that index
    /// is `None`. The frame every [`crate::KinematicsSolver::solve`] target
    /// pose is given in — see [`ChainInfo::root_pose_world`].
    pub(crate) base_frame: String,
    /// `KinematicsBase::tip_frames_[0]`: [`ChainInfo::tip_link_index`]'s link
    /// name.
    pub(crate) tip_frame: String,

    // ---- Full space (dimension_), depth-first order ------------------
    /// Each full-space entry's own joint name (== its one variable's name,
    /// see [`ChainInfo::build`]'s single-DOF check).
    pub(crate) joint_names: Vec<String>,
    /// `JointMimic::map_index`: which reduced-space column this full-space
    /// entry folds into.
    pub(crate) map_index: Vec<usize>,
    /// `JointMimic::multiplier`: `1.0` for an active joint, `getMimicFactor()`
    /// for a mimic.
    pub(crate) multiplier: Vec<f64>,
    /// `joint_min_`: `VariableBounds::min_position_`, taken unconditionally
    /// — see [`ChainInfo::build`]'s doc comment on why an unbounded
    /// (continuous) joint is not exempt from this.
    pub(crate) min: Vec<f64>,
    /// `joint_max_`, see [`ChainInfo::min`].
    pub(crate) max: Vec<f64>,
    /// This full-space entry's own child link — the link this joint is the
    /// parent of. Used only by [`ChainInfo::full_jacobian`]; see that
    /// method's doc comment for why this crate cannot reuse
    /// [`moveit_state::Posed::jacobian`] directly.
    pub(crate) link_index: Vec<usize>,
    /// This full-space entry's index into
    /// [`moveit_model::RobotModel::variable_names`] — where
    /// [`crate::cart_to_jnt`] writes it back into a whole-model positions
    /// buffer for [`moveit_state::RobotState::set_variable_positions`].
    pub(crate) variable_index: Vec<usize>,

    // ---- Reduced space (active joints only), same relative order ------
    /// The public solve API's vector order.
    pub(crate) active_joint_names: Vec<String>,
    /// Each active joint's own `min`/`max`, used only for
    /// `getRandomConfiguration`-equivalent re-seeding between
    /// [`crate::cart_to_jnt::search_position_ik`] attempts — a mimic
    /// joint's bounds are irrelevant there, since its value is never
    /// independently chosen.
    pub(crate) active_min: Vec<f64>,
    pub(crate) active_max: Vec<f64>,
}

impl ChainInfo {
    /// `KDLKinematicsPlugin::initialize`'s group-shape checks and the
    /// `mimic_joints_`/`joint_min_`/`joint_max_` setup that follows them.
    ///
    /// # Deviation from upstream: an in-chain mimic whose master is outside
    /// the group is rejected, not silently miscounted
    ///
    /// Upstream's `dimension_` counts every mimic joint model in the group,
    /// but its `mimic_joints_`-building loop only ever pushes an entry for
    /// a mimic whose master also has `joint_model_group_->hasJointModel(...)`
    /// — one whose master is not in the group is silently dropped, leaving
    /// `mimic_joints_.size() < dimension_` and desynchronising every
    /// `mimic_joints_[i]` lookup after the drop point for the rest of
    /// `initialize`. None of this port's fixtures have a mimic joint on a
    /// chain whose master sits outside the chain's own group, so this is
    /// a construction error here instead.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// [`Error::Other`] if the group is not a chain
    /// ([`moveit_model::JointModelGroup::is_chain`]), if it contains a
    /// joint with more than one variable (`isSingleDOFJoints()` — planar
    /// and floating joints), if it contains a joint that is neither
    /// revolute nor prismatic (a single-DOF continuous revolute is still
    /// revolute; this excludes only fixed — already skipped — and the
    /// unreachable-here planar/floating cases, which the DOF check above
    /// already catches first), or if an in-chain mimic's master is not
    /// itself in the group (see this method's deviation doc).
    pub(crate) fn build(model: &RobotModel, group_name: &str) -> Result<Self> {
        let group = model.joint_model_group(group_name)?;
        if !group.is_chain() {
            return Err(Error::other(format!(
                "group '{group_name}' is not a chain; only chain groups are supported"
            )));
        }

        let tip_link_index = *group
            .link_indices()
            .last()
            .expect("a chain group has at least one link");
        // The link `root_joint` hangs *from*: find `root_joint`'s own child
        // link (the one link whose parent joint is `root_joint`), then that
        // link's own parent link -- the same two-hop lookup
        // `moveit_state::RobotState::parent_link_of_joint` does with its
        // private `link_of_joint` index, rebuilt here from the public
        // `LinkModel::parent_joint_index`/`LinkModel::parent_link_index`
        // this crate does have.
        let root_joint = group.joint_indices()[0];
        let root_link_index = model
            .link_models()
            .iter()
            .find(|l| l.parent_joint_index() == root_joint)
            .and_then(|l| l.parent_link_index());

        let mut joint_names = Vec::new();
        let mut min = Vec::new();
        let mut max = Vec::new();
        let mut link_index = Vec::new();
        let mut variable_index = Vec::new();
        let mut is_active = Vec::new();
        let mut mimic_of: Vec<Option<(String, f64)>> = Vec::new();

        for &joint_index in group.joint_indices() {
            let joint = model.joint_model_at(joint_index);
            if joint.variable_count() == 0 {
                continue; // fixed: not part of dimension_
            }
            if joint.variable_count() != 1 {
                return Err(Error::other(format!(
                    "group '{group_name}' includes joint '{}' with {} DOF; only single-DOF joints are supported",
                    joint.name(),
                    joint.variable_count()
                )));
            }
            if !matches!(
                joint.joint_type(),
                JointType::Revolute | JointType::Prismatic
            ) {
                return Err(Error::other(format!(
                    "group '{group_name}' includes joint '{}' of unsupported type {}",
                    joint.name(),
                    joint.type_name()
                )));
            }

            let bounds = &joint.variable_bounds()[0];
            joint_names.push(joint.name().to_owned());
            min.push(bounds.min_position);
            max.push(bounds.max_position);
            link_index.push(
                model
                    .link_models()
                    .iter()
                    .find(|l| l.parent_joint_index() == joint_index)
                    .expect("every non-fixed joint in a chain has a child link")
                    .link_index(),
            );
            variable_index.push(
                model
                    .variable_index(joint.name())
                    .expect("a single-DOF joint's own name is one of its variable names"),
            );

            match joint.mimic() {
                None => {
                    is_active.push(true);
                    mimic_of.push(None);
                }
                Some(m) => {
                    is_active.push(false);
                    mimic_of.push(Some((m.joint_name.clone(), m.factor)));
                }
            }
        }

        let mut active_joint_names = Vec::new();
        let mut active_min = Vec::new();
        let mut active_max = Vec::new();
        let mut reduced_index_of: std::collections::HashMap<&str, usize> =
            std::collections::HashMap::new();
        for (i, (name, &active)) in joint_names.iter().zip(&is_active).enumerate() {
            if active {
                reduced_index_of.insert(name.as_str(), active_joint_names.len());
                active_joint_names.push(name.clone());
                active_min.push(min[i]);
                active_max.push(max[i]);
            }
        }

        let mut map_index = Vec::with_capacity(joint_names.len());
        let mut multiplier = Vec::with_capacity(joint_names.len());
        for (i, name) in joint_names.iter().enumerate() {
            match &mimic_of[i] {
                None => {
                    map_index.push(reduced_index_of[name.as_str()]);
                    multiplier.push(1.0);
                }
                Some((master, factor)) => {
                    let Some(&reduced) = reduced_index_of.get(master.as_str()) else {
                        return Err(Error::other(format!(
                            "group '{group_name}' includes mimic joint '{name}' whose master '{master}' is not itself in the group"
                        )));
                    };
                    map_index.push(reduced);
                    multiplier.push(*factor);
                }
            }
        }

        let base_frame = match root_link_index {
            Some(idx) => model.link_model_at(idx).name().to_owned(),
            None => model.root_link_name().to_owned(),
        };
        let tip_frame = model.link_model_at(tip_link_index).name().to_owned();

        Ok(Self {
            group_name: group_name.to_owned(),
            tip_link_index,
            root_link_index,
            base_frame,
            tip_frame,
            joint_names,
            map_index,
            multiplier,
            min,
            max,
            link_index,
            variable_index,
            active_joint_names,
            active_min,
            active_max,
        })
    }

    /// `dimension_`.
    pub(crate) fn dimension(&self) -> usize {
        self.joint_names.len()
    }

    /// The transform from the model's world frame into this chain's own
    /// base-link frame — `KDL::Chain`'s implicit base, and the frame every
    /// [`crate::KinematicsSolver::solve`] target pose is given in. Identity
    /// when the chain's root joint has no parent link (it hangs directly
    /// off the model root).
    pub(crate) fn root_pose_world(&self, posed: &Posed) -> Isometry3 {
        match self.root_link_index {
            Some(root_link) => posed.global_link_transform_at(root_link).inverse(),
            None => Isometry3::identity(),
        }
    }

    /// `KinematicsBase::getBaseFrame`.
    pub(crate) fn base_frame(&self) -> &str {
        &self.base_frame
    }

    /// `KinematicsBase::getTipFrame`.
    pub(crate) fn tip_frame(&self) -> &str {
        &self.tip_frame
    }

    /// `KDLKinematicsPlugin::getJointWeights`: every active joint defaults
    /// to weight `1.0`, then [`crate::SolverParams::joint_weights`]
    /// overrides by joint name.
    pub(crate) fn resolve_joint_weights(&self, params: &crate::params::SolverParams) -> Vec<f64> {
        self.active_joint_names
            .iter()
            .map(|name| *params.joint_weights.get(name).unwrap_or(&1.0))
            .collect()
    }

    /// The reduced (active-joint) space's size.
    pub(crate) fn reduced_dimension(&self) -> usize {
        self.active_joint_names.len()
    }

    /// [`crate::KinematicsSolver::joint_names`]'s return value: the public,
    /// reduced-space (active-joint-only) name order.
    pub(crate) fn solver_joint_names(&self) -> &[String] {
        &self.active_joint_names
    }

    /// This chain's full-space (`dimension()`-column), *unfolded*
    /// geometric Jacobian: column `i`'s linear/angular velocity
    /// contribution as if [`ChainInfo::joint_names`]`[i]` moved
    /// independently, mimic or not. Rows 0-2 are linear velocity, rows 3-5
    /// angular velocity, of the tip link's own origin, expressed in the
    /// chain root's frame — matching `KDL::ChainJntToJacSolver::JntToJac`'s
    /// output shape, which is what upstream's
    /// `ChainIkSolverVelMimicSVD::CartToJnt` folds through
    /// `jacToJacReduced`.
    ///
    /// # Deviation from upstream: cannot reuse
    /// [`moveit_state::Posed::jacobian`]
    ///
    /// That method matches `moveit_core::RobotState::getJacobian`, which
    /// only ever writes a column for a group's *active* joints
    /// (`group->getActiveJointModels()`) — a mimic joint's column is left
    /// all-zero. That is faithful for `moveit_core`'s own callers, none of
    /// which fold mimic contributions back onto a master column, but it is
    /// exactly wrong as an input to this crate's mimic fold: KDL's
    /// `ChainJntToJacSolver` treats every chain segment (mimic or not) as
    /// an independent joint and computes its own real column, and
    /// `jacToJacReduced` relies on that mimic column being non-zero before
    /// scaling it by [`ChainInfo::multiplier`] and summing it into the
    /// master's [`ChainInfo::map_index`] column. Reusing
    /// [`moveit_state::Posed::jacobian`] here would silently drop every
    /// mimic joint's geometric contribution to the tip's motion — the
    /// exact case `pr2`'s mimic-chain fixture exists to catch. This method
    /// therefore recomputes each column directly, reusing only
    /// [`moveit_state::Posed::global_link_transform_at`] (a public,
    /// already-fresh transform read) rather than
    /// [`moveit_state::Posed::jacobian`]'s active-only loop.
    pub(crate) fn full_jacobian(&self, posed: &Posed) -> DMatrix<f64> {
        let root_pose_world = self.root_pose_world(posed);
        let root_pose_tip = root_pose_world * posed.global_link_transform_at(self.tip_link_index);
        let tip_point = root_pose_tip.translation.vector;

        let mut jacobian = DMatrix::<f64>::zeros(6, self.dimension());
        for i in 0..self.dimension() {
            let root_pose_link =
                root_pose_world * posed.global_link_transform_at(self.link_index[i]);
            let joint = posed
                .model()
                .joint_model(&self.joint_names[i])
                .expect("ChainInfo::joint_names only ever holds real joint names");
            match joint.kind() {
                JointKind::Revolute(revolute) => {
                    let axis_wrt_origin = root_pose_link.rotation * revolute.axis();
                    let linear =
                        axis_wrt_origin.cross(&(tip_point - root_pose_link.translation.vector));
                    jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&linear);
                    jacobian
                        .fixed_view_mut::<3, 1>(3, i)
                        .copy_from(&axis_wrt_origin);
                }
                JointKind::Prismatic(prismatic) => {
                    let axis_wrt_origin = root_pose_link.rotation * prismatic.axis();
                    jacobian
                        .fixed_view_mut::<3, 1>(0, i)
                        .copy_from(&axis_wrt_origin);
                }
                JointKind::Planar(_) | JointKind::Floating(_) | JointKind::Fixed => {
                    unreachable!(
                        "ChainInfo::build only ever admits Revolute/Prismatic joints into joint_names"
                    );
                }
            }
        }
        jacobian
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_srdf::SrdfModel;

    fn build_model_from_str(urdf_xml: &str, srdf_xml: &str) -> RobotModel {
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("inline SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf).expect("inline model must build")
    }

    /// `arms` combines `left`/`right` as two independent one-joint chains:
    /// two roots, so `is_chain()` fails before `ChainInfo::build` even
    /// starts walking joints.
    #[test]
    fn build_rejects_a_non_chain_group() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="two_arms">
  <link name="root"/>
  <link name="left_tip"/>
  <link name="right_tip"/>
  <joint name="left" type="revolute">
    <parent link="root"/>
    <child link="left_tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="right" type="revolute">
    <parent link="root"/>
    <child link="right_tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="two_arms">
  <group name="left">
    <joint name="left"/>
  </group>
  <group name="right">
    <joint name="right"/>
  </group>
  <group name="arms">
    <group name="left"/>
    <group name="right"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let err = ChainInfo::build(&model, "arms").unwrap_err();
        assert!(err.to_string().contains("not a chain"), "got: {err}");
    }

    /// A group named by nothing in the SRDF is `Error::UnknownName`, not
    /// folded into the "not a chain" message — matches
    /// `moveit_state::Posed::jacobian`'s own boundary.
    #[test]
    fn build_reports_an_unknown_group_as_unknown_name() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="one_link">
  <link name="root"/>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="one_link">
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let err = ChainInfo::build(&model, "no_such_group").unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }));
    }

    /// A single floating joint is a trivial one-joint chain (nothing to
    /// fail the adjacency check), so `build` reaches its own DOF check and
    /// must reject it there for having 7 variables, not silently truncate
    /// it to one.
    #[test]
    fn build_rejects_a_multi_dof_joint() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="floaty">
  <link name="root"/>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="floaty">
  <virtual_joint name="virtual_joint" type="floating" parent_frame="world" child_link="root"/>
  <group name="whole">
    <joint name="virtual_joint"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let err = ChainInfo::build(&model, "whole").unwrap_err();
        assert!(err.to_string().contains("DOF"), "got: {err}");
    }

    /// `tip`'s mimic points at `hidden`, a joint on a sibling branch that
    /// never appears on the `root`-to-`tip` path and so is not part of
    /// `chain`'s own joint list — `ChainInfo::build`'s own deviation from
    /// upstream (see this method's doc comment) is to reject this rather
    /// than silently desynchronise `mimic_joints_`.
    #[test]
    fn build_rejects_an_in_chain_mimic_whose_master_is_outside_the_group() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="mimic_outside">
  <link name="root"/>
  <link name="hidden_tip"/>
  <link name="mid"/>
  <link name="tip"/>
  <joint name="hidden" type="revolute">
    <parent link="root"/>
    <child link="hidden_tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="root"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
    <mimic joint="hidden" multiplier="2.0" offset="0.0"/>
  </joint>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="mimic_outside">
  <group name="chain">
    <chain base_link="root" tip_link="tip"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let err = ChainInfo::build(&model, "chain").unwrap_err();
        assert!(
            err.to_string().contains("not itself in the group"),
            "got: {err}"
        );
    }

    /// `resolve_joint_weights`: unlisted joints default to `1.0`;
    /// [`crate::SolverParams::joint_weights`] overrides by name and only by
    /// name — a name it does not mention must not perturb any other
    /// column's weight.
    #[test]
    fn resolve_joint_weights_defaults_to_one_and_overrides_by_name_only() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <link name="root"/>
  <link name="mid"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="root"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <group name="chain">
    <chain base_link="root" tip_link="tip"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let chain = ChainInfo::build(&model, "chain").expect("valid two-joint chain");

        let mut params = crate::params::SolverParams::default();
        params.joint_weights.insert("j2".to_owned(), 0.25);
        let weights = chain.resolve_joint_weights(&params);

        assert_eq!(chain.active_joint_names, vec!["j1", "j2"]);
        assert_eq!(weights, vec![1.0, 0.25]);
    }

    /// The ordinary case: the chain's root joint's parent link is a real,
    /// non-root link, so `base_frame`/`tip_frame` resolve to that link's own
    /// name and the tip link's own name respectively — not the model root.
    #[test]
    fn base_frame_and_tip_frame_resolve_to_the_chain_endpoint_link_names() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <link name="root"/>
  <link name="mid"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="root"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <group name="chain">
    <chain base_link="root" tip_link="tip"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let chain = ChainInfo::build(&model, "chain").expect("valid two-joint chain");

        assert_eq!(chain.base_frame(), "root");
        assert_eq!(chain.tip_frame(), "tip");
    }

    /// When the chain's root joint *is* the model's own absolute root joint
    /// (here: the group names the virtual joint explicitly, ahead of `j1`),
    /// `root_link_index` is `None` (see `root_pose_world`'s doc comment) and
    /// `base_frame` must fall back to the model's own root link name rather
    /// than panicking or returning an empty string.
    #[test]
    fn base_frame_falls_back_to_the_model_root_link_when_the_chain_starts_there() {
        let urdf = r#"<?xml version="1.0"?>
<robot name="from_the_top">
  <link name="root"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="root"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf = r#"<?xml version="1.0"?>
<robot name="from_the_top">
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="root"/>
  <group name="chain">
    <joint name="virtual_joint"/>
    <joint name="j1"/>
  </group>
</robot>
"#;
        let model = build_model_from_str(urdf, srdf);
        let chain = ChainInfo::build(&model, "chain").expect("valid one-joint chain from the root");

        assert_eq!(chain.root_link_index, None);
        assert_eq!(chain.base_frame(), model.root_link_name());
        assert_eq!(chain.base_frame(), "root");
        assert_eq!(chain.tip_frame(), "tip");
    }
}
