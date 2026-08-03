// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/robot_model.hpp
//   moveit_core/robot_model/src/robot_model.cpp
//   moveit_core/robot_model/src/joint_model_group.cpp (group construction)

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, UnitQuaternion};
use moveit_srdf::{Group, SrdfModel, VirtualJointType};
use nalgebra::Translation3;

use crate::diagnostic::Diagnostic;
use crate::joint::{JointModel, JointType, PlanarMotionModel, joint_model_from_urdf};
use crate::joint_model_group::JointModelGroup;
use crate::link_model::LinkModel;

/// The tree/index bookkeeping upstream stores directly on `JointModel`
/// (`joint_index_`, `first_variable_index_`, `parent_link_model_`,
/// `child_link_model_`) but which this port's [`crate::joint::JointModel`]
/// deliberately excludes — see that type's doc comment. `RobotModel` is the
/// "later phase" that doc comment refers to; this struct is where the
/// bookkeeping actually lives, one per joint, indexed by position in
/// [`RobotModel::joints`].
#[derive(Debug, Clone, PartialEq)]
struct JointNode {
    model: JointModel,
    first_variable_index: usize,
    parent_link_index: Option<usize>,
    child_link_index: usize,
}

/// A robot's full kinematic model: every link and joint from a URDF, plus the
/// SRDF's virtual joint (the model's actual root) and planning groups.
///
/// Upstream `moveit::core::RobotModel`. Build one with
/// [`RobotModel::from_urdf_and_srdf`].
///
/// # Deviations from upstream
///
/// 1. **Cross-references are indices, not pointers**, for the same reason as
///    [`LinkModel`]: no raw pointers into a sibling `Vec`.
/// 2. **No collision/visual geometry, group states, end effectors, or
///    kinematics solver plumbing.** Upstream's `RobotModel` also carries
///    `link_models_with_collision_geometry_vector_`,
///    `default_states_`/`buildGroupStates`, `end_effectors_`, and
///    `group_kinematics_`. `PORTING-PLAN.md` puts collision geometry in
///    `moveit-collision` and kinematics solvers in `moveit-kinematics`, later
///    phases; SRDF `<group_state>` and `<end_effector>` elements are read by
///    `moveit-srdf` but not consumed here, for the same reason. None of these
///    are read by this phase's done-criteria (link/joint counts, group
///    composition, joint limits, mimic relationships).
/// 3. **No `common_root_`/`joint_roots_`/`is_chain_`/`is_single_dof_` on
///    groups, and no `computeDescendants`/`computeCommonRoots` on the
///    model.** These exist upstream purely to accelerate `RobotState`
///    forward kinematics (a later phase); the FK algorithm itself does not
///    need them to be *correct*, only fast.
/// 4. **No `getVariableRandomPositions`.** `PORTING-PLAN.md` §7.3: the C++
///    oracle owns randomness for differential testing; this port never
///    needs to generate a random state itself.
/// 5. **A `<joint_property>` numeric value must be entirely numeric.**
///    Upstream reads it with `std::stod`, which parses a numeric *prefix*
///    and only warns about trailing garbage — `"3.5garbage"` silently
///    becomes `3.5`. This port treats that as
///    [`Diagnostic::JointPropertyMalformedValue`] instead, matching the
///    stricter-than-upstream stance `moveit_srdf::SrdfModel::group_states`
///    already takes on the same class of mistake (a typo should not
///    silently become a valid, wrong number).
/// 6. **The root link's virtual-joint matching is narrower than upstream's.**
///    Upstream additionally special-cases a `world`-named root link with no
///    geometry (a Gazebo convention) when no virtual joint's `child_link`
///    matches, and reports every non-matching or empty-`parent_frame`
///    virtual joint through its logger. Neither panda's nor fanuc's SRDF
///    exercises those paths — each has exactly one `<virtual_joint>` whose
///    `child_link` is the URDF root — so this port skips them silently
///    rather than adding a `Diagnostic` variant untested by any fixture.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotModel {
    name: String,
    model_frame: String,
    root_link_index: usize,
    joints: Vec<JointNode>,
    joint_index_by_name: HashMap<String, usize>,
    joint_names: Vec<String>,
    links: Vec<LinkModel>,
    link_index_by_name: HashMap<String, usize>,
    link_names: Vec<String>,
    variable_names: Vec<String>,
    variable_index: HashMap<String, usize>,
    active_joint_indices: Vec<usize>,
    groups: BTreeMap<String, JointModelGroup>,
    diagnostics: Vec<Diagnostic>,
}

impl RobotModel {
    /// Build a model from a URDF and its matching SRDF.
    ///
    /// Upstream `RobotModel::RobotModel`/`buildModel`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if the URDF has no link with no parent joint (no
    /// root), more than one such link, or a joint
    /// [`joint_model_from_urdf`] rejects (a `Spherical` joint).
    pub fn from_urdf_and_srdf(urdf: &urdf_rs::Robot, srdf: &SrdfModel) -> Result<Self> {
        // Upstream builds this same adjacency in `urdf::ModelInterface::initTree`,
        // which iterates `joints_` — a `std::map<std::string, JointSharedPtr>` — in
        // ascending key order. Each parent link's `child_joints` therefore ends up
        // sorted by joint *name*, not by the joints' order in the XML document;
        // sorting here before grouping reproduces that (verified against fanuc's
        // `base_link`, which has two children — `base_link-base` and `joint_1` —
        // declared in the opposite order in the URDF but visited
        // `base_link-base` first, alphabetically, by the oracle).
        let mut sorted_joints: Vec<&urdf_rs::Joint> = urdf.joints.iter().collect();
        sorted_joints.sort_by(|a, b| a.name.cmp(&b.name));

        let mut child_link_names: HashSet<&str> = HashSet::new();
        let mut children: HashMap<&str, Vec<&urdf_rs::Joint>> = HashMap::new();
        for &joint in &sorted_joints {
            child_link_names.insert(joint.child.link.as_str());
            children
                .entry(joint.parent.link.as_str())
                .or_default()
                .push(joint);
        }

        let root_candidates: Vec<&str> = urdf
            .links
            .iter()
            .map(|l| l.name.as_str())
            .filter(|name| !child_link_names.contains(name))
            .collect();
        let root_link_name = match root_candidates.as_slice() {
            [name] => *name,
            [] => {
                return Err(Error::construct(format!(
                    "URDF '{}' has no root link",
                    urdf.name
                )));
            }
            names => {
                return Err(Error::construct(format!(
                    "URDF '{}' has {} root links, expected exactly one",
                    urdf.name,
                    names.len()
                )));
            }
        };

        let (root_joint, model_frame) = root_virtual_joint(srdf, root_link_name);

        let mut building = Building {
            srdf,
            children,
            joints: Vec::new(),
            joint_index_by_name: HashMap::new(),
            joint_names: Vec::new(),
            links: Vec::new(),
            link_index_by_name: HashMap::new(),
            link_names: Vec::new(),
            diagnostics: Vec::new(),
        };
        building.visit(None, root_link_name, root_joint, Isometry3::identity())?;
        building.resolve_mimic();

        let (variable_names, variable_index, active_joint_indices) =
            compute_variable_layout(&building.joints);
        let mut groups = building.build_groups();
        compute_subgroups(&mut groups);

        Ok(Self {
            name: urdf.name.clone(),
            model_frame,
            root_link_index: 0,
            joints: building.joints,
            joint_index_by_name: building.joint_index_by_name,
            joint_names: building.joint_names,
            links: building.links,
            link_index_by_name: building.link_index_by_name,
            link_names: building.link_names,
            variable_names,
            variable_index,
            active_joint_indices,
            groups,
            diagnostics: building.diagnostics,
        })
    }

    /// `getName`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `getModelFrame`
    pub fn model_frame(&self) -> &str {
        &self.model_frame
    }

    /// `getRootLinkName`
    pub fn root_link_name(&self) -> &str {
        self.links[self.root_link_index].name()
    }

    /// `getRootLink`, as an index.
    pub fn root_link_index(&self) -> usize {
        self.root_link_index
    }

    /// `getLinkModelNames`, in the order links are visited depth-first.
    pub fn link_names(&self) -> &[String] {
        &self.link_names
    }

    /// `getJointModelNames`, in depth-first order (the virtual/root joint
    /// first).
    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    /// `getVariableNames`: every non-fixed joint's variables, in joint order.
    pub fn variable_names(&self) -> &[String] {
        &self.variable_names
    }

    /// `getVariableCount`
    pub fn variable_count(&self) -> usize {
        self.variable_names.len()
    }

    /// The global variable index of a joint's own first variable, or of a
    /// single named variable. Upstream `getVariableIndex`, folded into one
    /// map the way `joint_variables_index_map_` is (it keys both variable
    /// names and joint names).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable or joint name in
    /// this model.
    pub fn variable_index(&self, name: &str) -> Result<usize> {
        self.variable_index
            .get(name)
            .copied()
            .ok_or_else(|| Error::unknown_name("variable", name))
    }

    /// `getActiveJointModels`, as indices: joints with controllable DOF
    /// (excludes fixed and mimic joints), in joint order.
    pub fn active_joint_indices(&self) -> &[usize] {
        &self.active_joint_indices
    }

    /// `getLinkModels`
    pub fn link_models(&self) -> &[LinkModel] {
        &self.links
    }

    /// `getLinkModel(std::size_t)`; panics if `index` is out of range, which
    /// cannot happen for an index this model itself produced.
    pub fn link_model_at(&self, index: usize) -> &LinkModel {
        &self.links[index]
    }

    /// `getLinkModel(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no link is named `name`.
    pub fn link_model(&self, name: &str) -> Result<&LinkModel> {
        self.link_index_by_name
            .get(name)
            .map(|&i| &self.links[i])
            .ok_or_else(|| Error::unknown_name("link", name))
    }

    /// `hasLinkModel`
    pub fn has_link_model(&self, name: &str) -> bool {
        self.link_index_by_name.contains_key(name)
    }

    /// `getJointModels`
    pub fn joint_models(&self) -> impl Iterator<Item = &JointModel> {
        self.joints.iter().map(|node| &node.model)
    }

    /// `getJointModel(std::size_t)`; panics if `index` is out of range, which
    /// cannot happen for an index this model itself produced.
    pub fn joint_model_at(&self, index: usize) -> &JointModel {
        &self.joints[index].model
    }

    /// `getJointModel(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_model(&self, name: &str) -> Result<&JointModel> {
        self.joint_index_by_name
            .get(name)
            .map(|&i| &self.joints[i].model)
            .ok_or_else(|| Error::unknown_name("joint", name))
    }

    /// `hasJointModel`
    pub fn has_joint_model(&self, name: &str) -> bool {
        self.joint_index_by_name.contains_key(name)
    }

    /// `getParentJointModel`, as an index.
    pub fn parent_joint_index(&self, joint_index: usize) -> Option<usize> {
        self.joints[joint_index]
            .parent_link_index
            .map(|link_index| self.links[link_index].parent_joint_index())
    }

    /// `getJointModelGroupNames`, alphabetically (matches upstream's
    /// `std::map<std::string, JointModelGroup*>` iteration order).
    pub fn joint_model_group_names(&self) -> impl Iterator<Item = &str> {
        self.groups.keys().map(String::as_str)
    }

    /// `getJointModelGroup`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `name`.
    pub fn joint_model_group(&self, name: &str) -> Result<&JointModelGroup> {
        self.groups
            .get(name)
            .ok_or_else(|| Error::unknown_name("group", name))
    }

    /// `hasJointModelGroup`
    pub fn has_joint_model_group(&self, name: &str) -> bool {
        self.groups.contains_key(name)
    }

    /// Everything [`RobotModel::from_urdf_and_srdf`] dropped or repaired
    /// while building this model.
    pub fn diagnostics(&self) -> &[Diagnostic] {
        &self.diagnostics
    }
}

/// Upstream `RobotModel::constructJointModel`'s `else` branch (root link,
/// `parent_joint == nullptr`): find the SRDF virtual joint whose `child_link`
/// is this URDF's root, and build the joint it describes. Falls back to an
/// assumed fixed joint if none matches, matching upstream's
/// `ASSUMED_FIXED_ROOT_JOINT`.
///
/// See [`RobotModel`]'s doc comment, deviation 6, for what this does not
/// reproduce from the upstream branch.
fn root_virtual_joint(srdf: &SrdfModel, root_link_name: &str) -> (JointModel, String) {
    for virtual_joint in srdf.virtual_joints() {
        if virtual_joint.child_link != root_link_name || virtual_joint.parent_frame.is_empty() {
            continue;
        }
        return match virtual_joint.joint_type {
            VirtualJointType::Fixed => (
                JointModel::new_fixed(virtual_joint.name.clone()),
                root_link_name.to_string(),
            ),
            VirtualJointType::Planar => (
                JointModel::new_planar(virtual_joint.name.clone()),
                virtual_joint.parent_frame.clone(),
            ),
            VirtualJointType::Floating => (
                JointModel::new_floating(virtual_joint.name.clone()),
                virtual_joint.parent_frame.clone(),
            ),
        };
    }
    (
        JointModel::new_fixed("ASSUMED_FIXED_ROOT_JOINT"),
        root_link_name.to_string(),
    )
}

/// Upstream `urdfPose2Isometry3d`.
fn isometry_from_urdf_pose(pose: &urdf_rs::Pose) -> Isometry3 {
    let [x, y, z] = pose.xyz.0;
    let [roll, pitch, yaw] = pose.rpy.0;
    Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_euler_angles(roll, pitch, yaw),
    )
}

/// The in-progress state of a [`RobotModel`] build: the tree walk
/// (`buildRecursive`), mimic resolution (`buildMimic`), and group
/// construction (`buildGroups`) all need the same link/joint index maps, so
/// they are methods on this rather than free functions threading the same
/// half-dozen parameters.
struct Building<'a> {
    srdf: &'a SrdfModel,
    children: HashMap<&'a str, Vec<&'a urdf_rs::Joint>>,
    joints: Vec<JointNode>,
    joint_index_by_name: HashMap<String, usize>,
    joint_names: Vec<String>,
    links: Vec<LinkModel>,
    link_index_by_name: HashMap<String, usize>,
    link_names: Vec<String>,
    diagnostics: Vec<Diagnostic>,
}

impl<'a> Building<'a> {
    /// Upstream `RobotModel::buildRecursive`.
    fn visit(
        &mut self,
        parent_link_index: Option<usize>,
        link_name: &str,
        mut model: JointModel,
        joint_origin: Isometry3,
    ) -> Result<()> {
        self.apply_joint_metadata(&mut model);

        let joint_index = self.joints.len();
        let first_variable_index = self
            .joints
            .last()
            .map_or(0, |j| j.first_variable_index + j.model.variable_count());
        let link_index = self.links.len();

        self.joint_index_by_name
            .insert(model.name().to_string(), joint_index);
        self.joint_names.push(model.name().to_string());
        self.link_index_by_name
            .insert(link_name.to_string(), link_index);
        self.link_names.push(link_name.to_string());

        self.links.push(LinkModel::new(
            link_name,
            link_index,
            joint_index,
            parent_link_index,
            joint_origin,
        ));
        self.joints.push(JointNode {
            model,
            first_variable_index,
            parent_link_index,
            child_link_index: link_index,
        });

        if let Some(parent_index) = parent_link_index {
            self.links[parent_index].add_child_joint_index(joint_index);
        }

        // Cloning this (small) Vec of `&Joint`s breaks the borrow of
        // `self.children` before the recursive call below needs `&mut self`.
        let children = self.children.get(link_name).cloned().unwrap_or_default();
        for child_joint in children {
            let child_model = joint_model_from_urdf(child_joint)?;
            let origin = isometry_from_urdf_pose(&child_joint.origin);
            self.visit(
                Some(link_index),
                &child_joint.child.link,
                child_model,
                origin,
            )?;
        }

        Ok(())
    }

    /// Upstream `RobotModel::constructJointModel`'s post-construction block:
    /// distance factor, passive flag, `<joint_property>` application.
    fn apply_joint_metadata(&mut self, model: &mut JointModel) {
        model.set_distance_factor(model.variable_count() as f64);

        let name = model.name().to_string();
        if self.srdf.passive_joints().iter().any(|p| p == &name) {
            model.set_passive(true);
        }

        for property in self.srdf.joint_properties_for(&name) {
            match property.property_name.as_str() {
                "angular_distance_weight" => match property.value.trim().parse::<f64>() {
                    Ok(weight) => match model.joint_type() {
                        JointType::Planar => model
                            .as_planar_mut()
                            .expect("just matched Planar")
                            .set_angular_distance_weight(weight),
                        JointType::Floating => model
                            .as_floating_mut()
                            .expect("just matched Floating")
                            .set_angular_distance_weight(weight),
                        _ => self.diagnostics.push(Diagnostic::JointPropertyWrongType {
                            joint: name.clone(),
                            property: property.property_name.clone(),
                            joint_type: model.type_name(),
                        }),
                    },
                    Err(_) => {
                        self.diagnostics
                            .push(Diagnostic::JointPropertyMalformedValue {
                                joint: name.clone(),
                                property: property.property_name.clone(),
                                value: property.value.clone(),
                            });
                    }
                },
                "motion_model" => {
                    if model.joint_type() != JointType::Planar {
                        self.diagnostics.push(Diagnostic::JointPropertyWrongType {
                            joint: name.clone(),
                            property: property.property_name.clone(),
                            joint_type: model.type_name(),
                        });
                        continue;
                    }
                    let motion_model = match property.value.as_str() {
                        "holonomic" => PlanarMotionModel::Holonomic,
                        "diff_drive" => PlanarMotionModel::DiffDrive,
                        _ => {
                            self.diagnostics
                                .push(Diagnostic::JointPropertyMalformedValue {
                                    joint: name.clone(),
                                    property: property.property_name.clone(),
                                    value: property.value.clone(),
                                });
                            continue;
                        }
                    };
                    model
                        .as_planar_mut()
                        .expect("just checked Planar")
                        .set_motion_model(motion_model);
                }
                "min_translational_distance" => {
                    if model.joint_type() != JointType::Planar {
                        self.diagnostics.push(Diagnostic::JointPropertyWrongType {
                            joint: name.clone(),
                            property: property.property_name.clone(),
                            joint_type: model.type_name(),
                        });
                        continue;
                    }
                    match property.value.trim().parse::<f64>() {
                        Ok(distance) => model
                            .as_planar_mut()
                            .expect("just checked Planar")
                            .set_min_translational_distance(distance),
                        Err(_) => {
                            self.diagnostics
                                .push(Diagnostic::JointPropertyMalformedValue {
                                    joint: name.clone(),
                                    property: property.property_name.clone(),
                                    value: property.value.clone(),
                                });
                        }
                    }
                }
                _ => self.diagnostics.push(Diagnostic::UnknownJointProperty {
                    joint: name.clone(),
                    property: property.property_name.clone(),
                }),
            }
        }
    }

    /// Upstream `RobotModel::buildMimic`.
    fn resolve_mimic(&mut self) {
        let variable_counts: Vec<usize> = self
            .joints
            .iter()
            .map(|j| j.model.variable_count())
            .collect();

        for i in 0..self.joints.len() {
            let joint_name = self.joints[i].model.name().to_string();
            let Some(mimic) = self.joints[i].model.mimic().cloned() else {
                continue;
            };
            match self.joint_index_by_name.get(&mimic.joint_name).copied() {
                None => {
                    self.diagnostics.push(Diagnostic::MimicUnknownJoint {
                        joint: joint_name,
                        mimicked: mimic.joint_name,
                    });
                    self.joints[i].model.clear_mimic();
                }
                Some(target) if variable_counts[target] != variable_counts[i] => {
                    self.diagnostics.push(Diagnostic::MimicDofMismatch {
                        joint: joint_name,
                        mimicked: mimic.joint_name,
                    });
                    self.joints[i].model.clear_mimic();
                }
                Some(_) => {}
            }
        }

        // Collapse mimic-of-a-mimic chains, and clear every mimic in the
        // model the instant one collapses into a self-cycle.
        loop {
            let mut changed = false;
            let mut cycle = false;
            for i in 0..self.joints.len() {
                let Some(mimic) = self.joints[i].model.mimic().cloned() else {
                    continue;
                };
                let Some(&target) = self.joint_index_by_name.get(&mimic.joint_name) else {
                    continue;
                };
                let Some(deeper) = self.joints[target].model.mimic().cloned() else {
                    continue;
                };
                let factor = mimic.factor * deeper.factor;
                let offset = mimic.offset + mimic.factor * deeper.offset;
                self.joints[i]
                    .model
                    .set_mimic(deeper.joint_name.clone(), factor, offset);
                changed = true;
                if deeper.joint_name == self.joints[i].model.name() {
                    cycle = true;
                    break;
                }
            }
            if cycle {
                self.diagnostics.push(Diagnostic::MimicCycle);
                for node in &mut self.joints {
                    node.model.clear_mimic();
                }
                break;
            }
            if !changed {
                break;
            }
        }
    }

    /// Upstream `RobotModel::addJointModelGroup`'s chain-expansion block: the
    /// joints on the walk from `tip_idx` up to `base_idx`, with the
    /// tip-and-base-both-descend-from-a-common-ancestor fallback for chains
    /// that do not directly nest (e.g. one end effector to another).
    fn expand_chain(&self, base_idx: usize, tip_idx: usize) -> BTreeSet<usize> {
        let mut chain_joints = Vec::new();
        let mut link = Some(tip_idx);
        while let Some(link_index) = link {
            if link_index == base_idx {
                break;
            }
            let joint_index = self.links[link_index].parent_joint_index();
            chain_joints.push(joint_index);
            link = self.joints[joint_index].parent_link_index;
        }

        if link == Some(base_idx) {
            return chain_joints.into_iter().collect();
        }

        let mut base_walk = Some(base_idx);
        let mut intersection_at = 0usize;
        let mut base_side_joints = Vec::new();
        while let Some(link_index) = base_walk {
            let joint_index = self.links[link_index].parent_joint_index();
            if let Some(position) = chain_joints.iter().position(|&j| j == joint_index) {
                intersection_at = position + 1;
                break;
            }
            base_side_joints.push(joint_index);
            base_walk = self.joints[joint_index].parent_link_index;
        }

        if intersection_at == 0 {
            return BTreeSet::new();
        }
        let mut joints: BTreeSet<usize> = chain_joints[..intersection_at].iter().copied().collect();
        joints.extend(base_side_joints);
        joints
    }

    /// Upstream `RobotModel::addJointModelGroup`'s joints/links/subgroups
    /// union, before the empty-group check.
    fn expand_group_joint_indices(
        &self,
        group: &Group,
        built: &HashMap<String, JointModelGroup>,
    ) -> BTreeSet<usize> {
        let mut joint_set = BTreeSet::new();

        for chain in &group.chains {
            if let (Some(&base_idx), Some(&tip_idx)) = (
                self.link_index_by_name.get(&chain.base_link),
                self.link_index_by_name.get(&chain.tip_link),
            ) {
                joint_set.extend(self.expand_chain(base_idx, tip_idx));
            }
        }

        for joint_name in &group.joints {
            if let Some(&index) = self.joint_index_by_name.get(joint_name) {
                joint_set.insert(index);
            }
        }

        for link_name in &group.links {
            if let Some(&link_index) = self.link_index_by_name.get(link_name) {
                joint_set.insert(self.links[link_index].parent_joint_index());
            }
        }

        for subgroup_name in &group.subgroups {
            if let Some(subgroup) = built.get(subgroup_name) {
                joint_set.extend(subgroup.joint_indices().iter().copied());
            }
        }

        joint_set
    }

    /// Upstream `JointModelGroup::JointModelGroup`'s classification pass
    /// (active/fixed/mimic joints, variables, member links). `joint_indices`
    /// is already depth-first order: it came from a [`BTreeSet`], and joint
    /// index order *is* depth-first order (see [`Building::visit`]).
    fn make_joint_model_group(
        &self,
        name: String,
        joint_indices: BTreeSet<usize>,
    ) -> JointModelGroup {
        let joint_indices: Vec<usize> = joint_indices.into_iter().collect();
        let joint_names: Vec<String> = joint_indices
            .iter()
            .map(|&i| self.joints[i].model.name().to_string())
            .collect();

        let mut active_joint_indices = Vec::new();
        let mut active_joint_names = Vec::new();
        let mut fixed_joint_indices = Vec::new();
        let mut mimic_joint_indices = Vec::new();
        let mut variable_names = Vec::new();
        let mut link_index_set = BTreeSet::new();

        for &i in &joint_indices {
            let node = &self.joints[i];
            link_index_set.insert(node.child_link_index);

            if node.model.variable_count() == 0 {
                fixed_joint_indices.push(i);
                continue;
            }
            if node.model.mimic().is_none() {
                active_joint_indices.push(i);
                active_joint_names.push(node.model.name().to_string());
            } else {
                mimic_joint_indices.push(i);
            }
            variable_names.extend(node.model.variable_names().iter().cloned());
        }

        let link_indices: Vec<usize> = link_index_set.into_iter().collect();
        let link_names: Vec<String> = link_indices
            .iter()
            .map(|&i| self.links[i].name().to_string())
            .collect();

        JointModelGroup {
            name,
            joint_indices,
            joint_names,
            active_joint_indices,
            active_joint_names,
            fixed_joint_indices,
            mimic_joint_indices,
            variable_names,
            link_indices,
            link_names,
            subgroup_names: Vec::new(),
        }
    }

    /// Upstream `RobotModel::buildGroups`: a fixpoint over the SRDF's group
    /// configs, since a group's subgroups may not be built yet on a given
    /// pass.
    fn build_groups(&mut self) -> BTreeMap<String, JointModelGroup> {
        let configs = self.srdf.groups();
        let mut processed = vec![false; configs.len()];
        let mut built: HashMap<String, JointModelGroup> = HashMap::new();

        loop {
            let mut added_any = false;
            for (i, group) in configs.iter().enumerate() {
                if processed[i] {
                    continue;
                }
                if group.subgroups.iter().any(|s| !built.contains_key(s)) {
                    continue;
                }
                processed[i] = true;
                added_any = true;

                if built.contains_key(&group.name) {
                    self.diagnostics.push(Diagnostic::DuplicateGroup {
                        group: group.name.clone(),
                    });
                    continue;
                }

                let joint_indices = self.expand_group_joint_indices(group, &built);
                if joint_indices.is_empty() {
                    self.diagnostics.push(Diagnostic::EmptyGroup {
                        group: group.name.clone(),
                    });
                    continue;
                }

                let jmg = self.make_joint_model_group(group.name.clone(), joint_indices);
                built.insert(group.name.clone(), jmg);
            }
            if !added_any {
                break;
            }
        }

        for (i, group) in configs.iter().enumerate() {
            if !processed[i] {
                self.diagnostics.push(Diagnostic::UnsatisfiedSubgroups {
                    group: group.name.clone(),
                });
            }
        }

        built.into_iter().collect()
    }
}

/// Upstream `RobotModel::buildJointInfo`'s variable-index and active-joint
/// bookkeeping (the fixed-transform caching in the same function is
/// collision-geometry-only and out of scope — see [`RobotModel`]'s doc
/// comment).
fn compute_variable_layout(
    joints: &[JointNode],
) -> (Vec<String>, HashMap<String, usize>, Vec<usize>) {
    let mut variable_names = Vec::new();
    let mut variable_index = HashMap::new();
    let mut active_joint_indices = Vec::new();

    for (i, node) in joints.iter().enumerate() {
        let names = node.model.variable_names();
        if names.is_empty() {
            continue;
        }
        for (j, name) in names.iter().enumerate() {
            variable_index.insert(name.clone(), node.first_variable_index + j);
        }
        variable_names.extend(names.iter().cloned());
        variable_index.insert(node.model.name().to_string(), node.first_variable_index);
        if node.model.mimic().is_none() {
            active_joint_indices.push(i);
        }
    }

    (variable_names, variable_index, active_joint_indices)
}

/// Upstream `RobotModel::buildGroupsInfoSubgroups`. Iterates groups (and, for
/// each, candidate subgroups) in alphabetical order, matching upstream's
/// `std::map<std::string, JointModelGroup*>` iteration order.
fn compute_subgroups(groups: &mut BTreeMap<String, JointModelGroup>) {
    let names: Vec<String> = groups.keys().cloned().collect();
    let joint_sets: HashMap<&str, HashSet<usize>> = groups
        .iter()
        .map(|(name, group)| {
            (
                name.as_str(),
                group.joint_indices().iter().copied().collect(),
            )
        })
        .collect();

    let mut subgroup_names_by_group: HashMap<String, Vec<String>> = HashMap::new();
    for name in &names {
        let this_set = &joint_sets[name.as_str()];
        let mut subgroup_names = Vec::new();
        for other in &names {
            if other == name {
                continue;
            }
            if joint_sets[other.as_str()].is_subset(this_set) {
                subgroup_names.push(other.clone());
            }
        }
        subgroup_names_by_group.insert(name.clone(), subgroup_names);
    }

    for (name, subgroup_names) in subgroup_names_by_group {
        if let Some(group) = groups.get_mut(&name) {
            group.subgroup_names = subgroup_names;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXED_BASE_SRDF: &str = r#"<robot name="test">
        <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    </robot>"#;

    fn build(urdf_xml: &str, srdf_xml: &str) -> Result<RobotModel> {
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &srdf)
    }

    fn revolute_joint(name: &str, parent: &str, child: &str, mimic: &str) -> String {
        format!(
            r#"<joint name="{name}" type="revolute">
                <parent link="{parent}"/>
                <child link="{child}"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
                {mimic}
            </joint>"#
        )
    }

    fn mimic_chain_urdf(j2_mimic: &str, j3_mimic: &str) -> String {
        format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="mid2"/>
                <link name="tip"/>
                {j1}
                {j2}
                {j3}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid", "mid2", j2_mimic),
            j3 = revolute_joint("j3", "mid2", "tip", j3_mimic),
        )
    }

    #[test]
    fn mimic_chain_collapses_transitively() {
        let urdf = mimic_chain_urdf(
            r#"<mimic joint="j1" multiplier="2.0" offset="0.5"/>"#,
            r#"<mimic joint="j2" multiplier="3.0" offset="0.1"/>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");
        assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

        let j3 = model.joint_model("j3").unwrap();
        let mimic = j3.mimic().expect("j3 still mimics after collapsing");
        assert_eq!(mimic.joint_name, "j1");
        assert_eq!(mimic.factor, 6.0);
        assert_eq!(mimic.offset, 1.6);
    }

    #[test]
    fn mimic_mutual_cycle_clears_every_mimic_in_the_model() {
        let urdf = mimic_chain_urdf(
            r#"<mimic joint="j3" multiplier="1.0" offset="0.0"/>"#,
            r#"<mimic joint="j2" multiplier="1.0" offset="0.0"/>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert!(matches!(model.diagnostics(), [Diagnostic::MimicCycle]));
        assert!(model.joint_model("j1").unwrap().mimic().is_none());
        assert!(model.joint_model("j2").unwrap().mimic().is_none());
        assert!(model.joint_model("j3").unwrap().mimic().is_none());
    }

    #[test]
    fn mimic_of_unknown_joint_is_dropped_with_a_diagnostic() {
        let urdf = mimic_chain_urdf(
            r#"<mimic joint="no_such_joint" multiplier="1" offset="0"/>"#,
            "",
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::MimicUnknownJoint {
                joint: "j2".to_string(),
                mimicked: "no_such_joint".to_string(),
            }]
        );
        assert!(model.joint_model("j2").unwrap().mimic().is_none());
    }

    #[test]
    fn mimic_with_mismatched_dof_is_dropped_with_a_diagnostic() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="tip"/>
                <joint name="j1" type="planar">
                    <parent link="base"/>
                    <child link="mid"/>
                </joint>
                {j2}
            </robot>"#,
            j2 = revolute_joint(
                "j2",
                "mid",
                "tip",
                r#"<mimic joint="j1" multiplier="1" offset="0"/>"#
            ),
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::MimicDofMismatch {
                joint: "j2".to_string(),
                mimicked: "j1".to_string(),
            }]
        );
        assert!(model.joint_model("j2").unwrap().mimic().is_none());
    }

    /// A chain whose `base_link` and `tip_link` are on different branches
    /// from a common ancestor (e.g. one end effector to another) cannot be
    /// walked directly from tip to base; `expand_chain`'s intersection
    /// fallback must find the shared ancestor joint instead of silently
    /// contributing nothing.
    #[test]
    fn chain_between_two_branches_finds_the_common_ancestor() {
        let urdf = r#"<robot name="test">
            <link name="root"/>
            <link name="a"/>
            <link name="b"/>
            <link name="tip_a"/>
            <link name="tip_b"/>
            <joint name="j_root_a" type="fixed">
                <parent link="root"/><child link="a"/>
            </joint>
            <joint name="j_root_b" type="fixed">
                <parent link="root"/><child link="b"/>
            </joint>
            <joint name="j_a_tip" type="fixed">
                <parent link="a"/><child link="tip_a"/>
            </joint>
            <joint name="j_b_tip" type="fixed">
                <parent link="b"/><child link="tip_b"/>
            </joint>
        </robot>"#;
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="root"/>
            <group name="cross">
                <chain base_link="tip_a" tip_link="tip_b"/>
            </group>
        </robot>"#;
        let model = build(urdf, srdf).expect("builds");

        let group = model.joint_model_group("cross").unwrap();
        let mut joints = group.joint_names().to_vec();
        joints.sort();
        assert_eq!(
            joints,
            ["fixed_base", "j_a_tip", "j_b_tip", "j_root_a", "j_root_b"]
        );
    }

    #[test]
    fn group_with_no_resolvable_joints_or_links_is_dropped() {
        let urdf = mimic_chain_urdf("", "");
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="empty">
                <joint name="no_such_joint"/>
                <link name="no_such_link"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::EmptyGroup {
                group: "empty".to_string(),
            }]
        );
        assert!(!model.has_joint_model_group("empty"));
    }

    /// `moveit_srdf::SrdfModel` already drops groups whose *subgroup name*
    /// does not resolve to another group in the file (its own
    /// `drop_groups_with_unsatisfied_subgroups`), so `RobotModel` never sees
    /// that case. The case that reaches `RobotModel::build_groups` is a
    /// subgroup name that *is* a real SRDF group, but resolves to no joints
    /// once checked against the URDF (something the SRDF parser cannot know,
    /// since it never sees the URDF) — that subgroup is dropped as
    /// `EmptyGroup`, and every group depending on it as a subgroup then fails
    /// to build in turn.
    #[test]
    fn group_with_a_subgroup_that_never_resolves_is_dropped() {
        let urdf = mimic_chain_urdf("", "");
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="leaf">
                <joint name="no_such_joint"/>
            </group>
            <group name="depends_on_missing">
                <group name="leaf"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [
                Diagnostic::EmptyGroup {
                    group: "leaf".to_string(),
                },
                Diagnostic::UnsatisfiedSubgroups {
                    group: "depends_on_missing".to_string(),
                },
            ]
        );
        assert!(!model.has_joint_model_group("leaf"));
        assert!(!model.has_joint_model_group("depends_on_missing"));
    }

    #[test]
    fn second_group_with_a_duplicate_name_is_dropped_and_the_first_kept() {
        let urdf = mimic_chain_urdf("", "");
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="g">
                <joint name="j1"/>
            </group>
            <group name="g">
                <joint name="j2"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::DuplicateGroup {
                group: "g".to_string(),
            }]
        );
        let group = model.joint_model_group("g").unwrap();
        assert_eq!(group.joint_names(), ["j1"]);
    }

    #[test]
    fn subgroup_detection_lists_every_strict_subset_alphabetically() {
        let urdf = mimic_chain_urdf("", "");
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="only_j1">
                <joint name="j1"/>
            </group>
            <group name="only_j2">
                <joint name="j2"/>
            </group>
            <group name="all">
                <joint name="j1"/>
                <joint name="j2"/>
                <joint name="j3"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        let all = model.joint_model_group("all").unwrap();
        assert_eq!(all.subgroup_names(), ["only_j1", "only_j2"]);
        assert!(
            model
                .joint_model_group("only_j1")
                .unwrap()
                .subgroup_names()
                .is_empty()
        );
    }

    #[test]
    fn no_root_link_errors() {
        let urdf = r#"<robot name="test">
            <link name="a"/>
            <link name="b"/>
            <joint name="j1" type="fixed">
                <parent link="a"/><child link="b"/>
            </joint>
            <joint name="j2" type="fixed">
                <parent link="b"/><child link="a"/>
            </joint>
        </robot>"#;
        assert!(build(urdf, FIXED_BASE_SRDF).is_err());
    }

    #[test]
    fn multiple_root_links_errors() {
        let urdf = r#"<robot name="test">
            <link name="a"/>
            <link name="b"/>
        </robot>"#;
        assert!(build(urdf, FIXED_BASE_SRDF).is_err());
    }
}
