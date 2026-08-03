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
use moveit_geometry::{Cuboid, Cylinder, Isometry3, Shape, Sphere, UnitQuaternion, Vector3};
use moveit_srdf::{Group, SrdfModel, VirtualJointType};
use nalgebra::Translation3;
use roxmltree::Document;

use crate::diagnostic::Diagnostic;
use crate::joint::{JointModel, JointType, PlanarMotionModel, joint_model_from_urdf};
#[cfg(test)]
use crate::joint_model_group::EndEffectorParent;
use crate::joint_model_group::JointModelGroup;
use crate::link_model::{LinkModel, LinkShape};

/// The `source_kind` every [`Error::Parse`] from this module carries.
const URDF: &str = "URDF";

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
/// 2. **No kinematics solver plumbing.** Upstream's `RobotModel` also
///    carries `group_kinematics_`; `PORTING-PLAN.md` puts kinematics
///    solvers in `moveit-kinematics`, a later phase. (Each link's own
///    collision/visual geometry, end-effector resolution and SRDF
///    `<group_state>` support *are* carried — see [`LinkModel`]'s doc
///    comment, [`RobotModel::get_end_effector`] and
///    [`JointModelGroup::default_state_names`] respectively for what each
///    does and does not cover.)
/// 3. **No `is_single_dof_`, no per-group `common_root_`, and no
///    precomputed `common_joint_roots_` table.**
///    [`RobotModel::get_common_root`],
///    [`JointModelGroup::is_chain`](crate::JointModelGroup::is_chain) and
///    [`JointModelGroup::joint_roots`](crate::JointModelGroup::joint_roots)
///    *are* carried — `moveit-state`'s `RobotState` dirty-subtree tracking
///    needs `getCommonRoot` to answer exactly what upstream's own does, not
///    a textbook LCA (`PORTING-PLAN.md` §8.2; see
///    [`RobotModel::get_common_root`]'s doc comment for a real upstream
///    quirk this reproduces on purpose), and `moveit-distance-field` needs
///    `joint_roots_` to compute `getUpdatedLinkModelNames` — but
///    [`RobotModel::get_common_root`] walks each joint's ancestor chain to
///    equal depth and then upward in lockstep (O(depth)) rather than
///    answering from upstream's precomputed `n`×`n` table (O(1)); nothing
///    in this port's scope calls it often enough to need the table.
///    `common_root_` (the per-group *cache* of one specific pair of
///    `get_common_root`'s answer, not the general query itself) remains
///    unported: nothing in this phase's done-criteria reads it.
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
    end_effector_group_names: HashMap<String, String>,
    diagnostics: Vec<Diagnostic>,
}

impl RobotModel {
    /// Build a model from a URDF and its matching SRDF.
    ///
    /// `urdf_xml` must be the same document `urdf` was parsed from: `urdf_rs`
    /// discards whether each `<joint>` had a `<limit>` element at all
    /// (`Joint::limit` is `#[serde(default)]`, not `Option`), which the joint
    /// layer's bounds computation needs to distinguish "no limit" from an
    /// explicit all-zero one (upstream tells them apart via a null
    /// `urdf_joint->limits` pointer). This is recovered here by reading the
    /// raw XML directly, see `joint_limit_presence`.
    ///
    /// Upstream `RobotModel::RobotModel`/`buildModel`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if the URDF has no link with no parent joint (no
    /// root), more than one such link, or a joint
    /// [`joint_model_from_urdf`] rejects (a `Spherical` joint).
    ///
    /// [`Error::Parse`] if `urdf_xml` is not well-formed XML.
    pub fn from_urdf_and_srdf(
        urdf: &urdf_rs::Robot,
        urdf_xml: &str,
        srdf: &SrdfModel,
    ) -> Result<Self> {
        let limit_presence = joint_limit_presence(urdf_xml)?;

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

        let links_by_name: HashMap<&str, &urdf_rs::Link> =
            urdf.links.iter().map(|l| (l.name.as_str(), l)).collect();

        let mut building = Building {
            srdf,
            children,
            links_by_name,
            limit_presence,
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
        let end_effector_group_names = build_end_effectors(&mut groups, srdf.end_effectors());
        building.build_group_states(&mut groups);

        let mut model = Self {
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
            end_effector_group_names,
            diagnostics: building.diagnostics,
        };
        model.compute_group_topology();
        Ok(model)
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

    /// `getCommonRoot`: `a` itself if `a == b`; the ancestor, if one of `a`/
    /// `b` is an ancestor of the other; otherwise upstream's actual answer
    /// for two joints that diverge below some ancestor — which, as detailed
    /// below, is *not* always their deepest common ancestor.
    ///
    /// # Deviation from upstream: reproduces a real upstream quirk, not a
    /// textbook LCA
    ///
    /// `RobotModel::computeCommonRootsHelper` (`robot_model.cpp`) precomputes
    /// `common_joint_roots_` by, at every link, pairing up that link's direct
    /// child joints `(ch[i], ch[j])` and writing the pairing joint as the
    /// common root for every element of `ch[i]->getDescendantJointModels()`
    /// crossed with every element of `ch[j]->getDescendantJointModels()`.
    /// `getDescendantJointModels()` excludes the joint itself
    /// (`computeDescendantsHelper` only inserts a joint into an *ancestor's*
    /// descendant set, never its own) — so the pairing loop never actually
    /// writes an entry for `(ch[i], ch[j])` itself, only for descendants
    /// *of* `ch[i]`/`ch[j]`. When `a` and `b` are themselves direct sibling
    /// joints (both immediate children of the same link, e.g. two joints
    /// both parented at `torso_lift_link`), no entry is ever written for
    /// that pair, and the zero-initialised table default —
    /// `joint_model_vector_[0]`, the model's global root joint — is what
    /// upstream actually returns. Any deeper divergence (either side a
    /// proper descendant, not the immediate child, of the branching link)
    /// *is* covered by the pairing loop and returns the true common
    /// ancestor.
    ///
    /// This was found empirically against the real oracle on PR2 — the
    /// exact case the porting task calls out ("PR2, whose two arms branch
    /// from `torso_lift_link`"): `getCommonRoot(l_shoulder_pan_joint,
    /// r_shoulder_pan_joint)` (both direct children of `torso_lift_link`)
    /// returns `world_joint`, not `torso_lift_joint`, and the same pattern
    /// reproduces for other direct-sibling pairs (`fl_caster_rotation_joint`
    /// / `fr_caster_rotation_joint`, both direct children of `base_link`).
    /// A pair where one side is a deeper descendant (e.g.
    /// `fl_caster_rotation_joint` / `l_shoulder_pan_joint`) correctly
    /// returns their true common ancestor (`base_footprint_joint`). Since
    /// `moveit-state`'s dirty-subtree tracking already depends on matching
    /// upstream's actual (over-conservative, in the sibling case) marking —
    /// not a "more correct" LCA that would under-mark upstream's own
    /// behaviour — this reproduces the quirk rather than fixing it.
    ///
    /// Upstream answers in O(1) from that precomputed table; this walks
    /// each joint's ancestor chain to equal depth and then upward in
    /// lockstep instead (O(depth), not O(1)) — see [`RobotModel`]'s doc
    /// comment, deviation 3, for why — tracking, at the final convergence
    /// step, whether the two joints about to be found equal are themselves
    /// the original `a`/`b` (the direct-sibling case) or deeper descendants
    /// (the normal case).
    ///
    /// Takes plain joint indices, not upstream's possibly-null pointers:
    /// upstream's `getCommonRoot(nullptr, b)` returning `b` (and vice versa)
    /// exists only because C++ pointers can be null. A caller here tracking
    /// "no joint yet" represents that as `Option<usize>` at its own call
    /// site instead (see `moveit-state`'s `mark_dirty`), so this method
    /// never needs to.
    pub fn get_common_root(&self, a: usize, b: usize) -> usize {
        let depth = |mut joint_index: usize| -> usize {
            let mut depth = 0;
            while let Some(parent) = self.parent_joint_index(joint_index) {
                joint_index = parent;
                depth += 1;
            }
            depth
        };

        let (mut a_walk, mut b_walk) = (a, b);
        let (mut depth_a, mut depth_b) = (depth(a), depth(b));
        while depth_a > depth_b {
            a_walk = self
                .parent_joint_index(a_walk)
                .expect("depth_a > depth_b implies a_walk is not yet the root");
            depth_a -= 1;
        }
        while depth_b > depth_a {
            b_walk = self
                .parent_joint_index(b_walk)
                .expect("depth_b > depth_a implies b_walk is not yet the root");
            depth_b -= 1;
        }
        if a_walk == b_walk {
            // `a == b`, or one is an ancestor of the other: upstream's
            // second table-fill pass sets these correctly regardless of the
            // sibling quirk above, so the true ancestor is always right
            // here.
            return a_walk;
        }
        loop {
            let parent_a = self
                .parent_joint_index(a_walk)
                .expect("a_walk and b_walk differ, so a global root above them exists");
            let parent_b = self
                .parent_joint_index(b_walk)
                .expect("a_walk and b_walk differ, so a global root above them exists");
            if parent_a == parent_b {
                if a_walk == a && b_walk == b {
                    // `a` and `b` are themselves direct siblings under
                    // `parent_a` — the case upstream's table never writes.
                    // Its default (the global root) survives; find it by
                    // continuing the same walk to the top.
                    let mut root = parent_a;
                    while let Some(parent) = self.parent_joint_index(root) {
                        root = parent;
                    }
                    return root;
                }
                return parent_a;
            }
            a_walk = parent_a;
            b_walk = parent_b;
        }
    }

    /// `JointModel::getDescendantLinkModels`, computed here rather than on
    /// [`crate::joint::JointModel`] for the same reason
    /// [`RobotModel::get_common_root`] lives here and not on `JointModel`:
    /// this port's `JointModel` deliberately excludes tree bookkeeping (see
    /// that type's doc comment) -- `RobotModel` is the "later phase" it
    /// defers to.
    ///
    /// Every link reachable from `joint_index`'s own child link, by
    /// repeatedly following either a link's child joints or a joint's mimic
    /// followers (upstream's `computeDescendantsHelper` recurses into both
    /// `LinkModel::getChildJointModels` and `JointModel::getMimicRequests`),
    /// including `joint_index`'s own child link.
    ///
    /// Returned as a [`BTreeSet`], sorted by link index -- the same order
    /// `OrderLinksByIndex` gives upstream's own `updated_link_model_vector_`
    /// (a `std::set`-deduplicated union of several of these, explicitly
    /// `std::sort`ed afterwards). Upstream's *own*
    /// `descendant_link_models_` is DFS-insertion-ordered instead, but
    /// nothing in this port ever needs that raw order: every caller
    /// ([`JointModelGroup`]'s `updated_link_*` construction) re-sorts by
    /// index immediately after unioning several of these together, so
    /// sorting once, here, gives byte-identical results more simply.
    pub fn descendant_link_indices(&self, joint_index: usize) -> BTreeSet<usize> {
        let mut seen_joints = BTreeSet::new();
        let mut links = BTreeSet::new();
        let mut stack = vec![joint_index];
        while let Some(current) = stack.pop() {
            if !seen_joints.insert(current) {
                continue;
            }
            let child_link = self.joints[current].child_link_index;
            links.insert(child_link);
            let link = &self.links[child_link];
            stack.extend(link.child_joint_indices().iter().copied());
            stack.extend(self.joints_mimicking(current));
        }
        links
    }

    /// `JointModel::getMimicRequests`: every joint whose own `mimic_` points
    /// at `joint_index`. Upstream stores this reverse mapping directly on
    /// `JointModel` (`addMimicRequest`, populated once during
    /// `RobotModel::buildMimic`); this port's `JointModel` carries no tree
    /// state (see its doc comment), so it is recovered here instead by
    /// scanning -- cheap enough since [`RobotModel::descendant_link_indices`]
    /// only calls it once per joint on its own traversal stack, not in a hot
    /// loop.
    fn joints_mimicking(&self, joint_index: usize) -> impl Iterator<Item = usize> + '_ {
        let name = self.joint_names[joint_index].as_str();
        self.joints.iter().enumerate().filter_map(move |(i, node)| {
            node.model
                .mimic()
                .is_some_and(|m| m.joint_name == name)
                .then_some(i)
        })
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

    /// `hasEndEffector`: whether `name` is a known end effector's own name
    /// (an SRDF `<end_effector name="...">`, not a group name).
    pub fn has_end_effector(&self, name: &str) -> bool {
        self.end_effector_group_names.contains_key(name)
    }

    /// `getEndEffector`: the group backing the end effector named `name`,
    /// falling back to treating `name` as a group name if that group is
    /// itself an end effector.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is neither a known end-effector name
    /// nor the name of a group for which
    /// [`JointModelGroup::is_end_effector`] is true.
    pub fn get_end_effector(&self, name: &str) -> Result<&JointModelGroup> {
        if let Some(group_name) = self.end_effector_group_names.get(name) {
            return Ok(self
                .groups
                .get(group_name)
                .expect("end_effector_group_names must reference a real group"));
        }
        self.groups
            .get(name)
            .filter(|group| group.is_end_effector())
            .ok_or_else(|| Error::unknown_name("end effector", name))
    }

    /// `getEndEffectors`: every group that is an end effector. Upstream
    /// sorts `end_effectors_` explicitly (`OrderGroupsByName`) after building
    /// it; here `groups` is already a [`BTreeMap`], so iterating it in group-
    /// name order comes for free.
    pub fn end_effectors(&self) -> impl Iterator<Item = &JointModelGroup> {
        self.groups.values().filter(|group| group.is_end_effector())
    }

    /// Everything [`RobotModel::from_urdf_and_srdf`] dropped or repaired
    /// while building this model.
    pub fn diagnostics(&self) -> &[Diagnostic] {
        &self.diagnostics
    }

    /// Sets [`JointModelGroup::joint_roots`] and, from the same roots list,
    /// [`JointModelGroup::is_chain`] on every group. Run once, after every
    /// other field of `self` (in particular `joints`/`links`, which the
    /// ancestor walks below need) is already in place.
    ///
    /// Collects into an owned `Vec` first rather than mutating `self.groups`
    /// while iterating it: `group_joint_roots`/`group_is_chain` need `&self`
    /// (the ancestor walks), which the borrow checker won't allow
    /// interleaved with a `&mut` borrow of `self.groups` from the same
    /// `self`.
    fn compute_group_topology(&mut self) {
        let topology: Vec<(String, Vec<usize>, bool)> = self
            .groups
            .iter()
            .map(|(name, group)| {
                let roots = self.group_joint_roots(group);
                let chain = self.group_is_chain(group, &roots);
                (name.clone(), roots, chain)
            })
            .collect();
        for (name, roots, chain) in topology {
            let group = self
                .groups
                .get_mut(&name)
                .expect("name came from self.groups.iter()");
            group.set_joint_roots(roots);
            group.set_is_chain(chain);
        }
    }

    /// `JointModelGroup`'s own `joint_roots_`: every active joint in `group`
    /// whose ancestor chain never passes through another active, non-mimic
    /// member of `group` (`includesParent`, ported as
    /// [`RobotModel::ancestor_is_group_member`]) — the roots of the group's
    /// (possibly several) distinct kinematic subtrees.
    fn group_joint_roots(&self, group: &JointModelGroup) -> Vec<usize> {
        group
            .active_joint_indices()
            .iter()
            .copied()
            .filter(|&joint_index| !self.ancestor_is_group_member(joint_index, group))
            .collect()
    }

    /// Upstream's `JointModelGroup` constructor: `is_chain_` is true iff the
    /// group has exactly one joint root (`joint_roots_.size() == 1`), has at
    /// least one active joint, and — walking the group's own joints in
    /// depth-first order, from the last to the first — each is directly
    /// preceded by the one before it (`jointPrecedes`, skipping any run of
    /// fixed joints in between). `roots` is `group`'s own
    /// [`RobotModel::group_joint_roots`] result, passed in rather than
    /// recomputed, since [`RobotModel::compute_group_topology`] already has
    /// it on hand for both facts.
    fn group_is_chain(&self, group: &JointModelGroup, roots: &[usize]) -> bool {
        if group.active_joint_indices().is_empty() {
            return false;
        }
        if roots.len() != 1 {
            return false;
        }

        let joints = group.joint_indices();
        (1..joints.len())
            .rev()
            .all(|k| self.joint_precedes(joints[k], joints[k - 1]))
    }

    /// `includesParent`: whether `joint_index`'s ancestor chain — in the
    /// full model, not just `group` — passes through another active,
    /// non-mimic joint that is also a member of `group`, either directly or
    /// (for a mimic ancestor) via the joint it mimics.
    ///
    /// The mimic branch is not the full recursion upstream's C++ writes:
    /// `resolve_mimic` already collapses mimic-of-a-mimic chains to a
    /// fixpoint before any group is built, so the mimicked joint reached
    /// here is always itself non-mimic, and checking its own ancestors once
    /// (the recursive call) is everything upstream's version could ever
    /// find beyond the direct check.
    fn ancestor_is_group_member(&self, joint_index: usize, group: &JointModelGroup) -> bool {
        let mut current = joint_index;
        while let Some(parent) = self.parent_joint_index(current) {
            let parent_model = self.joint_model_at(parent);
            if group.has_joint_model(&self.joint_names[parent])
                && !parent_model.variable_names().is_empty()
                && parent_model.mimic().is_none()
            {
                return true;
            }
            if let Some(mimic) = parent_model.mimic() {
                if let Some(&mimicked_index) = self.joint_index_by_name.get(&mimic.joint_name) {
                    let mimicked_model = self.joint_model_at(mimicked_index);
                    let mimicked_is_direct_member = group.has_joint_model(&mimic.joint_name)
                        && !mimicked_model.variable_names().is_empty()
                        && mimicked_model.mimic().is_none();
                    if mimicked_is_direct_member
                        || self.ancestor_is_group_member(mimicked_index, group)
                    {
                        return true;
                    }
                }
            }
            current = parent;
        }
        false
    }

    /// `jointPrecedes`: whether `a` sits immediately below `b` in the
    /// kinematic chain, tolerating any run of fixed joints in between.
    fn joint_precedes(&self, a: usize, b: usize) -> bool {
        let Some(mut p) = self.parent_joint_index(a) else {
            return false;
        };
        loop {
            if p == b {
                return true;
            }
            if self.joint_model_at(p).joint_type() != JointType::Fixed {
                return false;
            }
            match self.parent_joint_index(p) {
                Some(next) => p = next,
                None => return false,
            }
        }
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

/// [`construct_shape`]'s result: either the [`Shape`] upstream's
/// `constructShape` would have built, or the kind of geometry it named that
/// this port cannot build one for.
enum ShapeOrUnsupported {
    Shape(Shape),
    Unsupported(&'static str),
}

/// Upstream `RobotModel::constructShape`. See [`crate::link_model::LinkModel`]'s
/// doc comment, deviation 4, for why `<mesh>` and `<capsule>` (a urdf-rs
/// extension upstream's own URDF parser does not recognise) are unsupported
/// rather than built.
///
/// # Errors
///
/// [`Error::Construct`] if a `<box>`/`<cylinder>`/`<sphere>` dimension is
/// negative.
fn construct_shape(geometry: &urdf_rs::Geometry) -> Result<ShapeOrUnsupported> {
    Ok(match geometry {
        urdf_rs::Geometry::Sphere { radius } => {
            ShapeOrUnsupported::Shape(Shape::Sphere(Sphere::new(*radius)?))
        }
        urdf_rs::Geometry::Box { size } => {
            let [x, y, z] = size.0;
            ShapeOrUnsupported::Shape(Shape::Cuboid(Cuboid::new(x, y, z)?))
        }
        urdf_rs::Geometry::Cylinder { radius, length } => {
            ShapeOrUnsupported::Shape(Shape::Cylinder(Cylinder::new(*radius, *length)?))
        }
        urdf_rs::Geometry::Mesh { .. } => ShapeOrUnsupported::Unsupported("mesh"),
        urdf_rs::Geometry::Capsule { .. } => ShapeOrUnsupported::Unsupported("capsule"),
    })
}

/// Which of the raw URDF's `<joint>` elements have a `<limit>` child, keyed
/// by joint name.
///
/// `urdf_rs::Joint::limit` is `#[serde(default)]`, not `Option`, so a missing
/// `<limit>` element and an explicit all-zero one deserialize identically;
/// upstream's `jointBoundsFromURDF` tells them apart with a null
/// `urdf_joint->limits` pointer. Reading the raw XML directly — the same
/// approach `moveit_srdf` takes to parse the SRDF — recovers the
/// distinction for `joint::urdf::joint_bounds_from_urdf` to use.
fn joint_limit_presence(urdf_xml: &str) -> Result<HashMap<String, bool>> {
    let doc = Document::parse(urdf_xml).map_err(|e| Error::Parse {
        source_kind: URDF,
        message: e.to_string(),
    })?;
    Ok(doc
        .root_element()
        .children()
        .filter(|n| n.has_tag_name("joint"))
        .filter_map(|joint| {
            joint.attribute("name").map(|name| {
                let has_limit = joint.children().any(|c| c.has_tag_name("limit"));
                (name.to_string(), has_limit)
            })
        })
        .collect())
}

/// The in-progress state of a [`RobotModel`] build: the tree walk
/// (`buildRecursive`), mimic resolution (`buildMimic`), and group
/// construction (`buildGroups`) all need the same link/joint index maps, so
/// they are methods on this rather than free functions threading the same
/// half-dozen parameters.
struct Building<'a> {
    srdf: &'a SrdfModel,
    children: HashMap<&'a str, Vec<&'a urdf_rs::Joint>>,
    links_by_name: HashMap<&'a str, &'a urdf_rs::Link>,
    limit_presence: HashMap<String, bool>,
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

        if let Some(&urdf_link) = self.links_by_name.get(link_name) {
            self.apply_link_geometry(link_index, urdf_link)?;
        }

        // Cloning this (small) Vec of `&Joint`s breaks the borrow of
        // `self.children` before the recursive call below needs `&mut self`.
        let children = self.children.get(link_name).cloned().unwrap_or_default();
        for child_joint in children {
            let limit_present = self
                .limit_presence
                .get(&child_joint.name)
                .copied()
                .unwrap_or(false);
            let child_model = joint_model_from_urdf(child_joint, limit_present)?;
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

    /// Upstream `RobotModel::constructLinkModel`: this link's collision
    /// shapes (every `<collision>` element, upstream's `col_array`) and,
    /// separately, its visual mesh metadata (tried from the first
    /// `<visual>` element first, the first `<collision>` element second —
    /// plain filename/origin/scale, not a loaded mesh; see `LinkModel`'s
    /// doc comment, deviation 4, for why the two differ in what they need).
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if a `<collision>` shape's dimensions are
    /// negative (upstream constructs the shape unconditionally; this port's
    /// [`Shape`] constructors validate).
    fn apply_link_geometry(&mut self, link_index: usize, urdf_link: &urdf_rs::Link) -> Result<()> {
        let mut shapes = Vec::new();
        for collision in &urdf_link.collision {
            let origin_transform = isometry_from_urdf_pose(&collision.origin);
            match construct_shape(&collision.geometry)? {
                ShapeOrUnsupported::Shape(shape) => shapes.push(LinkShape {
                    shape,
                    origin_transform,
                }),
                ShapeOrUnsupported::Unsupported(kind) => {
                    self.diagnostics.push(Diagnostic::UnsupportedLinkGeometry {
                        link: urdf_link.name.clone(),
                        kind,
                    });
                }
            }
        }
        self.links[link_index].set_geometry(shapes);

        let inertial = &urdf_link.inertial;
        let inertial_origin = isometry_from_urdf_pose(&inertial.origin);
        let center_of_mass = inertial_origin.translation.vector;
        let ixx = inertial.inertia.ixx;
        let ixy = inertial.inertia.ixy;
        let ixz = inertial.inertia.ixz;
        let iyy = inertial.inertia.iyy;
        let iyz = inertial.inertia.iyz;
        let izz = inertial.inertia.izz;
        #[rustfmt::skip]
        let inertia_in_inertial_frame = nalgebra::Matrix3::new(
            ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz,
        );
        let rotation = inertial_origin.rotation.to_rotation_matrix();
        let inertia_in_link_frame =
            rotation.matrix() * inertia_in_inertial_frame * rotation.matrix().transpose();
        self.links[link_index].set_inertial(
            inertial.mass.value,
            center_of_mass,
            inertia_in_link_frame,
        );

        let visual_mesh = urdf_link
            .visual
            .first()
            .map(|v| (&v.geometry, &v.origin))
            .filter(|(geometry, _)| matches!(geometry, urdf_rs::Geometry::Mesh { .. }))
            .or_else(|| {
                urdf_link
                    .collision
                    .first()
                    .map(|c| (&c.geometry, &c.origin))
                    .filter(|(geometry, _)| matches!(geometry, urdf_rs::Geometry::Mesh { .. }))
            });
        if let Some((urdf_rs::Geometry::Mesh { filename, scale }, origin)) = visual_mesh {
            if !filename.is_empty() {
                let scale = scale.map_or(Vector3::new(1.0, 1.0, 1.0), |s| {
                    Vector3::new(s.0[0], s.0[1], s.0[2])
                });
                self.links[link_index].set_visual_mesh(
                    filename.clone(),
                    isometry_from_urdf_pose(origin),
                    scale,
                );
            }
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
            end_effector_name: None,
            end_effector_parent: None,
            attached_end_effector_names: Vec::new(),
            default_state_names: Vec::new(),
            default_states: HashMap::new(),
            is_chain: false,
            joint_roots: Vec::new(),
        }
    }

    /// Upstream `RobotModel::buildGroupStates`. Iterates the SRDF's group
    /// states in document order and, for each, its `joint_values` in the
    /// `BTreeMap` order [`moveit_srdf::GroupState`] already stores them in
    /// (alphabetical by joint name, matching upstream's `std::map`
    /// iteration).
    ///
    /// # Deviation from upstream
    ///
    /// Upstream reports three recoverable problems through its logger and
    /// otherwise proceeds unconditionally: a group state naming a group this
    /// model does not have, a `<joint>` value naming a joint that is not
    /// part of the named group, and a joint whose supplied value count does
    /// not match its own variable count. `moveit-srdf` already guarantees
    /// `GroupState::group` names a group the *SRDF document* defines, but
    /// not that the group survived `RobotModel`'s own build (an SRDF group
    /// can still be dropped for being empty, a duplicate, or having
    /// unsatisfied subgroups) or that a joint name is real. This reproduces
    /// upstream's silent-recovery stance for all three rather than adding a
    /// `Diagnostic` variant, matching the precedent [`RobotModel`]'s doc
    /// comment sets in deviation 6 — pr2's `tuck_arms` state is a real,
    /// oracle-verified example of the "missing joints" case (upstream logs
    /// `RCLCPP_WARN` for it and still stores the partial state).
    fn build_group_states(&self, groups: &mut BTreeMap<String, JointModelGroup>) {
        for group_state in self.srdf.group_states() {
            let Some(group) = groups.get(&group_state.group) else {
                continue;
            };

            let mut remaining_active: HashSet<&str> = group
                .active_joint_names()
                .iter()
                .map(String::as_str)
                .collect();
            let mut state: BTreeMap<String, f64> = BTreeMap::new();

            for (joint_name, values) in &group_state.joint_values {
                if !group.has_joint_model(joint_name) {
                    continue;
                }
                remaining_active.remove(joint_name.as_str());

                let Some(&joint_index) = self.joint_index_by_name.get(joint_name) else {
                    continue;
                };
                let variable_names = self.joints[joint_index].model.variable_names();
                if variable_names.len() != values.len() {
                    continue;
                }
                for (name, &value) in variable_names.iter().zip(values) {
                    state.insert(name.clone(), value);
                }
            }

            if !state.is_empty() {
                groups
                    .get_mut(&group_state.group)
                    .expect("just confirmed present above")
                    .add_default_state(group_state.name.clone(), state);
            }
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

/// Upstream `RobotModel::buildGroupsInfoEndEffectors`. Iterates groups (and,
/// for each, candidate parent groups) in alphabetical order, matching
/// upstream's `std::map<std::string, JointModelGroup*>` iteration order; the
/// SRDF's own end-effector list is walked in the document order
/// [`moveit_srdf::SrdfModel::end_effectors`] returns, matching upstream's
/// `std::vector`. Returns a map from each wired end effector's own name to
/// the name of the group it marks, matching `end_effectors_map_`.
///
/// # Deviation from upstream
///
/// Upstream reports three failures through its logger and otherwise falls
/// through to its non-error behaviour rather than stopping: an
/// `eef.parent_group_` naming a real group that does not contain the parent
/// link, one naming the end effector's own group, and (a `RCLCPP_WARN`, not
/// an error) no parent group being identifiable at all
/// (`possible_parent_groups` empty with no usable explicit parent either) —
/// in every case resolution proceeds exactly as if no `parent_group` had
/// been given. None of the four fixtures' `<end_effector>` elements reaches
/// the last case, or gives an invalid `parent_group` for the first two to
/// matter: panda's `parent_group` is explicit and valid; pr2's two eefs
/// carry no `parent_group` attribute at all and each always has at least
/// one candidate; fanuc has no `<end_effector>`; dual_arm_panda's two are
/// both dropped by `moveit-srdf` before reaching this function
/// (`Diagnostic::UnknownGroup`, since their `component_group`s don't name
/// real groups). This port therefore adds no `Diagnostic` for the silent
/// fallback, matching the precedent [`RobotModel`]'s doc comment sets in
/// deviation 6.
fn build_end_effectors(
    groups: &mut BTreeMap<String, JointModelGroup>,
    eefs: &[moveit_srdf::EndEffector],
) -> HashMap<String, String> {
    let group_names: Vec<String> = groups.keys().cloned().collect();
    // Owned rather than borrowed from `groups`, so `groups.get_mut` below is
    // not blocked by a live immutable borrow of the whole map.
    let joint_counts: HashMap<String, usize> = groups
        .iter()
        .map(|(name, group)| (name.clone(), group.joint_indices().len()))
        .collect();
    let link_membership: HashMap<String, HashSet<String>> = groups
        .iter()
        .map(|(name, group)| (name.clone(), group.link_names().iter().cloned().collect()))
        .collect();

    let mut end_effector_group_names = HashMap::new();

    for group_name in &group_names {
        for eef in eefs {
            if eef.component_group != *group_name {
                continue;
            }

            let mut possible_parent_groups: Vec<String> = Vec::new();
            for other_name in &group_names {
                if other_name == group_name {
                    continue;
                }
                if link_membership[other_name].contains(&eef.parent_link) {
                    groups
                        .get_mut(other_name)
                        .expect("other_name came from groups.keys()")
                        .attach_end_effector(eef.name.clone());
                    possible_parent_groups.push(other_name.clone());
                }
            }

            let explicit_parent = eef.parent_group.as_ref().filter(|parent_group| {
                *parent_group != group_name
                    && link_membership
                        .get(parent_group.as_str())
                        .is_some_and(|links| links.contains(&eef.parent_link))
            });

            let parent_group_name = explicit_parent.cloned().or_else(|| {
                possible_parent_groups
                    .iter()
                    .min_by_key(|name| joint_counts[name.as_str()])
                    .cloned()
            });

            let group = groups
                .get_mut(group_name)
                .expect("group_name came from groups.keys()");
            group.set_end_effector_name(eef.name.clone());
            group.set_end_effector_parent(parent_group_name, eef.parent_link.clone());
            end_effector_group_names.insert(eef.name.clone(), group_name.clone());
        }
    }

    end_effector_group_names
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
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf)
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

    /// The presence-detection boundary `panda`/`fanuc` never exercise: a
    /// `<joint>` with no `<limit>` element at all must build unbounded, not
    /// clamped to `[0, 0]` the way an explicit all-zero `<limit>` is.
    /// `urdf_rs::Joint::limit` deserializes identically in both cases
    /// (`#[serde(default)]`), so this specifically exercises
    /// `joint_limit_presence`'s raw-XML read, not just `joint_bounds_from_urdf`
    /// in isolation.
    ///
    /// Prismatic, not revolute: a non-continuous revolute joint's
    /// `position_bounded` is unconditionally forced `true` by
    /// `JointModel::set_continuous(false)` regardless of `<limit>` presence
    /// (matches upstream's `RevoluteJointModel::setContinuous`), so it can't
    /// show this distinction; prismatic joints never call `set_continuous`.
    #[test]
    fn joint_with_no_limit_element_is_unbounded_not_a_zero_width_bound() {
        let urdf = r#"<robot name="test">
            <link name="base"/>
            <link name="mid"/>
            <link name="tip"/>
            <joint name="with_limit" type="prismatic">
                <parent link="base"/>
                <child link="mid"/>
                <axis xyz="0 0 1"/>
                <limit lower="0" upper="0" effort="0" velocity="0"/>
            </joint>
            <joint name="without_limit" type="prismatic">
                <parent link="mid"/>
                <child link="tip"/>
                <axis xyz="0 0 1"/>
            </joint>
        </robot>"#;
        let model = build(urdf, FIXED_BASE_SRDF).expect("builds");

        let with_limit = model.joint_model("with_limit").unwrap().variable_bounds()[0];
        let without_limit = model
            .joint_model("without_limit")
            .unwrap()
            .variable_bounds()[0];

        assert!(with_limit.position_bounded);
        assert_eq!(with_limit.min_position, 0.0);
        assert_eq!(with_limit.max_position, 0.0);
        assert!(!with_limit.velocity_bounded);

        assert!(!without_limit.position_bounded);
        assert!(!without_limit.velocity_bounded);

        assert_ne!(with_limit, without_limit);
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

    fn link_with_geometry_urdf(link_extra: &str) -> String {
        format!(
            r#"<robot name="test">
                <link name="base">{link_extra}</link>
            </robot>"#
        )
    }

    #[test]
    fn box_collision_at_identity_produces_a_shape_and_a_centered_bounding_box() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><box size="2 4 6"/></geometry></collision>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");
        assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

        let base = model.link_model("base").unwrap();
        assert_eq!(
            base.shapes(),
            [LinkShape {
                shape: Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).unwrap()),
                origin_transform: Isometry3::identity(),
            }]
        );
        assert_eq!(base.centered_bounding_box_offset(), Vector3::zeros());
    }

    /// The boundary several real `pr2` links exercise: no `<collision>`
    /// element at all. The oracle reports `centered_bounding_box_offset:
    /// [0.0, 0.0, 0.0]` for these — an exact zero from Eigen's empty-box
    /// constant, not `NaN` — see `Aabb`'s doc comment.
    #[test]
    fn link_with_no_collision_has_no_shapes_and_a_zero_bounding_box_center() {
        let urdf = link_with_geometry_urdf("");
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        let base = model.link_model("base").unwrap();
        assert!(base.shapes().is_empty());
        assert_eq!(base.centered_bounding_box_offset(), Vector3::zeros());
    }

    #[test]
    fn mesh_collision_is_skipped_with_a_diagnostic_and_leaves_no_shape() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><mesh filename="package://x/foo.stl"/></geometry></collision>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::UnsupportedLinkGeometry {
                link: "base".to_string(),
                kind: "mesh",
            }]
        );
        assert!(model.link_model("base").unwrap().shapes().is_empty());
    }

    #[test]
    fn capsule_collision_is_skipped_with_a_diagnostic() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><capsule radius="0.1" length="0.4"/></geometry></collision>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::UnsupportedLinkGeometry {
                link: "base".to_string(),
                kind: "capsule",
            }]
        );
    }

    /// A link with both a valid and an unsupported `<collision>` element
    /// keeps the valid shape and diagnoses only the unsupported one — the
    /// two do not desync into "shapes.len() != collision element count".
    #[test]
    fn one_unsupported_collision_element_does_not_drop_the_others() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><sphere radius="1"/></geometry></collision>
               <collision><geometry><mesh filename="foo.stl"/></geometry></collision>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.diagnostics(),
            [Diagnostic::UnsupportedLinkGeometry {
                link: "base".to_string(),
                kind: "mesh",
            }]
        );
        assert_eq!(
            model.link_model("base").unwrap().shapes(),
            [LinkShape {
                shape: Shape::Sphere(Sphere::new(1.0).unwrap()),
                origin_transform: Isometry3::identity(),
            }]
        );
    }

    #[test]
    fn negative_shape_dimension_errors_the_whole_build() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><box size="-1 1 1"/></geometry></collision>"#,
        );
        assert!(build(&urdf, FIXED_BASE_SRDF).is_err());
    }

    #[test]
    fn visual_mesh_prefers_the_first_visual_mesh_over_collision() {
        let urdf = link_with_geometry_urdf(concat!(
            r#"<visual><geometry><mesh filename="visual.dae" scale="2 2 2"/></geometry></visual>"#,
            r#"<collision><geometry><mesh filename="collision.stl"/></geometry></collision>"#,
        ));
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        let base = model.link_model("base").unwrap();
        assert_eq!(base.visual_mesh_filename(), Some("visual.dae"));
        assert_eq!(base.visual_mesh_scale(), Vector3::new(2.0, 2.0, 2.0));
    }

    /// When the first `<visual>` element isn't a mesh, upstream falls back
    /// to the first `<collision>` mesh rather than leaving
    /// `visual_mesh_filename_` empty.
    #[test]
    fn visual_mesh_falls_back_to_collision_when_first_visual_is_not_a_mesh() {
        let urdf = link_with_geometry_urdf(concat!(
            r#"<visual><geometry><box size="1 1 1"/></geometry></visual>"#,
            r#"<collision><geometry><mesh filename="collision.stl"/></geometry></collision>"#,
        ));
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        let base = model.link_model("base").unwrap();
        assert_eq!(base.visual_mesh_filename(), Some("collision.stl"));
    }

    #[test]
    fn no_mesh_in_visual_or_collision_leaves_visual_mesh_filename_none() {
        let urdf = link_with_geometry_urdf(
            r#"<collision><geometry><box size="1 1 1"/></geometry></collision>"#,
        );
        let model = build(&urdf, FIXED_BASE_SRDF).expect("builds");

        assert_eq!(
            model.link_model("base").unwrap().visual_mesh_filename(),
            None
        );
    }

    /// A four-joint chain (`j1`..`j4`, the last fixed) plus a branch off
    /// `base` (`j5`), shaped so an end effector's `parent_link` (`link2`)
    /// has exactly two candidate parent groups of different sizes (`arm`:
    /// `j1`,`j2`; `full_arm`: `j1`,`j2`,`j3`) and one group that shares
    /// neither the link nor the eef's own name (`other`: `j5`).
    fn end_effector_test_urdf() -> &'static str {
        r#"<robot name="test">
            <link name="base"/>
            <link name="link1"/>
            <link name="link2"/>
            <link name="link3"/>
            <link name="hand_link"/>
            <link name="other_link"/>
            <joint name="j1" type="revolute">
                <parent link="base"/><child link="link1"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="j2" type="revolute">
                <parent link="link1"/><child link="link2"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="j3" type="revolute">
                <parent link="link2"/><child link="link3"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="j4" type="fixed">
                <parent link="link3"/><child link="hand_link"/>
            </joint>
            <joint name="j5" type="fixed">
                <parent link="base"/><child link="other_link"/>
            </joint>
        </robot>"#
    }

    /// `end_effector_element` is embedded verbatim as a child of `<robot>`,
    /// alongside `arm` (`j1`,`j2`), `full_arm` (`j1`,`j2`,`j3`), `hand`
    /// (`j4`) and `other` (`j5`) groups.
    fn end_effector_test_srdf(end_effector_element: &str) -> String {
        format!(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
                <group name="arm"><joint name="j1"/><joint name="j2"/></group>
                <group name="full_arm"><joint name="j1"/><joint name="j2"/><joint name="j3"/></group>
                <group name="hand"><joint name="j4"/></group>
                <group name="other"><joint name="j5"/></group>
                {end_effector_element}
            </robot>"#
        )
    }

    #[test]
    fn end_effector_wires_name_and_falls_back_to_fewest_joints_parent() {
        let srdf = end_effector_test_srdf(
            r#"<end_effector name="grasper" parent_link="link2" group="hand"/>"#,
        );
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        let hand = model.joint_model_group("hand").unwrap();
        assert!(hand.is_end_effector());
        assert_eq!(hand.end_effector_name(), "grasper");
        assert_eq!(
            hand.end_effector_parent(),
            Some(&EndEffectorParent {
                group: Some("arm".to_string()),
                link: "link2".to_string(),
            })
        );

        assert_eq!(
            model
                .joint_model_group("arm")
                .unwrap()
                .attached_end_effector_names(),
            ["grasper"]
        );
        assert_eq!(
            model
                .joint_model_group("full_arm")
                .unwrap()
                .attached_end_effector_names(),
            ["grasper"]
        );
        assert!(
            model
                .joint_model_group("other")
                .unwrap()
                .attached_end_effector_names()
                .is_empty()
        );

        assert_eq!(model.get_end_effector("grasper").unwrap().name(), "hand");
        // `hasEndEffector`/`getEndEffector` are asymmetric upstream: only
        // `getEndEffector` falls back to treating the argument as a group
        // name.
        assert!(model.has_end_effector("grasper"));
        assert!(!model.has_end_effector("hand"));
        assert_eq!(model.get_end_effector("hand").unwrap().name(), "hand");
        assert!(model.get_end_effector("arm").is_err());

        let names: Vec<&str> = model.end_effectors().map(JointModelGroup::name).collect();
        assert_eq!(names, ["hand"]);
    }

    #[test]
    fn end_effector_prefers_explicit_valid_parent_group_over_fewest_joints_fallback() {
        let srdf = end_effector_test_srdf(
            r#"<end_effector name="grasper" parent_link="link2" group="hand" parent_group="full_arm"/>"#,
        );
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        assert_eq!(
            model
                .joint_model_group("hand")
                .unwrap()
                .end_effector_parent(),
            Some(&EndEffectorParent {
                group: Some("full_arm".to_string()),
                link: "link2".to_string(),
            })
        );
    }

    #[test]
    fn end_effector_explicit_parent_naming_its_own_group_is_ignored() {
        let srdf = end_effector_test_srdf(
            r#"<end_effector name="grasper" parent_link="link2" group="hand" parent_group="hand"/>"#,
        );
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        assert_eq!(
            model
                .joint_model_group("hand")
                .unwrap()
                .end_effector_parent(),
            Some(&EndEffectorParent {
                group: Some("arm".to_string()),
                link: "link2".to_string(),
            })
        );
    }

    #[test]
    fn end_effector_explicit_parent_lacking_the_link_is_ignored() {
        let srdf = end_effector_test_srdf(
            r#"<end_effector name="grasper" parent_link="link2" group="hand" parent_group="other"/>"#,
        );
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        assert_eq!(
            model
                .joint_model_group("hand")
                .unwrap()
                .end_effector_parent(),
            Some(&EndEffectorParent {
                group: Some("arm".to_string()),
                link: "link2".to_string(),
            })
        );
    }

    #[test]
    fn end_effector_with_no_candidate_parent_group_has_no_group_but_keeps_the_link() {
        let srdf = end_effector_test_srdf(
            r#"<end_effector name="grasper" parent_link="hand_link" group="hand"/>"#,
        );
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        assert_eq!(
            model
                .joint_model_group("hand")
                .unwrap()
                .end_effector_parent(),
            Some(&EndEffectorParent {
                group: None,
                link: "hand_link".to_string(),
            })
        );
    }

    #[test]
    fn non_end_effector_group_reports_false_and_empty_defaults() {
        let srdf = end_effector_test_srdf("");
        let model = build(end_effector_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        assert!(!arm.is_end_effector());
        assert_eq!(arm.end_effector_name(), "");
        assert_eq!(arm.end_effector_parent(), None);
        assert!(arm.attached_end_effector_names().is_empty());
        assert_eq!(model.end_effectors().count(), 0);
    }

    /// A two-joint chain (`j1`,`j2`), a third joint (`j3`) outside the
    /// `arm` group, and `arm` = `{j1, j2}`.
    fn group_state_test_urdf() -> &'static str {
        r#"<robot name="test">
            <link name="base"/>
            <link name="mid"/>
            <link name="tip"/>
            <link name="other_tip"/>
            <joint name="j1" type="revolute">
                <parent link="base"/><child link="mid"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="j2" type="revolute">
                <parent link="mid"/><child link="tip"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
            <joint name="j3" type="revolute">
                <parent link="base"/><child link="other_tip"/><axis xyz="0 0 1"/>
                <limit lower="-1" upper="1" effort="1" velocity="1"/>
            </joint>
        </robot>"#
    }

    /// `group_state_element` is embedded verbatim as a child of `<robot>`,
    /// alongside the `arm` group (`j1`,`j2`).
    fn group_state_test_srdf(group_state_element: &str) -> String {
        format!(
            r#"<robot name="test">
                <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
                <group name="arm"><joint name="j1"/><joint name="j2"/></group>
                {group_state_element}
            </robot>"#
        )
    }

    #[test]
    fn group_state_with_full_coverage_stores_every_variable() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="home" group="arm">
                <joint name="j1" value="0.1"/>
                <joint name="j2" value="0.2"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        assert_eq!(arm.default_state_names(), ["home"]);
        let state = arm.variable_default_positions("home").unwrap();
        assert_eq!(state.len(), 2);
        assert_eq!(state["j1"], 0.1);
        assert_eq!(state["j2"], 0.2);
    }

    /// The real pr2 `tuck_arms` shape: a `<group_state>` that leaves some of
    /// the group's active joints unspecified. Upstream still stores the
    /// partial state (`RCLCPP_WARN`, not an error).
    #[test]
    fn group_state_missing_a_joint_still_stores_the_partial_state() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="partial" group="arm">
                <joint name="j1" value="0.1"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        let state = arm.variable_default_positions("partial").unwrap();
        assert_eq!(state.len(), 1);
        assert_eq!(state["j1"], 0.1);
    }

    #[test]
    fn group_state_value_for_joint_outside_the_group_is_ignored() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="outside" group="arm">
                <joint name="j1" value="0.1"/>
                <joint name="j3" value="0.5"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        let state = arm.variable_default_positions("outside").unwrap();
        assert_eq!(state.len(), 1);
        assert_eq!(state["j1"], 0.1);
    }

    #[test]
    fn group_state_naming_an_unknown_group_is_ignored() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="ghost" group="no_such_group">
                <joint name="j1" value="0.1"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        assert!(
            model
                .joint_model_group("arm")
                .unwrap()
                .default_state_names()
                .is_empty()
        );
    }

    #[test]
    fn group_state_with_mismatched_variable_count_drops_that_joint_only() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="mismatch" group="arm">
                <joint name="j1" value="0.1 0.2"/>
                <joint name="j2" value="0.2"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        let state = arm.variable_default_positions("mismatch").unwrap();
        assert_eq!(state.len(), 1);
        assert_eq!(state["j2"], 0.2);
    }

    #[test]
    fn group_state_where_every_joint_value_is_unusable_stores_no_state_at_all() {
        let srdf = group_state_test_srdf(
            r#"<group_state name="empty" group="arm">
                <joint name="j1" value="0.1 0.2"/>
            </group_state>"#,
        );
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        assert!(arm.default_state_names().is_empty());
        assert!(arm.variable_default_positions("empty").is_none());
    }

    #[test]
    fn variable_default_positions_returns_none_for_unknown_state_name() {
        let srdf = group_state_test_srdf("");
        let model = build(group_state_test_urdf(), &srdf).expect("builds");

        let arm = model.joint_model_group("arm").unwrap();
        assert!(arm.default_state_names().is_empty());
        assert!(arm.variable_default_positions("anything").is_none());
    }

    /// `jointPrecedes` skips over any run of *unlisted* fixed joints between
    /// two group members — the group here names only `j1`/`j2`, not the
    /// `mid_fixed` joint sitting between them, so `is_chain` must still see
    /// `j2` as directly preceded by `j1`.
    #[test]
    fn is_chain_true_across_an_unlisted_fixed_joint() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="mid2"/>
                <link name="tip"/>
                {j1}
                <joint name="mid_fixed" type="fixed">
                    <parent link="mid"/><child link="mid2"/>
                </joint>
                {j2}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid2", "tip", ""),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="arm">
                <joint name="j1"/>
                <joint name="j2"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert!(model.joint_model_group("arm").unwrap().is_chain());
    }

    /// A single active joint with two children that are *both* group
    /// members has exactly one root (`j1`), so the root-count condition
    /// alone would call this a chain — but `joint_indices`' depth-first
    /// order can't have every consecutive pair satisfy `jointPrecedes`
    /// when two of them are siblings, so `is_chain` must still be false.
    #[test]
    fn is_chain_false_for_a_branch_with_a_single_root() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="tip_a"/>
                <link name="tip_b"/>
                {j1}
                {j2}
                {j3}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid", "tip_a", ""),
            j3 = revolute_joint("j3", "mid", "tip_b", ""),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="branch">
                <joint name="j1"/>
                <joint name="j2"/>
                <joint name="j3"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert!(!model.joint_model_group("branch").unwrap().is_chain());
    }

    /// A group whose active joints have two *distinct* roots (neither `j2`
    /// nor `j3` has an ancestor inside the group) is the more direct
    /// `root_count != 1` boundary, as opposed to
    /// `is_chain_false_for_a_branch_with_a_single_root`'s single-root-but-
    /// unordered case above.
    #[test]
    fn is_chain_false_for_two_independently_rooted_joints() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="tip_a"/>
                <link name="tip_b"/>
                {j1}
                {j2}
                {j3}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid", "tip_a", ""),
            j3 = revolute_joint("j3", "mid", "tip_b", ""),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="branch_tips">
                <joint name="j2"/>
                <joint name="j3"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        assert!(!model.joint_model_group("branch_tips").unwrap().is_chain());
    }

    /// `RobotModel::get_common_root` on a minimal synthetic tree with a
    /// planar virtual root joint, covering every invariant boundary the
    /// porting task calls out: same joint, ancestor in each direction, and
    /// — the case a textbook LCA gets wrong — two joints that are
    /// themselves direct siblings (`branch_a`/`branch_b`, both parented at
    /// `trunk`). Per upstream's actual `computeCommonRootsHelper` (see
    /// `get_common_root`'s doc comment), that pair's common root is the
    /// *global* root joint (`planar_root`), not `stem` (`trunk`'s creating
    /// joint and their true nearest common ancestor) — this is the "pair
    /// spanning the planar joint at the root" case, realized here because
    /// PR2's own real topology turns out not to branch until below its
    /// (fixed, not planar) virtual joint.
    #[test]
    fn get_common_root_covers_same_ancestor_and_sibling_quirk_boundaries() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="root"/>
                <link name="trunk"/>
                <link name="tip_a"/>
                <link name="tip_b"/>
                {stem}
                {branch_a}
                {branch_b}
            </robot>"#,
            stem = revolute_joint("stem", "root", "trunk", ""),
            branch_a = revolute_joint("branch_a", "trunk", "tip_a", ""),
            branch_b = revolute_joint("branch_b", "trunk", "tip_b", ""),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="planar_root" type="planar" parent_frame="odom" child_link="root"/>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        let idx = |name: &str| -> usize {
            model
                .joint_names()
                .iter()
                .position(|n| n == name)
                .unwrap_or_else(|| panic!("no joint named '{name}'"))
        };

        assert_eq!(
            model.joint_names()[model.get_common_root(idx("branch_a"), idx("branch_a"))],
            "branch_a",
            "same joint"
        );
        assert_eq!(
            model.joint_names()[model.get_common_root(idx("stem"), idx("branch_a"))],
            "stem",
            "ancestor first"
        );
        assert_eq!(
            model.joint_names()[model.get_common_root(idx("branch_a"), idx("stem"))],
            "stem",
            "ancestor second"
        );
        assert_eq!(
            model.joint_names()[model.get_common_root(idx("planar_root"), idx("branch_b"))],
            "planar_root",
            "pair spanning the planar root joint"
        );
        assert_eq!(
            model.joint_names()[model.get_common_root(idx("branch_a"), idx("branch_b"))],
            "planar_root",
            "direct siblings under trunk resolve to the global root, not stem"
        );
    }

    /// The same tree as `is_chain_false_for_two_independently_rooted_joints`
    /// (`j2`/`j3` are siblings under `mid`, neither an ancestor of the
    /// other), but asserting `joint_roots()` itself rather than the
    /// `is_chain` boolean it feeds: both must be listed, in the group's own
    /// active-joint order.
    #[test]
    fn joint_roots_lists_every_root_of_a_multi_rooted_group() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="tip_a"/>
                <link name="tip_b"/>
                {j1}
                {j2}
                {j3}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid", "tip_a", ""),
            j3 = revolute_joint("j3", "mid", "tip_b", ""),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
            <group name="branch_tips">
                <joint name="j2"/>
                <joint name="j3"/>
            </group>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        let idx = |name: &str| -> usize {
            model
                .joint_names()
                .iter()
                .position(|n| n == name)
                .unwrap_or_else(|| panic!("no joint named '{name}'"))
        };

        let group = model.joint_model_group("branch_tips").unwrap();
        assert_eq!(group.joint_roots(), [idx("j2"), idx("j3")]);
    }

    /// Upstream's `computeDescendantsHelper` recurses into a joint's mimic
    /// followers as well as its own child joints
    /// (`JointModel::getMimicRequests`): when `j2` moves, `j3` (which mimics
    /// it) moves too, so `j3`'s own subtree counts as a descendant of `j2`
    /// even though `j3` is not itself a descendant *joint* of `j2` in the
    /// tree — they are siblings, both parented at `mid`.
    #[test]
    fn descendant_link_indices_follows_mimic_followers() {
        let urdf = format!(
            r#"<robot name="test">
                <link name="base"/>
                <link name="mid"/>
                <link name="tip_a"/>
                <link name="tip_b"/>
                {j1}
                {j2}
                {j3}
            </robot>"#,
            j1 = revolute_joint("j1", "base", "mid", ""),
            j2 = revolute_joint("j2", "mid", "tip_a", ""),
            j3 = revolute_joint(
                "j3",
                "mid",
                "tip_b",
                r#"<mimic joint="j2" multiplier="1.0" offset="0.0"/>"#,
            ),
        );
        let srdf = r#"<robot name="test">
            <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
        </robot>"#;
        let model = build(&urdf, srdf).expect("builds");

        let idx = |name: &str| -> usize {
            model
                .joint_names()
                .iter()
                .position(|n| n == name)
                .unwrap_or_else(|| panic!("no joint named '{name}'"))
        };
        let link_idx = |name: &str| -> usize {
            model
                .link_names()
                .iter()
                .position(|n| n == name)
                .unwrap_or_else(|| panic!("no link named '{name}'"))
        };

        assert_eq!(
            model.descendant_link_indices(idx("j2")),
            [link_idx("tip_a"), link_idx("tip_b")].into_iter().collect(),
            "j2's own child plus j3's, since j3 mimics j2"
        );

        assert_eq!(
            model.descendant_link_indices(idx("j1")),
            [link_idx("mid"), link_idx("tip_a"), link_idx("tip_b")]
                .into_iter()
                .collect(),
            "everything below j1 either way, mimic-follower or not"
        );
    }
}
