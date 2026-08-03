// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from srdfdom 2.0.8 — the SRDF parser that moveit2 @
// e017c91ee12984393a28ba246075c65f69cde3bf depends on. PORTING-PLAN.md §2
// records that no SRDF crate exists on crates.io, so this is written from
// scratch against:
//   srdfdom/include/srdfdom/model.h   (struct layout, accessor names)
//   srdfdom/src/model.cpp             (parsing and validation semantics)

use std::collections::BTreeMap;
use std::fmt;
use std::path::Path;

use moveit_error::{Error, Result};

use crate::Diagnostic;

/// The `source_kind` every [`Error::Parse`] from this crate carries.
pub(crate) const SRDF: &str = "SRDF";

/// A planning group: a set of joints and their descendant links.
///
/// Upstream `srdf::Model::Group`. A group can be specified four ways, and a
/// single group may use any combination of them: directly named [`joints`],
/// directly named [`links`], [`chains`] of links, and [`subgroups`] naming
/// other groups. Resolving those four into a joint set needs the URDF and is
/// therefore `moveit-model`'s job, not this crate's.
///
/// [`joints`]: Group::joints
/// [`links`]: Group::links
/// [`chains`]: Group::chains
/// [`subgroups`]: Group::subgroups
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Group {
    /// The name of the group.
    pub name: String,
    /// Joints named directly by a `<joint name="..."/>` child.
    pub joints: Vec<String>,
    /// Links named directly by a `<link name="..."/>` child.
    pub links: Vec<String>,
    /// Chains named by a `<chain base_link="..." tip_link="..."/>` child.
    pub chains: Vec<Chain>,
    /// Names of other groups included wholesale by a `<group name="..."/>`
    /// child. Every name here is guaranteed to name a group in
    /// [`SrdfModel::groups`] — see [`Diagnostic::UnsatisfiedSubgroups`].
    pub subgroups: Vec<String>,
}

/// A kinematic chain, given as its two endpoint links.
///
/// Upstream stores this as a `std::pair<std::string, std::string>`; the fields
/// are named here because the pair's order (base, then tip) is not otherwise
/// visible at a call site.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Chain {
    /// The link the chain starts at.
    pub base_link: String,
    /// The link the chain ends at.
    pub tip_link: String,
}

/// A named pose for one group, as joint values.
///
/// Upstream `srdf::Model::GroupState`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GroupState {
    /// The name of the state.
    pub name: String,
    /// The group this state is defined for. Guaranteed to name a group in
    /// [`SrdfModel::groups`].
    pub group: String,
    /// Value per joint. The value is a vector because a multi-DOF joint takes
    /// several numbers from one whitespace-separated `value` attribute.
    ///
    /// A [`BTreeMap`] matches upstream's `std::map` iteration order, so
    /// anything that walks a state in order sees the same sequence.
    pub joint_values: BTreeMap<String, Vec<f64>>,
}

/// The type of a [`VirtualJoint`].
///
/// # Deviation from upstream
///
/// Upstream stores `type_` as a `std::string` that the parser has already
/// lower-cased and normalised, so every consumer re-compares string literals
/// and an unrecognised value is only distinguishable by having gone through the
/// parser. Modelling the closed set as an enum removes that: the parser is the
/// single place that maps text to a variant, and `moveit-model` matches instead
/// of comparing strings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VirtualJointType {
    /// `type="fixed"` — 0 DOF.
    Fixed,
    /// `type="planar"` — 3 DOF: x, y, yaw.
    Planar,
    /// `type="floating"` — 6 DOF.
    Floating,
}

impl VirtualJointType {
    /// The SRDF spelling of this type.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Fixed => "fixed",
            Self::Planar => "planar",
            Self::Floating => "floating",
        }
    }
}

impl fmt::Display for VirtualJointType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A joint that is not in the URDF, connecting the robot to an external frame.
///
/// Upstream `srdf::Model::VirtualJoint`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VirtualJoint {
    /// The name of the new joint.
    pub name: String,
    /// How many degrees of freedom the joint has, and of what kind.
    pub joint_type: VirtualJointType,
    /// The external frame the joint's parent side is attached to.
    pub parent_frame: String,
    /// The robot link the joint's child side is attached to.
    pub child_link: String,
}

/// An end effector.
///
/// Upstream `srdf::Model::EndEffector`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EndEffector {
    /// The name of the end effector.
    pub name: String,
    /// The link the end effector is attached to.
    pub parent_link: String,
    /// The group that contains [`parent_link`](EndEffector::parent_link), if
    /// the SRDF names one.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream leaves `parent_group_` as `""` when the attribute is absent,
    /// which collides with `parent_group=""` written explicitly. [`Option`]
    /// separates them.
    pub parent_group: Option<String>,
    /// The group holding the joints and links the end effector consists of.
    /// Guaranteed to name a group in [`SrdfModel::groups`].
    pub component_group: String,
}

/// One sphere of a link's sphere approximation.
///
/// Upstream `srdf::Model::Sphere`, whose centre is three separate `double`
/// members; an array is used here so the centre can be handed to a vector type
/// without rebuilding it component by component.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sphere {
    /// The centre of the sphere, in the link's collision frame.
    pub center: [f64; 3],
    /// The radius of the sphere.
    pub radius: f64,
}

/// The set of spheres bounding one link.
///
/// Upstream `srdf::Model::LinkSpheres`. See
/// [`SrdfModel::link_sphere_approximations`] for the radius-zero rules the
/// parser applies while filling this in.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkSpheres {
    /// The link these spheres approximate.
    pub link: String,
    /// The spheres. Never empty — a link whose spheres all get discarded is
    /// left out of the model entirely.
    pub spheres: Vec<Sphere>,
}

/// A pair of links whose collision check is explicitly disabled or re-enabled.
///
/// Upstream `srdf::Model::CollisionPair`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CollisionPair {
    /// The first link of the pair.
    pub link1: String,
    /// The second link of the pair.
    pub link2: String,
    /// Why the check was disabled or enabled. Empty when the SRDF gives no
    /// `reason`. Carried verbatim, without trimming, as upstream does.
    pub reason: String,
}

/// An extra property attached to a joint.
///
/// Upstream `srdf::Model::JointProperty`. The value is untyped text on both
/// sides; whoever defined the property defines how to read it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct JointProperty {
    /// The joint the property belongs to.
    pub joint_name: String,
    /// The name of the property.
    pub property_name: String,
    /// The value, verbatim and untrimmed, as upstream stores it.
    pub value: String,
}

/// The semantic description of a robot, parsed from an SRDF document.
///
/// Upstream `srdf::Model`. Build one with [`SrdfModel::parse_str`] or
/// [`SrdfModel::parse_file`].
///
/// # Deviations from upstream
///
/// 1. **No URDF is consulted.** Upstream's `srdf::Model::initXml` takes a
///    `urdf::ModelInterface` and silently drops every element naming a link or
///    joint the URDF does not have. This crate sits below `moveit-model` in the
///    dependency order (PORTING-PLAN.md §3) and has no URDF to check against,
///    so those checks move to `moveit-model`, which holds both descriptions.
///    Concretely, the following are **not** validated here and every such
///    element is retained: link names in groups, chains, `disable_collisions`,
///    `enable_collisions`, `disable_default_collisions`,
///    `link_sphere_approximation` and end effectors; joint names in groups,
///    group states, passive joints and joint properties; whether a chain's two
///    links really form a chain; and whether the robot name matches the URDF's.
///    An `SrdfModel` therefore describes *the document*, not a URDF-validated
///    robot.
///
///    Checks that are intrinsic to the SRDF *are* performed, exactly as
///    upstream performs them: a required attribute must be present, a group
///    state and an end effector must name a group the document defines, and a
///    group's subgroups must resolve.
///
/// 2. **Nothing is dropped silently.** Upstream reports every skipped element
///    to `console_bridge` and returns a model with no record of what went
///    missing, so a typo in an SRDF becomes a robot that quietly lacks a group.
///    Here every such decision is recorded in [`SrdfModel::diagnostics`]; the
///    element is still dropped, so the resulting model matches upstream's.
///
/// 3. **A malformed joint value drops the joint instead of becoming `0.0`.**
///    See [`SrdfModel::group_states`].
///
/// 4. **`name` is an [`Option`].** Upstream leaves `name_` as `""` for an SRDF
///    with no `name` attribute, which is the same value as `name=""`. The two
///    are distinguished here; the absent case also raises
///    [`Diagnostic::MissingRobotName`].
///
/// 5. **Passive joints are plain names.** Upstream wraps each in a
///    `PassiveJoint` struct whose only member is the name.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SrdfModel {
    pub(crate) name: Option<String>,
    pub(crate) groups: Vec<Group>,
    pub(crate) group_states: Vec<GroupState>,
    pub(crate) virtual_joints: Vec<VirtualJoint>,
    pub(crate) end_effectors: Vec<EndEffector>,
    pub(crate) link_sphere_approximations: Vec<LinkSpheres>,
    pub(crate) no_default_collision_links: Vec<String>,
    pub(crate) enabled_collision_pairs: Vec<CollisionPair>,
    pub(crate) disabled_collision_pairs: Vec<CollisionPair>,
    pub(crate) passive_joints: Vec<String>,
    pub(crate) joint_properties: BTreeMap<String, Vec<JointProperty>>,
    pub(crate) diagnostics: Vec<Diagnostic>,
}

impl SrdfModel {
    /// Parse an SRDF document held in memory.
    ///
    /// Upstream `srdf::Model::initString`.
    ///
    /// # Errors
    ///
    /// [`Error::Parse`] when the text is not well-formed XML, or when its root
    /// element is not `robot`. Every other problem is recoverable and lands in
    /// [`SrdfModel::diagnostics`].
    pub fn parse_str(xml: &str) -> Result<Self> {
        crate::parse::parse(xml)
    }

    /// Read and parse an SRDF file.
    ///
    /// Upstream `srdf::Model::initFile`.
    ///
    /// # Errors
    ///
    /// [`Error::Parse`] when the file cannot be read, or for the reasons
    /// [`SrdfModel::parse_str`] gives.
    pub fn parse_file(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let xml = std::fs::read_to_string(path).map_err(|e| Error::Parse {
            source_kind: SRDF,
            message: format!("could not read {}: {e}", path.display()),
        })?;
        Self::parse_str(&xml)
    }

    /// The robot this description is for, or [`None`] when the document's
    /// `robot` element carries no `name` attribute.
    ///
    /// Upstream `getName`, which reports the absent case as `""`.
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// The planning groups, in document order.
    ///
    /// Upstream `getGroups`. Every [`Group::subgroups`] entry names another
    /// group in this slice; groups whose subgroups do not resolve are dropped
    /// with a [`Diagnostic::UnsatisfiedSubgroups`].
    pub fn groups(&self) -> &[Group] {
        &self.groups
    }

    /// The named group poses, in document order.
    ///
    /// Upstream `getGroupStates`.
    ///
    /// # Deviation from upstream
    ///
    /// A `value` attribute that does not parse as whitespace-separated numbers
    /// drops that joint from the state and raises a
    /// [`Diagnostic::MalformedValue`]. Upstream reads the attribute with an
    /// `std::istringstream` whose failure leaves the extracted `double` at `0`
    /// and stores that `0` anyway, so `value="oops"` and `value="0"` produce
    /// the same state. Since `0.0` is a legal joint position, that failure is
    /// undetectable downstream: a typo in a named pose silently becomes a
    /// command to drive the joint to zero.
    pub fn group_states(&self) -> &[GroupState] {
        &self.group_states
    }

    /// The virtual joints, in document order.
    ///
    /// Upstream `getVirtualJoints`.
    pub fn virtual_joints(&self) -> &[VirtualJoint] {
        &self.virtual_joints
    }

    /// The end effectors, in document order.
    ///
    /// Upstream `getEndEffectors`.
    pub fn end_effectors(&self) -> &[EndEffector] {
        &self.end_effectors
    }

    /// The per-link sphere approximations, in document order.
    ///
    /// Upstream `getLinkSphereApproximations`. The upstream radius-zero rules
    /// are reproduced exactly, because `collision_distance_field` reads three
    /// distinct meanings out of the result:
    ///
    /// - A link with **no** entry here gets one sphere generated for it that
    ///   encloses its whole collision geometry.
    /// - A link whose spheres are **all** radius zero is left out of collision
    ///   checking; it is represented by a single sphere at the origin with
    ///   radius zero.
    /// - A link with **at least one** positive-radius sphere keeps only those;
    ///   the radius-zero ones are discarded.
    ///
    /// "Positive" means strictly greater than [`f64::EPSILON`], matching
    /// upstream's `std::numeric_limits<double>::epsilon()` comparison.
    pub fn link_sphere_approximations(&self) -> &[LinkSpheres] {
        &self.link_sphere_approximations
    }

    /// Links whose collisions are disabled by default, to be re-enabled
    /// selectively through [`SrdfModel::enabled_collision_pairs`].
    ///
    /// Upstream `getNoDefaultCollisionLinks`, filled from
    /// `<disable_default_collisions link="..."/>`.
    pub fn no_default_collision_links(&self) -> &[String] {
        &self.no_default_collision_links
    }

    /// Link pairs whose collision check is explicitly re-enabled after a
    /// default disabled it.
    ///
    /// Upstream `getEnabledCollisionPairs`, filled from `<enable_collisions/>`.
    pub fn enabled_collision_pairs(&self) -> &[CollisionPair] {
        &self.enabled_collision_pairs
    }

    /// Link pairs whose collision check is explicitly disabled.
    ///
    /// Upstream `getDisabledCollisionPairs`, filled from
    /// `<disable_collisions/>`.
    pub fn disabled_collision_pairs(&self) -> &[CollisionPair] {
        &self.disabled_collision_pairs
    }

    /// Joints marked as not actuated, in document order.
    ///
    /// Upstream `getPassiveJoints`, which returns `PassiveJoint` structs.
    ///
    /// Only `<passive_joint/>` elements that are direct children of `<robot>`
    /// count, matching upstream's `FirstChildElement`/`NextSiblingElement`
    /// walk. A `<passive_joint/>` nested inside a `<group>` — which the
    /// `panda_moveit_config` SRDF contains — is not a passive joint.
    pub fn passive_joints(&self) -> &[String] {
        &self.passive_joints
    }

    /// Every joint property, keyed by joint name.
    ///
    /// Upstream `getJointProperties`. A [`BTreeMap`] matches upstream's
    /// `std::map` iteration order.
    pub fn joint_properties(&self) -> &BTreeMap<String, Vec<JointProperty>> {
        &self.joint_properties
    }

    /// The properties defined for one joint; empty when it has none.
    ///
    /// Upstream `getJointProperties(const std::string&)`.
    pub fn joint_properties_for(&self, joint_name: &str) -> &[JointProperty] {
        self.joint_properties
            .get(joint_name)
            .map_or(&[][..], Vec::as_slice)
    }

    /// Everything the parser dropped or repaired, in the order it decided.
    ///
    /// Empty for a document the parser had no complaint about. Each entry
    /// corresponds to one upstream `console_bridge` report; see deviation 2 on
    /// [`SrdfModel`].
    pub fn diagnostics(&self) -> &[Diagnostic] {
        &self.diagnostics
    }
}
