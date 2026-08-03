// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from srdfdom 2.0.8 — the SRDF parser that moveit2 @
// e017c91ee12984393a28ba246075c65f69cde3bf depends on:
//   srdfdom/src/model.cpp
//
// The load order below is upstream's `initXml` order. It is load-bearing in one
// place — groups must be complete, including the subgroup pass that drops
// unsatisfiable ones, before group states and end effectors can check the group
// they name — and is kept everywhere else so diagnostics come out in the same
// sequence as upstream's console output.

use std::collections::{BTreeMap, BTreeSet};

use moveit_error::{Error, Result};
use roxmltree::{Document, Node};

use crate::model::SRDF;
use crate::{
    Chain, CollisionPair, Diagnostic, EndEffector, Group, GroupState, JointProperty, LinkSpheres,
    Sphere, SrdfModel, VirtualJoint, VirtualJointType,
};

pub(crate) fn parse(xml: &str) -> Result<SrdfModel> {
    let doc = Document::parse(xml).map_err(|e| Error::Parse {
        source_kind: SRDF,
        message: e.to_string(),
    })?;

    // Upstream reaches the robot element with `XMLDocument::FirstChildElement
    // ("robot")` and errors out when that is null. XML admits exactly one root
    // element, so checking its name is the same test.
    let robot = doc.root_element();
    if !robot.has_tag_name("robot") {
        return Err(Error::Parse {
            source_kind: SRDF,
            message: format!(
                "expected a `robot` root element, found `{}`",
                robot.tag_name().name()
            ),
        });
    }

    let mut p = Parser::default();
    p.load_robot_name(robot);
    p.load_virtual_joints(robot);
    p.load_groups(robot);
    p.load_group_states(robot);
    p.load_end_effectors(robot);
    p.load_link_sphere_approximations(robot);
    p.load_collision_defaults(robot);
    p.load_collision_pairs(robot, "enable_collisions", CollisionKind::Enabled);
    p.load_collision_pairs(robot, "disable_collisions", CollisionKind::Disabled);
    p.load_passive_joints(robot);
    p.load_joint_properties(robot);
    Ok(p.model)
}

#[derive(Default)]
struct Parser {
    model: SrdfModel,
}

enum CollisionKind {
    Enabled,
    Disabled,
}

impl Parser {
    fn warn(&mut self, diagnostic: Diagnostic) {
        self.model.diagnostics.push(diagnostic);
    }

    /// Read a required attribute, recording a [`Diagnostic::MissingAttribute`]
    /// and returning [`None`] when it is absent.
    ///
    /// The value is returned untrimmed; each caller trims exactly where
    /// upstream trims, which is not everywhere (`reason` on a collision pair
    /// and `value` on a joint property are stored verbatim).
    fn required<'a>(
        &mut self,
        node: Node<'a, '_>,
        element: &'static str,
        attribute: &'static str,
        context: Option<String>,
    ) -> Option<&'a str> {
        let value = node.attribute(attribute);
        if value.is_none() {
            self.warn(Diagnostic::MissingAttribute {
                element,
                attribute,
                context,
            });
        }
        value
    }

    fn load_robot_name(&mut self, robot: Node<'_, '_>) {
        match robot.attribute("name") {
            Some(name) => self.model.name = Some(name.trim_ascii().to_owned()),
            None => self.warn(Diagnostic::MissingRobotName),
        }
    }

    fn load_virtual_joints(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "virtual_joint";
        for vj in children(robot, ELEMENT) {
            // Upstream checks name, child_link, parent_frame and type in this
            // order and stops at the first one missing, so a virtual joint
            // missing two attributes reports only the first.
            let Some(name) = self.required(vj, ELEMENT, "name", None) else {
                continue;
            };
            let name = name.trim_ascii().to_owned();
            let Some(child_link) = self.required(vj, ELEMENT, "child_link", Some(name.clone()))
            else {
                continue;
            };
            let Some(parent_frame) = self.required(vj, ELEMENT, "parent_frame", Some(name.clone()))
            else {
                continue;
            };
            let Some(raw_type) = self.required(vj, ELEMENT, "type", Some(name.clone())) else {
                continue;
            };

            let normalized = raw_type.trim_ascii().to_ascii_lowercase();
            let joint_type = match normalized.as_str() {
                "fixed" => VirtualJointType::Fixed,
                "planar" => VirtualJointType::Planar,
                "floating" => VirtualJointType::Floating,
                // Upstream keeps the joint and calls it fixed rather than
                // dropping it; a dropped virtual joint would detach the robot
                // from the world frame, which is a larger failure than assuming
                // it is bolted down.
                _ => {
                    self.warn(Diagnostic::UnknownVirtualJointType {
                        joint: name.clone(),
                        raw: raw_type.to_owned(),
                    });
                    VirtualJointType::Fixed
                }
            };

            self.model.virtual_joints.push(VirtualJoint {
                name,
                joint_type,
                parent_frame: parent_frame.trim_ascii().to_owned(),
                child_link: child_link.trim_ascii().to_owned(),
            });
        }
    }

    fn load_groups(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "group";
        for group_xml in children(robot, ELEMENT) {
            let Some(name) = self.required(group_xml, ELEMENT, "name", None) else {
                continue;
            };
            let name = name.trim_ascii().to_owned();
            let context = || Some(format!("group {name:?}"));
            let mut group = Group {
                name: name.clone(),
                ..Group::default()
            };

            for link in children(group_xml, "link") {
                if let Some(name) = self.required(link, "link", "name", context()) {
                    group.links.push(name.trim_ascii().to_owned());
                }
            }
            for joint in children(group_xml, "joint") {
                if let Some(name) = self.required(joint, "joint", "name", context()) {
                    group.joints.push(name.trim_ascii().to_owned());
                }
            }
            for chain in children(group_xml, "chain") {
                let Some(base_link) = self.required(chain, "chain", "base_link", context()) else {
                    continue;
                };
                let Some(tip_link) = self.required(chain, "chain", "tip_link", context()) else {
                    continue;
                };
                group.chains.push(Chain {
                    base_link: base_link.trim_ascii().to_owned(),
                    tip_link: tip_link.trim_ascii().to_owned(),
                });
            }
            // A nested `<group>` is a reference to another group, never a
            // definition: only direct children of `<robot>` define groups.
            for subgroup in children(group_xml, "group") {
                if let Some(name) = self.required(subgroup, "group", "name", context()) {
                    group.subgroups.push(name.trim_ascii().to_owned());
                }
            }

            if group.links.is_empty()
                && group.joints.is_empty()
                && group.chains.is_empty()
                && group.subgroups.is_empty()
            {
                self.warn(Diagnostic::EmptyGroup { group: name });
            }
            self.model.groups.push(group);
        }

        self.drop_groups_with_unsatisfied_subgroups();
    }

    /// Drop every group that cannot be resolved to a subgroup-free definition.
    ///
    /// Grow the set of resolvable groups until it stops growing: a group joins
    /// once all of its subgroups are already in. Anything left out is either
    /// naming a group that does not exist or is part of a subgroup cycle, and
    /// upstream drops both. Running to a fixpoint rather than in document order
    /// is what makes a forward reference — a group listing a subgroup defined
    /// later in the file — legal.
    fn drop_groups_with_unsatisfied_subgroups(&mut self) {
        let mut resolvable: BTreeSet<String> = BTreeSet::new();
        let mut grew = true;
        while grew {
            grew = false;
            for group in &self.model.groups {
                if resolvable.contains(&group.name) {
                    continue;
                }
                if group.subgroups.iter().all(|s| resolvable.contains(s)) {
                    resolvable.insert(group.name.clone());
                    grew = true;
                }
            }
        }

        let mut dropped = Vec::new();
        self.model.groups.retain(|group| {
            let keep = resolvable.contains(&group.name);
            if !keep {
                dropped.push(group.name.clone());
            }
            keep
        });
        for group in dropped {
            self.warn(Diagnostic::UnsatisfiedSubgroups { group });
        }
    }

    fn load_group_states(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "group_state";
        for state_xml in children(robot, ELEMENT) {
            let Some(name) = self.required(state_xml, ELEMENT, "name", None) else {
                continue;
            };
            let name = name.trim_ascii().to_owned();
            let Some(group) = self.required(state_xml, ELEMENT, "group", Some(name.clone())) else {
                continue;
            };
            let group = group.trim_ascii().to_owned();
            if !self.model.groups.iter().any(|g| g.name == group) {
                self.warn(Diagnostic::UnknownGroup {
                    element: ELEMENT,
                    name,
                    group,
                });
                continue;
            }

            let context = || Some(format!("group state {name:?}"));
            let mut joint_values: BTreeMap<String, Vec<f64>> = BTreeMap::new();
            for joint in children(state_xml, "joint") {
                let Some(joint_name) = self.required(joint, "joint", "name", context()) else {
                    continue;
                };
                let joint_name = joint_name.trim_ascii().to_owned();
                let joint_context = format!("joint {joint_name:?} of group state {name:?}");
                let Some(raw) = self.required(joint, "joint", "value", Some(joint_context.clone()))
                else {
                    continue;
                };
                match parse_value_list(raw) {
                    // Upstream appends, so repeating a joint inside one state
                    // concatenates the value lists rather than replacing.
                    Some(values) => joint_values.entry(joint_name).or_default().extend(values),
                    None => self.warn(Diagnostic::MalformedValue {
                        element: "joint",
                        attribute: "value",
                        value: raw.to_owned(),
                        context: Some(joint_context),
                    }),
                }
            }

            self.model.group_states.push(GroupState {
                name,
                group,
                joint_values,
            });
        }
    }

    fn load_end_effectors(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "end_effector";
        for eef in children(robot, ELEMENT) {
            let Some(name) = self.required(eef, ELEMENT, "name", None) else {
                continue;
            };
            let name = name.trim_ascii().to_owned();
            let Some(component_group) = self.required(eef, ELEMENT, "group", Some(name.clone()))
            else {
                continue;
            };
            let component_group = component_group.trim_ascii().to_owned();
            if !self.model.groups.iter().any(|g| g.name == component_group) {
                self.warn(Diagnostic::UnknownGroup {
                    element: ELEMENT,
                    name,
                    group: component_group,
                });
                continue;
            }
            let Some(parent_link) = self.required(eef, ELEMENT, "parent_link", Some(name.clone()))
            else {
                continue;
            };

            self.model.end_effectors.push(EndEffector {
                name,
                parent_link: parent_link.trim_ascii().to_owned(),
                parent_group: eef
                    .attribute("parent_group")
                    .map(|g| g.trim_ascii().to_owned()),
                component_group,
            });
        }
    }

    fn load_link_sphere_approximations(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "link_sphere_approximation";
        for link_xml in children(robot, ELEMENT) {
            let Some(link) = self.required(link_xml, ELEMENT, "link", None) else {
                continue;
            };
            let link = link.trim_ascii().to_owned();
            let context = || Some(format!("link_sphere_approximation for link {link:?}"));

            // Tracks which of the three cases documented on
            // `SrdfModel::link_sphere_approximations` we are in: while it is
            // zero, `spheres` holds at most the normalised radius-zero
            // placeholder, and the first positive sphere clears it.
            let mut positive_radius_count = 0usize;
            let mut spheres: Vec<Sphere> = Vec::new();

            for sphere_xml in children(link_xml, "sphere") {
                let Some(raw_center) = self.required(sphere_xml, "sphere", "center", context())
                else {
                    continue;
                };
                let Some(raw_radius) = self.required(sphere_xml, "sphere", "radius", context())
                else {
                    continue;
                };
                let Some(center) = parse_center(raw_center) else {
                    self.warn(Diagnostic::MalformedValue {
                        element: "sphere",
                        attribute: "center",
                        value: raw_center.to_owned(),
                        context: context(),
                    });
                    continue;
                };
                let Some(radius) = parse_scalar(raw_radius) else {
                    self.warn(Diagnostic::MalformedValue {
                        element: "sphere",
                        attribute: "radius",
                        value: raw_radius.to_owned(),
                        context: context(),
                    });
                    continue;
                };

                if radius > f64::EPSILON {
                    if positive_radius_count == 0 {
                        spheres.clear();
                    }
                    spheres.push(Sphere { center, radius });
                    positive_radius_count += 1;
                } else if positive_radius_count == 0 {
                    // Collapse every radius-zero sphere to one canonical
                    // "this link is not collision-checked" marker.
                    spheres.clear();
                    spheres.push(Sphere::default());
                }
            }

            if !spheres.is_empty() {
                self.model
                    .link_sphere_approximations
                    .push(LinkSpheres { link, spheres });
            }
        }
    }

    fn load_collision_defaults(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "disable_default_collisions";
        for xml in children(robot, ELEMENT) {
            if let Some(link) = self.required(xml, ELEMENT, "link", None) {
                self.model
                    .no_default_collision_links
                    .push(link.trim_ascii().to_owned());
            }
        }
    }

    fn load_collision_pairs(
        &mut self,
        robot: Node<'_, '_>,
        element: &'static str,
        kind: CollisionKind,
    ) {
        for xml in children(robot, element) {
            // Upstream tests `!link1 || !link2` together and reports one
            // message for either, so the diagnostic names `link1` even when
            // only `link2` is absent. Reporting each attribute separately is
            // strictly more informative and cannot change which pairs survive.
            let link1 = self.required(xml, element, "link1", None);
            let link2 = self.required(xml, element, "link2", None);
            let (Some(link1), Some(link2)) = (link1, link2) else {
                continue;
            };
            let pair = CollisionPair {
                link1: link1.trim_ascii().to_owned(),
                link2: link2.trim_ascii().to_owned(),
                // Upstream stores `reason` verbatim; it is a human note, not a
                // name to be matched.
                reason: xml.attribute("reason").unwrap_or("").to_owned(),
            };
            match kind {
                CollisionKind::Enabled => self.model.enabled_collision_pairs.push(pair),
                CollisionKind::Disabled => self.model.disabled_collision_pairs.push(pair),
            }
        }
    }

    fn load_passive_joints(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "passive_joint";
        for xml in children(robot, ELEMENT) {
            if let Some(name) = self.required(xml, ELEMENT, "name", None) {
                self.model.passive_joints.push(name.trim_ascii().to_owned());
            }
        }
    }

    fn load_joint_properties(&mut self, robot: Node<'_, '_>) {
        const ELEMENT: &str = "joint_property";
        for xml in children(robot, ELEMENT) {
            let Some(joint_name) = self.required(xml, ELEMENT, "joint_name", None) else {
                continue;
            };
            let joint_name = joint_name.trim_ascii().to_owned();
            let Some(property_name) =
                self.required(xml, ELEMENT, "property_name", Some(joint_name.clone()))
            else {
                continue;
            };
            let property_name = property_name.trim_ascii().to_owned();
            let Some(value) = self.required(
                xml,
                ELEMENT,
                "value",
                Some(format!(
                    "property {property_name:?} of joint {joint_name:?}"
                )),
            ) else {
                continue;
            };

            self.model
                .joint_properties
                .entry(joint_name.clone())
                .or_default()
                .push(JointProperty {
                    joint_name,
                    property_name,
                    // Verbatim: upstream does not trim a property value.
                    value: value.to_owned(),
                });
        }
    }
}

/// Direct element children of `node` named `tag`.
///
/// Upstream walks `FirstChildElement(tag)` / `NextSiblingElement(tag)`, which
/// never descends. That is what keeps a `<passive_joint>` written inside a
/// `<group>` from becoming a model-level passive joint.
fn children<'a, 'i>(
    node: Node<'a, 'i>,
    tag: &'static str,
) -> impl Iterator<Item = Node<'a, 'i>> + use<'a, 'i> {
    node.children().filter(move |n| n.has_tag_name(tag))
}

/// Parse a whitespace-separated list of numbers, as upstream's
/// `istringstream >> double >> std::ws` loop reads a `group_state` joint value.
///
/// [`None`] when the text holds no numbers or holds anything that is not one;
/// upstream instead stores the failed extraction's zero. See the deviation note
/// on [`SrdfModel::group_states`].
fn parse_value_list(raw: &str) -> Option<Vec<f64>> {
    let values: Option<Vec<f64>> = raw
        .split_ascii_whitespace()
        .map(|token| token.parse::<f64>().ok())
        .collect();
    values.filter(|v| !v.is_empty())
}

/// Parse a sphere centre: the first three whitespace-separated numbers.
///
/// Upstream extracts exactly three doubles from a stream and never checks for
/// what follows, so trailing text is ignored on both sides.
fn parse_center(raw: &str) -> Option<[f64; 3]> {
    let mut tokens = raw.split_ascii_whitespace();
    let mut center = [0.0; 3];
    for slot in &mut center {
        *slot = tokens.next()?.parse().ok()?;
    }
    Some(center)
}

/// Parse a lone number that must be the whole attribute.
///
/// Upstream's `toDouble` extracts from a stream and then rejects anything the
/// extraction did not consume, so leading whitespace is allowed and trailing
/// whitespace is not. `str::parse` allows neither, hence the leading trim.
fn parse_scalar(raw: &str) -> Option<f64> {
    raw.trim_ascii_start().parse().ok()
}
