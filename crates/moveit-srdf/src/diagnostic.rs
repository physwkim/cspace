// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from srdfdom 2.0.8 — the SRDF parser that moveit2 @
// e017c91ee12984393a28ba246075c65f69cde3bf depends on:
//   srdfdom/src/model.cpp   (every variant here replaces one CONSOLE_BRIDGE
//                            report from that file)

use std::fmt;

/// Something the parser dropped or repaired while reading an SRDF.
///
/// Upstream srdfdom writes these to `console_bridge` and returns a model that
/// carries no trace of them, so a caller cannot tell a group that was never
/// written from one that was thrown away over a typo. Each variant here stands
/// for one upstream report; the parser takes the same action upstream takes,
/// and additionally records it in [`SrdfModel::diagnostics`].
///
/// A diagnostic is never fatal. The two fatal conditions — text that is not
/// well-formed XML, and a root element that is not `robot` — come back as
/// [`moveit_error::Error::Parse`] instead.
///
/// [`SrdfModel::diagnostics`]: crate::SrdfModel::diagnostics
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Diagnostic {
    /// The `robot` element has no `name` attribute.
    MissingRobotName,

    /// A required attribute is absent, so the element was dropped.
    MissingAttribute {
        /// The element that was dropped, e.g. `"virtual_joint"`.
        element: &'static str,
        /// The attribute that was missing, e.g. `"child_link"`.
        attribute: &'static str,
        /// What the element belonged to, when the element itself cannot be
        /// named because the missing attribute *is* its name.
        context: Option<String>,
    },

    /// An attribute is present but its text does not parse, so the element was
    /// dropped.
    MalformedValue {
        /// The element that was dropped, e.g. `"sphere"`.
        element: &'static str,
        /// The attribute that would not parse, e.g. `"radius"`.
        attribute: &'static str,
        /// The text as written in the document.
        value: String,
        /// What the element belonged to.
        context: Option<String>,
    },

    /// A `virtual_joint` named a type that is not `fixed`, `planar` or
    /// `floating`. The joint is kept, as `fixed`, matching upstream.
    UnknownVirtualJointType {
        /// The name of the virtual joint.
        joint: String,
        /// The `type` attribute as written, before lower-casing.
        raw: String,
    },

    /// A `group_state` or `end_effector` named a group the document does not
    /// define, so it was dropped.
    UnknownGroup {
        /// `"group_state"` or `"end_effector"`.
        element: &'static str,
        /// The name of the dropped element.
        name: String,
        /// The group it referred to.
        group: String,
    },

    /// A group names no joints, links, chains or subgroups. It is kept — a
    /// group can legitimately be empty — but is almost always a mistake.
    EmptyGroup {
        /// The name of the empty group.
        group: String,
    },

    /// A group's subgroups do not all resolve to defined groups, so the group
    /// was dropped. This is transitive: dropping a group drops anything that
    /// listed it as a subgroup, and a subgroup cycle drops every group in it.
    UnsatisfiedSubgroups {
        /// The name of the dropped group.
        group: String,
    },
}

impl fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingRobotName => f.write_str("no name given for the robot"),
            Self::MissingAttribute {
                element,
                attribute,
                context,
            } => {
                write!(f, "`{element}` is missing the `{attribute}` attribute")?;
                write_context(f, context)
            }
            Self::MalformedValue {
                element,
                attribute,
                value,
                context,
            } => {
                write!(
                    f,
                    "`{element}` has an unparsable `{attribute}` attribute {value:?}"
                )?;
                write_context(f, context)
            }
            Self::UnknownVirtualJointType { joint, raw } => write!(
                f,
                "virtual joint {joint:?} has unknown type {raw:?}; treated as `fixed`"
            ),
            Self::UnknownGroup {
                element,
                name,
                group,
            } => write!(
                f,
                "`{element}` {name:?} names group {group:?}, which is not defined"
            ),
            Self::EmptyGroup { group } => write!(f, "group {group:?} is empty"),
            Self::UnsatisfiedSubgroups { group } => write!(
                f,
                "group {group:?} has subgroups that are not defined; the group is dropped"
            ),
        }
    }
}

fn write_context(f: &mut fmt::Formatter<'_>, context: &Option<String>) -> fmt::Result {
    match context {
        Some(context) => write!(f, " (in {context})"),
        None => Ok(()),
    }
}
