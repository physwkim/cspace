// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `robot_description`: the URDF (and SRDF) a `MoveGroupInterface`
//! constructor builds its own `RobotModel` from, published as latched
//! strings.
//!
//! `doc/client-endpoint-surface.md` lists this endpoint as "reads --
//! parameter or latched publisher", and the two are not interchangeable
//! from a provider's side. `SynchronizedStringParameter::loadInitialValue`
//! (`synchronized_string_parameter.cpp:41-94` (`loadInitialValue`)) tries
//! them in order:
//!
//! 1. `getMainParameter` (`:96-107` (`getMainParameter`)) declares and reads
//!    the parameter **on `node_`** -- the `rclcpp::Node` the caller handed
//!    `MoveGroupInterface`, i.e. the client's own node. A parameter set
//!    there comes from that process's own launch configuration; no other
//!    node can put it there. Nothing this port publishes can satisfy step 1.
//! 2. Only when that parameter is absent or empty does `waitForMessage`
//!    (`:121-145` (`waitForMessage`)) subscribe to a `std_msgs/msg/String`
//!    topic of the same name and take one message, waiting
//!    `<name>_timeout` seconds (10 by default, `:73-80`).
//!
//! So the mechanism a *server* can provide is exactly one of the two: the
//! topic. This module opens it, with the QoS upstream's own publisher uses
//! -- `rclcpp::QoS(1).transient_local()` (`:55`), matched by the client's
//! `rclcpp::QoS(1).transient_local().reliable()` (`:127`). Transient-local
//! is what makes the single message readable by a client that starts later,
//! which is the whole shape of this endpoint: it is published once, at
//! startup, not on a timer.
//!
//! # Why the SRDF is published too
//!
//! The client's `RDFLoader` constructor loads two strings through one
//! `SynchronizedStringParameter` each -- `ros_name` and `ros_name +
//! "_semantic"` (`rdf_loader.cpp:92-99` (`RDFLoader`)) -- and then
//! `loadFromStrings` (`:118-137` (`loadFromStrings`)) returns false unless
//! **both** parse. An empty SRDF does not parse: `srdf::Model::initString`
//! fails at `xml_doc.Parse` before it reaches any element
//! (`third_party/srdfdom/src/model.cpp:727-735`), because TinyXML2 rejects
//! an empty document. A client handed a perfect URDF and no
//! `robot_description_semantic` therefore still ends its constructor with
//! no `RobotModel` at all, which is the failure this endpoint exists to
//! prevent. `doc/client-endpoint-surface.md` names only
//! `robot_description`, because it derives endpoints from
//! `MoveGroupInterface`'s own declarations and the second name appears
//! inside `RDFLoader`; the endpoint is the model load, and the model load
//! is two topics.
//!
//! # One owner, one load
//!
//! The URDF this node plans against and the URDF a client reads must be the
//! same document: two loads from two paths can disagree, and a client that
//! plans against a different model than the server does not fail loudly --
//! it fails by naming joints the server does not have. [`RobotDescription`]
//! is that single owner. It performs the only read of either file in this
//! process, keeps both strings, and [`RobotDescription::parse`] builds the
//! [`RobotModel`] from the very string [`RobotDescription::publish`] sends.
//! Neither the model nor the message can be built from anything else,
//! because nothing else has the text.

use std::fs;
use std::io;
use std::path::Path;

use moveit_error::{Error, Result};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, ReliabilityPolicy};
use r2r::std_msgs::msg::String as StringMsg;
use r2r::{Publisher, QosProfile};

/// Upstream's own name for the URDF string, the default
/// `MoveGroupInterface::Options::robot_description`
/// (`move_group_interface.cpp:88`). Unqualified, so it resolves against the
/// node's namespace exactly as the client's own
/// `rclcpp::names::append(move_group_namespace, ...)` names do.
pub const URDF_TOPIC: &str = "robot_description";

/// `ros_name + "_semantic"`, `RDFLoader`'s own spelling
/// (`rdf_loader.cpp:96` (`RDFLoader`)).
pub const SRDF_TOPIC: &str = "robot_description_semantic";

/// The URDF and SRDF text this process was started with: one load, one
/// owner. See the module doc.
#[derive(Debug)]
pub struct RobotDescription {
    urdf_xml: String,
    srdf_xml: String,
}

/// The two latched publishers, alive for as long as this value is. Dropping
/// it takes both topics down -- r2r unregisters a publisher on drop, and a
/// transient-local topic with no live publisher has nothing to hand a late
/// subscriber.
pub struct LatchedDescription {
    _urdf: Publisher<StringMsg>,
    _srdf: Publisher<StringMsg>,
}

impl RobotDescription {
    /// Reads both files. The only read of either in this process.
    ///
    /// # Errors
    ///
    /// The underlying [`io::Error`], with the path that produced it already
    /// named in the message.
    pub fn load(urdf_path: &Path, srdf_path: &Path) -> io::Result<Self> {
        let urdf_xml = fs::read_to_string(urdf_path).map_err(|e| {
            io::Error::new(e.kind(), format!("reading {}: {e}", urdf_path.display()))
        })?;
        let srdf_xml = fs::read_to_string(srdf_path).map_err(|e| {
            io::Error::new(e.kind(), format!("reading {}: {e}", srdf_path.display()))
        })?;
        Ok(Self { urdf_xml, srdf_xml })
    }

    /// The same two strings, for a caller that already has them (this
    /// crate's own tests, which have no files).
    pub fn from_strings(urdf_xml: impl Into<String>, srdf_xml: impl Into<String>) -> Self {
        Self {
            urdf_xml: urdf_xml.into(),
            srdf_xml: srdf_xml.into(),
        }
    }

    /// The URDF text, verbatim -- both what [`Self::parse`] builds the model
    /// from and what [`Self::publish`] puts on the wire.
    pub fn urdf_xml(&self) -> &str {
        &self.urdf_xml
    }

    /// The SRDF text, verbatim.
    pub fn srdf_xml(&self) -> &str {
        &self.srdf_xml
    }

    /// Builds the model a client reading these two topics would build.
    ///
    /// # Errors
    ///
    /// [`Error::Other`] naming which of the three steps failed (URDF parse,
    /// SRDF parse, model build) and why.
    pub fn parse(&self, mesh_search_paths: &MeshSearchPaths) -> Result<(RobotModel, SrdfModel)> {
        let urdf = urdf_rs::read_from_string(&self.urdf_xml)
            .map_err(|e| Error::other(format!("parsing {URDF_TOPIC}: {e}")))?;
        let srdf = SrdfModel::parse_str(&self.srdf_xml)
            .map_err(|e| Error::other(format!("parsing {SRDF_TOPIC}: {e}")))?;
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &self.urdf_xml, &srdf, mesh_search_paths)?;
        Ok((model, srdf))
    }

    /// Opens both topics and publishes each string once.
    ///
    /// The return value must be kept alive; see [`LatchedDescription`].
    ///
    /// # Errors
    ///
    /// r2r's own error from `create_publisher` or `publish`.
    pub fn publish(
        &self,
        node: &mut r2r::Node,
    ) -> std::result::Result<LatchedDescription, r2r::Error> {
        let urdf = node.create_publisher::<StringMsg>(URDF_TOPIC, latched_qos())?;
        let srdf = node.create_publisher::<StringMsg>(SRDF_TOPIC, latched_qos())?;
        urdf.publish(&StringMsg {
            data: self.urdf_xml.clone(),
        })?;
        srdf.publish(&StringMsg {
            data: self.srdf_xml.clone(),
        })?;
        Ok(LatchedDescription {
            _urdf: urdf,
            _srdf: srdf,
        })
    }
}

/// `rclcpp::QoS(1).transient_local()`
/// (`synchronized_string_parameter.cpp:55`), spelled out field by field
/// because a client's `rclcpp::QoS(1).transient_local().reliable()`
/// (`:127`) will not match a publisher that differs on any one of them --
/// a best-effort publisher is incompatible with that reliable subscription
/// and a volatile one is invisible to a client that starts second, which is
/// every client of a node that publishes this once at startup.
///
/// `depth: 1` is `QoS(1)`'s own history depth. The three fields left to
/// [`QosProfile::default`] (deadline, lifespan, liveliness) are the ones
/// `rclcpp::QoS` also leaves at the RMW default.
pub fn latched_qos() -> QosProfile {
    QosProfile {
        history: HistoryPolicy::KeepLast,
        depth: 1,
        reliability: ReliabilityPolicy::Reliable,
        durability: DurabilityPolicy::TransientLocal,
        ..QosProfile::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const URDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>"#;

    const SRDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm">
    <chain base_link="base_link" tip_link="tip"/>
  </group>
</robot>"#;

    /// The endpoint's whole point: the string a client reads is the string
    /// the model was built from, not a second read of the same path.
    #[test]
    fn the_published_urdf_is_the_text_the_model_was_built_from() {
        let description = RobotDescription::from_strings(URDF, SRDF);
        let (model, _srdf) = description
            .parse(&MeshSearchPaths::none())
            .expect("fixture must build");
        assert_eq!(description.urdf_xml(), URDF);
        assert_eq!(description.srdf_xml(), SRDF);
        // The model really is this document's: its joint is the one only
        // this URDF declares.
        assert_eq!(model.name(), "one_joint");
        assert!(model.joint_model("j1").is_ok());
    }

    /// A client subscribing with `transient_local().reliable()` matches a
    /// publisher on all four of these and nothing less; a volatile or
    /// best-effort publisher is unreadable to it.
    #[test]
    fn latched_qos_is_transient_local_reliable_depth_one() {
        let qos = latched_qos();
        assert_eq!(qos.durability, DurabilityPolicy::TransientLocal);
        assert_eq!(qos.reliability, ReliabilityPolicy::Reliable);
        assert_eq!(qos.history, HistoryPolicy::KeepLast);
        assert_eq!(qos.depth, 1);
    }

    #[test]
    fn a_malformed_urdf_is_reported_against_the_urdf_topic() {
        let description = RobotDescription::from_strings("<robot", SRDF);
        let rendered = description
            .parse(&MeshSearchPaths::none())
            .expect_err("malformed URDF must be rejected")
            .to_string();
        assert!(
            rendered.contains(URDF_TOPIC) && !rendered.contains(SRDF_TOPIC),
            "expected the URDF step to name itself, got: {rendered}"
        );
    }

    #[test]
    fn a_malformed_srdf_is_reported_against_the_semantic_topic() {
        let description = RobotDescription::from_strings(URDF, "");
        let rendered = description
            .parse(&MeshSearchPaths::none())
            .expect_err("an empty SRDF must be rejected")
            .to_string();
        assert!(
            rendered.contains(SRDF_TOPIC),
            "expected the SRDF step to name itself, got: {rendered}"
        );
    }

    #[test]
    fn load_names_the_file_it_could_not_read() {
        let missing = Path::new("/nonexistent/moveit-rs/no_such.urdf");
        let rendered = RobotDescription::load(missing, missing)
            .expect_err("a missing file must be reported")
            .to_string();
        assert!(
            rendered.contains("no_such.urdf"),
            "expected the path in the message, got: {rendered}"
        );
    }
}
