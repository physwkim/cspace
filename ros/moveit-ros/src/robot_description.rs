// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The two latched robot descriptions an unmodified `MoveGroupInterface`
//! needs before its constructor can return.
//!
//! # The premise this module was written to check, and what it measured
//!
//! `PORTING-PLAN.md` §273.5 says the client "reads it from its OWN node
//! parameters via `getSharedRobotModel`/`RobotModelLoader`, never off the
//! graph". Re-derived against the pinned upstream checkout, the first half
//! is right and the words "never off the graph" are wrong -- the parameter
//! is the *first* of two sources, and the second is a latched topic:
//!
//! `getSharedRobotModel` builds a `RobotModelLoader`
//! (`common_objects.cpp:124`), which builds
//! `RDFLoader(node_, opt.robot_description)`
//! (`robot_model_loader.cpp:115`), whose constructor calls
//! `SynchronizedStringParameter::loadInitialValue` twice --
//! once for `robot_description` and once for `robot_description` +
//! `"_semantic"` (`rdf_loader.cpp:92-99`). And `loadInitialValue` reads the
//! node's own parameter first (`getMainParameter`,
//! `synchronized_string_parameter.cpp:96-107`), *returning early only if it
//! is non-empty* (`:50-62`); when it is empty it falls through to
//! `waitForMessage` (`:82`), which subscribes to a `std_msgs/msg/String` on
//! a topic of that same name with `rclcpp::QoS(1).transient_local().reliable()`
//! and blocks on a wait set (`:121-145`). The timeout is the
//! `<name>_timeout` parameter, defaulting to 10.0 s
//! (`rdf_loader.hpp:77`, `synchronized_string_parameter.hpp:63-64`).
//!
//! So a client that was *given* the XML in its own parameters never looks at
//! the graph, and a client that was not blocks 10 s per description and then
//! carries on with an empty string. The second is the case a bare client
//! hits, and it is the case this module serves. §273.5's own neighbour
//! already said so: `doc/client-endpoint-surface.md` classifies the
//! `robot_description` row as "parameter or latched publisher", derived from
//! `measure-client-endpoint-surface.py`'s `PORT_ROLE`.
//!
//! # Both, or neither
//!
//! The invariant this module exists to hold: **a running `move_group`
//! process has both descriptions latched, or it is not running.**
//!
//! Half-provision is worse than none, and not symmetrically so. With the
//! URDF latched and the SRDF missing, `RDFLoader` waits out the SRDF's 10 s,
//! then calls `loadFromStrings` (`rdf_loader.cpp:101`) with an empty SRDF
//! string; `srdf->initString(*urdf, "")` parses an SRDF with no groups, so
//! the client gets a *model* -- one whose `getJointModelGroup(group)` is
//! null. `MoveGroupInterface`'s constructor then throws on the group it was
//! asked for, and the reason it gives names the group, not the missing
//! description.
//!
//! Two things hold the invariant, neither of them a runtime check:
//!
//! * [`latch`] is the only constructor of [`Descriptions`], it creates both
//!   publishers before it publishes either, and it returns `Err` if any of
//!   the four calls fails. `main` turns that into `ExitCode::FAILURE`, and a
//!   process that exits takes its transient-local samples with it -- which
//!   is what makes "neither" reachable at all. Dropping a `Publisher` would
//!   not: `Node::create_publisher` pushes its own `Arc` onto the node's
//!   `pubs` and hands back a `Weak` (read from `r2r/src/nodes.rs` at the
//!   pinned rev), so letting this module's handle go does not close the
//!   socket.
//! * [`Descriptions`] has no accessor, no `Deref` and no public field, so no
//!   caller can reach one publisher without the other, or keep one alive
//!   past the other.
//!
//! The publish order is the other half. The SRDF goes first and the URDF
//! last, so that a failure *between* the two publishes leaves the URDF
//! missing rather than the SRDF: an empty URDF makes `urdf->initString`
//! fail, `loadFromStrings` return false (`rdf_loader.cpp:118-125`) and the
//! client end up with no model at all. No model is a louder failure than a
//! model with no groups.
//!
//! # Deviation: upstream's publisher is opt-in, this one is not
//!
//! Upstream publishes these two topics from the same
//! `SynchronizedStringParameter` that reads them, and only when the node's
//! own `publish_robot_description` parameter is true (`shouldPublish`,
//! `synchronized_string_parameter.cpp:109-119`) *and* the node already had
//! the XML in a parameter (`:50-60`) -- i.e. upstream republishes what it
//! was configured with. This node has no such parameter: it is given the two
//! files on `argv` and always latches them. The QoS is upstream's own
//! publisher QoS, `rclcpp::QoS(1).transient_local()` (`:55`), spelled out
//! including the reliability rclcpp leaves implicit, because the subscriber
//! it has to match asks for `.reliable()` explicitly (`:127`).

use r2r::{Node, Publisher, QosProfile};

/// The two latched descriptions, alive for as long as this value is.
///
/// Opaque on purpose: see this module's doc. There is nothing to read off
/// it and nothing to take out of it -- holding it is its whole contract, and
/// the type exists so that "holding both" is the only thing a caller can do.
pub struct Descriptions {
    _publishers: [Publisher<r2r::std_msgs::msg::String>; 2],
}

/// Latches `robot_description_semantic` and `robot_description`, in that
/// order, and returns the handle that keeps them on the graph.
///
/// # Errors
///
/// Any failure to create or publish on either topic. The caller must treat
/// this as fatal: see this module's doc for why a partial latch is worse
/// than none.
pub fn latch(node: &mut Node, urdf_xml: &str, srdf_xml: &str) -> Result<Descriptions, r2r::Error> {
    // `rclcpp::QoS(1).transient_local()`
    // (`synchronized_string_parameter.cpp:55`), against a subscriber that
    // asks for `rclcpp::QoS(1).transient_local().reliable()` (`:127`).
    // `transient_local` is the whole mechanism: the client subscribes long
    // after this call, and a volatile publisher would have nothing to give
    // it.
    let qos = QosProfile::default()
        .keep_last(1)
        .transient_local()
        .reliable();

    // Both literals are written here rather than as named constants for the
    // reason `src/bin/move_group.rs` records at its own
    // `create_action_server`: `tools/ci/measure-client-endpoint-surface.py`'s
    // `PORT_OPENER` matches a string literal inside the factory call, so a
    // constant would leave these endpoints reading `absent` in
    // `doc/client-endpoint-surface.md` with the publishers running.
    //
    // `robot_description_semantic` is not a name upstream writes down: it is
    // `ros_name + "_semantic"` built inside the `RDFLoader` constructor
    // (`rdf_loader.cpp:96`) from the `robot_description` that
    // `RobotModelLoader` passes it (`robot_model_loader.cpp:115`).
    //
    // Created before either is published, so a create failure leaves nothing
    // latched at all.
    let semantic = node.create_publisher::<r2r::std_msgs::msg::String>(
        "robot_description_semantic",
        qos.clone(),
    )?;
    let description =
        node.create_publisher::<r2r::std_msgs::msg::String>("robot_description", qos)?;

    semantic.publish(&r2r::std_msgs::msg::String {
        data: srdf_xml.to_string(),
    })?;
    description.publish(&r2r::std_msgs::msg::String {
        data: urdf_xml.to_string(),
    })?;

    Ok(Descriptions {
        _publishers: [semantic, description],
    })
}
