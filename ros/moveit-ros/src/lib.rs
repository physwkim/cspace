// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Optional ROS 2 interop for moveit-rs (`PORTING-PLAN.md` Phase 9, D2/D5/D6).
//!
//! This crate is the *only* place in the workspace allowed to depend on a
//! ROS 2 client library (r2r 0.9.5) or know that `moveit_msgs` exists (D2).
//! Core crates under `crates/` have no ROS-aware methods; every conversion
//! lives here, as an explicit, fallible [`TryFrom`] in both directions (D6).
//! `moveit-ros` lives outside the root workspace (`ros/moveit-ros/`, its own
//! `[workspace]`), because r2r requires a local ROS 2 install at build time
//! that neither this host nor the CI runner has -- see
//! `PORTING-PLAN.md` §129 and this crate's `Cargo.toml` header.
//!
//! # Round 1 scope (PORTING-PLAN.md Phase 9, this round)
//!
//! Type conversion only -- no `/plan_kinematic_path` service, no
//! `/move_action` action server, no planning-scene subscription (deferred to
//! a later round). This round also does not build `moveit_msgs` itself into
//! the image (see `ros/Dockerfile`'s header comment) -- only the
//! `geometry_msgs` primitives that `moveit_msgs` messages are themselves
//! built from, which are already installed in the base ROS 2 image. The full
//! `moveit_msgs` <-> core-crate mapping (including every non-1:1 spot and
//! every `TryFrom` failure condition, coded and not-yet-coded alike) is
//! tracked in `doc/message-mapping.md`.
//!
//! # The orphan-rule finding
//!
//! D6 says compatibility is `TryFrom`, both directions. Read literally as
//! `impl TryFrom<moveit_msgs::msg::X> for moveit_model::Y` (or the reverse),
//! this does not compile for *any* pair of types, from any round: `X` is
//! defined in the `r2r` crate (r2r generates message bindings into its own
//! crate, not the consuming crate -- confirmed by reading
//! `r2r`'s own generated-code layout, not assumed), `Y` is defined in a
//! `crates/moveit-*` crate, and `moveit-ros` is a *third* crate relative to
//! both. Rust's orphan rule forbids a third crate from implementing a
//! foreign trait (`std::convert::TryFrom`) between two foreign types,
//! unconditionally, regardless of which crate defines which side. This is
//! not specific to `nalgebra`-aliased types like [`moveit_geometry::Isometry3`]
//! -- it would block a `moveit_msgs::msg::JointConstraint` <->
//! `moveit_constraints::JointConstraint` `TryFrom` exactly the same way, in
//! whichever round implements that.
//!
//! The fix that keeps the letter of D6 (the actual `std::convert::TryFrom`
//! trait, not a look-alike local trait) is the standard one for this
//! situation: wrap the *message* type in a local newtype defined in this
//! crate, and `impl TryFrom` against the newtype rather than the bare `r2r`
//! type. One of the trait's two type positions (the newtype) is then local,
//! which is what the orphan rule actually requires -- it does not require
//! `Self` specifically to be local, only *some* position in the impl header
//! not preceded by an uncovered type parameter. [`geometry::Pose`],
//! [`geometry::Point`], [`geometry::Quaternion`] and [`geometry::Vector3`]
//! are exactly this: a `pub struct Foo(pub r2r::geometry_msgs::msg::Foo);`
//! wrapper, nothing more. Every later round's `moveit_msgs::msg::X`
//! conversion needs the same one-line wrapper before its `TryFrom` impl will
//! compile -- this is a crate-wide convention, not a one-off patch for
//! `geometry_msgs`.
//!
//! Verified empirically, not just reasoned about: `cargo build` for this
//! crate (with the wrapper) and a scratch crate without one (which failed
//! with `error[E0117]: only traits defined in the current crate can be
//! implemented for arbitrary types`) were both run inside `ros/`'s
//! container -- see this round's report for the exact command and output.

pub mod constraints;
pub mod geometry;
pub mod state;
pub mod trajectory;

pub use moveit_error::{Error, Result};
