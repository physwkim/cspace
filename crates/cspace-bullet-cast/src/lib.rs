// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp

//! MoveIt's Bullet continuous-collision layer: the swept shape its two-state
//! `checkRobotCollision` builds, and the contact conversion that reads a
//! contact back off it.
//!
//! # Why a separate crate
//!
//! Three upstreams meet on the continuous path, and they carry three licences.
//! Bullet itself is zlib and is [`cspace_bullet`]. `CollisionEnvBullet`'s
//! `checkRobotCollisionHelperCCD` is in `collision_env_bullet.cpp`, whose
//! header reads "BSD License" (three clauses), and belongs with the rest of
//! `cspace-collision`. Everything between them -- `CastHullShape`,
//! `getAverageSupport`, `addCastSingleResult`, `makeCastCollisionObject`,
//! `setCastCollisionObjectsTransform`, `processResult` -- lives in
//! `bullet_integration/`, whose per-file headers read **BSD-2-Clause**.
//!
//! `moveit_core`'s `package.xml` says BSD-3-Clause for the package as a whole,
//! and that is not the licence of these files:
//!
//! | upstream file | header |
//! | --- | --- |
//! | `bullet_integration/bullet_utils.{hpp,cpp}` | BSD-2-Clause |
//! | `bullet_integration/bullet_bvh_manager.{hpp,cpp}` | BSD-2-Clause |
//! | `bullet_integration/bullet_cast_bvh_manager.{hpp,cpp}` | BSD-2-Clause |
//! | `bullet_integration/contact_checker_common.cpp` | BSD-2-Clause |
//! | `bullet_integration/basic_types.hpp` | Apache-2.0 |
//! | `collision_env_bullet.cpp` | BSD License (3-clause) |
//!
//! `tools/ci/check-license-matches-upstream.sh` requires one SPDX identifier
//! per crate and a manifest that declares it, so these cannot sit inside
//! `cspace-collision` without relabelling them. Its own message names the
//! remedy -- "a crate counts against exactly one upstream; split it" -- and
//! `cspace-bullet` and `cspace-stomp-core` are the same arrangement for
//! bullet3 and ros-industrial/stomp.
//!
//! `basic_types.hpp` is Apache-2.0 and is *not* ported: `ContactTestData` is a
//! bundle of references to a request, a result and two flags, and the Rust
//! side threads `cspace_collision`'s own request and result types instead of
//! reproducing that struct.
//!
//! # Scope
//!
//! The two-state path and nothing else. MoveIt's discrete Bullet backend --
//! `BulletDiscreteBVHManager`, `addDiscreteSingleResult`, the discrete
//! `contactTest` -- is not here; `cspace-collision`'s discrete check stays on
//! `parry`, and porting a second discrete backend it would not call is work
//! with no caller.

/// Test-only: `probe_shapes` behind the `Arc` this crate's shapes are held by.
#[cfg(test)]
mod arc_probe;
pub mod cast_contact;
pub mod cast_hull_shape;
pub mod contact_test_data;
pub mod shape_primitive;
