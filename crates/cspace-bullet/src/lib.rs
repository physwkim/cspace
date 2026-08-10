// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib

//! Bullet's convex narrow phase, in the single-precision scalar configuration,
//! ported to Rust for `cspace_collision`'s continuous collision check.
//!
//! # This is an altered version of Bullet
//!
//! Upstream is `bulletphysics/bullet3` @
//! `7dee3436e747958e7088dfdcea0e4ae031ce619e` (tag `3.24`,
//! `BT_BULLET_VERSION 324`), the version `moveit_core`'s Bullet backend is
//! built against in the differential oracle image. This crate is a derivative
//! work of it, marked as such here in satisfaction of clause 2 of the zlib
//! licence every Bullet source carries:
//!
//! > This software is provided 'as-is', without any express or implied
//! > warranty. In no event will the authors be held liable for any damages
//! > arising from the use of this software. Permission is granted to anyone to
//! > use this software for any purpose, including commercial applications, and
//! > to alter it and redistribute it freely, subject to the following
//! > restrictions:
//! >
//! > 1. The origin of this software must not be misrepresented; you must not
//! >    claim that you wrote the original software. If you use this software in
//! >    a product, an acknowledgment in the product documentation would be
//! >    appreciated but is not required.
//! > 2. Altered source versions must be plainly marked as such, and must not be
//! >    misrepresented as being the original software.
//! > 3. This notice may not be removed or altered from any source distribution.
//!
//! `simplex` additionally carries Bullet's own credit for the Ericson
//! material it derives from; see that module's header.
//!
//! # Why a separate crate
//!
//! Every other crate in this workspace ports `moveit2`, which is BSD-3-Clause.
//! Bullet is zlib. `tools/ci/check-license-matches-upstream.sh` requires a
//! crate's sources to agree on one SPDX identifier and the manifest to declare
//! it, so a zlib-derived module inside `cspace-collision` would either relabel
//! Bullet's code as BSD-3-Clause or split that crate's identifier. The root
//! `Cargo.toml` states the same rule for `cspace-stomp-core`, which is this
//! arrangement for `ros-industrial/stomp`: one crate, one upstream, because
//! every audit command in this repo counts a crate's symbols against exactly
//! one upstream.
//!
//! The division of labour that follows from it: everything here is Bullet's,
//! and MoveIt's own Bullet integration -- `CastHullShape`, the cast broadphase
//! manager, `checkRobotCollisionHelperCCD` -- is BSD-3-Clause `moveit2` code
//! and lives in `cspace-collision`, built on the [`shapes::ConvexShape`] trait
//! this crate exposes for exactly that purpose.
//!
//! # Scope
//!
//! Only what MoveIt's continuous check reaches. `CollisionEnvBullet` builds a
//! `btBoxShape`, `btSphereShape`, `btCylinderShapeZ`, `btConeShapeZ`,
//! `btConvexHullShape` or a `btCompoundShape` of those per collision body
//! (`bullet_utils.cpp:84-210`), so the shape layer covers those and no others:
//! no capsule, no triangle mesh, no heightfield, no soft body, and no rigid
//! body dynamics of any kind.
//!
//! # Precision
//!
//! `btScalar` is `float` here, not `double`. That is not a simplification --
//! it is what the oracle's Bullet is built as, and reproducing it is what
//! makes an exact comparison possible at all. See `linear_math`'s module docs
//! for which build configuration this reproduces and why bit-exact agreement
//! with the C++ is reachable rather than aspirational.

pub mod discrete_detector;
pub mod epa;
pub mod gjk;
pub mod linear_math;
pub mod manifold;
pub mod pen_depth;
#[cfg(test)]
mod probe_fixture;
pub mod shapes;
pub mod simplex;
