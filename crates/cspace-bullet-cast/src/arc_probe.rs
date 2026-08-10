// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause

//! `cspace_bullet::probe_fixture::probe_shapes` behind the shared ownership
//! [`CastHullShape`](crate::cast_hull_shape::CastHullShape) takes.
//!
//! Not a second shape set: it is the same eight shapes `probe.cpp` builds, and
//! the reason a copy of them would be wrong is stated on `probe_shapes` itself
//! -- a divergent copy fails as a bit difference somewhere in a result, blamed
//! on the algorithm, rather than as a missing fixture.

use std::sync::Arc;

use cspace_bullet::probe_fixture::probe_shapes;

use crate::cast_hull_shape::ArcConvexShape;

/// The seven of `probe_shapes`'s eight shapes the cast tests use, as
/// [`ArcConvexShape`].
///
/// `small_sphere` is dropped rather than returned and ignored: it exists for
/// `cspace_bullet`'s sphere-sphere rows, and a cast shape sweeps one shape, so
/// nothing here has a second sphere to pair it with.
pub(crate) fn arc_probe_shapes() -> (
    ArcConvexShape,
    ArcConvexShape,
    ArcConvexShape,
    ArcConvexShape,
    ArcConvexShape,
    ArcConvexShape,
    ArcConvexShape,
) {
    let (unit_box, flat_box, margin_box, sphere, _small_sphere, cyl, cone, hull) = probe_shapes();
    (
        Arc::new(unit_box),
        Arc::new(flat_box),
        Arc::new(margin_box),
        Arc::new(sphere),
        Arc::new(cyl),
        Arc::new(cone),
        Arc::new(hull),
    )
}
