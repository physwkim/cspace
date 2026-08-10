// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (class PathSegment)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (LinearPathSegment, CircularPathSegment)

//! `PathSegment` and its two concrete kinds.
//!
//! Nothing declared in this module is accessible outside the crate: upstream's
//! `Path` keeps `path_segments_` private (`std::list<std::unique_ptr<PathSegment>>`)
//! and its own public API (`getConfig`/`getTangent`/`getCurvature`) never
//! hands a caller a `PathSegment`. `LinearPathSegment`/`CircularPathSegment`
//! aren't even declared in the header — they're defined in the `.cpp` file,
//! making them `Path::create`'s implementation detail twice over. The module
//! itself is `pub` only so this explanation is reachable from `Path`'s own
//! docs; every item in it stays `pub(crate)`.

mod circular;
mod linear;

use nalgebra::DVector;

use circular::Circular;
use linear::Linear;

/// One piece of a [`crate::Path`]: either a [`Linear`] segment between two
/// waypoints, or a [`Circular`] blend at one.
///
/// Upstream `PathSegment` is an abstract base (`getConfig`/`getTangent`/
/// `getCurvature`/`getSwitchingPoints` pure virtual, `clone` for the deep
/// copy `Path`'s copy constructor needs) with exactly two subclasses. This
/// port collapses that into a closed sum type — the same choice
/// `cspace-model`'s `JointModel`/`JointKind` makes for an analogous
/// two-subclass-in-practice hierarchy — which also means [`PathSegment`]
/// derives [`Clone`] for free instead of needing a hand-written deep-copy
/// constructor.
///
/// `length` and `position` are upstream's base-class `length_`/`position_`
/// fields. `length` is computed once by [`PathSegment::linear`]/
/// [`PathSegment::circular`] and handed to kind-specific methods that need
/// it (mirroring how [`crate::path_segment`]'s kinds take bounds/length as
/// parameters rather than storing a duplicate copy — the same pattern
/// `cspace-model`'s joint kinds use for `VariableBounds`). `position` is
/// upstream's `PathSegment::position_` public field, set by
/// [`crate::Path::create`] only after every segment exists (it needs the
/// running total length), so it starts at `0.0` and is filled in by
/// [`PathSegment::set_position`].
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PathSegment {
    length: f64,
    position: f64,
    kind: PathSegmentKind,
}

#[derive(Debug, Clone, PartialEq)]
enum PathSegmentKind {
    Linear(Linear),
    Circular(Circular),
}

impl PathSegment {
    /// `new LinearPathSegment(start, end)`.
    pub(crate) fn linear(start: DVector<f64>, end: DVector<f64>) -> Self {
        let (linear, length) = Linear::new(start, end);
        Self {
            length,
            position: 0.0,
            kind: PathSegmentKind::Linear(linear),
        }
    }

    /// `new CircularPathSegment(start, intersection, end, max_deviation)`.
    pub(crate) fn circular(
        start: &DVector<f64>,
        intersection: &DVector<f64>,
        end: &DVector<f64>,
        max_deviation: f64,
    ) -> Self {
        let (circular, length) = Circular::new(start, intersection, end, max_deviation);
        Self {
            length,
            position: 0.0,
            kind: PathSegmentKind::Circular(circular),
        }
    }

    /// `getLength`.
    pub(crate) fn length(&self) -> f64 {
        self.length
    }

    /// `position_`, the absolute arc length of this segment's start within
    /// the parent [`crate::Path`].
    pub(crate) fn position(&self) -> f64 {
        self.position
    }

    /// Set `position_`. Only [`crate::Path::create`] calls this, once every
    /// segment's length is known.
    pub(crate) fn set_position(&mut self, position: f64) {
        self.position = position;
    }

    /// `getConfig`.
    pub(crate) fn config(&self, s: f64) -> DVector<f64> {
        match &self.kind {
            PathSegmentKind::Linear(l) => l.config(s, self.length),
            PathSegmentKind::Circular(c) => c.config(s),
        }
    }

    /// `getTangent`.
    pub(crate) fn tangent(&self, s: f64) -> DVector<f64> {
        match &self.kind {
            PathSegmentKind::Linear(l) => l.tangent(self.length),
            PathSegmentKind::Circular(c) => c.tangent(s),
        }
    }

    /// `getCurvature`.
    pub(crate) fn curvature(&self, s: f64) -> DVector<f64> {
        match &self.kind {
            PathSegmentKind::Linear(l) => l.curvature(),
            PathSegmentKind::Circular(c) => c.curvature(s),
        }
    }

    /// `getSwitchingPoints`. A [`Linear`] segment has none.
    pub(crate) fn switching_points(&self) -> Vec<f64> {
        match &self.kind {
            PathSegmentKind::Linear(_) => Vec::new(),
            PathSegmentKind::Circular(c) => c.switching_points(self.length),
        }
    }
}
