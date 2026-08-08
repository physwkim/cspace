// Copyright 2008 Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from FCL `include/fcl/math/bv/OBB-inl.h` (`OBB<S>::operator+`) and
// geometric_shapes 2.3.3 `src/bodies.cpp` (`Box::intersectsRay`) — see
// `bodies.rs`'s provenance comment for how both sources were obtained.

//! `std::min`/`std::max`-compatible comparisons.
//!
//! # Deviation from upstream (a non-deviation, spelled out)
//!
//! `std::min(a, b)` is specified as `b < a ? b : a` and `std::max(a, b)` as
//! `a < b ? b : a`. Both are asymmetric in NaN: a NaN **first** argument
//! makes the comparison false and so is *returned*; a NaN **second**
//! argument also makes the comparison false and so is *discarded* in favor
//! of the first. [`f64::min`]/[`f64::max`] instead follow IEEE 754
//! `minNum`/`maxNum` — NaN is discarded wherever it sits, and propagates
//! only when both arguments are NaN.
//!
//! [`cxx_max`] reproduces `std::max`; use it for any site that ports a
//! literal `std::max` call, and get the operand order right — `acc.max(x)`
//! where upstream writes `std::max(x, acc)` diverges even though it reads
//! as equivalent. `OBB<S>::operator+`'s chained `std::max(std::max(extent[0],
//! extent[1]), extent[2])` (`OBB-inl.h:164-165`) is the case this module
//! exists for: a NaN at `extent[0]` propagates all the way out (first
//! operand of both the inner and outer call); a NaN at `extent[1]` or
//! `extent[2]` is silently discarded. See `bodies.rs`'s `OBB::extend_approx`
//! for the fix this bought. (No `cxx_min` here: nothing in this crate ports
//! a literal `std::min` call — every `.min(` site is either an integer or
//! the `fmax`/`fmin` case below. Add it back the day that changes.)
//!
//! # Not every upstream min/max is `std::min`/`std::max`
//!
//! `geometric_shapes::bodies::Box::intersectsRay` computes its ray-box `t`
//! bounds with `std::fmax`/`std::fmin`, not `std::max`/`std::min` — and says
//! why in its own comment (`bodies.cpp:717-718`): *"use fmax/fmin to handle
//! NaNs which can sneak in when dividing by d in tmpTmin and tmpTmax"*.
//! `fmax`/`fmin` are the C `<cmath>` functions, specified as IEEE 754
//! `minNum`/`maxNum` — the *same* rule [`f64::min`]/[`f64::max`] follow, not
//! [`cxx_max`]'s. A site porting `std::fmax`/`std::fmin` should use the
//! plain [`f64::max`]/[`f64::min`] methods directly and is correct as
//! ordinary Rust; reaching for [`cxx_max`] there would be the bug, not the
//! fix. Which of the two C++ families the upstream line calls is the actual
//! question when deciding whether a min/max site belongs to this module's
//! family at all.
//!
//! # Third copy, deferred consolidation
//!
//! This is the third copy of `cxx_min`/`cxx_max` in the workspace
//! (`moveit-trajectory::numeric` and `moveit-constraints::numeric` are the
//! other two, and both still carry `cxx_min`). `moveit-geometry` depends on
//! neither of those crates, and the only crate every numeric-comparison-
//! needing crate already shares is `moveit-error`, whose whole doc is
//! "Error types for moveit-rs" — these functions don't belong there either.
//! The structural fix is one shared crate; it is deferred, not skipped,
//! because adding a `moveit-numeric` workspace member means editing
//! `Cargo.toml`/`Cargo.lock` while five branches are divergent, and every
//! one of them would conflict on the lock at merge. Consolidate the three
//! copies (`moveit-trajectory::numeric`, `moveit-constraints::numeric`, this
//! module) into a shared crate once those branches land.

/// `std::max(a, b)`: `if a < b { b } else { a }`.
pub(crate) fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_as_first_argument_is_returned_by_max() {
        assert!(cxx_max(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn nan_as_second_argument_is_discarded_by_max() {
        assert_eq!(cxx_max(f64::MIN, f64::NAN), f64::MIN);
    }

    #[test]
    fn ordinary_values_match_normal_max() {
        assert_eq!(cxx_max(1.0, 2.0), 2.0);
        assert_eq!(cxx_max(2.0, 1.0), 2.0);
    }
}
