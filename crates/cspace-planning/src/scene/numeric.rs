// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from Eigen 3.4.0 (MPL2) — not one of this workspace's four listed
// upstreams (moveit2, stomp, ompl, fcl), but its text is in-tree all the
// same, as the copy bullet3 bundles under
// third_party/bullet3/examples/ThirdPartyLibs/Eigen. See `scene.rs`'s
// `isometry_is_approx` for why this formula is duplicated from Eigen rather
// than reused from `cspace-collision`.

//! `std::min`-compatible comparison.
//!
//! Same contract as `cspace_core::trajectory::numeric`'s [`cxx_min`] — see that
//! module's doc comment for the full NaN-asymmetry argument. Duplicated
//! here rather than shared: promoting either copy to a common crate is a
//! cross-crate API change out of scope for this pass.
//!
//! One site in this crate needed the fix, `scene.rs`'s
//! `isometry_is_approx`: `norm_a.min(norm_b)`, matching Eigen's
//! `Eigen/src/Core/Fuzzy.h:27` (`numext::mini(nested.cwiseAbs2().sum(),
//! otherNested.cwiseAbs2().sum())`), which on a non-GPU build resolves to
//! plain `std::min` (`Eigen/src/Core/MathFunctions.h:1083`). Both spans are
//! 3.4.0's, the release bullet3 bundles under
//! `third_party/bullet3/examples/ThirdPartyLibs/Eigen` and the one every
//! other Eigen citation in this workspace already reads; the reformatted
//! copy inside ITK that these two were first taken from wraps the same
//! statements onto different lines. `norm_a`/`norm_b` are
//! runtime sums of squares, not constants — but this is
//! fidelity/uniformity only, not independently reachable: `isometry_is_approx`'s
//! `diff_sq` sums over the same matrix entries as `norm_a`/`norm_b`, so any
//! NaN that would poison a receiver here also poisons `diff_sq`, and
//! `diff_sq <= _` is already `false` before this `min`'s result is ever
//! consulted. Confirmed empirically with a scratch probe (since deleted),
//! not by inspection — see `cspace_core::trajectory::numeric`'s own doc comment
//! for why that distinction matters.

/// `std::min(a, b)`: `if b < a { b } else { a }`.
pub(crate) fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_as_first_argument_is_returned() {
        assert!(cxx_min(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn nan_as_second_argument_is_discarded() {
        assert_eq!(cxx_min(f64::MAX, f64::NAN), f64::MAX);
    }

    #[test]
    fn ordinary_values_match_normal_min() {
        assert_eq!(cxx_min(1.0, 2.0), 1.0);
        assert_eq!(cxx_min(2.0, 1.0), 1.0);
    }
}
