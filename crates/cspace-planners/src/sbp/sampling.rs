// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Random-sampling primitives shared by more than one `StateSpace`
//! implementation.
//!
//! Every space that needs "a random direction" draws on the functions here
//! rather than reinventing its own version: `RealVectorSpace::sample_near`'s
//! box-vs-ball and rejection-vs-Box-Muller history earlier in this crate's
//! git log is exactly the kind of defect duplicating this logic invites.

use rand::{Rng, RngExt};

/// One sample from the standard normal distribution, via the Box-Muller
/// transform. Only the cosine branch is used (the sine branch would give a
/// second independent sample for free, unused here), which costs one extra
/// uniform draw beyond the theoretical minimum -- irrelevant next to the
/// `O(dim)`, no-rejection cost of the callers below.
pub(crate) fn standard_normal(rng: &mut dyn Rng) -> f64 {
    let u1: f64 = rng.random_range(f64::MIN_POSITIVE..1.0);
    let u2: f64 = rng.random_range(0.0..1.0);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// A direction drawn uniformly on the `dim`-dimensional unit sphere.
///
/// Independent standard-normal coordinates are spherically symmetric (their
/// joint density depends only on the vector's norm), so normalizing such a
/// vector to unit length gives a direction uniform on the sphere exactly:
/// `dim` draws, no rejection, no dimension-dependent blowup.
///
/// # Panics
/// If `dim == 0`.
pub(crate) fn sample_unit_vector(rng: &mut dyn Rng, dim: usize) -> Vec<f64> {
    assert!(dim > 0, "sample_unit_vector needs dim > 0, got 0");
    let mut v: Vec<f64> = (0..dim).map(|_| standard_normal(rng)).collect();
    let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    for x in &mut v {
        *x /= norm;
    }
    v
}

/// A radius fraction in `[0, 1)`, distributed so that multiplying a ball's
/// radius by this fraction and using the result as a draw's distance from
/// centre makes the resulting point uniform over the ball's *volume* in
/// `dim` dimensions -- not concentrated at the outer shell (which a plain
/// uniform `[0, radius)` draw would produce in high dimension) and not
/// concentrated at the centre.
///
/// Volume enclosed within radius `r` scales as `r^dim`, so `u^(1/dim)` for
/// `u` uniform in `[0, 1)` is the inverse-CDF transform that makes the
/// enclosed volume, and therefore the sample, uniform. See e.g. Barthe et
/// al., "A Probabilistic Approach to the Geometry of the l^n_p-Ball" (2005).
pub(crate) fn sample_ball_radius_fraction(rng: &mut dyn Rng, dim: usize) -> f64 {
    let u: f64 = rng.random_range(0.0..1.0);
    u.powf(1.0 / dim as f64)
}

/// `n` nonnegative fractions summing to `1.0`, drawn uniformly from the
/// `(n - 1)`-simplex -- e.g. `CompoundSpace`'s own `sample_near`
/// splitting a radius budget fairly across a heterogeneous set of subspaces.
///
/// `n` independent `Exponential(1)` draws (`-ln(uniform)`), normalized to
/// sum to `1`, is a standard construction for a uniform (`Dirichlet(1, ...,
/// 1)`) draw from the simplex: the `Gamma(1, 1)` distribution is exactly
/// `Exponential(1)`, and normalizing independent Gamma draws with a common
/// shape parameter is the standard Gamma-to-Dirichlet construction.
///
/// # Panics
/// If `n == 0`.
pub(crate) fn sample_simplex(rng: &mut dyn Rng, n: usize) -> Vec<f64> {
    assert!(n > 0, "sample_simplex needs n > 0, got 0");
    let draws: Vec<f64> = (0..n)
        .map(|_| {
            let u: f64 = rng.random_range(f64::MIN_POSITIVE..1.0);
            -u.ln()
        })
        .collect();
    let sum: f64 = draws.iter().sum();
    draws.into_iter().map(|d| d / sum).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    #[test]
    fn unit_vector_has_unit_norm() {
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        for dim in [1usize, 2, 3, 5, 15] {
            for _ in 0..200 {
                let v = sample_unit_vector(&mut rng, dim);
                let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
                assert!((norm - 1.0).abs() < 1e-9, "dim {dim}: norm {norm}");
            }
        }
    }

    #[test]
    fn ball_radius_fraction_is_within_unit_interval() {
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        for dim in [1usize, 3, 15] {
            for _ in 0..2000 {
                let f = sample_ball_radius_fraction(&mut rng, dim);
                assert!((0.0..1.0).contains(&f), "dim {dim}: fraction {f}");
            }
        }
    }

    #[test]
    fn simplex_fractions_are_nonnegative_and_sum_to_one() {
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        for n in [1usize, 2, 5, 10] {
            for _ in 0..2000 {
                let fractions = sample_simplex(&mut rng, n);
                assert_eq!(fractions.len(), n);
                let sum: f64 = fractions.iter().sum();
                assert!(
                    (sum - 1.0).abs() < 1e-9,
                    "n {n}: fractions {fractions:?} sum to {sum}"
                );
                for &f in &fractions {
                    assert!(
                        (0.0..=1.0).contains(&f),
                        "n {n}: fraction {f} out of [0, 1]"
                    );
                }
            }
        }
    }
}
