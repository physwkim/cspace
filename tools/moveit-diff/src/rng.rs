// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! A seeded PRNG, inlined so a differential run is reproducible from its seed
//! alone without pulling `rand` into the workspace.
//!
//! This is xoshiro256++ with a SplitMix64 seeder. It is used only to pick
//! joint values for test cases — never for anything a planner depends on.

/// xoshiro256++ state.
#[derive(Debug, Clone)]
pub struct Rng {
    s: [u64; 4],
}

impl Rng {
    /// Seed the generator. The same seed always yields the same case sequence.
    pub fn new(seed: u64) -> Self {
        let mut z = seed;
        let mut next = || {
            z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            x ^ (x >> 31)
        };
        Self {
            s: [next(), next(), next(), next()],
        }
    }

    fn next_u64(&mut self) -> u64 {
        let result = self.s[0]
            .wrapping_add(self.s[3])
            .rotate_left(23)
            .wrapping_add(self.s[0]);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        // 53 significant bits, the most an f64 can hold exactly.
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform in `[lo, hi)`. Returns `lo` when the range is empty or
    /// non-finite, which is what an unbounded continuous joint needs.
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        if !lo.is_finite() || !hi.is_finite() || hi <= lo {
            return lo;
        }
        lo + self.unit() * (hi - lo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_sequence() {
        let a: Vec<f64> = (0..16).scan(Rng::new(42), |r, _| Some(r.unit())).collect();
        let b: Vec<f64> = (0..16).scan(Rng::new(42), |r, _| Some(r.unit())).collect();
        assert_eq!(a, b);
    }

    #[test]
    fn different_seed_different_sequence() {
        let a: Vec<f64> = (0..16).scan(Rng::new(1), |r, _| Some(r.unit())).collect();
        let b: Vec<f64> = (0..16).scan(Rng::new(2), |r, _| Some(r.unit())).collect();
        assert_ne!(a, b);
    }

    #[test]
    fn unit_is_in_range() {
        let mut r = Rng::new(7);
        for _ in 0..10_000 {
            let v = r.unit();
            assert!((0.0..1.0).contains(&v), "{v}");
        }
    }

    #[test]
    fn range_respects_bounds_and_degenerate_input() {
        let mut r = Rng::new(9);
        for _ in 0..10_000 {
            let v = r.range(-2.5, 1.25);
            assert!((-2.5..1.25).contains(&v), "{v}");
        }
        assert_eq!(r.range(3.0, 3.0), 3.0);
        assert_eq!(r.range(5.0, 1.0), 5.0);
        assert_eq!(r.range(f64::NEG_INFINITY, f64::INFINITY), f64::NEG_INFINITY);
    }
}
