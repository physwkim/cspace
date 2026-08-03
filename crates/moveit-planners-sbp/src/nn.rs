// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Nearest-neighbour index over an arbitrary [`StateSpace`] metric.
//!
//! A kd-tree prunes by comparing one coordinate axis against a threshold,
//! which presumes a state decomposes into independent scalar axes that
//! `distance` treats the way Euclidean distance does. [`StateSpace::distance`]
//! promises only that it is *a metric* (see its docs): non-negative,
//! symmetric, triangle-inequality-respecting, nothing about axes. That
//! promise breaks the moment a joint wraps around (distance is the shorter
//! arc, not a linear difference) or composes an orientation (a geodesic on
//! SO(3), not a per-axis blend) — a kd-tree built by splitting on raw
//! coordinate values would prune subtrees that a wraparound or geodesic
//! metric still considers close. OMPL's GNAT (Geometric Near-neighbor
//! Access Tree, Brin 1995) exists for exactly this reason: it partitions
//! purely by relative distance to chosen pivots, so it needs nothing beyond
//! the metric. This module is a GNAT-family index for the same reason.
//!
//! # Deviation from the GNAT paper
//!
//! This is a simplified single-pivot-per-node member of the GNAT family, not
//! a transcription of Brin's construction: each node holds one pivot, a
//! covering radius (the maximum distance from the pivot to any point in its
//! own subtree) and up to `degree` children, and a new point is routed to
//! its nearest existing child once a node is full. Brin's GNAT additionally
//! precomputes distance *ranges* between every pair of sibling pivots at a
//! level, so a query can rule out several siblings after measuring distance
//! to just one of them. This version measures distance to every child's
//! pivot before applying the covering-radius bound, so it prunes less at
//! query time than the full construction. That is a performance gap, not a
//! correctness one: the covering-radius bound in `Node::search`'s doc
//! comment (a private implementation detail, not part of this crate's
//! public API) is exact, so `nearest` always agrees with a brute-force
//! scan — the property this module's tests check directly.

use crate::space::StateSpace;

/// A GNAT-family nearest-neighbour index over `S::State`, each entry
/// carrying a caller-supplied payload `T` (in [`crate::rrt_connect`], a
/// tree-node index).
pub struct Gnat<S: StateSpace, T> {
    root: Option<Node<S::State, T>>,
    degree: usize,
    len: usize,
}

struct Node<P, T> {
    pivot: P,
    data: T,
    /// Max distance from `pivot` to any point in this node's own subtree,
    /// including further-nested descendants. Maintained incrementally: every
    /// insertion updates this at every node on its path down the tree.
    radius: f64,
    children: Vec<Node<P, T>>,
}

impl<S: StateSpace, T> Gnat<S, T> {
    /// `degree` bounds how many direct children a node accumulates before
    /// further insertions are routed into an existing child instead of
    /// becoming a new sibling.
    ///
    /// # Panics
    /// If `degree == 0` (a node could then never accept a child and the
    /// index would degenerate to an unindexed list).
    pub fn new(degree: usize) -> Self {
        assert!(degree > 0, "Gnat degree must be at least 1, got 0");
        Self {
            root: None,
            degree,
            len: 0,
        }
    }

    /// Number of points stored.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the index holds no points.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Inserts `state` with payload `data`.
    pub fn insert(&mut self, space: &S, state: S::State, data: T) {
        match &mut self.root {
            None => {
                self.root = Some(Node {
                    pivot: state,
                    data,
                    radius: 0.0,
                    children: Vec::new(),
                });
            }
            Some(root) => root.insert(space, state, data, self.degree),
        }
        self.len += 1;
    }

    /// The stored `(state, data)` closest to `query` under `space`'s metric,
    /// or `None` if the index is empty.
    pub fn nearest(&self, space: &S, query: &S::State) -> Option<(&S::State, &T)> {
        let root = self.root.as_ref()?;
        let mut best: Option<(&Node<S::State, T>, f64)> = None;
        root.search(space, query, &mut best);
        best.map(|(node, _)| (&node.pivot, &node.data))
    }
}

impl<P: Clone, T> Node<P, T> {
    fn insert<S: StateSpace<State = P>>(&mut self, space: &S, state: P, data: T, degree: usize) {
        let d = space.distance(&self.pivot, &state);
        if d > self.radius {
            self.radius = d;
        }
        if self.children.len() < degree {
            self.children.push(Node {
                pivot: state,
                data,
                radius: 0.0,
                children: Vec::new(),
            });
            return;
        }
        // Route to the child whose pivot is nearest the new point: this
        // keeps each subtree's points clustered around its own pivot, which
        // is what keeps the covering radius (and therefore the pruning
        // bound in `search`) tight.
        let nearest = self
            .children
            .iter()
            .map(|c| space.distance(&c.pivot, &state))
            .enumerate()
            .min_by(|a, b| {
                a.1.partial_cmp(&b.1)
                    .expect("StateSpace::distance must not return NaN")
            })
            .map(|(index, _)| index)
            .expect("children is non-empty: this branch only runs once len == degree > 0");
        self.children[nearest].insert(space, state, data, degree);
    }

    /// Best-first search with covering-radius pruning.
    ///
    /// For a child `c` with pivot `p_c` and covering radius `r_c`, every
    /// point `x` in `c`'s subtree satisfies `distance(x, p_c) <= r_c` by
    /// definition of the covering radius, so the triangle inequality gives
    ///
    /// ```text
    /// distance(query, p_c) <= distance(query, x) + distance(x, p_c)
    ///                       <= distance(query, x) + r_c
    /// ```
    ///
    /// i.e. `distance(query, x) >= distance(query, p_c) - r_c` for every `x`
    /// in the subtree. If that lower bound already exceeds the best distance
    /// found so far, no point in the subtree can improve on it, so the whole
    /// subtree is skipped without being visited. Children are visited
    /// closest-pivot-first, which tends to establish a tight `best` early
    /// and prune more of what follows — but the bound above is what makes
    /// pruning *safe*, not the visiting order.
    fn search<'a, S: StateSpace<State = P>>(
        &'a self,
        space: &S,
        query: &P,
        best: &mut Option<(&'a Node<P, T>, f64)>,
    ) {
        let d = space.distance(&self.pivot, query);
        let improves = match best {
            Some((_, best_d)) => d < *best_d,
            None => true,
        };
        if improves {
            *best = Some((self, d));
        }

        let mut candidates: Vec<(f64, &Node<P, T>)> = self
            .children
            .iter()
            .map(|c| (space.distance(&c.pivot, query), c))
            .collect();
        candidates.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .expect("StateSpace::distance must not return NaN")
        });

        for (dc, child) in candidates {
            let lower_bound = dc - child.radius;
            let prune = match best {
                Some((_, best_d)) => lower_bound > *best_d,
                None => false,
            };
            if !prune {
                child.search(space, query, best);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space::RealVectorSpace;
    use rand::{RngExt, SeedableRng};
    use rand_chacha::ChaCha8Rng;

    fn brute_force_nearest<'a>(
        space: &RealVectorSpace,
        points: &'a [(Vec<f64>, usize)],
        query: &[f64],
    ) -> &'a (Vec<f64>, usize) {
        points
            .iter()
            .min_by(|a, b| {
                space
                    .distance(&a.0, &query.to_vec())
                    .partial_cmp(&space.distance(&b.0, &query.to_vec()))
                    .unwrap()
            })
            .expect("points is non-empty")
    }

    #[test]
    fn empty_index_has_no_nearest() {
        let space = RealVectorSpace::new(vec![(-1.0, 1.0)]).unwrap();
        let gnat: Gnat<RealVectorSpace, usize> = Gnat::new(4);
        assert!(gnat.nearest(&space, &vec![0.0]).is_none());
    }

    #[test]
    fn single_point_is_its_own_nearest() {
        let space = RealVectorSpace::new(vec![(-1.0, 1.0)]).unwrap();
        let mut gnat: Gnat<RealVectorSpace, usize> = Gnat::new(4);
        gnat.insert(&space, vec![0.25], 7);
        let (state, data) = gnat.nearest(&space, &vec![-1.0]).unwrap();
        assert_eq!(state, &vec![0.25]);
        assert_eq!(*data, 7);
    }

    #[test]
    fn len_and_is_empty_track_insertions() {
        let space = RealVectorSpace::new(vec![(-1.0, 1.0)]).unwrap();
        let mut gnat: Gnat<RealVectorSpace, usize> = Gnat::new(4);
        assert!(gnat.is_empty());
        gnat.insert(&space, vec![0.0], 0);
        gnat.insert(&space, vec![0.5], 1);
        assert_eq!(gnat.len(), 2);
        assert!(!gnat.is_empty());
    }

    /// The property that matters: for thousands of random point sets and
    /// queries, in 1, 3 and 8 dimensions and with tree degrees from 2 (deep,
    /// narrow trees that exercise routing the most) to 32 (shallow, wide
    /// trees), `Gnat::nearest` agrees exactly with a brute-force scan.
    #[test]
    fn nearest_agrees_with_brute_force() {
        let mut rng = ChaCha8Rng::seed_from_u64(0xC0FFEE);
        for &dim in &[1usize, 3, 8] {
            for &degree in &[2usize, 4, 32] {
                let bounds: Vec<(f64, f64)> = (0..dim).map(|_| (-100.0, 100.0)).collect();
                let space = RealVectorSpace::new(bounds).unwrap();

                let mut gnat: Gnat<RealVectorSpace, usize> = Gnat::new(degree);
                let mut points: Vec<(Vec<f64>, usize)> = Vec::new();
                for i in 0..300 {
                    let p: Vec<f64> = (0..dim).map(|_| rng.random_range(-100.0..=100.0)).collect();
                    gnat.insert(&space, p.clone(), i);
                    points.push((p, i));
                }

                for _ in 0..2000 {
                    let query: Vec<f64> =
                        (0..dim).map(|_| rng.random_range(-100.0..=100.0)).collect();
                    let (expected_state, expected_data) =
                        brute_force_nearest(&space, &points, &query);
                    let (got_state, got_data) = gnat.nearest(&space, &query).unwrap();
                    let expected_dist = space.distance(expected_state, &query);
                    let got_dist = space.distance(got_state, &query);
                    // Compare by distance, not by identity: several points
                    // can be exactly equidistant, and any of them is a
                    // correct answer. `Gnat` and brute force must still
                    // agree on the *distance* of the nearest point.
                    assert!(
                        (expected_dist - got_dist).abs() < 1e-9,
                        "dim={dim} degree={degree}: brute force found {expected_state:?} (data {expected_data}, dist {expected_dist}), \
                         Gnat found {got_state:?} (data {got_data}, dist {got_dist})"
                    );
                }
            }
        }
    }
}
