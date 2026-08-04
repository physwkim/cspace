// Copyright (c) 2016, Southwest Research Institute
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros-industrial/stomp @ b1a87c80f7338caae25a5c689b876da15492aa75:
//   include/stomp/utils.h
//   src/utils.cpp

//! STOMP's optimizer core, ported from the separate upstream repository
//! `ros-industrial/stomp` (not `moveit2`) -- see "Why a separate crate and a
//! separate upstream" below.
//!
//! # Round scope: `utils.cpp` only
//!
//! `ros-industrial/stomp` @ `b1a87c80f7338caae25a5c689b876da15492aa75` is
//! 1,551 lines across `src/stomp.cpp` (800), `src/utils.cpp` (179),
//! `include/stomp/{stomp,task,utils}.h` (572); confirmed ROS-independent by
//! grepping for `ros/ros.h`, `rclcpp`, and `ros::` across `src/utils.cpp`
//! and `include/stomp/utils.h` (zero matches) -- pure C++/Eigen, matching
//! PORTING-PLAN.md's D1. This round ports exactly `utils.h`/`utils.cpp`
//! (see [`utils`]'s own module doc for what that excludes even within
//! `utils.h` -- `Rollout`, `StompConfiguration`,
//! `TrajectoryInitializations`). The 800-line `stomp.cpp` optimizer loop
//! (`StompOptimizer`/`Stomp`, built on top of `utils.cpp`'s primitives) is
//! deferred to a following round.
//!
//! # Why a separate crate and a separate upstream
//!
//! `moveit2`'s own `moveit_planners/stomp/` (ported in `moveit-planners-stomp`)
//! only *binds to* STOMP's optimizer; it does not implement it. The actual
//! optimization loop and its math helpers live in `ros-industrial/stomp`, a
//! repository `moveit2` depends on but does not vendor. Every audit command
//! in this workspace (`tools/ci/count-relative-eq.pl`,
//! `count-public-declarations.sh`, and friends) counts a crate's symbols
//! against exactly one upstream; folding this content into
//! `moveit-planners-stomp` would put two upstreams (`moveit2` and
//! `ros-industrial/stomp`) in one crate and silently break that premise.
//! `moveit-stomp-core` exists so the one-crate-one-upstream invariant keeps
//! holding, and `moveit-planners-stomp` depends on it once
//! `filter_functions::simple_smoothing_matrix` is wired up (a following
//! round -- see `moveit-planners-stomp`'s own module doc for its current
//! "not ported: `stomp`, the optimizer core" note, written before this
//! upstream became available).
//!
//! # License: `Apache-2.0`, not this workspace's `BSD-3-Clause`
//!
//! Every other crate in this workspace ports `moveit2`
//! (`BSD-3-Clause`, `workspace.package.license`). `ros-industrial/stomp` is
//! `Apache-2.0` (its `LICENSE` file, `package.xml`'s `<license>Apache
//! 2.0</license>`, and every source file's own header agree). This crate's
//! `Cargo.toml` sets `license = "Apache-2.0"` explicitly rather than
//! inheriting `license.workspace = true`, and every file here carries an
//! `SPDX-License-Identifier: Apache-2.0` header instead of this workspace's
//! usual `BSD-3-Clause` -- labeling Apache-licensed code as BSD because it
//! happens to sit in a BSD-licensed workspace would misstate its actual
//! license. Root `Cargo.toml` records this exception where the rest of the
//! workspace's upstream pin lives. Checked by
//! `tools/ci/check-license-matches-upstream.sh`.
//!
//! # `assert_relative_eq!` reckoning (§79 convention, applied from the start)
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-stomp-core/src/*.rs
//! both=0 epsilon_only=6 max_relative_only=0 neither=0
//! ```
//!
//! Run for real against the tree as committed this round. All six are
//! `epsilon`-only, every one an exact algebraic invariant of the ported
//! math itself (identity-stencil rows, the smoothing matrix's post-scaling
//! diagonal) checked against a tiny FP-rounding epsilon (`1e-9`-`1e-12`),
//! not a measurement-sized tolerance -- this crate has no
//! randomized/empirical test this round to size a tolerance from.
//!
//! This crate's own first draft of [`utils::rows_to_string`]'s panic
//! message used a `\`-newline line continuation inside a Rust string
//! literal, which briefly made `count-relative-eq.pl` misreport
//! `epsilon_only=0` for this file (its string-stripping regex lacked `/s`,
//! so it could not cross the continuation and ran on to an unrelated later
//! quote, blanking the real calls in between). That was a bug in the
//! shared script, not in this crate's code, and it is now fixed at the
//! source (`c9780c7`, added the missing `/s`) rather than worked around
//! per-crate -- the fix moved workspace `both=` from 94 to 112, correcting
//! undercounts in `moveit-collision` (2 to 3) and `moveit-distance-field`
//! (27 to 44); this crate's own `epsilon_only=6` re-counts the same either
//! way. It was catchable at all only because `count-relative-eq.pl` has one
//! canonical copy (`tools/ci/`, enforced by
//! `check-audit-scripts-not-copied.sh`) -- six per-crate copies would have
//! made six different undercounts look like six different crates'
//! baselines instead of one shared bug.

/// `utils.h`/`utils.cpp` -- see this module's own doc for what's ported and
/// what's deferred.
pub mod utils;

pub use utils::{
    DerivativeOrder, FINITE_CENTRAL_DIFF_COEFFS, FINITE_DIFF_RULE_LENGTH,
    FINITE_FORWARD_DIFF_COEFFS, differentiate, generate_finite_difference_matrix,
    generate_smoothing_matrix, matrix_to_string, rows_to_string, to_vector, vector_to_string,
};
