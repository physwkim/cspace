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
//! # Round scope: all of `stomp.cpp`/`task.h`/`utils.h` except `bodies::Body`-adjacent glue
//!
//! `ros-industrial/stomp` @ `b1a87c80f7338caae25a5c689b876da15492aa75` is
//! 1,551 lines across `src/stomp.cpp` (800), `src/utils.cpp` (179),
//! `include/stomp/{stomp,task,utils}.h` (572); confirmed ROS-independent by
//! grepping for `ros/ros.h`, `rclcpp`, and `ros::` across every file this
//! crate ports (zero matches) -- pure C++/Eigen, matching PORTING-PLAN.md's
//! D1. An earlier round ported `utils.h`/`utils.cpp`'s free functions; this
//! round adds the rest of `utils.h` ([`utils::Rollout`],
//! [`utils::StompConfiguration`], [`utils::TrajectoryInitialization`]),
//! `task.h` in full ([`task::Task`]), and `stomp.cpp`/the remainder of
//! `stomp.h` in full ([`stomp::Stomp`], [`stomp::CancelHandle`]) -- see each
//! module's own doc for translation-level detail and preserved upstream
//! quirks.
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
//! holding; `moveit-planners-stomp` now depends on it and calls
//! `generate_smoothing_matrix` from `filter_functions::simple_smoothing_matrix`
//! -- see `moveit-planners-stomp`'s own module doc.
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
//!
//! # Symbol-completeness audit (round 26)
//!
//! This crate had never received a `moveit-scene`-style symbol-by-symbol
//! audit before this round -- it is the only crate in the workspace with a
//! different upstream and a different license, so no existing audit tool's
//! output covered it. Reference repo re-verified exactly at the pin this
//! round, not just close to it:
//!
//! ```text
//! $ git -C /home/stevek/work/stomp log --oneline -1
//! b1a87c8 Merge pull request #18 from mosfet80/patch-3
//! $ git -C /home/stevek/work/stomp rev-list --count b1a87c80...HEAD
//! 0
//! ```
//!
//! `HEAD` *is* `b1a87c80f7338caae25a5c689b876da15492aa75`, not merely close
//! to it -- no "checkout has drifted past the pin" gap to account for.
//!
//! Every symbol in `stomp.h`, `stomp.cpp`, `task.h`, and `utils.h` (`utils.cpp`
//! adds nothing beyond `utils.h`'s own declarations) is enumerated and
//! classified in the respective module's own doc -- see [`stomp`]'s
//! "Completeness audit" section (`stomp.h` + `stomp.cpp`, 47 symbols),
//! [`task`]'s (`task.h`, 10 symbols), and [`utils`]'s (`utils.h`, 14
//! symbols). `examples/` and `test/utest.cpp` are outside this audit's
//! scope (item 2 of this round's brief separately ports `test/stomp_3dof.cpp`
//! as an acceptance test, not a symbol-completeness target).
//!
//! ```text
//! rg -c '^\s*//! - ' crates/moveit-stomp-core/src/stomp.rs
//! 47
//! rg -c '^\s*//! - ' crates/moveit-stomp-core/src/task.rs
//! 10
//! rg -c '^\s*//! - ' crates/moveit-stomp-core/src/utils.rs
//! 14
//! ```
//!
//! 47 + 10 + 14 = 71, and 71 is independently the sum of every upstream
//! symbol counted by the `rg` commands shown in each module's own audit
//! section (`stomp.h`'s 40 + `stomp.cpp`'s 7 file-local = 47; `task.h`'s 10;
//! `utils.h`'s 14) -- the two counts (bullets written, symbols found in
//! upstream) agree by construction, not by coincidence, since each bullet
//! cites the exact upstream symbol it classifies. Of the 71: **67 `ported
//! as`, 4 `distinct`** -- 2 in [`stomp`] (the `Eigen::VectorXd` `solve`
//! overload, subsumed by `&[f64]`; `num_timesteps_padded_`, collapsed to a
//! local) and 2 in [`task`] (`TaskPtr`'s typedef; `Task()`'s trivial
//! constructor -- see each module's own doc for the per-symbol reasoning).
//! Zero `unported, in scope`. Zero `D1 exclusion` -- unlike `moveit_core`
//! headers, nothing in `stomp.h`/`stomp.cpp`/`task.h`/`utils.h` touches a
//! ROS message type or any other D1-excluded dependency; this crate's own
//! module doc above already confirmed that by grepping for
//! `ros/ros.h`/`rclcpp`/`ros::` across every ported file with zero matches.

/// `utils.h`/`utils.cpp` -- see this module's own doc for what's ported and
/// what's deferred.
pub mod utils;

/// `task.h` -- see this module's own doc for the `Task` trait's
/// out-parameter shape split.
pub mod task;

/// `stomp.h`/`stomp.cpp` -- see this module's own doc for `Stomp`'s
/// preserved upstream quirks and `CancelHandle`'s thread-safety rationale.
pub mod stomp;

pub use stomp::{CancelHandle, Stomp};
pub use task::Task;
pub use utils::{
    DerivativeOrder, FINITE_CENTRAL_DIFF_COEFFS, FINITE_DIFF_RULE_LENGTH,
    FINITE_FORWARD_DIFF_COEFFS, Rollout, StompConfiguration, TrajectoryInitialization,
    differentiate, full_piv_lu_try_inverse_or_empty, generate_finite_difference_matrix,
    generate_smoothing_matrix, matrix_to_string, rows_to_string, to_vector, vector_to_string,
};
