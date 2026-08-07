// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Shared surface between `moveit-diff`'s default binary and its
//! `src/bin/*.rs` siblings. Each one spawns the oracle as a JSON-lines
//! subprocess through `run-oracle.sh` (`docker run --rm -i ...`), and each
//! one had its own copy-pasted `Drop for Oracle` ending in a bare
//! `self.child.wait()` with no bound. See [`wait_or_kill`]'s own doc for why
//! that is a defect and not a style choice, and why the fix belongs here
//! rather than in any one of them.

use std::process::Child;
use std::time::{Duration, Instant};

/// How long [`wait_or_kill`] gives a child whose stdin has already been
/// closed before concluding it will not exit on its own and killing it.
/// Generous relative to every teardown this repo has ever measured
/// (`docker run --rm`'s container stop is sub-second in every wall clock
/// `verify-phase3-*.sh` prints), tiny relative to a CI job's own budget.
/// Picked as a hard, legible bound rather than a tuned one -- narrow it with
/// evidence, not intuition.
pub const ORACLE_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(30);

/// Waits for `child` to exit, killing it if it has not by `timeout`.
///
/// Every `Oracle::drop` in this package used to end in a bare
/// `self.child.wait()` after closing the child's stdin -- correct only if
/// the child reliably exits once its stdin closes. Two things can make that
/// false, and neither is hypothetical enough to leave unbounded:
///
/// - `oracle.cpp`'s main loop is `while (std::getline(std::cin, line))` with
///   no per-request bound. A request that never returns from
///   `oracle->handle(request)` leaves the process blocked *inside* that
///   call, never back at `getline` to observe the stdin close at all.
/// - The oracle runs inside `docker run --rm -i` (`run-oracle.sh`), so
///   `wait()` also depends on the `docker` CLI's own attach/teardown
///   returning -- which can block on a daemon that is itself stuck,
///   independent of whether the containerized process has exited.
///
/// No caller in this repository bounds either case: not `verify-all.sh`,
/// not `ci.yml` (no `timeout-minutes` on its one job), not `gate-lib.sh`.
/// An unbounded `wait()` here was therefore the only bound that could ever
/// have applied, and it was not one. A hang is worse than a failure -- a
/// failure is a verdict, a hang is a run that consumes its caller's whole
/// time budget and produces neither.
pub fn wait_or_kill(child: &mut Child, timeout: Duration) {
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) => {
                if Instant::now() >= deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    return;
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            // Nothing further to wait for, and `Drop` cannot itself fail --
            // there is no caller here to report this to.
            Err(_) => return,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    #[test]
    fn a_child_that_exits_on_its_own_is_not_held_to_the_full_timeout() {
        let mut child = Command::new("true").spawn().expect("spawn `true`");
        let start = Instant::now();
        wait_or_kill(&mut child, Duration::from_secs(5));
        assert!(
            start.elapsed() < Duration::from_secs(1),
            "a child that already exited must not wait out a 5s timeout"
        );
    }

    #[test]
    fn a_child_that_outlives_the_timeout_is_killed_rather_than_awaited_forever() {
        let mut child = Command::new("sleep")
            .arg("60")
            .spawn()
            .expect("spawn `sleep 60`");
        let start = Instant::now();
        wait_or_kill(&mut child, Duration::from_millis(200));
        let elapsed = start.elapsed();
        assert!(
            elapsed >= Duration::from_millis(200),
            "must not return before its own timeout elapses: {elapsed:?}"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "must not wait anywhere near the child's own 60s sleep: {elapsed:?}"
        );
    }
}
