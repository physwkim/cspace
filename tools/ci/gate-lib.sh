# shellcheck shell=bash
# Shared guard for the gate scripts in this directory. Source it:
#
#   . "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"
#
# Deliberately named neither `check-*` nor `verify-*`: it is a library, and
# both of those names are globs that run every match as a gate.

# Fails the calling gate when the set it was about to examine is empty.
#
# Every gate here ends with an unconditional `echo OK`, so a loop that runs
# zero times prints the same line as a loop that checked every file --
# "green" then means "nothing was examined", which is the one outcome a gate
# must never spell the same way as success. `verify-vendored-fixture-tests.sh`
# and `verify-all.sh` each grew this guard for their own list after the fact;
# this is that rule in one place, so a new gate gets it by calling one
# function rather than by its author remembering the failure mode.
#
# The current sets are all non-empty (the workspace has crates, the fixture
# directories have fixtures), so no call site below fires today. That is the
# point: the loops are driven off the filesystem precisely so they pick up new
# files without anyone updating a list, and the same property makes them go
# quietly empty if a path convention changes. The guard is what turns that
# into a failure instead of a pass.
#
# What this guard does NOT cover, and which gate needs what:
#
#   - An empty list because the *producer* failed. `mapfile -t x < <(cmd)`
#     throws away `cmd`'s exit status and `set -e` cannot see it, so a broken
#     `git ls-files` yields an empty array with no error anywhere. Where a
#     non-empty list is the failure condition, `require_nonempty` catches that
#     as a side effect -- the list is empty either way and the gate fails
#     either way. That is the only reason those sites are safe.
#   - Where an *empty* list is the pass condition, `require_nonempty` is not
#     applicable and there is nothing left to notice the producer's failure.
#     `check-audit-scripts-not-copied.sh` was the one such gate here and it
#     passed vacuously in a `.git`-less tree until it was rewritten to read the
#     list through a checked command substitution.
#
# So: `require_nonempty` asserts what the gate expected to find, not that the
# command that produced it worked. Read every list with `x="$(cmd)"` (under
# `set -e` and `pipefail`, that propagates) or an explicit `if ! x="$(cmd)"`,
# and use `require_nonempty` for the separate question of whether the result
# should have been non-empty.
require_nonempty() {
  local count="$1" what="$2"
  if [ "$count" -eq 0 ]; then
    echo "FAIL found no $what -- this gate would report OK having examined nothing." >&2
    echo "FAIL either the path convention changed or the tree is not what this gate assumes." >&2
    exit 1
  fi
}

# Fails the calling gate when it is about to examine a different worktree of
# this repository than the caller is standing in.
#
# Every gate here derives its root from its own path
# (`dirname "${BASH_SOURCE[0]}"`), which is correct when it is invoked as
# `tools/ci/foo.sh` from the tree under test and silently wrong when it is
# invoked by an absolute path from somewhere else. The gates that need docker
# must be invoked that way -- `sg docker -c` takes a command string, not a
# working directory -- so a worker in a worktree running
# `sg docker -c '/home/stevek/work/moveit-rs/tools/ci/verify-ros-interop.sh'`
# gates the session root instead. Observed: that command printed
# `all gates passed` with exit 0 and 174/174 tests, against a tree containing
# none of the caller's changes. A green measurement of the wrong subject is
# worse than a red one, because nothing about it looks wrong.
#
# The check is deliberately narrow. Being outside a git tree is allowed (a
# release tarball has no repository to disagree with), and so is standing in an
# unrelated repository -- gating this tree from `~/work/moveit2` is odd but not
# a mistake this guard can distinguish from a deliberate one. The failure is
# the specific case it can prove: caller and gate are in two different
# worktrees that share one `.git`, which is exactly the caucus layout and the
# only shape where "the gate ran against my changes" is false while everything
# printed says otherwise.
require_caller_tree() {
  local repo_root="$1" caller_root gate_common caller_common
  caller_root="$(git -C "$PWD" rev-parse --show-toplevel 2>/dev/null || true)"
  [ -n "$caller_root" ] || return 0
  [ "$caller_root" != "$repo_root" ] || return 0
  gate_common="$(git -C "$repo_root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)"
  caller_common="$(git -C "$caller_root" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)"
  [ -n "$gate_common" ] && [ "$gate_common" = "$caller_common" ] || return 0
  echo "FAIL this gate would examine $repo_root, but you are standing in $caller_root." >&2
  echo "FAIL both are worktrees of the same repository, so it would report on changes that are not yours." >&2
  echo "FAIL run that worktree's own copy instead: ${caller_root}/${BASH_SOURCE[1]#"$repo_root"/}" >&2
  exit 1
}

# `docker run` with the container's cargo output redirected OFF the
# bind-mounted worktree. Use it for every container that runs cargo, or runs
# something cargo built, against a mounted repo:
#
#   docker_cargo_run --rm -v "$REPO_ROOT:/repo" -w /repo/ros/cspace-ros ...
#
# taking everything `docker run` itself would take. Never spell a built
# binary `./target/debug/...`; which of the two names below to use depends on
# WHICH SHELL expands it, not on taste:
#
#   - inside the container's own command (a `bash -c '...'` single-quoted
#     word, or a `<<'EOS'` heredoc): `"$CARGO_TARGET_DIR/debug/move_group"`.
#   - as argv the host shell builds (`... "$IMAGE" <binary> "$URDF"`, the
#     shape the detached `-d` node containers use):
#     `"$DOCKER_CARGO_TARGET_MOUNT/debug/move_group"`. `CARGO_TARGET_DIR` is
#     set in the container and NOT on the host, so the first spelling dies
#     there under `set -u` -- measured, as `ros/verify-robot-description-
#     interop.sh: line 205: CARGO_TARGET_DIR: unbound variable`.
#
# A function rather than an argument array so that a call site cannot get
# half of it: the mount and the env var have to travel together (the volume
# alone leaves cargo writing to the tree; the variable alone points cargo at
# a path no volume backs), and a `"${ARGS[@]}"` a caller forgets to paste is
# exactly the omission this whole block exists to stop. An array here would
# also read as unused to the linter, whose only uses are in other files.
#
# The `ros/` gates run as root -- that is the ros-dev image's only user, and
# its `CARGO_HOME=/usr/local/cargo` is root-owned, so `--user` leaves cargo
# unable to write its registry. With the repo bind-mounted, cargo's default
# `ros/cspace-ros/target` therefore writes root-owned files into the *host's*
# tree. Measured on 2026-08-08: ~242k of them across this repo and its caucus
# worktrees, accumulated over earlier gate runs. `git status` stays clean the
# whole time (it is all gitignored build output), and the cost lands on
# somebody else -- the host user's next `cargo` fails with `Permission
# denied`, and they cannot `chown` it back without another container.
#
# A docker-managed named volume removes the shared writable path rather than
# guarding it: there is no longer a host directory for the container to own.
# One volume shared across a gate's containers preserves exactly the `target/`
# reuse the bind mount used to provide, so `fmt`/`clippy`/`test`/`doc` still
# do not each rebuild the dependency graph. It is not a durable cache --
# `CARGO_HOME` is container-local and `--rm`, so the registry is re-fetched
# every run either way.
#
# Here rather than per script for the reason `require_nonempty` is: six
# `ros/verify-*-interop.sh` scripts made 20 `docker run` calls between them,
# and every one of the 17 that runs the ros-dev image had this defect. A fix
# applied to the one call that was noticed would have left the other five
# scripts writing into the tree.
#
# The remaining 3 run `$PROBE_IMAGE` (upstream's C++ `MoveGroupInterface`
# probe, built into its own image at /ws) and take no cargo argument: they
# never invoke cargo, so they have nothing to redirect.
#
# One volume per worktree, not one global volume. Seven caucus worktrees share
# this docker daemon -- the live legs already derive a per-run
# `ROS_DOMAIN_ID` for that reason -- and a single shared target directory
# would let one worktree's `cargo build` replace the `move_group` binary
# another worktree's live leg is about to launch. `require_caller_tree` above
# exists because a green measurement of the wrong subject is worse than a red
# one; a build cache shared across worktrees is that same failure one level
# down, and the bind mount it replaces did not have it. The name is derived
# from the tree holding THIS copy of gate-lib.sh, so each worktree gets its
# own without any caller passing anything, and `docker volume prune` reclaims
# them all -- which is more than could be said for the root-owned `target/`
# directories they replace.
_gate_lib_tree="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER_CARGO_TARGET_VOLUME="${DOCKER_CARGO_TARGET_VOLUME:-moveit-rs-cargo-$(basename "$_gate_lib_tree")-$(printf '%s' "$_gate_lib_tree" | sha256sum | cut -c1-8)}"
unset _gate_lib_tree
DOCKER_CARGO_TARGET_MOUNT=/cargo-target

docker_cargo_run() {  # <everything `docker run` takes...>
  docker run \
    -v "$DOCKER_CARGO_TARGET_VOLUME:$DOCKER_CARGO_TARGET_MOUNT" \
    -e "CARGO_TARGET_DIR=$DOCKER_CARGO_TARGET_MOUNT" \
    "$@"
}

# Names what a nonzero `moveit-diff` status actually was: a comparison that ran
# and disagreed, or a run that never reached a verdict.
#
# The sweeps print `--- first 20 disagreements ---` and then "$robot disagreed
# with the oracle (exit $status)" for any nonzero status, which is a claim
# about the numbers. It is wrong whenever the run was killed. Observed on
# `main` at a746945: `verify-all.sh` failed with
#
#   === dual_arm_panda / left_panda_arm (10000 cases, seed 1) ===
#   Terminated
#   --- first 20 disagreements ---
#   dual_arm_panda / left_panda_arm disagreed with the oracle (exit 143)
#
# and zero `FAIL` lines under that heading, because there were none to print.
# 143 is 128+SIGTERM. Re-running the same gate alone passed 6/6 entries at
# 20006 cases each -- so the reported disagreement never existed, and the
# wording sends the reader after a numeric bug instead of after whoever sent
# the signal.
#
# The discriminator is the run's own verdict line, not the exit code: a
# comparison that completed prints `failed: N` (`cases:`/`passed:`/`failed:`
# for the fk/jacobian sweeps, per-clause counts for the state sweep). Absent
# that line, the process died before deciding anything, whatever its status.
# The exit code is used only to name the signal, which is the one thing the
# code carries that the output does not.
#
#   run_verdict <status> <output-file> <verdict-line-regex>
#
# echoes `disagreed` or `incomplete: <why>`, and returns 0 either way so the
# caller decides what to do with it.
run_verdict() {
  local status="$1" out="$2" verdict_re="$3"
  if [ "$status" -eq 0 ]; then
    echo "ok"
    return 0
  fi
  if ! grep -qE "$verdict_re" "$out" 2>/dev/null; then
    if [ "$status" -gt 128 ]; then
      local signal="$((status - 128))" name
      name="$(kill -l "$signal" 2>/dev/null || echo "signal $signal")"
      echo "incomplete: killed by SIG$name (exit $status) before reporting a verdict"
    else
      echo "incomplete: exited $status before reporting a verdict"
    fi
    return 0
  fi
  echo "disagreed"
}

# Names why the oracle image's stamp could not be confirmed to match the tree.
#
# Six gates each carried their own copy of
#
#   have="$(docker run --rm --entrypoint cat "$IMAGE" .../oracle-src.sha256 2>/dev/null || true)"
#   if [[ "$have" != "$want" ]]; then ... "image: ${have:-<missing or unstamped>}"
#
# in which `2>/dev/null || true` collapses four different causes into one empty
# string: the docker daemon was unreachable, the image was never built, the
# image predates the stamp, or the stamp is present and different. All four
# then printed `<missing or unstamped>` and the same remediation, "rebuild with
# build.sh" -- which is the right advice for exactly one of them. Observed:
# running the sweeps outside the `docker` group printed
# `image: <missing or unstamped>` and `exited 2 before reporting a verdict`,
# reading as a crashed comparison; the same gate under `sg docker` was RC=0
# with `failed: 0`. Two readers diagnosed that as a missing fixture tree.
#
# Each cause is decided by a command whose whole purpose is that question --
# `command -v docker`, `docker version`, `docker image inspect` -- rather than
# by grepping docker's prose, so a reworded error message cannot silently
# reclassify a run.
#
#   oracle_stamp_verdict <image> <want-stamp>
#
# echoes exactly one of:
#
#   ok                        stamp present and equal to the tree's
#   mismatch <have>           stamp present and different -- rebuild
#   unstamped                 image exists, was built before the stamp existed
#   image-absent              nothing built this tag yet -- rebuild
#   docker-absent             no docker on this machine at all
#   docker-unreachable <why>  docker is installed but refused us
#
# and returns 0 for all of them: the caller decides which are a skip and which
# are a failure. The two that callers must not treat alike are the last two.
# `docker-absent` is a legitimate environment (a tarball on a build box has no
# daemon to ask), so skipping is honest. `docker-unreachable` is never that --
# it means this shell is outside the `docker` group and the gate measured
# nothing, so a gate that exits 0 there reports a pass for a run that never
# happened.
oracle_stamp_verdict() {
  local image="$1" want="$2" have err
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker-absent"
    return 0
  fi
  # `docker version` is the probe rather than `docker info`, which exits 0 even
  # when it could not reach the daemon. Its `{{.Server.Version}}` still prints
  # an empty stdout line before the error lands on stderr, so the diagnosis is
  # the first *non-empty* line, not the first one.
  if ! err="$(docker version --format '{{.Server.Version}}' 2>&1)"; then
    echo "docker-unreachable $(printf '%s\n' "$err" | grep -m1 . || echo 'no diagnostic')"
    return 0
  fi
  if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "image-absent"
    return 0
  fi
  if ! have="$(docker run --rm --entrypoint cat "$image" /usr/local/share/oracle-src.sha256 2>/dev/null)"; then
    echo "unstamped"
    return 0
  fi
  if [ "$have" = "$want" ]; then
    echo "ok"
  else
    echo "mismatch $have"
  fi
}

# The shared wording for every `oracle_stamp_verdict` outcome that is not `ok`,
# so six gates cannot drift into six explanations of the same state.
#
#   oracle_stamp_explain <verdict> <image> <want-stamp> [prefix]
#
# prints the diagnosis on stderr under an optional per-gate prefix (the SKIP
# gates pass `SKIP ` so their output stays greppable), and returns 0 when the
# caller may legitimately skip, 1 when it must fail. Only `docker-unreachable`
# returns 1: see `oracle_stamp_verdict` for why that one is not a skip.
oracle_stamp_explain() {
  local verdict="$1" image="$2" want="$3" prefix="${4-}" cause="${1%% *}" detail="${1#* }"
  case "$cause" in
    docker-unreachable)
      # Deliberately not under `prefix`: the callers pass `SKIP ` so their skips
      # stay greppable, and this outcome is the one that must not read as one.
      echo "FAIL docker is installed but refused this shell: $detail" >&2
      echo "FAIL nothing was measured. Re-run under the docker group: sg docker -c '<this script>'." >&2
      echo "FAIL do not wrap a \${PIPESTATUS[0]} in that string -- sg runs it under sh." >&2
      return 1
      ;;
    docker-absent)  echo "${prefix}this machine has no docker; the oracle cannot be consulted here" >&2 ;;
    image-absent)   echo "${prefix}$image has never been built on this machine" >&2 ;;
    unstamped)      echo "${prefix}$image predates /usr/local/share/oracle-src.sha256 and cannot be identified" >&2 ;;
    mismatch)       echo "${prefix}$image was built from different oracle sources than the working tree" >&2
                    echo "${prefix}  image: $detail" >&2
                    echo "${prefix}  tree:  $want" >&2 ;;
    *)              echo "${prefix}FAIL unknown oracle stamp verdict '$verdict'" >&2; return 1 ;;
  esac
  echo "${prefix}rebuild with tools/moveit-oracle/build.sh" >&2
  return 0
}

# Reserved exit status meaning "this gate did not run its measurement" --
# distinct from 0 (ran, and it held) and 1 (ran, and it failed). Nothing else
# in this directory used exit 3 before this was reserved
# (`rg -n 'exit [0-9]+' tools/ci/*.sh` at the time this was written showed 0,
# 1 and one unrelated usage-error 2 in `verify-phase7-benchmark.sh`, which
# `verify-all.sh` never reaches because it always invokes that script with no
# argument). `verify-all.sh` is the one place that has to tell this apart
# from a pass, which it could not when every gate below spelled "did not
# measure" as `exit 0` -- the same code a real pass uses.
NOT_MEASURED=3

# Prints each line prefixed `SKIP (<kind>) ` and exits $NOT_MEASURED.
#
#   skip_not_measured blocked "docker is not on PATH" "this is not a pass."
#   skip_not_measured opt-in  "PHASE3_SWEEP is not 1" "run with PHASE3_SWEEP=1 ..."
#
# <kind> is the one thing this function forces every call site to declare,
# because "did not measure" is not one fact:
#
#   blocked  something this gate needs is absent from THIS environment --
#            docker, the oracle image at the tree's stamp, a dependency
#            checkout, a vendored fixture tree. It tried and could not.
#   opt-in   an explicit switch this gate requires was left at its default.
#            Nobody asked it to run this time; nothing is missing.
#
# Both are equally "not a pass", which is the only distinction `verify-all.sh`
# acts on -- but a run full of `blocked` lines says this machine cannot cover
# part of the suite, and a run full of `opt-in` lines says nobody asked for
# the expensive part yet, and those call for different responses from
# whoever reads the log. The tag is what keeps that readable without having
# to know, per script, which category it was.
#
# This function does not invent a "this is not a pass" sentence for you --
# every existing call site already says why in its own words; centralising
# the wording would just be a second place for it to drift from the reason.
# Its only job is the one thing that WAS drifting: the exit status.
skip_not_measured() {
  local kind="$1"
  shift
  case "$kind" in
    blocked | opt-in) ;;
    *)
      echo "FAIL skip_not_measured: unknown kind '$kind' (want blocked or opt-in)" >&2
      exit 1
      ;;
  esac
  local line
  for line in "$@"; do
    echo "SKIP ($kind) $line"
  done
  exit "$NOT_MEASURED"
}

# Reserved exit status meaning "this gate ran its measurement and its own
# hard checks held, but the run carries a qualification a plain pass would
# hide" -- distinct from 0 (ran, held, nothing to qualify), 1 (a hard check
# did not hold) and $NOT_MEASURED (no measurement happened at all). Nothing
# else in this directory used this exit before it was reserved (same
# `rg -n 'exit [0-9]+' tools/ci/*.sh` sweep `NOT_MEASURED`'s own comment
# describes, re-run at the time this was written).
#
# `verify-phase8-benchmark.sh` is the motivating case: its three CHOMP/STOMP
# property conditions compare an optimiser against a sampling planner, so an
# UNMET condition is not by itself a porting defect (see that script's
# header) -- the gate is right to exit 0 rather than fail. But a blanket 0
# is exactly the code a plain pass uses, so `verify-all.sh`'s summary folded
# a run that printed UNMET conditions into "passed" with no trace, the same
# shape `NOT_MEASURED` exists to close one level down. A gate that has this
# shape reports it through `report_qualified` instead of a bare `exit 0`.
QUALIFIED=4

# Prints each line prefixed `QUALIFIED ` and exits $QUALIFIED.
#
#   report_qualified "phase 8 condition 1 UNMET for chomp -- see the" \
#                    "CONDITIONS line above and this script's header"
#
# Like `skip_not_measured`, this does not invent the explanation -- every
# call site already printed its own qualified verdict in its own words
# before calling this; the only job here is the exit status, kept in one
# place so it cannot drift from what a gate that qualifies a pass ought to
# use.
report_qualified() {
  local line
  for line in "$@"; do
    echo "QUALIFIED $line"
  done
  exit "$QUALIFIED"
}

# Content digest of one entry in a `measured_sources` map -- the maps the two
# benchmark instruments write and `check-measured-sources-current.sh` enforces.
#
# A FILE digests to its own git blob id, unchanged from what those maps already
# hold, so records written before this function existed keep reading correctly.
# A DIRECTORY digests to a hash over `path blob` for every tracked file beneath
# it, sorted, so adding, deleting or editing any file under that subtree moves
# the value.
#
# Directories are why this exists. Both instruments listed only their own
# harness files, which left the code being measured outside the record they
# claim closes over it: Phase 8 named `optimize_benchmark_chomp.rs` but not
# `crates/cspace-planners-chomp/src`, and two behavioural CHOMP fixes
# (`112ec645` mapping goal-constraint construction failure, `823d771e`
# persisting `should_break_out`) landed after
# `doc/phase8-optimizer-properties.json` was measured with the gate structurally
# unable to see either. Extending the list file by file cannot close that -- it
# goes stale the next time a module is added -- so the unit is the subtree.
#
# The closure is role-based, not proximity-based: a path belongs iff it is
# (a) the arm's own algorithm, wherever that algorithm is actually factored,
# or (b) code every arm calls to decide whether a state or path is valid or
# what it costs. It is not "whatever the harness file imports" (a file
# digest already covers the harness itself, not what it calls into) and it
# is not "used by exactly one arm" -- fan-in undercounts too, since
# `cspace-collision`, `cspace-scene` and `cspace-constraints` are consulted
# by every arm and a single-consumer test would wrongly drop them.
#
# (a): CHOMP has no core crate of its own, so `crates/cspace-planners-chomp/src`
# alone covers its algorithm, but STOMP does -- `Stomp::solve` lives in
# `crates/cspace-stomp-core/src/stomp.rs`, and `a6a81a79` changed its seeding
# tolerance inside Phase 8's measurement window while the old list, built by
# treating the two arms as symmetric, was structurally unable to see it.
# Phase 7's analogue is `crates/cspace-planning/src/planner_registry`:
# `cspace-planners-sbp/src/registry.rs` registers into its `PLANNER_MANAGERS`
# slice, so it is how the SBP arm gets selected and constructed, not shared
# framework -- confirmed neither CHOMP's nor STOMP's source references it.
#
# (b): `cspace-collision`, `cspace-scene`, `cspace-constraints` and (CHOMP
# only) `cspace-distance-field` answer "is this state or path valid, and
# what does it cost" -- the question both phases report a rate or a score
# over. `crates/cspace-planning/src` is Phase 7's case of the same role:
# its `request_adapters/check_start_state_collision.rs` and
# `response_adapters/validate_path.rs` are where the SBP benchmark's start
# state and result actually get validated, and it is SBP-only among the
# three arms -- CHOMP's and STOMP's own references to `cspace_planning` are
# doc comments, not `use` imports, confirmed by reading the call sites
# rather than trusting the dependency edge. `fc908c51` (cspace-scene,
# attached-body touch tracking) and `73c44a25` (cspace-collision,
# exact-tangency tie dispatch) both changed that verdict inside Phase 8's
# window with the old list structurally unable to see either.
#
# Sharing a role does not mean sharing a dependency edge pulls a crate in.
# `cspace-octomap` is a normal dependency of both `cspace-distance-field`
# and `cspace-collision`, but it stays out: it is a generic occupancy-grid
# data structure consulted BY the validity code, the same role
# `cspace-geometry`'s shape types already play, not a place a validity or
# cost decision is made. The same reasoning keeps out `cspace-geometry`,
# `cspace-model`, `cspace-state`, `cspace-error`, `cspace-srdf`,
# `cspace-sampling`, `cspace-kinematics` and `cspace-trajectory` -- each
# gated independently by its own `-p` clippy/nextest, none of them deciding
# a validity or cost verdict. `cspace-planners-sbp` stays out of Phase 8's
# list for the mirror reason: CHOMP's and STOMP's examples import it only
# for `JointModelGroupSpace`'s length metric, a reported number rather than
# a decision.
#
# Derived from `cargo metadata`'s resolved dependency graph -- both normal
# and dev-dependency edges, since these harnesses are `[[example]]` binaries
# that link their own package's dev-dependencies too, so a dev-only edge
# like cspace-planners-chomp -> cspace-scene still ships inside the
# benchmark binary that gets measured -- then confirmed by reading the
# actual call sites, not by hand-listing crate names that look parallel.
# Hand-listing is what produced the stomp-core miss above, since
# `crates/cspace-planners-chomp/src` and `crates/cspace-planners-stomp/src`
# look symmetric and are not. It is still a real limit, not a complete one:
# digesting all of `crates` would invalidate a long-running measurement on
# any unrelated commit, and a gate that is always red is a gate nobody
# reads. A change to a crate outside this list (`cspace-octomap` included)
# can still move these numbers and will not be caught here.
#
# One more limit, measured rather than assumed: `git ls-files` is index-scoped,
# so an UNTRACKED new file under a subtree does not move its digest, while a
# tracked one does (both checked by mutation when this was written). That case
# is covered by a different field -- both instruments record `working_tree_dirty`
# and `dirty_paths` from `git status --porcelain`, which lists untracked files --
# so a run against a tree carrying an unstaged module is identifiable there and
# not here.
measured_source_digest() {
  local path="$1"
  if [ -f "$path" ]; then
    git hash-object "$path"
    return
  fi
  if [ ! -d "$path" ]; then
    echo "FAIL measured_source_digest: '$path' is neither a file nor a directory" >&2
    return 1
  fi
  local files
  files="$(git ls-files --deduplicate -- "$path")"
  if [ -z "$files" ]; then
    echo "FAIL measured_source_digest: no tracked files under '$path' -- a subtree" >&2
    echo "  that digests to nothing would record as unchanged forever" >&2
    return 1
  fi
  # `git hash-object` per file rather than the index's or HEAD's blob ids: the
  # digest has to describe the worktree the run actually compiled, which for a
  # dirty tree is neither of those. `|| exit 1` inside the subshell so a file
  # that is tracked but missing fails the digest instead of silently dropping a
  # line from it.
  local listing
  if ! listing="$(printf '%s\n' "$files" | LC_ALL=C sort | while IFS= read -r f; do
    local blob
    blob="$(git hash-object "$f")" || exit 1
    printf '%s %s\n' "$f" "$blob"
  done)"; then
    echo "FAIL measured_source_digest: hashing a tracked file under '$path' failed" >&2
    return 1
  fi
  printf '%s\n' "$listing" | sha256sum | cut -d' ' -f1
}

# Hard wall-clock bound around one oracle round trip through `sg docker -c`.
#
# Six call sites in tools/ci ran the oracle through `sg docker -c "$ORACLE
# ..." <in >out 2>err` with no bound at all -- not here, not in
# `verify-all.sh`, not in `ci.yml` (no `timeout-minutes` on its one job).
# `oracle.cpp`'s main loop is `while (std::getline(std::cin, line))` with no
# per-request bound, so a request that never returns from
# `oracle->handle(request)` leaves the process blocked *inside* that call,
# never back at `getline` to observe a closed stdin -- closing stdin cannot
# unstick it. `tools/moveit-diff/src/lib.rs`'s `wait_or_kill` closed the same
# defect for the Rust callers that spawn the oracle directly and tear it down
# after their work is done; this is its shell-side twin, for the calls that
# DO the work rather than clean up after it -- so a hang here is worse than a
# hang there. A hang is worse than a failure: a failure is a verdict, a hang
# consumes the caller's entire time budget and produces neither.
#
#   oracle_call <timeout-seconds> -- <command...>
#
# Runs `<command...>` (`sg docker -c "..."` at every current call site) under
# GNU `timeout`, passing the caller's own stdin/stdout/stderr through
# unchanged -- so redirect the CALL, exactly as any other command:
#
#   if ! oracle_call "$TIMEOUT" -- sg docker -c "$ORACLE ..." \
#        <"$req" >"$out" 2>"$err"; then
#     oracle_call_explain "$ORACLE_CALL_STATUS" "some_tag: "
#     ...
#   fi
#
# `timeout` reports a bound that fired as exit 124, the same code any process
# that happened to exit 124 on its own would produce -- indistinguishable
# from a plain `$?`. Worse, a caller written as `if ! oracle_call ...; then`
# cannot even see that much: bash's `!` collapses the wrapped exit status to
# 0 or 1 for the branch decision, discarding the original code before the
# `then` body ever runs. `run_verdict` above already works around the same
# loss by taking an explicit status argument rather than trusting `$?` to
# survive a negation; `oracle_call` does the same by setting
# `$ORACLE_CALL_STATUS` as a side effect (this function runs in the caller's
# own shell, not a subshell), so a caller already shaped
# `if ! sg docker -c ...; then` needs only to read that variable inside its
# existing branch, not restructure around a captured `$?`.
#
# `--kill-after=10`: `timeout`'s default sends only SIGTERM, and a process
# stuck inside a blocking syscall on a wedged daemon -- the exact case
# `wait_or_kill`'s own doc names -- can ignore that indefinitely. 10s is
# generous next to any process's own SIGTERM handling and small next to every
# timeout value passed at a call site.
#
# What this does NOT close, same as `wait_or_kill`: `sg docker -c` runs
# `run-oracle.sh`'s `docker run --rm -i` several processes down, and
# `timeout`'s signal reaches that whole tree only because none of `sg`, `sh`
# and `run-oracle.sh` move themselves to a new process group. Whether the
# CONTAINER itself stops when its `docker run` client is killed depends on
# the docker daemon forwarding that signal, which can itself be stuck -- the
# same daemon-level residual risk `wait_or_kill`'s own doc accepts rather
# than claims to solve.
oracle_call() {
  local secs="$1"
  shift
  if [ "$1" != "--" ]; then
    echo "FAIL oracle_call: usage: oracle_call <seconds> -- <command...>" >&2
    return 2
  fi
  shift
  timeout --kill-after=10 "$secs" "$@"
  ORACLE_CALL_STATUS=$?
  return "$ORACLE_CALL_STATUS"
}

# The shared wording for an `oracle_call` outcome that was not 0, so six call
# sites cannot drift into six different phrasings of "was this a timeout" --
# the same reason `oracle_stamp_explain` above centralises the stamp-mismatch
# wording instead of leaving it to each caller.
#
#   oracle_call_explain <status> [prefix]
#
# Prints one FAIL line to stderr, naming a bound that fired (status 124,
# `timeout`'s own reserved code for that) as a condition distinct from any
# other nonzero exit -- never folded into "the oracle returned an error", and
# never silent. `prefix` matches the indentation some call sites already give
# their own FAIL lines. Callers keep their own `failed+=(...)` bookkeeping
# and whatever stderr tail they already print; this supplies only the one
# sentence that must not vary by call site.
oracle_call_explain() {
  local status="$1" prefix="${2-}"
  if [ "$status" -eq 124 ]; then
    echo "${prefix}FAIL oracle call timed out and was killed -- no verdict, not a disagreement" >&2
  else
    echo "${prefix}FAIL oracle call exited $status" >&2
  fi
}
