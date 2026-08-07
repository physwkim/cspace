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
