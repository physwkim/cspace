#!/bin/bash
# Prints the digest of everything the oracle image is built from, and the
# image tag derived from it.
#
# One definition, used by build.sh (which tags with it), the Dockerfile
# (which stamps the image with it) and run-oracle.sh (which resolves the tag
# and verifies the stamp). Computing it separately in any of them would let
# them drift into disagreeing about which image is current -- and the
# disagreement would look exactly like a stale image.
#
# Every regular file under the directory is hashed, with no extension
# allowlist. build.sh `cp -a`s this whole directory into the build context
# and the Dockerfile `COPY`s all of it to /ws/oracle-src, so "hashed" and
# "went into the image" are the same set by construction; an allowlist makes
# them different sets, and every file kind outside it (Dockerfile, build.sh,
# the pinned MOVEIT2_SHA it carries, entrypoint.sh, a future .cmake or
# vendored data file) changes the image while leaving the stamp intact --
# which is exactly the stale-image failure the stamp exists to catch.
#
# The cost is the other direction: editing a host-side-only file
# (run-oracle.sh, capture-dynamics-fixtures.py) invalidates every built image
# and forces a rebuild that changes nothing inside it. That is the safe
# direction of the trade -- a spurious rebuild costs build time, a missed one
# costs a wrong fixture that looks exactly like a good one.
#
# The file digest alone was never the whole story: a `docker build
# --build-arg`, an env-var override, or upstream moving a mutable base tag
# all change the image without touching a file here. `oracle_stamp` closes
# that by hashing the *resolved* non-file build inputs alongside the files;
# see its comment for which bypass each one closes.
#
# Paths are relative on purpose: sha256sum hashes the name alongside the
# bytes, and these sources sit at /ws/oracle-src inside the image but under
# the repo out here.
#
# `sort` runs under LC_ALL=C so the digest is a function of the bytes alone.
# Collation is locale-dependent: this host sorts case-insensitively
# (`build.sh` before `CMakeLists.txt`), the container's C locale sorts by
# byte (`CMakeLists.txt` first). Same files, same contents, different
# concatenation order, different digest -- which run-oracle.sh reports as a
# stale image that rebuilding cannot fix.
#
# Deliberately no `set` line here. This file is `source`d, not executed, so
# any `set` here would change the *caller's* shell options for the rest of
# its own script, not just this file's. Ten of the twelve shell callers
# already declare `set -euo pipefail` before sourcing this (`rg -n 'src-
# digest\.sh' -g '*.sh'` plus `rg -n '^set -'` on each hit confirms it):
# build.sh, run-oracle.sh, verify-fcl-cylinder-box-distance.sh,
# verify-fcl-distance-tolerance.sh, verify-fcl-tangency-dispatch.sh,
# ros/build.sh, ros/verify-move-action-interop.sh,
# ros/verify-robot-description-interop.sh, ros/verify-joint-states-interop.sh,
# measure-phase8-cpp-baseline.sh. `verify-phase3-tangency-subset.sh` and
# `verify-phase3-penetration-subset.sh` deliberately choose `set -uo
# pipefail`, no `-e`: their per-robot loop needs a comparison binary's
# nonzero exit to reach `run_verdict` (gate-lib.sh) rather than abort the
# script, so it can tell "disagreed" from "killed before reporting a
# verdict" and keep scoring the rest of ROBOTS. A `set -e` sourced in from
# here used to override that silently from this point in the caller's
# script onward: the first nonzero-exit comparison (not a raw crash, an
# ordinary `std::process::exit(N)`) aborted the whole script mid-loop,
# skipping `rc=$?`, `run_verdict`, and every robot still queued, with no
# FAIL line -- the exact shape of a robot's subset going silently
# unmeasured. The twelfth caller, `tools/moveit-oracle/Dockerfile`'s `RUN
# ... && source .../src-digest.sh && oracle_stamp ... > oracle-src.sha256`,
# declares no posture of its own at all and was relying entirely on this
# file's sourced `-e` to make a failing `oracle_stamp` fail the build.
#
# `oracle_file_digest`/`oracle_stamp` below do not lean on any of that: both
# enforce their own strictness internally now (a subshelled `pipefail` in
# `oracle_file_digest`, an explicit `|| return 1` in `oracle_stamp`) so a
# real failure -- a missing directory, a file vanishing mid-`find` --
# produces a nonzero return no matter what the caller's own `-e`/`-u`/
# `pipefail` posture is, rather than depending on it.

# Files only -- deliberately NOT the tag input. `oracle_stamp` is. Two
# functions that both return a hex digest, only one of which names a real
# image, is the same footgun shape as the bypasses above: passing this one
# to `oracle_image_tag` yields a tag that has never been built, and the
# failure reads as a stale image. The name is the guard; there is no way to
# make it a type here.
#
# Runs in a subshell that sets its own `pipefail`, deliberately not relying
# on the caller's. `cd "$1"` failing (or any pipeline stage after it) must
# make this whole compound command's exit status nonzero regardless of
# whether the caller has `-e`/`pipefail` of their own -- `oracle_stamp`
# below turns that into a hard `return 1` no caller posture can bypass. See
# its comment for what silently swallowing this used to produce instead.
oracle_file_digest() {  # <tools/moveit-oracle dir>
  ( set -o pipefail
    cd "$1" &&
      find . -type f -print0 |
        LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1 )
}

# The canonical non-file build inputs. These live here, not in build.sh and
# not as Dockerfile defaults, because `oracle_stamp` has to hash the same
# values the build actually used -- two definitions is how they drift.
#
# ORACLE_BASE_IMAGE is pinned by manifest digest, not by the `rolling-ci`
# tag. A tag is mutable: upstream can move it and every local file digest
# stays identical while the image underneath changes. Repinning is a
# deliberate edit to this line, which changes the stamp, which changes the
# tag -- so a new base cannot arrive silently.
#
# There is deliberately no ROS_DISTRO here. It used to be a build arg, and
# it was one of the bypasses: `--build-arg ROS_DISTRO=...` selected the base
# tag while every `RUN` in the image kept reading the base image's own
# `ROS_DISTRO` ENV, which overrides a same-named ARG. With the base pinned
# by digest the distro is a property of that image and nothing else, so the
# argument that could disagree with it no longer exists rather than being
# checked for.
#
# Each takes an already-exported value if there is one. That is the single
# override mechanism: export before sourcing this file, and every consumer
# -- build.sh's `--build-arg`s, the Dockerfile's stamp, run-oracle.sh's
# check -- sees the same value, because they all read it from here. The
# Dockerfile depends on this: it exports the resolved ARGs before sourcing,
# so the stamp records what the build actually used and not these defaults.
ORACLE_BASE_IMAGE="${ORACLE_BASE_IMAGE:-moveit/moveit2@sha256:7c394edd1faac3eb2dda1519df0cda58e9d870c42aaa7a7678934f76e3d1acc0}"
# pilz_industrial_motion_planner joins moveit_core here for the
# `pilz_trajectory` op (Phase 8's LIN/PTP/CIRC completion condition). It is
# the one planner this file builds from source rather than taking prebuilt
# from the base image the way `plan` takes OMPL: pilz is a moveit2 package,
# so it tracks MOVEIT2_SHA, and a prebuilt copy would silently pair oracle
# trajectories with a different revision than every other op reports.
#
# moveit_kinematics comes with it, not for a generator but for LIN and CIRC:
# both run IK on their Cartesian goal, and a URDF+SRDF-only RobotModel has no
# solver, so both returned NO_IK_SOLUTION until `ensureKinematicsSolver`
# could load `kdl_kinematics_plugin/KDLKinematicsPlugin` by name.
#
# chomp_motion_planner is here for the `chomp_quad_cost_inverse` op, which
# links the real `ChompCost` so the comparison it feeds measures Eigen's
# inverse and nothing else.
ORACLE_MOVEIT2_PACKAGES="${ORACLE_MOVEIT2_PACKAGES:-moveit_core moveit_resources_fanuc_description pilz_industrial_motion_planner moveit_kinematics chomp_motion_planner}"
ORACLE_MOVEIT2_SHA="${ORACLE_MOVEIT2_SHA:-e017c91ee12984393a28ba246075c65f69cde3bf}"

# Serialized build inputs, one `NAME=value` per line. A deliberately
# re-pinned build produces a distinguishable image instead of impersonating
# the canonical one, because the override lands in the stamp.
oracle_build_inputs() {
  printf 'BASE_IMAGE=%s\nMOVEIT2_PACKAGES=%s\nMOVEIT2_SHA=%s\n' \
    "$ORACLE_BASE_IMAGE" "$ORACLE_MOVEIT2_PACKAGES" "$ORACLE_MOVEIT2_SHA"
}

# What build.sh tags with, the Dockerfile stamps, and run-oracle.sh checks:
# the file digest and the resolved build inputs, together.
#
# `oracle_file_digest` covers everything under this directory. This adds the
# inputs that are not files, closing the three bypasses that used to be
# documented as known gaps: a hand-passed `--build-arg MOVEIT2_PACKAGES`, an
# env-overridden `MOVEIT2_SHA` (build.sh's pin check compares the env value
# against the checkout, so moving both passed it), and upstream repushing
# the mutable base tag.
#
# The intermediate `--target moveit` stage carries no stamp file at all, so
# run-oracle.sh's `<missing or unstamped>` branch rejects it. That is relied
# on deliberately here: the stamp is written only in the `oracle` stage.
#
# `oracle_file_digest`'s exit status is checked explicitly with `|| return
# 1`, not left to the caller's `-e`. It used to be `{ oracle_file_digest
# "$1"; oracle_build_inputs; } | sha256sum | cut ...`: a brace group returns
# only its LAST command's status, so a failing `oracle_file_digest` (a
# nonexistent or unreadable directory, a file vanishing mid-`find`) was
# discarded by the always-succeeding `oracle_build_inputs` printf that ran
# right after it, and `sha256sum` still hashed whatever partial bytes
# `oracle_file_digest` had managed to emit before failing (nothing, for a
# `cd` failure) -- producing a well-formed 64-hex-char digest for a
# directory that was never actually read, silently. Under a caller's own
# `set -e` that command substitution's nonzero status (from `pipefail`,
# since the brace-group pipeline's last real failure was the killed
# subshell) aborted the script before this ever reached `oracle_stamp_verdict`.
# Under `verify-phase3-tangency-subset.sh` and
# `verify-phase3-penetration-subset.sh`'s own `set -uo pipefail` (deliberately
# no `-e`, see below this file's header) it did not: `want` came out as a
# plausible stamp for a directory `oracle_file_digest` never read, matching
# whichever built image happened to share that coincidence -- exactly the
# "well-formed stamp that was never built" failure this file's own header
# says the stamp exists to prevent. `local files` is declared on its own
# line and assigned after: `local files=$(...)` would return `local`'s own
# status, not the substitution's, and silently reopen the same swallow.
oracle_stamp() {  # <tools/moveit-oracle dir>
  local files
  files="$(oracle_file_digest "$1")" || return 1
  { printf '%s\n' "$files"; oracle_build_inputs; } | sha256sum | cut -d' ' -f1
}

# The tag carries the stamp so that two worktrees with different oracle
# sources build two different images instead of overwriting one shared
# `:latest`. Seven concurrent worktrees share this docker daemon; a single
# mutable tag makes whoever builds last silently the oracle for everyone.
#
# Not the same swallow shape as `oracle_stamp` above, checked: this is pure
# string slicing with nothing fallible to swallow, and a caller that ignores
# `oracle_stamp`'s `return 1` and passes it an empty `$1` anyway gets
# `moveit-rs/oracle:` -- `oracle_stamp_verdict` (gate-lib.sh)'s `docker image
# inspect` on that tag fails (no such tag has ever been built) and reports
# `image-absent`, a loud SKIP, not a silent `ok`. It stays a two-function
# split rather than folding this into `oracle_stamp`: one always returns a
# hex digest, the other always returns a tag string, and merging them would
# put the empty-stamp guard behind the one caller (`oracle_image_tag`) that
# cannot itself detect a stamp is empty without also being handed the
# reason, which only `oracle_stamp`'s own `return 1` already carries.
oracle_image_tag() {  # <stamp>
  echo "moveit-rs/oracle:${1:0:16}"
}
