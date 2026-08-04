# Claim audit: moveit-octomap

Per PORTING-PLAN.md §175. One row per item, appended as found, not
batched at report time. `evidence` is the upstream `file:line` actually
opened this round -- inference from the port is not evidence.

Upstream root for this crate: `third_party/octomap/octomap/` (octomap
1.9.7, tag `aa6372b87eaf7e89bb1c9421f61d58bd634477cb`).

## §172 sweep (round 33), upstream-first per `0ad8c67`

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-octomap/src/tree.rs:828` (`coord_to_key_checked_axis`) | Upstream narrows the scaled coordinate to `int` unconditionally before its own bounds check; port instead bounds in `f64` space first, a §153.1 deviation (NaN/huge/inf reject cleanly instead of silently landing in-range or overflow-panicking) | CONFIRMED same-defect, fixed | `third_party/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx:313` -- `int scaled_coord = ((int) floor(resolution_factor * coordinate)) + tree_max_val;` then `if ((scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2*tree_max_val)))` | `2584071` |
| `crates/moveit-octomap/src/tree.rs:325` (`coordToKeyChecked(double, unsigned depth, key_type&)` overload, doc bullet) | Same narrowing shape as the ported overload above, but this depth-parameterized overload itself is unported (no depth-write caller in this crate) | CONFIRMED not-applicable (no port exists to carry the defect) | `third_party/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx:325-336` -- identical `(int) floor(...)` narrowing, confirmed same pattern | (none) |
| `crates/moveit-octomap/src/tree.rs:402-403` (doc bullet: `getUnknownLeafCenters` unported) | `getUnknownLeafCenters` narrows `floor(diff[i]/step_size)` to `unsigned int` via `static_cast`; port correctly excludes the whole function (zero consumer) rather than reproducing or silently dropping the narrowing | CONFIRMED not-applicable | `third_party/octomap/octomap/include/octomap/OcTreeBaseImpl.hxx:1060` -- `steps[i] = static_cast<unsigned int>(floor(diff[i] / step_size));` | (none) |
| `crates/moveit-octomap/src/*.rs` (port-side anchor: `as i8..u128/usize` receiving an `f64` expr) | Every remaining `as usize`/`as i64` in this crate narrows an already-integer quantity (child index, key offset), not a real-valued one | CONFIRMED distinct, 3 sites (`iter.rs:46`, `tree.rs:869`, `tree.rs:1320`, `tree.rs:1423`) | Read in this tree only -- each narrows `compute_child_idx(...)`/a loop index, never a float | (none) |

## §167.6 bare-directory-citation sweep (this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| Every `Ported from` header in this crate (`lib.rs:5,25`, `key.rs:5`, `iter.rs:5`, `tree.rs:5`, `node.rs:5`) | None cite a bare package/directory line with no filenames indented beneath it -- every citation lists explicit headers (`OcTreeKey.h`, `OcTreeIterator.hxx`, `OcTreeBaseImpl.h`/`.hxx`, etc.) | CONFIRMED, 0 hits of the shape the parser now closes | Read all six headers in full in this tree; `tools/ci/verify-upstream-license-provenance.sh` also run over the whole workspace this round: `checked 334 upstream file(s) cited by 242 tracked source file(s)`, 0 findings | (none) |

## Trailing-byte acceptance in `read_binary_data`/`read_data` (found by p9-ros, this round)

| where | claim | verdict | evidence | commit |
|---|---|---|---|---|
| `crates/moveit-octomap/src/tree.rs` `read_binary_data`/`read_data` (neither checks the cursor is exhausted after the recursive decode returns) | Exact parity with `octomap_msgs::readTree`/`fullMsgToMap`, which write `msg.data` into a `std::stringstream` with no length header and call `readBinaryData`/`readData` with no exhaustion check of their own; upstream's own integrity check (`size != this->size()` in `AbstractOccupancyOcTree::readBinary`) only exists on the `.bt` file path, which has a node-count header the message path never carries -- so the message path cannot detect a short/over-long read, upstream included | CONFIRMED parity, inherited weakness named -- not a bug, see §153.1 expiry below | `/opt/ros/rolling/include/octomap_msgs/octomap_msgs/conversions.h` (`readTree` writes to a stringstream then calls `readBinaryData` with no size field; `fullMsgToMap` does the same via `readData`), read inside `moveit-rs/oracle:latest`; `third_party/octomap/octomap/src/AbstractOccupancyOcTree.cpp:172-176` (`if (size != this->size()) { ... }`), read in this tree -- confirms `size` is file-header-derived and has no message-path counterpart | (none; doc-only, `read_binary_data_ignores_trailing_bytes_after_a_complete_decode`/`read_data_ignores_trailing_bytes_after_a_complete_decode` added to lock the behavior in) |

**§153.1 expiry:** if a future round decides to reject trailing bytes on
either function, that is a deliberate *deviation* from upstream's
message-path behavior requiring sign-off -- not a bug fix, and not
something to "harden" silently. The two boundary tests above exist so
that decision has to be made explicitly, by breaking a named test,
rather than by accident.
