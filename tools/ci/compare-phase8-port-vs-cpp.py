#!/usr/bin/env python3
"""Per-problem comparison of a Phase 8 port sweep against the C++ baseline.

Both sides are NDJSON, one object per problem, keyed by `id` within a config.
The port side comes from `chomp_benchmark_port.rs` / `stomp_benchmark_port.rs`;
the C++ side from `tools/ci/measure-phase8-cpp-baseline.sh`.

Aggregates alone cannot answer the question this exists for. Two sides can post
the same success count and still fail on disjoint sets of problems, which is a
completely different finding from "they fail on the same problems" -- the first
says the two implementations have different weaknesses, the second says the
benchmark has hard problems. So the report is built around the four-way split
(both solve / port only / cpp only / neither) and prints the actual ids, not
just their counts.

Usage:
  compare-phase8-port-vs-cpp.py --config <name> --port <ndjson> --cpp <ndjson> [...]

Repeat the triple to fold several configs into one report.
"""

import argparse
import json
import statistics
import sys


def load(path):
    """NDJSON -> {id: record}, rejecting duplicate ids rather than overwriting.

    A duplicate id means two shards covered the same problem, which silently
    makes the population smaller than its line count; that has to be an error
    and not a last-write-wins merge.
    """
    out = {}
    with open(path) as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            problem_id = record["id"]
            if problem_id in out:
                sys.exit(f"{path}:{line_number}: duplicate id {problem_id}")
            out[problem_id] = record
    return out


def solved(record):
    return bool(record.get("solved"))


def median(values):
    return statistics.median(values) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", action="append", required=True)
    parser.add_argument("--port", action="append", required=True)
    parser.add_argument("--cpp", action="append", required=True)
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    if not (len(args.config) == len(args.port) == len(args.cpp)):
        sys.exit("--config/--port/--cpp must be repeated the same number of times")

    both, port_only, cpp_only, neither = [], [], [], []
    port_lengths, cpp_lengths = [], []
    port_c2_fail, cpp_c2_fail = [], []
    cpp_failures, port_failures = {}, {}

    for config, port_path, cpp_path in zip(args.config, args.port, args.cpp):
        port = load(port_path)
        cpp = load(cpp_path)
        if port.keys() != cpp.keys():
            only_port = sorted(port.keys() - cpp.keys())
            only_cpp = sorted(cpp.keys() - port.keys())
            sys.exit(
                f"{config}: the two sides do not cover the same problems "
                f"(port-only ids {only_port[:10]}, cpp-only ids {only_cpp[:10]})"
            )

        for problem_id in sorted(port):
            tag = f"{config}/{problem_id}"
            p, c = port[problem_id], cpp[problem_id]
            if solved(p) and solved(c):
                both.append(tag)
            elif solved(p):
                port_only.append(tag)
            elif solved(c):
                cpp_only.append(tag)
            else:
                neither.append(tag)

            if solved(p):
                port_lengths.append(p["length"])
                if p.get("condition2_valid") is False:
                    port_c2_fail.append(tag)
            else:
                port_failures[p.get("failure", "?")] = (
                    port_failures.get(p.get("failure", "?"), 0) + 1
                )
            if solved(c):
                cpp_lengths.append(c["length"])
                if c.get("condition2_valid") is False:
                    cpp_c2_fail.append(tag)
            else:
                cpp_failures[c.get("failure", "?")] = (
                    cpp_failures.get(c.get("failure", "?"), 0) + 1
                )

    total = len(both) + len(port_only) + len(cpp_only) + len(neither)
    port_solved = len(both) + len(port_only)
    cpp_solved = len(both) + len(cpp_only)

    print(f"=== {args.label or 'phase8'} per-problem comparison (n={total}) ===")
    print(f"port solved : {port_solved}/{total} = {100.0 * port_solved / total:.1f}%")
    print(f"cpp  solved : {cpp_solved}/{total} = {100.0 * cpp_solved / total:.1f}%")
    print()
    print(f"both solve    : {len(both)}")
    print(f"port only     : {len(port_only)}")
    print(f"cpp only      : {len(cpp_only)}")
    print(f"neither       : {len(neither)}")
    # The number the aggregate hides: how many problems the two sides disagree
    # on at all. Equal totals with a large disagreement set is the finding.
    print(f"disagreements : {len(port_only) + len(cpp_only)}")
    print()
    print(f"port-only ids : {' '.join(port_only) if port_only else '(none)'}")
    print(f"cpp-only ids  : {' '.join(cpp_only) if cpp_only else '(none)'}")
    print()
    print(f"port median length (solved) : {median(port_lengths)}")
    print(f"cpp  median length (solved) : {median(cpp_lengths)}")
    print()
    print(f"port condition-2 failures : {len(port_c2_fail)} {port_c2_fail}")
    print(f"cpp  condition-2 failures : {len(cpp_c2_fail)} {cpp_c2_fail}")
    print()
    print(f"port failure reasons : {port_failures}")
    print(f"cpp  failure reasons : {cpp_failures}")


if __name__ == "__main__":
    main()
