#!/usr/bin/env python3
"""A stand-in for the C++ oracle, used to test moveit-diff itself.

It answers with a fixed two-joint model so the runner's protocol handling,
sampling and failure reporting can be exercised without building moveit2.
Never used for a real comparison -- its numbers are made up.
"""
import json
import sys

MODEL = {
    "name": "fake",
    "model_frame": "world",
    "root_link": "base",
    "links": ["base", "link1", "link2"],
    "joints": ["joint1", "joint2"],
    "joint_details": [
        {
            "name": "joint1",
            "type_name": "REVOLUTE",
            "variable_names": ["joint1"],
            "bounds": [[-1.0, 1.0]],
            "position_bounded": [True],
        },
        {
            "name": "joint2",
            "type_name": "REVOLUTE",
            "variable_names": ["joint2"],
            "bounds": [[None, None]],
            "position_bounded": [False],
        },
    ],
    "groups": {"arm": ["joint1", "joint2"]},
}


def identity():
    return [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def main():
    # Arguments are accepted and ignored; the runner always passes them.
    #
    # readline, not `for line in sys.stdin`: the iterator form reads ahead into
    # an internal buffer and yields nothing until it fills, which deadlocks
    # against a runner that waits for each answer before sending the next
    # request.
    for line in iter(sys.stdin.readline, ""):
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        rid = req.get("id", 0)
        try:
            if req["op"] == "model_info":
                result = MODEL
            elif req["op"] == "random_states":
                result = {
                    "states": [
                        {"joint1": 0.0, "joint2": float(i)}
                        for i in range(req["count"])
                    ]
                }
            elif req["op"] == "fk":
                result = {
                    "link_transforms": {ln: identity() for ln in MODEL["links"]}
                }
            else:
                raise ValueError("unsupported op: " + req["op"])
            resp = {"id": rid, "ok": True, "result": result}
        except Exception as exc:  # noqa: BLE001 - reported over the wire
            resp = {"id": rid, "ok": False, "error": str(exc)}
        sys.stdout.write(json.dumps(resp) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
