#!/usr/bin/env python3
"""Offline (non-publishing) tests for the frozen Task 4 /joint_command contract.

Task 4 Phase 3 has no real command producer, so these tests use *offline
fixtures*: real ``sensor_msgs/msg/JointState`` objects constructed in-process.
No publisher, no subscriber and no ROS graph edge is created. They validate the
reference validator/cache in ``tools/joint_command_contract.py`` and act as the
executable proof required by Phase 3 section 6/7.

Evidence: docs/task4_phase3_evidence/command_contract_tests.json
          docs/task4_phase3_evidence/joint_limits.json
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from joint_command_contract import (  # noqa: E402
    JOINT_ORDER,
    CommandCache,
    REASON_BACK,
    REASON_DIM,
    REASON_DUP,
    REASON_EFF,
    REASON_FRAME,
    REASON_LIMIT,
    REASON_NONFINITE,
    REASON_ORDER,
    REASON_VEL,
    REASON_ZERO,
)

from sensor_msgs.msg import JointState  # noqa: E402

SDF = "src/nao/nao_description/urdf/nao_simple.sdf"


def load_limits(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    limits = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        axis = joint.find("axis")
        if axis is None:
            continue
        limit = axis.find("limit")
        if limit is None:
            continue
        limits[name] = (float(limit.find("lower").text), float(limit.find("upper").text))
    return {name: limits[name] for name in JOINT_ORDER}


def make_msg(limits, stamp_sec, stamp_nsec, name=None, position=None,
             velocity=None, effort=None, frame_id=""):
    msg = JointState()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    msg.header.frame_id = frame_id
    msg.name = list(JOINT_ORDER) if name is None else list(name)
    if position is None:
        position = [min(max(0.0, limits[n][0]), limits[n][1]) for n in JOINT_ORDER]
    msg.position = [float(v) for v in position]
    msg.velocity = [] if velocity is None else list(velocity)
    msg.effort = [] if effort is None else list(effort)
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdf", default=SDF)
    ap.add_argument("--out", default="docs/task4_phase3_evidence")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    limits = load_limits(args.sdf)
    assert len(limits) == 24, limits

    with open(os.path.join(args.out, "joint_limits.json"), "w") as f:
        json.dump(limits, f, indent=2, sort_keys=True)

    results = []

    def record(case, expected, actual, passed, detail=""):
        results.append({
            "case": case,
            "expected": expected,
            "actual": actual,
            "passed": bool(passed),
            "detail": detail,
        })

    def expect_reject(case, msg, reason, use_cache=None):
        cache = use_cache if use_cache is not None else CommandCache()
        res = cache.offer(msg, limits)
        rejected = (not res.ok) and res.reason == reason
        # rejected message must not become valid data, and the held command must
        # not be updated by an invalid message.
        before = cache.get_for(100.0)
        record(case, reason, res.reason, rejected,
               "cache_not_updated={}".format(before is None or use_cache is not None))

    # --- illegal cases 1-10 (plain rejection) ---
    expect_reject("1_dim23", make_msg(limits, 100, 0, name=JOINT_ORDER[:23],
                                      position=[0.0] * 23), REASON_DIM)
    expect_reject("2_dim25", make_msg(limits, 100, 0, name=JOINT_ORDER + ["Extra"],
                                      position=[0.0] * 25), REASON_DIM)
    bad_name = list(JOINT_ORDER)
    bad_name[0] = "HeadYawX"
    expect_reject("3_wrong_joint_name", make_msg(limits, 100, 0, name=bad_name),
                  REASON_ORDER)
    bad_order = list(JOINT_ORDER)
    bad_order[0], bad_order[1] = bad_order[1], bad_order[0]
    expect_reject("4_wrong_joint_order", make_msg(limits, 100, 0, name=bad_order),
                  REASON_ORDER)
    expect_reject("5_empty_position", make_msg(limits, 100, 0, position=[]),
                  REASON_DIM)
    expect_reject("6_velocity_not_empty",
                  make_msg(limits, 100, 0, velocity=[0.0] * 24), REASON_VEL)
    expect_reject("7_effort_not_empty",
                  make_msg(limits, 100, 0, effort=[0.0] * 24), REASON_EFF)
    nan_pos = [min(max(0.0, limits[n][0]), limits[n][1]) for n in JOINT_ORDER]
    nan_pos[0] = float("nan")
    expect_reject("8_non_finite_nan", make_msg(limits, 100, 0, position=nan_pos),
                  REASON_NONFINITE)
    inf_pos = [min(max(0.0, limits[n][0]), limits[n][1]) for n in JOINT_ORDER]
    inf_pos[0] = float("inf")
    expect_reject("8b_non_finite_inf", make_msg(limits, 100, 0, position=inf_pos),
                  REASON_NONFINITE)
    oob = [min(max(0.0, limits[n][0]), limits[n][1]) for n in JOINT_ORDER]
    oob[0] = limits[JOINT_ORDER[0]][1] + 1.0
    expect_reject("9_out_of_limit", make_msg(limits, 100, 0, position=oob),
                  REASON_LIMIT)
    expect_reject("10_zero_timestamp", make_msg(limits, 0, 0), REASON_ZERO)
    expect_reject("10b_frame_mismatch", make_msg(limits, 100, 0, frame_id="world"),
                  REASON_FRAME)

    # --- timestamp duplicate / backward (cache must keep previous valid cmd) ---
    cache = CommandCache()
    base = make_msg(limits, 100, 0)
    assert cache.offer(base, limits).ok
    prev = cache.get_for(100.0)
    res = cache.offer(make_msg(limits, 100, 0), limits)
    record("11_duplicate_timestamp", REASON_DUP, res.reason,
           res.reason == REASON_DUP and cache.get_for(100.0) == prev,
           "previous_command_preserved")
    res = cache.offer(make_msg(limits, 99, 0), limits)
    record("12_backward_timestamp", REASON_BACK, res.reason,
           res.reason == REASON_BACK and cache.get_for(100.0) == prev,
           "previous_command_preserved")

    # --- future / stale / producer-disappear (not usable for a sample) ---
    cache = CommandCache()
    assert cache.offer(make_msg(limits, 100, 0), limits).ok
    record("13_future_timestamp", "get_for(99.0) is None",
           "None" if cache.get_for(99.0) is None else "value",
           cache.get_for(99.0) is None, "command after sample time is not used")
    fresh = cache.get_for(100.04)
    stale = cache.get_for(100.051)
    record("14_stale_beyond_hold", "0.04s valid / 0.051s None",
           "valid" if fresh is not None else "None",
           fresh is not None, "within hold_max=0.05")
    record("14b_stale_beyond_hold", "0.04s valid / 0.051s None",
           "None" if stale is None else "value",
           stale is None, "beyond hold_max=0.05")
    early = cache.get_for(100.02)
    late = cache.get_for(100.2)
    record("15_producer_disappears", "early valid / late None",
           "valid" if early is not None else "None",
           early is not None, "no new command, within hold")
    record("15b_producer_disappears", "early valid / late None",
           "None" if late is None else "value",
           late is None, "no fabricated command after producer stops")

    # --- no producer: empty cache must never yield a command ---
    empty = CommandCache()
    record("16_no_producer_empty_cache",
           "get_for(any) is None (no sample may be emitted)",
           "None" if empty.get_for(123.0) is None else "value",
           empty.get_for(123.0) is None,
           "no accepted command -> command_not_available -> reject_sample")

    # --- legal acceptance ---
    cache = CommandCache()
    res = cache.offer(make_msg(limits, 200, 500000000), limits)
    held = cache.get_for(200.5 + 0.04)
    legal_ok = (res.ok and res.reason is None and cache.accepted == 1
                and isinstance(held, list) and len(held) == 24)
    record("legal_accept_and_hold", "accepted, 24-dim held within 0.05s",
           "accepted" if res.ok else res.reason, legal_ok,
           "accepted={}".format(cache.accepted))

    passed = all(r["passed"] for r in results)
    summary = {
        "n_cases": len(results),
        "n_passed": sum(1 for r in results if r["passed"]),
        "verdict": "PASS" if passed else "FAIL",
        "note": ("Offline fixtures only. No ROS publisher/subscriber created; "
                 "no /joint_command topic is claimed to exist."),
        "last_cache_rejects": cache.reject_counts,
        "results": results,
    }
    with open(os.path.join(args.out, "command_contract_tests.json"), "w") as f:
        json.dump(summary, f, indent=2)

    for r in results:
        print("[{}] {} expected={} actual={}".format(
            "PASS" if r["passed"] else "FAIL", r["case"], r["expected"], r["actual"]))
    print("TASK4 PHASE3 COMMAND CONTRACT TESTS: {} ({}/{})".format(
        summary["verdict"], summary["n_passed"], summary["n_cases"]))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
