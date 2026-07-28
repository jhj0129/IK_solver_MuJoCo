#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, List

import yaml


JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

WRIST_INDEX = {
    "JOINT4": 3,
    "JOINT5": 4,
    "JOINT6": 5,
}


def quintic_progress(ratio: float) -> float:
    return (
        10.0 * ratio ** 3
        - 15.0 * ratio ** 4
        + 6.0 * ratio ** 5
    )


def load_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(
            f"파일을 찾지 못했습니다: {path}"
        )

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise RuntimeError(
            f"올바르지 않은 YAML입니다: {path}"
        )

    return data


def to_degrees(value: float) -> float:
    return math.degrees(value)


def interpolate_poly5(
    start: List[float],
    goal: List[float],
    sample_count: int,
) -> List[List[float]]:
    if len(start) != 6 or len(goal) != 6:
        raise RuntimeError(
            "branch_switch 관절 벡터는 6개여야 합니다."
        )

    if sample_count < 2:
        raise RuntimeError(
            "branch_switch.samples는 2 이상이어야 합니다."
        )

    path: List[List[float]] = []

    for sample_index in range(sample_count):
        ratio = sample_index / (sample_count - 1)
        progress = quintic_progress(ratio)

        q = [
            start_value
            + progress * (goal_value - start_value)
            for start_value, goal_value
            in zip(start, goal)
        ]

        path.append(q)

    return path


def validate_limit_order(
    cable_limits: dict,
) -> None:
    if not cable_limits.get(
        "collection",
        {},
    ).get("complete", False):
        raise RuntimeError(
            "cable_limits.yaml의 collection.complete가 "
            "true가 아닙니다."
        )

    for joint in WRIST_INDEX:
        data = cable_limits["limits"][joint]

        values = [
            data.get("mechanical_lower"),
            data.get("hard_lower"),
            data.get("soft_lower"),
            data.get("neutral"),
            data.get("soft_upper"),
            data.get("hard_upper"),
            data.get("mechanical_upper"),
        ]

        if any(value is None for value in values):
            raise RuntimeError(
                f"{joint} cable limit이 완전하지 않습니다."
            )

        (
            mechanical_lower,
            hard_lower,
            soft_lower,
            neutral,
            soft_upper,
            hard_upper,
            mechanical_upper,
        ) = [float(value) for value in values]

        valid = (
            mechanical_lower
            <= hard_lower
            <= soft_lower
            <= neutral
            <= soft_upper
            <= hard_upper
            <= mechanical_upper
        )

        if not valid:
            raise RuntimeError(
                f"{joint} cable limit 순서가 잘못됐습니다."
            )


def main() -> int:
    root = Path.home() / "IK_solver_MuJoCo"

    task_path = (
        Path(sys.argv[1]).expanduser()
        if len(sys.argv) >= 2
        else root
        / "src/drok_arm_control/config/pick_place_task.yaml"
    )

    cable_path = (
        Path(sys.argv[2]).expanduser()
        if len(sys.argv) >= 3
        else root
        / "src/drok_arm_control/config/cable_limits.yaml"
    )

    task = load_yaml(task_path)
    cable = load_yaml(cable_path)

    validate_limit_order(cable)

    branch = task.get("branch_switch")

    if not isinstance(branch, dict):
        raise RuntimeError(
            "pick_place_task.yaml에 branch_switch가 없습니다."
        )

    start_q = [
        float(value)
        for value in branch["pick_joint_positions"]
    ]

    goal_q = [
        float(value)
        for value in branch["place_joint_positions"]
    ]

    sample_count = int(branch["samples"])

    path = interpolate_poly5(
        start=start_q,
        goal=goal_q,
        sample_count=sample_count,
    )

    print("=" * 88)
    print("DROK ARM V4 CABLE-LIMIT PREFLIGHT")
    print("=" * 88)

    print("Task YAML :", task_path)
    print("Cable YAML:", cable_path)
    print("Samples   :", sample_count)

    print()
    print("VIRTUAL CABLE LIMITS")

    for joint in WRIST_INDEX:
        data = cable["limits"][joint]

        print()
        print(joint)

        for field in [
            "neutral",
            "soft_lower",
            "soft_upper",
            "hard_lower",
            "hard_upper",
        ]:
            value = float(data[field])

            print(
                f"  {field:12s}: "
                f"{value:+.9f} rad "
                f"({to_degrees(value):+8.3f} deg)"
            )

    first_soft_violation = None
    first_hard_violation = None

    minimum_soft_margin = float("inf")
    minimum_hard_margin = float("inf")

    minimum_soft_info = None
    minimum_hard_info = None

    maximum_sample_step = 0.0

    previous_q = None

    for sample_index, q in enumerate(path):
        if previous_q is not None:
            step = max(
                abs(current - previous)
                for current, previous
                in zip(q, previous_q)
            )

            maximum_sample_step = max(
                maximum_sample_step,
                step,
            )

        previous_q = q

        for joint, joint_index in WRIST_INDEX.items():
            value = q[joint_index]
            data = cable["limits"][joint]

            soft_lower = float(data["soft_lower"])
            soft_upper = float(data["soft_upper"])
            hard_lower = float(data["hard_lower"])
            hard_upper = float(data["hard_upper"])

            soft_margin = min(
                value - soft_lower,
                soft_upper - value,
            )

            hard_margin = min(
                value - hard_lower,
                hard_upper - value,
            )

            if soft_margin < minimum_soft_margin:
                minimum_soft_margin = soft_margin
                minimum_soft_info = (
                    sample_index,
                    joint,
                    value,
                )

            if hard_margin < minimum_hard_margin:
                minimum_hard_margin = hard_margin
                minimum_hard_info = (
                    sample_index,
                    joint,
                    value,
                )

            if (
                soft_margin < 0.0
                and first_soft_violation is None
            ):
                first_soft_violation = (
                    sample_index,
                    joint,
                    value,
                    soft_lower,
                    soft_upper,
                )

            if (
                hard_margin < 0.0
                and first_hard_violation is None
            ):
                first_hard_violation = (
                    sample_index,
                    joint,
                    value,
                    hard_lower,
                    hard_upper,
                )

    print()
    print("=" * 88)
    print("BRANCH-SWITCH ENDPOINTS")
    print("=" * 88)

    for label, q in [
        ("PICK_BRANCH", start_q),
        ("PLACE_BRANCH", goal_q),
    ]:
        print()
        print(label)

        for joint in ["JOINT4", "JOINT5", "JOINT6"]:
            index = WRIST_INDEX[joint]
            value = q[index]

            print(
                f"  {joint}: "
                f"{value:+.9f} rad "
                f"({to_degrees(value):+8.3f} deg)"
            )

    print()
    print("=" * 88)
    print("PREFLIGHT RESULT")
    print("=" * 88)

    print(
        "maximum_sample_step:",
        f"{maximum_sample_step:.9f} rad",
        f"({to_degrees(maximum_sample_step):.3f} deg)",
    )

    if minimum_soft_info is not None:
        sample, joint, value = minimum_soft_info

        print(
            "minimum_soft_margin:",
            f"{minimum_soft_margin:+.9f} rad",
            f"sample={sample}",
            f"joint={joint}",
            f"q={value:+.9f}",
        )

    if minimum_hard_info is not None:
        sample, joint, value = minimum_hard_info

        print(
            "minimum_hard_margin:",
            f"{minimum_hard_margin:+.9f} rad",
            f"sample={sample}",
            f"joint={joint}",
            f"q={value:+.9f}",
        )

    print()

    if first_soft_violation is None:
        print("SOFT LIMIT: PASS")
    else:
        (
            sample,
            joint,
            value,
            lower,
            upper,
        ) = first_soft_violation

        print("SOFT LIMIT: FAIL")
        print(
            f"  first violation: sample={sample}, "
            f"joint={joint}"
        )
        print(
            f"  q={value:+.9f} rad "
            f"({to_degrees(value):+.3f} deg)"
        )
        print(
            f"  allowed=[{lower:+.9f}, "
            f"{upper:+.9f}] rad"
        )

    print()

    if first_hard_violation is None:
        print("HARD LIMIT: PASS")
    else:
        (
            sample,
            joint,
            value,
            lower,
            upper,
        ) = first_hard_violation

        print("HARD LIMIT: FAIL")
        print(
            f"  first violation: sample={sample}, "
            f"joint={joint}"
        )
        print(
            f"  q={value:+.9f} rad "
            f"({to_degrees(value):+.3f} deg)"
        )
        print(
            f"  allowed=[{lower:+.9f}, "
            f"{upper:+.9f}] rad"
        )

    overall_pass = (
        first_soft_violation is None
        and first_hard_violation is None
    )

    print()
    print(
        "OVERALL:",
        "PASS" if overall_pass else "FAIL",
    )

    if not overall_pass:
        print()
        print(
            "현재 V4 wrist branch-switch는 "
            "V5 cable-aware planner에서 실행하면 안 됩니다."
        )

    return 0 if overall_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
