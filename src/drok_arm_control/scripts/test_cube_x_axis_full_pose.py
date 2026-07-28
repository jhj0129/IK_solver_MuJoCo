#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import re
import subprocess
from pathlib import Path
from typing import List, Sequence


ROOT = Path.home() / "IK_solver_MuJoCo"

GEOMETRY_PATH = (
    ROOT
    / "src/drok_arm_kinematics/config/robot_geometry.yaml"
)

# 기존 exact full-pose IK에서 이미 성공했던 PICK_DOWN 해
PICK_GRASP_Q = [
    0.665755843173,
    1.549939642234,
    0.801245753006,
    -0.714118916746,
    0.956806949236,
    -1.403017825801,
]

# 기존 exact full-pose IK에서 이미 성공했던 PLACE_DOWN 해
PLACE_RELEASE_Q = [
    -0.664027967214,
    1.546815502872,
    0.804858584557,
    0.712251414446,
    -0.951431335140,
    0.560006343795,
]

RPY = [
    0.0,
    0.0,
    0.0,
]

# GRASP 위치에서 -X 방향으로 10 mm씩 후퇴
X_VALUES = [
    0.45,
    0.44,
    0.43,
    0.42,
    0.41,
    0.40,
    0.39,
    0.38,
    0.37,
    0.36,
]


def solve_full_pose(
    position: Sequence[float],
    seed: Sequence[float],
) -> tuple[bool, List[float], str]:
    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "solve_ik_pose",
        str(GEOMETRY_PATH),
        f"{position[0]:.12f}",
        f"{position[1]:.12f}",
        f"{position[2]:.12f}",
        f"{RPY[0]:.12f}",
        f"{RPY[1]:.12f}",
        f"{RPY[2]:.12f}",
        *[
            f"{value:.12f}"
            for value in seed
        ],
    ]

    environment = os.environ.copy()
    environment["DROK_IK_MODE"] = "full"

    try:
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=20.0,
            env=environment,
        )
    except subprocess.TimeoutExpired:
        return False, [], "timeout"

    success_match = re.search(
        r"Success\s*:\s*(true|false)",
        result.stdout,
        flags=re.IGNORECASE,
    )

    joint_match = re.search(
        r"JOINT_RESULT=([^\r\n]+)",
        result.stdout,
    )

    if (
        result.returncode != 0
        or success_match is None
        or success_match.group(1).lower() != "true"
        or joint_match is None
    ):
        return False, [], result.stdout

    try:
        q = [
            float(value)
            for value
            in joint_match.group(1).split(",")
        ]
    except ValueError:
        return False, [], result.stdout

    if len(q) != 6:
        return False, [], result.stdout

    return True, q, result.stdout


def format_degrees(
    values: Sequence[float],
) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+8.3f}°"
            for value in values
        )
        + "]"
    )


def test_line(
    name: str,
    y: float,
    z: float,
    initial_q: Sequence[float],
) -> bool:
    print()
    print("=" * 100)
    print(name)
    print("=" * 100)

    print(
        "Fixed RPY:",
        format_degrees(RPY),
    )

    print(
        "Initial q:",
        format_degrees(initial_q),
    )

    reference_q = list(initial_q)
    largest_step = 0.0
    largest_step_x = 0.0

    for index, x in enumerate(
        X_VALUES,
        start=1,
    ):
        position = [
            x,
            y,
            z,
        ]

        success, solved_q, output = solve_full_pose(
            position,
            reference_q,
        )

        print()
        print(
            f"[{index:02d}/{len(X_VALUES):02d}] "
            f"xyz=[{x:+.3f}, {y:+.3f}, {z:+.3f}]"
        )

        if not success:
            print("  RESULT: FAIL")
            print()

            for line in output.splitlines():
                if any(
                    key in line
                    for key in [
                        "Success",
                        "Position error",
                        "Orientation error",
                        "Message",
                    ]
                ):
                    print(" ", line)

            return False

        deltas = [
            solved - previous
            for solved, previous
            in zip(solved_q, reference_q)
        ]

        maximum_delta = max(
            abs(value)
            for value in deltas
        )

        if maximum_delta > largest_step:
            largest_step = maximum_delta
            largest_step_x = x

        print(
            "  q      :",
            format_degrees(solved_q),
        )

        print(
            "  delta  :",
            format_degrees(deltas),
        )

        print(
            "  max Δ  :",
            f"{math.degrees(maximum_delta):.3f}°",
        )

        reference_q = solved_q

    print()
    print(
        "Largest step:",
        f"{math.degrees(largest_step):.3f}°",
        f"at x={largest_step_x:.3f}",
    )

    if largest_step > math.radians(30.0):
        print(
            "RESULT: WARNING — X축 경로에서 "
            "30°를 넘는 branch 변화가 있습니다."
        )
        return False

    print(
        "RESULT: PASS — 고정된 면 정렬 자세로 "
        "X축 접근/후퇴가 가능합니다."
    )

    return True


def main() -> int:
    print("=" * 100)
    print("DROK ARM CUBE X-AXIS FULL-POSE DRY RUN")
    print("=" * 100)

    print("Robot execution: 비활성화")
    print("IK mode        : full")
    print("Orientation    : RPY [0, 0, 0] 고정")
    print("Motion         : x=0.45 → 0.36 m")
    print("Step           : 10 mm")

    pick_success = test_line(
        name="PICK GRASP → PICK PREGRASP",
        y=0.180,
        z=0.272,
        initial_q=PICK_GRASP_Q,
    )

    place_success = test_line(
        name="PLACE RELEASE → PLACE PREPLACE",
        y=-0.180,
        z=0.274,
        initial_q=PLACE_RELEASE_Q,
    )

    print()
    print("=" * 100)
    print("FINAL SUMMARY")
    print("=" * 100)

    print(
        "PICK X-axis path :",
        "PASS" if pick_success else "FAIL",
    )

    print(
        "PLACE X-axis path:",
        "PASS" if place_success else "FAIL",
    )

    if pick_success and place_success:
        print()
        print(
            "OVERALL PASS — 같은 orientation을 유지한 채 "
            "X축 접근과 후퇴가 가능합니다."
        )
        return 0

    print()
    print(
        "OVERALL FAIL — 실패한 지점에서 "
        "orientation 후보를 다시 선택해야 합니다."
    )

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
