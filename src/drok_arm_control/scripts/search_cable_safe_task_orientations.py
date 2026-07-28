#!/usr/bin/env python3

from __future__ import annotations

import math
import re
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

GEOMETRY_PATH = (
    ROOT
    / "src/drok_arm_kinematics/config/robot_geometry.yaml"
)

CABLE_PATH = (
    ROOT
    / "src/drok_arm_control/config/cable_limits.yaml"
)

# 현재 검증된 작업 좌표
PICK_GRASP = [0.450, 0.180, 0.272]
PICK_SAFE = [0.450, 0.180, 0.360]

PLACE_RELEASE = [0.450, -0.180, 0.274]
PLACE_SAFE = [0.450, -0.180, 0.360]

# 접근축 X는 유지하고, X축 중심 roll만 변경한다.
ROLL_CANDIDATES_DEG = [
    -60.0,
    -30.0,
    0.0,
    30.0,
    60.0,
]

HOME_Q = [
    -0.000001628,
    0.297361544,
    0.296742637,
    -0.000030712,
    0.000061231,
    0.000102331,
]

KNOWN_PICK_GRASP_Q = [
    0.665747879,
    1.549955746,
    0.801253505,
    -0.714124649,
    0.956803404,
    -1.403018230,
]

KNOWN_PICK_SAFE_Q = [
    0.665742897,
    1.452671010,
    0.987980466,
    -0.518551043,
    0.791099757,
    -1.100682648,
]

KNOWN_PLACE_RELEASE_Q = [
    -0.664033948,
    1.546809382,
    0.804865295,
    0.712247861,
    -0.951429386,
    0.560003204,
]

KNOWN_PLACE_SAFE_Q = [
    -0.664031336,
    1.452671458,
    0.987985633,
    0.520062981,
    -0.789770260,
    0.263982400,
]


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


CABLE = load_yaml(CABLE_PATH)

NEUTRAL = {
    joint: float(CABLE["limits"][joint]["neutral"])
    for joint in ["JOINT4", "JOINT5", "JOINT6"]
}


def neutralized_seed(q: List[float]) -> List[float]:
    result = q.copy()

    result[3] = NEUTRAL["JOINT4"]
    result[4] = NEUTRAL["JOINT5"]
    result[5] = NEUTRAL["JOINT6"]

    return result


PICK_SEEDS = [
    HOME_Q,
    KNOWN_PICK_GRASP_Q,
    KNOWN_PICK_SAFE_Q,
    neutralized_seed(KNOWN_PICK_GRASP_Q),
    neutralized_seed(KNOWN_PICK_SAFE_Q),
]

PLACE_SEEDS = [
    HOME_Q,
    KNOWN_PLACE_RELEASE_Q,
    KNOWN_PLACE_SAFE_Q,
    neutralized_seed(KNOWN_PLACE_RELEASE_Q),
    neutralized_seed(KNOWN_PLACE_SAFE_Q),
]


def solve_pose(
    position: List[float],
    roll: float,
    seed: List[float],
    timeout_seconds: float = 20.0,
) -> Optional[List[float]]:
    x, y, z = position

    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "solve_ik_pose",
        str(GEOMETRY_PATH),
        f"{x:.12f}",
        f"{y:.12f}",
        f"{z:.12f}",
        f"{roll:.12f}",
        "0.0",
        "0.0",
        *[f"{value:.12f}" for value in seed],
    ]

    try:
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired:
        return None

    success_match = re.search(
        r"Success\s*:\s*(true|false)",
        result.stdout,
        flags=re.IGNORECASE,
    )

    joint_match = re.search(
        r"JOINT_RESULT=([^\r\n]+)",
        result.stdout,
    )

    success = (
        result.returncode == 0
        and success_match is not None
        and success_match.group(1).lower() == "true"
        and joint_match is not None
    )

    if not success:
        return None

    q = [
        float(value)
        for value in joint_match.group(1).split(",")
    ]

    return q if len(q) == 6 else None


def is_duplicate(
    solutions: List[List[float]],
    candidate: List[float],
    tolerance: float = 1.0e-4,
) -> bool:
    return any(
        max(
            abs(a - b)
            for a, b in zip(solution, candidate)
        ) < tolerance
        for solution in solutions
    )


def cable_status(q: List[float]) -> Dict[str, object]:
    soft_pass = True
    hard_pass = True

    minimum_soft_margin = float("inf")
    minimum_hard_margin = float("inf")

    soft_worst = None
    hard_worst = None

    for joint, index in [
        ("JOINT4", 3),
        ("JOINT5", 4),
        ("JOINT6", 5),
    ]:
        value = q[index]
        data = CABLE["limits"][joint]

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
            soft_worst = (joint, value)

        if hard_margin < minimum_hard_margin:
            minimum_hard_margin = hard_margin
            hard_worst = (joint, value)

        if soft_margin < 0.0:
            soft_pass = False

        if hard_margin < 0.0:
            hard_pass = False

    neutral_cost = (
        abs(q[3] - NEUTRAL["JOINT4"])
        + abs(q[4] - NEUTRAL["JOINT5"])
        + abs(q[5] - NEUTRAL["JOINT6"])
    )

    return {
        "soft_pass": soft_pass,
        "hard_pass": hard_pass,
        "soft_margin": minimum_soft_margin,
        "hard_margin": minimum_hard_margin,
        "soft_worst": soft_worst,
        "hard_worst": hard_worst,
        "neutral_cost": neutral_cost,
    }


def evaluate_pair(
    first_q: List[float],
    second_q: List[float],
) -> Dict[str, object]:
    first_status = cable_status(first_q)
    second_status = cable_status(second_q)

    maximum_delta = max(
        abs(a - b)
        for a, b in zip(first_q, second_q)
    )

    return {
        "hard_pass": (
            first_status["hard_pass"]
            and second_status["hard_pass"]
        ),
        "soft_pass": (
            first_status["soft_pass"]
            and second_status["soft_pass"]
        ),
        "soft_margin": min(
            float(first_status["soft_margin"]),
            float(second_status["soft_margin"]),
        ),
        "hard_margin": min(
            float(first_status["hard_margin"]),
            float(second_status["hard_margin"]),
        ),
        "neutral_cost": (
            float(first_status["neutral_cost"])
            + float(second_status["neutral_cost"])
        ),
        "maximum_delta": maximum_delta,
    }


def search_pick() -> List[dict]:
    results = []

    for roll_deg in ROLL_CANDIDATES_DEG:
        roll = math.radians(roll_deg)

        grasp_solutions: List[List[float]] = []

        print(
            f"PICK roll={roll_deg:+.1f} deg 탐색 중..."
        )

        for seed in PICK_SEEDS:
            q_grasp = solve_pose(
                PICK_GRASP,
                roll,
                seed,
            )

            if q_grasp is None:
                continue

            if is_duplicate(grasp_solutions, q_grasp):
                continue

            grasp_solutions.append(q_grasp)

            q_safe = solve_pose(
                PICK_SAFE,
                roll,
                q_grasp,
            )

            if q_safe is None:
                continue

            evaluation = evaluate_pair(
                q_grasp,
                q_safe,
            )

            results.append(
                {
                    "side": "PICK",
                    "roll_deg": roll_deg,
                    "low_q": q_grasp,
                    "safe_q": q_safe,
                    **evaluation,
                }
            )

    return results


def search_place() -> List[dict]:
    results = []

    for roll_deg in ROLL_CANDIDATES_DEG:
        roll = math.radians(roll_deg)

        safe_solutions: List[List[float]] = []

        print(
            f"PLACE roll={roll_deg:+.1f} deg 탐색 중..."
        )

        for seed in PLACE_SEEDS:
            q_safe = solve_pose(
                PLACE_SAFE,
                roll,
                seed,
            )

            if q_safe is None:
                continue

            if is_duplicate(safe_solutions, q_safe):
                continue

            safe_solutions.append(q_safe)

            q_release = solve_pose(
                PLACE_RELEASE,
                roll,
                q_safe,
            )

            if q_release is None:
                continue

            evaluation = evaluate_pair(
                q_safe,
                q_release,
            )

            results.append(
                {
                    "side": "PLACE",
                    "roll_deg": roll_deg,
                    "low_q": q_release,
                    "safe_q": q_safe,
                    **evaluation,
                }
            )

    return results


def ranking_key(item: dict) -> Tuple:
    return (
        0 if item["hard_pass"] else 1,
        0 if item["soft_pass"] else 1,
        -float(item["soft_margin"]),
        float(item["neutral_cost"]),
        float(item["maximum_delta"]),
    )


def print_results(
    title: str,
    results: List[dict],
) -> None:
    print()
    print("=" * 100)
    print(title)
    print("=" * 100)

    if not results:
        print("유효한 IK endpoint 쌍을 찾지 못했습니다.")
        return

    results.sort(key=ranking_key)

    for rank, item in enumerate(
        results[:8],
        start=1,
    ):
        print()
        print(
            f"RANK {rank}: "
            f"roll={item['roll_deg']:+.1f} deg"
        )

        print(
            f"  HARD={'PASS' if item['hard_pass'] else 'FAIL'}  "
            f"SOFT={'PASS' if item['soft_pass'] else 'FAIL'}"
        )

        print(
            f"  soft_margin={item['soft_margin']:+.6f} rad  "
            f"hard_margin={item['hard_margin']:+.6f} rad"
        )

        print(
            f"  max_endpoint_delta="
            f"{item['maximum_delta']:.6f} rad"
        )

        print(
            "  LOW_Q =["
            + ", ".join(
                f"{value:.9f}"
                for value in item["low_q"]
            )
            + "]"
        )

        print(
            "  SAFE_Q=["
            + ", ".join(
                f"{value:.9f}"
                for value in item["safe_q"]
            )
            + "]"
        )

        print(
            "  wrist low = ["
            + ", ".join(
                f"{math.degrees(item['low_q'][index]):+.2f}°"
                for index in [3, 4, 5]
            )
            + "]"
        )

        print(
            "  wrist safe= ["
            + ", ".join(
                f"{math.degrees(item['safe_q'][index]):+.2f}°"
                for index in [3, 4, 5]
            )
            + "]"
        )


def main() -> int:
    print("=" * 100)
    print("DROK ARM CABLE-SAFE PICK/PLACE ORIENTATION SEARCH")
    print("=" * 100)

    print(
        "Roll candidates:",
        ROLL_CANDIDATES_DEG,
    )

    print(
        "Pitch=0°, Yaw=0°를 유지하며 "
        "tool X축 중심 roll만 변경합니다."
    )

    print()

    pick_results = search_pick()
    place_results = search_place()

    print_results(
        "PICK RESULTS",
        pick_results,
    )

    print_results(
        "PLACE RESULTS",
        place_results,
    )

    pick_soft = [
        result
        for result in pick_results
        if result["soft_pass"]
    ]

    place_soft = [
        result
        for result in place_results
        if result["soft_pass"]
    ]

    print()
    print("=" * 100)
    print("SUMMARY")
    print("=" * 100)

    print(
        "PICK soft-safe candidates :",
        len(pick_soft),
    )

    print(
        "PLACE soft-safe candidates:",
        len(place_soft),
    )

    if pick_soft and place_soft:
        print()
        print(
            "SOFT-SAFE PICK과 PLACE 후보가 모두 발견됐습니다."
        )
        print(
            "다음 단계에서 각 1위 후보를 사용해 "
            "Cartesian 전체 경로와 중립 자세 연결을 검사합니다."
        )
        return 0

    print()
    print(
        "현재 roll 후보만으로는 soft-safe 작업 자세가 "
        "충분하지 않습니다."
    )
    print(
        "다음 탐색에서는 roll 간격을 줄이거나 "
        "접근 위치·pitch 후보를 추가해야 합니다."
    )

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
