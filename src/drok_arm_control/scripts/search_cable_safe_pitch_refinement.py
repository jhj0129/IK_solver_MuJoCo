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

PITCH_CANDIDATES_DEG = [
    -25.0,
    -20.0,
    -15.0,
    -10.0,
    -5.0,
    0.0,
    5.0,
    10.0,
    15.0,
    20.0,
    25.0,
]

HOME_Q = [
    -0.000001628,
    0.297361544,
    0.296742637,
    -0.000030712,
    0.000061231,
    0.000102331,
]

# 이전 탐색에서 가장 유리했던 PICK roll=-60° 해
PICK_LOW_Q = [
    0.665747907,
    1.549955937,
    0.801253497,
    -0.714124777,
    0.956803618,
    -0.355820901,
]

PICK_SAFE_Q = [
    0.665742897,
    1.452671010,
    0.987980466,
    -0.518551044,
    0.791099756,
    -0.053485099,
]

# 이전 탐색에서 가장 유리했던 PLACE roll=+30° 해
PLACE_LOW_Q = [
    -0.664031558,
    1.546807229,
    0.804862093,
    0.712251926,
    -0.951428007,
    0.036407113,
]

PLACE_SAFE_Q = [
    -0.664033779,
    1.452672469,
    0.987988602,
    0.520057781,
    -0.789771824,
    -0.259621303,
]

PICK_LOW_POSITION = [
    0.450,
    0.180,
    0.272,
]

PICK_SAFE_POSITION = [
    0.450,
    0.180,
    0.360,
]

PLACE_LOW_POSITION = [
    0.450,
    -0.180,
    0.274,
]

PLACE_SAFE_POSITION = [
    0.450,
    -0.180,
    0.360,
]


def load_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(
            f"파일을 찾지 못했습니다: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise RuntimeError(
            f"올바르지 않은 YAML입니다: {path}"
        )

    return data


CABLE = load_yaml(CABLE_PATH)

WRIST_JOINTS = [
    ("JOINT4", 3),
    ("JOINT5", 4),
    ("JOINT6", 5),
]

NEUTRAL = {
    joint: float(
        CABLE["limits"][joint]["neutral"]
    )
    for joint, _ in WRIST_JOINTS
}


def neutralized_seed(
    source: List[float],
) -> List[float]:
    result = source.copy()

    result[3] = NEUTRAL["JOINT4"]
    result[4] = NEUTRAL["JOINT5"]
    result[5] = NEUTRAL["JOINT6"]

    return result


PICK_SEEDS = [
    PICK_LOW_Q,
    PICK_SAFE_Q,
    neutralized_seed(PICK_LOW_Q),
    HOME_Q,
]

PLACE_SEEDS = [
    PLACE_SAFE_Q,
    PLACE_LOW_Q,
    neutralized_seed(PLACE_SAFE_Q),
    HOME_Q,
]


def solve_pose(
    position: List[float],
    roll_rad: float,
    pitch_rad: float,
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
        f"{roll_rad:.12f}",
        f"{pitch_rad:.12f}",
        "0.0",
        *[
            f"{value:.12f}"
            for value in seed
        ],
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
        and success_match.group(1).lower()
        == "true"
        and joint_match is not None
    )

    if not success:
        return None

    values = [
        float(value)
        for value
        in joint_match.group(1).split(",")
    ]

    if len(values) != 6:
        return None

    return values


def is_duplicate(
    existing_pairs: List[
        Tuple[List[float], List[float]]
    ],
    first_q: List[float],
    second_q: List[float],
    tolerance: float = 1.0e-4,
) -> bool:
    for saved_first, saved_second in existing_pairs:
        first_error = max(
            abs(a - b)
            for a, b
            in zip(saved_first, first_q)
        )

        second_error = max(
            abs(a - b)
            for a, b
            in zip(saved_second, second_q)
        )

        if (
            first_error < tolerance
            and second_error < tolerance
        ):
            return True

    return False


def evaluate_configuration(
    q: List[float],
) -> Dict[str, object]:
    soft_pass = True
    hard_pass = True

    minimum_soft_margin = float("inf")
    minimum_hard_margin = float("inf")

    worst_soft_joint = ""
    worst_hard_joint = ""

    for joint, index in WRIST_JOINTS:
        value = q[index]
        data = CABLE["limits"][joint]

        soft_lower = float(
            data["soft_lower"]
        )
        soft_upper = float(
            data["soft_upper"]
        )

        hard_lower = float(
            data["hard_lower"]
        )
        hard_upper = float(
            data["hard_upper"]
        )

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
            worst_soft_joint = joint

        if hard_margin < minimum_hard_margin:
            minimum_hard_margin = hard_margin
            worst_hard_joint = joint

        if soft_margin < 0.0:
            soft_pass = False

        if hard_margin < 0.0:
            hard_pass = False

    neutral_cost = sum(
        abs(
            q[index]
            - NEUTRAL[joint]
        )
        for joint, index
        in WRIST_JOINTS
    )

    return {
        "soft_pass": soft_pass,
        "hard_pass": hard_pass,
        "soft_margin": minimum_soft_margin,
        "hard_margin": minimum_hard_margin,
        "worst_soft_joint": worst_soft_joint,
        "worst_hard_joint": worst_hard_joint,
        "neutral_cost": neutral_cost,
    }


def evaluate_pair(
    low_q: List[float],
    safe_q: List[float],
) -> Dict[str, object]:
    low_status = evaluate_configuration(
        low_q
    )

    safe_status = evaluate_configuration(
        safe_q
    )

    maximum_delta = max(
        abs(low - safe)
        for low, safe
        in zip(low_q, safe_q)
    )

    if (
        float(low_status["soft_margin"])
        <= float(safe_status["soft_margin"])
    ):
        worst_soft_joint = str(
            low_status["worst_soft_joint"]
        )
    else:
        worst_soft_joint = str(
            safe_status["worst_soft_joint"]
        )

    if (
        float(low_status["hard_margin"])
        <= float(safe_status["hard_margin"])
    ):
        worst_hard_joint = str(
            low_status["worst_hard_joint"]
        )
    else:
        worst_hard_joint = str(
            safe_status["worst_hard_joint"]
        )

    return {
        "soft_pass": (
            bool(low_status["soft_pass"])
            and bool(safe_status["soft_pass"])
        ),
        "hard_pass": (
            bool(low_status["hard_pass"])
            and bool(safe_status["hard_pass"])
        ),
        "soft_margin": min(
            float(low_status["soft_margin"]),
            float(safe_status["soft_margin"]),
        ),
        "hard_margin": min(
            float(low_status["hard_margin"]),
            float(safe_status["hard_margin"]),
        ),
        "worst_soft_joint": worst_soft_joint,
        "worst_hard_joint": worst_hard_joint,
        "neutral_cost": (
            float(low_status["neutral_cost"])
            + float(safe_status["neutral_cost"])
        ),
        "maximum_delta": maximum_delta,
    }


def search_pick() -> List[dict]:
    results: List[dict] = []

    roll_deg = -60.0
    roll_rad = math.radians(roll_deg)

    for pitch_deg in PITCH_CANDIDATES_DEG:
        pitch_rad = math.radians(pitch_deg)

        print(
            f"PICK: roll={roll_deg:+.1f}°, "
            f"pitch={pitch_deg:+.1f}° 탐색 중..."
        )

        saved_pairs: List[
            Tuple[List[float], List[float]]
        ] = []

        for seed in PICK_SEEDS:
            low_q = solve_pose(
                PICK_LOW_POSITION,
                roll_rad,
                pitch_rad,
                seed,
            )

            if low_q is None:
                continue

            safe_q = solve_pose(
                PICK_SAFE_POSITION,
                roll_rad,
                pitch_rad,
                low_q,
            )

            if safe_q is None:
                continue

            if is_duplicate(
                saved_pairs,
                low_q,
                safe_q,
            ):
                continue

            saved_pairs.append(
                (low_q, safe_q)
            )

            result = evaluate_pair(
                low_q,
                safe_q,
            )

            results.append(
                {
                    "task": "PICK",
                    "roll_deg": roll_deg,
                    "pitch_deg": pitch_deg,
                    "low_q": low_q,
                    "safe_q": safe_q,
                    **result,
                }
            )

    return results


def search_place() -> List[dict]:
    results: List[dict] = []

    roll_deg = 30.0
    roll_rad = math.radians(roll_deg)

    for pitch_deg in PITCH_CANDIDATES_DEG:
        pitch_rad = math.radians(pitch_deg)

        print(
            f"PLACE: roll={roll_deg:+.1f}°, "
            f"pitch={pitch_deg:+.1f}° 탐색 중..."
        )

        saved_pairs: List[
            Tuple[List[float], List[float]]
        ] = []

        for seed in PLACE_SEEDS:
            safe_q = solve_pose(
                PLACE_SAFE_POSITION,
                roll_rad,
                pitch_rad,
                seed,
            )

            if safe_q is None:
                continue

            low_q = solve_pose(
                PLACE_LOW_POSITION,
                roll_rad,
                pitch_rad,
                safe_q,
            )

            if low_q is None:
                continue

            if is_duplicate(
                saved_pairs,
                low_q,
                safe_q,
            ):
                continue

            saved_pairs.append(
                (low_q, safe_q)
            )

            result = evaluate_pair(
                low_q,
                safe_q,
            )

            results.append(
                {
                    "task": "PLACE",
                    "roll_deg": roll_deg,
                    "pitch_deg": pitch_deg,
                    "low_q": low_q,
                    "safe_q": safe_q,
                    **result,
                }
            )

    return results


def ranking_key(
    item: dict,
) -> Tuple:
    return (
        0 if item["hard_pass"] else 1,
        0 if item["soft_pass"] else 1,
        -float(item["soft_margin"]),
        float(item["neutral_cost"]),
        float(item["maximum_delta"]),
    )


def format_wrist(
    q: List[float],
) -> str:
    values = [
        math.degrees(q[index])
        for _, index in WRIST_JOINTS
    ]

    return (
        "["
        + ", ".join(
            f"{value:+.2f}°"
            for value in values
        )
        + "]"
    )


def print_results(
    title: str,
    results: List[dict],
) -> None:
    print()
    print("=" * 104)
    print(title)
    print("=" * 104)

    if not results:
        print(
            "유효한 IK endpoint 쌍이 없습니다."
        )
        return

    results.sort(key=ranking_key)

    for rank, item in enumerate(
        results[:12],
        start=1,
    ):
        print()
        print(
            f"RANK {rank}: "
            f"roll={item['roll_deg']:+.1f}°, "
            f"pitch={item['pitch_deg']:+.1f}°"
        )

        print(
            f"  HARD="
            f"{'PASS' if item['hard_pass'] else 'FAIL'}  "
            f"SOFT="
            f"{'PASS' if item['soft_pass'] else 'FAIL'}"
        )

        print(
            f"  soft_margin="
            f"{item['soft_margin']:+.6f} rad "
            f"({math.degrees(item['soft_margin']):+.2f}°), "
            f"worst={item['worst_soft_joint']}"
        )

        print(
            f"  hard_margin="
            f"{item['hard_margin']:+.6f} rad "
            f"({math.degrees(item['hard_margin']):+.2f}°), "
            f"worst={item['worst_hard_joint']}"
        )

        print(
            f"  endpoint_delta="
            f"{item['maximum_delta']:.6f} rad "
            f"({math.degrees(item['maximum_delta']):.2f}°)"
        )

        print(
            f"  wrist LOW : "
            f"{format_wrist(item['low_q'])}"
        )

        print(
            f"  wrist SAFE: "
            f"{format_wrist(item['safe_q'])}"
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


def main() -> int:
    print("=" * 104)
    print(
        "DROK ARM CABLE-SAFE PITCH REFINEMENT"
    )
    print("=" * 104)

    print(
        "PICK : roll=-60° 고정"
    )
    print(
        "PLACE: roll=+30° 고정"
    )
    print(
        "Yaw  : 0° 고정"
    )
    print(
        "Pitch candidates:",
        PITCH_CANDIDATES_DEG,
    )
    print()

    pick_results = search_pick()
    place_results = search_place()

    print_results(
        "PICK PITCH RESULTS",
        pick_results,
    )

    print_results(
        "PLACE PITCH RESULTS",
        place_results,
    )

    pick_soft_safe = [
        item
        for item in pick_results
        if item["soft_pass"]
    ]

    place_soft_safe = [
        item
        for item in place_results
        if item["soft_pass"]
    ]

    print()
    print("=" * 104)
    print("SUMMARY")
    print("=" * 104)

    print(
        "PICK soft-safe candidates :",
        len(pick_soft_safe),
    )

    print(
        "PLACE soft-safe candidates:",
        len(place_soft_safe),
    )

    if pick_soft_safe:
        pick_soft_safe.sort(
            key=ranking_key
        )

        best = pick_soft_safe[0]

        print()
        print(
            "BEST PICK:",
            f"roll={best['roll_deg']:+.1f}°,",
            f"pitch={best['pitch_deg']:+.1f}°",
        )

    if place_soft_safe:
        place_soft_safe.sort(
            key=ranking_key
        )

        best = place_soft_safe[0]

        print(
            "BEST PLACE:",
            f"roll={best['roll_deg']:+.1f}°,",
            f"pitch={best['pitch_deg']:+.1f}°",
        )

    if (
        pick_soft_safe
        and place_soft_safe
    ):
        print()
        print(
            "PICK과 PLACE 모두 soft-safe "
            "endpoint 후보를 찾았습니다."
        )
        return 0

    print()
    print(
        "일부 작업 자세에서 soft-safe 후보를 "
        "찾지 못했습니다."
    )
    print(
        "다음 단계에서는 위치 또는 yaw를 "
        "추가 조정해야 합니다."
    )

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
