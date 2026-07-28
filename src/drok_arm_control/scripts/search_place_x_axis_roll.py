#!/usr/bin/env python3

from __future__ import annotations

import concurrent.futures
import math
import os
import re
import subprocess
from pathlib import Path
from typing import List, Optional, Sequence

import yaml

import baseline_nearest_ik_dry_run as base


ROOT = Path.home() / "IK_solver_MuJoCo"

GEOMETRY_PATH = (
    ROOT
    / "src/drok_arm_kinematics/config/robot_geometry.yaml"
)

POSITION_PLAN_PATH = (
    ROOT
    / "baseline_incremental_nearest_ik_plan.yaml"
)

OUTPUT_PATH = (
    ROOT
    / "place_x_axis_roll_search.yaml"
)

ROLL_CANDIDATES_DEG = [
    0.0,
    90.0,
    -90.0,
    180.0,
]

PLACE_Y = -0.180
PLACE_Z = 0.274

# 실제 동작 방향:
# 물체 뒤쪽에서 +X 방향으로 접근
X_VALUES = [
    0.36,
    0.37,
    0.38,
    0.39,
    0.40,
    0.41,
    0.42,
    0.43,
    0.44,
    0.45,
]


def load_reference_q() -> List[float]:
    if not POSITION_PLAN_PATH.exists():
        raise FileNotFoundError(
            f"Position-only plan이 없습니다: "
            f"{POSITION_PLAN_PATH}"
        )

    with POSITION_PLAN_PATH.open(
        "r",
        encoding="utf-8",
    ) as file:
        plan = yaml.safe_load(file)

    for waypoint in plan["waypoints"]:
        if waypoint["name"] == "PLACE_ABOVE":
            return [
                float(value)
                for value
                in waypoint["selected_q"]
            ]

    raise RuntimeError(
        "Position-only plan에서 "
        "PLACE_ABOVE를 찾지 못했습니다."
    )


def make_local_seeds(
    reference_q: Sequence[float],
    limits,
    broad: bool,
) -> List[List[float]]:
    seeds: List[List[float]] = []

    def append_unique(
        values: Sequence[float],
    ) -> None:
        clipped = base.clip_to_limits(
            values,
            limits,
        )

        for existing in seeds:
            if max(
                abs(a - b)
                for a, b in zip(
                    existing,
                    clipped,
                )
            ) < 1.0e-8:
                return

        seeds.append(clipped)

    # 가장 중요한 seed: 바로 이전 관절각
    append_unique(reference_q)

    for joint_index in range(6):
        for offset_deg in [-10.0, 10.0]:
            candidate = list(reference_q)
            candidate[joint_index] += math.radians(
                offset_deg
            )
            append_unique(candidate)

    for joint_index in [3, 4, 5]:
        for offset_deg in [-30.0, 30.0]:
            candidate = list(reference_q)
            candidate[joint_index] += math.radians(
                offset_deg
            )
            append_unique(candidate)

    # 첫 정렬점에서는 넓은 범위도 탐색
    if broad:
        append_unique([0.0] * 6)

        primes = [2, 3, 5, 7, 11, 13]

        for sample_index in range(1, 13):
            seed = []

            for joint_index, joint_name in enumerate(
                base.JOINT_NAMES
            ):
                lower, upper = limits[joint_name]

                ratio = base.halton_value(
                    sample_index,
                    primes[joint_index],
                )

                seed.append(
                    lower
                    + ratio * (upper - lower)
                )

            append_unique(seed)

    return seeds


def solve_one(
    position: Sequence[float],
    rpy: Sequence[float],
    seed: Sequence[float],
) -> Optional[List[float]]:
    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "solve_ik_pose",
        str(GEOMETRY_PATH),
        f"{position[0]:.12f}",
        f"{position[1]:.12f}",
        f"{position[2]:.12f}",
        f"{rpy[0]:.12f}",
        f"{rpy[1]:.12f}",
        f"{rpy[2]:.12f}",
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

    if (
        result.returncode != 0
        or success_match is None
        or success_match.group(1).lower()
        != "true"
        or joint_match is None
    ):
        return None

    try:
        q = [
            float(value)
            for value
            in joint_match.group(1).split(",")
        ]
    except ValueError:
        return None

    return q if len(q) == 6 else None


def solve_candidates(
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
    broad: bool,
) -> List[List[float]]:
    seeds = make_local_seeds(
        reference_q,
        limits,
        broad,
    )

    raw_results: List[
        Optional[List[float]]
    ] = []

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=4
    ) as executor:
        futures = [
            executor.submit(
                solve_one,
                position,
                rpy,
                seed,
            )
            for seed in seeds
        ]

        for future in concurrent.futures.as_completed(
            futures
        ):
            try:
                raw_results.append(
                    future.result()
                )
            except Exception:
                raw_results.append(None)

    candidates: List[List[float]] = []

    for raw_q in raw_results:
        if raw_q is None:
            continue

        normalized = base.normalize_candidate(
            raw_q,
            reference_q,
            limits,
        )

        if normalized is None:
            continue

        if base.is_duplicate(
            candidates,
            normalized,
        ):
            continue

        candidates.append(normalized)

    candidates.sort(
        key=lambda candidate:
        base.candidate_score(
            candidate,
            reference_q,
        )
    )

    return candidates


def format_degrees(
    values: Sequence[float],
) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+8.2f}°"
            for value in values
        )
        + "]"
    )


def evaluate_roll(
    roll_deg: float,
    initial_reference_q: Sequence[float],
    limits,
) -> dict:
    rpy = [
        math.radians(roll_deg),
        0.0,
        0.0,
    ]

    reference_q = list(
        initial_reference_q
    )

    solved_waypoints = []
    failed_x = None

    alignment_step = 0.0
    largest_approach_step = 0.0
    total_travel = 0.0

    print()
    print("=" * 104)
    print(
        f"PLACE ROLL CANDIDATE: "
        f"{roll_deg:+.1f}°"
    )
    print("=" * 104)

    for index, x in enumerate(
        X_VALUES
    ):
        name = (
            "PLACE_PREPLACE"
            if index == 0
            else (
                "PLACE_RELEASE"
                if index == len(X_VALUES) - 1
                else f"PLACE_APPROACH_{index:02d}"
            )
        )

        position = [
            x,
            PLACE_Y,
            PLACE_Z,
        ]

        candidates = solve_candidates(
            position=position,
            rpy=rpy,
            reference_q=reference_q,
            limits=limits,
            broad=(index == 0),
        )

        print()
        print(
            f"[{index + 1:02d}/"
            f"{len(X_VALUES):02d}] "
            f"{name}"
        )

        print(
            "  xyz       :",
            f"[{x:+.3f}, "
            f"{PLACE_Y:+.3f}, "
            f"{PLACE_Z:+.3f}]",
        )

        print(
            "  candidates:",
            len(candidates),
        )

        if not candidates:
            failed_x = x
            print("  RESULT    : IK FAIL")
            break

        selected_q = candidates[0]

        delta = [
            selected - previous
            for selected, previous
            in zip(selected_q, reference_q)
        ]

        maximum_delta = max(
            abs(value)
            for value in delta
        )

        sum_delta = sum(
            abs(value)
            for value in delta
        )

        if index == 0:
            alignment_step = maximum_delta
        else:
            largest_approach_step = max(
                largest_approach_step,
                maximum_delta,
            )

        total_travel += sum_delta

        print(
            "  selected  :",
            format_degrees(selected_q),
        )

        print(
            "  delta     :",
            format_degrees(delta),
        )

        print(
            "  max Δ     :",
            f"{math.degrees(maximum_delta):.3f}°",
        )

        solved_waypoints.append(
            {
                "name": name,
                "position": [
                    float(value)
                    for value in position
                ],
                "rpy": [
                    float(value)
                    for value in rpy
                ],
                "candidate_count": len(
                    candidates
                ),
                "selected_q": [
                    float(value)
                    for value in selected_q
                ],
                "maximum_delta_rad": float(
                    maximum_delta
                ),
                "maximum_delta_deg": float(
                    math.degrees(maximum_delta)
                ),
            }
        )

        reference_q = selected_q

    success = (
        failed_x is None
        and len(solved_waypoints)
        == len(X_VALUES)
        and largest_approach_step
        <= math.radians(20.0)
    )

    print()
    print(
        "RESULT:",
        "PASS" if success else "FAIL",
    )

    print(
        "Initial alignment step:",
        f"{math.degrees(alignment_step):.3f}°",
    )

    print(
        "Largest approach step:",
        f"{math.degrees(largest_approach_step):.3f}°",
    )

    return {
        "roll_deg": float(roll_deg),
        "success": bool(success),
        "failed_x": (
            None
            if failed_x is None
            else float(failed_x)
        ),
        "alignment_step_rad": float(
            alignment_step
        ),
        "alignment_step_deg": float(
            math.degrees(alignment_step)
        ),
        "largest_approach_step_rad": float(
            largest_approach_step
        ),
        "largest_approach_step_deg": float(
            math.degrees(
                largest_approach_step
            )
        ),
        "total_joint_travel_rad": float(
            total_travel
        ),
        "waypoints": solved_waypoints,
    }


def ranking_key(result: dict):
    return (
        0 if result["success"] else 1,
        float(
            result[
                "largest_approach_step_rad"
            ]
        ),
        float(result["alignment_step_rad"]),
        float(
            result["total_joint_travel_rad"]
        ),
    )


def main() -> int:
    limits = base.load_joint_limits(
        base.URDF_PATH
    )

    reference_q = load_reference_q()

    print("=" * 104)
    print("DROK ARM PLACE X-AXIS ROLL SEARCH")
    print("=" * 104)

    print("Robot execution : 비활성화")
    print("IK mode         : full")
    print("Approach        : world +X")
    print("Pitch/Yaw       : 0° / 0°")
    print(
        "Roll candidates :",
        ROLL_CANDIDATES_DEG,
    )

    print(
        "Initial reference:",
        format_degrees(reference_q),
    )

    results = [
        evaluate_roll(
            roll_deg,
            reference_q,
            limits,
        )
        for roll_deg
        in ROLL_CANDIDATES_DEG
    ]

    results.sort(key=ranking_key)

    print()
    print("=" * 104)
    print("FINAL RANKING")
    print("=" * 104)

    for rank, result in enumerate(
        results,
        start=1,
    ):
        failed_text = (
            "-"
            if result["failed_x"] is None
            else f"{result['failed_x']:.3f}"
        )

        print(
            f"{rank}. "
            f"roll={result['roll_deg']:+7.1f}° "
            f"result="
            f"{'PASS' if result['success'] else 'FAIL'} "
            f"align="
            f"{result['alignment_step_deg']:.3f}° "
            f"approach_max="
            f"{result['largest_approach_step_deg']:.3f}° "
            f"failed_x={failed_text}"
        )

    successful = [
        result
        for result in results
        if result["success"]
    ]

    output = {
        "mode": "place_x_axis_roll_search",
        "robot_execution": False,
        "approach_axis": "world_positive_x",
        "place_y": PLACE_Y,
        "place_z": PLACE_Z,
        "x_values": X_VALUES,
        "roll_candidates_deg": (
            ROLL_CANDIDATES_DEG
        ),
        "initial_reference_q": [
            float(value)
            for value in reference_q
        ],
        "results": results,
        "best_result": (
            successful[0]
            if successful
            else None
        ),
    }

    with OUTPUT_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        yaml.safe_dump(
            output,
            file,
            sort_keys=False,
            allow_unicode=True,
        )

    print()
    print("Saved:")
    print(OUTPUT_PATH)

    if not successful:
        print()
        print(
            "OVERALL FAIL — 네 방향 모두 "
            "PLACE X축 접근 실패"
        )
        return 1

    best = successful[0]

    print()
    print(
        "BEST PLACE ROLL:",
        f"{best['roll_deg']:+.1f}°",
    )

    print(
        "BEST APPROACH MAX Δ:",
        f"{best['largest_approach_step_deg']:.3f}°",
    )

    print()
    print(
        "OVERALL PASS — 선택된 면 정렬 자세로 "
        "PLACE 접근과 동일 경로 후퇴가 가능합니다."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
