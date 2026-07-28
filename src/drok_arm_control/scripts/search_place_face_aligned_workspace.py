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
    / "place_face_aligned_workspace_search.yaml"
)

PLACE_CENTER_X = 0.450
PLACE_Y = -0.180
PLACE_Z = 0.274

CUBE_HALF_SIZE = 0.040

# 물체 중심에서 TCP가 최소 80 mm 떨어지는 경로를 목표로 한다.
# 80 mm 정육면체라면 가까운 면으로부터 약 40 mm 바깥이다.
TARGET_RETREAT_M = 0.080
MAX_RETREAT_M = 0.120
STEP_M = 0.005

ROLL_CANDIDATES_DEG = [
    0.0,
    90.0,
    -90.0,
    180.0,
]

# 기존 roll=0, yaw=0의 성공한 PLACE_RELEASE 해
KNOWN_PLACE_RELEASE_Q = [
    -0.664027967214,
    1.546815502872,
    0.804858584557,
    0.712251414446,
    -0.951431335140,
    0.560006343795,
]


def load_position_reference() -> List[float]:
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
                for value in waypoint["selected_q"]
            ]

    raise RuntimeError(
        "PLACE_ABOVE를 찾지 못했습니다."
    )


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
        or success_match.group(1).lower() != "true"
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


def append_unique_seed(
    seeds: List[List[float]],
    values: Sequence[float],
    limits,
) -> None:
    clipped = base.clip_to_limits(
        values,
        limits,
    )

    for existing in seeds:
        if max(
            abs(a - b)
            for a, b in zip(existing, clipped)
        ) < 1.0e-8:
            return

    seeds.append(clipped)


def make_broad_seeds(
    reference_q: Sequence[float],
    limits,
) -> List[List[float]]:
    seeds: List[List[float]] = []

    append_unique_seed(
        seeds,
        reference_q,
        limits,
    )

    append_unique_seed(
        seeds,
        KNOWN_PLACE_RELEASE_Q,
        limits,
    )

    append_unique_seed(
        seeds,
        [0.0] * 6,
        limits,
    )

    for center in [
        reference_q,
        KNOWN_PLACE_RELEASE_Q,
    ]:
        for joint_index in range(6):
            for offset_deg in [-15.0, 15.0]:
                seed = list(center)
                seed[joint_index] += math.radians(
                    offset_deg
                )

                append_unique_seed(
                    seeds,
                    seed,
                    limits,
                )

    primes = [
        2,
        3,
        5,
        7,
        11,
        13,
    ]

    for sample_index in range(1, 25):
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

        append_unique_seed(
            seeds,
            seed,
            limits,
        )

    return seeds


def make_local_seeds(
    reference_q: Sequence[float],
    limits,
) -> List[List[float]]:
    seeds: List[List[float]] = []

    append_unique_seed(
        seeds,
        reference_q,
        limits,
    )

    for joint_index in range(6):
        for offset_deg in [-5.0, 5.0]:
            seed = list(reference_q)
            seed[joint_index] += math.radians(
                offset_deg
            )

            append_unique_seed(
                seeds,
                seed,
                limits,
            )

    for joint_index in [3, 4, 5]:
        for offset_deg in [-15.0, 15.0]:
            seed = list(reference_q)
            seed[joint_index] += math.radians(
                offset_deg
            )

            append_unique_seed(
                seeds,
                seed,
                limits,
            )

    return seeds


def solve_from_seeds(
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    seeds: Sequence[Sequence[float]],
    limits,
) -> List[List[float]]:
    raw_results: List[Optional[List[float]]] = []

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


def solve_release(
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
) -> List[List[float]]:
    position = [
        PLACE_CENTER_X,
        PLACE_Y,
        PLACE_Z,
    ]

    return solve_from_seeds(
        position=position,
        rpy=rpy,
        reference_q=reference_q,
        seeds=make_broad_seeds(
            reference_q,
            limits,
        ),
        limits=limits,
    )


def solve_next_continuous(
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
) -> Optional[List[float]]:
    # 먼저 직전 해 하나만 seed로 시도한다.
    direct = solve_one(
        position,
        rpy,
        reference_q,
    )

    if direct is not None:
        normalized = base.normalize_candidate(
            direct,
            reference_q,
            limits,
        )

        if normalized is not None:
            return normalized

    # 직전 해로 실패했을 때만 주변 seed를 탐색한다.
    candidates = solve_from_seeds(
        position=position,
        rpy=rpy,
        reference_q=reference_q,
        seeds=make_local_seeds(
            reference_q,
            limits,
        ),
        limits=limits,
    )

    return candidates[0] if candidates else None


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


def evaluate_candidate(
    side_name: str,
    retreat_sign: float,
    yaw_deg: float,
    roll_deg: float,
    initial_reference_q: Sequence[float],
    limits,
) -> dict:
    rpy = [
        math.radians(roll_deg),
        0.0,
        math.radians(yaw_deg),
    ]

    print()
    print("=" * 110)

    print(
        f"SIDE={side_name}  "
        f"ROLL={roll_deg:+.1f}°  "
        f"YAW={yaw_deg:+.1f}°"
    )

    print("=" * 110)

    release_candidates = solve_release(
        rpy,
        initial_reference_q,
        limits,
    )

    print(
        "PLACE_RELEASE candidates:",
        len(release_candidates),
    )

    if not release_candidates:
        print("PLACE_RELEASE IK FAIL")

        return {
            "side": side_name,
            "roll_deg": float(roll_deg),
            "yaw_deg": float(yaw_deg),
            "success": False,
            "release_success": False,
            "target_retreat_reached": False,
            "last_success_x": None,
            "retreat_distance_m": 0.0,
            "clearance_beyond_cube_face_m": 0.0,
            "largest_step_deg": 0.0,
            "failed_x": PLACE_CENTER_X,
            "waypoints": [],
        }

    release_q = release_candidates[0]

    alignment_delta = max(
        abs(a - b)
        for a, b in zip(
            release_q,
            initial_reference_q,
        )
    )

    print(
        "PLACE_RELEASE q:",
        format_degrees(release_q),
    )

    print(
        "Initial alignment max Δ:",
        f"{math.degrees(alignment_delta):.3f}°",
    )

    waypoints = [
        {
            "name": "PLACE_RELEASE",
            "position": [
                PLACE_CENTER_X,
                PLACE_Y,
                PLACE_Z,
            ],
            "rpy": [
                float(value)
                for value in rpy
            ],
            "selected_q": [
                float(value)
                for value in release_q
            ],
            "maximum_delta_deg": float(
                math.degrees(alignment_delta)
            ),
        }
    ]

    reference_q = release_q
    last_success_x = PLACE_CENTER_X
    failed_x = None
    largest_step = 0.0

    step_count = int(
        round(MAX_RETREAT_M / STEP_M)
    )

    for index in range(
        1,
        step_count + 1,
    ):
        retreat_distance = index * STEP_M

        x = (
            PLACE_CENTER_X
            + retreat_sign * retreat_distance
        )

        position = [
            x,
            PLACE_Y,
            PLACE_Z,
        ]

        solved_q = solve_next_continuous(
            position,
            rpy,
            reference_q,
            limits,
        )

        print(
            f"[{index:02d}/{step_count:02d}] "
            f"x={x:+.3f}  "
            f"retreat={retreat_distance * 1000:.0f} mm",
            end="",
        )

        if solved_q is None:
            failed_x = x
            print("  IK FAIL")
            break

        delta = [
            solved - previous
            for solved, previous
            in zip(solved_q, reference_q)
        ]

        maximum_delta = max(
            abs(value)
            for value in delta
        )

        largest_step = max(
            largest_step,
            maximum_delta,
        )

        print(
            f"  maxΔ="
            f"{math.degrees(maximum_delta):.3f}°"
        )

        waypoints.append(
            {
                "name": (
                    f"RETREAT_{index:02d}"
                ),
                "position": [
                    float(value)
                    for value in position
                ],
                "rpy": [
                    float(value)
                    for value in rpy
                ],
                "selected_q": [
                    float(value)
                    for value in solved_q
                ],
                "maximum_delta_deg": float(
                    math.degrees(maximum_delta)
                ),
            }
        )

        reference_q = solved_q
        last_success_x = x

    retreat_distance_m = abs(
        last_success_x - PLACE_CENTER_X
    )

    clearance_beyond_face_m = max(
        0.0,
        retreat_distance_m - CUBE_HALF_SIZE,
    )

    target_reached = (
        retreat_distance_m
        >= TARGET_RETREAT_M - 1.0e-9
    )

    continuous = (
        largest_step
        <= math.radians(20.0)
    )

    success = (
        target_reached
        and continuous
    )

    print()
    print(
        "Last success x:",
        f"{last_success_x:.3f} m",
    )

    print(
        "Maximum retreat:",
        f"{retreat_distance_m * 1000:.1f} mm",
    )

    print(
        "TCP clearance beyond cube face:",
        f"{clearance_beyond_face_m * 1000:.1f} mm",
    )

    print(
        "Largest path step:",
        f"{math.degrees(largest_step):.3f}°",
    )

    print(
        "RESULT:",
        "PASS" if success else "FAIL",
    )

    return {
        "side": side_name,
        "roll_deg": float(roll_deg),
        "yaw_deg": float(yaw_deg),
        "success": bool(success),
        "release_success": True,
        "target_retreat_reached": bool(
            target_reached
        ),
        "last_success_x": float(
            last_success_x
        ),
        "retreat_distance_m": float(
            retreat_distance_m
        ),
        "clearance_beyond_cube_face_m": float(
            clearance_beyond_face_m
        ),
        "largest_step_deg": float(
            math.degrees(largest_step)
        ),
        "alignment_step_deg": float(
            math.degrees(alignment_delta)
        ),
        "failed_x": (
            None
            if failed_x is None
            else float(failed_x)
        ),
        "waypoints": waypoints,
    }


def ranking_key(result: dict):
    return (
        0 if result["success"] else 1,
        -float(result["retreat_distance_m"]),
        float(result["largest_step_deg"]),
        float(
            result.get(
                "alignment_step_deg",
                9999.0,
            )
        ),
    )


def main() -> int:
    limits = base.load_joint_limits(
        base.URDF_PATH
    )

    initial_reference_q = (
        load_position_reference()
    )

    print("=" * 110)
    print(
        "DROK ARM PLACE FACE-ALIGNED "
        "WORKSPACE SEARCH"
    )
    print("=" * 110)

    print("Robot execution : 비활성화")
    print("IK mode         : full")
    print(
        "Target retreat  :",
        f"{TARGET_RETREAT_M * 1000:.0f} mm",
    )

    print(
        "Maximum search  :",
        f"{MAX_RETREAT_M * 1000:.0f} mm",
    )

    print(
        "Step            :",
        f"{STEP_M * 1000:.0f} mm",
    )

    print(
        "Initial reference:",
        format_degrees(initial_reference_q),
    )

    results = []

    # 물체의 -X 쪽에서 +X 방향으로 접근
    for roll_deg in ROLL_CANDIDATES_DEG:
        results.append(
            evaluate_candidate(
                side_name="NEGATIVE_X_SIDE",
                retreat_sign=-1.0,
                yaw_deg=0.0,
                roll_deg=roll_deg,
                initial_reference_q=(
                    initial_reference_q
                ),
                limits=limits,
            )
        )

    # 물체의 +X 쪽에서 -X 방향으로 접근
    for roll_deg in ROLL_CANDIDATES_DEG:
        results.append(
            evaluate_candidate(
                side_name="POSITIVE_X_SIDE",
                retreat_sign=1.0,
                yaw_deg=180.0,
                roll_deg=roll_deg,
                initial_reference_q=(
                    initial_reference_q
                ),
                limits=limits,
            )
        )

    results.sort(key=ranking_key)

    print()
    print("=" * 110)
    print("FINAL RANKING")
    print("=" * 110)

    for rank, result in enumerate(
        results,
        start=1,
    ):
        print(
            f"{rank}. "
            f"side={result['side']:17s} "
            f"roll={result['roll_deg']:+7.1f}° "
            f"yaw={result['yaw_deg']:+7.1f}° "
            f"result="
            f"{'PASS' if result['success'] else 'FAIL'} "
            f"retreat="
            f"{result['retreat_distance_m'] * 1000:6.1f} mm "
            f"clearance="
            f"{result['clearance_beyond_cube_face_m'] * 1000:6.1f} mm "
            f"path_max="
            f"{result['largest_step_deg']:7.3f}°"
        )

    successful = [
        result
        for result in results
        if result["success"]
    ]

    output = {
        "mode": (
            "place_face_aligned_workspace_search"
        ),
        "robot_execution": False,
        "place_center": [
            PLACE_CENTER_X,
            PLACE_Y,
            PLACE_Z,
        ],
        "cube_half_size_m": CUBE_HALF_SIZE,
        "target_retreat_m": TARGET_RETREAT_M,
        "maximum_retreat_m": MAX_RETREAT_M,
        "step_m": STEP_M,
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
            "OVERALL FAIL — 목표한 80 mm 후퇴가 "
            "가능한 면 정렬 경로를 찾지 못했습니다."
        )
        return 1

    best = successful[0]

    print()
    print(
        "BEST SIDE:",
        best["side"],
    )

    print(
        "BEST ROLL:",
        f"{best['roll_deg']:+.1f}°",
    )

    print(
        "BEST YAW:",
        f"{best['yaw_deg']:+.1f}°",
    )

    print(
        "BEST RETREAT:",
        f"{best['retreat_distance_m'] * 1000:.1f} mm",
    )

    print()
    print(
        "OVERALL PASS — 이 경로를 역순으로 사용하면 "
        "면 정렬을 유지한 PLACE 접근이 가능합니다."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
