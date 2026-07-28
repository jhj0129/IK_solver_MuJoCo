#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np
import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

HELPER_PATH = (
    ROOT
    / "src/drok_arm_control/scripts"
    / "dry_run_cylinder_cartesian_ik.py"
)

SPEC = importlib.util.spec_from_file_location(
    "cylinder_ik_helper",
    HELPER_PATH,
)

if SPEC is None or SPEC.loader is None:
    raise RuntimeError(
        f"도우미 스크립트를 불러올 수 없습니다: "
        f"{HELPER_PATH}"
    )

HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--candidate-file",
        default=str(
            ROOT / "cylinder_grasp_candidates.yaml"
        ),
    )

    parser.add_argument(
        "--geometry",
        default=str(
            ROOT
            / "src/drok_arm_kinematics"
            / "config/robot_geometry.yaml"
        ),
    )

    parser.add_argument(
        "--urdf",
        default=str(
            ROOT
            / "src/drok_arm_mujoco"
            / "urdf/drok_arm_mujoco.urdf"
        ),
    )

    parser.add_argument(
        "--joint-state-timeout",
        type=float,
        default=10.0,
    )

    parser.add_argument(
        "--solver-timeout",
        type=float,
        default=15.0,
    )

    parser.add_argument(
        "--output",
        default=str(
            ROOT
            / "cylinder_pregrasp_candidate_scan.yaml"
        ),
    )

    return parser.parse_args()


def maximum_absolute_difference(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    return max(
        abs(a - b)
        for a, b in zip(first, second)
    )


def squared_joint_distance(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    return sum(
        (a - b) ** 2
        for a, b in zip(first, second)
    )


def solution_key(
    result: Dict[str, Any],
) -> tuple[float, float, float]:
    # 우선순위:
    # 1. 현재 자세로부터 최대 관절 변화 최소
    # 2. joint-limit margin 최대
    # 3. 전체 관절 이동량 최소
    return (
        float(result["maximum_change_rad"]),
        -float(result["minimum_limit_margin_rad"]),
        float(result["squared_distance"]),
    )


def append_unique_seed(
    seeds: List[Dict[str, Any]],
    label: str,
    values: Sequence[float],
    tolerance: float = 1.0e-6,
) -> None:
    candidate = [
        float(value)
        for value in values
    ]

    for existing in seeds:
        difference = maximum_absolute_difference(
            candidate,
            existing["values"],
        )

        if difference <= tolerance:
            return

    seeds.append(
        {
            "label": label,
            "values": candidate,
        }
    )


def main() -> int:
    arguments = parse_arguments()

    candidate_path = Path(
        arguments.candidate_file
    ).expanduser().resolve()

    geometry_path = Path(
        arguments.geometry
    ).expanduser().resolve()

    urdf_path = Path(
        arguments.urdf
    ).expanduser().resolve()

    output_path = Path(
        arguments.output
    ).expanduser().resolve()

    with candidate_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        document = yaml.safe_load(stream)

    candidates = document["candidates"]

    limits = HELPER.read_joint_limits(
        urdf_path
    )

    print("=" * 92)
    print("CYLINDER PREGRASP CANDIDATE SCAN")
    print("=" * 92)

    print(
        "현재 /joint_states를 기다립니다."
    )

    current_q = HELPER.read_current_joint_positions(
        arguments.joint_state_timeout
    )

    print(
        "Current q [rad]: "
        + ", ".join(
            f"{value:+.9f}"
            for value in current_q
        )
    )

    successful_candidates: List[
        Dict[str, Any]
    ] = []

    previous_success_q: List[float] | None = None

    for scan_index, candidate in enumerate(
        candidates,
        start=1,
    ):
        candidate_index = int(
            candidate["index"]
        )

        phi_deg = float(
            candidate["phi_deg"]
        )

        position = np.asarray(
            candidate["pickup"][
                "pregrasp_tcp_xyz"
            ],
            dtype=float,
        )

        rotation = np.asarray(
            candidate["rotation_world_tcp"],
            dtype=float,
        )

        target_rpy = HELPER.matrix_to_rpy(
            rotation
        )

        print()
        print(
            f"[{scan_index:02d}/{len(candidates):02d}] "
            f"candidate={candidate_index:02d}, "
            f"phi={phi_deg:7.3f} deg"
        )

        seeds: List[Dict[str, Any]] = []

        append_unique_seed(
            seeds,
            "current_joint_state",
            current_q,
        )

        if previous_success_q is not None:
            append_unique_seed(
                seeds,
                "previous_phi_solution",
                previous_success_q,
            )

        # 먼저 위치만 만족하는 해를 구한다.
        position_result = HELPER.run_ik_solver(
            geometry_path,
            position,
            target_rpy,
            current_q,
            "position",
            arguments.solver_timeout,
        )

        if position_result["success"]:
            append_unique_seed(
                seeds,
                "position_only_solution",
                position_result[
                    "joint_positions"
                ],
            )

        candidate_solutions: List[
            Dict[str, Any]
        ] = []

        for seed_record in seeds:
            full_result = HELPER.run_ik_solver(
                geometry_path,
                position,
                target_rpy,
                seed_record["values"],
                "full",
                arguments.solver_timeout,
            )

            if not full_result["success"]:
                print(
                    f"  FAIL seed="
                    f"{seed_record['label']}"
                )
                continue

            q_solution = [
                float(value)
                for value
                in full_result[
                    "joint_positions"
                ]
            ]

            maximum_change = (
                maximum_absolute_difference(
                    q_solution,
                    current_q,
                )
            )

            squared_distance = (
                squared_joint_distance(
                    q_solution,
                    current_q,
                )
            )

            limit_margin = (
                HELPER.minimum_limit_margin(
                    q_solution,
                    limits,
                )
            )

            if limit_margin <= 0.0:
                print(
                    f"  REJECT seed="
                    f"{seed_record['label']} "
                    f"joint-limit margin <= 0"
                )
                continue

            solution = {
                "seed_source": (
                    seed_record["label"]
                ),
                "joint_positions": q_solution,
                "maximum_change_rad": (
                    maximum_change
                ),
                "maximum_change_deg": (
                    math.degrees(
                        maximum_change
                    )
                ),
                "minimum_limit_margin_rad": (
                    limit_margin
                ),
                "minimum_limit_margin_deg": (
                    math.degrees(
                        limit_margin
                    )
                ),
                "squared_distance": (
                    squared_distance
                ),
                "position_error": (
                    full_result[
                        "position_error"
                    ]
                ),
                "orientation_error": (
                    full_result[
                        "orientation_error"
                    ]
                ),
            }

            candidate_solutions.append(
                solution
            )

            print(
                f"  PASS seed="
                f"{seed_record['label']}, "
                f"max-change="
                f"{solution['maximum_change_deg']:.3f} deg, "
                f"margin="
                f"{solution['minimum_limit_margin_deg']:.3f} deg"
            )

        if not candidate_solutions:
            print("  RESULT: no full-pose solution")
            continue

        candidate_solutions.sort(
            key=solution_key
        )

        best_solution = (
            candidate_solutions[0]
        )

        previous_success_q = list(
            best_solution[
                "joint_positions"
            ]
        )

        successful_record = {
            "candidate_index": candidate_index,
            "phi_deg": phi_deg,
            "pregrasp_tcp_xyz": (
                position.tolist()
            ),
            "target_rpy_rad": (
                target_rpy.tolist()
            ),
            "best_solution": (
                best_solution
            ),
            "all_solutions": (
                candidate_solutions
            ),
        }

        successful_candidates.append(
            successful_record
        )

        print(
            "  RESULT: PASS, selected seed="
            f"{best_solution['seed_source']}"
        )

    successful_candidates.sort(
        key=lambda record: solution_key(
            record["best_solution"]
        )
    )

    best_candidate = (
        successful_candidates[0]
        if successful_candidates
        else None
    )

    output_document = {
        "version": 1,
        "candidate_file": str(
            candidate_path
        ),
        "initial_joint_positions": (
            current_q
        ),
        "tested_candidate_count": (
            len(candidates)
        ),
        "successful_candidate_count": (
            len(successful_candidates)
        ),
        "best_candidate": best_candidate,
        "successful_candidates": (
            successful_candidates
        ),
    }

    output_path.write_text(
        yaml.safe_dump(
            output_document,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    print()
    print("=" * 92)

    if best_candidate is None:
        print("OVERALL RESULT: NO FEASIBLE PREGRASP")
        print(
            "다음 단계는 upright 5-DoF IK "
            "또는 deterministic multi-start IK입니다."
        )
    else:
        best = best_candidate[
            "best_solution"
        ]

        print("OVERALL RESULT: PASS")

        print(
            "Best candidate index : "
            f"{best_candidate['candidate_index']}"
        )

        print(
            "Best phi             : "
            f"{best_candidate['phi_deg']:.3f} deg"
        )

        print(
            "Maximum joint change : "
            f"{best['maximum_change_deg']:.3f} deg"
        )

        print(
            "Minimum limit margin : "
            f"{best['minimum_limit_margin_deg']:.3f} deg"
        )

    print("Saved:")
    print(output_path)
    print("=" * 92)

    return (
        0
        if best_candidate is not None
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
