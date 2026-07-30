#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, List

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
        f"IK helper를 읽지 못했습니다: {HELPER_PATH}"
    )

HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--target",
        default=str(
            ROOT
            / "cylinder_place_retreat_target.yaml"
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
        "--cartesian-step",
        type=float,
        default=0.003,
    )

    parser.add_argument(
        "--maximum-joint-jump-deg",
        type=float,
        default=5.0,
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
            / "cylinder_place_retreat_full_path.yaml"
        ),
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    target_path = Path(
        arguments.target
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

    if arguments.cartesian_step <= 0.0:
        raise RuntimeError(
            "cartesian-step은 양수여야 합니다."
        )

    with target_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        target = yaml.safe_load(stream)

    start_position = np.asarray(
        target["place_grasp_tcp_xyz"],
        dtype=float,
    )

    goal_position = np.asarray(
        target["place_retreat_tcp_xyz"],
        dtype=float,
    )

    target_rpy = np.asarray(
        target["retreat_target_rpy_rad"],
        dtype=float,
    )

    seed_q = [
        float(value)
        for value in target[
            "place_grasp_joint_positions"
        ]
    ]

    expected_retreat_direction = np.asarray(
        target["retreat_direction_world"],
        dtype=float,
    )

    displacement = (
        goal_position - start_position
    )

    distance = float(
        np.linalg.norm(displacement)
    )

    if distance <= 1.0e-12:
        raise RuntimeError(
            "후퇴 거리가 0입니다."
        )

    actual_retreat_direction = (
        displacement / distance
    )

    direction_error = float(
        np.linalg.norm(
            actual_retreat_direction
            - expected_retreat_direction
        )
    )

    if direction_error > 1.0e-8:
        raise RuntimeError(
            "계산된 후퇴 방향과 목표 방향이 "
            f"일치하지 않습니다: {direction_error}"
        )

    limits = HELPER.read_joint_limits(
        urdf_path
    )

    maximum_jump_allowed = math.radians(
        arguments.maximum_joint_jump_deg
    )

    print("=" * 88)
    print("CYLINDER PLACE RETREAT FULL-POSE IK")
    print("=" * 88)

    print(
        "Start TCP : "
        + ", ".join(
            f"{value:+.9f}"
            for value in start_position
        )
    )

    print(
        "Goal TCP  : "
        + ", ".join(
            f"{value:+.9f}"
            for value in goal_position
        )
    )

    print(
        "Retreat direction: "
        + ", ".join(
            f"{value:+.9f}"
            for value in actual_retreat_direction
        )
    )

    print(
        f"Distance  : {distance:.9f} m"
    )

    print(
        "Fixed RPY : "
        + ", ".join(
            f"{value:+.9f}"
            for value in target_rpy
        )
    )

    print()
    print(
        "[ALIGN] PLACE_GRASP에서 최종 yaw를 "
        "full-pose로 정확히 고정합니다."
    )

    alignment_result = HELPER.run_ik_solver(
        geometry_path,
        start_position,
        target_rpy,
        seed_q,
        "full",
        arguments.solver_timeout,
    )

    if not alignment_result["success"]:
        print("[FAIL] PLACE_GRASP full-pose alignment")
        print(alignment_result["output"])
        return 2

    q_previous = [
        float(value)
        for value in alignment_result[
            "joint_positions"
        ]
    ]

    alignment_jump = (
        HELPER.maximum_joint_difference(
            q_previous,
            seed_q,
        )
    )

    if alignment_jump > maximum_jump_allowed:
        print(
            "[FAIL] Full-pose alignment에서 "
            "관절 branch jump 발생"
        )

        print(
            f"Jump={math.degrees(alignment_jump):.6f} deg"
        )

        return 3

    print(
        "[PASS] alignment max jump="
        f"{math.degrees(alignment_jump):.6f} deg"
    )

    positions = HELPER.interpolate_segment(
        start_position,
        goal_position,
        arguments.cartesian_step,
    )

    records: List[Dict[str, Any]] = [
        {
            "sample_index": 0,
            "position": start_position.tolist(),
            "joint_positions": q_previous,
            "maximum_joint_jump_rad": (
                alignment_jump
            ),
        }
    ]

    global_maximum_jump = alignment_jump

    global_minimum_margin = (
        HELPER.minimum_limit_margin(
            q_previous,
            limits,
        )
    )

    print()
    print(
        f"[PLACE_RETREAT] samples={len(positions)}, "
        f"step<={arguments.cartesian_step:.6f} m"
    )

    for sample_index, position in enumerate(
        positions,
        start=1,
    ):
        result = HELPER.run_ik_solver(
            geometry_path,
            position,
            target_rpy,
            q_previous,
            "full",
            arguments.solver_timeout,
        )

        if not result["success"]:
            print(
                f"[FAIL] sample={sample_index}/"
                f"{len(positions)}"
            )

            print(
                "Position = "
                + ", ".join(
                    f"{value:+.9f}"
                    for value in position
                )
            )

            print(result["output"])
            return 4

        q_current = [
            float(value)
            for value in result[
                "joint_positions"
            ]
        ]

        jump = HELPER.maximum_joint_difference(
            q_current,
            q_previous,
        )

        margin = HELPER.minimum_limit_margin(
            q_current,
            limits,
        )

        if margin <= 0.0:
            print(
                f"[FAIL] joint-limit violation "
                f"sample={sample_index}"
            )
            return 5

        if jump > maximum_jump_allowed:
            print(
                f"[FAIL] branch jump "
                f"sample={sample_index}"
            )

            print(
                f"Jump={math.degrees(jump):.6f} deg"
            )

            return 6

        global_maximum_jump = max(
            global_maximum_jump,
            jump,
        )

        global_minimum_margin = min(
            global_minimum_margin,
            margin,
        )

        records.append(
            {
                "sample_index": sample_index,
                "position": position.tolist(),
                "joint_positions": q_current,
                "position_error": result[
                    "position_error"
                ],
                "orientation_error": result[
                    "orientation_error"
                ],
                "maximum_joint_jump_rad": jump,
                "minimum_limit_margin_rad": margin,
            }
        )

        q_previous = q_current

    output_document = {
        "version": 1,
        "segment": "PLACE_RETREAT",
        "ik_mode": "full",
        "start_tcp_xyz": (
            start_position.tolist()
        ),
        "goal_tcp_xyz": (
            goal_position.tolist()
        ),
        "retreat_direction_world": (
            actual_retreat_direction.tolist()
        ),
        "distance": distance,
        "target_rpy_rad": target_rpy.tolist(),
        "sample_count": len(records),
        "maximum_joint_jump_rad": (
            global_maximum_jump
        ),
        "maximum_joint_jump_deg": (
            math.degrees(
                global_maximum_jump
            )
        ),
        "minimum_joint_limit_margin_rad": (
            global_minimum_margin
        ),
        "minimum_joint_limit_margin_deg": (
            math.degrees(
                global_minimum_margin
            )
        ),
        "final_joint_positions": (
            q_previous
        ),
        "path": records,
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
    print("=" * 88)
    print("OVERALL RESULT: PASS")
    print("=" * 88)

    print(
        f"Samples             : {len(records)}"
    )

    print(
        "Maximum joint jump  : "
        f"{math.degrees(global_maximum_jump):.6f} deg"
    )

    print(
        "Minimum limit margin: "
        f"{math.degrees(global_minimum_margin):.6f} deg"
    )

    print("Saved:")
    print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
