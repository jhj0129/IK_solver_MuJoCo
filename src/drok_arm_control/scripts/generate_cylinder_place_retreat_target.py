#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--hybrid-path",
        default=str(
            ROOT / "cylinder_best_hybrid_path.yaml"
        ),
    )

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
            / "src/drok_arm_kinematics/config"
            / "robot_geometry.yaml"
        ),
    )

    parser.add_argument(
        "--output",
        default=str(
            ROOT
            / "cylinder_best_place_retreat_target.yaml"
        ),
    )

    return parser.parse_args()


def load_yaml(
    path: Path,
) -> dict[str, Any]:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict):
        raise RuntimeError(
            f"잘못된 YAML 문서입니다: {path}"
        )

    return document


def main() -> int:
    arguments = parse_arguments()

    hybrid_path = Path(
        arguments.hybrid_path
    ).expanduser().resolve()

    candidate_path = Path(
        arguments.candidate_file
    ).expanduser().resolve()

    geometry_path = Path(
        arguments.geometry
    ).expanduser().resolve()

    output_path = Path(
        arguments.output
    ).expanduser().resolve()

    hybrid = load_yaml(
        hybrid_path
    )

    candidates = load_yaml(
        candidate_path
    )

    place_records = [
        record
        for record in hybrid["path"]
        if record["segment"] == "PLACE_DESCEND"
    ]

    if not place_records:
        raise RuntimeError(
            "PLACE_DESCEND record가 없습니다."
        )

    final_record = place_records[-1]

    final_position = [
        float(value)
        for value in final_record["position"]
    ]

    final_seed_q = [
        float(value)
        for value in final_record[
            "joint_positions"
        ]
    ]

    candidate_index = int(
        hybrid["candidate_index"]
    )

    pregrasp_distance = float(
        candidates["derived_geometry"][
            "pregrasp_distance"
        ]
    )

    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "solve_ik_upright",
        str(geometry_path),
        *(
            f"{value:.12f}"
            for value in final_position
        ),
        *(
            f"{value:.12f}"
            for value in final_seed_q
        ),
    ]

    environment = os.environ.copy()

    environment.update(
        {
            "DROK_UPRIGHT_POSITION_TOLERANCE":
                "0.000050",

            "DROK_UPRIGHT_ANGLE_TOLERANCE":
                "0.000050",

            "DROK_UPRIGHT_SEED_GAIN":
                "0.08",

            "DROK_UPRIGHT_LIMIT_SOFT_MARGIN":
                "0.261799388",

            "DROK_UPRIGHT_LIMIT_BARRIER_GAIN":
                "0.02",

            "DROK_UPRIGHT_MAX_SECONDARY_STEP":
                "0.002",
        }
    )

    completed = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=environment,
        check=False,
    )

    print(completed.stdout)

    if completed.returncode != 0:
        raise RuntimeError(
            "PLACE 최종 upright 자세 계산 실패"
        )

    yaw_match = re.search(
        r"Solved TCP yaw\s*:\s*"
        r"([0-9eE+\-.]+)",
        completed.stdout,
    )

    joint_match = re.search(
        r"JOINT_RESULT=([^\n\r]+)",
        completed.stdout,
    )

    if yaw_match is None:
        raise RuntimeError(
            "Solved TCP yaw를 찾지 못했습니다."
        )

    if joint_match is None:
        raise RuntimeError(
            "JOINT_RESULT를 찾지 못했습니다."
        )

    final_yaw = float(
        yaw_match.group(1)
    )

    refined_q = [
        float(value)
        for value
        in joint_match.group(1).split(",")
    ]

    # TCP local +X가 원기둥을 향하는 접근축이다.
    approach_direction = [
        math.cos(final_yaw),
        math.sin(final_yaw),
        0.0,
    ]

    retreat_direction = [
        -approach_direction[0],
        -approach_direction[1],
        0.0,
    ]

    retreat_position = [
        final_position[index]
        + pregrasp_distance
        * retreat_direction[index]
        for index in range(3)
    ]

    output = {
        "version": 1,
        "source_hybrid_path": str(
            hybrid_path
        ),
        "candidate_index": candidate_index,
        "place_grasp_tcp_xyz": (
            final_position
        ),
        "place_grasp_joint_positions": (
            refined_q
        ),
        "final_tcp_yaw_rad": final_yaw,
        "final_tcp_yaw_deg": (
            math.degrees(final_yaw)
        ),
        "approach_direction_world": (
            approach_direction
        ),
        "retreat_direction_world": (
            retreat_direction
        ),
        "retreat_distance": (
            pregrasp_distance
        ),
        "place_retreat_tcp_xyz": (
            retreat_position
        ),
        "retreat_target_rpy_rad": [
            0.0,
            0.0,
            final_yaw,
        ],
    }

    output_path.write_text(
        yaml.safe_dump(
            output,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    print("=" * 88)
    print("NEW HEIGHT PLACE RETREAT TARGET")
    print("=" * 88)

    print(
        "Candidate index : "
        f"{candidate_index}"
    )

    print(
        "Final yaw       : "
        f"{math.degrees(final_yaw):+.6f} deg"
    )

    print(
        "Place grasp TCP : "
        + ", ".join(
            f"{value:+.9f}"
            for value in final_position
        )
    )

    print(
        "Retreat TCP     : "
        + ", ".join(
            f"{value:+.9f}"
            for value in retreat_position
        )
    )

    print(
        "Retreat vector  : "
        + ", ".join(
            f"{value:+.9f}"
            for value in retreat_direction
        )
    )

    print("Saved:")
    print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
