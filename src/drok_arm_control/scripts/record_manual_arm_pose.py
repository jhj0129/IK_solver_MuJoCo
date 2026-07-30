#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml

from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path.home() / "IK_solver_MuJoCo"

GEOMETRY_PATH = (
    ROOT
    / "src/drok_arm_kinematics/config/robot_geometry.yaml"
)

OUTPUT_PATH = (
    ROOT
    / "manual_arm_pose_samples.yaml"
)

JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]


class JointStateReader(Node):
    def __init__(self) -> None:
        super().__init__("manual_arm_pose_recorder")

        self.positions: Dict[str, float] = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            20,
        )

    def callback(self, message: JointState) -> None:
        for name, position in zip(
            message.name,
            message.position,
        ):
            if name in JOINT_NAMES:
                self.positions[name] = float(position)

    def ready(self) -> bool:
        return all(
            joint in self.positions
            for joint in JOINT_NAMES
        )

    def current_q(self) -> List[float]:
        return [
            self.positions[name]
            for name in JOINT_NAMES
        ]


def format_rad(values: List[float]) -> str:
    return (
        "["
        + ", ".join(
            f"{value:+.12f}"
            for value in values
        )
        + "]"
    )


def format_deg(values: List[float]) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+9.3f}°"
            for value in values
        )
        + "]"
    )


def run_fk(q: List[float]) -> str:
    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "test_fk",
        str(GEOMETRY_PATH),
        *[
            f"{value:.12f}"
            for value in q
        ],
    ]

    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=15.0,
    )

    return result.stdout


def append_sample(
    label: str,
    q: List[float],
    fk_output: str,
) -> None:
    document = {
        "version": 1,
        "units": {
            "joint_positions": "rad",
        },
        "samples": [],
    }

    if OUTPUT_PATH.exists():
        with OUTPUT_PATH.open(
            "r",
            encoding="utf-8",
        ) as file:
            loaded = yaml.safe_load(file)

        if isinstance(loaded, dict):
            document = loaded

        document.setdefault("samples", [])

    document["samples"].append(
        {
            "label": label,
            "recorded_at": (
                datetime.datetime.now(
                    datetime.timezone.utc
                ).astimezone().isoformat()
            ),
            "joint_names": JOINT_NAMES,
            "joint_positions_rad": [
                float(value)
                for value in q
            ],
            "joint_positions_deg": [
                float(math.degrees(value))
                for value in q
            ],
            "fk_output": fk_output,
        }
    )

    with OUTPUT_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        yaml.safe_dump(
            document,
            file,
            sort_keys=False,
            allow_unicode=True,
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--label",
        default="PICK_GRASP_SAMPLE",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    rclpy.init()
    node = JointStateReader()

    try:
        deadline = time.monotonic() + 10.0

        while (
            rclpy.ok()
            and not node.ready()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if not node.ready():
            print(
                "/joint_states에서 JOINT1~6을 "
                "받지 못했습니다."
            )
            return 1

        q = node.current_q()

        print("=" * 88)
        print("DROK ARM MANUAL POSE RECORDER")
        print("=" * 88)

        print("Label:")
        print(arguments.label)

        print()
        print("JOINT POSITIONS [rad]")
        print(format_rad(q))

        print()
        print("JOINT POSITIONS [deg]")
        print(format_deg(q))

        print()
        print("FORWARD KINEMATICS")
        print("-" * 88)

        fk_output = run_fk(q)
        print(fk_output)

        append_sample(
            arguments.label,
            q,
            fk_output,
        )

        print()
        print("저장 완료:")
        print(OUTPUT_PATH)

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
