#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


ROOT = Path.home() / "IK_solver_MuJoCo"

ARM_JOINTS = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

GRIPPER_TARGETS = {
    "JOINT7": 0.0488,
    "GRIPPER_RIGHT_JOINT": -0.0488,
}

CONFIRMATION_TEXT = "EXECUTE_PLACE_DESCEND_ONLY"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--timed-path",
        default=str(
            ROOT
            / "cylinder_best_timed_joint_path_segmented.yaml"
        ),
    )

    parser.add_argument(
        "--config",
        default=str(
            ROOT
            / "src/drok_arm_control/config"
            / "cylinder_timed_path_executor.yaml"
        ),
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    parser.add_argument(
        "--confirmation",
        default="",
    )

    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
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


def duration_message(seconds: float) -> Duration:
    nanoseconds = int(
        round(seconds * 1.0e9)
    )

    message = Duration()
    message.sec = (
        nanoseconds // 1_000_000_000
    )
    message.nanosec = (
        nanoseconds % 1_000_000_000
    )

    return message


def trajectory_duration(
    trajectory: JointTrajectory,
) -> float:
    final = trajectory.points[
        -1
    ].time_from_start

    return (
        float(final.sec)
        + float(final.nanosec) * 1.0e-9
    )


def maximum_error(
    actual: list[float],
    target: list[float],
) -> float:
    return max(
        abs(a - b)
        for a, b in zip(
            actual,
            target,
        )
    )


class TransferExecutor(Node):
    def __init__(
        self,
        action_name: str,
        joint_state_topic: str,
    ) -> None:
        super().__init__(
            "cylinder_place_descend_only_executor"
        )

        self.latest_state: (
            dict[str, float] | None
        ) = None

        self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name,
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        if len(message.name) != len(
            message.position
        ):
            return

        values = {
            name: float(position)
            for name, position in zip(
                message.name,
                message.position,
            )
        }

        required = (
            ARM_JOINTS
            + list(
                GRIPPER_TARGETS.keys()
            )
        )

        if not all(
            name in values
            for name in required
        ):
            return

        if not all(
            math.isfinite(values[name])
            for name in required
        ):
            return

        self.latest_state = values

    def wait_for_state(
        self,
        timeout_sec: float,
    ) -> dict[str, float]:
        deadline = (
            time.monotonic()
            + timeout_sec
        )

        while (
            rclpy.ok()
            and self.latest_state is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.latest_state is None:
            raise RuntimeError(
                "/joint_states 수신 timeout"
            )

        return dict(
            self.latest_state
        )

    def refresh_state(
        self,
    ) -> dict[str, float]:
        rclpy.spin_once(
            self,
            timeout_sec=0.2,
        )

        if self.latest_state is None:
            raise RuntimeError(
                "현재 관절 상태가 없습니다."
            )

        return dict(
            self.latest_state
        )

    def execute(
        self,
        trajectory: JointTrajectory,
        server_timeout: float,
        result_timeout: float,
    ) -> None:
        if not self.action_client.wait_for_server(
            timeout_sec=server_timeout
        ):
            raise RuntimeError(
                "arm action server를 찾지 못했습니다."
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"PLACE_DESCEND_ONLY: "
            f"{len(trajectory.points)} points 전송"
        )

        send_future = (
            self.action_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=server_timeout,
        )

        if not send_future.done():
            raise RuntimeError(
                "goal 응답 timeout"
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "TRANSFER goal rejected"
            )

        self.get_logger().info(
            "PLACE_DESCEND_ONLY: goal accepted"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=result_timeout,
        )

        if not result_future.done():
            cancel_future = (
                goal_handle.cancel_goal_async()
            )

            rclpy.spin_until_future_complete(
                self,
                cancel_future,
                timeout_sec=2.0,
            )

            raise RuntimeError(
                "TRANSFER action result timeout"
            )

        response = result_future.result()

        if response is None:
            raise RuntimeError(
                "action result가 없습니다."
            )

        result = response.result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                "TRANSFER controller 오류: "
                f"code={result.error_code}, "
                f"message={result.error_string}"
            )

        self.get_logger().info(
            "PLACE_DESCEND_ONLY: completed"
        )


def create_transfer_trajectory(
    block: dict[str, Any],
    speed_scale: float,
) -> JointTrajectory:
    source_points = block.get(
        "points",
        [],
    )

    if len(source_points) < 2:
        raise RuntimeError(
            "TRANSFER point가 부족합니다."
        )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(
        ARM_JOINTS
    )

    # 첫 point는 현재 PICK_GRASP 경계점이므로
    # 다음 point부터 controller로 전송한다.
    for source in source_points[1:]:
        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
            for value in source[
                "positions"
            ]
        ]

        point.velocities = [
            speed_scale * float(value)
            for value in source[
                "velocities"
            ]
        ]

        point.accelerations = [
            speed_scale**2 * float(value)
            for value in source[
                "accelerations"
            ]
        ]

        scaled_time = (
            float(
                source["time_from_start"]
            )
            / speed_scale
        )

        point.time_from_start = (
            duration_message(
                scaled_time
            )
        )

        trajectory.points.append(point)

    return trajectory


def main() -> int:
    arguments = parse_arguments()

    timed_path = Path(
        arguments.timed_path
    ).expanduser().resolve()

    config_path = Path(
        arguments.config
    ).expanduser().resolve()

    timed_document = load_yaml(
        timed_path
    )

    config = load_yaml(
        config_path
    )

    blocks = timed_document.get(
        "blocks",
        [],
    )

    matching_blocks = [
        block
        for block in blocks
        if block.get("source_segment")
        == "PLACE_DESCEND"
    ]

    if len(matching_blocks) != 1:
        raise RuntimeError(
            "TRANSFER block을 정확히 "
            "하나 찾지 못했습니다."
        )

    block = matching_blocks[0]

    speed_scale = float(
        config["execution"][
            "speed_scale"
        ]
    )

    start_tolerance = float(
        config["validation"][
            "maximum_start_error_rad"
        ]
    )

    goal_tolerance = float(
        config["validation"][
            "maximum_goal_error_rad"
        ]
    )

    state_timeout = float(
        config["execution"][
            "joint_state_timeout_sec"
        ]
    )

    server_timeout = float(
        config["execution"][
            "action_server_timeout_sec"
        ]
    )

    result_margin = float(
        config["execution"][
            "action_result_margin_sec"
        ]
    )

    trajectory = create_transfer_trajectory(
        block,
        speed_scale,
    )

    start_target = [
        float(value)
        for value in block[
            "points"
        ][0]["positions"]
    ]

    goal_target = [
        float(value)
        for value in block[
            "points"
        ][-1]["positions"]
    ]

    duration = trajectory_duration(
        trajectory
    )

    print("=" * 88)
    print("CYLINDER PLACE DESCEND ONLY")
    print("=" * 88)

    print(
        f"Timed path      : {timed_path}"
    )

    print(
        f"Block           : {block['name']}"
    )

    print(
        f"Speed scale     : {speed_scale:.3f}"
    )

    print(
        f"Lift duration   : {duration:.3f} s"
    )

    print(
        "Expected motion : "
        "PLACE 상공에서 받침대까지 수직 50 mm 하강"
    )

    print(
        "Gripper command : 전송하지 않음"
    )

    execute = (
        arguments.execute
        and arguments.confirmation
        == CONFIRMATION_TEXT
    )

    if not execute:
        print()
        print("DRY-RUN RESULT: PASS")
        print(
            "실행하려면 다음 인자가 필요합니다:"
        )
        print(
            "--execute "
            "--confirmation "
            f"{CONFIRMATION_TEXT}"
        )

        return 0

    rclpy.init()

    node = TransferExecutor(
        action_name=str(
            config["interfaces"][
                "arm_action"
            ]
        ),
        joint_state_topic=str(
            config["interfaces"][
                "joint_state_topic"
            ]
        ),
    )

    try:
        state = node.wait_for_state(
            state_timeout
        )

        current_arm = [
            state[name]
            for name in ARM_JOINTS
        ]

        start_error = maximum_error(
            current_arm,
            start_target,
        )

        print()
        print(
            "Arm start error : "
            f"{math.degrees(start_error):.6f} deg"
        )

        for name, target in (
            GRIPPER_TARGETS.items()
        ):
            actual = state[name]
            error = abs(
                actual - target
            )

            print(
                f"{name:24s}: "
                f"{actual:+.9f} m "
                f"(error={error * 1000.0:.3f} mm)"
            )

            if error > 0.003:
                raise RuntimeError(
                    f"{name}이 50% 닫힘 상태가 "
                    "아닙니다."
                )

        if start_error > start_tolerance:
            raise RuntimeError(
                "현재 팔이 TRANSFER 시작점과 "
                "일치하지 않습니다: "
                f"{start_error:.9f} rad"
            )

        node.execute(
            trajectory,
            server_timeout,
            duration + result_margin,
        )

        final_state = node.refresh_state()

        final_arm = [
            final_state[name]
            for name in ARM_JOINTS
        ]

        goal_error = maximum_error(
            final_arm,
            goal_target,
        )

        print(
            "Arm goal error  : "
            f"{math.degrees(goal_error):.6f} deg"
        )

        if goal_error > goal_tolerance:
            raise RuntimeError(
                "TRANSFER 종료 관절 오차가 "
                "큽니다."
            )

        for name, target in (
            GRIPPER_TARGETS.items()
        ):
            actual = final_state[name]
            error = abs(
                actual - target
            )

            print(
                f"Final {name:18s}: "
                f"{actual:+.9f} m "
                f"(error={error * 1000.0:.3f} mm)"
            )

            if error > 0.003:
                raise RuntimeError(
                    "LIFT 도중 그리퍼가 "
                    "열렸습니다."
                )

        print()
        print("=" * 88)
        print("PLACE DESCEND RESULT: PASS")
        print("=" * 88)

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
