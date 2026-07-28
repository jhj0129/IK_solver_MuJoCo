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

CONFIRMATION_TEXT = "EXECUTE_ARM_ONLY"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--timed-path",
        default=str(
            ROOT
            / "cylinder_timed_joint_path_segmented.yaml"
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
        "--stop-after-block",
        type=int,
        default=0,
        help=(
            "0=entry only, 1~5=해당 block까지 실행"
        ),
    )

    parser.add_argument(
        "--execute-arm-only",
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
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError(
            f"잘못된 시간입니다: {seconds}"
        )

    nanoseconds = int(
        round(seconds * 1.0e9)
    )

    message = Duration()
    message.sec = int(
        nanoseconds // 1_000_000_000
    )
    message.nanosec = int(
        nanoseconds % 1_000_000_000
    )

    return message


def maximum_difference(
    first: list[float],
    second: list[float],
) -> float:
    if len(first) != len(second):
        raise RuntimeError(
            "관절 벡터 크기가 다릅니다."
        )

    return max(
        abs(a - b)
        for a, b in zip(first, second)
    )


def quintic_values(
    tau: float,
) -> tuple[float, float, float]:
    s = (
        10.0 * tau**3
        - 15.0 * tau**4
        + 6.0 * tau**5
    )

    ds = (
        30.0 * tau**2
        - 60.0 * tau**3
        + 30.0 * tau**4
    )

    d2s = (
        60.0 * tau
        - 180.0 * tau**2
        + 120.0 * tau**3
    )

    return s, ds, d2s


def calculate_entry_duration(
    start: list[float],
    goal: list[float],
    joint_names: list[str],
    timed_document: dict[str, Any],
    speed_scale: float,
    duration_margin: float,
    minimum_duration: float,
) -> float:
    duration = 0.0

    for index, joint_name in enumerate(
        joint_names
    ):
        limit = timed_document["limits"][
            joint_name
        ]

        velocity_limit = float(
            limit["used_velocity_limit"]
        )

        acceleration_limit = float(
            limit["used_acceleration_limit"]
        )

        jerk_limit = float(
            limit["used_jerk_limit"]
        )

        displacement = abs(
            goal[index] - start[index]
        )

        velocity_duration = (
            1.875
            * displacement
            / velocity_limit
        )

        acceleration_duration = math.sqrt(
            (
                10.0
                / math.sqrt(3.0)
            )
            * displacement
            / acceleration_limit
        )

        jerk_duration = (
            60.0
            * displacement
            / jerk_limit
        ) ** (1.0 / 3.0)

        duration = max(
            duration,
            velocity_duration,
            acceleration_duration,
            jerk_duration,
        )

    duration *= duration_margin
    duration /= speed_scale

    return max(
        duration,
        minimum_duration,
    )


def create_entry_trajectory(
    start: list[float],
    goal: list[float],
    joint_names: list[str],
    controller_rate: float,
    duration: float,
) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(
        joint_names
    )

    interval_count = max(
        2,
        int(
            math.ceil(
                duration * controller_rate
            )
        ),
    )

    duration = (
        interval_count / controller_rate
    )

    for sample_index in range(
        1,
        interval_count + 1,
    ):
        time_from_start = (
            sample_index / controller_rate
        )

        tau = min(
            time_from_start / duration,
            1.0,
        )

        progress, first, second = (
            quintic_values(tau)
        )

        point = JointTrajectoryPoint()

        point.positions = [
            q0 + (q1 - q0) * progress
            for q0, q1 in zip(
                start,
                goal,
            )
        ]

        point.velocities = [
            (q1 - q0)
            * first
            / duration
            for q0, q1 in zip(
                start,
                goal,
            )
        ]

        point.accelerations = [
            (q1 - q0)
            * second
            / duration**2
            for q0, q1 in zip(
                start,
                goal,
            )
        ]

        point.time_from_start = (
            duration_message(
                time_from_start
            )
        )

        trajectory.points.append(point)

    return trajectory


def create_block_trajectory(
    block: dict[str, Any],
    joint_names: list[str],
    speed_scale: float,
) -> JointTrajectory:
    points = block.get("points", [])

    if len(points) < 2:
        raise RuntimeError(
            f"{block['name']}의 point가 부족합니다."
        )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(
        joint_names
    )

    # index 0은 현재 상태와 일치해야 하는 경계점이다.
    # action에는 그 다음 point부터 넣는다.
    for source in points[1:]:
        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
            for value in source["positions"]
        ]

        point.velocities = [
            speed_scale * float(value)
            for value in source["velocities"]
        ]

        point.accelerations = [
            speed_scale**2 * float(value)
            for value in source["accelerations"]
        ]

        scaled_time = (
            float(source["time_from_start"])
            / speed_scale
        )

        point.time_from_start = (
            duration_message(scaled_time)
        )

        trajectory.points.append(point)

    return trajectory


class TimedArmExecutor(Node):
    def __init__(
        self,
        joint_names: list[str],
        joint_state_topic: str,
        arm_action: str,
    ) -> None:
        super().__init__(
            "cylinder_timed_arm_executor"
        )

        self.joint_names = list(
            joint_names
        )

        self.latest_positions: (
            dict[str, float] | None
        ) = None

        self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            arm_action,
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        if len(message.name) != len(
            message.position
        ):
            return

        values = dict(
            zip(
                message.name,
                message.position,
            )
        )

        if not all(
            name in values
            for name in self.joint_names
        ):
            return

        positions = {
            name: float(values[name])
            for name in self.joint_names
        }

        if not all(
            math.isfinite(value)
            for value in positions.values()
        ):
            return

        self.latest_positions = positions

    def wait_for_joint_state(
        self,
        timeout: float,
    ) -> list[float]:
        deadline = (
            time.monotonic() + timeout
        )

        while (
            rclpy.ok()
            and self.latest_positions is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.latest_positions is None:
            raise RuntimeError(
                "시간 내에 JOINT1~6 상태를 "
                "받지 못했습니다."
            )

        return [
            self.latest_positions[name]
            for name in self.joint_names
        ]

    def current_positions(
        self,
    ) -> list[float]:
        rclpy.spin_once(
            self,
            timeout_sec=0.1,
        )

        if self.latest_positions is None:
            raise RuntimeError(
                "현재 관절 상태가 없습니다."
            )

        return [
            self.latest_positions[name]
            for name in self.joint_names
        ]

    def send_and_wait(
        self,
        trajectory: JointTrajectory,
        label: str,
        server_timeout: float,
        result_timeout: float,
    ) -> None:
        if not trajectory.points:
            raise RuntimeError(
                f"{label} trajectory가 비어 있습니다."
            )

        if not self.arm_client.wait_for_server(
            timeout_sec=server_timeout
        ):
            raise RuntimeError(
                "arm action server를 찾지 못했습니다."
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"{label}: "
            f"{len(trajectory.points)} points 전송"
        )

        send_future = (
            self.arm_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=server_timeout,
        )

        if not send_future.done():
            raise RuntimeError(
                f"{label}: goal 응답 timeout"
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f"{label}: goal rejected"
            )

        self.get_logger().info(
            f"{label}: goal accepted"
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
            goal_handle.cancel_goal_async()

            raise RuntimeError(
                f"{label}: action result timeout"
            )

        response = result_future.result()

        if response is None:
            raise RuntimeError(
                f"{label}: result가 없습니다."
            )

        result = response.result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"{label}: controller error_code="
                f"{result.error_code}, "
                f"error_string={result.error_string}"
            )

        self.get_logger().info(
            f"{label}: completed"
        )


def trajectory_duration(
    trajectory: JointTrajectory,
) -> float:
    if not trajectory.points:
        return 0.0

    duration = trajectory.points[
        -1
    ].time_from_start

    return (
        float(duration.sec)
        + 1.0e-9
        * float(duration.nanosec)
    )


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

    joint_names = [
        str(name)
        for name in timed_document[
            "joint_names"
        ]
    ]

    if joint_names != [
        "JOINT1",
        "JOINT2",
        "JOINT3",
        "JOINT4",
        "JOINT5",
        "JOINT6",
    ]:
        raise RuntimeError(
            "timed path의 joint 순서가 "
            "JOINT1~6과 일치하지 않습니다."
        )

    blocks = timed_document.get(
        "blocks",
        [],
    )

    if not blocks:
        raise RuntimeError(
            "timed path에 block이 없습니다."
        )

    if not (
        0
        <= arguments.stop_after_block
        <= len(blocks)
    ):
        raise RuntimeError(
            "stop-after-block 범위가 잘못됐습니다."
        )

    interface_config = config[
        "interfaces"
    ]

    execution_config = config[
        "execution"
    ]

    validation_config = config[
        "validation"
    ]

    entry_config = config[
        "entry_motion"
    ]

    speed_scale = float(
        execution_config["speed_scale"]
    )

    if not (
        0.0 < speed_scale <= 1.0
    ):
        raise RuntimeError(
            "speed_scale은 0보다 크고 "
            "1 이하여야 합니다."
        )

    controller_rate = float(
        timed_document[
            "controller_rate_hz"
        ]
    )

    first_path_position = [
        float(value)
        for value in blocks[0][
            "points"
        ][0]["positions"]
    ]

    nominal_block_duration = sum(
        float(block["duration"])
        for block in blocks[
            :arguments.stop_after_block
        ]
    )

    print("=" * 88)
    print("CYLINDER TIMED ARM PATH EXECUTOR")
    print("=" * 88)

    print(
        f"Timed path       : {timed_path}"
    )

    print(
        f"Arm action       : "
        f"{interface_config['arm_action']}"
    )

    print(
        f"Controller rate  : "
        f"{controller_rate:.3f} Hz"
    )

    print(
        f"Speed scale      : "
        f"{speed_scale:.3f}"
    )

    print(
        f"Stop after block : "
        f"{arguments.stop_after_block}"
    )

    print(
        "Selected blocks  : "
        + (
            ", ".join(
                block["name"]
                for block in blocks[
                    :arguments.stop_after_block
                ]
            )
            if arguments.stop_after_block > 0
            else "ENTRY ONLY"
        )
    )

    print(
        "Scaled block time: "
        f"{nominal_block_duration / speed_scale:.3f} s"
    )

    print()
    print(
        "GRIPPER COMMAND  : DISABLED"
    )

    execute = (
        arguments.execute_arm_only
        and arguments.confirmation
        == CONFIRMATION_TEXT
    )

    if not execute:
        print()
        print("DRY-RUN RESULT: PASS")
        print(
            "아무 trajectory도 전송하지 않았습니다."
        )
        print()
        print(
            "실행에는 다음 두 인자가 모두 필요합니다:"
        )
        print(
            "--execute-arm-only "
            f"--confirmation {CONFIRMATION_TEXT}"
        )

        return 0

    rclpy.init()

    node = TimedArmExecutor(
        joint_names=joint_names,
        joint_state_topic=str(
            interface_config[
                "joint_state_topic"
            ]
        ),
        arm_action=str(
            interface_config["arm_action"]
        ),
    )

    try:
        state_timeout = float(
            execution_config[
                "joint_state_timeout_sec"
            ]
        )

        server_timeout = float(
            execution_config[
                "action_server_timeout_sec"
            ]
        )

        result_margin = float(
            execution_config[
                "action_result_margin_sec"
            ]
        )

        start_tolerance = float(
            validation_config[
                "maximum_start_error_rad"
            ]
        )

        goal_tolerance = float(
            validation_config[
                "maximum_goal_error_rad"
            ]
        )

        current_q = node.wait_for_joint_state(
            state_timeout
        )

        entry_duration = (
            calculate_entry_duration(
                current_q,
                first_path_position,
                joint_names,
                timed_document,
                speed_scale,
                float(
                    entry_config[
                        "duration_margin"
                    ]
                ),
                float(
                    entry_config[
                        "minimum_duration_sec"
                    ]
                ),
            )
        )

        entry_trajectory = (
            create_entry_trajectory(
                current_q,
                first_path_position,
                joint_names,
                controller_rate,
                entry_duration,
            )
        )

        print()
        print(
            "Current q        : "
            + ", ".join(
                f"{value:+.6f}"
                for value in current_q
            )
        )

        print(
            "PICK_PREGRASP q  : "
            + ", ".join(
                f"{value:+.6f}"
                for value in first_path_position
            )
        )

        print(
            "Entry max change : "
            f"{math.degrees(maximum_difference(current_q, first_path_position)):.3f} deg"
        )

        print(
            "Entry duration   : "
            f"{trajectory_duration(entry_trajectory):.3f} s"
        )

        node.send_and_wait(
            entry_trajectory,
            "ENTRY_TO_PICK_PREGRASP",
            server_timeout,
            trajectory_duration(
                entry_trajectory
            ) + result_margin,
        )

        measured_q = node.current_positions()

        entry_error = maximum_difference(
            measured_q,
            first_path_position,
        )

        print(
            "Entry goal error : "
            f"{math.degrees(entry_error):.6f} deg"
        )

        if entry_error > goal_tolerance:
            raise RuntimeError(
                "ENTRY 종료 관절 오차가 큽니다: "
                f"{entry_error:.9f} rad"
            )

        if arguments.stop_after_block == 0:
            print()
            print(
                "EXECUTION RESULT: PASS — ENTRY ONLY"
            )
            return 0

        for block_number, block in enumerate(
            blocks[
                :arguments.stop_after_block
            ],
            start=1,
        ):
            block_start = [
                float(value)
                for value in block[
                    "points"
                ][0]["positions"]
            ]

            current_q = node.current_positions()

            start_error = maximum_difference(
                current_q,
                block_start,
            )

            print()
            print(
                f"[BLOCK {block_number}] "
                f"{block['name']}"
            )

            print(
                "Start error      : "
                f"{math.degrees(start_error):.6f} deg"
            )

            if start_error > start_tolerance:
                raise RuntimeError(
                    f"{block['name']} 시작점 오차가 "
                    f"큽니다: {start_error:.9f} rad"
                )

            trajectory = (
                create_block_trajectory(
                    block,
                    joint_names,
                    speed_scale,
                )
            )

            duration = trajectory_duration(
                trajectory
            )

            print(
                f"Points           : "
                f"{len(trajectory.points)}"
            )

            print(
                f"Scaled duration  : "
                f"{duration:.3f} s"
            )

            node.send_and_wait(
                trajectory,
                str(block["name"]),
                server_timeout,
                duration + result_margin,
            )

            block_goal = [
                float(value)
                for value in block[
                    "points"
                ][-1]["positions"]
            ]

            measured_q = node.current_positions()

            goal_error = maximum_difference(
                measured_q,
                block_goal,
            )

            print(
                "Goal error       : "
                f"{math.degrees(goal_error):.6f} deg"
            )

            if goal_error > goal_tolerance:
                raise RuntimeError(
                    f"{block['name']} 종료점 오차가 "
                    f"큽니다: {goal_error:.9f} rad"
                )

        print()
        print("=" * 88)
        print("EXECUTION RESULT: PASS")
        print("=" * 88)
        print(
            "Gripper commands were not sent."
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
