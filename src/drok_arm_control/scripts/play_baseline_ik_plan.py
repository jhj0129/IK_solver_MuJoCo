#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]


class PlanPlayer(Node):
    def __init__(self) -> None:
        super().__init__("baseline_ik_plan_player")

        self.positions: Dict[str, float] = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
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
            self.positions[joint]
            for joint in JOINT_NAMES
        ]


def duration_message(seconds: float) -> Duration:
    whole_seconds = int(seconds)

    nanoseconds = int(
        round(
            (seconds - whole_seconds)
            * 1_000_000_000
        )
    )

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    return Duration(
        sec=whole_seconds,
        nanosec=nanoseconds,
    )


def max_delta(
    first: List[float],
    second: List[float],
) -> float:
    return max(
        abs(a - b)
        for a, b in zip(first, second)
    )


def format_degrees(values: List[float]) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+7.2f}°"
            for value in values
        )
        + "]"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--plan",
        default=str(
            Path.home()
            / "IK_solver_MuJoCo"
            / "baseline_incremental_nearest_ik_plan.yaml"
        ),
    )

    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=12.0,
    )

    parser.add_argument(
        "--minimum-segment-s",
        type=float,
        default=0.60,
    )

    parser.add_argument(
        "--start-tolerance-deg",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    plan_path = Path(
        arguments.plan
    ).expanduser()

    if not plan_path.exists():
        print("Plan 파일이 없습니다:")
        print(plan_path)
        return 1

    if arguments.speed_deg_s <= 0.0:
        print("speed-deg-s는 0보다 커야 합니다.")
        return 1

    if arguments.minimum_segment_s <= 0.0:
        print(
            "minimum-segment-s는 0보다 커야 합니다."
        )
        return 1

    with plan_path.open(
        "r",
        encoding="utf-8",
    ) as file:
        plan = yaml.safe_load(file)

    start_q = [
        float(value)
        for value in plan["start_q"]
    ]

    waypoints = plan["waypoints"]

    if not waypoints:
        print("Plan에 waypoint가 없습니다.")
        return 1

    waypoint_positions = [
        [
            float(value)
            for value in waypoint["selected_q"]
        ]
        for waypoint in waypoints
    ]

    for index, q in enumerate(
        waypoint_positions,
        start=1,
    ):
        if len(q) != 6:
            print(
                f"Waypoint {index}의 관절 수가 "
                "6개가 아닙니다."
            )
            return 1

    rclpy.init()
    node = PlanPlayer()

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
            return 2

        current_q = node.current_q()

        start_error = max_delta(
            current_q,
            start_q,
        )

        print("=" * 88)
        print("DROK ARM BASELINE IK PLAN PLAYER")
        print("=" * 88)

        print("Plan:")
        print(plan_path)

        print()
        print("현재 관절각:")
        print(" ", format_degrees(current_q))

        print("계획 시작각:")
        print(" ", format_degrees(start_q))

        print(
            "시작 자세 오차:",
            f"{math.degrees(start_error):.3f}°",
        )

        print(
            "Waypoint 수:",
            len(waypoints),
        )

        print(
            "기준 속도:",
            f"{arguments.speed_deg_s:.3f} deg/s",
        )

        if (
            start_error
            > math.radians(
                arguments.start_tolerance_deg
            )
        ):
            print()
            print(
                "실행 중단: 현재 자세가 dry-run의 "
                "시작 자세와 너무 다릅니다."
            )
            print(
                "MuJoCo를 HOME으로 초기화한 뒤 "
                "다시 실행하세요."
            )
            return 3

        speed_rad_per_second = math.radians(
            arguments.speed_deg_s
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        cumulative_time = 0.0
        previous_q = current_q

        print()
        print("TRAJECTORY SEGMENTS")

        for index, (waypoint, target_q) in enumerate(
            zip(waypoints, waypoint_positions),
            start=1,
        ):
            segment_delta = max_delta(
                previous_q,
                target_q,
            )

            segment_time = max(
                arguments.minimum_segment_s,
                segment_delta
                / speed_rad_per_second,
            )

            cumulative_time += segment_time

            point = JointTrajectoryPoint()
            point.positions = target_q
            point.velocities = [0.0] * 6
            point.time_from_start = duration_message(
                cumulative_time
            )

            goal.trajectory.points.append(point)

            print(
                f"{index:02d}. "
                f"{waypoint['name']:16s} "
                f"maxΔ="
                f"{math.degrees(segment_delta):7.3f}° "
                f"dt={segment_time:6.3f}s "
                f"t={cumulative_time:7.3f}s"
            )

            previous_q = target_q

        print()
        print(
            "총 예상 시간:",
            f"{cumulative_time:.3f} s",
        )

        if not arguments.execute:
            print()
            print(
                "PREVIEW ONLY: 팔은 움직이지 않았습니다."
            )
            print(
                "실행하려면 --execute를 추가하세요."
            )
            return 0

        if not node.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            print(
                "arm_controller action server가 없습니다."
            )
            return 4

        print()
        print(
            "팔 경로를 실행합니다. "
            "그리퍼 명령은 전송하지 않습니다."
        )

        send_future = (
            node.action_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            node,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            print("Goal 전송 시간이 초과됐습니다.")
            return 5

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            print("Trajectory goal이 거부됐습니다.")
            return 5

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            node,
            result_future,
            timeout_sec=cumulative_time + 20.0,
        )

        if not result_future.done():
            print("Trajectory 실행 시간이 초과됐습니다.")
            return 6

        wrapped_result = result_future.result()

        if wrapped_result is None:
            print("Trajectory 결과가 없습니다.")
            return 6

        result = wrapped_result.result

        if result.error_code != 0:
            print("Trajectory 실행 실패")
            print("error_code:", result.error_code)
            print("error_string:", result.error_string)
            return 7

        for _ in range(10):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        final_q = node.current_q()
        target_final_q = waypoint_positions[-1]

        final_error = max_delta(
            final_q,
            target_final_q,
        )

        print()
        print("Trajectory 실행 완료")

        print("최종 실제 관절각:")
        print(" ", format_degrees(final_q))

        print("최종 목표 관절각:")
        print(" ", format_degrees(target_final_q))

        print(
            "최종 최대 관절 오차:",
            f"{math.degrees(final_error):.3f}°",
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
