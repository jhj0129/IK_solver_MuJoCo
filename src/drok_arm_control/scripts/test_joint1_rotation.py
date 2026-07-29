#!/usr/bin/env python3

import argparse
import math
import sys
import time
from typing import Dict, Optional

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_ACTION = "/arm_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/joint_states"

JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

# 기존 Pick-and-Place에서 사용한 초기 HOME 자세
HOME_Q = [
    -0.000001628,
    +0.297361544,
    +0.296742637,
    -0.000030712,
    +0.000061231,
    +0.000102331,
]

HOME_TOLERANCE_RAD = 0.060
FINAL_TOLERANCE_RAD = math.radians(2.0)

JOINT1_LOWER_RAD = -6.5
JOINT1_UPPER_RAD = +6.5

CONFIRMATION_TOKEN = "EXECUTE_MUJOCO_JOINT1_ROTATION"


class Joint1RotationTester(Node):
    def __init__(self) -> None:
        super().__init__("joint1_rotation_tester")

        self.current_positions: Dict[str, float] = {}
        self.last_joint_state_time = 0.0

        self.subscription = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            20,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION,
        )

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = float(position)

        self.last_joint_state_time = time.monotonic()

    def wait_for_joint_state(
        self,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if all(
                name in self.current_positions
                for name in JOINT_NAMES
            ):
                return True

        return False

    def current_q(self) -> list[float]:
        return [
            self.current_positions[name]
            for name in JOINT_NAMES
        ]

    def check_home_shape(self) -> None:
        q = self.current_q()

        errors = [
            abs(q[index] - HOME_Q[index])
            for index in range(1, 6)
        ]

        maximum_error = max(errors)

        print(
            "[상태 확인] JOINT2~6 HOME 형상 오차: "
            f"{math.degrees(maximum_error):.3f}°"
        )

        if maximum_error > HOME_TOLERANCE_RAD:
            raise RuntimeError(
                "JOINT2~6이 HOME 형상에서 벗어나 있습니다. "
                "MuJoCo를 Reset하거나 다시 실행하십시오."
            )

    def wait_for_action_server(
        self,
        timeout_sec: float,
    ) -> None:
        if not self.action_client.wait_for_server(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError(
                f"Action server를 찾지 못했습니다: "
                f"{ARM_ACTION}"
            )

    def send_target(
        self,
        target_deg: float,
        duration_sec: float,
    ) -> None:
        target_rad = math.radians(target_deg)

        if not (
            JOINT1_LOWER_RAD
            <= target_rad
            <= JOINT1_UPPER_RAD
        ):
            raise RuntimeError(
                "JOINT1 목표가 설정 범위를 벗어났습니다: "
                f"{target_deg:.3f}°"
            )

        current = self.current_q()

        target = current.copy()
        target[0] = target_rad

        # JOINT2~6은 검증된 HOME 값으로 유지한다.
        for index in range(1, 6):
            target[index] = HOME_Q[index]

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec))
            * 1_000_000_000
        )

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        print(
            "[현재 동작] JOINT1 회전: "
            f"{math.degrees(current[0]):.2f}°"
            f" → {target_deg:.2f}°"
        )

        send_future = self.action_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            raise RuntimeError(
                "JOINT1 목표 전송 시간이 초과됐습니다."
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "JOINT1 목표가 거부됐습니다."
            )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=duration_sec + 8.0,
        )

        if not result_future.done():
            raise RuntimeError(
                "JOINT1 동작 완료 대기 시간이 초과됐습니다."
            )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                "Action 결과를 받지 못했습니다."
            )

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise RuntimeError(
                "JOINT1 Action이 실패했습니다. "
                f"status={wrapped_result.status}"
            )

        if wrapped_result.result.error_code != 0:
            raise RuntimeError(
                "Trajectory controller 오류가 발생했습니다. "
                f"error_code="
                f"{wrapped_result.result.error_code}, "
                f"error_string="
                f"{wrapped_result.result.error_string}"
            )

        # 최종 joint state 갱신 대기
        end_time = time.monotonic() + 1.0

        while (
            rclpy.ok()
            and time.monotonic() < end_time
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        final_q1 = self.current_positions["JOINT1"]
        final_error = abs(final_q1 - target_rad)

        print(
            "[결과] JOINT1 실제 위치: "
            f"{math.degrees(final_q1):.3f}°"
        )

        print(
            "[결과] JOINT1 목표 오차: "
            f"{math.degrees(final_error):.3f}°"
        )

        if final_error > FINAL_TOLERANCE_RAD:
            raise RuntimeError(
                "JOINT1 최종 오차가 2°를 초과했습니다."
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo HOME 자세에서 JOINT1만 "
            "절대각도로 회전시킵니다."
        )
    )

    parser.add_argument(
        "--target-deg",
        type=float,
        required=True,
        help=(
            "JOINT1 절대 목표각도. "
            "예: 90, 180, 270, 360"
        ),
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="회전 시간(초). 기본값 3.0",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="실제 MuJoCo trajectory를 실행합니다.",
    )

    parser.add_argument(
        "--confirmation",
        type=str,
        default="",
        help="실행 확인 문자열",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    if args.duration <= 0.0:
        print(
            "duration은 0보다 커야 합니다.",
            file=sys.stderr,
        )
        return 2

    if not args.execute:
        print("JOINT1 ROTATION TEST — DRY RUN")
        print(f"Target   : {args.target_deg:.3f}°")
        print(f"Duration : {args.duration:.3f} sec")
        print()
        print("실행하려면 다음 옵션이 필요합니다:")
        print(
            "--execute "
            f"--confirmation {CONFIRMATION_TOKEN}"
        )
        return 0

    if args.confirmation != CONFIRMATION_TOKEN:
        print(
            "실행 확인 문자열이 올바르지 않습니다.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()

    node: Optional[Joint1RotationTester] = None

    try:
        node = Joint1RotationTester()

        print(
            "[시작 대기] joint state와 "
            "arm controller 확인"
        )

        if not node.wait_for_joint_state(5.0):
            raise RuntimeError(
                "JOINT1~6 joint state를 받지 못했습니다."
            )

        node.check_home_shape()
        node.wait_for_action_server(5.0)

        node.send_target(
            target_deg=args.target_deg,
            duration_sec=args.duration,
        )

        print(
            f"[종료] JOINT1 "
            f"{args.target_deg:.2f}° 회전 PASS"
        )

        return 0

    except Exception as exception:
        print(
            f"[실패] {exception}",
            file=sys.stderr,
        )
        return 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
