#!/usr/bin/env python3

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

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

ARM_JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

GRIPPER_JOINT_NAMES = [
    "JOINT7",
    "GRIPPER_RIGHT_JOINT",
]

EXPECTED_RETREAT_Q = [
    -0.587524586,
    +1.209627227,
    +0.566751133,
    +1.039245857,
    -0.715373374,
    +0.734186776,
]

HOME_Q = [
    -0.000001628,
    +0.297361544,
    +0.296742637,
    -0.000030712,
    +0.000061231,
    +0.000102331,
]

CONTROLLER_RATE_HZ = 100.0

RETREAT_START_TOLERANCE_RAD = 0.060
GRIPPER_OPEN_TOLERANCE_M = 0.003
FINAL_HOME_TOLERANCE_RAD = math.radians(2.0)

DEFAULT_FOLD_DURATION_SEC = 8.0
DEFAULT_BASE_DURATION_SEC = 4.0

CONFIRMATION_TOKEN = "EXECUTE_MUJOCO_RETURN_HOME"


def quintic_scaling(
    normalized_time: float,
    duration_sec: float,
) -> Tuple[float, float, float]:
    u = min(
        1.0,
        max(0.0, normalized_time),
    )

    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u

    position_scale = (
        10.0 * u3
        - 15.0 * u4
        + 6.0 * u5
    )

    velocity_scale = (
        30.0 * u2
        - 60.0 * u3
        + 30.0 * u4
    ) / duration_sec

    acceleration_scale = (
        60.0 * u
        - 180.0 * u2
        + 120.0 * u3
    ) / (duration_sec * duration_sec)

    return (
        position_scale,
        velocity_scale,
        acceleration_scale,
    )


def interpolate_quintic(
    start: List[float],
    goal: List[float],
    duration_sec: float,
    local_time_sec: float,
) -> Tuple[
    List[float],
    List[float],
    List[float],
]:
    position_scale, velocity_scale, acceleration_scale = (
        quintic_scaling(
            local_time_sec / duration_sec,
            duration_sec,
        )
    )

    delta = [
        goal_value - start_value
        for start_value, goal_value in zip(
            start,
            goal,
        )
    ]

    positions = [
        start_value + position_scale * difference
        for start_value, difference in zip(
            start,
            delta,
        )
    ]

    velocities = [
        velocity_scale * difference
        for difference in delta
    ]

    accelerations = [
        acceleration_scale * difference
        for difference in delta
    ]

    return (
        positions,
        velocities,
        accelerations,
    )


def set_time_from_start(
    point: JointTrajectoryPoint,
    time_sec: float,
) -> None:
    whole_seconds = int(math.floor(time_sec))

    nanoseconds = int(
        round(
            (time_sec - whole_seconds)
            * 1_000_000_000
        )
    )

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    point.time_from_start.sec = whole_seconds
    point.time_from_start.nanosec = nanoseconds


class ReturnHomeExecutor(Node):
    def __init__(self) -> None:
        super().__init__(
            "cylinder_return_home_only_executor"
        )

        self.current_positions: Dict[str, float] = {}

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

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        for name, position in zip(
            message.name,
            message.position,
        ):
            self.current_positions[name] = float(position)

    def wait_for_joint_state(
        self,
        timeout_sec: float,
    ) -> bool:
        required_names = (
            ARM_JOINT_NAMES
            + GRIPPER_JOINT_NAMES
        )

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
                for name in required_names
            ):
                return True

        return False

    def current_arm_q(self) -> List[float]:
        return [
            self.current_positions[name]
            for name in ARM_JOINT_NAMES
        ]

    def validate_start_state(self) -> None:
        current_q = self.current_arm_q()

        maximum_retreat_error = max(
            abs(current - expected)
            for current, expected in zip(
                current_q,
                EXPECTED_RETREAT_Q,
            )
        )

        gripper_open_error = max(
            abs(
                self.current_positions[name]
            )
            for name in GRIPPER_JOINT_NAMES
        )

        print(
            "[상태 확인] RETREAT 시작 오차: "
            f"{math.degrees(maximum_retreat_error):.3f}°"
        )

        print(
            "[상태 확인] 그리퍼 개방 오차: "
            f"{gripper_open_error * 1000.0:.3f} mm"
        )

        if (
            maximum_retreat_error
            > RETREAT_START_TOLERANCE_RAD
        ):
            raise RuntimeError(
                "현재 팔 자세가 PLACE_RETREAT 종료점에서 "
                "0.060 rad 이상 벗어났습니다."
            )

        if (
            gripper_open_error
            > GRIPPER_OPEN_TOLERANCE_M
        ):
            raise RuntimeError(
                "HOME 복귀 전에 그리퍼가 완전히 "
                "열려 있지 않습니다."
            )

    def build_trajectory(
        self,
        fold_duration_sec: float,
        base_duration_sec: float,
    ) -> JointTrajectory:
        start_q = self.current_arm_q()

        # 첫 단계:
        # JOINT1은 현재 방향을 유지하고 JOINT2~6만 HOME으로 접는다.
        folded_q = [
            start_q[0],
            HOME_Q[1],
            HOME_Q[2],
            HOME_Q[3],
            HOME_Q[4],
            HOME_Q[5],
        ]

        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINT_NAMES

        controller_period_sec = (
            1.0 / CONTROLLER_RATE_HZ
        )

        fold_interval_count = int(
            round(
                fold_duration_sec
                * CONTROLLER_RATE_HZ
            )
        )

        base_interval_count = int(
            round(
                base_duration_sec
                * CONTROLLER_RATE_HZ
            )
        )

        # PHASE 1: JOINT2~6 접기
        for index in range(
            fold_interval_count + 1
        ):
            local_time_sec = min(
                index * controller_period_sec,
                fold_duration_sec,
            )

            positions, velocities, accelerations = (
                interpolate_quintic(
                    start=start_q,
                    goal=folded_q,
                    duration_sec=fold_duration_sec,
                    local_time_sec=local_time_sec,
                )
            )

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations

            set_time_from_start(
                point,
                local_time_sec,
            )

            trajectory.points.append(point)

        # PHASE 2: 접힌 상태에서 JOINT1을 HOME으로 회전
        # 시작점 중복을 피하려고 index=1부터 추가한다.
        for index in range(
            1,
            base_interval_count + 1,
        ):
            local_time_sec = min(
                index * controller_period_sec,
                base_duration_sec,
            )

            positions, velocities, accelerations = (
                interpolate_quintic(
                    start=folded_q,
                    goal=HOME_Q,
                    duration_sec=base_duration_sec,
                    local_time_sec=local_time_sec,
                )
            )

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations

            set_time_from_start(
                point,
                fold_duration_sec + local_time_sec,
            )

            trajectory.points.append(point)

        return trajectory

    def execute(
        self,
        fold_duration_sec: float,
        base_duration_sec: float,
    ) -> None:
        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Action server를 찾지 못했습니다: "
                f"{ARM_ACTION}"
            )

        trajectory = self.build_trajectory(
            fold_duration_sec=fold_duration_sec,
            base_duration_sec=base_duration_sec,
        )

        total_duration_sec = (
            fold_duration_sec
            + base_duration_sec
        )

        print(
            "[현재 동작] JOINT2~6을 안전 HOME 형상으로 접기"
        )

        print(
            "[현재 동작] 접힌 상태에서 JOINT1을 0°로 복귀"
        )

        print(
            f"[경로] 총 {len(trajectory.points)} points, "
            f"{total_duration_sec:.2f} sec"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = (
            self.action_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            raise RuntimeError(
                "HOME 복귀 목표 전송 시간이 초과됐습니다."
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "HOME 복귀 목표가 거부됐습니다."
            )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=total_duration_sec + 10.0,
        )

        if not result_future.done():
            raise RuntimeError(
                "HOME 복귀 완료 대기 시간이 초과됐습니다."
            )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                "HOME 복귀 결과를 받지 못했습니다."
            )

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise RuntimeError(
                "HOME 복귀 action이 실패했습니다. "
                f"status={wrapped_result.status}"
            )

        if wrapped_result.result.error_code != 0:
            raise RuntimeError(
                "Trajectory controller 오류: "
                f"error_code="
                f"{wrapped_result.result.error_code}, "
                f"error_string="
                f"{wrapped_result.result.error_string}"
            )

        deadline = time.monotonic() + 1.0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        final_q = self.current_arm_q()

        maximum_home_error = max(
            abs(current - target)
            for current, target in zip(
                final_q,
                HOME_Q,
            )
        )

        print(
            "[결과] 최대 HOME 오차: "
            f"{math.degrees(maximum_home_error):.3f}°"
        )

        if (
            maximum_home_error
            > FINAL_HOME_TOLERANCE_RAD
        ):
            raise RuntimeError(
                "최종 HOME 오차가 2°를 초과했습니다."
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "PLACE_RETREAT 종료점에서 "
            "안전하게 HOME으로 복귀합니다."
        )
    )

    parser.add_argument(
        "--fold-duration",
        type=float,
        default=DEFAULT_FOLD_DURATION_SEC,
        help="JOINT2~6을 접는 시간",
    )

    parser.add_argument(
        "--base-duration",
        type=float,
        default=DEFAULT_BASE_DURATION_SEC,
        help="접힌 상태에서 JOINT1을 복귀하는 시간",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    parser.add_argument(
        "--confirmation",
        type=str,
        default="",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    if (
        arguments.fold_duration <= 0.0
        or arguments.base_duration <= 0.0
    ):
        print(
            "동작 시간은 0보다 커야 합니다.",
            file=sys.stderr,
        )
        return 2

    total_duration = (
        arguments.fold_duration
        + arguments.base_duration
    )

    if not arguments.execute:
        print("CYLINDER RETURN HOME ONLY — DRY RUN")
        print(
            f"Fold duration : "
            f"{arguments.fold_duration:.2f} sec"
        )
        print(
            f"Base duration : "
            f"{arguments.base_duration:.2f} sec"
        )
        print(
            f"Total duration: "
            f"{total_duration:.2f} sec"
        )
        print()
        print(
            "경로:"
        )
        print(
            "PLACE_RETREAT"
        )
        print(
            "→ JOINT1 유지 + JOINT2~6 HOME 형상"
        )
        print(
            "→ JOINT1 0° 복귀"
        )
        print(
            "→ HOME"
        )
        return 0

    if (
        arguments.confirmation
        != CONFIRMATION_TOKEN
    ):
        print(
            "실행 확인 문자열이 올바르지 않습니다.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()

    node: Optional[ReturnHomeExecutor] = None

    try:
        node = ReturnHomeExecutor()

        print(
            "[시작 대기] PLACE_RETREAT 및 "
            "그리퍼 상태 확인"
        )

        if not node.wait_for_joint_state(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "필요한 joint state를 받지 못했습니다."
            )

        node.validate_start_state()

        node.execute(
            fold_duration_sec=arguments.fold_duration,
            base_duration_sec=arguments.base_duration,
        )

        print(
            "[종료] PLACE_RETREAT → HOME 복귀 PASS"
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
