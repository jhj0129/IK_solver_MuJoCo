#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict

import rclpy
import yaml

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

WRIST_JOINTS = [
    "JOINT4",
    "JOINT5",
    "JOINT6",
]


class WristManualJogger(Node):
    def __init__(
        self,
        config_path: Path,
        action_name: str,
    ) -> None:
        super().__init__("wrist_manual_jogger")

        self.config_path = config_path
        self.action_name = action_name

        with config_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            self.config = yaml.safe_load(file)

        self.positions: Dict[str, float] = {}
        self.lock = threading.Lock()

        # 기본 이동 속도
        self.speed_deg_per_second = 10.0

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
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
        with self.lock:
            for name, position in zip(
                message.name,
                message.position,
            ):
                if name in ARM_JOINTS:
                    self.positions[name] = float(position)

    def get_positions(self) -> Dict[str, float]:
        with self.lock:
            return dict(self.positions)

    def wait_for_joint_states(
        self,
        timeout_seconds: float = 10.0,
    ) -> bool:
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            current = self.get_positions()

            if all(
                joint in current
                for joint in ARM_JOINTS
            ):
                return True

            time.sleep(0.05)

        return False

    @staticmethod
    def normalize_joint(token: str) -> str:
        token = token.strip().upper()

        aliases = {
            "J4": "JOINT4",
            "J5": "JOINT5",
            "J6": "JOINT6",
        }

        return aliases.get(token, token)

    def mechanical_limits(
        self,
        joint: str,
    ) -> tuple[float, float]:
        data = self.config["limits"][joint]

        return (
            float(data["mechanical_lower"]),
            float(data["mechanical_upper"]),
        )

    def neutral_position(
        self,
        joint: str,
    ) -> float:
        value = self.config["limits"][joint].get(
            "neutral"
        )

        if value is None:
            return 0.0

        return float(value)

    def show_positions(self) -> None:
        current = self.get_positions()

        print()
        print("=" * 76)
        print("CURRENT ARM JOINT POSITIONS")
        print("=" * 76)

        for joint in ARM_JOINTS:
            value = current.get(joint)

            if value is None:
                print(f"{joint}: unavailable")
                continue

            print(
                f"{joint}: "
                f"{value:+.9f} rad   "
                f"{math.degrees(value):+8.3f} deg"
            )

        print()

    def show_limits(self) -> None:
        print()
        print("=" * 76)
        print("WRIST MECHANICAL LIMITS")
        print("=" * 76)

        for joint in WRIST_JOINTS:
            lower, upper = self.mechanical_limits(
                joint
            )

            neutral = self.neutral_position(joint)

            print()
            print(joint)

            print(
                f"  lower  : {lower:+.9f} rad   "
                f"{math.degrees(lower):+8.3f} deg"
            )

            print(
                f"  neutral: {neutral:+.9f} rad   "
                f"{math.degrees(neutral):+8.3f} deg"
            )

            print(
                f"  upper  : {upper:+.9f} rad   "
                f"{math.degrees(upper):+8.3f} deg"
            )

        print()
        print(
            "soft/hard cable limit은 이동 검사에 "
            "사용하지 않습니다."
        )
        print()

    def calculate_duration(
        self,
        start_positions: Dict[str, float],
        target_positions: Dict[str, float],
    ) -> float:
        maximum_delta = max(
            abs(
                target_positions[joint]
                - start_positions[joint]
            )
            for joint in ARM_JOINTS
        )

        speed_rad_per_second = math.radians(
            self.speed_deg_per_second
        )

        calculated = (
            maximum_delta / speed_rad_per_second
            if speed_rad_per_second > 1.0e-9
            else 2.0
        )

        return max(1.0, calculated)

    def validate_targets(
        self,
        targets: Dict[str, float],
    ) -> bool:
        for joint in WRIST_JOINTS:
            lower, upper = self.mechanical_limits(
                joint
            )

            value = targets[joint]

            if value < lower or value > upper:
                print()
                print("명령 거부: 기계적 관절 제한 초과")
                print(
                    f"{joint} target = "
                    f"{value:+.9f} rad "
                    f"({math.degrees(value):+.3f} deg)"
                )
                print(
                    f"allowed = "
                    f"[{lower:+.9f}, {upper:+.9f}] rad"
                )
                print()

                return False

        return True

    def send_targets(
        self,
        targets: Dict[str, float],
    ) -> bool:
        current = self.get_positions()

        if not all(
            joint in current
            for joint in ARM_JOINTS
        ):
            print("현재 J1~J6 값을 모두 받지 못했습니다.")
            return False

        if not self.validate_targets(targets):
            return False

        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            print(
                "Trajectory action server가 없습니다:"
            )
            print(self.action_name)
            return False

        duration_seconds = self.calculate_duration(
            current,
            targets,
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()

        point.positions = [
            targets[joint]
            for joint in ARM_JOINTS
        ]

        point.velocities = [0.0] * len(ARM_JOINTS)

        whole_seconds = int(duration_seconds)
        nanoseconds = int(
            round(
                (duration_seconds - whole_seconds)
                * 1_000_000_000
            )
        )

        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        point.time_from_start = Duration(
            sec=whole_seconds,
            nanosec=nanoseconds,
        )

        goal.trajectory.points = [point]

        print()
        print("Trajectory 전송")

        for joint in ARM_JOINTS:
            delta = targets[joint] - current[joint]

            print(
                f"  {joint}: "
                f"{math.degrees(current[joint]):+8.3f}° "
                f"-> {math.degrees(targets[joint]):+8.3f}° "
                f"(delta {math.degrees(delta):+8.3f}°)"
            )

        print(
            f"  duration: {duration_seconds:.3f} s"
        )
        print(
            f"  speed reference: "
            f"{self.speed_deg_per_second:.3f} deg/s"
        )
        print()

        send_future = self.action_client.send_goal_async(
            goal
        )

        send_deadline = time.monotonic() + 10.0

        while (
            not send_future.done()
            and time.monotonic() < send_deadline
        ):
            time.sleep(0.02)

        if not send_future.done():
            print("Goal 전송 시간이 초과됐습니다.")
            return False

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            print("Trajectory goal이 거부됐습니다.")
            return False

        result_future = goal_handle.get_result_async()

        result_deadline = (
            time.monotonic()
            + duration_seconds
            + 10.0
        )

        while (
            not result_future.done()
            and time.monotonic() < result_deadline
        ):
            time.sleep(0.02)

        if not result_future.done():
            print("Trajectory 실행 시간이 초과됐습니다.")
            return False

        wrapped_result = result_future.result()

        if wrapped_result is None:
            print("Trajectory 결과가 없습니다.")
            return False

        result = wrapped_result.result

        if result.error_code != 0:
            print()
            print("Trajectory 실행 실패")
            print("error_code:", result.error_code)
            print("error_string:", result.error_string)
            print()

            return False

        time.sleep(0.3)

        print("Trajectory 완료")
        self.show_positions()

        return True

    def step_joint(
        self,
        joint: str,
        delta_degrees: float,
    ) -> bool:
        if joint not in WRIST_JOINTS:
            print("J4, J5, J6만 조그할 수 있습니다.")
            return False

        current = self.get_positions()

        if joint not in current:
            print(f"{joint} 현재값이 없습니다.")
            return False

        targets = {
            name: current[name]
            for name in ARM_JOINTS
        }

        targets[joint] = (
            current[joint]
            + math.radians(delta_degrees)
        )

        return self.send_targets(targets)

    def set_joint(
        self,
        joint: str,
        target_degrees: float,
    ) -> bool:
        if joint not in WRIST_JOINTS:
            print("J4, J5, J6만 설정할 수 있습니다.")
            return False

        current = self.get_positions()

        targets = {
            name: current[name]
            for name in ARM_JOINTS
        }

        targets[joint] = math.radians(
            target_degrees
        )

        return self.send_targets(targets)

    def neutral_wrist(self) -> bool:
        current = self.get_positions()

        targets = {
            name: current[name]
            for name in ARM_JOINTS
        }

        for joint in WRIST_JOINTS:
            targets[joint] = self.neutral_position(
                joint
            )

        return self.send_targets(targets)


def print_help() -> None:
    print(
        """
사용 명령:

  show
      현재 J1~J6 각도 표시

  limits
      J4~J6 기계적 제한 및 중립각 표시

  step J4 5
      현재 위치에서 J4를 +5도 이동

  step J4 -5
      현재 위치에서 J4를 -5도 이동

  set J5 30
      J5를 절대각 +30도로 이동

  neutral-wrist
      J4·J5·J6를 YAML의 neutral 값으로 복귀
      neutral이 null이면 0도로 복귀

  speed 10
      기준 이동 속도를 10 deg/s로 변경

  help
      도움말 표시

  quit
      종료

중요:
  cable soft/hard limit은 사용하지 않습니다.
  mechanical_lower/upper만 검사합니다.
  각도 입력 단위는 degree입니다.
"""
    )


def main() -> int:
    default_config = (
        Path.home()
        / "IK_solver_MuJoCo"
        / "src/drok_arm_control/config/cable_limits.yaml"
    )

    config_path = (
        Path(sys.argv[1]).expanduser()
        if len(sys.argv) >= 2
        else default_config
    )

    action_name = (
        sys.argv[2]
        if len(sys.argv) >= 3
        else "/arm_controller/follow_joint_trajectory"
    )

    if not config_path.exists():
        print("설정 파일이 없습니다:")
        print(config_path)
        return 1

    rclpy.init()

    node = WristManualJogger(
        config_path=config_path,
        action_name=action_name,
    )

    stop_event = threading.Event()

    def ros_spin() -> None:
        while (
            rclpy.ok()
            and not stop_event.is_set()
        ):
            try:
                rclpy.spin_once(
                    node,
                    timeout_sec=0.1,
                )
            except ExternalShutdownException:
                break
            except Exception as error:
                if rclpy.ok():
                    print(
                        "ROS 처리 오류:",
                        type(error).__name__,
                        error,
                    )
                break

    spin_thread = threading.Thread(
        target=ros_spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        print()
        print("=" * 76)
        print("DROK ARM WRIST MANUAL JOGGER")
        print("=" * 76)
        print("Config:", config_path)
        print("Action:", action_name)

        if not node.wait_for_joint_states():
            print(
                "/joint_states에서 JOINT1~6을 "
                "받지 못했습니다."
            )
            return 2

        if not node.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            print("Action server가 없습니다:")
            print(action_name)
            return 3

        node.show_positions()
        node.show_limits()
        print_help()

        while rclpy.ok():
            try:
                line = input(
                    "wrist-jog> "
                ).strip()
            except KeyboardInterrupt:
                print()
                print("Ctrl+C 종료")
                break
            except EOFError:
                break

            if not line:
                continue

            tokens = line.split()
            command = tokens[0].lower()

            try:
                if command in {
                    "quit",
                    "exit",
                    "q",
                }:
                    break

                if command == "help":
                    print_help()

                elif command == "show":
                    node.show_positions()

                elif command == "limits":
                    node.show_limits()

                elif command == "step":
                    if len(tokens) != 3:
                        print("사용법: step J4 5")
                        continue

                    joint = node.normalize_joint(
                        tokens[1]
                    )

                    delta_degrees = float(
                        tokens[2]
                    )

                    node.step_joint(
                        joint,
                        delta_degrees,
                    )

                elif command == "set":
                    if len(tokens) != 3:
                        print("사용법: set J5 30")
                        continue

                    joint = node.normalize_joint(
                        tokens[1]
                    )

                    target_degrees = float(
                        tokens[2]
                    )

                    node.set_joint(
                        joint,
                        target_degrees,
                    )

                elif command in {
                    "neutral-wrist",
                    "home-wrist",
                }:
                    node.neutral_wrist()

                elif command == "speed":
                    if len(tokens) != 2:
                        print("사용법: speed 10")
                        continue

                    speed = float(tokens[1])

                    if speed <= 0.0:
                        print(
                            "속도는 0보다 커야 합니다."
                        )
                        continue

                    node.speed_deg_per_second = speed

                    print(
                        "기준 속도:",
                        f"{speed:.3f} deg/s",
                    )

                else:
                    print(
                        "알 수 없는 명령입니다. "
                        "'help'를 입력하세요."
                    )

            except ValueError as error:
                print("숫자 입력 오류:", error)
            except Exception as error:
                print(
                    "명령 오류:",
                    type(error).__name__,
                    error,
                )

    finally:
        stop_event.set()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)
        node.destroy_node()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
