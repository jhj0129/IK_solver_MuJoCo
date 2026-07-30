#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
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

FIELD_NAMES = {
    "neutral",
    "soft_lower",
    "soft_upper",
    "hard_lower",
    "hard_upper",
}


class WristLimitCollector(Node):
    def __init__(
        self,
        config_path: Path,
        action_name: str,
    ) -> None:
        super().__init__("wrist_limit_collector")

        self.config_path = config_path
        self.action_name = action_name

        with config_path.open("r", encoding="utf-8") as file:
            self.config = yaml.safe_load(file)

        self.current_positions: Dict[str, float] = {}
        self.state_lock = threading.Lock()

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
        with self.state_lock:
            for name, position in zip(
                message.name,
                message.position,
            ):
                if name in ARM_JOINTS:
                    self.current_positions[name] = float(position)

    def wait_for_joint_states(
        self,
        timeout: float = 10.0,
    ) -> bool:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            with self.state_lock:
                ready = all(
                    joint in self.current_positions
                    for joint in ARM_JOINTS
                )

            if ready:
                return True

            time.sleep(0.05)

        return False

    def get_current_positions(self) -> Dict[str, float]:
        with self.state_lock:
            return {
                joint: self.current_positions[joint]
                for joint in ARM_JOINTS
                if joint in self.current_positions
            }

    def show_positions(self) -> None:
        positions = self.get_current_positions()

        print()
        print("=" * 72)
        print("CURRENT JOINT POSITIONS")
        print("=" * 72)

        for joint in ARM_JOINTS:
            value = positions.get(joint)

            if value is None:
                print(f"{joint}: unavailable")
                continue

            print(
                f"{joint}: "
                f"{value:+.9f} rad  "
                f"{math.degrees(value):+.3f} deg"
            )

        print()

    def normalize_joint_name(self, token: str) -> str:
        token = token.strip().upper()

        aliases = {
            "J1": "JOINT1",
            "J2": "JOINT2",
            "J3": "JOINT3",
            "J4": "JOINT4",
            "J5": "JOINT5",
            "J6": "JOINT6",
        }

        return aliases.get(token, token)

    def get_configured_limit(
        self,
        joint: str,
        field: str,
    ) -> Optional[float]:
        value = self.config["limits"][joint].get(field)

        if value is None:
            return None

        return float(value)

    def allowable_range(
        self,
        joint: str,
    ) -> tuple[float, float]:
        joint_config = self.config["limits"][joint]

        mechanical_lower = float(
            joint_config["mechanical_lower"]
        )
        mechanical_upper = float(
            joint_config["mechanical_upper"]
        )

        hard_lower = self.get_configured_limit(
            joint,
            "hard_lower",
        )
        hard_upper = self.get_configured_limit(
            joint,
            "hard_upper",
        )

        lower = (
            mechanical_lower
            if hard_lower is None
            else max(mechanical_lower, hard_lower)
        )

        upper = (
            mechanical_upper
            if hard_upper is None
            else min(mechanical_upper, hard_upper)
        )

        return lower, upper

    def command_joint(
        self,
        joint: str,
        target: float,
        duration_seconds: float = 2.0,
    ) -> bool:
        if joint not in WRIST_JOINTS:
            print(
                "이 수집 도구에서는 "
                "JOINT4, JOINT5, JOINT6만 움직입니다."
            )
            return False

        current = self.get_current_positions()

        if not all(name in current for name in ARM_JOINTS):
            print("현재 관절 상태가 완전하지 않습니다.")
            return False

        lower, upper = self.allowable_range(joint)

        if target < lower or target > upper:
            print()
            print("명령 거부")
            print(
                f"{joint} target={target:+.6f} rad "
                f"({math.degrees(target):+.2f} deg)"
            )
            print(
                f"allowed=[{lower:+.6f}, {upper:+.6f}] rad"
            )
            return False

        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            print(
                f"Action server를 찾지 못했습니다: "
                f"{self.action_name}"
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()

        point.positions = [
            target if name == joint else current[name]
            for name in ARM_JOINTS
        ]

        point.velocities = [0.0] * len(ARM_JOINTS)

        whole_seconds = int(duration_seconds)
        nanoseconds = int(
            (duration_seconds - whole_seconds)
            * 1_000_000_000
        )

        point.time_from_start = Duration(
            sec=whole_seconds,
            nanosec=nanoseconds,
        )

        goal.trajectory.points = [point]

        print()
        print(
            f"Sending {joint}: "
            f"{current[joint]:+.6f} -> {target:+.6f} rad "
            f"({math.degrees(target):+.2f} deg)"
        )

        send_future = self.action_client.send_goal_async(
            goal
        )

        while not send_future.done():
            time.sleep(0.02)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            print("Trajectory goal이 거부됐습니다.")
            return False

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            time.sleep(0.02)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            print("Trajectory 결과를 받지 못했습니다.")
            return False

        result = wrapped_result.result

        if result.error_code != 0:
            print(
                "Trajectory 실패:",
                result.error_code,
                result.error_string,
            )
            return False

        time.sleep(0.2)

        print("Trajectory 완료")
        self.show_positions()

        return True

    def save_current(
        self,
        joint: str,
        field: str,
    ) -> bool:
        if joint not in WRIST_JOINTS:
            print("J4, J5, J6만 저장할 수 있습니다.")
            return False

        if field not in FIELD_NAMES:
            print(
                "저장 가능한 필드:",
                ", ".join(sorted(FIELD_NAMES)),
            )
            return False

        current = self.get_current_positions()

        if joint not in current:
            print(f"{joint} 현재값이 없습니다.")
            return False

        value = float(current[joint])

        self.config["limits"][joint][field] = value

        print(
            f"저장 예정: {joint}.{field} = "
            f"{value:+.9f} rad "
            f"({math.degrees(value):+.3f} deg)"
        )

        return True

    def validate_config(self) -> bool:
        print()
        print("=" * 72)
        print("CABLE LIMIT VALIDATION")
        print("=" * 72)

        overall_valid = True

        for joint in WRIST_JOINTS:
            values = self.config["limits"][joint]

            mechanical_lower = float(
                values["mechanical_lower"]
            )
            mechanical_upper = float(
                values["mechanical_upper"]
            )

            neutral = values.get("neutral")
            soft_lower = values.get("soft_lower")
            soft_upper = values.get("soft_upper")
            hard_lower = values.get("hard_lower")
            hard_upper = values.get("hard_upper")

            complete = all(
                value is not None
                for value in [
                    neutral,
                    soft_lower,
                    soft_upper,
                    hard_lower,
                    hard_upper,
                ]
            )

            if not complete:
                print(f"{joint}: INCOMPLETE")
                overall_valid = False
                continue

            neutral = float(neutral)
            soft_lower = float(soft_lower)
            soft_upper = float(soft_upper)
            hard_lower = float(hard_lower)
            hard_upper = float(hard_upper)

            valid = (
                mechanical_lower
                <= hard_lower
                <= soft_lower
                <= neutral
                <= soft_upper
                <= hard_upper
                <= mechanical_upper
            )

            status = "PASS" if valid else "FAIL"

            print()
            print(f"{joint}: {status}")
            print(
                f"  mechanical: "
                f"[{mechanical_lower:+.6f}, "
                f"{mechanical_upper:+.6f}]"
            )
            print(
                f"  hard      : "
                f"[{hard_lower:+.6f}, "
                f"{hard_upper:+.6f}]"
            )
            print(
                f"  soft      : "
                f"[{soft_lower:+.6f}, "
                f"{soft_upper:+.6f}]"
            )
            print(
                f"  neutral   : {neutral:+.6f}"
            )

            if not valid:
                overall_valid = False

        self.config["collection"]["complete"] = (
            overall_valid
        )

        print()
        print(
            "OVERALL:",
            "PASS" if overall_valid else "FAIL",
        )
        print()

        return overall_valid

    def write_config(self) -> None:
        with self.config_path.open(
            "w",
            encoding="utf-8",
        ) as file:
            yaml.safe_dump(
                self.config,
                file,
                sort_keys=False,
                allow_unicode=True,
            )

        print(
            "설정 저장 완료:",
            self.config_path,
        )


def print_help() -> None:
    print(
        """
사용 명령:

  show
      현재 J1~J6 각도 표시

  step J4 5
      J4를 현재 위치에서 +5도 이동

  step J6 -2
      J6를 현재 위치에서 -2도 이동

  set J5 30
      J5를 절대각 +30도로 이동

  save J4 neutral
  save J4 soft_lower
  save J4 soft_upper
  save J4 hard_lower
  save J4 hard_upper
      현재 관절각을 해당 값으로 임시 저장

  validate
      mechanical/hard/soft/neutral 순서 검사

  write
      cable_limits.yaml에 기록

  help
      명령 설명

  quit
      종료

주의:
  입력 각도 단위는 degree입니다.
  파일 저장 단위는 radian입니다.
"""
    )


def main() -> int:
    if len(sys.argv) >= 2:
        config_path = Path(sys.argv[1]).expanduser()
    else:
        config_path = (
            Path.home()
            / "IK_solver_MuJoCo"
            / "src/drok_arm_control/config/cable_limits.yaml"
        )

    action_name = (
        sys.argv[2]
        if len(sys.argv) >= 3
        else "/arm_controller/follow_joint_trajectory"
    )

    if not config_path.exists():
        print(
            "설정 파일이 없습니다:",
            config_path,
        )
        return 1

    rclpy.init()

    node = WristLimitCollector(
        config_path=config_path,
        action_name=action_name,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        print()
        print("=" * 72)
        print("DROK ARM WRIST LIMIT COLLECTOR")
        print("=" * 72)
        print("Config :", config_path)
        print("Action :", action_name)

        if not node.wait_for_joint_states():
            print(
                "/joint_states에서 JOINT1~6을 "
                "받지 못했습니다."
            )
            return 2

        if not node.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            print(
                "Action server가 없습니다:",
                action_name,
            )
            return 3

        node.show_positions()
        print_help()

        while rclpy.ok():
            try:
                command_line = input(
                    "wrist-limit> "
                ).strip()
            except EOFError:
                break

            if not command_line:
                continue

            tokens = command_line.split()
            command = tokens[0].lower()

            try:
                if command in {"quit", "exit", "q"}:
                    break

                if command == "help":
                    print_help()
                    continue

                if command == "show":
                    node.show_positions()
                    continue

                if command in {"step", "set"}:
                    if len(tokens) != 3:
                        print(
                            f"사용법: {command} J4 5"
                        )
                        continue

                    joint = node.normalize_joint_name(
                        tokens[1]
                    )

                    value_degrees = float(tokens[2])
                    value_radians = math.radians(
                        value_degrees
                    )

                    current = (
                        node.get_current_positions()
                    )

                    if joint not in current:
                        print(
                            f"{joint} 현재값이 없습니다."
                        )
                        continue

                    target = (
                        current[joint] + value_radians
                        if command == "step"
                        else value_radians
                    )

                    node.command_joint(
                        joint,
                        target,
                    )
                    continue

                if command == "save":
                    if len(tokens) != 3:
                        print(
                            "사용법: "
                            "save J4 soft_lower"
                        )
                        continue

                    joint = node.normalize_joint_name(
                        tokens[1]
                    )

                    field = tokens[2].lower()

                    node.save_current(
                        joint,
                        field,
                    )
                    continue

                if command == "validate":
                    node.validate_config()
                    continue

                if command == "write":
                    node.write_config()
                    continue

                print(
                    "알 수 없는 명령입니다. "
                    "'help'를 입력하세요."
                )

            except ValueError as error:
                print("숫자 변환 오류:", error)
            except Exception as error:
                print(
                    "명령 처리 오류:",
                    type(error).__name__,
                    error,
                )

    finally:
        try:
            node.write_config()
        except Exception as error:
            print("최종 저장 실패:", error)

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
