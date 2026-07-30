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

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


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

FIELDS = [
    "neutral",
    "soft_lower",
    "soft_upper",
    "hard_lower",
    "hard_upper",
]


class PassiveRecorder(Node):
    def __init__(self, config_path: Path) -> None:
        super().__init__("wrist_limit_passive_recorder")

        self.config_path = config_path
        self.config = self.load_config()
        self.positions: Dict[str, float] = {}
        self.lock = threading.Lock()
        self.dirty = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )

    def load_config(self):
        with self.config_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            return yaml.safe_load(file)

    def reload_config(self) -> None:
        self.config = self.load_config()
        self.dirty = False
        print("YAML 파일의 저장된 값을 다시 불러왔습니다.")

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

    def show_current(self) -> None:
        current = self.get_positions()

        print()
        print("=" * 72)
        print("CURRENT JOINT POSITIONS")
        print("=" * 72)

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

    def show_saved(self) -> None:
        print()
        print("=" * 72)
        print("SAVED CABLE LIMIT VALUES")
        print("=" * 72)

        for joint in WRIST_JOINTS:
            print()
            print(joint)

            data = self.config["limits"][joint]

            for field in FIELDS:
                value = data.get(field)

                if value is None:
                    print(f"  {field:12s}: null")
                else:
                    value = float(value)

                    print(
                        f"  {field:12s}: "
                        f"{value:+.9f} rad   "
                        f"{math.degrees(value):+8.3f} deg"
                    )

        print()

    def save_current(
        self,
        joint: str,
        field: str,
    ) -> None:
        if joint not in WRIST_JOINTS:
            raise ValueError(
                "J4, J5, J6만 기록할 수 있습니다."
            )

        if field not in FIELDS:
            raise ValueError(
                "필드는 neutral, soft_lower, soft_upper, "
                "hard_lower, hard_upper 중 하나여야 합니다."
            )

        current = self.get_positions()

        if joint not in current:
            raise RuntimeError(
                f"{joint} 현재 각도를 받지 못했습니다."
            )

        value = float(current[joint])

        self.config["limits"][joint][field] = value
        self.config["collection"]["complete"] = False
        self.dirty = True

        print()
        print(
            f"임시 기록: {joint}.{field} = "
            f"{value:+.9f} rad "
            f"({math.degrees(value):+.3f} deg)"
        )
        print("아직 YAML에는 저장되지 않았습니다.")
        print()

    def clear_field(
        self,
        joint: str,
        field: str,
    ) -> None:
        if joint not in WRIST_JOINTS:
            raise ValueError(
                "J4, J5, J6만 사용할 수 있습니다."
            )

        if field not in FIELDS:
            raise ValueError(
                "올바르지 않은 필드입니다."
            )

        self.config["limits"][joint][field] = None
        self.config["collection"]["complete"] = False
        self.dirty = True

        print(f"{joint}.{field} 값을 null로 변경했습니다.")

    def clear_joint(self, joint: str) -> None:
        if joint not in WRIST_JOINTS:
            raise ValueError(
                "J4, J5, J6만 사용할 수 있습니다."
            )

        for field in FIELDS:
            self.config["limits"][joint][field] = None

        self.config["collection"]["complete"] = False
        self.dirty = True

        print(f"{joint}의 모든 기록값을 null로 변경했습니다.")

    def validate(self) -> bool:
        print()
        print("=" * 72)
        print("CABLE LIMIT VALIDATION")
        print("=" * 72)

        overall = True

        for joint in WRIST_JOINTS:
            data = self.config["limits"][joint]

            missing = [
                field
                for field in FIELDS
                if data.get(field) is None
            ]

            if missing:
                print(
                    f"{joint}: INCOMPLETE "
                    f"({', '.join(missing)})"
                )
                overall = False
                continue

            mechanical_lower = float(
                data["mechanical_lower"]
            )
            mechanical_upper = float(
                data["mechanical_upper"]
            )

            neutral = float(data["neutral"])
            soft_lower = float(data["soft_lower"])
            soft_upper = float(data["soft_upper"])
            hard_lower = float(data["hard_lower"])
            hard_upper = float(data["hard_upper"])

            valid = (
                mechanical_lower
                <= hard_lower
                <= soft_lower
                <= neutral
                <= soft_upper
                <= hard_upper
                <= mechanical_upper
            )

            print(
                f"{joint}: "
                f"{'PASS' if valid else 'FAIL'}"
            )

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
                overall = False

        self.config["collection"]["complete"] = overall
        self.dirty = True

        print()
        print(
            "OVERALL:",
            "PASS" if overall else "FAIL",
        )
        print()

        return overall

    def write(self) -> None:
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

        self.dirty = False

        print()
        print("YAML 저장 완료:")
        print(self.config_path)
        print()


def print_help() -> None:
    print(
        """
사용 명령:

  show
      현재 JOINT1~6 각도 표시

  saved
      현재 임시 기록값 표시

  save J4 neutral
  save J4 hard_lower
  save J4 soft_lower
  save J4 soft_upper
  save J4 hard_upper
      현재 관절각을 해당 항목에 임시 기록

  clear J4 hard_lower
      잘못 기록한 항목 하나를 null로 변경

  clear-joint J4
      해당 관절의 모든 기록값을 null로 변경

  reload
      저장하지 않은 변경을 버리고 YAML 다시 읽기

  validate
      제한값 순서 검사

  write
      임시 기록값을 YAML 파일에 저장

  quit
      프로그램 종료

이 프로그램은 관절에 명령을 보내지 않습니다.
MuJoCo에서 직접 관절을 움직인 후 현재 각도만 기록합니다.
"""
    )


def main() -> int:
    default_path = (
        Path.home()
        / "IK_solver_MuJoCo"
        / "src/drok_arm_control/config/cable_limits.yaml"
    )

    config_path = (
        Path(sys.argv[1]).expanduser()
        if len(sys.argv) >= 2
        else default_path
    )

    if not config_path.exists():
        print("설정 파일을 찾지 못했습니다:")
        print(config_path)
        return 1

    rclpy.init()

    node = PassiveRecorder(config_path)
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
        print("=" * 72)
        print("DROK ARM PASSIVE WRIST LIMIT RECORDER")
        print("=" * 72)
        print("Config:", config_path)
        print("관절 제어 명령: 비활성화")

        if not node.wait_for_joint_states():
            print(
                "/joint_states에서 JOINT1~6을 "
                "받지 못했습니다."
            )
            return 2

        node.show_current()
        print_help()

        while rclpy.ok():
            try:
                line = input(
                    "wrist-record> "
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
                    node.show_current()

                elif command == "saved":
                    node.show_saved()

                elif command == "save":
                    if len(tokens) != 3:
                        print(
                            "사용법: save J4 neutral"
                        )
                        continue

                    joint = node.normalize_joint(
                        tokens[1]
                    )

                    field = tokens[2].lower()

                    node.save_current(
                        joint,
                        field,
                    )

                elif command == "clear":
                    if len(tokens) != 3:
                        print(
                            "사용법: clear J4 hard_lower"
                        )
                        continue

                    joint = node.normalize_joint(
                        tokens[1]
                    )

                    field = tokens[2].lower()

                    node.clear_field(
                        joint,
                        field,
                    )

                elif command == "clear-joint":
                    if len(tokens) != 2:
                        print(
                            "사용법: clear-joint J4"
                        )
                        continue

                    joint = node.normalize_joint(
                        tokens[1]
                    )

                    node.clear_joint(joint)

                elif command == "reload":
                    node.reload_config()

                elif command == "validate":
                    node.validate()

                elif command == "write":
                    node.write()

                else:
                    print(
                        "알 수 없는 명령입니다. "
                        "'help'를 입력하세요."
                    )

            except Exception as error:
                print(
                    "명령 오류:",
                    type(error).__name__,
                    error,
                )

    finally:
        if node.dirty:
            print()
            print(
                "저장하지 않은 변경사항은 "
                "YAML에 기록되지 않습니다."
            )

        stop_event.set()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)
        node.destroy_node()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
