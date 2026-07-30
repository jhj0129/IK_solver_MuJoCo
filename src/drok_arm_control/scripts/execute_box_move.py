#!/usr/bin/env python3

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ROOT = Path.home() / "IK_solver_MuJoCo"

ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/joint_states"
EXECUTION_STAGE_TOPIC = "/pick_place/execution_stage"

ARM_JOINTS = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

GRIPPER_JOINTS = [
    "JOINT7",
    "GRIPPER_RIGHT_JOINT",
]

HOME_Q = [
    -0.000001628,
    +0.297361544,
    +0.296742637,
    -0.000030712,
    +0.000061231,
    +0.000102331,
]

EXPECTED_SEGMENTS = [
    "PICK_APPROACH",
    "PICK_LIFT",
    "TRANSFER",
    "PLACE_DESCEND",
    "PLACE_RETREAT",
]

CONFIRMATION_TOKEN = "EXECUTE_MUJOCO_BOX_MOVE"

HOME_TOLERANCE_RAD = 0.060
BLOCK_START_TOLERANCE_RAD = 0.060
ARM_GOAL_TOLERANCE_RAD = math.radians(2.0)
GRIPPER_GOAL_TOLERANCE_M = 0.003
CONTROLLER_RATE_HZ = 100.0


def load_yaml(path: Path) -> Dict[str, Any]:
    document = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    if not isinstance(document, dict):
        raise RuntimeError(
            f"YAML root가 dictionary가 아닙니다: {path}"
        )

    return document


def set_time(
    point: JointTrajectoryPoint,
    seconds: float,
) -> None:
    whole = int(math.floor(seconds))
    nanoseconds = int(
        round((seconds - whole) * 1_000_000_000)
    )

    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000

    point.time_from_start.sec = whole
    point.time_from_start.nanosec = nanoseconds


def max_difference(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    return max(
        abs(a - b)
        for a, b in zip(first, second)
    )


def quintic_terms(
    tau: float,
    duration: float,
) -> tuple[float, float, float]:
    tau = min(1.0, max(0.0, tau))

    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    position_scale = (
        10.0 * tau3
        - 15.0 * tau4
        + 6.0 * tau5
    )

    velocity_scale = (
        30.0 * tau2
        - 60.0 * tau3
        + 30.0 * tau4
    ) / duration

    acceleration_scale = (
        60.0 * tau
        - 180.0 * tau2
        + 120.0 * tau3
    ) / (duration * duration)

    return (
        position_scale,
        velocity_scale,
        acceleration_scale,
    )


def calculate_quintic_duration(
    start: Sequence[float],
    goal: Sequence[float],
    joint_names: Sequence[str],
    limits: Dict[str, Dict[str, Any]],
    speed_scale: float,
    minimum_duration: float,
) -> float:
    required_duration = minimum_duration

    for index, name in enumerate(joint_names):
        delta = abs(goal[index] - start[index])
        joint_limit = limits[name]

        velocity = (
            float(joint_limit["used_velocity_limit"])
            * speed_scale
        )
        acceleration = (
            float(joint_limit["used_acceleration_limit"])
            * speed_scale**2
        )
        jerk = (
            float(joint_limit["used_jerk_limit"])
            * speed_scale**3
        )

        velocity_duration = (
            1.875 * delta / velocity
            if velocity > 0.0
            else 0.0
        )

        acceleration_duration = (
            math.sqrt(5.773503 * delta / acceleration)
            if acceleration > 0.0
            else 0.0
        )

        jerk_duration = (
            (60.0 * delta / jerk) ** (1.0 / 3.0)
            if jerk > 0.0
            else 0.0
        )

        required_duration = max(
            required_duration,
            velocity_duration,
            acceleration_duration,
            jerk_duration,
        )

    return required_duration * 1.15


def create_quintic_trajectory(
    start: Sequence[float],
    goal: Sequence[float],
    joint_names: Sequence[str],
    duration: float,
) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    interval_count = max(
        2,
        int(math.ceil(duration * CONTROLLER_RATE_HZ)),
    )

    delta = [
        goal_value - start_value
        for start_value, goal_value in zip(start, goal)
    ]

    for index in range(interval_count + 1):
        local_time = min(
            duration,
            index / CONTROLLER_RATE_HZ,
        )

        position_scale, velocity_scale, acceleration_scale = (
            quintic_terms(
                local_time / duration,
                duration,
            )
        )

        point = JointTrajectoryPoint()
        point.positions = [
            start_value + position_scale * difference
            for start_value, difference in zip(start, delta)
        ]
        point.velocities = [
            velocity_scale * difference
            for difference in delta
        ]
        point.accelerations = [
            acceleration_scale * difference
            for difference in delta
        ]

        set_time(point, local_time)
        trajectory.points.append(point)

    return trajectory


def create_scaled_block_trajectory(
    block: Dict[str, Any],
    joint_names: Sequence[str],
    speed_scale: float,
) -> JointTrajectory:
    points = block.get("points")

    if not isinstance(points, list) or len(points) < 2:
        raise RuntimeError(
            f"{block.get('source_segment')} block point가 부족합니다."
        )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    for source in points:
        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
            for value in source["positions"]
        ]

        point.velocities = [
            float(value) * speed_scale
            for value in source.get(
                "velocities",
                [0.0] * len(joint_names),
            )
        ]

        point.accelerations = [
            float(value) * speed_scale**2
            for value in source.get(
                "accelerations",
                [0.0] * len(joint_names),
            )
        ]

        set_time(
            point,
            float(source["time_from_start"])
            / speed_scale,
        )

        trajectory.points.append(point)

    return trajectory


def trajectory_duration(
    trajectory: JointTrajectory,
) -> float:
    point = trajectory.points[-1]
    return (
        float(point.time_from_start.sec)
        + float(point.time_from_start.nanosec)
        / 1_000_000_000.0
    )


class BoxMoveExecutor(Node):
    def __init__(self) -> None:
        super().__init__("box_move_executor")

        self.positions: Dict[str, float] = {}

        self.subscription = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            20,
        )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION,
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            GRIPPER_ACTION,
        )

        self.execution_stage_publisher = self.create_publisher(
            String,
            EXECUTION_STAGE_TOPIC,
            10,
        )

    def publish_execution_stage(
        self,
        stage: str,
    ) -> None:
        message = String()
        message.data = stage
        self.execution_stage_publisher.publish(message)
        print(f"[VR STAGE] {stage}")

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        for name, position in zip(
            message.name,
            message.position,
        ):
            self.positions[name] = float(position)

    def wait_for_state(
        self,
        names: Sequence[str],
        timeout: float,
    ) -> List[float]:
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if all(name in self.positions for name in names):
                return [
                    self.positions[name]
                    for name in names
                ]

        raise RuntimeError(
            "필요한 joint state를 받지 못했습니다: "
            + ", ".join(names)
        )

    def current(
        self,
        names: Sequence[str],
    ) -> List[float]:
        return [
            self.positions[name]
            for name in names
        ]

    def wait_for_servers(self) -> None:
        if not self.arm_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Arm action server를 찾지 못했습니다: "
                f"{ARM_ACTION}"
            )

        if not self.gripper_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Gripper action server를 찾지 못했습니다: "
                f"{GRIPPER_ACTION}"
            )

    def send_and_wait(
        self,
        client: ActionClient,
        trajectory: JointTrajectory,
        label: str,
        result_margin: float = 10.0,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            raise RuntimeError(
                f"{label}: goal 전송 시간 초과"
            )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f"{label}: goal rejected"
            )

        result_future = goal_handle.get_result_async()

        timeout = (
            trajectory_duration(trajectory)
            + result_margin
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=timeout,
        )

        if not result_future.done():
            raise RuntimeError(
                f"{label}: action result timeout"
            )

        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError(
                f"{label}: action result가 없습니다."
            )

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label}: action status={wrapped.status}"
            )

        if wrapped.result.error_code != 0:
            raise RuntimeError(
                f"{label}: controller error_code="
                f"{wrapped.result.error_code}, "
                f"error_string={wrapped.result.error_string}"
            )

        settle_deadline = time.monotonic() + 0.5

        while (
            rclpy.ok()
            and time.monotonic() < settle_deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

    def send_arm_trajectory(
        self,
        trajectory: JointTrajectory,
        label: str,
        expected_goal: Sequence[float],
    ) -> None:
        self.send_and_wait(
            self.arm_client,
            trajectory,
            label,
        )

        measured = self.current(ARM_JOINTS)
        error = max_difference(
            measured,
            expected_goal,
        )

        print(
            f"[확인] {label} 종료 오차: "
            f"{math.degrees(error):.3f}°"
        )

        if error > ARM_GOAL_TOLERANCE_RAD:
            raise RuntimeError(
                f"{label}: 최종 관절 오차가 "
                "2°를 초과했습니다."
            )

    def send_gripper(
        self,
        target: Sequence[float],
        duration: float,
        label: str,
    ) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(GRIPPER_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [
            float(value)
            for value in target
        ]
        point.velocities = [0.0, 0.0]

        set_time(point, duration)
        trajectory.points = [point]

        self.send_and_wait(
            self.gripper_client,
            trajectory,
            label,
        )

        measured = self.current(GRIPPER_JOINTS)
        error = max_difference(
            measured,
            target,
        )

        print(
            f"[확인] {label} 종료 오차: "
            f"{error * 1000.0:.3f} mm"
        )

        if error > GRIPPER_GOAL_TOLERANCE_M:
            raise RuntimeError(
                f"{label}: 그리퍼 오차가 "
                "3 mm를 초과했습니다."
            )


def validate_path(
    timed: Dict[str, Any],
) -> List[Dict[str, Any]]:
    joint_names = timed.get("joint_names")

    if joint_names != ARM_JOINTS:
        raise RuntimeError(
            f"timed path 관절 이름이 다릅니다: "
            f"{joint_names}"
        )

    blocks = timed.get("blocks")

    if not isinstance(blocks, list) or len(blocks) != 5:
        raise RuntimeError(
            "timed path의 motion block 개수가 "
            "5개가 아닙니다."
        )

    for block, expected in zip(
        blocks,
        EXPECTED_SEGMENTS,
    ):
        actual = block.get("source_segment")

        if actual != expected:
            raise RuntimeError(
                f"segment 불일치: {actual} != {expected}"
            )

        points = block.get("points")

        if not isinstance(points, list) or len(points) < 2:
            raise RuntimeError(
                f"{actual} point가 부족합니다."
            )

    return blocks


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "생성된 timed path를 이용해 BOX 간 "
            "MuJoCo Pick-and-Place와 HOME 복귀를 실행합니다."
        )
    )

    parser.add_argument(
        "--from-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument(
        "--to-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument(
        "--timed-path",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.5,
        help="0보다 크고 1 이하. 기본값 0.5",
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

    if arguments.from_box == arguments.to_box:
        print(
            "출발 상자와 도착 상자가 같습니다.",
            file=sys.stderr,
        )
        return 2

    if not (
        0.0 < arguments.speed_scale <= 1.0
    ):
        print(
            "speed-scale은 0보다 크고 1 이하여야 합니다.",
            file=sys.stderr,
        )
        return 2

    path_directory = (
        ROOT
        / "generated_box_paths"
        / (
            f"box_{arguments.from_box}"
            f"_to_{arguments.to_box}"
        )
    )

    timed_path = (
        arguments.timed_path.expanduser().resolve()
        if arguments.timed_path is not None
        else path_directory / "timed_joint_path.yaml"
    )

    report_path = (
        timed_path.parent
        / "generation_report.yaml"
    )

    gripper_config_path = (
        ROOT
        / "src/drok_arm_control/config"
        / "cylinder_gripper_grasp.yaml"
    )

    for path in (
        timed_path,
        report_path,
        gripper_config_path,
    ):
        if not path.exists():
            print(
                f"필요한 파일이 없습니다: {path}",
                file=sys.stderr,
            )
            return 2

    timed = load_yaml(timed_path)
    report = load_yaml(report_path)
    gripper_config = load_yaml(
        gripper_config_path
    )

    if (
        int(report["from_box"])
        != arguments.from_box
        or int(report["to_box"])
        != arguments.to_box
    ):
        print(
            "generation_report와 요청한 BOX 번호가 "
            "일치하지 않습니다.",
            file=sys.stderr,
        )
        return 2

    blocks = validate_path(timed)
    limits = timed["limits"]

    open_target = [
        float(value)
        for value in gripper_config["positions"]["open"]
    ]

    close_target = [
        float(value)
        for value in gripper_config[
            "positions"
        ]["initial_grasp"]
    ]

    partial_release = [
        +0.0400,
        -0.0400,
    ]

    first_q = [
        float(value)
        for value in blocks[0]["points"][0]["positions"]
    ]

    retreat_q = [
        float(value)
        for value in blocks[-1]["points"][-1]["positions"]
    ]

    total_nominal_duration = sum(
        float(block["duration"])
        for block in blocks
    )

    print("=" * 72)
    print("MUJOCO GENERIC BOX MOVE")
    print("=" * 72)
    print(
        f"Move              : BOX {arguments.from_box}"
        f" -> BOX {arguments.to_box}"
    )
    print(f"Timed path        : {timed_path}")
    print(
        f"Selected candidate: "
        f"{report['selected_candidate']['candidate_index']}"
    )
    print(
        f"Speed scale       : "
        f"{arguments.speed_scale:.3f}"
    )
    print(
        f"Arm path duration : "
        f"{total_nominal_duration / arguments.speed_scale:.2f} sec"
    )
    print(
        "Final action      : PLACE_RETREAT -> HOME"
    )
    print()

    if not arguments.execute:
        print("DRY RUN RESULT: PASS")
        print()
        print("실행 전 확인:")
        print(
            f"- 원기둥이 실제로 BOX "
            f"{arguments.from_box} 위에 있어야 합니다."
        )
        print("- 로봇은 HOME 자세여야 합니다.")
        print("- MuJoCo 전용 실행입니다.")
        print()
        print(
            "실행 옵션:"
        )
        print(
            "--execute "
            f"--confirmation {CONFIRMATION_TOKEN}"
        )
        return 0

    if arguments.confirmation != CONFIRMATION_TOKEN:
        print(
            "실행 확인 문자열이 올바르지 않습니다.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node: Optional[BoxMoveExecutor] = None
    current_stage = "INITIALIZATION"

    try:
        node = BoxMoveExecutor()

        print("[시작 대기] joint state 및 action server 확인")

        node.wait_for_state(
            ARM_JOINTS + GRIPPER_JOINTS,
            timeout=5.0,
        )

        node.wait_for_servers()
        node.publish_execution_stage("STARTED")

        current_arm = node.current(ARM_JOINTS)
        home_error = max_difference(
            current_arm,
            HOME_Q,
        )

        print(
            "[상태 확인] HOME 오차: "
            f"{math.degrees(home_error):.3f}°"
        )

        if home_error > HOME_TOLERANCE_RAD:
            raise RuntimeError(
                "현재 팔이 HOME 허용범위 "
                "0.060 rad 밖에 있습니다."
            )

        current_stage = "GRIPPER_OPEN"
        node.publish_execution_stage(current_stage)
        print("[현재 동작] 그리퍼 초기 개방")
        node.send_gripper(
            open_target,
            duration=2.0,
            label=current_stage,
        )

        current_stage = "ENTRY_TO_PICK_PREGRASP"
        node.publish_execution_stage(current_stage)
        print(
            f"[현재 동작] BOX {arguments.from_box} "
            "파지 전 자세로 진입"
        )

        current_arm = node.current(ARM_JOINTS)

        entry_duration = calculate_quintic_duration(
            current_arm,
            first_q,
            ARM_JOINTS,
            limits,
            arguments.speed_scale,
            minimum_duration=4.0,
        )

        entry = create_quintic_trajectory(
            current_arm,
            first_q,
            ARM_JOINTS,
            entry_duration,
        )

        print(
            f"[경로] ENTRY "
            f"{len(entry.points)} points, "
            f"{entry_duration:.2f} sec"
        )

        node.send_arm_trajectory(
            entry,
            current_stage,
            first_q,
        )

        for block_index, block in enumerate(
            blocks,
            start=1,
        ):
            segment = str(
                block["source_segment"]
            )

            block_first_q = [
                float(value)
                for value in block["points"][0]["positions"]
            ]

            current_arm = node.current(ARM_JOINTS)
            start_error = max_difference(
                current_arm,
                block_first_q,
            )

            if (
                start_error
                > BLOCK_START_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    f"{segment} 시작 오차가 "
                    "0.060 rad를 초과했습니다: "
                    f"{start_error:.9f} rad"
                )

            current_stage = segment
            node.publish_execution_stage(segment)

            display_names = {
                "PICK_APPROACH": (
                    f"BOX {arguments.from_box} "
                    "원기둥 파지 위치로 접근"
                ),
                "PICK_LIFT": (
                    "원기둥 들어 올리기"
                ),
                "TRANSFER": (
                    f"BOX {arguments.to_box}로 운반"
                ),
                "PLACE_DESCEND": (
                    f"BOX {arguments.to_box}에 내려놓기"
                ),
                "PLACE_RETREAT": (
                    "원기둥에서 후퇴"
                ),
            }

            print(
                "[현재 동작] "
                + display_names[segment]
            )

            trajectory = (
                create_scaled_block_trajectory(
                    block,
                    ARM_JOINTS,
                    arguments.speed_scale,
                )
            )

            goal_q = [
                float(value)
                for value in block["points"][-1]["positions"]
            ]

            node.send_arm_trajectory(
                trajectory,
                current_stage,
                goal_q,
            )

            if block_index == 1:
                current_stage = "CYLINDER_GRASP"
                print("[현재 동작] 원기둥 파지")
                node.send_gripper(
                    close_target,
                    duration=float(
                        gripper_config["motion"][
                            "close_duration_sec"
                        ]
                    ),
                    label=current_stage,
                )
                node.publish_execution_stage("GRASPED")

            elif block_index == 4:
                current_stage = "PARTIAL_RELEASE"
                print("[현재 동작] 그리퍼 1차 해제")
                node.send_gripper(
                    partial_release,
                    duration=2.0,
                    label=current_stage,
                )

                print("[현재 동작] 원기둥 안정화 대기")
                time.sleep(1.0)

                current_stage = "GRIPPER_FULL_OPEN"
                print("[현재 동작] 그리퍼 완전 개방")
                node.send_gripper(
                    open_target,
                    duration=2.0,
                    label=current_stage,
                )
                node.publish_execution_stage("RELEASED")

        current_arm = node.current(ARM_JOINTS)
        retreat_error = max_difference(
            current_arm,
            retreat_q,
        )

        if retreat_error > ARM_GOAL_TOLERANCE_RAD:
            raise RuntimeError(
                "PLACE_RETREAT 종료 자세 검증 실패"
            )

        current_stage = "RETURN_HOME_FOLD"
        node.publish_execution_stage("RETURN_HOME")
        print(
            "[현재 동작] JOINT1 유지 후 "
            "JOINT2~6 HOME 형상으로 접기"
        )

        folded_q = [
            current_arm[0],
            HOME_Q[1],
            HOME_Q[2],
            HOME_Q[3],
            HOME_Q[4],
            HOME_Q[5],
        ]

        fold_duration = calculate_quintic_duration(
            current_arm,
            folded_q,
            ARM_JOINTS,
            limits,
            arguments.speed_scale,
            minimum_duration=8.0,
        )

        fold_trajectory = create_quintic_trajectory(
            current_arm,
            folded_q,
            ARM_JOINTS,
            fold_duration,
        )

        node.send_arm_trajectory(
            fold_trajectory,
            current_stage,
            folded_q,
        )

        current_stage = "RETURN_HOME_BASE"
        print(
            "[현재 동작] 접힌 상태에서 "
            "JOINT1을 HOME으로 복귀"
        )

        current_arm = node.current(ARM_JOINTS)

        base_duration = calculate_quintic_duration(
            current_arm,
            HOME_Q,
            ARM_JOINTS,
            limits,
            arguments.speed_scale,
            minimum_duration=4.0,
        )

        base_trajectory = create_quintic_trajectory(
            current_arm,
            HOME_Q,
            ARM_JOINTS,
            base_duration,
        )

        node.send_arm_trajectory(
            base_trajectory,
            current_stage,
            HOME_Q,
        )

        final_home_error = max_difference(
            node.current(ARM_JOINTS),
            HOME_Q,
        )

        print()
        print("=" * 72)
        print("BOX MOVE RESULT: PASS")
        print("=" * 72)
        print(
            f"Completed move : BOX "
            f"{arguments.from_box} -> BOX "
            f"{arguments.to_box}"
        )
        print(
            "Final HOME error: "
            f"{math.degrees(final_home_error):.3f}°"
        )
        print(
            f"Cylinder state : BOX "
            f"{arguments.to_box}"
        )

        node.publish_execution_stage("COMPLETE")
        time.sleep(0.20)
        return 0

    except Exception as exception:
        if node is not None:
            node.publish_execution_stage("FAILED")
            time.sleep(0.20)
        print(
            f"[실패 단계] {current_stage}",
            file=sys.stderr,
        )
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
