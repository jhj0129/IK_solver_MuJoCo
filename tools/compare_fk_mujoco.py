#!/usr/bin/env python3

import math
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

BASE_BODY_NAME = "ARM_BASE_LINK"
TCP_SITE_NAME = "gripper_tcp"

POSITION_TOLERANCE_M = 1.0e-6
ORIENTATION_TOLERANCE_RAD = 1.0e-6


class LiveModelReader(Node):
    def __init__(self):
        super().__init__("compare_fk_mujoco")

        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        model_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.joint_positions = {}
        self.mjcf_xml = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            joint_qos,
        )

        self.create_subscription(
            String,
            "/mujoco_robot_description",
            self.model_callback,
            model_qos,
        )

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = float(position)

    def model_callback(self, msg):
        self.mjcf_xml = msg.data

    def wait_for_data(self, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            has_joints = all(
                name in self.joint_positions
                for name in JOINT_NAMES
            )

            has_model = bool(self.mjcf_xml)

            if has_joints and has_model:
                return True

        return False


def remove_rendering_elements(xml_text):
    """
    기구학 검증에는 mesh와 geom이 필요하지 않다.

    live MJCF의 meshdir가 임시 디렉터리를 가리키므로,
    asset과 geom을 제거한 순수 기구학 모델을 생성한다.
    """

    root = ET.fromstring(xml_text)

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)

    asset = root.find("asset")
    if asset is not None:
        root.remove(asset)

    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom":
                parent.remove(child)

    return ET.tostring(
        root,
        encoding="unicode",
    )


def load_mujoco_transform(xml_text, joint_positions):
    kinematic_xml = remove_rendering_elements(xml_text)

    model = mujoco.MjModel.from_xml_string(
        kinematic_xml
    )
    data = mujoco.MjData(model)

    data.qpos[:] = model.qpos0

    for joint_name, position in zip(
        JOINT_NAMES,
        joint_positions,
    ):
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        if joint_id < 0:
            raise RuntimeError(
                f"MuJoCo joint를 찾지 못했습니다: "
                f"{joint_name}"
            )

        qpos_address = int(
            model.jnt_qposadr[joint_id]
        )

        data.qpos[qpos_address] = position

    mujoco.mj_forward(model, data)

    base_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        BASE_BODY_NAME,
    )

    site_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        TCP_SITE_NAME,
    )

    if base_id < 0:
        raise RuntimeError(
            f"MuJoCo body를 찾지 못했습니다: "
            f"{BASE_BODY_NAME}"
        )

    if site_id < 0:
        raise RuntimeError(
            f"MuJoCo site를 찾지 못했습니다: "
            f"{TCP_SITE_NAME}"
        )

    world_to_base_rotation = (
        data.xmat[base_id]
        .reshape(3, 3)
        .copy()
    )

    world_to_base_position = (
        data.xpos[base_id].copy()
    )

    world_to_site_rotation = (
        data.site_xmat[site_id]
        .reshape(3, 3)
        .copy()
    )

    world_to_site_position = (
        data.site_xpos[site_id].copy()
    )

    # T_base_tcp = inverse(T_world_base) * T_world_tcp
    base_to_site_rotation = (
        world_to_base_rotation.T
        @ world_to_site_rotation
    )

    base_to_site_position = (
        world_to_base_rotation.T
        @ (
            world_to_site_position
            - world_to_base_position
        )
    )

    transform = np.eye(4)
    transform[:3, :3] = base_to_site_rotation
    transform[:3, 3] = base_to_site_position

    return transform


def run_cpp_fk(joint_positions):
    geometry_path = (
        Path.home()
        / "IK_solver_MuJoCo"
        / "src"
        / "drok_arm_kinematics"
        / "config"
        / "robot_geometry.yaml"
    )

    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "test_fk",
        str(geometry_path),
    ]

    command.extend(
        f"{position:.17g}"
        for position in joint_positions
    )

    result = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )

    lines = result.stdout.splitlines()

    matrix_start = None

    for index, line in enumerate(lines):
        if line.strip() == "Transformation matrix":
            matrix_start = index + 1
            break

    if matrix_start is None:
        print(result.stdout)
        raise RuntimeError(
            "test_fk 출력에서 변환행렬을 찾지 못했습니다."
        )

    rows = []

    for line in lines[
        matrix_start:matrix_start + 4
    ]:
        values = [
            float(value)
            for value in line.split()
        ]

        if len(values) != 4:
            print(result.stdout)
            raise RuntimeError(
                "test_fk 변환행렬 형식이 예상과 다릅니다."
            )

        rows.append(values)

    return np.array(rows, dtype=float)


def rotation_error_angle(rotation_a, rotation_b):
    relative_rotation = (
        rotation_a.T
        @ rotation_b
    )

    cosine = (
        np.trace(relative_rotation) - 1.0
    ) / 2.0

    cosine = float(
        np.clip(cosine, -1.0, 1.0)
    )

    return math.acos(cosine)


def matrix_to_rpy(rotation):
    """
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """

    pitch = math.asin(
        float(
            np.clip(
                -rotation[2, 0],
                -1.0,
                1.0,
            )
        )
    )

    cos_pitch = math.cos(pitch)

    if abs(cos_pitch) > 1.0e-9:
        roll = math.atan2(
            rotation[2, 1],
            rotation[2, 2],
        )

        yaw = math.atan2(
            rotation[1, 0],
            rotation[0, 0],
        )
    else:
        roll = math.atan2(
            -rotation[1, 2],
            rotation[1, 1],
        )

        yaw = 0.0

    return np.array(
        [roll, pitch, yaw],
        dtype=float,
    )


def print_transform(title, transform):
    position = transform[:3, 3]
    rotation = transform[:3, :3]
    rpy = matrix_to_rpy(rotation)

    print("\n" + title)
    print("-" * len(title))

    for row in transform:
        print(
            " ".join(
                f"{value: .9f}"
                for value in row
            )
        )

    print("\nposition [m]")
    print(f"x = {position[0]: .9f}")
    print(f"y = {position[1]: .9f}")
    print(f"z = {position[2]: .9f}")

    print("\nRPY [rad]")
    print(f"roll  = {rpy[0]: .9f}")
    print(f"pitch = {rpy[1]: .9f}")
    print(f"yaw   = {rpy[2]: .9f}")


def main():
    rclpy.init()
    node = LiveModelReader()

    try:
        print(
            "/joint_states와 "
            "/mujoco_robot_description을 기다리는 중..."
        )

        if not node.wait_for_data():
            raise RuntimeError(
                "15초 안에 필요한 ROS 데이터를 "
                "받지 못했습니다."
            )

        joint_positions = [
            node.joint_positions[name]
            for name in JOINT_NAMES
        ]

        mjcf_xml = node.mjcf_xml

    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n===== 사용한 관절값 =====")

    for name, position in zip(
        JOINT_NAMES,
        joint_positions,
    ):
        print(
            f"{name}: "
            f"{position:+.12f} rad"
        )

    fk_transform = run_cpp_fk(
        joint_positions
    )

    mujoco_transform = load_mujoco_transform(
        mjcf_xml,
        joint_positions,
    )

    print_transform(
        "자체 C++ FK",
        fk_transform,
    )

    print_transform(
        "MuJoCo gripper_tcp site",
        mujoco_transform,
    )

    position_difference = (
        fk_transform[:3, 3]
        - mujoco_transform[:3, 3]
    )

    position_error = float(
        np.linalg.norm(position_difference)
    )

    orientation_error = rotation_error_angle(
        fk_transform[:3, :3],
        mujoco_transform[:3, :3],
    )

    matrix_max_error = float(
        np.max(
            np.abs(
                fk_transform
                - mujoco_transform
            )
        )
    )

    print("\n===== 비교 결과 =====")
    print(
        "dx = "
        f"{position_difference[0]:+.12e} m"
    )
    print(
        "dy = "
        f"{position_difference[1]:+.12e} m"
    )
    print(
        "dz = "
        f"{position_difference[2]:+.12e} m"
    )
    print(
        "위치 오차 norm = "
        f"{position_error:.12e} m"
    )
    print(
        "자세 오차 angle = "
        f"{orientation_error:.12e} rad"
    )
    print(
        "행렬 최대 절댓값 오차 = "
        f"{matrix_max_error:.12e}"
    )

    passed = (
        position_error
        <= POSITION_TOLERANCE_M
        and orientation_error
        <= ORIENTATION_TOLERANCE_RAD
    )

    print("\n===== 최종 판정 =====")

    if passed:
        print(
            "PASS: 자체 FK와 MuJoCo site pose가 "
            "허용오차 안에서 일치합니다."
        )
        return

    print(
        "FAIL: 자체 FK와 MuJoCo site pose가 "
        "허용오차를 초과했습니다."
    )

    raise SystemExit(1)


if __name__ == "__main__":
    main()
