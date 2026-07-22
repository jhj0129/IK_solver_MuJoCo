#!/usr/bin/env python3

import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
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


def remove_visual_assets(xml_text: str) -> str:
    root = ET.fromstring(xml_text)

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)

    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag in {"asset", "geom"}:
                parent.remove(child)

    return ET.tostring(
        root,
        encoding="unicode",
    )


class GravityComparisonNode(Node):
    def __init__(self):
        super().__init__("gravity_model_effort_comparison")

        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.joint_state = None
        self.mjcf_text = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            joint_qos,
        )

        self.create_subscription(
            String,
            "/mujoco_robot_description",
            self.description_callback,
            description_qos,
        )

    def joint_state_callback(self, message):
        self.joint_state = message

    def description_callback(self, message):
        self.mjcf_text = message.data

    def ready(self):
        return (
            self.joint_state is not None
            and self.mjcf_text is not None
        )


def main():
    rclpy.init()
    node = GravityComparisonNode()

    try:
        print(
            "/joint_states와 "
            "/mujoco_robot_description을 기다리는 중..."
        )

        start_time = time.monotonic()

        while rclpy.ok() and not node.ready():
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if time.monotonic() - start_time > 10.0:
                raise RuntimeError(
                    "필요한 ROS 토픽을 10초 안에 받지 못했습니다."
                )

        message = node.joint_state

        position_map = {
            name: float(message.position[index])
            for index, name in enumerate(message.name)
            if index < len(message.position)
        }

        effort_map = {
            name: float(message.effort[index])
            for index, name in enumerate(message.name)
            if index < len(message.effort)
        }

        for name in JOINT_NAMES:
            if name not in position_map:
                raise RuntimeError(
                    f"/joint_states에서 {name}을 찾지 못했습니다."
                )

        clean_mjcf = remove_visual_assets(
            node.mjcf_text
        )

        model = mujoco.MjModel.from_xml_string(
            clean_mjcf
        )
        data = mujoco.MjData(model)

        data.qvel[:] = 0.0
        data.qacc[:] = 0.0

        joint_info = []

        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            if joint_id < 0:
                raise RuntimeError(
                    f"MuJoCo 모델에서 {name}을 찾지 못했습니다."
                )

            qpos_address = int(
                model.jnt_qposadr[joint_id]
            )
            dof_address = int(
                model.jnt_dofadr[joint_id]
            )

            position = position_map[name]
            data.qpos[qpos_address] = position

            actuator_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                name,
            )

            if actuator_id >= 0:
                data.ctrl[actuator_id] = position

            joint_info.append(
                (
                    name,
                    qpos_address,
                    dof_address,
                )
            )

        mujoco.mj_forward(
            model,
            data,
        )

        print()
        print("===== 중력 모델과 실제 effort 비교 =====")
        print()
        print(
            "bias는 중력과 속도 관련 항입니다. "
            "현재 qvel=0이므로 주로 중력 토크입니다."
        )
        print()

        for name, _, dof_address in joint_info:
            measured_effort = effort_map.get(
                name,
                float("nan"),
            )

            bias = float(
                data.qfrc_bias[dof_address]
            )
            passive = float(
                data.qfrc_passive[dof_address]
            )
            constraint = float(
                data.qfrc_constraint[dof_address]
            )
            friction_limit = float(
                model.dof_frictionloss[dof_address]
            )

            model_minus_measured = (
                bias - measured_effort
            )

            print(name)
            print(
                f"  q                    = "
                f"{position_map[name]:+.12f} rad"
            )
            print(
                f"  measured effort      = "
                f"{measured_effort:+.12f} Nm"
            )
            print(
                f"  model qfrc_bias      = "
                f"{bias:+.12f} Nm"
            )
            print(
                f"  model passive        = "
                f"{passive:+.12f} Nm"
            )
            print(
                f"  model constraint     = "
                f"{constraint:+.12f} Nm"
            )
            print(
                f"  frictionloss limit   = "
                f"{friction_limit:+.12f} Nm"
            )
            print(
                f"  bias - measured      = "
                f"{model_minus_measured:+.12f} Nm"
            )
            print()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
