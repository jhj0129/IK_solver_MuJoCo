#!/usr/bin/env python3

import math
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
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


class DescriptionReceiver(Node):
    def __init__(self):
        super().__init__(
            "gravity_compensated_target_calculator"
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.mjcf_text = None

        self.create_subscription(
            String,
            "/mujoco_robot_description",
            self.callback,
            qos,
        )

    def callback(self, message):
        self.mjcf_text = message.data


def parse_target(arguments):
    if len(arguments) != 6:
        raise RuntimeError(
            "사용법:\n"
            "  compute_gravity_compensated_target.py "
            "<q1> <q2> <q3> <q4> <q5> <q6>"
        )

    values = []

    for index, text in enumerate(arguments):
        value = float(text)

        if not math.isfinite(value):
            raise RuntimeError(
                f"q{index + 1} 값이 유한하지 않습니다."
            )

        values.append(value)

    return values


def main():
    target_positions = parse_target(
        sys.argv[1:]
    )

    rclpy.init()
    node = DescriptionReceiver()

    try:
        print(
            "/mujoco_robot_description을 "
            "기다리는 중..."
        )

        start_time = time.monotonic()

        while (
            rclpy.ok()
            and node.mjcf_text is None
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if time.monotonic() - start_time > 10.0:
                raise RuntimeError(
                    "MJCF를 10초 안에 받지 못했습니다."
                )

        clean_mjcf = remove_visual_assets(
            node.mjcf_text
        )

        model = mujoco.MjModel.from_xml_string(
            clean_mjcf
        )
        data = mujoco.MjData(model)

        joint_data = []

        for name, target in zip(
            JOINT_NAMES,
            target_positions,
        ):
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )

            actuator_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                name,
            )

            if joint_id < 0:
                raise RuntimeError(
                    f"Joint를 찾지 못했습니다: {name}"
                )

            if actuator_id < 0:
                raise RuntimeError(
                    f"Actuator를 찾지 못했습니다: {name}"
                )

            qpos_address = int(
                model.jnt_qposadr[joint_id]
            )

            dof_address = int(
                model.jnt_dofadr[joint_id]
            )

            data.qpos[qpos_address] = target

            joint_data.append(
                (
                    name,
                    target,
                    dof_address,
                    actuator_id,
                )
            )

        data.qvel[:] = 0.0
        data.qacc[:] = 0.0

        mujoco.mj_forward(
            model,
            data,
        )

        compensated_positions = []

        print()
        print(
            "===== 중력 보상 관절 명령 계산 ====="
        )
        print()

        for (
            name,
            target,
            dof_address,
            actuator_id,
        ) in joint_data:
            gravity_torque = float(
                data.qfrc_bias[dof_address]
            )

            kp = float(
                model.actuator_gainprm[
                    actuator_id,
                    0,
                ]
            )

            if kp <= 0.0:
                raise RuntimeError(
                    f"{name}의 kp가 유효하지 않습니다: "
                    f"{kp}"
                )

            position_offset = (
                gravity_torque / kp
            )

            compensated = (
                target + position_offset
            )

            if int(
                model.actuator_ctrllimited[
                    actuator_id
                ]
            ) != 0:
                lower = float(
                    model.actuator_ctrlrange[
                        actuator_id,
                        0,
                    ]
                )

                upper = float(
                    model.actuator_ctrlrange[
                        actuator_id,
                        1,
                    ]
                )

                compensated = min(
                    max(compensated, lower),
                    upper,
                )

            compensated_positions.append(
                compensated
            )

            print(name)
            print(
                f"  target            = "
                f"{target:+.12f} rad"
            )
            print(
                f"  gravity torque    = "
                f"{gravity_torque:+.12f} Nm"
            )
            print(
                f"  kp                = "
                f"{kp:.6f}"
            )
            print(
                f"  gravity offset    = "
                f"{position_offset:+.12f} rad"
            )
            print(
                f"  command           = "
                f"{compensated:+.12f} rad"
            )
            print()

        print(
            "Machine-readable compensated result"
        )

        print(
            "COMMAND_RESULT=" +
            ",".join(
                f"{value:.12f}"
                for value in compensated_positions
            )
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
