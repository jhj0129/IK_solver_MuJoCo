#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime
import json
import math
import signal
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import mujoco
import mujoco.viewer


JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
    "JOINT7",
    "GRIPPER_RIGHT_JOINT",
]

ARM_JOINT_NAMES = JOINT_NAMES[:6]

HOME_VALUES = {
    "JOINT1": -0.000001628,
    "JOINT2": 0.297361544,
    "JOINT3": 0.296742637,
    "JOINT4": -0.000030712,
    "JOINT5": 0.000061231,
    "JOINT6": 0.000102331,
    "JOINT7": 0.0,
    "GRIPPER_RIGHT_JOINT": 0.0,
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--xml",
        required=True,
        help="생성된 MuJoCo XML 경로",
    )

    parser.add_argument(
        "--label",
        default="PICK_GRASP_SAMPLE",
    )

    parser.add_argument(
        "--output",
        default=str(
            Path.home()
            / "IK_solver_MuJoCo"
            / "manual_mujoco_pose_samples.json"
        ),
    )

    return parser.parse_args()


def object_id(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    name: str,
) -> Optional[int]:
    identifier = mujoco.mj_name2id(
        model,
        object_type,
        name,
    )

    if identifier < 0:
        return None

    return int(identifier)


def set_joint_position(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
    value: float,
) -> None:
    joint_id = object_id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        name,
    )

    if joint_id is None:
        return

    joint_type = model.jnt_type[joint_id]

    if joint_type not in (
        mujoco.mjtJoint.mjJNT_HINGE,
        mujoco.mjtJoint.mjJNT_SLIDE,
    ):
        return

    qpos_address = int(
        model.jnt_qposadr[joint_id]
    )

    data.qpos[qpos_address] = value


def set_actuator_control(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
    value: float,
) -> None:
    actuator_id = object_id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        name,
    )

    if actuator_id is None:
        return

    data.ctrl[actuator_id] = value


def read_joint_position(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
) -> Optional[float]:
    joint_id = object_id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        name,
    )

    if joint_id is None:
        return None

    joint_type = model.jnt_type[joint_id]

    if joint_type not in (
        mujoco.mjtJoint.mjJNT_HINGE,
        mujoco.mjtJoint.mjJNT_SLIDE,
    ):
        return None

    qpos_address = int(
        model.jnt_qposadr[joint_id]
    )

    return float(data.qpos[qpos_address])


def read_body_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
) -> Optional[Dict[str, Any]]:
    body_id = object_id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_name,
    )

    if body_id is None:
        return None

    position = [
        float(value)
        for value in data.xpos[body_id]
    ]

    rotation_matrix = [
        [
            float(
                data.xmat[body_id][
                    row * 3 + column
                ]
            )
            for column in range(3)
        ]
        for row in range(3)
    ]

    quaternion = [
        float(value)
        for value in data.xquat[body_id]
    ]

    return {
        "position_xyz_m": position,
        "quaternion_wxyz": quaternion,
        "rotation_matrix": rotation_matrix,
    }


def append_sample(
    output_path: Path,
    sample: Dict[str, Any],
) -> None:
    document: Dict[str, Any] = {
        "version": 1,
        "samples": [],
    }

    if output_path.exists():
        try:
            loaded = json.loads(
                output_path.read_text(
                    encoding="utf-8"
                )
            )

            if isinstance(loaded, dict):
                document = loaded
        except Exception:
            pass

    document.setdefault("samples", [])
    document["samples"].append(sample)

    output_path.write_text(
        json.dumps(
            document,
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def print_joint_values(
    joint_positions: Dict[str, float],
) -> None:
    print()
    print("JOINT POSITIONS [rad]")

    for name in JOINT_NAMES:
        if name not in joint_positions:
            continue

        print(
            f"  {name:22s} "
            f"{joint_positions[name]:+.12f}"
        )

    print()
    print("JOINT POSITIONS [deg]")

    for name in ARM_JOINT_NAMES:
        if name not in joint_positions:
            continue

        print(
            f"  {name:22s} "
            f"{math.degrees(joint_positions[name]):+.6f}°"
        )


def main() -> int:
    arguments = parse_arguments()

    xml_path = Path(
        arguments.xml
    ).expanduser().resolve()

    output_path = Path(
        arguments.output
    ).expanduser().resolve()

    if not xml_path.exists():
        print("MuJoCo XML 파일이 없습니다:")
        print(xml_path)
        return 1

    model = mujoco.MjModel.from_xml_path(
        str(xml_path)
    )

    data = mujoco.MjData(model)

    # 기존 프로젝트 HOME 자세로 시작한다.
    for name, value in HOME_VALUES.items():
        set_joint_position(
            model,
            data,
            name,
            value,
        )

        set_actuator_control(
            model,
            data,
            name,
            value,
        )

    mujoco.mj_forward(model, data)

    stop_requested = False

    def request_stop(
        signum,
        frame,
    ) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(
        signal.SIGINT,
        request_stop,
    )

    signal.signal(
        signal.SIGTERM,
        request_stop,
    )

    print("=" * 92)
    print("DROK ARM MANUAL MUJOCO POSE EDITOR")
    print("=" * 92)

    print("ROS control        : 실행 안 함")
    print("Arm controller     : 실행 안 함")
    print("Gripper controller : 실행 안 함")
    print("Control sliders    : 수동 조작 가능")
    print()

    print(
        "MuJoCo Control 패널에서 JOINT1~6을 "
        "직접 움직이세요."
    )

    print(
        "자세가 완성되면 MuJoCo 창을 닫거나 "
        "터미널에서 Ctrl+C를 누르세요."
    )

    timestep = float(model.opt.timestep)

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
        ) as viewer:
            while (
                viewer.is_running()
                and not stop_requested
            ):
                loop_start = time.perf_counter()

                # ros2_control 없이 MuJoCo actuator slider만 사용한다.
                mujoco.mj_step(
                    model,
                    data,
                )

                viewer.sync()

                elapsed = (
                    time.perf_counter()
                    - loop_start
                )

                sleep_time = timestep - elapsed

                if sleep_time > 0.0:
                    time.sleep(sleep_time)

    finally:
        mujoco.mj_forward(
            model,
            data,
        )

        joint_positions: Dict[str, float] = {}

        for name in JOINT_NAMES:
            value = read_joint_position(
                model,
                data,
                name,
            )

            if value is not None:
                joint_positions[name] = value

        sample: Dict[str, Any] = {
            "label": arguments.label,
            "recorded_at": (
                datetime.datetime.now(
                    datetime.timezone.utc
                ).astimezone().isoformat()
            ),
            "source": (
                "manual_mujoco_without_ros2_control"
            ),
            "xml_path": str(xml_path),
            "joint_positions_rad": (
                joint_positions
            ),
            "joint_positions_deg": {
                name: math.degrees(value)
                for name, value
                in joint_positions.items()
                if name in ARM_JOINT_NAMES
            },
            "gripper_tcp": read_body_pose(
                model,
                data,
                "gripper_tcp",
            ),
            "gripper_center": read_body_pose(
                model,
                data,
                "gripper_center",
            ),
            "pickup_cube": read_body_pose(
                model,
                data,
                "pickup_cube",
            ),
        }

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        append_sample(
            output_path,
            sample,
        )

        print()
        print("=" * 92)
        print("MANUAL POSE SAVED")
        print("=" * 92)

        print("Label:")
        print(arguments.label)

        print_joint_values(
            joint_positions
        )

        print()
        print("TCP POSE")

        if sample["gripper_tcp"] is None:
            print(
                "  gripper_tcp body를 "
                "찾지 못했습니다."
            )
        else:
            print(
                json.dumps(
                    sample["gripper_tcp"],
                    ensure_ascii=False,
                    indent=2,
                )
            )

        print()
        print("저장 파일:")
        print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
