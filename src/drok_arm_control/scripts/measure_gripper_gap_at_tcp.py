#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

ARM_JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

LEFT_JOINT_NAME = "JOINT7"
RIGHT_JOINT_NAME = "GRIPPER_RIGHT_JOINT"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--model",
        required=True,
        help="로봇이 포함된 runtime MJCF",
    )

    parser.add_argument(
        "--path",
        default=str(
            ROOT
            / "cylinder_best_full_cartesian_ik_path.yaml"
        ),
    )

    parser.add_argument(
        "--candidates",
        default=str(
            ROOT
            / "cylinder_grasp_candidates.yaml"
        ),
    )

    parser.add_argument(
        "--grasp-index",
        type=int,
        default=27,
    )

    parser.add_argument(
        "--current-left",
        type=float,
        default=0.020032111,
    )

    parser.add_argument(
        "--current-right",
        type=float,
        default=-0.019967891,
    )

    parser.add_argument(
        "--cylinder-diameter",
        type=float,
        default=0.060,
    )

    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict):
        raise RuntimeError(
            f"잘못된 YAML 문서입니다: {path}"
        )

    return document


def require_joint_id(
    model: mujoco.MjModel,
    joint_name: str,
) -> int:
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    if joint_id < 0:
        raise RuntimeError(
            f"{joint_name}을 찾지 못했습니다."
        )

    return int(joint_id)


def object_name(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    object_id: int,
    fallback: str,
) -> str:
    name = mujoco.mj_id2name(
        model,
        object_type,
        object_id,
    )

    return name if name else fallback


def subtree_body_ids(
    model: mujoco.MjModel,
    root_body_id: int,
) -> set[int]:
    result: set[int] = set()

    for body_id in range(model.nbody):
        current = body_id

        while current > 0:
            if current == root_body_id:
                result.add(body_id)
                break

            current = int(
                model.body_parentid[current]
            )

    result.add(root_body_id)

    return result


def collect_geoms(
    model: mujoco.MjModel,
    body_ids: set[int],
) -> list[int]:
    return [
        geom_id
        for geom_id in range(model.ngeom)
        if int(
            model.geom_bodyid[geom_id]
        ) in body_ids
    ]


def set_joint_qpos(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_name: str,
    value: float,
) -> None:
    joint_id = require_joint_id(
        model,
        joint_name,
    )

    address = int(
        model.jnt_qposadr[joint_id]
    )

    data.qpos[address] = value


def find_grasp_record(
    path_document: dict[str, Any],
    global_index: int,
) -> dict[str, Any]:
    records = path_document.get(
        "path",
        [],
    )

    matches = [
        record
        for record in records
        if int(
            record["global_index"]
        ) == global_index
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"global_index={global_index} record를 "
            "정확히 하나 찾지 못했습니다."
        )

    return matches[0]


def find_candidate(
    candidate_document: dict[str, Any],
    candidate_index: int,
) -> dict[str, Any]:
    matches = [
        candidate
        for candidate
        in candidate_document.get(
            "candidates",
            [],
        )
        if int(
            candidate["index"]
        ) == candidate_index
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"candidate {candidate_index}를 "
            "정확히 하나 찾지 못했습니다."
        )

    return matches[0]


def ray_distance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    origin: np.ndarray,
    direction: np.ndarray,
    geom_group: np.ndarray,
) -> tuple[float, int]:
    geom_id = np.array(
        [-1],
        dtype=np.int32,
    )

    distance = float(
        mujoco.mj_ray(
            model,
            data,
            origin,
            direction,
            geom_group,
            1,
            -1,
            geom_id,
        )
    )

    return distance, int(geom_id[0])


def measure_gap(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm_q: list[float],
    tcp_position: np.ndarray,
    closing_axis: np.ndarray,
    left_q: float,
    right_q: float,
    geom_group: np.ndarray,
) -> dict[str, Any]:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    for joint_name, value in zip(
        ARM_JOINT_NAMES,
        arm_q,
    ):
        set_joint_qpos(
            model,
            data,
            joint_name,
            value,
        )

    set_joint_qpos(
        model,
        data,
        LEFT_JOINT_NAME,
        left_q,
    )

    set_joint_qpos(
        model,
        data,
        RIGHT_JOINT_NAME,
        right_q,
    )

    mujoco.mj_forward(
        model,
        data,
    )

    positive_distance, positive_geom = (
        ray_distance(
            model,
            data,
            tcp_position,
            closing_axis,
            geom_group,
        )
    )

    negative_distance, negative_geom = (
        ray_distance(
            model,
            data,
            tcp_position,
            -closing_axis,
            geom_group,
        )
    )

    if positive_distance < 0.0:
        raise RuntimeError(
            "TCP +closing 방향에서 "
            "손가락 파지면을 찾지 못했습니다."
        )

    if negative_distance < 0.0:
        raise RuntimeError(
            "TCP -closing 방향에서 "
            "손가락 파지면을 찾지 못했습니다."
        )

    return {
        "left_q": left_q,
        "right_q": right_q,
        "positive_distance": (
            positive_distance
        ),
        "negative_distance": (
            negative_distance
        ),
        "gap": (
            positive_distance
            + negative_distance
        ),
        "positive_geom": positive_geom,
        "negative_geom": negative_geom,
    }


def print_result(
    model: mujoco.MjModel,
    label: str,
    result: dict[str, Any],
) -> None:
    positive_geom = int(
        result["positive_geom"]
    )

    negative_geom = int(
        result["negative_geom"]
    )

    positive_body = int(
        model.geom_bodyid[
            positive_geom
        ]
    )

    negative_body = int(
        model.geom_bodyid[
            negative_geom
        ]
    )

    print()
    print(f"[{label}]")

    print(
        "  q                      : "
        f"left={result['left_q']:+.9f} m, "
        f"right={result['right_q']:+.9f} m"
    )

    print(
        "  TCP -> +closing finger : "
        f"{result['positive_distance'] * 1000.0:.3f} mm"
    )

    print(
        "  TCP -> -closing finger : "
        f"{result['negative_distance'] * 1000.0:.3f} mm"
    )

    print(
        "  actual finger gap      : "
        f"{result['gap'] * 1000.0:.3f} mm"
    )

    print(
        "  +closing hit           : "
        f"{object_name(model, mujoco.mjtObj.mjOBJ_GEOM, positive_geom, f'geom_{positive_geom}')} "
        f"/ "
        f"{object_name(model, mujoco.mjtObj.mjOBJ_BODY, positive_body, f'body_{positive_body}')}"
    )

    print(
        "  -closing hit           : "
        f"{object_name(model, mujoco.mjtObj.mjOBJ_GEOM, negative_geom, f'geom_{negative_geom}')} "
        f"/ "
        f"{object_name(model, mujoco.mjtObj.mjOBJ_BODY, negative_body, f'body_{negative_body}')}"
    )


def main() -> int:
    arguments = parse_arguments()

    model_path = Path(
        arguments.model
    ).expanduser().resolve()

    path_document = load_yaml(
        Path(arguments.path).expanduser().resolve()
    )

    candidate_document = load_yaml(
        Path(
            arguments.candidates
        ).expanduser().resolve()
    )

    grasp_record = find_grasp_record(
        path_document,
        arguments.grasp_index,
    )

    candidate_index = int(
        path_document["candidate_index"]
    )

    candidate = find_candidate(
        candidate_document,
        candidate_index,
    )

    arm_q = [
        float(value)
        for value in grasp_record[
            "joint_positions"
        ]
    ]

    tcp_position = np.asarray(
        grasp_record["position"],
        dtype=float,
    )

    rotation = np.asarray(
        candidate[
            "rotation_world_tcp"
        ],
        dtype=float,
    )

    if rotation.shape != (3, 3):
        raise RuntimeError(
            "rotation_world_tcp가 3x3이 아닙니다."
        )

    # TCP local +Y가 그리퍼 닫힘 축이다.
    closing_axis = rotation[:, 1]
    closing_axis /= np.linalg.norm(
        closing_axis
    )

    model = mujoco.MjModel.from_xml_path(
        str(model_path)
    )

    data = mujoco.MjData(model)

    left_joint_id = require_joint_id(
        model,
        LEFT_JOINT_NAME,
    )

    right_joint_id = require_joint_id(
        model,
        RIGHT_JOINT_NAME,
    )

    left_body_id = int(
        model.jnt_bodyid[left_joint_id]
    )

    right_body_id = int(
        model.jnt_bodyid[right_joint_id]
    )

    gripper_body_ids = (
        subtree_body_ids(
            model,
            left_body_id,
        )
        | subtree_body_ids(
            model,
            right_body_id,
        )
    )

    gripper_geoms = collect_geoms(
        model,
        gripper_body_ids,
    )

    if not gripper_geoms:
        raise RuntimeError(
            "그리퍼 geom을 찾지 못했습니다."
        )

    # ray가 손가락 geom만 보도록 임시 그룹 5에 배치한다.
    original_groups = np.asarray(
        model.geom_group,
        dtype=np.int32,
    ).copy()

    model.geom_group[:] = 0

    for geom_id in gripper_geoms:
        model.geom_group[geom_id] = 5

    geom_group = np.zeros(
        6,
        dtype=np.uint8,
    )

    geom_group[5] = 1

    print("=" * 88)
    print("GRIPPER GAP AT CYLINDER TCP")
    print("=" * 88)

    print(f"Model             : {model_path}")
    print(f"Candidate         : {candidate_index}")
    print(f"Grasp index       : {arguments.grasp_index}")

    print(
        "TCP position      : "
        + ", ".join(
            f"{value:+.9f}"
            for value in tcp_position
        )
    )

    print(
        "Closing axis      : "
        + ", ".join(
            f"{value:+.9f}"
            for value in closing_axis
        )
    )

    print(
        "Cylinder diameter : "
        f"{arguments.cylinder_diameter * 1000.0:.3f} mm"
    )

    tests = [
        (
            "FULLY_OPEN",
            0.0,
            0.0,
        ),
        (
            "CURRENT",
            float(
                arguments.current_left
            ),
            float(
                arguments.current_right
            ),
        ),
        (
            "SYMMETRIC_20MM",
            0.020,
            -0.020,
        ),
        (
            "URDF_MAX_CLOSE",
            0.040,
            -0.040,
        ),
    ]

    results: list[
        tuple[str, dict[str, Any]]
    ] = []

    for label, left_q, right_q in tests:
        result = measure_gap(
            model,
            data,
            arm_q,
            tcp_position,
            closing_axis,
            left_q,
            right_q,
            geom_group,
        )

        results.append(
            (label, result)
        )

        print_result(
            model,
            label,
            result,
        )

    symmetric_results = [
        result
        for label, result in results
        if label in {
            "FULLY_OPEN",
            "SYMMETRIC_20MM",
            "URDF_MAX_CLOSE",
        }
    ]

    q_values = np.asarray(
        [
            abs(
                result["left_q"]
            )
            for result in symmetric_results
        ],
        dtype=float,
    )

    gap_values = np.asarray(
        [
            result["gap"]
            for result in symmetric_results
        ],
        dtype=float,
    )

    slope, intercept = np.polyfit(
        q_values,
        gap_values,
        1,
    )

    target_gap = float(
        arguments.cylinder_diameter
    )

    touch_q = (
        target_gap - intercept
    ) / slope

    print()
    print("=" * 88)
    print("60 mm CYLINDER CONTACT")
    print("=" * 88)

    print(
        "Measured gap model : "
        f"gap={intercept:.9f} "
        f"{slope:+.9f}*q"
    )

    print(
        "Calculated touch q  : "
        f"left={touch_q:+.9f} m, "
        f"right={-touch_q:+.9f} m"
    )

    print(
        "Within URDF limits  : "
        + (
            "YES"
            if 0.0 <= touch_q <= 0.040
            else "NO"
        )
    )

    maximum_close_gap = next(
        result["gap"]
        for label, result in results
        if label == "URDF_MAX_CLOSE"
    )

    print(
        "Gap at q=±0.040 m   : "
        f"{maximum_close_gap * 1000.0:.3f} mm"
    )

    model.geom_group[:] = (
        original_groups
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
