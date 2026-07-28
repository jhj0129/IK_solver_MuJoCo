#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path.home() / "IK_solver_MuJoCo"

DEFAULT_MODEL = (
    ROOT
    / "src/drok_arm_mujoco/model/scene.xml"
)

LEFT_JOINT_NAME = "JOINT7"
RIGHT_JOINT_NAME = "GRIPPER_RIGHT_JOINT"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--model",
        default=str(DEFAULT_MODEL),
    )

    parser.add_argument(
        "--current-left",
        type=float,
        default=0.019967627,
    )

    parser.add_argument(
        "--current-right",
        type=float,
        default=-0.019967954,
    )

    parser.add_argument(
        "--cylinder-diameter",
        type=float,
        default=0.060,
    )

    return parser.parse_args()


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

    return name if name is not None else fallback


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
            f"MuJoCo 모델에서 {joint_name}을 "
            "찾지 못했습니다."
        )

    joint_type = int(
        model.jnt_type[joint_id]
    )

    if joint_type != int(
        mujoco.mjtJoint.mjJNT_SLIDE
    ):
        raise RuntimeError(
            f"{joint_name}은 slide joint가 아닙니다."
        )

    return int(joint_id)


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
    collision_geoms = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(
            model.geom_bodyid[geom_id]
        ) in body_ids
        and (
            int(
                model.geom_contype[geom_id]
            ) != 0
            or int(
                model.geom_conaffinity[geom_id]
            ) != 0
        )
    ]

    if collision_geoms:
        return collision_geoms

    # collision geom이 따로 없다면 해당 body의
    # 모든 geom을 fallback으로 사용한다.
    return [
        geom_id
        for geom_id in range(model.ngeom)
        if int(
            model.geom_bodyid[geom_id]
        ) in body_ids
    ]


def set_joint_position(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_id: int,
    position: float,
) -> None:
    qpos_address = int(
        model.jnt_qposadr[joint_id]
    )

    data.qpos[qpos_address] = position


def geom_name(
    model: mujoco.MjModel,
    geom_id: int,
) -> str:
    return object_name(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        geom_id,
        f"geom_{geom_id}",
    )


def minimum_geom_distance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    left_geoms: list[int],
    right_geoms: list[int],
) -> tuple[
    float,
    int,
    int,
    np.ndarray,
]:
    minimum_distance = float("inf")
    minimum_left_geom = -1
    minimum_right_geom = -1
    minimum_fromto = np.zeros(
        6,
        dtype=np.float64,
    )

    for left_geom in left_geoms:
        for right_geom in right_geoms:
            fromto = np.zeros(
                6,
                dtype=np.float64,
            )

            distance = float(
                mujoco.mj_geomDistance(
                    model,
                    data,
                    left_geom,
                    right_geom,
                    1.0,
                    fromto,
                )
            )

            if distance < minimum_distance:
                minimum_distance = distance
                minimum_left_geom = left_geom
                minimum_right_geom = right_geom
                minimum_fromto = fromto.copy()

    if minimum_left_geom < 0:
        raise RuntimeError(
            "좌우 geom 사이 거리를 계산하지 못했습니다."
        )

    return (
        minimum_distance,
        minimum_left_geom,
        minimum_right_geom,
        minimum_fromto,
    )


def measure(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    left_joint_id: int,
    right_joint_id: int,
    left_geoms: list[int],
    right_geoms: list[int],
    left_position: float,
    right_position: float,
) -> dict:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    set_joint_position(
        model,
        data,
        left_joint_id,
        left_position,
    )

    set_joint_position(
        model,
        data,
        right_joint_id,
        right_position,
    )

    mujoco.mj_forward(
        model,
        data,
    )

    left_anchor = np.asarray(
        data.xanchor[left_joint_id],
        dtype=float,
    )

    right_anchor = np.asarray(
        data.xanchor[right_joint_id],
        dtype=float,
    )

    joint_origin_distance = float(
        np.linalg.norm(
            left_anchor - right_anchor
        )
    )

    (
        collision_gap,
        left_geom,
        right_geom,
        fromto,
    ) = minimum_geom_distance(
        model,
        data,
        left_geoms,
        right_geoms,
    )

    closest_point_distance = float(
        np.linalg.norm(
            fromto[3:6] - fromto[0:3]
        )
    )

    return {
        "left_position": left_position,
        "right_position": right_position,
        "joint_origin_distance": (
            joint_origin_distance
        ),
        "collision_gap": collision_gap,
        "closest_point_distance": (
            closest_point_distance
        ),
        "left_geom_id": left_geom,
        "right_geom_id": right_geom,
        "fromto": fromto,
    }


def print_measurement(
    model: mujoco.MjModel,
    label: str,
    result: dict,
) -> None:
    print()
    print(f"[{label}]")

    print(
        "  joint positions        : "
        f"left={result['left_position']:+.9f} m, "
        f"right={result['right_position']:+.9f} m"
    )

    print(
        "  joint-origin distance  : "
        f"{result['joint_origin_distance'] * 1000.0:.3f} mm"
    )

    print(
        "  minimum collision gap  : "
        f"{result['collision_gap'] * 1000.0:.3f} mm"
    )

    print(
        "  closest-point distance : "
        f"{result['closest_point_distance'] * 1000.0:.3f} mm"
    )

    print(
        "  closest left geom      : "
        f"{geom_name(model, result['left_geom_id'])}"
    )

    print(
        "  closest right geom     : "
        f"{geom_name(model, result['right_geom_id'])}"
    )

    fromto = result["fromto"]

    print(
        "  closest point left     : "
        + ", ".join(
            f"{value:+.6f}"
            for value in fromto[0:3]
        )
    )

    print(
        "  closest point right    : "
        + ", ".join(
            f"{value:+.6f}"
            for value in fromto[3:6]
        )
    )


def main() -> int:
    arguments = parse_arguments()

    model_path = Path(
        arguments.model
    ).expanduser().resolve()

    cylinder_diameter = float(
        arguments.cylinder_diameter
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

    left_body_name = object_name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        left_body_id,
        f"body_{left_body_id}",
    )

    right_body_name = object_name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        right_body_id,
        f"body_{right_body_id}",
    )

    left_body_ids = subtree_body_ids(
        model,
        left_body_id,
    )

    right_body_ids = subtree_body_ids(
        model,
        right_body_id,
    )

    left_geoms = collect_geoms(
        model,
        left_body_ids,
    )

    right_geoms = collect_geoms(
        model,
        right_body_ids,
    )

    if not left_geoms:
        raise RuntimeError(
            "왼쪽 그리퍼 geom을 찾지 못했습니다."
        )

    if not right_geoms:
        raise RuntimeError(
            "오른쪽 그리퍼 geom을 찾지 못했습니다."
        )

    print("=" * 88)
    print("MUJOCO GRIPPER COLLISION GAP MEASUREMENT")
    print("=" * 88)

    print(f"Model              : {model_path}")

    print(
        f"Left joint/body    : "
        f"{LEFT_JOINT_NAME} / {left_body_name}"
    )

    print(
        f"Right joint/body   : "
        f"{RIGHT_JOINT_NAME} / {right_body_name}"
    )

    print(
        "Left geom count    : "
        f"{len(left_geoms)}"
    )

    print(
        "Right geom count   : "
        f"{len(right_geoms)}"
    )

    print(
        "Cylinder diameter  : "
        f"{cylinder_diameter * 1000.0:.3f} mm"
    )

    measurements = []

    test_positions = [
        (
            "FULLY_OPEN",
            0.000,
            0.000,
        ),
        (
            "CURRENT_MEASURED",
            float(arguments.current_left),
            float(arguments.current_right),
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

    for label, left_q, right_q in test_positions:
        result = measure(
            model,
            data,
            left_joint_id,
            right_joint_id,
            left_geoms,
            right_geoms,
            left_q,
            right_q,
        )

        result["label"] = label
        measurements.append(result)

        print_measurement(
            model,
            label,
            result,
        )

    # 대칭 위치 q와 collision gap 관계를
    # q=0, 0.02, 0.04에서 선형 근사한다.
    symmetric_results = [
        result
        for result in measurements
        if result["label"] in {
            "FULLY_OPEN",
            "SYMMETRIC_20MM",
            "URDF_MAX_CLOSE",
        }
    ]

    q_values = np.asarray(
        [
            abs(
                result["left_position"]
            )
            for result in symmetric_results
        ],
        dtype=float,
    )

    gap_values = np.asarray(
        [
            result["collision_gap"]
            for result in symmetric_results
        ],
        dtype=float,
    )

    slope, intercept = np.polyfit(
        q_values,
        gap_values,
        1,
    )

    if abs(slope) < 1.0e-12:
        raise RuntimeError(
            "그리퍼 간격 변화율이 0입니다."
        )

    touch_q = (
        cylinder_diameter - intercept
    ) / slope

    # 전체 간격을 1 mm 더 좁히는 값.
    preload_gap = (
        cylinder_diameter - 0.001
    )

    preload_q = (
        preload_gap - intercept
    ) / slope

    print()
    print("=" * 88)
    print("60 mm CYLINDER RESULT")
    print("=" * 88)

    print(
        "Fitted gap model   : "
        f"gap = {intercept:.9f} "
        f"{slope:+.9f} * q"
    )

    print(
        "Estimated touch q  : "
        f"left={touch_q:+.9f} m, "
        f"right={-touch_q:+.9f} m"
    )

    print(
        "Estimated 1mm preload q: "
        f"left={preload_q:+.9f} m, "
        f"right={-preload_q:+.9f} m"
    )

    max_close_gap = next(
        result["collision_gap"]
        for result in measurements
        if result["label"] == "URDF_MAX_CLOSE"
    )

    print(
        "Gap at URDF limit : "
        f"{max_close_gap * 1000.0:.3f} mm"
    )

    feasible = (
        0.0 <= touch_q <= 0.040
    )

    print(
        "60 mm feasibility : "
        + (
            "POSSIBLE WITH CURRENT LIMITS"
            if feasible
            else "NOT POSSIBLE WITH CURRENT LIMITS"
        )
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
