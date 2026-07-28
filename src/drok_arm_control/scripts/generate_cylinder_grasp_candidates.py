#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import yaml


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--scene",
        default=(
            "src/drok_arm_mujoco/model/scene.xml"
        ),
    )

    parser.add_argument(
        "--config",
        default=(
            "src/drok_arm_control/config/"
            "cylinder_grasp_geometry.yaml"
        ),
    )

    parser.add_argument(
        "--output",
        default="cylinder_grasp_candidates.yaml",
    )

    return parser.parse_args()


def parse_vector(
    text: str | None,
    size: int,
    default: List[float] | None = None,
) -> np.ndarray:
    if text is None:
        if default is None:
            raise ValueError(
                "필수 벡터 속성이 없습니다."
            )

        values = default
    else:
        values = [
            float(value)
            for value in text.split()
        ]

    if len(values) != size:
        raise ValueError(
            f"벡터 크기가 {size}가 아닙니다: "
            f"{values}"
        )

    return np.asarray(
        values,
        dtype=float,
    )


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(
        np.linalg.norm(vector)
    )

    if norm < 1.0e-12:
        raise ValueError(
            "영벡터는 단위벡터로 만들 수 없습니다."
        )

    return vector / norm


def find_named_element(
    root: ET.Element,
    tag: str,
    name: str,
) -> ET.Element:
    for element in root.iter(tag):
        if element.attrib.get("name") == name:
            return element

    raise RuntimeError(
        f"<{tag} name='{name}'>을 찾지 못했습니다."
    )


def read_box_top_z(
    geom: ET.Element,
) -> float:
    geom_type = geom.attrib.get(
        "type",
        "sphere",
    )

    if geom_type != "box":
        raise RuntimeError(
            f"{geom.attrib.get('name')}은 "
            f"box geom이 아닙니다: {geom_type}"
        )

    position = parse_vector(
        geom.attrib.get("pos"),
        3,
        [0.0, 0.0, 0.0],
    )

    size = parse_vector(
        geom.attrib.get("size"),
        3,
    )

    return float(
        position[2] + size[2]
    )


def read_cylinder(
    body: ET.Element,
) -> tuple[np.ndarray, float, float]:
    body_position = parse_vector(
        body.attrib.get("pos"),
        3,
        [0.0, 0.0, 0.0],
    )

    cylinder_geom = None

    for geom in body.findall("geom"):
        if geom.attrib.get("type") == "cylinder":
            cylinder_geom = geom
            break

    if cylinder_geom is None:
        raise RuntimeError(
            "pickup object 내부에서 "
            "cylinder geom을 찾지 못했습니다."
        )

    geom_position = parse_vector(
        cylinder_geom.attrib.get("pos"),
        3,
        [0.0, 0.0, 0.0],
    )

    size = parse_vector(
        cylinder_geom.attrib.get("size"),
        2,
    )

    radius = float(size[0])
    half_height = float(size[1])

    center = (
        body_position
        + geom_position
    )

    return (
        center,
        radius,
        2.0 * half_height,
    )


def rotation_to_list(
    rotation: np.ndarray,
) -> List[List[float]]:
    return [
        [
            float(rotation[row, column])
            for column in range(3)
        ]
        for row in range(3)
    ]


def vector_to_list(
    vector: np.ndarray,
) -> List[float]:
    return [
        float(value)
        for value in vector
    ]


def make_tcp_position(
    grasp_center_world: np.ndarray,
    rotation_world_tcp: np.ndarray,
    tcp_to_grasp_center: np.ndarray,
) -> np.ndarray:
    return (
        grasp_center_world
        - rotation_world_tcp
        @ tcp_to_grasp_center
    )


def main() -> int:
    arguments = parse_arguments()

    scene_path = Path(
        arguments.scene
    ).expanduser().resolve()

    config_path = Path(
        arguments.config
    ).expanduser().resolve()

    output_path = Path(
        arguments.output
    ).expanduser().resolve()

    with config_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        config = yaml.safe_load(stream)

    scene_root = ET.parse(
        scene_path
    ).getroot()

    scene_config = config["scene"]
    frame_config = config["frames"]
    grasp_config = config["grasp"]
    search_config = config["search"]

    pickup_body = find_named_element(
        scene_root,
        "body",
        scene_config["pickup_object_body"],
    )

    pickup_pedestal = find_named_element(
        scene_root,
        "geom",
        scene_config["pickup_pedestal_geom"],
    )

    place_pedestal = find_named_element(
        scene_root,
        "geom",
        scene_config["place_pedestal_geom"],
    )

    (
        pickup_object_center,
        cylinder_radius,
        cylinder_height,
    ) = read_cylinder(
        pickup_body
    )

    pickup_pedestal_position = parse_vector(
        pickup_pedestal.attrib.get("pos"),
        3,
        [0.0, 0.0, 0.0],
    )

    place_pedestal_position = parse_vector(
        place_pedestal.attrib.get("pos"),
        3,
        [0.0, 0.0, 0.0],
    )

    pickup_top_z = read_box_top_z(
        pickup_pedestal
    )

    place_top_z = read_box_top_z(
        place_pedestal
    )

    place_object_center = np.array(
        [
            place_pedestal_position[0],
            place_pedestal_position[1],
            place_top_z
            + 0.5 * cylinder_height,
        ],
        dtype=float,
    )

    approach_axis_tcp = normalize(
        np.asarray(
            frame_config[
                "approach_axis_tcp"
            ],
            dtype=float,
        )
    )

    closing_axis_tcp = normalize(
        np.asarray(
            frame_config[
                "closing_axis_tcp"
            ],
            dtype=float,
        )
    )

    upright_axis_tcp = normalize(
        np.asarray(
            frame_config[
                "upright_axis_tcp"
            ],
            dtype=float,
        )
    )

    # R_TS:
    # semantic grasp frame의 각 축을
    # TCP 좌표계에서 표현한 회전행렬
    rotation_tcp_semantic = np.column_stack(
        (
            approach_axis_tcp,
            closing_axis_tcp,
            upright_axis_tcp,
        )
    )

    orthogonality_error = float(
        np.linalg.norm(
            rotation_tcp_semantic.T
            @ rotation_tcp_semantic
            - np.eye(3)
        )
    )

    determinant = float(
        np.linalg.det(
            rotation_tcp_semantic
        )
    )

    if orthogonality_error > 1.0e-9:
        raise RuntimeError(
            "TCP 의미론적 축이 직교하지 않습니다."
        )

    if abs(determinant - 1.0) > 1.0e-9:
        raise RuntimeError(
            "TCP 의미론적 축이 오른손 좌표계가 아닙니다."
        )

    tcp_to_grasp_center = np.asarray(
        frame_config[
            "tcp_to_grasp_center"
        ],
        dtype=float,
    )

    height_ratio = float(
        grasp_config["height_ratio"]
    )

    if not 0.0 <= height_ratio <= 1.0:
        raise RuntimeError(
            "grasp.height_ratio는 "
            "0 이상 1 이하여야 합니다."
        )

    # 원기둥 중심으로부터 실제 파지 중심까지의
    # world Z 방향 offset.
    grasp_vertical_offset = (
        height_ratio - 0.5
    ) * cylinder_height

    surface_clearance = float(
        grasp_config[
            "pregrasp_surface_clearance"
        ]
    )

    vertical_clearance = float(
        grasp_config["vertical_clearance"]
    )

    candidate_count = int(
        search_config[
            "azimuth_candidate_count"
        ]
    )

    if candidate_count < 1:
        raise RuntimeError(
            "azimuth_candidate_count는 "
            "1 이상이어야 합니다."
        )

    world_upright = np.array(
        [0.0, 0.0, 1.0],
        dtype=float,
    )

    pickup_grasp_center = (
        pickup_object_center
        + world_upright
        * grasp_vertical_offset
    )

    place_grasp_center = (
        place_object_center
        + world_upright
        * grasp_vertical_offset
    )

    # 먼저 운반 중 원기둥 중심의 안전 높이를 계산한다.
    safe_object_center_z = (
        max(
            pickup_top_z,
            place_top_z,
        )
        + 0.5 * cylinder_height
        + vertical_clearance
    )

    # 파지 중심이 원기둥 중심보다 위에 있으면
    # TCP의 LIFT 높이도 같은 offset만큼 올려야 한다.
    safe_center_z = (
        safe_object_center_z
        + grasp_vertical_offset
    )

    pregrasp_distance = (
        cylinder_radius
        + surface_clearance
    )

    candidates: List[Dict[str, Any]] = []

    for index in range(candidate_count):
        phi = (
            2.0
            * math.pi
            * index
            / candidate_count
        )

        radial_outward = np.array(
            [
                math.cos(phi),
                math.sin(phi),
                0.0,
            ],
            dtype=float,
        )

        # 그리퍼가 물체로 이동하는 방향
        approach_world = (
            -radial_outward
        )

        closing_world = normalize(
            np.cross(
                world_upright,
                approach_world,
            )
        )

        rotation_world_semantic = (
            np.column_stack(
                (
                    approach_world,
                    closing_world,
                    world_upright,
                )
            )
        )

        # R_WT * R_TS = R_WS
        rotation_world_tcp = (
            rotation_world_semantic
            @ rotation_tcp_semantic.T
        )

        rotation_error = float(
            np.linalg.norm(
                rotation_world_tcp.T
                @ rotation_world_tcp
                - np.eye(3)
            )
        )

        rotation_determinant = float(
            np.linalg.det(
                rotation_world_tcp
            )
        )

        if rotation_error > 1.0e-9:
            raise RuntimeError(
                f"후보 {index}의 회전행렬이 "
                "직교하지 않습니다."
            )

        if abs(
            rotation_determinant - 1.0
        ) > 1.0e-9:
            raise RuntimeError(
                f"후보 {index}의 회전행렬이 "
                "오른손 좌표계가 아닙니다."
            )

        pickup_grasp_tcp = make_tcp_position(
            pickup_grasp_center,
            rotation_world_tcp,
            tcp_to_grasp_center,
        )

        pickup_pregrasp_tcp = (
            pickup_grasp_tcp
            - pregrasp_distance
            * approach_world
        )

        pickup_lift_center = (
            pickup_grasp_center.copy()
        )

        pickup_lift_center[2] = (
            safe_center_z
        )

        pickup_lift_tcp = make_tcp_position(
            pickup_lift_center,
            rotation_world_tcp,
            tcp_to_grasp_center,
        )

        place_grasp_tcp = make_tcp_position(
            place_grasp_center,
            rotation_world_tcp,
            tcp_to_grasp_center,
        )

        place_pregrasp_tcp = (
            place_grasp_tcp
            - pregrasp_distance
            * approach_world
        )

        place_lift_center = (
            place_grasp_center.copy()
        )

        place_lift_center[2] = (
            safe_center_z
        )

        place_lift_tcp = make_tcp_position(
            place_lift_center,
            rotation_world_tcp,
            tcp_to_grasp_center,
        )

        candidates.append(
            {
                "index": index,
                "phi_rad": float(phi),
                "phi_deg": float(
                    math.degrees(phi)
                ),
                "axes_world": {
                    "approach": vector_to_list(
                        approach_world
                    ),
                    "closing": vector_to_list(
                        closing_world
                    ),
                    "upright": vector_to_list(
                        world_upright
                    ),
                },
                "rotation_world_tcp": (
                    rotation_to_list(
                        rotation_world_tcp
                    )
                ),
                "pickup": {
                    "grasp_tcp_xyz": (
                        vector_to_list(
                            pickup_grasp_tcp
                        )
                    ),
                    "pregrasp_tcp_xyz": (
                        vector_to_list(
                            pickup_pregrasp_tcp
                        )
                    ),
                    "lift_tcp_xyz": (
                        vector_to_list(
                            pickup_lift_tcp
                        )
                    ),
                },
                "place": {
                    "grasp_tcp_xyz": (
                        vector_to_list(
                            place_grasp_tcp
                        )
                    ),
                    "pregrasp_tcp_xyz": (
                        vector_to_list(
                            place_pregrasp_tcp
                        )
                    ),
                    "lift_tcp_xyz": (
                        vector_to_list(
                            place_lift_tcp
                        )
                    ),
                },
            }
        )

    document: Dict[str, Any] = {
        "version": 1,
        "source": {
            "scene": str(scene_path),
            "config": str(config_path),
        },
        "derived_geometry": {
            "cylinder_radius": (
                cylinder_radius
            ),
            "cylinder_height": (
                cylinder_height
            ),
            "pickup_object_center": (
                vector_to_list(
                    pickup_object_center
                )
            ),
            "place_object_center": (
                vector_to_list(
                    place_object_center
                )
            ),
            "pickup_pedestal_top_z": (
                pickup_top_z
            ),
            "place_pedestal_top_z": (
                place_top_z
            ),
            "safe_object_center_z": (
                safe_object_center_z
            ),
            "safe_grasp_center_z": (
                safe_center_z
            ),
            "grasp_height_ratio": (
                height_ratio
            ),
            "grasp_vertical_offset": (
                grasp_vertical_offset
            ),
            "pregrasp_distance": (
                pregrasp_distance
            ),
        },
        "frame_validation": {
            "rotation_tcp_semantic": (
                rotation_to_list(
                    rotation_tcp_semantic
                )
            ),
            "orthogonality_error": (
                orthogonality_error
            ),
            "determinant": determinant,
        },
        "candidates": candidates,
    }

    output_path.write_text(
        yaml.safe_dump(
            document,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    print("=" * 88)
    print("CYLINDER GRASP GEOMETRY")
    print("=" * 88)

    print(
        "Cylinder radius : "
        f"{cylinder_radius:.6f} m"
    )

    print(
        "Cylinder height : "
        f"{cylinder_height:.6f} m"
    )

    print(
        "Pickup center   : "
        f"{pickup_object_center.tolist()}"
    )

    print(
        "Place center    : "
        f"{place_object_center.tolist()}"
    )

    print(
        "Grasp ratio     : "
        f"{height_ratio:.6f}"
    )

    print(
        "Grasp z offset  : "
        f"{grasp_vertical_offset:.6f} m"
    )

    print(
        "Safe object z   : "
        f"{safe_object_center_z:.6f} m"
    )

    print(
        "Safe grasp z    : "
        f"{safe_center_z:.6f} m"
    )

    print(
        "Pregrasp travel : "
        f"{pregrasp_distance:.6f} m"
    )

    print(
        "Candidate count : "
        f"{candidate_count}"
    )

    print()
    print("TCP semantic frame validation")
    print(
        "  orthogonality error : "
        f"{orthogonality_error:.3e}"
    )
    print(
        "  determinant         : "
        f"{determinant:.12f}"
    )

    print()
    print("First candidate")

    first = candidates[0]

    print(
        "  phi                 : "
        f"{first['phi_deg']:.3f} deg"
    )

    print(
        "  approach world      : "
        f"{first['axes_world']['approach']}"
    )

    print(
        "  closing world       : "
        f"{first['axes_world']['closing']}"
    )

    print(
        "  upright world       : "
        f"{first['axes_world']['upright']}"
    )

    print(
        "  pickup pregrasp TCP : "
        f"{first['pickup']['pregrasp_tcp_xyz']}"
    )

    print(
        "  pickup grasp TCP    : "
        f"{first['pickup']['grasp_tcp_xyz']}"
    )

    print()
    print("Saved:")
    print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
