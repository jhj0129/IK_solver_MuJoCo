#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import random
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

DEFAULT_LAYOUT = (
    ROOT
    / "src/drok_arm_control/config/box_layout.yaml"
)

DEFAULT_SCENE = (
    ROOT
    / "src/drok_arm_mujoco/model/scene.xml"
)

DEFAULT_STATE = (
    ROOT
    / "runtime_state/random_cylinder_spawn.yaml"
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "선택한 받침대 내부의 안전 범위에서 원기둥 "
            "중심 좌표를 무작위로 생성하고 scene.xml에 반영합니다."
        )
    )

    parser.add_argument(
        "--box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
        help="원기둥을 생성할 받침대 번호",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=1,
        help="재현 가능한 난수 seed",
    )

    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.010,
        help="원기둥 외곽과 받침대 끝 사이의 최소 여유 [m]",
    )

    parser.add_argument(
        "--layout",
        type=Path,
        default=DEFAULT_LAYOUT,
    )

    parser.add_argument(
        "--scene",
        type=Path,
        default=DEFAULT_SCENE,
    )

    parser.add_argument(
        "--state-output",
        type=Path,
        default=DEFAULT_STATE,
    )

    parser.add_argument(
        "--apply",
        action="store_true",
        help="생성한 위치를 scene.xml에 실제로 기록",
    )

    return parser.parse_args()


def require_mapping(
    value: Any,
    name: str,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RuntimeError(
            f"{name} 항목이 mapping 형식이 아닙니다."
        )

    return value


def read_configuration(
    path: Path,
) -> dict[str, Any]:
    if not path.exists():
        raise RuntimeError(
            f"box layout 파일이 없습니다: {path}"
        )

    document = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    return require_mapping(
        document,
        "box_layout",
    )


def read_box(
    layout: dict[str, Any],
    box_number: int,
) -> dict[str, Any]:
    boxes = require_mapping(
        layout.get("boxes"),
        "boxes",
    )

    box = boxes.get(box_number)

    if box is None:
        box = boxes.get(str(box_number))

    return require_mapping(
        box,
        f"boxes[{box_number}]",
    )


def finite_float(
    value: Any,
    name: str,
) -> float:
    result = float(value)

    if not math.isfinite(result):
        raise RuntimeError(
            f"{name} 값이 유한한 실수가 아닙니다."
        )

    return result


def write_scene_position(
    scene_path: Path,
    position: list[float],
) -> str:
    if not scene_path.exists():
        raise RuntimeError(
            f"scene.xml이 없습니다: {scene_path}"
        )

    backup_path = scene_path.with_name(
        "scene_before_random_spawn.xml"
    )

    if not backup_path.exists():
        shutil.copy2(
            scene_path,
            backup_path,
        )

    tree = ET.parse(scene_path)
    root = tree.getroot()

    body = root.find(
        ".//body[@name='pickup_cube']"
    )

    if body is None:
        raise RuntimeError(
            "scene.xml에서 pickup_cube body를 찾지 못했습니다."
        )

    old_position = body.get("pos", "")

    body.set(
        "pos",
        " ".join(
            f"{value:.9f}"
            for value in position
        ),
    )

    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        scene_path,
        encoding="utf-8",
        xml_declaration=True,
    )

    verification_tree = ET.parse(
        scene_path
    )

    verification_body = (
        verification_tree.getroot().find(
            ".//body[@name='pickup_cube']"
        )
    )

    if verification_body is None:
        raise RuntimeError(
            "scene.xml 기록 후 pickup_cube 검증 실패"
        )

    actual = [
        float(value)
        for value in verification_body.get(
            "pos",
            "",
        ).split()
    ]

    if len(actual) != 3:
        raise RuntimeError(
            "기록된 pickup_cube pos 형식 오류"
        )

    maximum_error = max(
        abs(actual_value - expected_value)
        for actual_value, expected_value in zip(
            actual,
            position,
        )
    )

    if maximum_error > 1.0e-9:
        raise RuntimeError(
            "scene.xml 위치 기록 검증 실패: "
            f"maximum error={maximum_error:.9e}"
        )

    return old_position


def main() -> int:
    arguments = parse_arguments()

    if arguments.safety_margin < 0.0:
        raise RuntimeError(
            "safety-margin은 0 이상이어야 합니다."
        )

    layout = read_configuration(
        arguments.layout.resolve()
    )

    geometry = require_mapping(
        layout.get("geometry"),
        "geometry",
    )

    pedestal_half_size = require_mapping(
        geometry.get("pedestal_half_size"),
        "geometry.pedestal_half_size",
    )

    cylinder = require_mapping(
        geometry.get("cylinder"),
        "geometry.cylinder",
    )

    box = read_box(
        layout,
        arguments.box,
    )

    box_position = require_mapping(
        box.get("position"),
        f"boxes[{arguments.box}].position",
    )

    center_x = finite_float(
        box_position.get("x"),
        "box center x",
    )

    center_y = finite_float(
        box_position.get("y"),
        "box center y",
    )

    half_x = finite_float(
        pedestal_half_size.get("x"),
        "pedestal half size x",
    )

    half_y = finite_float(
        pedestal_half_size.get("y"),
        "pedestal half size y",
    )

    cylinder_radius = finite_float(
        cylinder.get("radius"),
        "cylinder radius",
    )

    center_z = finite_float(
        cylinder.get("initial_center_z"),
        "cylinder center z",
    )

    safe_half_x = (
        half_x
        - cylinder_radius
        - arguments.safety_margin
    )

    safe_half_y = (
        half_y
        - cylinder_radius
        - arguments.safety_margin
    )

    if safe_half_x <= 0.0 or safe_half_y <= 0.0:
        raise RuntimeError(
            "안전여유를 적용한 랜덤 생성 범위가 없습니다."
        )

    minimum_x = center_x - safe_half_x
    maximum_x = center_x + safe_half_x
    minimum_y = center_y - safe_half_y
    maximum_y = center_y + safe_half_y

    generator = random.Random(
        arguments.seed
    )

    object_x = round(
        generator.uniform(
            minimum_x,
            maximum_x,
        ),
        6,
    )

    object_y = round(
        generator.uniform(
            minimum_y,
            maximum_y,
        ),
        6,
    )

    object_position = [
        object_x,
        object_y,
        center_z,
    ]

    x_edge_clearance = (
        half_x
        - abs(object_x - center_x)
        - cylinder_radius
    )

    y_edge_clearance = (
        half_y
        - abs(object_y - center_y)
        - cylinder_radius
    )

    if (
        x_edge_clearance
        < arguments.safety_margin - 1.0e-9
        or y_edge_clearance
        < arguments.safety_margin - 1.0e-9
    ):
        raise RuntimeError(
            "생성 위치의 받침대 경계 여유 검증 실패"
        )

    state = {
        "version": 1,
        "coordinate_frame": "world",
        "source_box": int(arguments.box),
        "seed": int(arguments.seed),
        "object_center": {
            "x": object_x,
            "y": object_y,
            "z": center_z,
        },
        "safe_center_range": {
            "x": {
                "minimum": minimum_x,
                "maximum": maximum_x,
            },
            "y": {
                "minimum": minimum_y,
                "maximum": maximum_y,
            },
        },
        "geometry": {
            "pedestal_half_size": {
                "x": half_x,
                "y": half_y,
            },
            "cylinder_radius": cylinder_radius,
            "requested_safety_margin": (
                float(arguments.safety_margin)
            ),
            "actual_edge_clearance": {
                "x": x_edge_clearance,
                "y": y_edge_clearance,
            },
        },
        "scene_applied": bool(arguments.apply),
    }

    arguments.state_output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    arguments.state_output.write_text(
        yaml.safe_dump(
            state,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    old_position = None

    if arguments.apply:
        old_position = write_scene_position(
            arguments.scene.resolve(),
            object_position,
        )

    print("=" * 76)
    print("RANDOM CYLINDER SPAWN")
    print("=" * 76)
    print(f"BOX             : {arguments.box}")
    print(f"Seed            : {arguments.seed}")
    print(
        "Safe x range    : "
        f"[{minimum_x:.6f}, {maximum_x:.6f}]"
    )
    print(
        "Safe y range    : "
        f"[{minimum_y:.6f}, {maximum_y:.6f}]"
    )
    print(
        "Object center   : "
        f"[{object_x:.6f}, "
        f"{object_y:.6f}, "
        f"{center_z:.6f}]"
    )
    print(
        "Edge clearance  : "
        f"x={x_edge_clearance * 1000.0:.2f} mm, "
        f"y={y_edge_clearance * 1000.0:.2f} mm"
    )
    print(
        f"State file      : "
        f"{arguments.state_output.resolve()}"
    )

    if arguments.apply:
        print(f"Previous scene  : {old_position}")
        print(
            "Scene update    : PASS "
            "(MuJoCo 재시작 필요)"
        )
    else:
        print(
            "Scene update    : DRY RUN "
            "(--apply 사용 시 기록)"
        )

    print("=" * 76)
    print(
        "RANDOM SPAWN RESULT: "
        + ("PASS" if arguments.apply else "DRY-RUN PASS")
    )
    print("=" * 76)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())

    except KeyboardInterrupt:
        raise SystemExit(130)

    except Exception as exception:
        print(
            f"[실패] {exception}",
            file=sys.stderr,
        )

        raise SystemExit(1)
