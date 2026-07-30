#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

BASE_GENERATOR_PATH = (
    ROOT
    / "src/drok_arm_control/scripts/generate_box_to_box_path.py"
)

BOX_LAYOUT_PATH = (
    ROOT
    / "src/drok_arm_control/config/box_layout.yaml"
)

DEFAULT_OUTPUT_ROOT = (
    ROOT
    / "generated_vr_target_paths"
)


def coordinate_token(value: float) -> str:
    return (
        f"{value:.4f}"
        .replace("-", "m")
        .replace(".", "p")
    )


def default_output_directory(
    from_box: int,
    to_box: int,
    object_xyz: tuple[float, float, float],
    target_xyz: tuple[float, float, float],
) -> Path:
    ox, oy, oz = object_xyz
    tx, ty, tz = target_xyz

    name = (
        f"box_{from_box}"
        f"_x{coordinate_token(ox)}"
        f"_y{coordinate_token(oy)}"
        f"_z{coordinate_token(oz)}"
        f"_to_box_{to_box}"
        f"_x{coordinate_token(tx)}"
        f"_y{coordinate_token(ty)}"
        f"_z{coordinate_token(tz)}"
    )

    return DEFAULT_OUTPUT_ROOT / name


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "알고 있는 원기둥 world 좌표에서 Pick하고, "
            "VR이 지정한 정확한 world 좌표로 Place하는 "
            "Cartesian IK 경로를 생성합니다."
        )
    )

    parser.add_argument(
        "--from-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument("--object-x", type=float, required=True)
    parser.add_argument("--object-y", type=float, required=True)
    parser.add_argument("--object-z", type=float, required=True)

    parser.add_argument(
        "--to-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument("--target-x", type=float, required=True)
    parser.add_argument("--target-y", type=float, required=True)
    parser.add_argument("--target-z", type=float, required=True)

    parser.add_argument(
        "--candidate-index",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
    )

    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise RuntimeError(f"파일이 없습니다: {path}")

    document = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    if not isinstance(document, dict):
        raise RuntimeError(
            f"YAML root가 mapping이 아닙니다: {path}"
        )

    return document


def validate_finite(values: tuple[float, ...]) -> None:
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("좌표는 유한한 실수여야 합니다.")


def validate_arguments(arguments: argparse.Namespace) -> None:
    if arguments.from_box == arguments.to_box:
        raise RuntimeError(
            "현재 v0.1에서는 같은 받침대 내부 이동을 지원하지 않습니다."
        )

    values = (
        arguments.object_x,
        arguments.object_y,
        arguments.object_z,
        arguments.target_x,
        arguments.target_y,
        arguments.target_z,
    )

    validate_finite(values)

    for label, x, y, z in (
        (
            "object",
            arguments.object_x,
            arguments.object_y,
            arguments.object_z,
        ),
        (
            "target",
            arguments.target_x,
            arguments.target_y,
            arguments.target_z,
        ),
    ):
        if not 0.15 <= x <= 0.65:
            raise RuntimeError(
                f"{label}-x가 작업 영역을 벗어났습니다: {x}"
            )

        if not -0.65 <= y <= 0.65:
            raise RuntimeError(
                f"{label}-y가 작업 영역을 벗어났습니다: {y}"
            )

        if not 0.20 <= z <= 0.40:
            raise RuntimeError(
                f"{label}-z가 작업 높이를 벗어났습니다: {z}"
            )


def load_base_generator() -> Any:
    if not BASE_GENERATOR_PATH.exists():
        raise RuntimeError(
            f"기존 경로 생성기가 없습니다: {BASE_GENERATOR_PATH}"
        )

    specification = importlib.util.spec_from_file_location(
        "drok_box_to_box_generator_for_vr",
        BASE_GENERATOR_PATH,
    )

    if specification is None or specification.loader is None:
        raise RuntimeError("기존 생성기 module spec 생성 실패")

    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)

    if not hasattr(module, "make_planning_scene"):
        raise RuntimeError(
            "기존 생성기에서 make_planning_scene을 찾지 못했습니다."
        )

    if not hasattr(module, "main"):
        raise RuntimeError(
            "기존 생성기에서 main을 찾지 못했습니다."
        )

    return module


def parse_vector(text: str, expected_size: int) -> list[float]:
    values = [float(value) for value in text.split()]

    if len(values) != expected_size:
        raise RuntimeError(
            f"벡터 크기가 {expected_size}가 아닙니다: {values}"
        )

    return values


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


def patch_planning_scene(
    planning_scene_path: Path,
    pickup_xyz: tuple[float, float, float],
    target_geom_name: str,
    target_xyz: tuple[float, float, float],
) -> None:
    tree = ET.parse(planning_scene_path)
    root = tree.getroot()

    pickup_body = find_named_element(
        root,
        "body",
        "pickup_cube",
    )

    pickup_body.set(
        "pos",
        " ".join(f"{value:.9f}" for value in pickup_xyz),
    )

    target_geom = find_named_element(
        root,
        "geom",
        target_geom_name,
    )

    if target_geom.attrib.get("type", "sphere") != "box":
        raise RuntimeError(
            f"목표 geom이 box가 아닙니다: {target_geom_name}"
        )

    geom_size = parse_vector(
        target_geom.attrib.get("size", ""),
        3,
    )

    cylinder_geom = None

    for child in pickup_body.findall("geom"):
        if child.attrib.get("type") == "cylinder":
            cylinder_geom = child
            break

    if cylinder_geom is None:
        raise RuntimeError(
            "pickup_cube 내부에서 cylinder geom을 찾지 못했습니다."
        )

    cylinder_size = parse_vector(
        cylinder_geom.attrib.get("size", ""),
        2,
    )

    target_x, target_y, target_object_center_z = target_xyz
    target_geom_center_z = (
        target_object_center_z
        - cylinder_size[1]
        - geom_size[2]
    )

    target_geom.set(
        "pos",
        (
            f"{target_x:.9f} "
            f"{target_y:.9f} "
            f"{target_geom_center_z:.9f}"
        ),
    )

    ET.indent(tree, space="  ")
    tree.write(
        planning_scene_path,
        encoding="utf-8",
        xml_declaration=True,
    )


def recursively_find_key(node: Any, key: str) -> Any:
    if isinstance(node, dict):
        if key in node:
            return node[key]

        for value in node.values():
            found = recursively_find_key(value, key)
            if found is not None:
                return found

    elif isinstance(node, list):
        for value in node:
            found = recursively_find_key(value, key)
            if found is not None:
                return found

    return None


def verify_xyz(
    document_path: Path,
    key: str,
    expected: tuple[float, float, float],
) -> list[float]:
    document = load_yaml(document_path)
    raw = recursively_find_key(document, key)

    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise RuntimeError(
            f"{document_path}에서 {key}를 찾지 못했습니다."
        )

    actual = [float(value) for value in raw]
    error = max(
        abs(actual_value - expected_value)
        for actual_value, expected_value in zip(actual, expected)
    )

    if error > 1.0e-6:
        raise RuntimeError(
            f"{key} 검증 실패: actual={actual}, "
            f"expected={list(expected)}, error={error:.9e}"
        )

    return actual


def save_metadata(
    output_directory: Path,
    arguments: argparse.Namespace,
    target_geom_name: str,
) -> None:
    metadata = {
        "version": 1,
        "mode": "known_object_pose_to_exact_vr_target_pose",
        "source_box": int(arguments.from_box),
        "source_object_center_world": {
            "x": float(arguments.object_x),
            "y": float(arguments.object_y),
            "z": float(arguments.object_z),
        },
        "target_box": int(arguments.to_box),
        "target_object_center_world": {
            "x": float(arguments.target_x),
            "y": float(arguments.target_y),
            "z": float(arguments.target_z),
        },
        "target_support_geom": target_geom_name,
        "candidate_index_request": (
            None
            if arguments.candidate_index is None
            else int(arguments.candidate_index)
        ),
        "notes": [
            "실제 MuJoCo scene.xml은 수정하지 않습니다.",
            (
                "경로 생성용 planning_scene.xml에서 목표 받침대 geom의 "
                "중심을 VR 목표 x/y로 임시 이동하여 기존 검증 파이프라인을 재사용합니다."
            ),
            (
                "실제 실행 시 목표 좌표가 원래 받침대 경계 내부인지 "
                "vr_pick_place_node.py가 먼저 검증합니다."
            ),
        ],
    }

    (output_directory / "vr_target_request.yaml").write_text(
        yaml.safe_dump(
            metadata,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def main() -> int:
    arguments = parse_arguments()
    validate_arguments(arguments)

    layout = load_yaml(BOX_LAYOUT_PATH)
    boxes = layout.get("boxes")

    if not isinstance(boxes, dict):
        raise RuntimeError("box_layout.yaml에 boxes가 없습니다.")

    target_box = boxes.get(arguments.to_box)
    if target_box is None:
        target_box = boxes.get(str(arguments.to_box))

    if not isinstance(target_box, dict):
        raise RuntimeError(
            f"BOX {arguments.to_box} 설정을 찾지 못했습니다."
        )

    target_geom_name = str(target_box["geom_name"])

    pickup_xyz = (
        float(arguments.object_x),
        float(arguments.object_y),
        float(arguments.object_z),
    )

    target_xyz = (
        float(arguments.target_x),
        float(arguments.target_y),
        float(arguments.target_z),
    )

    output_directory = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else default_output_directory(
            from_box=arguments.from_box,
            to_box=arguments.to_box,
            object_xyz=pickup_xyz,
            target_xyz=target_xyz,
        )
    )

    module = load_base_generator()
    original_make_planning_scene = module.make_planning_scene

    def patched_make_planning_scene(*args: Any, **kwargs: Any) -> Any:
        result = original_make_planning_scene(*args, **kwargs)

        output_path_value = kwargs.get("output_path")

        if output_path_value is None:
            if len(args) < 3:
                raise RuntimeError(
                    "planning scene output_path를 확인하지 못했습니다."
                )

            output_path_value = args[2]

        planning_scene_path = Path(output_path_value)

        patch_planning_scene(
            planning_scene_path=planning_scene_path,
            pickup_xyz=pickup_xyz,
            target_geom_name=target_geom_name,
            target_xyz=target_xyz,
        )

        print(
            "  [VR EXACT TARGET] "
            f"pickup={list(pickup_xyz)}, "
            f"target={list(target_xyz)}, "
            f"support={target_geom_name}"
        )

        return result

    module.make_planning_scene = patched_make_planning_scene

    forwarded_arguments = [
        str(BASE_GENERATOR_PATH),
        "--from-box",
        str(arguments.from_box),
        "--to-box",
        str(arguments.to_box),
        "--output-dir",
        str(output_directory),
    ]

    if arguments.candidate_index is not None:
        forwarded_arguments.extend(
            [
                "--candidate-index",
                str(arguments.candidate_index),
            ]
        )

    if arguments.overwrite:
        forwarded_arguments.append("--overwrite")

    original_argv = sys.argv[:]

    print("=" * 80)
    print("KNOWN OBJECT POSE TO EXACT VR TARGET PATH GENERATION")
    print("=" * 80)
    print(f"Source BOX   : {arguments.from_box}")
    print(f"Pickup center: {list(pickup_xyz)}")
    print(f"Target BOX   : {arguments.to_box}")
    print(f"Target center: {list(target_xyz)}")
    print(f"Output        : {output_directory}")
    print()

    try:
        sys.argv = forwarded_arguments
        result = module.main()
    finally:
        sys.argv = original_argv

    return_code = 0 if result is None else int(result)

    if return_code != 0:
        return return_code

    candidate_path = output_directory / "grasp_candidates.yaml"

    pickup_actual = verify_xyz(
        candidate_path,
        "pickup_object_center",
        pickup_xyz,
    )

    target_actual = verify_xyz(
        candidate_path,
        "place_object_center",
        target_xyz,
    )

    save_metadata(
        output_directory=output_directory,
        arguments=arguments,
        target_geom_name=target_geom_name,
    )

    print()
    print("=" * 80)
    print("EXACT VR TARGET PATH GENERATION: PASS")
    print("=" * 80)
    print(f"Verified pickup center: {pickup_actual}")
    print(f"Verified target center: {target_actual}")
    print(
        "Timed path: "
        f"{output_directory / 'timed_joint_path.yaml'}"
    )

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exception:
        print(f"[실패] {exception}", file=sys.stderr)
        raise SystemExit(1)
