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

DEFAULT_OUTPUT_ROOT = (
    ROOT
    / "generated_known_object_paths"
)


def coordinate_token(value: float) -> str:
    text = f"{value:.3f}"

    return (
        text.replace("-", "m")
        .replace(".", "p")
    )


def default_output_directory(
    from_box: int,
    to_box: int,
    object_x: float,
    object_y: float,
    object_z: float,
) -> Path:
    name = (
        f"box_{from_box}"
        f"_x{coordinate_token(object_x)}"
        f"_y{coordinate_token(object_y)}"
        f"_z{coordinate_token(object_z)}"
        f"_to_box_{to_box}"
    )

    return DEFAULT_OUTPUT_ROOT / name


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "알고 있는 원기둥 world 좌표에서 Pick하고 "
            "선택한 BOX의 기존 Place 위치로 이동하는 "
            "Cartesian IK 경로를 생성합니다."
        )
    )

    parser.add_argument(
        "--from-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
        help="원기둥이 놓여 있는 받침대 번호",
    )

    parser.add_argument(
        "--object-x",
        type=float,
        required=True,
        help="원기둥 중심의 world x 좌표 [m]",
    )

    parser.add_argument(
        "--object-y",
        type=float,
        required=True,
        help="원기둥 중심의 world y 좌표 [m]",
    )

    parser.add_argument(
        "--object-z",
        type=float,
        required=True,
        help="원기둥 중심의 world z 좌표 [m]",
    )

    parser.add_argument(
        "--to-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
        help="기존 고정 Place 위치를 사용할 목표 BOX",
    )

    parser.add_argument(
        "--candidate-index",
        type=int,
        default=None,
        help=(
            "특정 grasp candidate만 시험합니다. "
            "생략하면 모든 candidate를 검사합니다."
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="경로 생성 결과 폴더",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="기존 결과 폴더를 지우고 다시 생성",
    )

    return parser.parse_args()


def validate_arguments(
    arguments: argparse.Namespace,
) -> None:
    if arguments.from_box == arguments.to_box:
        raise RuntimeError(
            "출발 BOX와 목표 BOX가 같습니다."
        )

    coordinates = (
        arguments.object_x,
        arguments.object_y,
        arguments.object_z,
    )

    if not all(
        math.isfinite(value)
        for value in coordinates
    ):
        raise RuntimeError(
            "원기둥 좌표는 유한한 실수여야 합니다."
        )

    # 현재 DROK 작업 영역을 크게 벗어난 입력만 차단한다.
    if not 0.15 <= arguments.object_x <= 0.65:
        raise RuntimeError(
            "object-x가 현재 작업 영역을 벗어났습니다."
        )

    if not -0.65 <= arguments.object_y <= 0.65:
        raise RuntimeError(
            "object-y가 현재 작업 영역을 벗어났습니다."
        )

    if not 0.20 <= arguments.object_z <= 0.40:
        raise RuntimeError(
            "object-z가 현재 받침대 작업 높이를 벗어났습니다."
        )


def load_base_generator() -> Any:
    if not BASE_GENERATOR_PATH.exists():
        raise RuntimeError(
            f"기존 경로 생성기가 없습니다: "
            f"{BASE_GENERATOR_PATH}"
        )

    specification = (
        importlib.util.spec_from_file_location(
            "drok_box_to_box_generator",
            BASE_GENERATOR_PATH,
        )
    )

    if (
        specification is None
        or specification.loader is None
    ):
        raise RuntimeError(
            "기존 경로 생성기 module spec 생성 실패"
        )

    module = importlib.util.module_from_spec(
        specification
    )

    specification.loader.exec_module(module)

    if not hasattr(
        module,
        "make_planning_scene",
    ):
        raise RuntimeError(
            "기존 생성기에서 make_planning_scene을 "
            "찾지 못했습니다."
        )

    if not hasattr(module, "main"):
        raise RuntimeError(
            "기존 생성기에서 main을 찾지 못했습니다."
        )

    return module


def set_pickup_cube_position(
    planning_scene_path: Path,
    object_x: float,
    object_y: float,
    object_z: float,
) -> None:
    if not planning_scene_path.exists():
        raise RuntimeError(
            f"planning scene이 없습니다: "
            f"{planning_scene_path}"
        )

    tree = ET.parse(
        planning_scene_path
    )

    root = tree.getroot()

    pickup_body = root.find(
        ".//body[@name='pickup_cube']"
    )

    if pickup_body is None:
        raise RuntimeError(
            "planning scene에서 pickup_cube body를 "
            "찾지 못했습니다."
        )

    pickup_body.set(
        "pos",
        (
            f"{object_x:.9f} "
            f"{object_y:.9f} "
            f"{object_z:.9f}"
        ),
    )

    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        planning_scene_path,
        encoding="utf-8",
        xml_declaration=True,
    )


def read_pickup_body_position(
    planning_scene_path: Path,
) -> list[float]:
    tree = ET.parse(
        planning_scene_path
    )

    pickup_body = tree.getroot().find(
        ".//body[@name='pickup_cube']"
    )

    if pickup_body is None:
        raise RuntimeError(
            "검증 중 pickup_cube body를 찾지 못했습니다."
        )

    raw_position = pickup_body.get(
        "pos",
        "",
    ).split()

    if len(raw_position) != 3:
        raise RuntimeError(
            "pickup_cube pos 형식이 올바르지 않습니다."
        )

    return [
        float(value)
        for value in raw_position
    ]


def recursively_find_key(
    node: Any,
    target_key: str,
) -> Any:
    if isinstance(node, dict):
        if target_key in node:
            return node[target_key]

        for value in node.values():
            result = recursively_find_key(
                value,
                target_key,
            )

            if result is not None:
                return result

    elif isinstance(node, list):
        for value in node:
            result = recursively_find_key(
                value,
                target_key,
            )

            if result is not None:
                return result

    return None


def verify_candidate_object_center(
    candidate_path: Path,
    expected: list[float],
) -> list[float]:
    if not candidate_path.exists():
        raise RuntimeError(
            f"candidate 파일이 없습니다: "
            f"{candidate_path}"
        )

    document = yaml.safe_load(
        candidate_path.read_text(
            encoding="utf-8"
        )
    )

    raw_center = recursively_find_key(
        document,
        "pickup_object_center",
    )

    if not isinstance(
        raw_center,
        (list, tuple),
    ) or len(raw_center) != 3:
        raise RuntimeError(
            "candidate YAML에서 pickup_object_center를 "
            "찾지 못했습니다."
        )

    actual = [
        float(value)
        for value in raw_center
    ]

    maximum_error = max(
        abs(actual_value - expected_value)
        for actual_value, expected_value in zip(
            actual,
            expected,
        )
    )

    if maximum_error > 1.0e-6:
        raise RuntimeError(
            "candidate에 전달된 원기둥 좌표가 "
            "입력 좌표와 다릅니다. "
            f"maximum error={maximum_error:.9e}"
        )

    return actual


def save_request_metadata(
    output_directory: Path,
    arguments: argparse.Namespace,
    selected_output: Path,
) -> None:
    metadata = {
        "version": 1,
        "mode": "known_object_pose_to_fixed_box_place",
        "source_box": int(
            arguments.from_box
        ),
        "known_object_center_world": {
            "x": float(arguments.object_x),
            "y": float(arguments.object_y),
            "z": float(arguments.object_z),
        },
        "target_box": int(
            arguments.to_box
        ),
        "place_position_source": (
            "box_layout_fixed_place_position"
        ),
        "candidate_index_request": (
            None
            if arguments.candidate_index is None
            else int(arguments.candidate_index)
        ),
        "generated_output_directory": str(
            selected_output
        ),
        "side_grasp_posture_config": str(
            ROOT
            / "src/drok_arm_control/config/"
            "side_grasp_posture.yaml"
        ),
        "notes": [
            (
                "현재 단계에서는 알려진 object 좌표를 "
                "planning scene에 주입합니다."
            ),
            (
                "목표 Place 좌표는 기존 BOX 고정 위치를 "
                "사용합니다."
            ),
            (
                "LINK4 자세 검증은 생성 후 별도 분석으로 "
                "수행합니다."
            ),
        ],
    }

    metadata_path = (
        output_directory
        / "known_object_request.yaml"
    )

    metadata_path.write_text(
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

    output_directory = (
        arguments.output_dir.resolve()
        if arguments.output_dir is not None
        else default_output_directory(
            from_box=arguments.from_box,
            to_box=arguments.to_box,
            object_x=arguments.object_x,
            object_y=arguments.object_y,
            object_z=arguments.object_z,
        )
    )

    module = load_base_generator()

    original_make_planning_scene = (
        module.make_planning_scene
    )

    def patched_make_planning_scene(
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        result = original_make_planning_scene(
            *args,
            **kwargs,
        )

        output_path_value = kwargs.get(
            "output_path"
        )

        if output_path_value is None:
            if len(args) < 3:
                raise RuntimeError(
                    "planning scene output_path를 "
                    "확인하지 못했습니다."
                )

            output_path_value = args[2]

        planning_scene_path = Path(
            output_path_value
        )

        set_pickup_cube_position(
            planning_scene_path=planning_scene_path,
            object_x=arguments.object_x,
            object_y=arguments.object_y,
            object_z=arguments.object_z,
        )

        print(
            "  [KNOWN OBJECT POSE] "
            "pickup_cube 위치 변경: "
            f"[{arguments.object_x:.6f}, "
            f"{arguments.object_y:.6f}, "
            f"{arguments.object_z:.6f}]"
        )

        return result

    module.make_planning_scene = (
        patched_make_planning_scene
    )

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
        forwarded_arguments.append(
            "--overwrite"
        )

    original_argv = sys.argv[:]

    print("=" * 76)
    print("KNOWN OBJECT POSE TO BOX PATH GENERATION")
    print("=" * 76)
    print(f"Source BOX    : {arguments.from_box}")
    print(
        "Object center: "
        f"[{arguments.object_x:.6f}, "
        f"{arguments.object_y:.6f}, "
        f"{arguments.object_z:.6f}]"
    )
    print(f"Target BOX    : {arguments.to_box}")
    print(f"Output        : {output_directory}")
    print()

    try:
        sys.argv = forwarded_arguments

        result = module.main()

    finally:
        sys.argv = original_argv

    return_code = (
        0
        if result is None
        else int(result)
    )

    if return_code != 0:
        return return_code

    planning_scene_path = (
        output_directory
        / "planning_scene.xml"
    )

    candidate_path = (
        output_directory
        / "grasp_candidates.yaml"
    )

    expected_position = [
        float(arguments.object_x),
        float(arguments.object_y),
        float(arguments.object_z),
    ]

    scene_position = (
        read_pickup_body_position(
            planning_scene_path
        )
    )

    scene_error = max(
        abs(actual - expected)
        for actual, expected in zip(
            scene_position,
            expected_position,
        )
    )

    if scene_error > 1.0e-8:
        raise RuntimeError(
            "planning scene 원기둥 위치 검증 실패"
        )

    candidate_position = (
        verify_candidate_object_center(
            candidate_path,
            expected_position,
        )
    )

    save_request_metadata(
        output_directory=output_directory,
        arguments=arguments,
        selected_output=output_directory,
    )

    print()
    print("=" * 76)
    print("KNOWN OBJECT PATH GENERATION: PASS")
    print("=" * 76)
    print(
        "Planning scene object: "
        f"{scene_position}"
    )
    print(
        "Candidate object center: "
        f"{candidate_position}"
    )
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
        print(
            f"[실패] {exception}",
            file=sys.stderr,
        )

        raise SystemExit(1)
