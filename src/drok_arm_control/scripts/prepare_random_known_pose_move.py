#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[3]

SCRIPTS_DIR = (
    ROOT
    / "src/drok_arm_control/scripts"
)

LAYOUT_PATH = (
    ROOT
    / "src/drok_arm_control/config/box_layout.yaml"
)

SOURCE_SCENE_PATH = (
    ROOT
    / "src/drok_arm_mujoco/model/scene.xml"
)

INSTALL_SCENE_PATH = (
    ROOT
    / "install/drok_arm_mujoco/share/"
    "drok_arm_mujoco/model/scene.xml"
)

RANDOM_STATE_PATH = (
    ROOT
    / "runtime_state/random_cylinder_spawn.yaml"
)

PREPARED_STATE_PATH = (
    ROOT
    / "runtime_state/prepared_random_move.yaml"
)

KINEMATICS_MODEL_PATH = (
    ROOT
    / "src/drok_arm_control/config/"
    "drok_arm_kinematics_only.urdf"
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "안전한 랜덤 물체 위치 생성부터 경로 생성, "
            "측면 파지 분석, 실행 Dry-run, 빌드까지 수행합니다."
        )
    )

    parser.add_argument(
        "--from-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument(
        "--to-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
    )

    parser.add_argument(
        "--seed",
        type=int,
        required=True,
    )

    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.010,
        help="원기둥 외곽과 받침대 경계 사이 여유 [m]",
    )

    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.5,
    )

    parser.add_argument(
        "--candidate-index",
        type=int,
        default=None,
        help=(
            "지정하지 않으면 전체 36개 후보를 검사합니다."
        ),
    )

    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="경로 준비 후 colcon build를 생략합니다.",
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


def finite_float(
    value: Any,
    name: str,
) -> float:
    result = float(value)

    if not math.isfinite(result):
        raise RuntimeError(
            f"{name} 값이 유한한 숫자가 아닙니다."
        )

    return result


def run_step(
    name: str,
    command: list[str],
) -> None:
    print()
    print("=" * 80)
    print(name)
    print("=" * 80)
    print(
        "COMMAND: "
        + " ".join(
            str(item)
            for item in command
        )
    )
    print()

    result = subprocess.run(
        command,
        cwd=ROOT,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"{name} 실패: return code "
            f"{result.returncode}"
        )

    print()
    print(f"{name}: PASS")


def encode_coordinate(
    value: float,
) -> str:
    return (
        f"{value:.6f}"
        .replace("-", "m")
        .replace(".", "p")
    )


def read_yaml(
    path: Path,
) -> dict[str, Any]:
    if not path.exists():
        raise RuntimeError(
            f"YAML 파일이 없습니다: {path}"
        )

    document = yaml.safe_load(
        path.read_text(
            encoding="utf-8"
        )
    )

    return require_mapping(
        document,
        str(path),
    )


def read_pickup_cube_position(
    path: Path,
) -> list[float]:
    if not path.exists():
        raise RuntimeError(
            f"scene 파일이 없습니다: {path}"
        )

    tree = ET.parse(path)

    body = tree.getroot().find(
        ".//body[@name='pickup_cube']"
    )

    if body is None:
        raise RuntimeError(
            f"pickup_cube를 찾지 못했습니다: {path}"
        )

    values = [
        float(value)
        for value in body.get(
            "pos",
            "",
        ).split()
    ]

    if len(values) != 3:
        raise RuntimeError(
            f"pickup_cube pos 형식 오류: {path}"
        )

    return values


def verify_position(
    actual: list[float],
    expected: list[float],
    name: str,
) -> None:
    maximum_error = max(
        abs(a - b)
        for a, b in zip(
            actual,
            expected,
        )
    )

    print(
        f"{name:<24}: "
        f"[{actual[0]:.6f}, "
        f"{actual[1]:.6f}, "
        f"{actual[2]:.6f}]"
    )

    if maximum_error > 1.0e-9:
        raise RuntimeError(
            f"{name} 위치 불일치: "
            f"maximum error={maximum_error:.9e}"
        )


def main() -> int:
    arguments = parse_arguments()

    if arguments.from_box == arguments.to_box:
        raise RuntimeError(
            "from-box와 to-box는 서로 달라야 합니다."
        )

    if arguments.safety_margin < 0.0:
        raise RuntimeError(
            "safety-margin은 0 이상이어야 합니다."
        )

    if not (
        0.0
        < arguments.speed_scale
        <= 1.0
    ):
        raise RuntimeError(
            "speed-scale은 0보다 크고 "
            "1 이하여야 합니다."
        )

    required_files = [
        SCRIPTS_DIR
        / "set_random_cylinder_spawn.py",
        SCRIPTS_DIR
        / "generate_known_object_to_box_path.py",
        SCRIPTS_DIR
        / "analyze_link4_side_grasp.py",
        SCRIPTS_DIR
        / "execute_box_move.py",
        LAYOUT_PATH,
        KINEMATICS_MODEL_PATH,
    ]

    for path in required_files:
        if not path.exists():
            raise RuntimeError(
                f"필수 파일이 없습니다: {path}"
            )

    run_step(
        "STEP 1 - RANDOM SPAWN GENERATION",
        [
            sys.executable,
            str(
                SCRIPTS_DIR
                / "set_random_cylinder_spawn.py"
            ),
            "--box",
            str(arguments.from_box),
            "--seed",
            str(arguments.seed),
            "--safety-margin",
            f"{arguments.safety_margin:.9f}",
            "--apply",
        ],
    )

    random_state = read_yaml(
        RANDOM_STATE_PATH
    )

    object_center = require_mapping(
        random_state.get(
            "object_center"
        ),
        "object_center",
    )

    object_x = finite_float(
        object_center.get("x"),
        "object x",
    )

    object_y = finite_float(
        object_center.get("y"),
        "object y",
    )

    object_z = finite_float(
        object_center.get("z"),
        "object z",
    )

    expected_position = [
        object_x,
        object_y,
        object_z,
    ]

    source_position = (
        read_pickup_cube_position(
            SOURCE_SCENE_PATH
        )
    )

    verify_position(
        source_position,
        expected_position,
        "Source scene object",
    )

    output_name = (
        f"seed_{arguments.seed}_"
        f"box_{arguments.from_box}_"
        f"x{encode_coordinate(object_x)}_"
        f"y{encode_coordinate(object_y)}_"
        f"z{encode_coordinate(object_z)}_"
        f"to_box_{arguments.to_box}"
    )

    output_directory = (
        ROOT
        / "generated_random_known_pose_moves"
        / output_name
    )

    generator_command = [
        sys.executable,
        str(
            SCRIPTS_DIR
            / "generate_known_object_to_box_path.py"
        ),
        "--from-box",
        str(arguments.from_box),
        "--object-x",
        f"{object_x:.9f}",
        "--object-y",
        f"{object_y:.9f}",
        "--object-z",
        f"{object_z:.9f}",
        "--to-box",
        str(arguments.to_box),
        "--output-dir",
        str(output_directory),
        "--overwrite",
    ]

    if arguments.candidate_index is not None:
        generator_command.extend(
            [
                "--candidate-index",
                str(
                    arguments.candidate_index
                ),
            ]
        )

    run_step(
        "STEP 2 - IK AND PATH GENERATION",
        generator_command,
    )

    timed_path = (
        output_directory
        / "timed_joint_path.yaml"
    )

    if not timed_path.exists():
        raise RuntimeError(
            f"timed path가 생성되지 않았습니다: "
            f"{timed_path}"
        )

    layout = read_yaml(
        LAYOUT_PATH
    )

    geometry = require_mapping(
        layout.get("geometry"),
        "geometry",
    )

    cylinder_geometry = require_mapping(
        geometry.get("cylinder"),
        "geometry.cylinder",
    )

    grasp_geometry = require_mapping(
        geometry.get("grasp"),
        "geometry.grasp",
    )

    reference_center_z = finite_float(
        cylinder_geometry.get(
            "initial_center_z"
        ),
        "initial_center_z",
    )

    reference_grasp_z = finite_float(
        grasp_geometry.get(
            "cylinder_center_z"
        ),
        "reference grasp z",
    )

    grasp_offset_z = (
        reference_grasp_z
        - reference_center_z
    )

    grasp_z = (
        object_z
        + grasp_offset_z
    )

    posture_report = (
        output_directory
        / "link4_side_grasp_report.yaml"
    )

    run_step(
        "STEP 3 - LINK4 SIDE GRASP ANALYSIS",
        [
            sys.executable,
            str(
                SCRIPTS_DIR
                / "analyze_link4_side_grasp.py"
            ),
            "--model",
            str(KINEMATICS_MODEL_PATH),
            "--path-yaml",
            str(timed_path),
            "--grasp-z",
            f"{grasp_z:.9f}",
            "--report",
            str(posture_report),
        ],
    )

    if not posture_report.exists():
        raise RuntimeError(
            "LINK4 분석 보고서가 생성되지 않았습니다."
        )

    run_step(
        "STEP 4 - EXECUTOR DRY RUN",
        [
            sys.executable,
            str(
                SCRIPTS_DIR
                / "execute_box_move.py"
            ),
            "--from-box",
            str(arguments.from_box),
            "--to-box",
            str(arguments.to_box),
            "--timed-path",
            str(timed_path),
            "--speed-scale",
            f"{arguments.speed_scale:.9f}",
        ],
    )

    if not arguments.skip_build:
        run_step(
            "STEP 5 - COLCON BUILD",
            [
                "colcon",
                "build",
                "--packages-select",
                "drok_arm_mujoco",
                "drok_arm_control",
                "--symlink-install",
            ],
        )

        installed_position = (
            read_pickup_cube_position(
                INSTALL_SCENE_PATH
            )
        )

        verify_position(
            installed_position,
            expected_position,
            "Installed scene object",
        )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    manifest = {
        "version": 1,
        "status": "prepared",
        "mujoco_restart_required": True,
        "from_box": int(
            arguments.from_box
        ),
        "to_box": int(
            arguments.to_box
        ),
        "seed": int(
            arguments.seed
        ),
        "safety_margin": float(
            arguments.safety_margin
        ),
        "speed_scale": float(
            arguments.speed_scale
        ),
        "object_center": {
            "x": object_x,
            "y": object_y,
            "z": object_z,
        },
        "grasp_z": grasp_z,
        "timed_path": str(
            timed_path.resolve()
        ),
        "posture_report": str(
            posture_report.resolve()
        ),
        "output_directory": str(
            output_directory.resolve()
        ),
    }

    manifest_text = yaml.safe_dump(
        manifest,
        allow_unicode=True,
        sort_keys=False,
    )

    PREPARED_STATE_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    PREPARED_STATE_PATH.write_text(
        manifest_text,
        encoding="utf-8",
    )

    (
        output_directory
        / "prepared_random_move.yaml"
    ).write_text(
        manifest_text,
        encoding="utf-8",
    )

    execution_command = [
        "python3",
        "src/drok_arm_control/scripts/"
        "execute_box_move.py",
        "--from-box",
        str(arguments.from_box),
        "--to-box",
        str(arguments.to_box),
        "--timed-path",
        str(
            timed_path.relative_to(ROOT)
        ),
        "--speed-scale",
        f"{arguments.speed_scale:.3f}",
        "--execute",
        "--confirmation",
        "EXECUTE_MUJOCO_BOX_MOVE",
    ]

    print()
    print("=" * 80)
    print("RANDOM KNOWN-POSE MOVE PREPARATION: PASS")
    print("=" * 80)
    print(
        f"Move          : "
        f"BOX {arguments.from_box} "
        f"-> BOX {arguments.to_box}"
    )
    print(
        "Object center : "
        f"[{object_x:.6f}, "
        f"{object_y:.6f}, "
        f"{object_z:.6f}]"
    )
    print(
        f"Grasp z       : "
        f"{grasp_z:.6f}"
    )
    print(
        f"Timed path    : "
        f"{timed_path}"
    )
    print(
        f"Posture report: "
        f"{posture_report}"
    )
    print(
        f"Prepared state: "
        f"{PREPARED_STATE_PATH}"
    )
    print()
    print(
        "[중요] scene.xml의 원기둥 위치가 변경되었습니다."
    )
    print(
        "[중요] 실제 실행 전에 MuJoCo를 반드시 "
        "종료하고 재시작해야 합니다."
    )
    print()
    print("재시작 후 실제 실행 명령:")
    print()
    print("cd ~/IK_solver_MuJoCo")
    print("source /opt/ros/humble/setup.bash")
    print("source install/setup.bash")
    print()
    print(
        " \\\n  ".join(
            execution_command
        )
    )
    print("=" * 80)

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
