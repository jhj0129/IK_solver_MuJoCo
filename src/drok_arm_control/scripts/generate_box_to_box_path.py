#!/usr/bin/env python3

import argparse
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

SCRIPT_DIR = (
    ROOT
    / "src/drok_arm_control/scripts"
)

CONTROL_CONFIG_DIR = (
    ROOT
    / "src/drok_arm_control/config"
)

MUJOCO_DIR = (
    ROOT
    / "src/drok_arm_mujoco"
)

KINEMATICS_CONFIG_DIR = (
    ROOT
    / "src/drok_arm_kinematics/config"
)

BOX_LAYOUT_PATH = (
    CONTROL_CONFIG_DIR
    / "box_layout.yaml"
)

BASE_SCENE_PATH = (
    MUJOCO_DIR
    / "model/scene.xml"
)

BASE_GRASP_CONFIG_PATH = (
    CONTROL_CONFIG_DIR
    / "cylinder_grasp_geometry.yaml"
)

ROBOT_GEOMETRY_PATH = (
    KINEMATICS_CONFIG_DIR
    / "robot_geometry.yaml"
)

URDF_PATH = (
    MUJOCO_DIR
    / "urdf/drok_arm_mujoco.urdf"
)

CONTROLLERS_PATH = (
    MUJOCO_DIR
    / "config/controllers.yaml"
)

TIMING_CONFIG_PATH = (
    CONTROL_CONFIG_DIR
    / "cylinder_trajectory_timing.yaml"
)


GENERATE_CANDIDATES_SCRIPT = (
    SCRIPT_DIR
    / "generate_cylinder_grasp_candidates.py"
)

HYBRID_IK_SCRIPT = (
    SCRIPT_DIR
    / "dry_run_cylinder_hybrid_ik.py"
)

GENERATE_RETREAT_TARGET_SCRIPT = (
    SCRIPT_DIR
    / "generate_cylinder_place_retreat_target.py"
)

RETREAT_IK_SCRIPT = (
    SCRIPT_DIR
    / "dry_run_cylinder_place_retreat.py"
)

ASSEMBLE_SCRIPT = (
    SCRIPT_DIR
    / "assemble_cylinder_full_path.py"
)

TIME_PARAMETERIZE_SCRIPT = (
    SCRIPT_DIR
    / "time_parameterize_cylinder_path.py"
)


def load_yaml(path: Path) -> Dict[str, Any]:
    document = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    if not isinstance(document, dict):
        raise RuntimeError(
            f"YAML root가 dictionary가 아닙니다: {path}"
        )

    return document


def save_yaml(
    path: Path,
    document: Dict[str, Any],
) -> None:
    path.write_text(
        yaml.safe_dump(
            document,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def normalized_boxes(
    document: Dict[str, Any],
) -> Dict[int, Dict[str, Any]]:
    raw_boxes = document.get("boxes")

    if not isinstance(raw_boxes, dict):
        raise RuntimeError(
            "box_layout.yaml에 boxes가 없습니다."
        )

    boxes: Dict[int, Dict[str, Any]] = {}

    for key, value in raw_boxes.items():
        box_id = int(key)

        if not isinstance(value, dict):
            raise RuntimeError(
                f"BOX {box_id} 설정이 올바르지 않습니다."
            )

        boxes[box_id] = value

    return boxes


def require_files(paths: Sequence[Path]) -> None:
    missing = [
        str(path)
        for path in paths
        if not path.exists()
    ]

    if missing:
        raise RuntimeError(
            "필요한 파일이 없습니다:\n"
            + "\n".join(missing)
        )


def make_planning_scene(
    source_box: Dict[str, Any],
    cylinder_center_z: float,
    output_path: Path,
) -> None:
    tree = ET.parse(BASE_SCENE_PATH)
    root = tree.getroot()

    body = root.find(
        ".//body[@name='pickup_cube']"
    )

    if body is None:
        raise RuntimeError(
            "scene.xml에서 pickup_cube body를 "
            "찾지 못했습니다."
        )

    position = source_box["position"]

    source_x = float(position["x"])
    source_y = float(position["y"])

    body.set(
        "pos",
        (
            f"{source_x:.6f} "
            f"{source_y:.6f} "
            f"{cylinder_center_z:.6f}"
        ),
    )

    ET.indent(
        tree,
        space="  ",
    )

    tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )


def make_grasp_config(
    source_box: Dict[str, Any],
    target_box: Dict[str, Any],
    output_path: Path,
) -> None:
    document = load_yaml(
        BASE_GRASP_CONFIG_PATH
    )

    scene = document.get("scene")

    if not isinstance(scene, dict):
        raise RuntimeError(
            "cylinder_grasp_geometry.yaml의 "
            "scene 설정이 올바르지 않습니다."
        )

    scene["pickup_object_body"] = "pickup_cube"

    scene["pickup_pedestal_geom"] = str(
        source_box["geom_name"]
    )

    scene["place_pedestal_geom"] = str(
        target_box["geom_name"]
    )

    save_yaml(
        output_path,
        document,
    )


def collect_candidate_indices(
    value: Any,
    result: set[int],
) -> None:
    if isinstance(value, dict):
        candidate_index = value.get(
            "candidate_index"
        )

        if isinstance(candidate_index, int):
            result.add(candidate_index)

        for child in value.values():
            collect_candidate_indices(
                child,
                result,
            )

    elif isinstance(value, list):
        for child in value:
            collect_candidate_indices(
                child,
                result,
            )


def run_command(
    label: str,
    command: Sequence[str],
    log_path: Path,
) -> Tuple[bool, str]:
    print(
        f"  [{label}] 실행",
        flush=True,
    )

    result = subprocess.run(
        list(command),
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        check=False,
    )

    output = result.stdout or ""

    log_path.write_text(
        "$ "
        + " ".join(command)
        + "\n\n"
        + output,
        encoding="utf-8",
    )

    if result.returncode == 0:
        print(
            f"  [{label}] PASS",
            flush=True,
        )

        return True, output

    print(
        f"  [{label}] FAIL "
        f"(status={result.returncode})",
        flush=True,
    )

    output_lines = output.strip().splitlines()

    for line in output_lines[-8:]:
        print(
            f"    {line}",
            flush=True,
        )

    return False, output


def read_path_score(
    full_path: Path,
) -> Tuple[float, float]:
    document = load_yaml(full_path)

    summary = document.get(
        "summary",
        {},
    )

    if not isinstance(summary, dict):
        return (
            float("-inf"),
            float("inf"),
        )

    margin = float(
        summary.get(
            "minimum_joint_limit_margin_rad",
            float("-inf"),
        )
    )

    maximum_jump = float(
        summary.get(
            "maximum_joint_jump_rad",
            float("inf"),
        )
    )

    return (
        margin,
        maximum_jump,
    )


def copy_selected_outputs(
    selected_directory: Path,
    candidate_file: Path,
    output_directory: Path,
) -> None:
    output_mapping = {
        candidate_file: (
            output_directory
            / "grasp_candidates.yaml"
        ),
        selected_directory
        / "hybrid_path.yaml": (
            output_directory
            / "hybrid_path.yaml"
        ),
        selected_directory
        / "retreat_target.yaml": (
            output_directory
            / "retreat_target.yaml"
        ),
        selected_directory
        / "retreat_path.yaml": (
            output_directory
            / "retreat_path.yaml"
        ),
        selected_directory
        / "full_cartesian_path.yaml": (
            output_directory
            / "full_cartesian_path.yaml"
        ),
        selected_directory
        / "timed_joint_path.yaml": (
            output_directory
            / "timed_joint_path.yaml"
        ),
    }

    for source, destination in output_mapping.items():
        if not source.exists():
            raise RuntimeError(
                f"선택 결과 파일이 없습니다: {source}"
            )

        shutil.copy2(
            source,
            destination,
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "BOX 1~4 중 지정된 출발 상자에서 "
            "도착 상자로 이동하는 Pick-and-Place "
            "경로를 생성합니다."
        )
    )

    parser.add_argument(
        "--from-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
        help="원기둥이 현재 놓여 있는 상자",
    )

    parser.add_argument(
        "--to-box",
        type=int,
        required=True,
        choices=[1, 2, 3, 4],
        help="원기둥을 옮길 목표 상자",
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
        help="기존 생성 폴더를 삭제하고 다시 만듭니다.",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    if arguments.from_box == arguments.to_box:
        print(
            "출발 상자와 도착 상자가 같습니다.",
            file=sys.stderr,
        )

        return 2

    require_files(
        [
            BOX_LAYOUT_PATH,
            BASE_SCENE_PATH,
            BASE_GRASP_CONFIG_PATH,
            ROBOT_GEOMETRY_PATH,
            URDF_PATH,
            CONTROLLERS_PATH,
            TIMING_CONFIG_PATH,
            GENERATE_CANDIDATES_SCRIPT,
            HYBRID_IK_SCRIPT,
            GENERATE_RETREAT_TARGET_SCRIPT,
            RETREAT_IK_SCRIPT,
            ASSEMBLE_SCRIPT,
            TIME_PARAMETERIZE_SCRIPT,
        ]
    )

    layout = load_yaml(
        BOX_LAYOUT_PATH
    )

    boxes = normalized_boxes(layout)

    source_box = boxes[
        arguments.from_box
    ]

    target_box = boxes[
        arguments.to_box
    ]

    cylinder_center_z = float(
        layout["geometry"]["cylinder"][
            "initial_center_z"
        ]
    )

    if arguments.output_dir is None:
        output_directory = (
            ROOT
            / "generated_box_paths"
            / (
                f"box_{arguments.from_box}"
                f"_to_{arguments.to_box}"
            )
        )
    else:
        output_directory = (
            arguments.output_dir.expanduser().resolve()
        )

    if output_directory.exists():
        existing_files = list(
            output_directory.iterdir()
        )

        if existing_files:
            if not arguments.overwrite:
                print(
                    "출력 폴더가 이미 존재합니다:",
                    output_directory,
                    file=sys.stderr,
                )

                print(
                    "--overwrite를 사용하십시오.",
                    file=sys.stderr,
                )

                return 2

            shutil.rmtree(
                output_directory
            )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    work_directory = (
        output_directory
        / "candidate_trials"
    )

    work_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    planning_scene = (
        output_directory
        / "planning_scene.xml"
    )

    planning_grasp_config = (
        output_directory
        / "grasp_geometry.yaml"
    )

    candidate_file = (
        output_directory
        / "all_grasp_candidates.yaml"
    )

    make_planning_scene(
        source_box=source_box,
        cylinder_center_z=cylinder_center_z,
        output_path=planning_scene,
    )

    make_grasp_config(
        source_box=source_box,
        target_box=target_box,
        output_path=planning_grasp_config,
    )

    print("=" * 72)
    print("BOX-TO-BOX PATH GENERATION")
    print("=" * 72)

    print(
        f"From BOX : {arguments.from_box} "
        f"({source_box['geom_name']})"
    )

    print(
        f"To BOX   : {arguments.to_box} "
        f"({target_box['geom_name']})"
    )

    print(
        f"Output   : {output_directory}"
    )

    print()

    success, _ = run_command(
        "GRASP CANDIDATES",
        [
            sys.executable,
            str(
                GENERATE_CANDIDATES_SCRIPT
            ),
            "--scene",
            str(planning_scene),
            "--config",
            str(planning_grasp_config),
            "--output",
            str(candidate_file),
        ],
        output_directory
        / "generate_candidates.log",
    )

    if not success:
        print(
            "Grasp candidate 생성 실패",
            file=sys.stderr,
        )

        return 1

    candidate_document = load_yaml(
        candidate_file
    )

    detected_indices: set[int] = set()

    collect_candidate_indices(
        candidate_document,
        detected_indices,
    )

    if arguments.candidate_index is not None:
        candidate_indices = [
            arguments.candidate_index
        ]
    elif detected_indices:
        candidate_indices = sorted(
            detected_indices
        )
    else:
        candidate_count = int(
            load_yaml(
                planning_grasp_config
            )
            .get("search", {})
            .get(
                "azimuth_candidate_count",
                36,
            )
        )

        candidate_indices = list(
            range(candidate_count)
        )

    print()
    print(
        "Candidate indices:",
        candidate_indices,
    )

    successful_candidates: List[
        Dict[str, Any]
    ] = []

    failed_candidates: List[
        Dict[str, Any]
    ] = []

    for candidate_index in candidate_indices:
        print()
        print("-" * 72)
        print(
            f"CANDIDATE {candidate_index}"
        )
        print("-" * 72)

        candidate_directory = (
            work_directory
            / f"candidate_{candidate_index:02d}"
        )

        candidate_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        hybrid_path = (
            candidate_directory
            / "hybrid_path.yaml"
        )

        retreat_target = (
            candidate_directory
            / "retreat_target.yaml"
        )

        retreat_path = (
            candidate_directory
            / "retreat_path.yaml"
        )

        full_path = (
            candidate_directory
            / "full_cartesian_path.yaml"
        )

        timed_path = (
            candidate_directory
            / "timed_joint_path.yaml"
        )

        pipeline_steps = [
            (
                "HYBRID IK",
                [
                    sys.executable,
                    str(HYBRID_IK_SCRIPT),
                    "--candidate-file",
                    str(candidate_file),
                    "--candidate-index",
                    str(candidate_index),
                    "--geometry",
                    str(ROBOT_GEOMETRY_PATH),
                    "--urdf",
                    str(URDF_PATH),
                    "--output",
                    str(hybrid_path),
                ],
                candidate_directory
                / "01_hybrid_ik.log",
            ),
            (
                "RETREAT TARGET",
                [
                    sys.executable,
                    str(
                        GENERATE_RETREAT_TARGET_SCRIPT
                    ),
                    "--hybrid-path",
                    str(hybrid_path),
                    "--candidate-file",
                    str(candidate_file),
                    "--geometry",
                    str(ROBOT_GEOMETRY_PATH),
                    "--output",
                    str(retreat_target),
                ],
                candidate_directory
                / "02_retreat_target.log",
            ),
            (
                "RETREAT IK",
                [
                    sys.executable,
                    str(RETREAT_IK_SCRIPT),
                    "--target",
                    str(retreat_target),
                    "--geometry",
                    str(ROBOT_GEOMETRY_PATH),
                    "--urdf",
                    str(URDF_PATH),
                    "--output",
                    str(retreat_path),
                ],
                candidate_directory
                / "03_retreat_ik.log",
            ),
            (
                "ASSEMBLE",
                [
                    sys.executable,
                    str(ASSEMBLE_SCRIPT),
                    "--hybrid-path",
                    str(hybrid_path),
                    "--retreat-path",
                    str(retreat_path),
                    "--urdf",
                    str(URDF_PATH),
                    "--output",
                    str(full_path),
                ],
                candidate_directory
                / "04_assemble.log",
            ),
            (
                "TIME PARAMETERIZATION",
                [
                    sys.executable,
                    str(
                        TIME_PARAMETERIZE_SCRIPT
                    ),
                    "--path",
                    str(full_path),
                    "--timing-config",
                    str(TIMING_CONFIG_PATH),
                    "--urdf",
                    str(URDF_PATH),
                    "--controllers",
                    str(CONTROLLERS_PATH),
                    "--output",
                    str(timed_path),
                ],
                candidate_directory
                / "05_time_parameterization.log",
            ),
        ]

        failed_stage: Optional[str] = None

        for (
            stage_name,
            command,
            log_path,
        ) in pipeline_steps:
            stage_success, _ = run_command(
                stage_name,
                command,
                log_path,
            )

            if not stage_success:
                failed_stage = stage_name
                break

        if failed_stage is not None:
            failed_candidates.append(
                {
                    "candidate_index": (
                        candidate_index
                    ),
                    "failed_stage": failed_stage,
                }
            )

            continue

        margin, maximum_jump = (
            read_path_score(full_path)
        )

        successful_candidates.append(
            {
                "candidate_index": (
                    candidate_index
                ),
                "directory": str(
                    candidate_directory
                ),
                "minimum_joint_limit_margin_rad": (
                    margin
                ),
                "minimum_joint_limit_margin_deg": (
                    math.degrees(margin)
                ),
                "maximum_joint_jump_rad": (
                    maximum_jump
                ),
                "maximum_joint_jump_deg": (
                    math.degrees(maximum_jump)
                ),
            }
        )

        print(
            "  Candidate result: PASS"
        )

        print(
            "  Minimum margin : "
            f"{math.degrees(margin):.3f}°"
        )

        print(
            "  Maximum jump   : "
            f"{math.degrees(maximum_jump):.3f}°"
        )

    if not successful_candidates:
        report = {
            "version": 1,
            "from_box": arguments.from_box,
            "to_box": arguments.to_box,
            "result": "FAIL",
            "successful_candidates": [],
            "failed_candidates": (
                failed_candidates
            ),
        }

        save_yaml(
            output_directory
            / "generation_report.yaml",
            report,
        )

        print()
        print(
            "사용 가능한 경로를 생성하지 못했습니다.",
            file=sys.stderr,
        )

        print(
            "실패 로그:",
            work_directory,
            file=sys.stderr,
        )

        return 1

    successful_candidates.sort(
        key=lambda item: (
            -float(
                item[
                    "minimum_joint_limit_margin_rad"
                ]
            ),
            float(
                item[
                    "maximum_joint_jump_rad"
                ]
            ),
        )
    )

    selected = successful_candidates[0]

    selected_directory = Path(
        selected["directory"]
    )

    copy_selected_outputs(
        selected_directory=selected_directory,
        candidate_file=candidate_file,
        output_directory=output_directory,
    )

    report = {
        "version": 1,
        "from_box": arguments.from_box,
        "to_box": arguments.to_box,
        "source_geom": (
            source_box["geom_name"]
        ),
        "target_geom": (
            target_box["geom_name"]
        ),
        "result": "PASS",
        "selected_candidate": selected,
        "successful_candidates": (
            successful_candidates
        ),
        "failed_candidates": (
            failed_candidates
        ),
        "output_files": {
            "grasp_candidates": (
                "grasp_candidates.yaml"
            ),
            "hybrid_path": (
                "hybrid_path.yaml"
            ),
            "retreat_target": (
                "retreat_target.yaml"
            ),
            "retreat_path": (
                "retreat_path.yaml"
            ),
            "full_cartesian_path": (
                "full_cartesian_path.yaml"
            ),
            "timed_joint_path": (
                "timed_joint_path.yaml"
            ),
        },
    }

    save_yaml(
        output_directory
        / "generation_report.yaml",
        report,
    )

    print()
    print("=" * 72)
    print("BOX-TO-BOX PATH GENERATION: PASS")
    print("=" * 72)

    print(
        "Selected candidate :",
        selected["candidate_index"],
    )

    print(
        "Minimum margin     : "
        f"{selected['minimum_joint_limit_margin_deg']:.3f}°"
    )

    print(
        "Maximum joint jump : "
        f"{selected['maximum_joint_jump_deg']:.3f}°"
    )

    print(
        "Timed path         :",
        output_directory
        / "timed_joint_path.yaml",
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
