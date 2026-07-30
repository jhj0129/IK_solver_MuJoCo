#!/usr/bin/env python3

import argparse
import math
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import mujoco
import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

DEFAULT_MODEL_PATH = (
    ROOT
    / "src/drok_arm_mujoco/model/scene.xml"
)

DEFAULT_PATH_YAML = (
    ROOT
    / "generated_box_paths/box_2_to_1/timed_joint_path.yaml"
)

DEFAULT_REPORT_PATH = (
    ROOT
    / "analysis_reports/link4_side_grasp_baseline_box_2_to_1.yaml"
)

ARM_JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

LABEL_KEYS = (
    "name",
    "block_name",
    "block",
    "phase",
    "segment",
    "action",
)

POSITION_KEYS = (
    "positions",
    "joint_positions",
)


def object_name(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    object_id: int,
) -> str:
    name = mujoco.mj_id2name(
        model,
        object_type,
        object_id,
    )

    return name if name is not None else f"<unnamed:{object_id}>"


def find_named_id_case_insensitive(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    count: int,
    target_name: str,
) -> int:
    target_upper = target_name.upper()

    for object_id in range(count):
        name = object_name(
            model,
            object_type,
            object_id,
        )

        if name.upper() == target_upper:
            return object_id

    return -1


def is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
    )


def numeric_joint_vector(
    value: Any,
) -> Optional[List[float]]:
    if not isinstance(value, (list, tuple)):
        return None

    if len(value) < 6:
        return None

    first_six = value[:6]

    if not all(is_number(item) for item in first_six):
        return None

    return [float(item) for item in first_six]


def find_label(
    mapping: Dict[str, Any],
    inherited_label: str,
) -> str:
    for key in LABEL_KEYS:
        value = mapping.get(key)

        if isinstance(value, str) and value.strip():
            return value.strip()

    return inherited_label


def collect_joint_samples(
    node: Any,
    inherited_label: str = "UNLABELED",
    path: str = "root",
) -> Iterable[Tuple[str, str, List[float]]]:
    if isinstance(node, dict):
        current_label = find_label(
            node,
            inherited_label,
        )

        for position_key in POSITION_KEYS:
            if position_key not in node:
                continue

            raw_positions = node[position_key]
            vector = numeric_joint_vector(
                raw_positions
            )

            if vector is not None:
                yield (
                    current_label,
                    f"{path}.{position_key}",
                    vector,
                )

            elif isinstance(raw_positions, list):
                for index, item in enumerate(
                    raw_positions
                ):
                    vector = numeric_joint_vector(
                        item
                    )

                    if vector is not None:
                        yield (
                            current_label,
                            (
                                f"{path}.{position_key}"
                                f"[{index}]"
                            ),
                            vector,
                        )

        for key, value in node.items():
            if key in POSITION_KEYS:
                continue

            yield from collect_joint_samples(
                value,
                inherited_label=current_label,
                path=f"{path}.{key}",
            )

    elif isinstance(node, list):
        for index, item in enumerate(node):
            yield from collect_joint_samples(
                item,
                inherited_label=inherited_label,
                path=f"{path}[{index}]",
            )


def choose_tcp_frame(
    model: mujoco.MjModel,
) -> Tuple[str, int, str]:
    preferred_tokens = (
        "tcp",
        "tool",
        "end_effector",
        "endeffector",
        "ee",
        "grasp",
    )

    site_candidates: List[
        Tuple[int, int, str]
    ] = []

    for site_id in range(model.nsite):
        name = object_name(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            site_id,
        )

        lowered = name.lower()
        score = 0

        for rank, token in enumerate(
            preferred_tokens
        ):
            if token in lowered:
                score = max(
                    score,
                    100 - rank,
                )

        if score > 0:
            site_candidates.append(
                (score, site_id, name)
            )

    if site_candidates:
        site_candidates.sort(
            key=lambda item: (
                -item[0],
                item[2],
            )
        )

        _, site_id, name = site_candidates[0]

        return "site", site_id, name

    body_tokens = (
        "tcp",
        "tool",
        "end_effector",
        "endeffector",
        "gripper_base",
        "gripper",
        "link6",
    )

    body_candidates: List[
        Tuple[int, int, str]
    ] = []

    for body_id in range(1, model.nbody):
        name = object_name(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_id,
        )

        lowered = name.lower()
        score = 0

        for rank, token in enumerate(
            body_tokens
        ):
            if token in lowered:
                score = max(
                    score,
                    100 - rank,
                )

        if score > 0:
            body_candidates.append(
                (score, body_id, name)
            )

    if not body_candidates:
        raise RuntimeError(
            "TCP로 사용할 site/body를 자동으로 찾지 못했습니다."
        )

    body_candidates.sort(
        key=lambda item: (
            -item[0],
            item[2],
        )
    )

    _, body_id, name = body_candidates[0]

    return "body", body_id, name


def frame_pose(
    data: mujoco.MjData,
    frame_type: str,
    frame_id: int,
) -> Tuple[List[float], List[List[float]]]:
    if frame_type == "site":
        position = data.site_xpos[
            frame_id
        ].copy()

        rotation = data.site_xmat[
            frame_id
        ].reshape(3, 3).copy()

    elif frame_type == "body":
        position = data.xpos[
            frame_id
        ].copy()

        rotation = data.xmat[
            frame_id
        ].reshape(3, 3).copy()

    else:
        raise RuntimeError(
            f"지원하지 않는 frame type: {frame_type}"
        )

    return (
        [float(value) for value in position],
        [
            [float(value) for value in row]
            for row in rotation
        ],
    )


def set_arm_configuration(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_ids: Sequence[int],
    positions: Sequence[float],
) -> None:
    data.qpos[:] = model.qpos0

    for joint_id, position in zip(
        joint_ids,
        positions,
    ):
        qpos_address = int(
            model.jnt_qposadr[joint_id]
        )

        data.qpos[qpos_address] = float(
            position
        )

    mujoco.mj_forward(
        model,
        data,
    )


def angle_from_horizontal_degrees(
    approach_axis_z: float,
) -> float:
    clamped = max(
        -1.0,
        min(
            1.0,
            abs(approach_axis_z),
        ),
    )

    return math.degrees(
        math.asin(clamped)
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "기존 성공 Pick-and-Place 경로에서 "
            "JOINT4 body 높이와 TCP 접근축 수평도를 "
            "분석합니다."
        )
    )

    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
    )

    parser.add_argument(
        "--path-yaml",
        type=Path,
        default=DEFAULT_PATH_YAML,
    )

    parser.add_argument(
        "--grasp-z",
        type=float,
        default=0.282,
        help=(
            "기존 원기둥 측면 파지점의 world z. "
            "기본값 0.282 m"
        ),
    )

    parser.add_argument(
        "--report",
        type=Path,
        default=DEFAULT_REPORT_PATH,
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    if not arguments.model.exists():
        raise RuntimeError(
            f"MuJoCo 모델 파일이 없습니다: "
            f"{arguments.model}"
        )

    if not arguments.path_yaml.exists():
        raise RuntimeError(
            f"경로 YAML이 없습니다: "
            f"{arguments.path_yaml}"
        )

    model = mujoco.MjModel.from_xml_path(
        str(arguments.model)
    )

    data = mujoco.MjData(model)

    joint_ids: List[int] = []

    for joint_name in ARM_JOINT_NAMES:
        joint_id = find_named_id_case_insensitive(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            model.njnt,
            joint_name,
        )

        if joint_id < 0:
            raise RuntimeError(
                f"관절을 찾지 못했습니다: "
                f"{joint_name}"
            )

        joint_ids.append(joint_id)

    joint4_id = joint_ids[3]
    joint4_body_id = int(
        model.jnt_bodyid[joint4_id]
    )

    joint4_body_name = object_name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        joint4_body_id,
    )

    tcp_type, tcp_id, tcp_name = (
        choose_tcp_frame(model)
    )

    document = yaml.safe_load(
        arguments.path_yaml.read_text(
            encoding="utf-8"
        )
    )

    raw_samples = list(
        collect_joint_samples(document)
    )

    if not raw_samples:
        raise RuntimeError(
            "timed_joint_path.yaml에서 "
            "관절 position 샘플을 찾지 못했습니다."
        )

    phase_samples: "OrderedDict[str, List[Dict[str, Any]]]" = (
        OrderedDict()
    )

    for label, source_path, positions in raw_samples:
        set_arm_configuration(
            model,
            data,
            joint_ids,
            positions,
        )

        link4_position = data.xpos[
            joint4_body_id
        ].copy()

        tcp_position, tcp_rotation = frame_pose(
            data,
            tcp_type,
            tcp_id,
        )

        # 현재 grasp 설정의 접근축은 TCP local +X이다.
        approach_axis = [
            tcp_rotation[0][0],
            tcp_rotation[1][0],
            tcp_rotation[2][0],
        ]

        approach_axis_z = float(
            approach_axis[2]
        )

        sample = {
            "source_path": source_path,
            "joint_positions": positions,
            "link4_position": [
                float(value)
                for value in link4_position
            ],
            "link4_z": float(
                link4_position[2]
            ),
            "link4_grasp_z_offset": float(
                link4_position[2]
                - arguments.grasp_z
            ),
            "tcp_position": tcp_position,
            "tcp_approach_axis_world": (
                approach_axis
            ),
            "tcp_approach_axis_z": (
                approach_axis_z
            ),
            "tcp_tilt_from_horizontal_deg": (
                angle_from_horizontal_degrees(
                    approach_axis_z
                )
            ),
        }

        phase_samples.setdefault(
            label,
            [],
        ).append(sample)

    phase_report: Dict[str, Any] = {}

    print("=" * 76)
    print("LINK4 SIDE-GRASP BASELINE ANALYSIS")
    print("=" * 76)

    print(
        f"Model       : {arguments.model}"
    )

    print(
        f"Path        : {arguments.path_yaml}"
    )

    print(
        f"JOINT4 body : "
        f"{joint4_body_name} "
        f"(body id={joint4_body_id})"
    )

    print(
        f"TCP frame   : "
        f"{tcp_type}:{tcp_name} "
        f"(id={tcp_id})"
    )

    print(
        f"Grasp z     : "
        f"{arguments.grasp_z:.6f} m"
    )

    print(
        f"Samples     : {len(raw_samples)}"
    )

    print()

    for label, samples in phase_samples.items():
        link4_z_values = [
            item["link4_z"]
            for item in samples
        ]

        offset_values = [
            item["link4_grasp_z_offset"]
            for item in samples
        ]

        tilt_values = [
            item["tcp_tilt_from_horizontal_deg"]
            for item in samples
        ]

        axis_z_values = [
            item["tcp_approach_axis_z"]
            for item in samples
        ]

        summary = {
            "sample_count": len(samples),
            "link4_z": {
                "first": link4_z_values[0],
                "last": link4_z_values[-1],
                "minimum": min(link4_z_values),
                "maximum": max(link4_z_values),
            },
            "link4_grasp_z_offset": {
                "first": offset_values[0],
                "last": offset_values[-1],
                "minimum": min(offset_values),
                "maximum": max(offset_values),
            },
            "tcp_tilt_from_horizontal_deg": {
                "first": tilt_values[0],
                "last": tilt_values[-1],
                "minimum": min(tilt_values),
                "maximum": max(tilt_values),
            },
            "tcp_approach_axis_z": {
                "first": axis_z_values[0],
                "last": axis_z_values[-1],
                "minimum": min(axis_z_values),
                "maximum": max(axis_z_values),
            },
        }

        phase_report[label] = summary

        print("-" * 76)
        print(f"PHASE: {label}")
        print(f"  Samples             : {len(samples)}")

        print(
            "  LINK4 z first/last  : "
            f"{link4_z_values[0]:.6f} / "
            f"{link4_z_values[-1]:.6f} m"
        )

        print(
            "  LINK4 z min/max     : "
            f"{min(link4_z_values):.6f} / "
            f"{max(link4_z_values):.6f} m"
        )

        print(
            "  LINK4-grasp Δz end  : "
            f"{offset_values[-1]:+.6f} m"
        )

        print(
            "  TCP tilt end        : "
            f"{tilt_values[-1]:.3f}° "
            "from horizontal"
        )

        print(
            "  TCP |axis_z| max    : "
            f"{max(abs(value) for value in axis_z_values):.6f}"
        )

    global_link4_z = [
        sample["link4_z"]
        for samples in phase_samples.values()
        for sample in samples
    ]

    global_tilt = [
        sample["tcp_tilt_from_horizontal_deg"]
        for samples in phase_samples.values()
        for sample in samples
    ]

    report = {
        "analysis_version": 1,
        "model_path": str(
            arguments.model
        ),
        "trajectory_path": str(
            arguments.path_yaml
        ),
        "joint4": {
            "joint_name": ARM_JOINT_NAMES[3],
            "joint_id": joint4_id,
            "body_name": joint4_body_name,
            "body_id": joint4_body_id,
        },
        "tcp": {
            "frame_type": tcp_type,
            "frame_name": tcp_name,
            "frame_id": tcp_id,
            "approach_axis_local": "+X",
        },
        "reference_grasp_z": float(
            arguments.grasp_z
        ),
        "sample_count": len(raw_samples),
        "global": {
            "link4_z_minimum": min(
                global_link4_z
            ),
            "link4_z_maximum": max(
                global_link4_z
            ),
            "tcp_tilt_from_horizontal_minimum_deg": (
                min(global_tilt)
            ),
            "tcp_tilt_from_horizontal_maximum_deg": (
                max(global_tilt)
            ),
        },
        "phases": phase_report,
    }

    arguments.report.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    arguments.report.write_text(
        yaml.safe_dump(
            report,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    print()
    print("=" * 76)
    print("ANALYSIS RESULT: PASS")
    print("=" * 76)

    print(
        f"Report: {arguments.report}"
    )

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())

    except KeyboardInterrupt:
        raise SystemExit(130)

    except Exception as exception:
        print(
            f"[실패] {exception}"
        )

        raise SystemExit(1)
