#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

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


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--hybrid-path",
        default=str(
            ROOT
            / "cylinder_candidate_21_hybrid_path.yaml"
        ),
    )

    parser.add_argument(
        "--retreat-path",
        default=str(
            ROOT
            / "cylinder_place_retreat_full_path.yaml"
        ),
    )

    parser.add_argument(
        "--urdf",
        default=str(
            ROOT
            / "src/drok_arm_mujoco"
            / "urdf/drok_arm_mujoco.urdf"
        ),
    )

    parser.add_argument(
        "--output",
        default=str(
            ROOT
            / "cylinder_full_cartesian_ik_path.yaml"
        ),
    )

    return parser.parse_args()


def load_yaml(path: Path) -> Dict[str, Any]:
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


def read_joint_limits(
    urdf_path: Path,
) -> List[Tuple[float, float]]:
    root = ET.parse(
        urdf_path
    ).getroot()

    joint_map = {
        joint.attrib.get("name", ""): joint
        for joint in root.findall("joint")
    }

    limits: List[Tuple[float, float]] = []

    for joint_name in ARM_JOINT_NAMES:
        joint = joint_map.get(joint_name)

        if joint is None:
            raise RuntimeError(
                f"URDF에서 {joint_name}을 "
                "찾지 못했습니다."
            )

        limit = joint.find("limit")

        if limit is None:
            raise RuntimeError(
                f"{joint_name} limit이 없습니다."
            )

        lower = float(
            limit.attrib["lower"]
        )

        upper = float(
            limit.attrib["upper"]
        )

        if not (
            math.isfinite(lower)
            and math.isfinite(upper)
            and lower < upper
        ):
            raise RuntimeError(
                f"{joint_name} limit이 잘못되었습니다."
            )

        limits.append(
            (
                lower,
                upper,
            )
        )

    return limits


def maximum_difference(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    if len(first) != len(second):
        raise RuntimeError(
            "관절 벡터 크기가 다릅니다."
        )

    return max(
        abs(a - b)
        for a, b in zip(
            first,
            second,
        )
    )


def minimum_limit_margin(
    joint_positions: Sequence[float],
    limits: Sequence[Tuple[float, float]],
) -> float:
    return min(
        min(
            value - lower,
            upper - value,
        )
        for value, (lower, upper)
        in zip(
            joint_positions,
            limits,
        )
    )


def as_vector(
    record: Dict[str, Any],
    key: str,
) -> np.ndarray:
    vector = np.asarray(
        record[key],
        dtype=float,
    )

    if vector.shape != (3,):
        raise RuntimeError(
            f"{key}가 3차원 벡터가 아닙니다."
        )

    if not np.all(
        np.isfinite(vector)
    ):
        raise RuntimeError(
            f"{key}에 유한하지 않은 값이 있습니다."
        )

    return vector


def normalize_record(
    source: Dict[str, Any],
    segment: str | None = None,
    ik_mode: str | None = None,
) -> Dict[str, Any]:
    record = dict(source)

    if segment is not None:
        record["segment"] = segment

    if ik_mode is not None:
        record["ik_mode"] = ik_mode

    record["joint_positions"] = [
        float(value)
        for value in record["joint_positions"]
    ]

    if "position" in record:
        record["position"] = [
            float(value)
            for value in record["position"]
        ]

    return record


def main() -> int:
    arguments = parse_arguments()

    hybrid_path = Path(
        arguments.hybrid_path
    ).expanduser().resolve()

    retreat_path = Path(
        arguments.retreat_path
    ).expanduser().resolve()

    urdf_path = Path(
        arguments.urdf
    ).expanduser().resolve()

    output_path = Path(
        arguments.output
    ).expanduser().resolve()

    hybrid = load_yaml(
        hybrid_path
    )

    retreat = load_yaml(
        retreat_path
    )

    limits = read_joint_limits(
        urdf_path
    )

    hybrid_records = hybrid.get("path", [])
    retreat_records = retreat.get("path", [])

    if not hybrid_records:
        raise RuntimeError(
            "Hybrid path가 비어 있습니다."
        )

    if len(retreat_records) < 2:
        raise RuntimeError(
            "Retreat path sample이 부족합니다."
        )

    hybrid_last = hybrid_records[-1]
    retreat_first = retreat_records[0]

    hybrid_last_position = as_vector(
        hybrid_last,
        "position",
    )

    retreat_first_position = as_vector(
        retreat_first,
        "position",
    )

    position_gap = float(
        np.linalg.norm(
            hybrid_last_position
            - retreat_first_position
        )
    )

    joint_gap = maximum_difference(
        hybrid_last["joint_positions"],
        retreat_first["joint_positions"],
    )

    # 두 경로의 연결 위치는 동일해야 한다.
    if position_gap > 1.0e-8:
        raise RuntimeError(
            "PLACE_DESCEND와 PLACE_RETREAT의 "
            f"위치가 연결되지 않습니다: "
            f"{position_gap:.9e} m"
        )

    # PLACE에서 full-pose alignment에 의한
    # 작은 수치 차이만 허용한다.
    if joint_gap > math.radians(0.1):
        raise RuntimeError(
            "PLACE 연결점의 관절 차이가 큽니다: "
            f"{math.degrees(joint_gap):.6f} deg"
        )

    merged_records: List[Dict[str, Any]] = []

    for record in hybrid_records:
        normalized = normalize_record(
            record
        )

        if "ik_mode" not in normalized:
            normalized["ik_mode"] = "entry"

        merged_records.append(
            normalized
        )

    # retreat record 0은 PLACE_GRASP와 중복이므로 제거한다.
    for record in retreat_records[1:]:
        merged_records.append(
            normalize_record(
                record,
                segment="PLACE_RETREAT",
                ik_mode="full",
            )
        )

    maximum_joint_jump = 0.0
    minimum_margin = math.inf

    previous_q: List[float] | None = None

    segment_ranges: Dict[str, Dict[str, int]] = {}

    for global_index, record in enumerate(
        merged_records
    ):
        record["global_index"] = global_index

        segment = str(
            record["segment"]
        )

        if segment not in segment_ranges:
            segment_ranges[segment] = {
                "start_index": global_index,
                "end_index": global_index,
            }
        else:
            segment_ranges[segment][
                "end_index"
            ] = global_index

        q = [
            float(value)
            for value in record[
                "joint_positions"
            ]
        ]

        if len(q) != len(ARM_JOINT_NAMES):
            raise RuntimeError(
                f"global index {global_index}의 "
                "관절 벡터 크기가 잘못됐습니다."
            )

        margin = minimum_limit_margin(
            q,
            limits,
        )

        minimum_margin = min(
            minimum_margin,
            margin,
        )

        if margin <= 0.0:
            raise RuntimeError(
                f"global index {global_index}에서 "
                "관절 제한을 위반했습니다."
            )

        if previous_q is not None:
            jump = maximum_difference(
                q,
                previous_q,
            )

            maximum_joint_jump = max(
                maximum_joint_jump,
                jump,
            )

            record[
                "maximum_joint_jump_from_previous_rad"
            ] = jump

        previous_q = q

    required_segments = [
        "PICK_PREGRASP",
        "PICK_APPROACH",
        "PICK_LIFT",
        "TRANSFER",
        "PLACE_DESCEND",
        "PLACE_RETREAT",
    ]

    missing_segments = [
        segment
        for segment in required_segments
        if segment not in segment_ranges
    ]

    if missing_segments:
        raise RuntimeError(
            "필수 segment가 없습니다: "
            + ", ".join(missing_segments)
        )

    close_index = segment_ranges[
        "PICK_APPROACH"
    ]["end_index"]

    open_index = segment_ranges[
        "PLACE_DESCEND"
    ]["end_index"]

    events = [
        {
            "type": "GRIPPER_CLOSE",
            "after_global_index": (
                close_index
            ),
            "description": (
                "PICK_APPROACH가 완료된 후 "
                "원기둥을 파지한다."
            ),
        },
        {
            "type": "GRIPPER_OPEN",
            "after_global_index": (
                open_index
            ),
            "description": (
                "PLACE_DESCEND가 완료된 후 "
                "원기둥을 놓는다."
            ),
        },
    ]

    initial_joint_positions = [
        float(value)
        for value in hybrid[
            "initial_joint_positions"
        ]
    ]

    pregrasp_joint_positions = [
        float(value)
        for value in merged_records[0][
            "joint_positions"
        ]
    ]

    entry_joint_difference = (
        maximum_difference(
            initial_joint_positions,
            pregrasp_joint_positions,
        )
    )

    output_document = {
        "version": 1,
        "joint_names": ARM_JOINT_NAMES,
        "candidate_index": int(
            hybrid["candidate_index"]
        ),
        "candidate_phi_deg": float(
            hybrid["phi_deg"]
        ),
        "source_files": {
            "hybrid_path": str(
                hybrid_path
            ),
            "retreat_path": str(
                retreat_path
            ),
            "urdf": str(
                urdf_path
            ),
        },
        "runtime_entry": {
            "mode": "joint_space_quintic",
            "start_source": "/joint_states",
            "goal_global_index": 0,
            "goal_joint_positions": (
                pregrasp_joint_positions
            ),
            "dry_run_initial_joint_positions": (
                initial_joint_positions
            ),
            "dry_run_maximum_joint_change_rad": (
                entry_joint_difference
            ),
            "dry_run_maximum_joint_change_deg": (
                math.degrees(
                    entry_joint_difference
                )
            ),
        },
        "events": events,
        "segment_ranges": segment_ranges,
        "summary": {
            "cartesian_sample_count": (
                len(merged_records)
            ),
            "maximum_joint_jump_rad": (
                maximum_joint_jump
            ),
            "maximum_joint_jump_deg": (
                math.degrees(
                    maximum_joint_jump
                )
            ),
            "minimum_joint_limit_margin_rad": (
                minimum_margin
            ),
            "minimum_joint_limit_margin_deg": (
                math.degrees(
                    minimum_margin
                )
            ),
            "place_connection_position_gap_m": (
                position_gap
            ),
            "place_connection_joint_gap_rad": (
                joint_gap
            ),
            "place_connection_joint_gap_deg": (
                math.degrees(joint_gap)
            ),
        },
        "path": merged_records,
    }

    output_path.write_text(
        yaml.safe_dump(
            output_document,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    print("=" * 88)
    print("CYLINDER FULL CARTESIAN IK PATH")
    print("=" * 88)

    print(
        f"Samples             : "
        f"{len(merged_records)}"
    )

    print(
        "Maximum joint jump  : "
        f"{math.degrees(maximum_joint_jump):.6f} deg"
    )

    print(
        "Minimum limit margin: "
        f"{math.degrees(minimum_margin):.6f} deg"
    )

    print(
        "PLACE joint gap     : "
        f"{math.degrees(joint_gap):.6f} deg"
    )

    print()
    print("Segment ranges")

    for segment in required_segments:
        interval = segment_ranges[
            segment
        ]

        print(
            f"  {segment:<16} "
            f"{interval['start_index']:3d}"
            f" -> "
            f"{interval['end_index']:3d}"
        )

    print()
    print("Gripper events")

    for event in events:
        print(
            f"  {event['type']:<14} "
            f"after index "
            f"{event['after_global_index']}"
        )

    print()
    print("Saved:")
    print(output_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
