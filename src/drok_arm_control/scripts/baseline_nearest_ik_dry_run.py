#!/usr/bin/env python3

from __future__ import annotations

import concurrent.futures
import math
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
import yaml

from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path.home() / "IK_solver_MuJoCo"

GEOMETRY_PATH = (
    ROOT
    / "src/drok_arm_kinematics/config/robot_geometry.yaml"
)

URDF_PATH = (
    ROOT
    / "src/drok_arm_mujoco/urdf/drok_arm_mujoco.urdf"
)

OUTPUT_PATH = (
    ROOT
    / "baseline_nearest_ik_plan.yaml"
)

JOINT_NAMES = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

# 기존 작업 좌표와 기존 기본 orientation만 사용한다.
# Cable limit, branch switch, neutral wrist는 사용하지 않는다.
WAYPOINTS = [
    {
        "name": "PICK_ABOVE",
        "position": [0.450, 0.180, 0.360],
        "rpy": [0.0, 0.0, 0.0],
    },
    {
        "name": "PICK_DOWN",
        "position": [0.450, 0.180, 0.272],
        "rpy": [0.0, 0.0, 0.0],
    },
    {
        "name": "PICK_RETREAT",
        "position": [0.450, 0.180, 0.360],
        "rpy": [0.0, 0.0, 0.0],
    },
    {
        "name": "PLACE_ABOVE",
        "position": [0.450, -0.180, 0.360],
        "rpy": [0.0, 0.0, 0.0],
    },
    {
        "name": "PLACE_DOWN",
        "position": [0.450, -0.180, 0.274],
        "rpy": [0.0, 0.0, 0.0],
    },
    {
        "name": "PLACE_RETREAT",
        "position": [0.450, -0.180, 0.360],
        "rpy": [0.0, 0.0, 0.0],
    },
]


class JointStateReader(Node):
    def __init__(self) -> None:
        super().__init__("baseline_nearest_ik_joint_reader")

        self.positions: Dict[str, float] = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            20,
        )

    def callback(self, message: JointState) -> None:
        for name, position in zip(
            message.name,
            message.position,
        ):
            if name in JOINT_NAMES:
                self.positions[name] = float(position)

    def ready(self) -> bool:
        return all(
            joint in self.positions
            for joint in JOINT_NAMES
        )

    def current_q(self) -> List[float]:
        return [
            self.positions[joint]
            for joint in JOINT_NAMES
        ]


def load_joint_limits(
    urdf_path: Path,
) -> Dict[str, Tuple[float, float]]:
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF 파일이 없습니다: {urdf_path}"
        )

    root = ET.parse(urdf_path).getroot()

    limits: Dict[str, Tuple[float, float]] = {}

    for joint_element in root.findall("joint"):
        name = joint_element.attrib.get("name", "")

        if name not in JOINT_NAMES:
            continue

        limit_element = joint_element.find("limit")

        if limit_element is None:
            raise RuntimeError(
                f"{name}에 limit 태그가 없습니다."
            )

        lower = float(
            limit_element.attrib["lower"]
        )
        upper = float(
            limit_element.attrib["upper"]
        )

        limits[name] = (lower, upper)

    missing = [
        joint
        for joint in JOINT_NAMES
        if joint not in limits
    ]

    if missing:
        raise RuntimeError(
            "URDF에서 joint limit을 찾지 못했습니다: "
            + ", ".join(missing)
        )

    return limits


def clip_to_limits(
    q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> List[float]:
    result = []

    for joint, value in zip(JOINT_NAMES, q):
        lower, upper = limits[joint]

        result.append(
            min(max(float(value), lower), upper)
        )

    return result


def halton_value(
    index: int,
    base: int,
) -> float:
    result = 0.0
    fraction = 1.0 / base
    current = index

    while current > 0:
        result += fraction * (current % base)
        current //= base
        fraction /= base

    return result


def make_seed_set(
    reference_q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> List[List[float]]:
    seeds: List[List[float]] = []

    def append_unique(seed: Sequence[float]) -> None:
        clipped = clip_to_limits(seed, limits)

        for existing in seeds:
            error = max(
                abs(a - b)
                for a, b in zip(existing, clipped)
            )

            if error < 1.0e-8:
                return

        seeds.append(clipped)

    # 가장 중요한 seed: 바로 이전 자세
    append_unique(reference_q)

    # 0 rad seed
    append_unique([0.0] * 6)

    # 현재 branch 주변 탐색
    for joint_index in range(6):
        for offset_deg in [-15.0, 15.0]:
            seed = list(reference_q)

            seed[joint_index] += math.radians(
                offset_deg
            )

            append_unique(seed)

    # 손목 branch 주변을 조금 더 넓게 탐색
    for joint_index in [3, 4, 5]:
        for offset_deg in [-45.0, 45.0]:
            seed = list(reference_q)

            seed[joint_index] += math.radians(
                offset_deg
            )

            append_unique(seed)

    # 전체 기계적 범위에 분산된 deterministic multi-start seed
    primes = [2, 3, 5, 7, 11, 13]

    for sample_index in range(1, 13):
        seed = []

        for joint_index, joint in enumerate(
            JOINT_NAMES
        ):
            lower, upper = limits[joint]

            ratio = halton_value(
                sample_index,
                primes[joint_index],
            )

            seed.append(
                lower + ratio * (upper - lower)
            )

        append_unique(seed)

    return seeds


def solve_one_seed(
    position: Sequence[float],
    rpy: Sequence[float],
    seed: Sequence[float],
) -> Optional[List[float]]:
    x, y, z = position
    roll, pitch, yaw = rpy

    command = [
        "ros2",
        "run",
        "drok_arm_kinematics",
        "solve_ik_pose",
        str(GEOMETRY_PATH),
        f"{x:.12f}",
        f"{y:.12f}",
        f"{z:.12f}",
        f"{roll:.12f}",
        f"{pitch:.12f}",
        f"{yaw:.12f}",
        *[
            f"{value:.12f}"
            for value in seed
        ],
    ]

    try:
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
        )
    except subprocess.TimeoutExpired:
        return None

    success_match = re.search(
        r"Success\s*:\s*(true|false)",
        result.stdout,
        flags=re.IGNORECASE,
    )

    joint_match = re.search(
        r"JOINT_RESULT=([^\r\n]+)",
        result.stdout,
    )

    if (
        result.returncode != 0
        or success_match is None
        or success_match.group(1).lower() != "true"
        or joint_match is None
    ):
        return None

    try:
        q = [
            float(value)
            for value
            in joint_match.group(1).split(",")
        ]
    except ValueError:
        return None

    if len(q) != 6:
        return None

    return q


def nearest_equivalent_angle(
    value: float,
    reference: float,
    lower: float,
    upper: float,
) -> Optional[float]:
    candidates = []

    for winding in range(-3, 4):
        candidate = (
            value
            + winding * 2.0 * math.pi
        )

        if (
            lower - 1.0e-8
            <= candidate
            <= upper + 1.0e-8
        ):
            candidates.append(candidate)

    if not candidates:
        return None

    return min(
        candidates,
        key=lambda candidate: abs(
            candidate - reference
        ),
    )


def normalize_candidate(
    candidate: Sequence[float],
    reference_q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> Optional[List[float]]:
    normalized = []

    for joint, value, reference in zip(
        JOINT_NAMES,
        candidate,
        reference_q,
    ):
        lower, upper = limits[joint]

        equivalent = nearest_equivalent_angle(
            float(value),
            float(reference),
            lower,
            upper,
        )

        if equivalent is None:
            return None

        normalized.append(equivalent)

    return normalized


def is_duplicate(
    solutions: Sequence[Sequence[float]],
    candidate: Sequence[float],
    tolerance: float = 1.0e-4,
) -> bool:
    return any(
        max(
            abs(a - b)
            for a, b in zip(solution, candidate)
        ) < tolerance
        for solution in solutions
    )


def candidate_score(
    candidate: Sequence[float],
    reference_q: Sequence[float],
) -> Tuple[float, float, float]:
    deltas = [
        abs(candidate_value - reference_value)
        for candidate_value, reference_value
        in zip(candidate, reference_q)
    ]

    # 1순위: 가장 많이 움직이는 단일 조인트 최소화
    maximum_delta = max(deltas)

    # 2순위: 전체 관절 이동량 합 최소화
    sum_delta = sum(deltas)

    # 3순위: 전체 제곱 이동량 최소화
    squared_delta = sum(
        delta * delta
        for delta in deltas
    )

    return (
        maximum_delta,
        sum_delta,
        squared_delta,
    )


def solve_all_candidates(
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> List[List[float]]:
    seeds = make_seed_set(
        reference_q,
        limits,
    )

    raw_solutions: List[Optional[List[float]]] = []

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=4
    ) as executor:
        futures = [
            executor.submit(
                solve_one_seed,
                position,
                rpy,
                seed,
            )
            for seed in seeds
        ]

        for future in concurrent.futures.as_completed(
            futures
        ):
            try:
                raw_solutions.append(
                    future.result()
                )
            except Exception:
                raw_solutions.append(None)

    candidates: List[List[float]] = []

    for raw_solution in raw_solutions:
        if raw_solution is None:
            continue

        normalized = normalize_candidate(
            raw_solution,
            reference_q,
            limits,
        )

        if normalized is None:
            continue

        if is_duplicate(
            candidates,
            normalized,
        ):
            continue

        candidates.append(normalized)

    candidates.sort(
        key=lambda candidate: candidate_score(
            candidate,
            reference_q,
        )
    )

    return candidates


def format_q_deg(
    q: Sequence[float],
) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+8.3f}°"
            for value in q
        )
        + "]"
    )


def print_candidate(
    rank: int,
    candidate: Sequence[float],
    reference_q: Sequence[float],
) -> None:
    deltas = [
        candidate_value - reference_value
        for candidate_value, reference_value
        in zip(candidate, reference_q)
    ]

    maximum_delta, sum_delta, _ = candidate_score(
        candidate,
        reference_q,
    )

    print()
    print(f"  Candidate {rank}")

    print(
        "    q     :",
        format_q_deg(candidate),
    )

    print(
        "    delta :",
        format_q_deg(deltas),
    )

    print(
        "    max Δ :",
        f"{math.degrees(maximum_delta):.3f}°",
    )

    print(
        "    sum |Δ|:",
        f"{math.degrees(sum_delta):.3f}°",
    )


def main() -> int:
    if not GEOMETRY_PATH.exists():
        print(
            "robot_geometry.yaml이 없습니다:",
            GEOMETRY_PATH,
        )
        return 1

    limits = load_joint_limits(
        URDF_PATH
    )

    print("=" * 96)
    print("DROK ARM BASELINE NEAREST-IK DRY RUN")
    print("=" * 96)
    print("Cable limit      : 사용 안 함")
    print("Wrist branch     : 사용 안 함")
    print("Neutral waypoint : 사용 안 함")
    print("Orientation scan : 사용 안 함")
    print("Mechanical limit : URDF 값만 사용")
    print("Robot execution  : 비활성화")
    print()

    print("URDF JOINT LIMITS")

    for joint in JOINT_NAMES:
        lower, upper = limits[joint]

        print(
            f"  {joint}: "
            f"{lower:+.6f} ~ {upper:+.6f} rad  "
            f"({math.degrees(lower):+.2f}° ~ "
            f"{math.degrees(upper):+.2f}°)"
        )

    rclpy.init()
    node = JointStateReader()

    try:
        deadline = time.monotonic() + 10.0

        while (
            rclpy.ok()
            and not node.ready()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if not node.ready():
            print()
            print(
                "/joint_states에서 JOINT1~6을 "
                "받지 못했습니다."
            )
            return 2

        start_q = node.current_q()
        reference_q = start_q.copy()

        print()
        print("START Q:")
        print(" ", format_q_deg(start_q))

        selected_waypoints = []
        total_joint_travel = [0.0] * 6
        largest_step = 0.0
        largest_step_name = ""

        for waypoint in WAYPOINTS:
            name = str(waypoint["name"])
            position = waypoint["position"]
            rpy = waypoint["rpy"]

            print()
            print("=" * 96)
            print(name)
            print("=" * 96)

            print(
                "Target position:",
                position,
            )

            print(
                "Target RPY     :",
                rpy,
            )

            print(
                "Reference q    :",
                format_q_deg(reference_q),
            )

            candidates = solve_all_candidates(
                position,
                rpy,
                reference_q,
                limits,
            )

            print(
                "Unique IK candidates:",
                len(candidates),
            )

            if not candidates:
                print()
                print(
                    f"{name}: 유효한 IK 후보를 "
                    "찾지 못했습니다."
                )
                return 3

            for rank, candidate in enumerate(
                candidates[:5],
                start=1,
            ):
                print_candidate(
                    rank,
                    candidate,
                    reference_q,
                )

            selected = candidates[0]

            step_deltas = [
                abs(selected_value - reference_value)
                for selected_value, reference_value
                in zip(selected, reference_q)
            ]

            step_maximum = max(step_deltas)

            if step_maximum > largest_step:
                largest_step = step_maximum
                largest_step_name = name

            for index, delta in enumerate(
                step_deltas
            ):
                total_joint_travel[index] += delta

            print()
            print("SELECTED:")
            print(" ", format_q_deg(selected))

            print(
                "SELECTED DELTA:",
                format_q_deg(
                    [
                        selected_value - reference_value
                        for selected_value, reference_value
                        in zip(selected, reference_q)
                    ]
                ),
            )

            print(
                "MAX JOINT DELTA:",
                f"{math.degrees(step_maximum):.3f}°",
            )

            selected_waypoints.append(
                {
                    "name": name,
                    "position": list(position),
                    "rpy": list(rpy),
                    "candidate_count": len(candidates),
                    "selected_q": [
                        float(value)
                        for value in selected
                    ],
                    "delta_from_previous": [
                        float(
                            selected_value
                            - reference_value
                        )
                        for selected_value, reference_value
                        in zip(selected, reference_q)
                    ],
                    "max_joint_delta": float(
                        step_maximum
                    ),
                }
            )

            # 가장 중요한 부분:
            # 이번에 선택한 해를 다음 waypoint의 기준으로 사용한다.
            reference_q = selected.copy()

        result = {
            "mode": "baseline_nearest_ik_dry_run",
            "robot_execution": False,
            "uses_cable_limits": False,
            "uses_branch_switch": False,
            "uses_neutral_waypoint": False,
            "uses_orientation_search": False,
            "selection_order": [
                "minimum_max_joint_delta",
                "minimum_sum_absolute_joint_delta",
                "minimum_sum_squared_joint_delta",
            ],
            "joint_names": JOINT_NAMES,
            "start_q": [
                float(value)
                for value in start_q
            ],
            "waypoints": selected_waypoints,
            "total_joint_travel": [
                float(value)
                for value in total_joint_travel
            ],
            "largest_step": {
                "waypoint": largest_step_name,
                "radians": float(largest_step),
                "degrees": float(
                    math.degrees(largest_step)
                ),
            },
        }

        with OUTPUT_PATH.open(
            "w",
            encoding="utf-8",
        ) as file:
            yaml.safe_dump(
                result,
                file,
                sort_keys=False,
                allow_unicode=True,
            )

        print()
        print("=" * 96)
        print("FINAL SUMMARY")
        print("=" * 96)

        print(
            "Total joint travel:",
            format_q_deg(total_joint_travel),
        )

        print(
            "Largest single waypoint step:",
            f"{math.degrees(largest_step):.3f}°",
            f"at {largest_step_name}",
        )

        print()
        print("Dry-run plan saved:")
        print(OUTPUT_PATH)

        if largest_step > math.radians(90.0):
            print()
            print(
                "RESULT: WARNING — 90°를 넘는 "
                "단일 관절 이동이 있습니다."
            )
            return 4

        print()
        print(
            "RESULT: PASS — 각 waypoint가 직전 해를 "
            "기준으로 연결됐습니다."
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
