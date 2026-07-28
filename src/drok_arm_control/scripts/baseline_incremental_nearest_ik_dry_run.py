#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import rclpy
import yaml

import baseline_nearest_ik_dry_run as base


ROOT = Path.home() / "IK_solver_MuJoCo"

OUTPUT_PATH = (
    ROOT
    / "baseline_incremental_nearest_ik_plan.yaml"
)

TRANSFER_SEGMENTS = 18

FIXED_RPY = [
    0.0,
    0.0,
    0.0,
]


def make_waypoints() -> List[dict]:
    waypoints = [
        {
            "name": "PICK_ABOVE",
            "position": [0.450, 0.180, 0.360],
            "rpy": FIXED_RPY,
            "phase": "pick",
        },
        {
            "name": "PICK_DOWN",
            "position": [0.450, 0.180, 0.272],
            "rpy": FIXED_RPY,
            "phase": "pick",
        },
        {
            "name": "PICK_RETREAT",
            "position": [0.450, 0.180, 0.360],
            "rpy": FIXED_RPY,
            "phase": "pick",
        },
    ]

    start_y = 0.180
    goal_y = -0.180

    for index in range(
        1,
        TRANSFER_SEGMENTS + 1,
    ):
        ratio = index / TRANSFER_SEGMENTS

        y = (
            start_y
            + ratio * (goal_y - start_y)
        )

        waypoints.append(
            {
                "name": (
                    f"TRANSFER_{index:02d}"
                    if index < TRANSFER_SEGMENTS
                    else "PLACE_ABOVE"
                ),
                "position": [
                    0.450,
                    y,
                    0.360,
                ],
                "rpy": FIXED_RPY,
                "phase": "transfer",
            }
        )

    waypoints.extend(
        [
            {
                "name": "PLACE_DOWN",
                "position": [
                    0.450,
                    -0.180,
                    0.274,
                ],
                "rpy": FIXED_RPY,
                "phase": "place",
            },
            {
                "name": "PLACE_RETREAT",
                "position": [
                    0.450,
                    -0.180,
                    0.360,
                ],
                "rpy": FIXED_RPY,
                "phase": "place",
            },
        ]
    )

    return waypoints


def format_degrees(values) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+7.2f}°"
            for value in values
        )
        + "]"
    )


def main() -> int:
    limits = base.load_joint_limits(
        base.URDF_PATH
    )

    waypoints = make_waypoints()

    print("=" * 100)
    print(
        "DROK ARM INCREMENTAL NEAREST-IK DRY RUN"
    )
    print("=" * 100)

    print(
        f"Transfer segments : {TRANSFER_SEGMENTS}"
    )
    print(
        "Transfer interval : "
        f"{360.0 / TRANSFER_SEGMENTS:.1f} mm"
    )

    print("Cable limits      : 사용 안 함")
    print("Branch switch     : 사용 안 함")
    print("Neutral waypoint  : 사용 안 함")
    print("Orientation search: 사용 안 함")
    print("Mechanical limits : URDF만 사용")
    print("Robot execution   : 비활성화")
    print()

    rclpy.init()

    node = base.JointStateReader()

    try:
        deadline = (
            node.get_clock().now().nanoseconds
            + 10_000_000_000
        )

        while (
            rclpy.ok()
            and not node.ready()
            and node.get_clock().now().nanoseconds
            < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if not node.ready():
            print(
                "/joint_states에서 J1~J6을 "
                "받지 못했습니다."
            )
            return 2

        start_q = node.current_q()
        reference_q = start_q.copy()

        print("START Q:")
        print(" ", format_degrees(start_q))
        print()

        result_waypoints = []

        largest_overall_step = 0.0
        largest_overall_name = ""

        largest_transfer_step = 0.0
        largest_transfer_name = ""

        failed_waypoint = ""

        for waypoint_index, waypoint in enumerate(
            waypoints,
            start=1,
        ):
            name = waypoint["name"]
            position = waypoint["position"]
            rpy = waypoint["rpy"]
            phase = waypoint["phase"]

            print(
                f"[{waypoint_index:02d}/"
                f"{len(waypoints):02d}] "
                f"{name}"
            )

            print(
                "  xyz:",
                "["
                + ", ".join(
                    f"{value:+.4f}"
                    for value in position
                )
                + "]",
            )

            candidates = base.solve_all_candidates(
                position=position,
                rpy=rpy,
                reference_q=reference_q,
                limits=limits,
            )

            print(
                "  candidates:",
                len(candidates),
            )

            if not candidates:
                failed_waypoint = name

                print(
                    "  RESULT: IK 후보 없음"
                )
                break

            selected = candidates[0]

            deltas = [
                selected_value - reference_value
                for selected_value, reference_value
                in zip(selected, reference_q)
            ]

            absolute_deltas = [
                abs(value)
                for value in deltas
            ]

            maximum_delta = max(
                absolute_deltas
            )

            sum_delta = sum(
                absolute_deltas
            )

            print(
                "  selected:",
                format_degrees(selected),
            )

            print(
                "  delta   :",
                format_degrees(deltas),
            )

            print(
                "  max Δ   :",
                f"{math.degrees(maximum_delta):.3f}°",
            )

            print(
                "  sum |Δ| :",
                f"{math.degrees(sum_delta):.3f}°",
            )

            if (
                maximum_delta
                > largest_overall_step
            ):
                largest_overall_step = (
                    maximum_delta
                )
                largest_overall_name = name

            if (
                phase == "transfer"
                and maximum_delta
                > largest_transfer_step
            ):
                largest_transfer_step = (
                    maximum_delta
                )
                largest_transfer_name = name

            result_waypoints.append(
                {
                    "name": name,
                    "phase": phase,
                    "position": [
                        float(value)
                        for value in position
                    ],
                    "rpy": [
                        float(value)
                        for value in rpy
                    ],
                    "candidate_count": len(
                        candidates
                    ),
                    "selected_q": [
                        float(value)
                        for value in selected
                    ],
                    "delta_from_previous": [
                        float(value)
                        for value in deltas
                    ],
                    "maximum_delta_rad": float(
                        maximum_delta
                    ),
                    "maximum_delta_deg": float(
                        math.degrees(
                            maximum_delta
                        )
                    ),
                }
            )

            # 핵심:
            # 현재 선택 해를 다음 waypoint seed로 사용한다.
            reference_q = selected.copy()

            print()

        output = {
            "mode": (
                "incremental_nearest_ik_dry_run"
            ),
            "robot_execution": False,
            "uses_cable_limits": False,
            "uses_branch_switch": False,
            "uses_neutral_waypoint": False,
            "uses_orientation_search": False,
            "transfer_segments": (
                TRANSFER_SEGMENTS
            ),
            "transfer_interval_m": (
                0.360 / TRANSFER_SEGMENTS
            ),
            "selection_order": [
                "minimum_max_joint_delta",
                "minimum_sum_absolute_joint_delta",
                "minimum_sum_squared_joint_delta",
            ],
            "start_q": [
                float(value)
                for value in start_q
            ],
            "waypoints": result_waypoints,
            "failed_waypoint": failed_waypoint,
            "largest_overall_step": {
                "waypoint": (
                    largest_overall_name
                ),
                "radians": float(
                    largest_overall_step
                ),
                "degrees": float(
                    math.degrees(
                        largest_overall_step
                    )
                ),
            },
            "largest_transfer_step": {
                "waypoint": (
                    largest_transfer_name
                ),
                "radians": float(
                    largest_transfer_step
                ),
                "degrees": float(
                    math.degrees(
                        largest_transfer_step
                    )
                ),
            },
        }

        with OUTPUT_PATH.open(
            "w",
            encoding="utf-8",
        ) as file:
            yaml.safe_dump(
                output,
                file,
                sort_keys=False,
                allow_unicode=True,
            )

        print("=" * 100)
        print("FINAL SUMMARY")
        print("=" * 100)

        if failed_waypoint:
            print(
                "Failed waypoint:",
                failed_waypoint,
            )

            print(
                "RESULT: FAIL — 연속 IK가 "
                "중간에 끊어졌습니다."
            )

            return 3

        print(
            "Largest overall step:",
            f"{math.degrees(largest_overall_step):.3f}°",
            f"at {largest_overall_name}",
        )

        print(
            "Largest transfer step:",
            f"{math.degrees(largest_transfer_step):.3f}°",
            f"at {largest_transfer_name}",
        )

        print()
        print("Plan saved:")
        print(OUTPUT_PATH)

        if (
            largest_transfer_step
            > math.radians(30.0)
        ):
            print()
            print(
                "RESULT: WARNING — transfer 중 "
                "30°를 넘는 branch 변화가 있습니다."
            )

            return 4

        print()
        print(
            "RESULT: PASS — 직전 IK 해를 따라 "
            "PICK에서 PLACE까지 연속 연결됐습니다."
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
