#!/usr/bin/env python3

import argparse
import fcntl
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

BOX_LAYOUT_PATH = (
    ROOT
    / "src/drok_arm_control/config/box_layout.yaml"
)

BOX_MOVE_EXECUTOR = (
    ROOT
    / "src/drok_arm_control/scripts/execute_box_move.py"
)

STATE_DIRECTORY = (
    ROOT
    / "runtime_state"
)

STATE_PATH = (
    STATE_DIRECTORY
    / "cylinder_box_state.yaml"
)

LOCK_PATH = (
    STATE_DIRECTORY
    / "cylinder_box_state.lock"
)

PATH_ROOT = (
    ROOT
    / "generated_box_paths"
)

EXECUTION_CONFIRMATION = (
    "EXECUTE_MUJOCO_BOX_TARGET"
)

INNER_EXECUTION_CONFIRMATION = (
    "EXECUTE_MUJOCO_BOX_MOVE"
)


def now_string() -> str:
    return datetime.now().astimezone().isoformat(
        timespec="seconds"
    )


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise RuntimeError(
            f"파일이 없습니다: {path}"
        )

    document = yaml.safe_load(
        path.read_text(encoding="utf-8")
    )

    if not isinstance(document, dict):
        raise RuntimeError(
            f"YAML 형식이 올바르지 않습니다: {path}"
        )

    return document


def save_state(
    state: Dict[str, Any],
) -> None:
    STATE_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True,
    )

    temporary_path = STATE_PATH.with_suffix(
        ".yaml.tmp"
    )

    temporary_path.write_text(
        yaml.safe_dump(
            state,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    os.replace(
        temporary_path,
        STATE_PATH,
    )


def read_layout() -> Dict[str, Any]:
    layout = load_yaml(
        BOX_LAYOUT_PATH
    )

    raw_boxes = layout.get("boxes")

    if not isinstance(raw_boxes, dict):
        raise RuntimeError(
            "box_layout.yaml에 boxes 설정이 없습니다."
        )

    valid_boxes = {
        int(box_id)
        for box_id in raw_boxes.keys()
    }

    if valid_boxes != {1, 2, 3, 4}:
        raise RuntimeError(
            f"상자 번호가 올바르지 않습니다: "
            f"{sorted(valid_boxes)}"
        )

    return layout


def initial_box_from_layout(
    layout: Dict[str, Any],
) -> int:
    initial_box = int(
        layout.get(
            "initial_cylinder_box",
            1,
        )
    )

    if initial_box not in {1, 2, 3, 4}:
        raise RuntimeError(
            "initial_cylinder_box가 1~4 범위가 아닙니다."
        )

    return initial_box


def make_state(
    current_box: int,
    reason: str,
) -> Dict[str, Any]:
    return {
        "version": 1,
        "current_box": int(current_box),
        "updated_at": now_string(),
        "update_reason": reason,
        "last_successful_move": None,
        "warning": (
            "MuJoCo를 Reset하거나 재실행하면 "
            "초기 상자 상태로 다시 설정해야 합니다."
        ),
    }


def read_state(
    layout: Dict[str, Any],
) -> Dict[str, Any]:
    if not STATE_PATH.exists():
        state = make_state(
            current_box=initial_box_from_layout(
                layout
            ),
            reason="automatic_initialization",
        )

        save_state(state)

        return state

    state = load_yaml(
        STATE_PATH
    )

    current_box = int(
        state.get("current_box", -1)
    )

    if current_box not in {1, 2, 3, 4}:
        raise RuntimeError(
            f"저장된 current_box가 올바르지 않습니다: "
            f"{current_box}"
        )

    return state


def print_state(
    state: Dict[str, Any],
) -> None:
    print("=" * 72)
    print("CYLINDER BOX STATE")
    print("=" * 72)

    print(
        f"Current box : BOX "
        f"{int(state['current_box'])}"
    )

    print(
        f"Updated at  : "
        f"{state.get('updated_at', 'unknown')}"
    )

    print(
        f"Reason      : "
        f"{state.get('update_reason', 'unknown')}"
    )

    last_move = state.get(
        "last_successful_move"
    )

    if isinstance(last_move, dict):
        print(
            "Last move   : "
            f"BOX {last_move.get('from_box')} "
            f"-> BOX {last_move.get('to_box')}"
        )
    else:
        print(
            "Last move   : none"
        )


def validate_box(
    box_id: int,
) -> None:
    if box_id not in {1, 2, 3, 4}:
        raise RuntimeError(
            f"상자 번호는 1~4여야 합니다: {box_id}"
        )


def timed_path_for(
    from_box: int,
    to_box: int,
) -> Path:
    return (
        PATH_ROOT
        / f"box_{from_box}_to_{to_box}"
        / "timed_joint_path.yaml"
    )


def report_path_for(
    from_box: int,
    to_box: int,
) -> Path:
    return (
        PATH_ROOT
        / f"box_{from_box}_to_{to_box}"
        / "generation_report.yaml"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "저장된 현재 원기둥 위치에서 목표 BOX로 "
            "Pick-and-Place를 실행합니다."
        )
    )

    action_group = parser.add_mutually_exclusive_group(
        required=True
    )

    action_group.add_argument(
        "--to-box",
        type=int,
        help="원기둥을 이동할 목표 상자",
    )

    action_group.add_argument(
        "--show-state",
        action="store_true",
        help="현재 저장된 원기둥 위치 확인",
    )

    action_group.add_argument(
        "--set-current-box",
        type=int,
        help="현재 원기둥 위치를 수동 지정",
    )

    action_group.add_argument(
        "--reset-state",
        action="store_true",
        help=(
            "MuJoCo 초기 상태에 맞춰 "
            "원기둥 위치를 BOX 1로 초기화"
        ),
    )

    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.5,
        help="동작 속도 배율. 기본값 0.5",
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="실제 MuJoCo 동작 실행",
    )

    parser.add_argument(
        "--confirmation",
        type=str,
        default="",
        help="실행 확인 문자열",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    STATE_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True,
    )

    layout = read_layout()

    with LOCK_PATH.open(
        "a+",
        encoding="utf-8",
    ) as lock_file:
        fcntl.flock(
            lock_file.fileno(),
            fcntl.LOCK_EX,
        )

        state = read_state(layout)

        if arguments.show_state:
            print_state(state)
            return 0

        if arguments.set_current_box is not None:
            validate_box(
                arguments.set_current_box
            )

            new_state = make_state(
                current_box=(
                    arguments.set_current_box
                ),
                reason="manual_state_override",
            )

            save_state(new_state)

            print(
                "현재 원기둥 위치를 "
                f"BOX {arguments.set_current_box}로 "
                "설정했습니다."
            )

            print_state(new_state)
            return 0

        if arguments.reset_state:
            initial_box = initial_box_from_layout(
                layout
            )

            new_state = make_state(
                current_box=initial_box,
                reason="mujoco_reset",
            )

            save_state(new_state)

            print(
                "MuJoCo 초기 상태에 맞춰 "
                f"BOX {initial_box}로 초기화했습니다."
            )

            print_state(new_state)
            return 0

        if arguments.to_box is None:
            raise RuntimeError(
                "목표 BOX가 지정되지 않았습니다."
            )

        validate_box(arguments.to_box)

        current_box = int(
            state["current_box"]
        )

        target_box = int(
            arguments.to_box
        )

        if current_box == target_box:
            print(
                f"원기둥이 이미 BOX {target_box}에 있습니다."
            )

            return 0

        if not (
            0.0 < arguments.speed_scale <= 1.0
        ):
            raise RuntimeError(
                "speed-scale은 0보다 크고 "
                "1 이하여야 합니다."
            )

        timed_path = timed_path_for(
            current_box,
            target_box,
        )

        report_path = report_path_for(
            current_box,
            target_box,
        )

        print("=" * 72)
        print("AUTOMATIC CYLINDER BOX TARGET")
        print("=" * 72)

        print(
            f"Stored current box : BOX {current_box}"
        )

        print(
            f"Requested target   : BOX {target_box}"
        )

        print(
            f"Selected movement  : "
            f"BOX {current_box} -> BOX {target_box}"
        )

        print(
            f"Speed scale        : "
            f"{arguments.speed_scale:.3f}"
        )

        print(
            f"Timed path         : {timed_path}"
        )

        if (
            not timed_path.exists()
            or not report_path.exists()
        ):
            print()
            print(
                "선택된 이동 경로가 아직 생성되지 않았습니다.",
                file=sys.stderr,
            )

            print(
                "다음 명령으로 경로를 먼저 생성하십시오:",
                file=sys.stderr,
            )

            print(
                "",
                file=sys.stderr,
            )

            print(
                "python3 "
                "src/drok_arm_control/scripts/"
                "generate_box_to_box_path.py "
                f"--from-box {current_box} "
                f"--to-box {target_box} "
                "--overwrite",
                file=sys.stderr,
            )

            return 1

        if not arguments.execute:
            print()
            print("DRY RUN RESULT: PASS")

            print()
            print(
                "실제 실행 옵션:"
            )

            print(
                "--execute "
                f"--confirmation "
                f"{EXECUTION_CONFIRMATION}"
            )

            return 0

        if (
            arguments.confirmation
            != EXECUTION_CONFIRMATION
        ):
            print(
                "실행 확인 문자열이 올바르지 않습니다.",
                file=sys.stderr,
            )

            return 2

        if not BOX_MOVE_EXECUTOR.exists():
            raise RuntimeError(
                f"실행기가 없습니다: "
                f"{BOX_MOVE_EXECUTOR}"
            )

        command = [
            sys.executable,
            str(BOX_MOVE_EXECUTOR),
            "--from-box",
            str(current_box),
            "--to-box",
            str(target_box),
            "--speed-scale",
            str(arguments.speed_scale),
            "--execute",
            "--confirmation",
            INNER_EXECUTION_CONFIRMATION,
        ]

        print()
        print(
            "[실행] 저장된 위치를 기준으로 "
            "BOX 이동을 시작합니다."
        )

        result = subprocess.run(
            command,
            cwd=ROOT,
            check=False,
        )

        if result.returncode != 0:
            print()
            print(
                "BOX 이동이 실패했습니다.",
                file=sys.stderr,
            )

            print(
                "저장된 원기둥 위치는 변경하지 않았습니다.",
                file=sys.stderr,
            )

            print(
                f"Stored current box: BOX {current_box}",
                file=sys.stderr,
            )

            return result.returncode

        updated_state = {
            "version": 1,
            "current_box": target_box,
            "updated_at": now_string(),
            "update_reason": (
                "successful_box_move"
            ),
            "last_successful_move": {
                "from_box": current_box,
                "to_box": target_box,
                "speed_scale": float(
                    arguments.speed_scale
                ),
                "completed_at": now_string(),
            },
            "warning": (
                "MuJoCo를 Reset하거나 재실행하면 "
                "초기 상자 상태로 다시 설정해야 합니다."
            ),
        }

        save_state(updated_state)

        print()
        print("=" * 72)
        print("BOX TARGET STATE UPDATE: PASS")
        print("=" * 72)

        print(
            f"Previous state : BOX {current_box}"
        )

        print(
            f"Current state  : BOX {target_box}"
        )

        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())

    except KeyboardInterrupt:
        print(
            "\n사용자에 의해 중단되었습니다.",
            file=sys.stderr,
        )

        raise SystemExit(130)

    except Exception as exception:
        print(
            f"[실패] {exception}",
            file=sys.stderr,
        )

        raise SystemExit(1)
