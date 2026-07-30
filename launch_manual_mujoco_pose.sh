#!/usr/bin/env bash
set -eo pipefail

ROOT="${HOME}/IK_solver_MuJoCo"
RUNTIME="${ROOT}/manual_pose_runtime"
OUTPUT_DIR="${RUNTIME}/generated"

ROBOT_XML="${OUTPUT_DIR}/mujoco_description_formatted.xml"
SCENE_XML="${OUTPUT_DIR}/scene.xml"
MERGED_XML="${OUTPUT_DIR}/manual_robot_with_scene.xml"

LABEL="${1:-PICK_GRASP_SAMPLE}"

echo "기존 ROS/MuJoCo 제어 프로세스를 종료합니다."

pkill -f drok_arm_mujoco.launch.py 2>/dev/null || true
pkill -f robot_description_to_mjcf.sh 2>/dev/null || true
pkill -f ros2_control_node 2>/dev/null || true
pkill -f trajectory_bridge 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f pick_place_task_node 2>/dev/null || true
pkill -f manual_mujoco_pose_editor.py 2>/dev/null || true

sleep 2

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"

rm -rf "${RUNTIME}"
mkdir -p "${OUTPUT_DIR}"

echo
echo "로봇 XML과 작업 장면 XML을 생성합니다."

ros2 run mujoco_ros2_control \
  robot_description_to_mjcf.sh \
  --urdf \
  "${ROOT}/src/drok_arm_mujoco/urdf/drok_arm_mujoco.urdf" \
  --mujoco_inputs \
  "${ROOT}/src/drok_arm_mujoco/config/mujoco_inputs.xml" \
  --scene \
  "${ROOT}/src/drok_arm_mujoco/model/scene.xml" \
  --convert_stl_to_obj \
  --no-fuse \
  --save_only \
  --output \
  "${OUTPUT_DIR}"

if [[ ! -f "${ROBOT_XML}" ]]; then
  echo
  echo "[ERROR] 로봇 XML이 없습니다:"
  echo "${ROBOT_XML}"
  exit 1
fi

if [[ ! -f "${SCENE_XML}" ]]; then
  echo
  echo "[ERROR] 장면 XML이 없습니다:"
  echo "${SCENE_XML}"
  exit 1
fi

echo
echo "로봇 XML과 장면 XML을 하나로 결합합니다."

python3 - \
  "${ROBOT_XML}" \
  "${SCENE_XML}" \
  "${MERGED_XML}" <<'PY'
from __future__ import annotations

import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


robot_path = Path(sys.argv[1])
scene_path = Path(sys.argv[2])
output_path = Path(sys.argv[3])

robot_tree = ET.parse(robot_path)
scene_tree = ET.parse(scene_path)

robot_root = robot_tree.getroot()
scene_root = scene_tree.getroot()

if robot_root.tag != "mujoco":
    raise RuntimeError(
        f"로봇 XML의 루트가 mujoco가 아닙니다: "
        f"{robot_root.tag}"
    )

if scene_root.tag != "mujoco":
    raise RuntimeError(
        f"장면 XML의 루트가 mujoco가 아닙니다: "
        f"{scene_root.tag}"
    )


CONTAINER_TAGS = {
    "asset",
    "worldbody",
    "contact",
    "equality",
    "tendon",
    "actuator",
    "sensor",
    "keyframe",
    "custom",
    "extension",
}


def find_direct_child(
    parent: ET.Element,
    tag: str,
) -> ET.Element | None:
    for child in parent:
        if child.tag == tag:
            return child

    return None


def merge_recursive(
    destination: ET.Element,
    source: ET.Element,
) -> None:
    # Scene 쪽의 속성을 우선 적용한다.
    destination.attrib.update(
        source.attrib
    )

    for source_child in source:
        matching_child = None

        source_name = source_child.attrib.get(
            "name"
        )

        for destination_child in destination:
            if destination_child.tag != source_child.tag:
                continue

            destination_name = (
                destination_child.attrib.get("name")
            )

            if (
                source_name is not None
                and destination_name == source_name
            ):
                matching_child = destination_child
                break

        if matching_child is None:
            destination.append(
                copy.deepcopy(source_child)
            )
        else:
            merge_recursive(
                matching_child,
                source_child,
            )


for scene_section in scene_root:
    tag = scene_section.tag

    robot_section = find_direct_child(
        robot_root,
        tag,
    )

    if tag in CONTAINER_TAGS:
        if robot_section is None:
            robot_root.append(
                copy.deepcopy(scene_section)
            )
            continue

        for scene_child in scene_section:
            robot_section.append(
                copy.deepcopy(scene_child)
            )

        continue

    # compiler, option, visual, statistic, default 등의
    # 단일 섹션은 기존 섹션에 속성과 하위 요소를 병합한다.
    if robot_section is None:
        robot_root.append(
            copy.deepcopy(scene_section)
        )
    else:
        merge_recursive(
            robot_section,
            scene_section,
        )


robot_root.attrib["model"] = (
    "DROK_ARM_MANUAL_PICK_PLACE"
)

try:
    ET.indent(
        robot_tree,
        space="  ",
    )
except AttributeError:
    pass

robot_tree.write(
    output_path,
    encoding="utf-8",
    xml_declaration=True,
)

print("결합 완료:")
print(output_path)
PY

PYTHON_BIN="${HOME}/.ros/ros2_control/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="python3"
fi

echo
echo "결합된 모델을 검증합니다."

"${PYTHON_BIN}" - "${MERGED_XML}" <<'PY'
from pathlib import Path
import sys

import mujoco


xml_path = Path(sys.argv[1])

model = mujoco.MjModel.from_xml_path(
    str(xml_path)
)


def has_object(
    object_type,
    name: str,
) -> bool:
    return (
        mujoco.mj_name2id(
            model,
            object_type,
            name,
        )
        >= 0
    )


checks = {
    "JOINT1": has_object(
        mujoco.mjtObj.mjOBJ_JOINT,
        "JOINT1",
    ),
    "JOINT6": has_object(
        mujoco.mjtObj.mjOBJ_JOINT,
        "JOINT6",
    ),
    "pickup_cube": has_object(
        mujoco.mjtObj.mjOBJ_BODY,
        "pickup_cube",
    ),
    "pickup_pedestal": has_object(
        mujoco.mjtObj.mjOBJ_GEOM,
        "pickup_pedestal",
    ),
    "place_pedestal": has_object(
        mujoco.mjtObj.mjOBJ_GEOM,
        "place_pedestal",
    ),
}

print(
    f"body={model.nbody} "
    f"joint={model.njnt} "
    f"geom={model.ngeom} "
    f"actuator={model.nu}"
)

for name, result in checks.items():
    print(
        f"{name:20s}: "
        f"{'PASS' if result else 'FAIL'}"
    )

if not all(checks.values()):
    raise SystemExit(
        "결합된 모델에 필요한 요소가 없습니다."
    )
PY

echo
echo "ROS 제어 없이 전체 작업 장면을 실행합니다."
echo "Label: ${LABEL}"
echo

exec "${PYTHON_BIN}" \
  "${ROOT}/src/drok_arm_control/scripts/manual_mujoco_pose_editor.py" \
  --xml "${MERGED_XML}" \
  --label "${LABEL}" \
  --output "${ROOT}/manual_mujoco_pose_samples.json"
