#!/usr/bin/env python3

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


ROOT = Path.home() / "IK_solver_MuJoCo"

CONFIG_PATH = (
    ROOT
    / "src/drok_arm_control/config/box_layout.yaml"
)

SCENE_PATH = (
    ROOT
    / "src/drok_arm_mujoco/model/scene.xml"
)


def main() -> int:
    config = yaml.safe_load(
        CONFIG_PATH.read_text(encoding="utf-8")
    )

    scene_root = ET.parse(SCENE_PATH).getroot()

    scene_geoms = {
        geom.attrib["name"]: geom
        for geom in scene_root.findall(".//geom")
        if "name" in geom.attrib
    }

    boxes = config["boxes"]

    if set(boxes.keys()) != {1, 2, 3, 4}:
        raise RuntimeError(
            f"상자 번호가 1~4가 아닙니다: {list(boxes.keys())}"
        )

    seen_positions = set()

    print("===== BOX LAYOUT =====")

    for box_id in sorted(boxes):
        box = boxes[box_id]
        geom_name = box["geom_name"]
        position = box["position"]

        if geom_name not in scene_geoms:
            raise RuntimeError(
                f"scene.xml에 geom이 없습니다: {geom_name}"
            )

        config_xyz = (
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
        )

        scene_xyz = tuple(
            float(value)
            for value in scene_geoms[
                geom_name
            ].attrib["pos"].split()
        )

        error = max(
            abs(config_value - scene_value)
            for config_value, scene_value in zip(
                config_xyz,
                scene_xyz,
            )
        )

        if error > 1e-9:
            raise RuntimeError(
                f"BOX {box_id} 좌표 불일치: "
                f"config={config_xyz}, scene={scene_xyz}"
            )

        xy = (
            round(config_xyz[0], 6),
            round(config_xyz[1], 6),
        )

        if xy in seen_positions:
            raise RuntimeError(
                f"상자 위치가 중복됐습니다: {xy}"
            )

        seen_positions.add(xy)

        radius = math.hypot(
            config_xyz[0],
            config_xyz[1],
        )

        angle_deg = math.degrees(
            math.atan2(
                config_xyz[1],
                config_xyz[0],
            )
        )

        print(
            f"BOX {box_id}: "
            f"({config_xyz[0]:+.3f}, "
            f"{config_xyz[1]:+.3f}) m, "
            f"direction={angle_deg:+.2f}°, "
            f"radius={radius:.3f} m, "
            f"geom={geom_name}"
        )

    initial_box = config["initial_cylinder_box"]

    if initial_box not in boxes:
        raise RuntimeError(
            "initial_cylinder_box가 유효하지 않습니다."
        )

    cylinders = [
        geom
        for geom in scene_root.findall(".//geom")
        if geom.attrib.get("type") == "cylinder"
    ]

    if len(cylinders) != 1:
        raise RuntimeError(
            f"원기둥 개수가 1개가 아닙니다: {len(cylinders)}"
        )

    print()
    print(f"Initial cylinder box: {initial_box}")
    print(f"Cylinder count      : {len(cylinders)}")
    print("BOX LAYOUT VALIDATION: PASS")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
