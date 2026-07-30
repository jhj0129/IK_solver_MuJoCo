#!/usr/bin/env python3

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class GripperCollisionPatcher(Node):
    def __init__(self) -> None:
        super().__init__("drok_gripper_collision_patcher")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            String,
            "/mujoco_robot_description",
            qos,
        )

        self.subscription = self.create_subscription(
            String,
            "/mujoco_robot_description_raw",
            self.description_callback,
            qos,
        )

        self.get_logger().info(
            "Waiting for /mujoco_robot_description_raw"
        )

    @staticmethod
    def find_body(
        root: ET.Element,
        body_name: str,
    ) -> ET.Element:
        body = root.find(
            f".//body[@name='{body_name}']"
        )

        if body is None:
            raise RuntimeError(
                f"MJCF body not found: {body_name}"
            )

        return body

    @staticmethod
    def remove_collision_geoms(
        body: ET.Element,
    ) -> int:
        removed = 0

        for geom in list(body.findall("geom")):
            if geom.attrib.get("class") != "collision":
                continue

            body.remove(geom)
            removed += 1

        return removed

    @staticmethod
    def append_box_collision(
        body: ET.Element,
        name: str,
        position: str,
    ) -> None:
        # MJCF box size는 각 축의 half-size.
        #
        # 실제 전체 collision 크기:
        # X = 40 mm
        # Y = 3 mm
        # Z = 40 mm
        ET.SubElement(
            body,
            "geom",
            {
                "name": name,
                "class": "collision",
                "type": "box",
                "size": "0.020 0.0015 0.020",
                "pos": position,
                "contype": "1",
                "conaffinity": "1",
                "friction": "1.5 0.01 0.001",
            },
        )

    @staticmethod
    def configure_solver(
        root: ET.Element,
    ) -> None:
        option = root.find("option")

        if option is None:
            option = ET.Element("option")

            insert_index = 0

            for index, child in enumerate(root):
                if child.tag in {
                    "compiler",
                    "size",
                    "statistic",
                }:
                    insert_index = index + 1

            root.insert(insert_index, option)

        # 접촉력 계산을 안정화한다.
        # 기존 timestep과 gravity 등은 건드리지 않는다.
        option.set("solver", "Newton")
        option.set("cone", "elliptic")
        option.set("iterations", "100")
        option.set("tolerance", "1e-8")
        option.set("impratio", "10")
        option.set("noslip_iterations", "5")
        option.set("noslip_tolerance", "1e-8")

    @staticmethod
    def get_contact_section(
        root: ET.Element,
    ) -> ET.Element:
        contact = root.find("contact")

        if contact is not None:
            return contact

        contact = ET.Element("contact")

        worldbody_index = None

        for index, child in enumerate(root):
            if child.tag == "worldbody":
                worldbody_index = index
                break

        if worldbody_index is None:
            root.append(contact)
        else:
            root.insert(
                worldbody_index + 1,
                contact,
            )

        return contact

    @staticmethod
    def remove_existing_gripper_pairs(
        contact: ET.Element,
    ) -> None:
        target_names = {
            "left_pad_cube_pair",
            "right_pad_cube_pair",
        }

        target_geoms = {
            "GRIPPER_LEFT_collision_box",
            "GRIPPER_RIGHT_collision_box",
        }

        for pair in list(contact.findall("pair")):
            name = pair.attrib.get("name", "")
            geom1 = pair.attrib.get("geom1", "")
            geom2 = pair.attrib.get("geom2", "")

            if (
                name in target_names
                or geom1 in target_geoms
                or geom2 in target_geoms
            ):
                contact.remove(pair)

    @staticmethod
    def append_gripper_contact_pairs(
        root: ET.Element,
    ) -> None:
        contact = (
            GripperCollisionPatcher.get_contact_section(
                root
            )
        )

        GripperCollisionPatcher.remove_existing_gripper_pairs(
            contact
        )

        common_attributes = {
            # condim 4:
            # normal + 두 방향 sliding friction
            # + 접촉면 torsional friction
            "condim": "4",

            # pair friction 형식:
            # tangent1 tangent2 torsional
            "friction": "2.0 2.0 0.02",

            # 강하고 감쇠된 접촉
            "solref": "0.005 1",

            # 높은 contact impedance
            "solimp": "0.98 0.995 0.001 0.5 2",
        }

        ET.SubElement(
            contact,
            "pair",
            {
                "name": "left_pad_cube_pair",
                "geom1": "GRIPPER_LEFT_collision_box",
                "geom2": "pickup_cube_geom",
                **common_attributes,
            },
        )

        ET.SubElement(
            contact,
            "pair",
            {
                "name": "right_pad_cube_pair",
                "geom1": "GRIPPER_RIGHT_collision_box",
                "geom2": "pickup_cube_geom",
                **common_attributes,
            },
        )

    def description_callback(
        self,
        message: String,
    ) -> None:
        try:
            root = ET.fromstring(message.data)

            base = self.find_body(
                root,
                "GRIPPER_BASE",
            )
            left = self.find_body(
                root,
                "GRIPPER_LEFT",
            )
            right = self.find_body(
                root,
                "GRIPPER_RIGH",
            )

            base_removed = self.remove_collision_geoms(
                base
            )
            left_removed = self.remove_collision_geoms(
                left
            )
            right_removed = self.remove_collision_geoms(
                right
            )

            self.append_box_collision(
                left,
                "GRIPPER_LEFT_collision_box",
                "0 0.0015 0",
            )

            self.append_box_collision(
                right,
                "GRIPPER_RIGHT_collision_box",
                "0 -0.0015 0",
            )

            self.configure_solver(root)
            self.append_gripper_contact_pairs(root)

            patched_message = String()
            patched_message.data = ET.tostring(
                root,
                encoding="unicode",
            )

            self.publisher.publish(
                patched_message
            )

            self.get_logger().info(
                "Published grip-stabilized MJCF: "
                f"base_removed={base_removed}, "
                f"left_removed={left_removed}, "
                f"right_removed={right_removed}, "
                "pair_friction=2.0, condim=4, "
                "noslip_iterations=5"
            )

        except Exception as error:
            self.get_logger().error(
                f"Failed to patch MJCF: {error}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GripperCollisionPatcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
