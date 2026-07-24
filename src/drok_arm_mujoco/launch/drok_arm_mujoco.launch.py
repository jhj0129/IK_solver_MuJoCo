#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def launch_setup(context, *args, **kwargs):
    package_share = Path(
        get_package_share_directory("drok_arm_mujoco")
    )

    urdf_path = (
        package_share
        / "urdf"
        / "drok_arm_mujoco.urdf"
    )

    mujoco_inputs_path = (
        package_share
        / "config"
        / "mujoco_inputs.xml"
    )

    scene_path = (
        package_share
        / "model"
        / "scene.xml"
    )

    controllers_path = (
        package_share
        / "config"
        / "controllers.yaml"
    )

    robot_description_string = urdf_path.read_text(
        encoding="utf-8"
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_string,
            value_type=str,
        )
    }

    nodes = []

    # URDF 파일을 직접 읽어 MJCF로 변환하고 토픽으로 발행한다.
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="robot_description_to_mjcf.sh",
            name="drok_robot_description_to_mjcf",
            output="both",
            emulate_tty=True,
            arguments=[
                "--urdf",
                str(urdf_path),
                "--mujoco_inputs",
                str(mujoco_inputs_path),
                "--scene",
                str(scene_path),
                "--convert_stl_to_obj",
                "--no-fuse",
                "--publish_topic",
                "/mujoco_robot_description_raw",
            ],
        )
    )

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                robot_description,
                {"use_sim_time": True},
            ],
        )
    )

    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            output="both",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                ParameterFile(
                    str(controllers_path),
                    allow_substs=True,
                ),
            ],
            remappings=(
                [
                    (
                        "~/robot_description",
                        "/robot_description",
                    )
                ]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        )
    )

    arm_controller_path = (
        package_share
        / "config"
        / "arm_controller.yaml"
    )

    gripper_controller_path = (
        package_share
        / "config"
        / "gripper_controller.yaml"
    )

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_joint_state_broadcaster",
            output="both",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "180",
            ],
        )
    )

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_arm_controller",
            output="both",
            arguments=[
                "arm_controller",
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                str(arm_controller_path),
                "--controller-manager-timeout",
                "180",
            ],
        )
    )


    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_gripper_controller",
            output="both",
            arguments=[
                "gripper_controller",
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                str(gripper_controller_path),
                "--controller-manager-timeout",
                "180",
            ],
        )
    )


    # External control API:
    #   /arm_controller/joint_trajectory
    #
    # MuJoCo controller input:
    #   /controller_manager/joint_trajectory
    #
    # A dedicated C++ bridge is used instead of topic_tools relay
    # so the controller-facing BEST_EFFORT QoS can be set explicitly.
    nodes.append(
        Node(
            package="drok_arm_control",
            executable="trajectory_bridge",
            name="trajectory_bridge",
            output="screen",
            parameters=[
                {
                    "input_topic":
                        "/arm_controller/joint_trajectory",
                    "output_topic":
                        "/controller_manager/joint_trajectory",
                }
            ],
        )
    )

    return nodes


def generate_launch_description():

    gripper_collision_patcher = Node(
        package="drok_arm_mujoco",
        executable="patch_mjcf_gripper_collision.py",
        name="drok_gripper_collision_patcher",
        output="screen",
    )

    return LaunchDescription(
        [
            gripper_collision_patcher,
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without GUI.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
