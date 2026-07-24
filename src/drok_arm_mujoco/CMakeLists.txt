cmake_minimum_required(VERSION 3.8)

project(drok_arm_mujoco)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    model
    urdf
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/patch_mjcf_gripper_collision.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
