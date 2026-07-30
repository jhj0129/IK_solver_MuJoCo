cmake_minimum_required(VERSION 3.8)

project(drok_arm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(drok_arm_kinematics REQUIRED)
find_package(drok_arm_trajectory REQUIRED)

add_executable(
  pose_motion_node
  src/pose_motion_node.cpp
)

target_include_directories(
  pose_motion_node
  PRIVATE
    include
)

ament_target_dependencies(
  pose_motion_node
  rclcpp
  sensor_msgs
  drok_arm_kinematics
  drok_arm_trajectory
)

target_link_libraries(
  pose_motion_node
  Eigen3::Eigen
)

target_compile_options(
  pose_motion_node
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(
  pose_joint_trajectory_node
  src/pose_joint_trajectory_node.cpp
)

target_include_directories(
  pose_joint_trajectory_node
  PRIVATE
    include
)

ament_target_dependencies(
  pose_joint_trajectory_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  drok_arm_kinematics
  drok_arm_trajectory
)

target_link_libraries(
  pose_joint_trajectory_node
  Eigen3::Eigen
)

target_compile_options(
  pose_joint_trajectory_node
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)



add_executable(
  pick_place_task_node
  src/pick_place_task_node.cpp
)

ament_target_dependencies(
  pick_place_task_node
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  drok_arm_kinematics
  drok_arm_trajectory
)

target_link_libraries(
  pick_place_task_node
  Eigen3::Eigen
  yaml-cpp
)

target_compile_options(
  pick_place_task_node
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(
  trajectory_bridge
  src/trajectory_bridge.cpp
)

ament_target_dependencies(
  trajectory_bridge
  rclcpp
  trajectory_msgs
)

target_compile_options(
  trajectory_bridge
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

install(
  TARGETS
    pose_motion_node
    pose_joint_trajectory_node
    pick_place_task_node
    trajectory_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
