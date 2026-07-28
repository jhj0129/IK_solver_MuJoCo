cmake_minimum_required(VERSION 3.8)

project(drok_arm_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_path(
  TINYXML2_INCLUDE_DIR
  NAMES tinyxml2.h
  PATHS
    /usr/include
    /usr/local/include
)

find_library(
  TINYXML2_LIBRARY
  NAMES tinyxml2
  PATHS
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

if(NOT TINYXML2_INCLUDE_DIR)
  message(FATAL_ERROR "tinyxml2.h not found.")
endif()

if(NOT TINYXML2_LIBRARY)
  message(FATAL_ERROR "tinyxml2 library not found.")
endif()

add_library(
  ${PROJECT_NAME}
  src/robot_model_loader.cpp
  src/transform.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(
  urdf_geometry_exporter
  src/urdf_geometry_exporter.cpp
)

target_include_directories(
  urdf_geometry_exporter
  PRIVATE
    ${TINYXML2_INCLUDE_DIR}
)

target_link_libraries(
  urdf_geometry_exporter
  PRIVATE
    yaml-cpp
    ${TINYXML2_LIBRARY}
)

add_executable(
  test_fk
  src/test_fk.cpp
)

target_link_libraries(
  test_fk
  PRIVATE
    ${PROJECT_NAME}
)

add_executable(
  test_ik
  src/test_ik.cpp
)

target_link_libraries(
  test_ik
  PRIVATE
    ${PROJECT_NAME}
)

add_executable(
  solve_ik_pose
  src/solve_ik_pose.cpp
)

target_link_libraries(
  solve_ik_pose
  PRIVATE
    ${PROJECT_NAME}
)


add_executable(
  solve_ik_upright
  src/solve_ik_upright.cpp
)

target_link_libraries(
  solve_ik_upright
  PRIVATE
    ${PROJECT_NAME}
)


install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  TARGETS
    urdf_geometry_exporter
    test_fk
    test_ik
    solve_ik_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  Eigen3
)


install(
  TARGETS
    solve_ik_upright
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  validate_timed_cylinder_fk
  src/validate_timed_cylinder_fk.cpp
)

target_link_libraries(
  validate_timed_cylinder_fk
  PRIVATE
    ${PROJECT_NAME}
    yaml-cpp
)

target_compile_options(
  validate_timed_cylinder_fk
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

install(
  TARGETS
    validate_timed_cylinder_fk
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
