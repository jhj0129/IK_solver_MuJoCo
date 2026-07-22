#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/inverse_kinematics.hpp"
#include "drok_arm_kinematics/robot_model_loader.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{

void printJointPositions(
  const std::string & title,
  const std::vector<double> & positions)
{
  std::cout << title << '\n';

  for (std::size_t index = 0;
    index < positions.size();
    ++index)
  {
    std::cout
      << "q" << index + 1
      << " = "
      << positions[index]
      << " rad\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr
      << "Usage:\n"
      << "  test_ik <robot_geometry.yaml>\n";

    return 1;
  }

  try {
    const std::filesystem::path yaml_path =
      argv[1];

    const auto model =
      drok_arm_kinematics::RobotModelLoader::
      loadFromYaml(yaml_path);

    if (model.movable_joint_count != 6) {
      throw std::runtime_error(
              "This test expects a six-joint robot.");
    }

    const std::vector<double> target_joints{
      0.30,
      0.50,
      1.00,
      -0.40,
      0.60,
      -0.80
    };

    const std::vector<double> initial_joints{
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    };

    const drok_arm_kinematics::ForwardKinematics
      forward_kinematics(model);

    const Eigen::Matrix4d target_transform =
      forward_kinematics.compute(
      target_joints);

    drok_arm_kinematics::IkOptions options;

    options.max_iterations = 1000;

    options.position_tolerance = 1.0e-5;
    options.orientation_tolerance = 1.0e-5;

    options.damping = 1.0e-2;
    options.numerical_delta = 1.0e-6;

    options.position_weight = 1.0;
    options.orientation_weight = 0.5;

    options.maximum_joint_step = 0.05;

    const drok_arm_kinematics::InverseKinematics
      inverse_kinematics(
      model,
      options);

    const auto result =
      inverse_kinematics.solve(
      target_transform,
      initial_joints);

    const Eigen::Matrix4d solved_transform =
      forward_kinematics.compute(
      result.joint_positions);

    std::cout
      << std::fixed
      << std::setprecision(9);

    std::cout
      << "========================================\n"
      << " DROK Inverse Kinematics Test\n"
      << "========================================\n";

    printJointPositions(
      "Target-generating joints",
      target_joints);

    std::cout
      << "----------------------------------------\n";

    printJointPositions(
      "Initial joints",
      initial_joints);

    std::cout
      << "----------------------------------------\n"
      << "Target transform\n"
      << target_transform << '\n'
      << "----------------------------------------\n";

    printJointPositions(
      "IK result joints",
      result.joint_positions);

    std::cout
      << "----------------------------------------\n"
      << "Solved transform\n"
      << solved_transform << '\n'
      << "----------------------------------------\n"
      << "Success           : "
      << std::boolalpha
      << result.success << '\n'
      << "Iterations        : "
      << result.iterations << '\n'
      << "Position error    : "
      << result.position_error
      << " m\n"
      << "Orientation error : "
      << result.orientation_error
      << " rad\n"
      << "Message           : "
      << result.message << '\n'
      << "========================================\n";

    return result.success ? 0 : 2;

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }
}
