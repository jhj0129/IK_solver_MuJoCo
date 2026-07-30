#include "drok_arm_trajectory/poly5_trajectory.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace
{

void printVector(
  const std::string & name,
  const Eigen::VectorXd & vector)
{
  std::cout << name << " = [";

  for (Eigen::Index index = 0;
    index < vector.size();
    ++index)
  {
    if (index > 0) {
      std::cout << ", ";
    }

    std::cout << vector[index];
  }

  std::cout << "]\n";
}

bool isNear(
  const Eigen::VectorXd & lhs,
  const Eigen::VectorXd & rhs,
  const double tolerance)
{
  return
    lhs.size() == rhs.size() &&
    (lhs - rhs).norm() <= tolerance;
}

}  // namespace

int main()
{
  try {
    constexpr Eigen::Index joint_count = 6;

    Eigen::VectorXd start_position =
      Eigen::VectorXd::Zero(joint_count);

    Eigen::VectorXd goal_position(joint_count);

    goal_position <<
      0.299074309,
      0.500092802,
      1.000253834,
      -0.399108772,
      0.599299616,
      -0.799564448;

    constexpr double duration = 3.0;
    constexpr double sample_period = 0.25;

    drok_arm_trajectory::Poly5Trajectory trajectory;

    trajectory.configureRestToRest(
      start_position,
      goal_position,
      duration);

    std::cout
      << std::fixed
      << std::setprecision(9);

    std::cout
      << "========================================\n"
      << " DROK Poly5 Trajectory Test\n"
      << "========================================\n"
      << "Joint count : "
      << trajectory.jointCount()
      << '\n'
      << "Duration    : "
      << trajectory.duration()
      << " s\n"
      << "Sample time : "
      << sample_period
      << " s\n"
      << "----------------------------------------\n";

    for (double time = 0.0;
      time <= duration + 1.0e-9;
      time += sample_period)
    {
      const auto state =
        trajectory.sample(time);

      std::cout
        << "t = "
        << time
        << " s\n";

      printVector("q  ", state.position);
      printVector("dq ", state.velocity);
      printVector("ddq", state.acceleration);

      std::cout
        << "----------------------------------------\n";
    }

    const auto start_state =
      trajectory.sample(0.0);

    const auto goal_state =
      trajectory.sample(duration);

    const Eigen::VectorXd zero =
      Eigen::VectorXd::Zero(joint_count);

    constexpr double tolerance = 1.0e-9;

    const bool start_position_ok =
      isNear(
      start_state.position,
      start_position,
      tolerance);

    const bool start_velocity_ok =
      isNear(
      start_state.velocity,
      zero,
      tolerance);

    const bool start_acceleration_ok =
      isNear(
      start_state.acceleration,
      zero,
      tolerance);

    const bool goal_position_ok =
      isNear(
      goal_state.position,
      goal_position,
      tolerance);

    const bool goal_velocity_ok =
      isNear(
      goal_state.velocity,
      zero,
      tolerance);

    const bool goal_acceleration_ok =
      isNear(
      goal_state.acceleration,
      zero,
      tolerance);

    const bool all_tests_passed =
      start_position_ok &&
      start_velocity_ok &&
      start_acceleration_ok &&
      goal_position_ok &&
      goal_velocity_ok &&
      goal_acceleration_ok;

    std::cout
      << "Boundary-condition verification\n"
      << "----------------------------------------\n"
      << "Start position     : "
      << std::boolalpha
      << start_position_ok
      << '\n'
      << "Start velocity     : "
      << start_velocity_ok
      << '\n'
      << "Start acceleration : "
      << start_acceleration_ok
      << '\n'
      << "Goal position      : "
      << goal_position_ok
      << '\n'
      << "Goal velocity      : "
      << goal_velocity_ok
      << '\n'
      << "Goal acceleration  : "
      << goal_acceleration_ok
      << '\n'
      << "----------------------------------------\n"
      << "Overall result     : "
      << (all_tests_passed ? "PASS" : "FAIL")
      << '\n'
      << "========================================\n";

    return all_tests_passed ? 0 : 2;

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }
}
