#include "drok_arm_trajectory/poly5_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace drok_arm_trajectory
{

Poly5Trajectory::Poly5Trajectory(
  const Poly5BoundaryCondition & boundary_condition)
{
  configure(boundary_condition);
}

void Poly5Trajectory::validateBoundaryCondition(
  const Poly5BoundaryCondition & boundary_condition) const
{
  if (!std::isfinite(boundary_condition.duration) ||
    boundary_condition.duration <= 0.0)
  {
    throw std::invalid_argument(
            "Trajectory duration must be finite and greater than zero.");
  }

  const Eigen::Index joint_count =
    boundary_condition.start_position.size();

  if (joint_count <= 0) {
    throw std::invalid_argument(
            "Trajectory must contain at least one joint.");
  }

  const auto validate_vector =
    [joint_count](
    const Eigen::VectorXd & vector,
    const std::string & name)
    {
      if (vector.size() != joint_count) {
        throw std::invalid_argument(
                name + " size does not match start_position size.");
      }

      if (!vector.allFinite()) {
        throw std::invalid_argument(
                name + " contains a non-finite value.");
      }
    };

  validate_vector(
    boundary_condition.start_position,
    "start_position");

  validate_vector(
    boundary_condition.start_velocity,
    "start_velocity");

  validate_vector(
    boundary_condition.start_acceleration,
    "start_acceleration");

  validate_vector(
    boundary_condition.goal_position,
    "goal_position");

  validate_vector(
    boundary_condition.goal_velocity,
    "goal_velocity");

  validate_vector(
    boundary_condition.goal_acceleration,
    "goal_acceleration");
}

void Poly5Trajectory::configure(
  const Poly5BoundaryCondition & boundary_condition)
{
  validateBoundaryCondition(boundary_condition);

  boundary_condition_ = boundary_condition;

  const Eigen::Index joint_count =
    boundary_condition_.start_position.size();

  coefficients_ =
    Eigen::MatrixXd::Zero(joint_count, 6);

  const double duration =
    boundary_condition_.duration;

  Eigen::Matrix3d terminal_matrix;

  terminal_matrix <<
    std::pow(duration, 3),
    std::pow(duration, 4),
    std::pow(duration, 5),

    3.0 * std::pow(duration, 2),
    4.0 * std::pow(duration, 3),
    5.0 * std::pow(duration, 4),

    6.0 * duration,
    12.0 * std::pow(duration, 2),
    20.0 * std::pow(duration, 3);

  const Eigen::FullPivLU<Eigen::Matrix3d>
    decomposition(terminal_matrix);

  if (!decomposition.isInvertible()) {
    throw std::runtime_error(
            "Failed to solve Poly5 coefficient matrix.");
  }

  for (Eigen::Index joint_index = 0;
    joint_index < joint_count;
    ++joint_index)
  {
    const double q0 =
      boundary_condition_.start_position[joint_index];

    const double dq0 =
      boundary_condition_.start_velocity[joint_index];

    const double ddq0 =
      boundary_condition_.start_acceleration[joint_index];

    const double qf =
      boundary_condition_.goal_position[joint_index];

    const double dqf =
      boundary_condition_.goal_velocity[joint_index];

    const double ddqf =
      boundary_condition_.goal_acceleration[joint_index];

    const double a0 = q0;
    const double a1 = dq0;
    const double a2 = 0.5 * ddq0;

    Eigen::Vector3d terminal_target;

    terminal_target <<
      qf -
      (
        a0 +
        a1 * duration +
        a2 * std::pow(duration, 2)
      ),

      dqf -
      (
        a1 +
        2.0 * a2 * duration
      ),

      ddqf -
      (
        2.0 * a2
      );

    const Eigen::Vector3d higher_order_coefficients =
      decomposition.solve(terminal_target);

    coefficients_(joint_index, 0) = a0;
    coefficients_(joint_index, 1) = a1;
    coefficients_(joint_index, 2) = a2;
    coefficients_(joint_index, 3) =
      higher_order_coefficients[0];

    coefficients_(joint_index, 4) =
      higher_order_coefficients[1];

    coefficients_(joint_index, 5) =
      higher_order_coefficients[2];
  }

  configured_ = true;
}

void Poly5Trajectory::configureRestToRest(
  const Eigen::VectorXd & start_position,
  const Eigen::VectorXd & goal_position,
  const double duration)
{
  if (start_position.size() != goal_position.size()) {
    throw std::invalid_argument(
            "Start and goal position sizes do not match.");
  }

  Poly5BoundaryCondition boundary_condition;

  boundary_condition.start_position =
    start_position;

  boundary_condition.goal_position =
    goal_position;

  boundary_condition.start_velocity =
    Eigen::VectorXd::Zero(start_position.size());

  boundary_condition.start_acceleration =
    Eigen::VectorXd::Zero(start_position.size());

  boundary_condition.goal_velocity =
    Eigen::VectorXd::Zero(start_position.size());

  boundary_condition.goal_acceleration =
    Eigen::VectorXd::Zero(start_position.size());

  boundary_condition.duration = duration;

  configure(boundary_condition);
}

TrajectoryState Poly5Trajectory::sample(
  const double time) const
{
  if (!configured_) {
    throw std::runtime_error(
            "Poly5 trajectory has not been configured.");
  }

  if (!std::isfinite(time)) {
    throw std::invalid_argument(
            "Sample time must be finite.");
  }

  const double clamped_time =
    std::clamp(
    time,
    0.0,
    boundary_condition_.duration);

  const Eigen::Index joint_count =
    coefficients_.rows();

  TrajectoryState state;

  state.position =
    Eigen::VectorXd::Zero(joint_count);

  state.velocity =
    Eigen::VectorXd::Zero(joint_count);

  state.acceleration =
    Eigen::VectorXd::Zero(joint_count);

  const double t2 =
    clamped_time * clamped_time;

  const double t3 =
    t2 * clamped_time;

  const double t4 =
    t3 * clamped_time;

  const double t5 =
    t4 * clamped_time;

  for (Eigen::Index joint_index = 0;
    joint_index < joint_count;
    ++joint_index)
  {
    const double a0 =
      coefficients_(joint_index, 0);

    const double a1 =
      coefficients_(joint_index, 1);

    const double a2 =
      coefficients_(joint_index, 2);

    const double a3 =
      coefficients_(joint_index, 3);

    const double a4 =
      coefficients_(joint_index, 4);

    const double a5 =
      coefficients_(joint_index, 5);

    state.position[joint_index] =
      a0 +
      a1 * clamped_time +
      a2 * t2 +
      a3 * t3 +
      a4 * t4 +
      a5 * t5;

    state.velocity[joint_index] =
      a1 +
      2.0 * a2 * clamped_time +
      3.0 * a3 * t2 +
      4.0 * a4 * t3 +
      5.0 * a5 * t4;

    state.acceleration[joint_index] =
      2.0 * a2 +
      6.0 * a3 * clamped_time +
      12.0 * a4 * t2 +
      20.0 * a5 * t3;
  }

  return state;
}

double Poly5Trajectory::duration() const noexcept
{
  return boundary_condition_.duration;
}

std::size_t Poly5Trajectory::jointCount() const noexcept
{
  return static_cast<std::size_t>(
    coefficients_.rows());
}

bool Poly5Trajectory::isConfigured() const noexcept
{
  return configured_;
}

}  // namespace drok_arm_trajectory
