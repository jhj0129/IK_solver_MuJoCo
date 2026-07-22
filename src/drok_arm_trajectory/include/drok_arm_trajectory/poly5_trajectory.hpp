#ifndef DROK_ARM_TRAJECTORY__POLY5_TRAJECTORY_HPP_
#define DROK_ARM_TRAJECTORY__POLY5_TRAJECTORY_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <string>

namespace drok_arm_trajectory
{

struct TrajectoryState
{
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
};

struct Poly5BoundaryCondition
{
  Eigen::VectorXd start_position;
  Eigen::VectorXd start_velocity;
  Eigen::VectorXd start_acceleration;

  Eigen::VectorXd goal_position;
  Eigen::VectorXd goal_velocity;
  Eigen::VectorXd goal_acceleration;

  double duration{0.0};
};

class Poly5Trajectory
{
public:
  Poly5Trajectory() = default;

  explicit Poly5Trajectory(
    const Poly5BoundaryCondition & boundary_condition);

  void configure(
    const Poly5BoundaryCondition & boundary_condition);

  void configureRestToRest(
    const Eigen::VectorXd & start_position,
    const Eigen::VectorXd & goal_position,
    double duration);

  [[nodiscard]]
  TrajectoryState sample(double time) const;

  [[nodiscard]]
  double duration() const noexcept;

  [[nodiscard]]
  std::size_t jointCount() const noexcept;

  [[nodiscard]]
  bool isConfigured() const noexcept;

private:
  void validateBoundaryCondition(
    const Poly5BoundaryCondition & boundary_condition) const;

  Poly5BoundaryCondition boundary_condition_;

  Eigen::MatrixXd coefficients_;

  bool configured_{false};
};

}  // namespace drok_arm_trajectory

#endif  // DROK_ARM_TRAJECTORY__POLY5_TRAJECTORY_HPP_
