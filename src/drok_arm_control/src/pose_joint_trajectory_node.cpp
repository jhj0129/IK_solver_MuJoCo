#include "drok_arm_control/pose_joint_trajectory_node.hpp"

#include "drok_arm_kinematics/robot_model_loader.hpp"
#include "drok_arm_kinematics/transform.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace drok_arm_control
{

PoseJointTrajectoryNode::PoseJointTrajectoryNode()
: Node("pose_joint_trajectory_node")
{
  geometry_yaml_ =
    declare_parameter<std::string>("geometry_yaml", "");

  joint_state_topic_ =
    declare_parameter<std::string>(
    "joint_state_topic",
    "/joint_states");

  command_topic_ =
    declare_parameter<std::string>(
    "command_topic",
    "/arm_controller/joint_trajectory");

  target_x_ =
    declare_parameter<double>("target_x", 0.0);

  target_y_ =
    declare_parameter<double>("target_y", 0.0);

  target_z_ =
    declare_parameter<double>("target_z", 0.0);

  target_roll_ =
    declare_parameter<double>("target_roll", 0.0);

  target_pitch_ =
    declare_parameter<double>("target_pitch", 0.0);

  target_yaw_ =
    declare_parameter<double>("target_yaw", 0.0);

  duration_ =
    declare_parameter<double>("duration", 3.0);

  point_rate_ =
    declare_parameter<double>("point_rate", 100.0);

  use_current_joint_state_ =
    declare_parameter<bool>(
    "use_current_joint_state",
    true);

  initial_joint_positions_ =
    declare_parameter<std::vector<double>>(
    "initial_joint_positions",
    std::vector<double>(6, 0.0));

  if (geometry_yaml_.empty()) {
    throw std::invalid_argument(
            "geometry_yaml must not be empty.");
  }

  if (!std::isfinite(duration_) || duration_ <= 0.0) {
    throw std::invalid_argument(
            "duration must be greater than zero.");
  }

  if (!std::isfinite(point_rate_) || point_rate_ <= 0.0) {
    throw std::invalid_argument(
            "point_rate must be greater than zero.");
  }

  if (initial_joint_positions_.size() != 6) {
    throw std::invalid_argument(
            "initial_joint_positions must contain six values.");
  }

  robot_model_ =
    drok_arm_kinematics::RobotModelLoader::loadFromYaml(
    geometry_yaml_);

  if (robot_model_.movable_joint_count != 6) {
    throw std::runtime_error(
            "Exactly six movable joints are required.");
  }

  drok_arm_kinematics::IkOptions ik_options;

  ik_options.max_iterations = 1500;
  ik_options.position_tolerance = 1.0e-5;
  ik_options.orientation_tolerance = 1.0e-5;
  ik_options.damping = 1.0e-2;
  ik_options.numerical_delta = 1.0e-6;
  ik_options.position_weight = 1.0;
  ik_options.orientation_weight = 0.5;
  ik_options.maximum_joint_step = 0.05;

  inverse_kinematics_ =
    std::make_unique<
    drok_arm_kinematics::InverseKinematics>(
    robot_model_,
    ik_options);

  trajectory_publisher_ =
    create_publisher<
    trajectory_msgs::msg::JointTrajectory>(
    command_topic_,
    rclcpp::QoS(10));

  if (use_current_joint_state_) {
    joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::QoS(10),
      std::bind(
        &PoseJointTrajectoryNode::jointStateCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for JOINT1 through JOINT6 on %s",
      joint_state_topic_.c_str());

  } else {
    planAndPublish(initial_joint_positions_);
  }
}

void PoseJointTrajectoryNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (trajectory_published_) {
    return;
  }

  std::vector<double> current_joint_positions;

  if (!extractArmJointPositions(
      *message,
      current_joint_positions))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "A JointState message was received, but JOINT1 "
      "through JOINT6 were not all valid.");

    return;
  }

  planAndPublish(current_joint_positions);
}

bool PoseJointTrajectoryNode::extractArmJointPositions(
  const sensor_msgs::msg::JointState & message,
  std::vector<double> & joint_positions) const
{
  if (message.name.size() != message.position.size()) {
    return false;
  }

  std::unordered_map<std::string, double> position_map;

  for (std::size_t index = 0;
    index < message.name.size();
    ++index)
  {
    position_map[message.name[index]] =
      message.position[index];
  }

  joint_positions.clear();
  joint_positions.reserve(joint_names_.size());

  for (const auto & joint_name : joint_names_) {
    const auto iterator = position_map.find(joint_name);

    if (iterator == position_map.end()) {
      return false;
    }

    if (!std::isfinite(iterator->second)) {
      return false;
    }

    joint_positions.push_back(iterator->second);
  }

  return true;
}

Eigen::Matrix4d
PoseJointTrajectoryNode::createTargetTransform() const
{
  Eigen::Matrix4d target_transform =
    Eigen::Matrix4d::Identity();

  target_transform.block<3, 3>(0, 0) =
    drok_arm_kinematics::rotationFromRpy(
    Eigen::Vector3d(
      target_roll_,
      target_pitch_,
      target_yaw_));

  target_transform(0, 3) = target_x_;
  target_transform(1, 3) = target_y_;
  target_transform(2, 3) = target_z_;

  return target_transform;
}

void PoseJointTrajectoryNode::planAndPublish(
  const std::vector<double> & initial_joint_positions)
{
  if (trajectory_published_) {
    return;
  }

  if (initial_joint_positions.size() != 6) {
    throw std::invalid_argument(
            "Initial joint vector must contain six values.");
  }

  const auto ik_result =
    inverse_kinematics_->solve(
    createTargetTransform(),
    initial_joint_positions);

  RCLCPP_INFO(
    get_logger(),
    "IK result: success=%s, iterations=%zu, "
    "position_error=%.9f m, orientation_error=%.9f rad",
    ik_result.success ? "true" : "false",
    ik_result.iterations,
    ik_result.position_error,
    ik_result.orientation_error);

  if (!ik_result.success) {
    throw std::runtime_error(
            "IK failed: " + ik_result.message);
  }

  if (ik_result.joint_positions.size() != 6) {
    throw std::runtime_error(
            "IK result must contain six joints.");
  }

  Eigen::VectorXd start_position(6);
  Eigen::VectorXd goal_position(6);

  for (Eigen::Index index = 0; index < 6; ++index) {
    start_position[index] =
      initial_joint_positions[
      static_cast<std::size_t>(index)];

    goal_position[index] =
      ik_result.joint_positions[
      static_cast<std::size_t>(index)];

    RCLCPP_INFO(
      get_logger(),
      "Goal JOINT%ld = %.9f rad",
      static_cast<long>(index + 1),
      goal_position[index]);
  }

  auto trajectory_message =
    createTrajectoryMessage(
    start_position,
    goal_position);

  trajectory_publisher_->publish(trajectory_message);

  trajectory_published_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Published %zu Poly5 points to %s",
    trajectory_message.points.size(),
    command_topic_.c_str());
}

trajectory_msgs::msg::JointTrajectory
PoseJointTrajectoryNode::createTrajectoryMessage(
  const Eigen::VectorXd & start_position,
  const Eigen::VectorXd & goal_position) const
{
  drok_arm_trajectory::Poly5Trajectory trajectory;

  trajectory.configureRestToRest(
    start_position,
    goal_position,
    duration_);

  trajectory_msgs::msg::JointTrajectory message;

  message.header.stamp = now();
  message.joint_names = joint_names_;

  const std::size_t interval_count =
    static_cast<std::size_t>(
    std::ceil(duration_ * point_rate_));

  message.points.reserve(interval_count + 1);

  for (std::size_t index = 0;
    index <= interval_count;
    ++index)
  {
    const double time =
      std::min(
      static_cast<double>(index) / point_rate_,
      duration_);

    const auto state = trajectory.sample(time);

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions.resize(6);
    point.velocities.resize(6);
    point.accelerations.resize(6);

    for (Eigen::Index joint_index = 0;
      joint_index < 6;
      ++joint_index)
    {
      const auto output_index =
        static_cast<std::size_t>(joint_index);

      point.positions[output_index] =
        state.position[joint_index];

      point.velocities[output_index] =
        state.velocity[joint_index];

      point.accelerations[output_index] =
        state.acceleration[joint_index];
    }

    const auto total_nanoseconds =
      static_cast<int64_t>(
      std::llround(time * 1.0e9));

    point.time_from_start.sec =
      static_cast<int32_t>(
      total_nanoseconds / 1000000000LL);

    point.time_from_start.nanosec =
      static_cast<uint32_t>(
      total_nanoseconds % 1000000000LL);

    message.points.push_back(std::move(point));
  }

  return message;
}

}  // namespace drok_arm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node =
      std::make_shared<
      drok_arm_control::PoseJointTrajectoryNode>();

    rclcpp::spin(node);

  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "pose_joint_trajectory_node"),
      "%s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
