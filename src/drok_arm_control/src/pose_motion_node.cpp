#include "drok_arm_control/pose_motion_node.hpp"

#include "drok_arm_kinematics/robot_model_loader.hpp"
#include "drok_arm_kinematics/transform.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace drok_arm_control
{

PoseMotionNode::PoseMotionNode()
: Node("pose_motion_node")
{
  declareAndReadParameters();

  robot_model_ =
    drok_arm_kinematics::RobotModelLoader::loadFromYaml(
    geometry_yaml_);

  if (robot_model_.movable_joint_count != 6) {
    throw std::runtime_error(
            "PoseMotionNode expects exactly six movable joints.");
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
    std::make_unique<drok_arm_kinematics::InverseKinematics>(
    robot_model_,
    ik_options);

  createInterfaces();

  if (!use_current_joint_state_) {
    planMotion(parameter_initial_positions_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Waiting for the first valid /joint_states message.");
  }
}

void PoseMotionNode::declareAndReadParameters()
{
  geometry_yaml_ =
    declare_parameter<std::string>(
    "geometry_yaml",
    "");

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

  trajectory_duration_ =
    declare_parameter<double>("duration", 3.0);

  publish_rate_ =
    declare_parameter<double>("publish_rate", 100.0);

  use_current_joint_state_ =
    declare_parameter<bool>(
    "use_current_joint_state",
    false);

  hold_final_position_ =
    declare_parameter<bool>(
    "hold_final_position",
    true);

  parameter_initial_positions_ =
    declare_parameter<std::vector<double>>(
    "initial_joint_positions",
    std::vector<double>(6, 0.0));

  if (geometry_yaml_.empty()) {
    throw std::invalid_argument(
            "geometry_yaml parameter must not be empty.");
  }

  if (!std::isfinite(trajectory_duration_) ||
    trajectory_duration_ <= 0.0)
  {
    throw std::invalid_argument(
            "duration must be finite and greater than zero.");
  }

  if (!std::isfinite(publish_rate_) ||
    publish_rate_ <= 0.0)
  {
    throw std::invalid_argument(
            "publish_rate must be finite and greater than zero.");
  }

  if (parameter_initial_positions_.size() != 6) {
    throw std::invalid_argument(
            "initial_joint_positions must contain six values.");
  }

  for (const double value : parameter_initial_positions_) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
              "initial_joint_positions contains a non-finite value.");
    }
  }
}

void PoseMotionNode::createInterfaces()
{
  joint_state_publisher_ =
    create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::QoS(10));

  if (use_current_joint_state_) {
    joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(10),
      std::bind(
        &PoseMotionNode::jointStateCallback,
        this,
        std::placeholders::_1));
  }

  const auto timer_period =
    std::chrono::duration<double>(
    1.0 / publish_rate_);

  control_timer_ =
    create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      timer_period),
    std::bind(
      &PoseMotionNode::controlTimerCallback,
      this));
}

void PoseMotionNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (received_initial_joint_state_ ||
    trajectory_planned_)
  {
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
      "Received /joint_states, but JOINT1 through JOINT6 "
      "were not all present.");

    return;
  }

  received_initial_joint_state_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Received current arm position from /joint_states.");

  planMotion(current_joint_positions);
}

bool PoseMotionNode::extractArmJointPositions(
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
  joint_positions.reserve(arm_joint_names_.size());

  for (const auto & joint_name : arm_joint_names_) {
    const auto iterator =
      position_map.find(joint_name);

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

Eigen::Matrix4d PoseMotionNode::createTargetTransform() const
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

void PoseMotionNode::planMotion(
  const std::vector<double> & start_joint_positions)
{
  if (start_joint_positions.size() != 6) {
    throw std::invalid_argument(
            "Start joint position vector must contain six values.");
  }

  const Eigen::Matrix4d target_transform =
    createTargetTransform();

  const auto ik_result =
    inverse_kinematics_->solve(
    target_transform,
    start_joint_positions);

  RCLCPP_INFO(
    get_logger(),
    "IK finished: success=%s, iterations=%zu, "
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
            "IK returned an unexpected number of joints.");
  }

  Eigen::VectorXd start_position(6);
  Eigen::VectorXd goal_position(6);

  for (Eigen::Index index = 0;
    index < 6;
    ++index)
  {
    start_position[index] =
      start_joint_positions[
      static_cast<std::size_t>(index)];

    goal_position[index] =
      ik_result.joint_positions[
      static_cast<std::size_t>(index)];
  }

  trajectory_.configureRestToRest(
    start_position,
    goal_position,
    trajectory_duration_);

  final_position_ = goal_position;
  zero_velocity_ = Eigen::VectorXd::Zero(6);

  trajectory_start_time_ =
    std::chrono::steady_clock::now();

  trajectory_planned_ = true;
  trajectory_finished_ = false;

  for (Eigen::Index index = 0;
    index < goal_position.size();
    ++index)
  {
    RCLCPP_INFO(
      get_logger(),
      "Goal JOINT%ld = %.9f rad",
      static_cast<long>(index + 1),
      goal_position[index]);
  }

  RCLCPP_INFO(
    get_logger(),
    "Poly5 trajectory started: duration=%.3f s, rate=%.1f Hz",
    trajectory_duration_,
    publish_rate_);
}

void PoseMotionNode::controlTimerCallback()
{
  if (!trajectory_planned_) {
    return;
  }

  if (trajectory_finished_) {
    if (hold_final_position_) {
      publishState(
        final_position_,
        zero_velocity_);
    }

    return;
  }

  const auto current_time =
    std::chrono::steady_clock::now();

  const double elapsed_time =
    std::chrono::duration<double>(
    current_time - trajectory_start_time_).count();

  const double sample_time =
    std::clamp(
    elapsed_time,
    0.0,
    trajectory_duration_);

  const auto state =
    trajectory_.sample(sample_time);

  publishState(
    state.position,
    state.velocity);

  if (elapsed_time >= trajectory_duration_) {
    trajectory_finished_ = true;

    publishState(
      final_position_,
      zero_velocity_);

    RCLCPP_INFO(
      get_logger(),
      "Poly5 trajectory completed. Final position hold=%s",
      hold_final_position_ ? "true" : "false");
  }
}

void PoseMotionNode::publishState(
  const Eigen::VectorXd & position,
  const Eigen::VectorXd & velocity)
{
  if (position.size() != 6 ||
    velocity.size() != 6)
  {
    throw std::runtime_error(
            "Published arm state must contain six joints.");
  }

  sensor_msgs::msg::JointState message;

  message.header.stamp = now();
  message.name = published_joint_names_;

  message.position.reserve(8);
  message.velocity.reserve(8);

  for (Eigen::Index index = 0;
    index < 6;
    ++index)
  {
    message.position.push_back(position[index]);
    message.velocity.push_back(velocity[index]);
  }

  message.position.push_back(0.0);
  message.position.push_back(0.0);

  message.velocity.push_back(0.0);
  message.velocity.push_back(0.0);

  joint_state_publisher_->publish(message);
}

}  // namespace drok_arm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node =
      std::make_shared<drok_arm_control::PoseMotionNode>();

    rclcpp::spin(node);

  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pose_motion_node"),
      "%s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
