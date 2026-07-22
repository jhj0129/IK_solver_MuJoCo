#ifndef DROK_ARM_CONTROL__POSE_MOTION_NODE_HPP_
#define DROK_ARM_CONTROL__POSE_MOTION_NODE_HPP_

#include "drok_arm_kinematics/inverse_kinematics.hpp"
#include "drok_arm_kinematics/robot_model.hpp"
#include "drok_arm_trajectory/poly5_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace drok_arm_control
{

class PoseMotionNode : public rclcpp::Node
{
public:
  PoseMotionNode();

private:
  void declareAndReadParameters();

  void createInterfaces();

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr message);

  void controlTimerCallback();

  void planMotion(
    const std::vector<double> & start_joint_positions);

  void publishState(
    const Eigen::VectorXd & position,
    const Eigen::VectorXd & velocity);

  Eigen::Matrix4d createTargetTransform() const;

  bool extractArmJointPositions(
    const sensor_msgs::msg::JointState & message,
    std::vector<double> & joint_positions) const;

  std::string geometry_yaml_;

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};

  double target_roll_{0.0};
  double target_pitch_{0.0};
  double target_yaw_{0.0};

  double trajectory_duration_{3.0};
  double publish_rate_{100.0};

  bool use_current_joint_state_{false};
  bool hold_final_position_{true};

  std::vector<double> parameter_initial_positions_;

  std::vector<std::string> arm_joint_names_{
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6"
  };

  std::vector<std::string> published_joint_names_{
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
    "GRIPPER_LEFT_JOINT",
    "GRIPPER_RIGHT_JOINT"
  };

  drok_arm_kinematics::RobotModel robot_model_;

  std::unique_ptr<drok_arm_kinematics::InverseKinematics>
    inverse_kinematics_;

  drok_arm_trajectory::Poly5Trajectory trajectory_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::chrono::steady_clock::time_point trajectory_start_time_;

  Eigen::VectorXd final_position_;
  Eigen::VectorXd zero_velocity_;

  bool received_initial_joint_state_{false};
  bool trajectory_planned_{false};
  bool trajectory_finished_{false};
};

}  // namespace drok_arm_control

#endif  // DROK_ARM_CONTROL__POSE_MOTION_NODE_HPP_
