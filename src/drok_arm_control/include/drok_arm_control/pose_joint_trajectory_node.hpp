#ifndef DROK_ARM_CONTROL__POSE_JOINT_TRAJECTORY_NODE_HPP_
#define DROK_ARM_CONTROL__POSE_JOINT_TRAJECTORY_NODE_HPP_

#include "drok_arm_kinematics/inverse_kinematics.hpp"
#include "drok_arm_kinematics/robot_model.hpp"
#include "drok_arm_trajectory/poly5_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

namespace drok_arm_control
{

class PoseJointTrajectoryNode : public rclcpp::Node
{
public:
  PoseJointTrajectoryNode();

private:
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr message);

  bool extractArmJointPositions(
    const sensor_msgs::msg::JointState & message,
    std::vector<double> & joint_positions) const;

  Eigen::Matrix4d createTargetTransform() const;

  void planAndPublish(
    const std::vector<double> & initial_joint_positions);

  trajectory_msgs::msg::JointTrajectory createTrajectoryMessage(
    const Eigen::VectorXd & start_position,
    const Eigen::VectorXd & goal_position) const;

  std::string geometry_yaml_;
  std::string joint_state_topic_;
  std::string command_topic_;

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};

  double target_roll_{0.0};
  double target_pitch_{0.0};
  double target_yaw_{0.0};

  double duration_{3.0};
  double point_rate_{100.0};

  bool use_current_joint_state_{true};
  bool trajectory_published_{false};

  std::vector<double> initial_joint_positions_;

  const std::vector<std::string> joint_names_{
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6"
  };

  drok_arm_kinematics::RobotModel robot_model_;

  std::unique_ptr<drok_arm_kinematics::InverseKinematics>
    inverse_kinematics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  rclcpp::Publisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
};

}  // namespace drok_arm_control

#endif  // DROK_ARM_CONTROL__POSE_JOINT_TRAJECTORY_NODE_HPP_
