#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class TrajectoryBridge : public rclcpp::Node
{
public:
  TrajectoryBridge()
  : Node("trajectory_bridge"),
    received_count_(0)
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic",
      "/arm_controller/joint_trajectory");

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic",
      "/controller_manager/joint_trajectory");

    /*
     * 입력:
     * 자체 IK, Poly5, VR 등의 제어 노드가 보내는 궤적
     */
    const auto input_qos = rclcpp::QoS(
      rclcpp::KeepLast(10)).reliable().durability_volatile();

    /*
     * 출력:
     * 현재 MuJoCo controller_manager subscriber가
     * BEST_EFFORT QoS를 사용하므로 동일하게 맞춘다.
     */
    const auto output_qos = rclcpp::QoS(
      rclcpp::KeepLast(10)).best_effort().durability_volatile();

    trajectory_pub_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      output_topic_,
      output_qos);

    trajectory_sub_ =
      this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      input_topic_,
      input_qos,
      std::bind(
        &TrajectoryBridge::trajectoryCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory bridge started");

    RCLCPP_INFO(
      this->get_logger(),
      "Input : %s",
      input_topic_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Output: %s",
      output_topic_.c_str());
  }

private:
  void trajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    ++received_count_;

    if (msg->joint_names.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected trajectory: joint_names is empty");
      return;
    }

    if (msg->points.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected trajectory: points is empty");
      return;
    }

    trajectory_pub_->publish(*msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Forwarded trajectory #%zu: joints=%zu, points=%zu",
      received_count_,
      msg->joint_names.size(),
      msg->points.size());
  }

  std::string input_topic_;
  std::string output_topic_;

  std::size_t received_count_;

  rclcpp::Publisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  rclcpp::Subscription<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<TrajectoryBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("trajectory_bridge"),
      "Unhandled exception: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
