#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lite3_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

class IKGazeboValidator : public rclcpp::Node
{
public:
  IKGazeboValidator()
  : Node("ik_gazebo_validator"),
    received_joint_state_(false),
    settled_reported_(false)
  {
    // ------------------------------------------------------------
    // Joint order must match controllers.yaml
    // ------------------------------------------------------------
    joint_names_ = {
      "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
      "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
      "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
      "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
    };

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = i;
    }

    // ------------------------------------------------------------
    // Initial standing command currently used in Gazebo
    // ------------------------------------------------------------
    q_command_.resize(12);

    q_command_ <<
      -0.05, -0.67, 1.32,
       0.015, -0.68, 1.34,
      -0.025, -0.65, 1.32,
       0.017, -0.65, 1.335;

    // ------------------------------------------------------------
    // Current FL Cartesian position from the validated FK
    // ------------------------------------------------------------
    lite3_kinematics::Lite3Kinematics kin;

    Eigen::Vector3d q_fl(
      q_command_(0),
      q_command_(1),
      q_command_(2));

    Eigen::Vector3d p_fl =
      kin.forward(lite3_kinematics::Leg::FL, q_fl);

    RCLCPP_INFO(
      this->get_logger(),
      "Current FL FK: [%.6f, %.6f, %.6f]",
      p_fl.x(), p_fl.y(), p_fl.z());

    // ------------------------------------------------------------
    // Small Cartesian displacement.
    //
    // We deliberately use a small displacement so this is a
    // controller/kinematics validation, not a locomotion test.
    // ------------------------------------------------------------
    target_ = p_fl;
    target_.x() -= 0.030;
    target_.z() += 0.030;

    RCLCPP_INFO(
      this->get_logger(),
      "FL target: [%.6f, %.6f, %.6f]",
      target_.x(), target_.y(), target_.z());

    // ------------------------------------------------------------
    // Solve IK using the existing validated kinematics library.
    // ------------------------------------------------------------
    Eigen::Vector3d seed = q_fl;

    auto result =
      kin.inverse(
        lite3_kinematics::Leg::FL,
        target_,
        &seed);

    if (!result.success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "IK failed. Status=%d, position_error=%e",
        static_cast<int>(result.status),
        result.position_error);

      throw std::runtime_error("FL IK failed");
    }

    q_target_fl_ = result.q;

    RCLCPP_INFO(
      this->get_logger(),
      "FL IK solution: [%.9f, %.9f, %.9f]",
      q_target_fl_.x(),
      q_target_fl_.y(),
      q_target_fl_.z());

    RCLCPP_INFO(
      this->get_logger(),
      "IK position error: %.3e m",
      result.position_error);

    // ------------------------------------------------------------
    // ROS interfaces
    // ------------------------------------------------------------
    command_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lite3_position_controller/commands", 10);

    joint_state_sub_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
          &IKGazeboValidator::jointStateCallback,
          this,
          std::placeholders::_1));

    // Publish at 50 Hz.
    timer_ = this->create_wall_timer(
      20ms,
      std::bind(
        &IKGazeboValidator::controlLoop,
        this));

    start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "IK -> Gazebo validator started.");
  }

private:

  void jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  size_t matched = 0;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = joint_index_.find(msg->name[i]);

    if (it != joint_index_.end() &&
        i < msg->position.size())
    {
      q_actual_(static_cast<Eigen::Index>(it->second)) =
        msg->position[i];

      matched++;
    }
  }

  if (matched == 12) {
    received_joint_state_ = true;
  }
}

  void controlLoop()
  {
    if (!received_joint_state_) {
      return;
    }

    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);

    // Keep the three non-FL legs at the standing posture.
    for (size_t i = 0; i < 12; ++i) {
      msg.data[i] = q_command_(i);
    }

    // Replace only FL joint commands by IK result.
    msg.data[0] = q_target_fl_(0);
    msg.data[1] = q_target_fl_(1);
    msg.data[2] = q_target_fl_(2);

    command_pub_->publish(msg);

    // ------------------------------------------------------------
    // Validate after enough time for the controller to settle.
    // ------------------------------------------------------------
    const double elapsed =
      (this->now() - start_time_).seconds();

    if (elapsed < 1.5) {
      return;
    }

    Eigen::Vector3d q_actual_fl(
      q_actual_(0),
      q_actual_(1),
      q_actual_(2));

    lite3_kinematics::Lite3Kinematics kin;

    Eigen::Vector3d p_actual =
      kin.forward(
        lite3_kinematics::Leg::FL,
        q_actual_fl);

    Eigen::Vector3d error =
      target_ - p_actual;

    double position_error = error.norm();

    if (!settled_reported_) {
      RCLCPP_INFO(
        this->get_logger(),
        "==========================================");

      RCLCPP_INFO(
        this->get_logger(),
        "FL Gazebo Cartesian Validation");

      RCLCPP_INFO(
        this->get_logger(),
        "Target   : [%+.6f, %+.6f, %+.6f]",
        target_.x(),
        target_.y(),
        target_.z());

      RCLCPP_INFO(
        this->get_logger(),
        "Actual   : [%+.6f, %+.6f, %+.6f]",
        p_actual.x(),
        p_actual.y(),
        p_actual.z());

      RCLCPP_INFO(
        this->get_logger(),
        "Error    : [%+.6e, %+.6e, %+.6e] m",
        error.x(),
        error.y(),
        error.z());

      RCLCPP_INFO(
        this->get_logger(),
        "Norm     : %.6e m",
        position_error);

      RCLCPP_INFO(
        this->get_logger(),
        "q_actual : [%+.9f, %+.9f, %+.9f]",
        q_actual_fl.x(),
        q_actual_fl.y(),
        q_actual_fl.z());

      RCLCPP_INFO(
        this->get_logger(),
        "q_target : [%+.9f, %+.9f, %+.9f]",
        q_target_fl_.x(),
        q_target_fl_.y(),
        q_target_fl_.z());

      RCLCPP_INFO(
        this->get_logger(),
        "==========================================");

      if (position_error < 1e-4) {
        RCLCPP_INFO(
          this->get_logger(),
          "RESULT: PASS - Cartesian error < 0.1 mm");
      } else if (position_error < 1e-3) {
        RCLCPP_WARN(
          this->get_logger(),
          "RESULT: ACCEPTABLE - Cartesian error < 1 mm");
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "RESULT: FAIL - Cartesian error >= 1 mm");
      }

      settled_reported_ = true;
    }
  }

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint mapping
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;

  // Joint states
  Eigen::VectorXd q_command_;
  Eigen::VectorXd q_actual_ = Eigen::VectorXd::Zero(12);

  // IK target
  Eigen::Vector3d target_;
  Eigen::Vector3d q_target_fl_;

  bool received_joint_state_;
  bool settled_reported_;

  rclcpp::Time start_time_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<IKGazeboValidator>();

    rclcpp::spin(node);
  }
  catch (const std::exception & e) {
    std::cerr
      << "Exception: "
      << e.what()
      << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}