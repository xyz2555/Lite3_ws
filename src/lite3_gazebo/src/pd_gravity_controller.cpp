#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <lite3_dynamics/dynamics.hpp>
#include <lite3_kinematics/kinematics.hpp>

class PdGravityController : public rclcpp::Node
{
public:
  PdGravityController()
  : Node("pd_gravity_controller"),
    dynamics_(),
    received_state_(false)
  {
    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------
    kp_ = this->declare_parameter<double>("kp", 20.0);
    kd_ = this->declare_parameter<double>("kd", 1.5);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 500.0);

    // Standing target from validated Lite3 telemetry.
    q_des_ <<
      -0.02073, -0.67214,  1.32366,
       0.01497, -0.67765,  1.33907,
      -0.02465, -0.64953,  1.32289,
       0.01714, -0.65116,  1.33529;

    qdot_des_.setZero();

    // ------------------------------------------------------------
    // Joint names
    // ------------------------------------------------------------
    joint_names_ = {
      "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
      "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
      "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
      "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
    };

    // ------------------------------------------------------------
    // ROS interfaces
    // ------------------------------------------------------------
    joint_state_sub_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        std::bind(
          &PdGravityController::jointStateCallback,
          this,
          std::placeholders::_1));

    effort_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lite3_effort_controller/commands",
        10);

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&PdGravityController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "PD + Gravity controller started. Kp=%.3f, Kd=%.3f, rate=%.1f Hz",
      kp_, kd_, publish_rate_);
  }

private:
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 12 || msg->velocity.size() < 12) {
      return;
    }

    // The JointState ordering from the current Lite3 broadcaster
    // is already the validated 12-DOF order.
    for (std::size_t i = 0; i < 12; ++i) {
      q_(i) = msg->position[i];
      qdot_(i) = msg->velocity[i];
    }

    received_state_ = true;
  }

  void controlLoop()
  {
    if (!received_state_) {
      return;
    }

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

    computeLegTorque(
      lite3_kinematics::Leg::FL,
      q_.segment<3>(0),
      qdot_.segment<3>(0),
      q_des_.segment<3>(0),
      qdot_des_.segment<3>(0),
      tau.segment<3>(0));

    computeLegTorque(
      lite3_kinematics::Leg::FR,
      q_.segment<3>(3),
      qdot_.segment<3>(3),
      q_des_.segment<3>(3),
      qdot_des_.segment<3>(3),
      tau.segment<3>(3));

    computeLegTorque(
      lite3_kinematics::Leg::HL,
      q_.segment<3>(6),
      qdot_.segment<3>(6),
      q_des_.segment<3>(6),
      qdot_des_.segment<3>(6),
      tau.segment<3>(6));

    computeLegTorque(
      lite3_kinematics::Leg::HR,
      q_.segment<3>(9),
      qdot_.segment<3>(9),
      q_des_.segment<3>(9),
      qdot_des_.segment<3>(9),
      tau.segment<3>(9));

    // ------------------------------------------------------------
    // Publish torque command
    // ------------------------------------------------------------
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12);

    for (std::size_t i = 0; i < 12; ++i) {
      msg.data[i] = tau(static_cast<Eigen::Index>(i));
    }

    effort_pub_->publish(msg);
  }

  void computeLegTorque(
    lite3_kinematics::Leg leg,
    const Eigen::Vector3d &q,
    const Eigen::Vector3d &qdot,
    const Eigen::Vector3d &q_des,
    const Eigen::Vector3d &qdot_des,
    Eigen::Vector3d tau)
  {
    const Eigen::Vector3d e = q_des - q;
    const Eigen::Vector3d edot = qdot_des - qdot;

    const Eigen::Vector3d gravity =
      dynamics_.gravityTorque(leg, q);

    Eigen::Vector3d torque =
      kp_ * e +
      kd_ * edot +
      gravity;

    // Actuator limits:
    // HipX, HipY = 24 Nm
    // Knee       = 36 Nm
    torque(0) = std::clamp(torque(0), -24.0, 24.0);
    torque(1) = std::clamp(torque(1), -24.0, 24.0);
    torque(2) = std::clamp(torque(2), -36.0, 36.0);

    tau = torque;
  }

  // --------------------------------------------------------------
  // State
  // --------------------------------------------------------------
  lite3_dynamics::Lite3SingleLegDynamics dynamics_;

  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qdot_ = Eigen::VectorXd::Zero(12);

  Eigen::VectorXd q_des_ = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd qdot_des_ = Eigen::VectorXd::Zero(12);

  std::array<std::string, 12> joint_names_{};

  double kp_;
  double kd_;
  double publish_rate_;

  bool received_state_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    effort_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PdGravityController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}