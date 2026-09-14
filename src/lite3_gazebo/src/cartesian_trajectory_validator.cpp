#include <algorithm>
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

class CartesianTrajectoryValidator : public rclcpp::Node
{
public:
  CartesianTrajectoryValidator()
  : Node("cartesian_trajectory_validator")
  {
    joint_names_ = {
      "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
      "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
      "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
      "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
    };

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = i;
    }

    q_actual_ = Eigen::VectorXd::Zero(12);

    // Standing posture currently used in Gazebo
    q_standing_ <<
      -0.02073, -0.67214, 1.32366,
       0.01497, -0.67765, 1.33907,
      -0.02465, -0.64953, 1.32289,
       0.01714, -0.65116, 1.33529;

    // Validate initial FL Cartesian position using the
    // already validated kinematics library.
    Eigen::Vector3d q_fl(
      q_standing_(0),
      q_standing_(1),
      q_standing_(2));

    p0_ = kin_.forward(
      lite3_kinematics::Leg::FL,
      q_fl);

    // Small smooth Cartesian displacement.
    p1_ = p0_;
    p1_.x() -= 0.030;
    p1_.z() += 0.030;

    declare_parameter<double>("duration", 1.0);
duration_ = get_parameter("duration").as_double();

if (duration_ <= 0.0) {
  throw std::runtime_error("Trajectory duration must be > 0");
}

    // Start from current FL configuration.
    q_seed_ = q_fl;

    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lite3_position_controller/commands", 10);

    joint_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(
          &CartesianTrajectoryValidator::jointStateCallback,
          this,
          std::placeholders::_1));

    timer_ = create_wall_timer(
      10ms,
      std::bind(
        &CartesianTrajectoryValidator::controlLoop,
        this));

    // start_time_ = now();
    phase_ = Phase::WAIT_FOR_STATE;

    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Lite3 Cartesian Trajectory Validator");

    RCLCPP_INFO(
      get_logger(),
      "P0 = [%+.6f, %+.6f, %+.6f]",
      p0_.x(), p0_.y(), p0_.z());

    RCLCPP_INFO(
      get_logger(),
      "P1 = [%+.6f, %+.6f, %+.6f]",
      p1_.x(), p1_.y(), p1_.z());

    RCLCPP_INFO(
      get_logger(),
      "Trajectory duration = %.3f s",
      duration_);

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }

private:

  enum class Phase
{
  WAIT_FOR_STATE,
  SETTLING,
  TRAJECTORY,
  HOLD
};

Phase phase_{Phase::WAIT_FOR_STATE};

rclcpp::Time phase_start_time_;

  static double quintic(double u)
  {
    u = std::clamp(u, 0.0, 1.0);

    return
      10.0 * std::pow(u, 3) -
      15.0 * std::pow(u, 4) +
       6.0 * std::pow(u, 5);
  }

  static double quinticDerivative(double u)
  {
    u = std::clamp(u, 0.0, 1.0);

    return
      30.0 * std::pow(u, 2) -
      60.0 * std::pow(u, 3) +
      30.0 * std::pow(u, 4);
  }

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t matched = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = joint_index_.find(msg->name[i]);

      if (it != joint_index_.end() &&
          i < msg->position.size())
      {
        q_actual_(
          static_cast<Eigen::Index>(it->second)) =
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
  // ------------------------------------------------------------
  // Phase 1: wait until all 12 joint states are available
  // ------------------------------------------------------------
  if (phase_ == Phase::WAIT_FOR_STATE)
  {
    if (!received_joint_state_) {
      return;
    }

    phase_ = Phase::SETTLING;
    phase_start_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Joint states received. Entering settling phase.");

    return;
  }

  // ------------------------------------------------------------
  // Phase 2: hold standing posture before starting trajectory
  // ------------------------------------------------------------
  if (phase_ == Phase::SETTLING)
  {
    publishStanding();

    const double elapsed =
      (now() - phase_start_time_).seconds();

    if (elapsed >= 1.0)
    {
      // Check whether actual configuration is close enough
      // to the standing configuration.
      Eigen::VectorXd q_error =
        q_standing_ - q_actual_;

      const double error_norm =
        q_error.norm();

      RCLCPP_INFO(
        get_logger(),
        "Standing settling error = %.6e rad",
        error_norm);

      if (error_norm < 1e-3)
      {
        phase_ = Phase::TRAJECTORY;
        phase_start_time_ = now();

        RCLCPP_INFO(
          get_logger(),
          "Standing posture settled.");
        RCLCPP_INFO(
          get_logger(),
          "Starting Cartesian trajectory.");
      }
    }

    return;
  }

  // ------------------------------------------------------------
  // Phase 3: execute Cartesian trajectory
  // ------------------------------------------------------------
  if (phase_ == Phase::TRAJECTORY)
  {
    const double elapsed =
      (now() - phase_start_time_).seconds();

    if (elapsed >= duration_)
{
  reportFinalError();

  phase_ = Phase::HOLD;
  publishStanding();

  return;
}

    const double u =
      std::clamp(elapsed / duration_, 0.0, 1.0);

    const double s = quintic(u);

    Eigen::Vector3d p_des =
      p0_ + s * (p1_ - p0_);

    // ----------------------------------------------------------
    // IK with previous solution as seed
    // ----------------------------------------------------------
    auto ik_result =
      kin_.inverse(
        lite3_kinematics::Leg::FL,
        p_des,
        &q_seed_);

    if (!ik_result.success)
    {
      RCLCPP_ERROR(
        get_logger(),
        "IK failed during trajectory at t=%.4f s",
        elapsed);

      return;
    }

    Eigen::Vector3d q_fl =
      ik_result.q;

    q_seed_ = q_fl;

    // ----------------------------------------------------------
    // Full-body joint command
    // ----------------------------------------------------------
    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);

    for (size_t i = 0; i < 12; ++i) {
      msg.data[i] = q_standing_(i);
    }

    msg.data[0] = q_fl(0);
    msg.data[1] = q_fl(1);
    msg.data[2] = q_fl(2);

    command_pub_->publish(msg);

    // ----------------------------------------------------------
    // Actual Cartesian position
    // ----------------------------------------------------------
    Eigen::Vector3d q_actual_fl(
      q_actual_(0),
      q_actual_(1),
      q_actual_(2));

    Eigen::Vector3d p_actual =
      kin_.forward(
        lite3_kinematics::Leg::FL,
        q_actual_fl);

    Eigen::Vector3d error =
      p_des - p_actual;

    const double e_norm =
      error.norm();

    const double q_error =
      (q_fl - q_actual_fl).norm();

    max_cartesian_error_ =
      std::max(
        max_cartesian_error_,
        e_norm);

    max_joint_error_ =
      std::max(
        max_joint_error_,
        q_error);

    if (sample_count_ % 25 == 0)
    {
      RCLCPP_INFO(
        get_logger(),
        "t=%+.3f | "
        "Pdes=[%+.4f %+.4f %+.4f] | "
        "Perr=%.3e m | "
        "Qerr=%.3e rad",
        elapsed,
        p_des.x(),
        p_des.y(),
        p_des.z(),
        e_norm,
        q_error);
    }

    sample_count_++;

    return;
  }

  // ------------------------------------------------------------
  // Phase 4: hold standing
  // ------------------------------------------------------------
  if (phase_ == Phase::HOLD)
  {
    publishStanding();
    return;
  }
}

  void publishStanding()
  {
    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);

    for (size_t i = 0; i < 12; ++i) {
      msg.data[i] = q_standing_(i);
    }

    command_pub_->publish(msg);
  }

  void reportFinalError()
{
  Eigen::Vector3d q_actual_fl(
    q_actual_(0),
    q_actual_(1),
    q_actual_(2));

  Eigen::Vector3d p_actual =
    kin_.forward(
      lite3_kinematics::Leg::FL,
      q_actual_fl);

  Eigen::Vector3d final_error =
    p1_ - p_actual;

  const double final_error_norm =
    final_error.norm();

  RCLCPP_INFO(
    get_logger(),
    "==============================================");

  RCLCPP_INFO(
    get_logger(),
    "Cartesian Trajectory Result");

  RCLCPP_INFO(
    get_logger(),
    "Maximum Cartesian error = %.6e m",
    max_cartesian_error_);

  RCLCPP_INFO(
    get_logger(),
    "Maximum joint error     = %.6e rad",
    max_joint_error_);

  RCLCPP_INFO(
    get_logger(),
    "Target final position   = [%+.6f, %+.6f, %+.6f]",
    p1_.x(),
    p1_.y(),
    p1_.z());

  RCLCPP_INFO(
    get_logger(),
    "Actual final position   = [%+.6f, %+.6f, %+.6f]",
    p_actual.x(),
    p_actual.y(),
    p_actual.z());

  RCLCPP_INFO(
    get_logger(),
    "Final Cartesian error   = %.6e m",
    final_error_norm);

  if (max_cartesian_error_ < 1e-3)
  {
    RCLCPP_INFO(
      get_logger(),
      "RESULT: PASS - max Cartesian error < 1 mm");
  }
  else
  {
    RCLCPP_WARN(
      get_logger(),
      "RESULT: CHECK - max Cartesian error >= 1 mm");
  }

  RCLCPP_INFO(
    get_logger(),
    "==============================================");
}

  lite3_kinematics::Lite3Kinematics kin_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;

  Eigen::VectorXd q_standing_ =
    Eigen::VectorXd::Zero(12);

  Eigen::VectorXd q_actual_;
  Eigen::Vector3d q_seed_;

  Eigen::Vector3d p0_;
  Eigen::Vector3d p1_;

  double duration_;
  double max_cartesian_error_{0.0};
  double max_joint_error_{0.0};

  size_t sample_count_{0};

  bool received_joint_state_{false};
  bool reported_{false};

  rclcpp::Time start_time_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<CartesianTrajectoryValidator>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}