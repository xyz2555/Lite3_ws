#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
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

class StairTrajectoryValidator : public rclcpp::Node
{
public:
  StairTrajectoryValidator()
  : Node("stair_trajectory_validator")
  {
    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------
    declare_parameter<double>("height", 0.185);
    declare_parameter<double>("duration", 1.5);

    height_ = get_parameter("height").as_double();
    total_duration_ =
      get_parameter("duration").as_double();

    if (height_ <= 0.0 ||
        total_duration_ <= 0.0)
    {
      throw std::runtime_error(
        "height and duration must be > 0");
    }

    // ------------------------------------------------------------
    // Joint order = controllers.yaml
    // ------------------------------------------------------------
    joint_names_ = {
      "FL_HipX_joint",
      "FL_HipY_joint",
      "FL_Knee_joint",

      "FR_HipX_joint",
      "FR_HipY_joint",
      "FR_Knee_joint",

      "HL_HipX_joint",
      "HL_HipY_joint",
      "HL_Knee_joint",

      "HR_HipX_joint",
      "HR_HipY_joint",
      "HR_Knee_joint"
    };

    for (size_t i = 0;
         i < joint_names_.size();
         ++i)
    {
      joint_index_[joint_names_[i]] = i;
    }

    q_actual_ =
      Eigen::VectorXd::Zero(12);

    // ------------------------------------------------------------
    // Analytical standing posture
    //
    // These are the same values used in the
    // smooth_stair_trajectory_analyzer.
    // ------------------------------------------------------------
    
    q_standing_ = Eigen::VectorXd::Zero(12);

    q_standing_ <<
      -0.02073, -0.67214, 1.32366,
       0.01497, -0.67765, 1.33907,
      -0.02465, -0.64953, 1.32289,
       0.01714, -0.65116, 1.33529;

    // ------------------------------------------------------------
    // Stair geometry
    // Must match analyzer exactly.
    // ------------------------------------------------------------
    constexpr double TREAD_DEPTH = 0.315;
    constexpr double CLEARANCE = 0.040;
    constexpr double FOOT_RADIUS = 0.022;
    constexpr double PRE_RISER = 0.125;
    constexpr double LANDING_OFFSET = 0.050;

    (void)TREAD_DEPTH;  // The waypoint definition does not use
                        // tread depth directly.

    // ------------------------------------------------------------
    // Current standing FL foot position.
    // ------------------------------------------------------------
    Eigen::Vector3d q_fl_stand(
      q_standing_(0),
      q_standing_(1),
      q_standing_(2));

    p_stand_ =
      kin_.forward(
        lite3_kinematics::Leg::FL,
        q_fl_stand);

    // ------------------------------------------------------------
    // Exact waypoint construction from analyzer
    // ------------------------------------------------------------
    const double x_riser =
      p_stand_.x() + PRE_RISER;

    const double x_land =
      x_riser + LANDING_OFFSET;

    const double z_ground =
      p_stand_.z() - FOOT_RADIUS;

    const double z_tread =
      z_ground + height_;

    const double z_clear =
      z_tread +
      FOOT_RADIUS +
      CLEARANCE;

    const double z_land =
      z_tread + FOOT_RADIUS;

    P0_ = p_stand_;

    P1_ = Eigen::Vector3d(
      p_stand_.x(),
      p_stand_.y(),
      p_stand_.z() + CLEARANCE);

    P2_ = Eigen::Vector3d(
      x_riser - FOOT_RADIUS,
      p_stand_.y(),
      z_clear);

    P3_ = Eigen::Vector3d(
      x_riser + FOOT_RADIUS,
      p_stand_.y(),
      z_clear);

    P4_ = Eigen::Vector3d(
      x_land,
      p_stand_.y(),
      z_clear);

    P5_ = Eigen::Vector3d(
      x_land,
      p_stand_.y(),
      z_land);

    // ------------------------------------------------------------
    // Same segment ratios as analyzer:
    //
    // 0.20 / 0.30 / 0.20 / 0.30 / 0.20
    // ------------------------------------------------------------
    constexpr double BASE_T1 = 0.20;
    constexpr double BASE_T2 = 0.30;
    constexpr double BASE_T3 = 0.20;
    constexpr double BASE_T4 = 0.30;
    constexpr double BASE_T5 = 0.20;

    constexpr double BASE_TOTAL =
      BASE_T1 +
      BASE_T2 +
      BASE_T3 +
      BASE_T4 +
      BASE_T5;

    const double scale =
      total_duration_ / BASE_TOTAL;

    segment_duration_ = {
      BASE_T1 * scale,
      BASE_T2 * scale,
      BASE_T3 * scale,
      BASE_T4 * scale,
      BASE_T5 * scale
    };

    segment_start_.resize(5);

    segment_start_[0] = 0.0;

    for (size_t i = 1; i < 5; ++i)
    {
      segment_start_[i] =
        segment_start_[i - 1] +
        segment_duration_[i - 1];
    }

    waypoints_ = {
      P0_, P1_, P2_, P3_, P4_, P5_
    };

    // Seed starts from analytical standing configuration.
    q_seed_ = q_fl_stand;

    // ------------------------------------------------------------
    // ROS interfaces
    // ------------------------------------------------------------
    command_pub_ =
      create_publisher<
        std_msgs::msg::Float64MultiArray>(
          "/lite3_position_controller/commands",
          10);

    joint_state_sub_ =
      create_subscription<
        sensor_msgs::msg::JointState>(
          "/joint_states",
          10,
          std::bind(
            &StairTrajectoryValidator::
              jointStateCallback,
            this,
            std::placeholders::_1));

    timer_ =
      create_wall_timer(
        10ms,
        std::bind(
          &StairTrajectoryValidator::
            controlLoop,
          this));

    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Lite3 Stair Trajectory Validator");

    RCLCPP_INFO(
      get_logger(),
      "Height   = %.3f m",
      height_);

    RCLCPP_INFO(
      get_logger(),
      "Duration = %.3f s",
      total_duration_);

    printWaypoint("P0", P0_);
    printWaypoint("P1", P1_);
    printWaypoint("P2", P2_);
    printWaypoint("P3", P3_);
    printWaypoint("P4", P4_);
    printWaypoint("P5", P5_);

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }

private:

  static double quintic(double u)
  {
    u = std::clamp(u, 0.0, 1.0);

    return
      10.0 * std::pow(u, 3) -
      15.0 * std::pow(u, 4) +
       6.0 * std::pow(u, 5);
  }

  void printWaypoint(
    const std::string & name,
    const Eigen::Vector3d & p)
  {
    RCLCPP_INFO(
      get_logger(),
      "%s = [%+.6f, %+.6f, %+.6f]",
      name.c_str(),
      p.x(),
      p.y(),
      p.z());
  }

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    size_t matched = 0;

    for (size_t i = 0;
         i < msg->name.size();
         ++i)
    {
      auto it =
        joint_index_.find(msg->name[i]);

      if (it != joint_index_.end() &&
          i < msg->position.size())
      {
        q_actual_(
          static_cast<Eigen::Index>(
            it->second)) =
          msg->position[i];

        ++matched;
      }
    }

    if (matched == 12)
    {
      received_joint_state_ = true;
    }
  }

  void publishStanding()
  {
    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);

    for (size_t i = 0; i < 12; ++i)
    {
      msg.data[i] =
        q_standing_(i);
    }

    command_pub_->publish(msg);
  }

  void controlLoop()
  {
    // ------------------------------------------------------------
    // Wait for all joint states.
    // ------------------------------------------------------------
    if (!received_joint_state_)
    {
      return;
    }

    // ------------------------------------------------------------
    // Settling phase: force analytical standing posture.
    // ------------------------------------------------------------
    if (phase_ == Phase::SETTLING)
    {
      publishStanding();

      if (!settling_started_)
      {
        phase_start_time_ = now();
        settling_started_ = true;

        RCLCPP_INFO(
          get_logger(),
          "Joint states received. "
          "Entering settling phase.");

        return;
      }

      const double settling_time =
        (now() - phase_start_time_).seconds();

      if (settling_time >= 1.0)
      {
        const double q_error =
          (
            q_standing_ -
            q_actual_
          ).norm();

        RCLCPP_INFO(
          get_logger(),
          "Standing settling error = %.6e rad",
          q_error);

        if (q_error < 1e-3)
        {
          phase_ = Phase::TRAJECTORY;
          phase_start_time_ = now();

          RCLCPP_INFO(
            get_logger(),
            "Standing posture settled.");

          RCLCPP_INFO(
            get_logger(),
            "Starting stair trajectory.");
        }
      }

      return;
    }

    // ------------------------------------------------------------
    // Trajectory phase
    // ------------------------------------------------------------
    if (phase_ == Phase::TRAJECTORY)
    {
      const double t =
        (now() - phase_start_time_).seconds();

      if (t >= total_duration_)
      {
        phase_ = Phase::HOLD;

        printFinalResult();

        publishStanding();

        return;
      }

      // Determine current segment.
      size_t segment = 4;

      for (size_t i = 0; i < 5; ++i)
      {
        if (t <
            segment_start_[i] +
            segment_duration_[i])
        {
          segment = i;
          break;
        }
      }

      const double local_t =
        t -
        segment_start_[segment];

      const double u =
        local_t /
        segment_duration_[segment];

      const double s =
        quintic(u);

      const Eigen::Vector3d & p0 =
        waypoints_[segment];

      const Eigen::Vector3d & p1 =
        waypoints_[segment + 1];

      const Eigen::Vector3d p_des =
        p0 + s * (p1 - p0);

      // ----------------------------------------------------------
      // IK
      // ----------------------------------------------------------
      const auto ik =
        kin_.inverse(
          lite3_kinematics::Leg::FL,
          p_des,
          &q_seed_);

      if (!ik.success)
      {
        RCLCPP_ERROR(
          get_logger(),
          "IK failed at t=%.6f s",
          t);

        return;
      }

      const Eigen::Vector3d q_fl =
        ik.q;

      q_seed_ = q_fl;

      // ----------------------------------------------------------
      // Full-body command
      //
      // Only FL moves.
      // FR, HL, HR stay at analytical standing posture.
      // ----------------------------------------------------------
      std_msgs::msg::Float64MultiArray msg;

      msg.data.resize(12);

      for (size_t i = 0; i < 12; ++i)
      {
        msg.data[i] =
          q_standing_(i);
      }

      msg.data[0] = q_fl(0);
      msg.data[1] = q_fl(1);
      msg.data[2] = q_fl(2);

      command_pub_->publish(msg);

      // ----------------------------------------------------------
      // Actual FL Cartesian position.
      // ----------------------------------------------------------
      Eigen::Vector3d q_actual_fl(
        q_actual_(0),
        q_actual_(1),
        q_actual_(2));

      const Eigen::Vector3d p_actual =
        kin_.forward(
          lite3_kinematics::Leg::FL,
          q_actual_fl);

      const double position_error =
        (p_des - p_actual).norm();

      max_position_error_ =
        std::max(
          max_position_error_,
          position_error);

      const double q_error =
        (q_fl - q_actual_fl).norm();

      max_joint_error_ =
        std::max(
          max_joint_error_,
          q_error);

      if (sample_count_ % 25 == 0)
      {
        RCLCPP_INFO(
          get_logger(),
          "t=%+.3f seg=%zu "
          "Pdes=[%+.4f %+.4f %+.4f] "
          "Perr=%.3e m "
          "Qerr=%.3e rad",
          t,
          segment + 1,
          p_des.x(),
          p_des.y(),
          p_des.z(),
          position_error,
          q_error);
      }

      ++sample_count_;

      return;
    }

    // ------------------------------------------------------------
    // Hold final standing posture.
    // ------------------------------------------------------------
    if (phase_ == Phase::HOLD)
    {
      publishStanding();
    }
  }

  void printFinalResult()
  {
    Eigen::Vector3d q_actual_fl(
      q_actual_(0),
      q_actual_(1),
      q_actual_(2));

    const Eigen::Vector3d p_actual =
      kin_.forward(
        lite3_kinematics::Leg::FL,
        q_actual_fl);

    const double final_error =
      (P5_ - p_actual).norm();

    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Stair Trajectory Result");

    RCLCPP_INFO(
      get_logger(),
      "Height                  : %.1f cm",
      height_ * 100.0);

    RCLCPP_INFO(
      get_logger(),
      "Duration                : %.3f s",
      total_duration_);

    RCLCPP_INFO(
      get_logger(),
      "Maximum Cartesian error : %.6e m",
      max_position_error_);

    RCLCPP_INFO(
      get_logger(),
      "Maximum joint error     : %.6e rad",
      max_joint_error_);

    RCLCPP_INFO(
      get_logger(),
      "Target P5               : [%+.6f, %+.6f, %+.6f]",
      P5_.x(),
      P5_.y(),
      P5_.z());

    RCLCPP_INFO(
      get_logger(),
      "Actual final position   : [%+.6f, %+.6f, %+.6f]",
      p_actual.x(),
      p_actual.y(),
      p_actual.z());

    RCLCPP_INFO(
      get_logger(),
      "Final Cartesian error   : %.6e m",
      final_error);

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }

  enum class Phase
  {
    SETTLING,
    TRAJECTORY,
    HOLD
  };

  lite3_kinematics::Lite3Kinematics kin_;

  std::vector<std::string> joint_names_;

  std::unordered_map<
    std::string,
    size_t> joint_index_;

  Eigen::VectorXd q_standing_;
  Eigen::VectorXd q_actual_;

  Eigen::Vector3d q_seed_;

  Eigen::Vector3d p_stand_;

  Eigen::Vector3d P0_;
  Eigen::Vector3d P1_;
  Eigen::Vector3d P2_;
  Eigen::Vector3d P3_;
  Eigen::Vector3d P4_;
  Eigen::Vector3d P5_;

  std::vector<
    Eigen::Vector3d> waypoints_;

  std::vector<double>
    segment_duration_;

  std::vector<double>
    segment_start_;

  double height_;
  double total_duration_;

  double max_position_error_{0.0};
  double max_joint_error_{0.0};

  size_t sample_count_{0};

  bool received_joint_state_{false};
  bool settling_started_{false};

  Phase phase_{Phase::SETTLING};

  rclcpp::Time phase_start_time_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<StairTrajectoryValidator>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}