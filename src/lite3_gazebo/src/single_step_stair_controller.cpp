#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lite3_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

class SingleStepStairController : public rclcpp::Node
{
public:
  enum class Phase
  {
    WAITING,
    PREPARE,
    SWING_FL,
    SETTLE_FL,
    SWING_FR,
    SETTLE_FR,
    SWING_HL,
    SETTLE_HL,
    SWING_HR,
    SETTLE_HR,
    DONE
  };

  SingleStepStairController()
  : Node("single_step_stair_controller"),
    phase_(Phase::WAITING),
    experiment_started_(false),
    got_joint_state_(false),
    got_odom_(false),
    targets_initialized_(false)
  {
    // ============================================================
    // Parameters
    // ============================================================
    declare_parameter<bool>("start_experiment", false);

    declare_parameter<double>("stair_height", 0.185);
    declare_parameter<double>("tread_depth", 0.315);
    declare_parameter<double>("stair_start_x", 0.0);

    declare_parameter<double>("front_landing_x", 0.22);
    declare_parameter<double>("hind_landing_x", 0.08);

    declare_parameter<double>("clearance", 0.060);
    declare_parameter<double>("foot_radius", 0.022);

    declare_parameter<double>("swing_duration", 3.0);
    declare_parameter<double>("settle_duration", 1.0);
    declare_parameter<double>("prepare_duration", 2.0);

    declare_parameter<double>("position_command_rate_hz", 100.0);

    declare_parameter<double>("riser_clearance_x", 0.050);
    declare_parameter<double>("landing_margin", 0.040);

    declare_parameter<double>("ik_warning_period_s", 1.0);

    stair_height_ = get_parameter("stair_height").as_double();
    tread_depth_ = get_parameter("tread_depth").as_double();
    stair_start_x_ = get_parameter("stair_start_x").as_double();

    front_landing_x_ =
      get_parameter("front_landing_x").as_double();

    hind_landing_x_ =
      get_parameter("hind_landing_x").as_double();

    clearance_ =
      get_parameter("clearance").as_double();

    foot_radius_ =
      get_parameter("foot_radius").as_double();

    swing_duration_ =
      get_parameter("swing_duration").as_double();

    settle_duration_ =
      get_parameter("settle_duration").as_double();

    prepare_duration_ =
      get_parameter("prepare_duration").as_double();

    riser_clearance_x_ =
      get_parameter("riser_clearance_x").as_double();

    landing_margin_ =
      get_parameter("landing_margin").as_double();

    const double rate_hz =
      get_parameter("position_command_rate_hz").as_double();

    ik_warning_period_s_ =
      get_parameter("ik_warning_period_s").as_double();

    if (stair_height_ <= 0.0 ||
        tread_depth_ <= 0.0 ||
        clearance_ <= 0.0 ||
        foot_radius_ <= 0.0 ||
        swing_duration_ <= 0.0 ||
        settle_duration_ < 0.0 ||
        prepare_duration_ < 0.0 ||
        rate_hz <= 0.0)
    {
      throw std::runtime_error(
        "Invalid stair controller parameter.");
    }

    // ============================================================
    // Joint order
    // Harus sesuai dengan controllers.yaml
    // ============================================================
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

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_index_[joint_names_[i]] = i;
    }

    legs_ = {
      lite3_kinematics::Leg::FL,
      lite3_kinematics::Leg::FR,
      lite3_kinematics::Leg::HL,
      lite3_kinematics::Leg::HR
    };

    // One-foot-at-a-time crawl.
    swing_order_ = {0, 1, 2, 3};

    q_actual_ = Eigen::VectorXd::Zero(12);
    q_seed_ = Eigen::VectorXd::Zero(12);
    q_command_ = Eigen::VectorXd::Zero(12);
    qdot_actual_ = Eigen::VectorXd::Zero(12);

    foot_target_world_.fill(Eigen::Vector3d::Zero());
    foot_initial_world_.fill(Eigen::Vector3d::Zero());
    foot_landing_world_.fill(Eigen::Vector3d::Zero());

    // ============================================================
    // ROS interfaces
    // ============================================================
    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        "/lite3_position_controller/commands",
        10);

    joint_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        20,
        std::bind(
          &SingleStepStairController::jointStateCallback,
          this,
          std::placeholders::_1));

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/lite3/odometry",
        20,
        std::bind(
          &SingleStepStairController::odometryCallback,
          this,
          std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      std::bind(
        &SingleStepStairController::controlLoop,
        this));

    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Lite3 Single-Step Stair Controller");

    RCLCPP_INFO(
      get_logger(),
      "Stair: height=%.3f m, tread=%.3f m, start_x=%.3f m",
      stair_height_,
      tread_depth_,
      stair_start_x_);

    RCLCPP_INFO(
      get_logger(),
      "Landing x: front=%.3f m, hind=%.3f m",
      front_landing_x_,
      hind_landing_x_);

    RCLCPP_INFO(
      get_logger(),
      "Sequence: FL -> FR -> HL -> HR");

    RCLCPP_INFO(
      get_logger(),
      "Position command rate = %.1f Hz",
      rate_hz);

    RCLCPP_INFO(
      get_logger(),
      "Set start_experiment:=true to begin");

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }

private:

  // ============================================================
  // ROS callbacks
  // ============================================================

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::size_t matched = 0;

    for (std::size_t i = 0;
         i < msg->name.size();
         ++i)
    {
      const auto it =
        joint_index_.find(msg->name[i]);

      if (it == joint_index_.end())
      {
        continue;
      }

      if (i >= msg->position.size())
      {
        continue;
      }

      const std::size_t idx = it->second;

      q_actual_(
        static_cast<Eigen::Index>(idx)) =
        msg->position[i];

      if (i < msg->velocity.size())
      {
        qdot_actual_(
          static_cast<Eigen::Index>(idx)) =
          msg->velocity[i];
      }

      ++matched;
    }

    got_joint_state_ =
      (matched == 12);
  }

  void odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    base_position_world_ =
      Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    base_orientation_world_ =
      Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    const double norm =
      base_orientation_world_.norm();

    if (norm < 1e-9)
    {
      got_odom_ = false;
      return;
    }

    base_orientation_world_.normalize();
    got_odom_ = true;
  }

  // ============================================================
  // Frame transformation
  // ============================================================

  Eigen::Vector3d torsoToWorld(
    const Eigen::Vector3d &p_torso) const
  {
    return base_position_world_ +
           base_orientation_world_ *
           p_torso;
  }

  Eigen::Vector3d worldToTorso(
    const Eigen::Vector3d &p_world) const
  {
    return base_orientation_world_.inverse() *
           (p_world - base_position_world_);
  }

  // ============================================================
  // Initialize world-space foot targets
  // ============================================================

  bool initializeFootTargets()
  {
    if (!got_joint_state_ ||
        !got_odom_)
    {
      return false;
    }

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      const Eigen::Vector3d q_leg =
        getLegQ(q_actual_, leg);

      const Eigen::Vector3d p_torso =
        kinematics_.forward(
          legs_[leg],
          q_leg);

      const Eigen::Vector3d p_world =
        torsoToWorld(p_torso);

      foot_initial_world_[leg] =
        p_world;

      foot_target_world_[leg] =
        p_world;

      q_seed_.segment<3>(
        3 * leg) =
        q_leg;
    }

    // Estimate ground level from current feet.
    double ground_center_z = 0.0;

    for (const auto &p :
         foot_initial_world_)
    {
      ground_center_z += p.z();
    }

    ground_center_z /= 4.0;

    // Kinematic foot location is treated as approximately
    // the foot sphere center.
    ground_surface_z_ =
      ground_center_z - foot_radius_;

    top_surface_z_ =
      ground_surface_z_ +
      stair_height_;

    top_foot_center_z_ =
      top_surface_z_ +
      foot_radius_;

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      const bool front =
        (leg == 0 || leg == 1);

      const double landing_x =
        front
        ? front_landing_x_
        : hind_landing_x_;

      foot_landing_world_[leg] =
        Eigen::Vector3d(
          landing_x,
          foot_initial_world_[leg].y(),
          top_foot_center_z_);
    }

    targets_initialized_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Initial world foot centers:");

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      RCLCPP_INFO(
        get_logger(),
        "  %s = [%+.3f, %+.3f, %+.3f] m",
        legName(leg).c_str(),
        foot_initial_world_[leg].x(),
        foot_initial_world_[leg].y(),
        foot_initial_world_[leg].z());
    }

    RCLCPP_INFO(
      get_logger(),
      "Estimated ground z = %.3f m",
      ground_surface_z_);

    RCLCPP_INFO(
      get_logger(),
      "Top foot-center z = %.3f m",
      top_foot_center_z_);

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      RCLCPP_INFO(
        get_logger(),
        "  %s landing = [%+.3f, %+.3f, %+.3f] m",
        legName(leg).c_str(),
        foot_landing_world_[leg].x(),
        foot_landing_world_[leg].y(),
        foot_landing_world_[leg].z());
    }

    return true;
  }

  // ============================================================
  // Quintic interpolation
  // Zero velocity and acceleration at endpoints.
  // ============================================================

  static double quintic(double u)
  {
    u = std::clamp(
      u,
      0.0,
      1.0);

    return
      10.0 * std::pow(u, 3) -
      15.0 * std::pow(u, 4) +
       6.0 * std::pow(u, 5);
  }

  // ============================================================
  // Generate stair swing trajectory
  // ============================================================

  Eigen::Vector3d generateSwingWorldPosition(
    int leg,
    double u) const
  {
    const Eigen::Vector3d p0 =
      foot_initial_world_[leg];

    const Eigen::Vector3d pf =
      foot_landing_world_[leg];

    const double riser_x =
      stair_start_x_;

    const double pre_riser_x =
      riser_x -
      riser_clearance_x_;

    const double post_riser_x =
      riser_x +
      landing_margin_;

    const double y =
      p0.y();

    const double z_lift =
      top_foot_center_z_ +
      clearance_;

    // ----------------------------------------------------------
    // Waypoints:
    //
    // p0:
    // initial position
    //
    // p1:
    // initial vertical lift
    //
    // p2:
    // move in front of riser
    //
    // p3:
    // cross riser
    //
    // p4:
    // move toward landing x
    //
    // p5:
    // descend to landing
    // ----------------------------------------------------------

    const std::array<
      Eigen::Vector3d,
      6> waypoints = {

      p0,

      Eigen::Vector3d(
        p0.x(),
        y,
        std::max(
          p0.z() + clearance_,
          z_lift)),

      Eigen::Vector3d(
        pre_riser_x,
        y,
        z_lift),

      Eigen::Vector3d(
        post_riser_x,
        y,
        z_lift),

      Eigen::Vector3d(
        pf.x(),
        y,
        z_lift),

      pf
    };

    constexpr std::array<
      double,
      5> segment_ratio = {
        0.20,
        0.25,
        0.20,
        0.20,
        0.15
    };

    double t0 = 0.0;

    std::size_t segment = 4;

    for (std::size_t i = 0;
         i < segment_ratio.size();
         ++i)
    {
      const double t1 =
        t0 +
        segment_ratio[i];

      if (u <= t1 ||
          i == segment_ratio.size() - 1)
      {
        segment = i;

        const double local_u =
          (u - t0) /
          segment_ratio[i];

        const double s =
          quintic(local_u);

        return
          waypoints[segment] +
          s *
          (waypoints[segment + 1] -
           waypoints[segment]);
      }

      t0 = t1;
    }

    return pf;
  }

  // ============================================================
  // Current active swing leg
  // ============================================================

  int activeSwingLeg() const
  {
    switch (phase_)
    {
      case Phase::SWING_FL:
        return 0;

      case Phase::SWING_FR:
        return 1;

      case Phase::SWING_HL:
        return 2;

      case Phase::SWING_HR:
        return 3;

      default:
        return -1;
    }
  }

  // ============================================================
  // Phase timing
  // ============================================================

  double phaseElapsed() const
  {
    return
      (now() - phase_start_time_)
      .seconds();
  }

  void setPhase(
    Phase next)
  {
    phase_ = next;
    phase_start_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "State -> %s",
      phaseName(phase_).c_str());
  }

  // ============================================================
  // Compute joint command from foot world targets
  // ============================================================

  bool computeJointCommand(
    const std::array<
      Eigen::Vector3d,
      4> &foot_world_targets,
    Eigen::VectorXd &q_command)
  {
    q_command =
      q_actual_;

    bool all_ok = true;

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      const Eigen::Vector3d
        target_torso =
          worldToTorso(
            foot_world_targets[leg]);

      const Eigen::Vector3d
        q_seed =
          q_seed_.segment<3>(
            3 * leg);

      const auto ik =
        kinematics_.inverse(
          legs_[leg],
          target_torso,
          &q_seed);

      if (!ik.success)
      {
        all_ok = false;

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          static_cast<int>(
            ik_warning_period_s_ *
            1000.0),
          "IK failed for %s: "
          "target torso=[%.3f %.3f %.3f], "
          "status=%d",
          legName(leg).c_str(),
          target_torso.x(),
          target_torso.y(),
          target_torso.z(),
          static_cast<int>(
            ik.status));

        continue;
      }

      q_command.segment<3>(
        3 * leg) =
        ik.q;

      q_seed_.segment<3>(
        3 * leg) =
        ik.q;
    }

    return all_ok;
  }

  // ============================================================
  // Publish position command
  // ============================================================

  void publishPositionCommand(
    const Eigen::VectorXd &q_command)
  {
    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);

    for (int i = 0;
         i < 12;
         ++i)
    {
      msg.data[
        static_cast<std::size_t>(i)] =
        q_command(i);
    }

    command_pub_->publish(msg);
  }

  // ============================================================
  // Main control loop
  // ============================================================

  void controlLoop()
  {
    if (!got_joint_state_ ||
        !got_odom_)
    {
      return;
    }

    const bool
      start_requested =
        get_parameter(
          "start_experiment")
        .as_bool();

    // ----------------------------------------------------------
    // Start experiment
    // ----------------------------------------------------------

    if (!experiment_started_ &&
        start_requested)
    {
      if (!targets_initialized_)
      {
        if (!initializeFootTargets())
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Waiting for valid "
            "joint state and odometry...");

          return;
        }
      }

      experiment_started_ =
        true;

      setPhase(
        prepare_duration_ > 0.0
        ? Phase::PREPARE
        : Phase::SWING_FL);
    }

    // ----------------------------------------------------------
    // Before start: hold current world footholds
    // ----------------------------------------------------------

    if (!experiment_started_)
    {
      if (!targets_initialized_)
      {
        if (!initializeFootTargets())
        {
          return;
        }
      }

      computeJointCommand(
        foot_target_world_,
        q_command_);

      publishPositionCommand(
        q_command_);

      return;
    }

    // ----------------------------------------------------------
    // PREPARE
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::PREPARE)
    {
      computeJointCommand(
        foot_target_world_,
        q_command_);

      publishPositionCommand(
        q_command_);

      if (phaseElapsed() >=
          prepare_duration_)
      {
        setPhase(
          Phase::SWING_FL);
      }

      return;
    }

    // ----------------------------------------------------------
    // Swing one leg
    // ----------------------------------------------------------

    const int swing_leg =
      activeSwingLeg();

    if (swing_leg >= 0)
    {
      const double u =
        std::clamp(
          phaseElapsed() /
          swing_duration_,
          0.0,
          1.0);

      auto
        desired_targets =
          foot_target_world_;

      desired_targets[swing_leg] =
        generateSwingWorldPosition(
          swing_leg,
          u);

      computeJointCommand(
        desired_targets,
        q_command_);

      publishPositionCommand(
        q_command_);

      // Store current desired world targets.
      foot_target_world_ =
        desired_targets;

      if (u >= 1.0)
      {
        switch (swing_leg)
        {
          case 0:
            setPhase(
              Phase::SETTLE_FL);
            break;

          case 1:
            setPhase(
              Phase::SETTLE_FR);
            break;

          case 2:
            setPhase(
              Phase::SETTLE_HL);
            break;

          case 3:
            setPhase(
              Phase::SETTLE_HR);
            break;

          default:
            break;
        }
      }

      return;
    }

    // ----------------------------------------------------------
    // SETTLE
    // ----------------------------------------------------------

    if (phase_ ==
          Phase::SETTLE_FL ||
        phase_ ==
          Phase::SETTLE_FR ||
        phase_ ==
          Phase::SETTLE_HL ||
        phase_ ==
          Phase::SETTLE_HR)
    {
      computeJointCommand(
        foot_target_world_,
        q_command_);

      publishPositionCommand(
        q_command_);

      if (phaseElapsed() <
          settle_duration_)
      {
        return;
      }

      if (phase_ ==
          Phase::SETTLE_FL)
      {
        setPhase(
          Phase::SWING_FR);
      }
      else if (phase_ ==
               Phase::SETTLE_FR)
      {
        setPhase(
          Phase::SWING_HL);
      }
      else if (phase_ ==
               Phase::SETTLE_HL)
      {
        setPhase(
          Phase::SWING_HR);
      }
      else
      {
        setPhase(
          Phase::DONE);
      }

      return;
    }

    // ----------------------------------------------------------
    // DONE
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::DONE)
    {
      computeJointCommand(
        foot_target_world_,
        q_command_);

      publishPositionCommand(
        q_command_);

      if (!done_reported_)
      {
        done_reported_ = true;
        printFinalState();
      }
    }
  }

  // ============================================================
  // Diagnostics
  // ============================================================

  void printFinalState()
  {
    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Single-step stair sequence completed.");

    RCLCPP_INFO(
      get_logger(),
      "Stair height = %.1f cm, tread = %.1f cm",
      stair_height_ * 100.0,
      tread_depth_ * 100.0);

    RCLCPP_INFO(
      get_logger(),
      "Final base position = [%+.3f %+.3f %+.3f] m",
      base_position_world_.x(),
      base_position_world_.y(),
      base_position_world_.z());

    for (int leg = 0;
         leg < 4;
         ++leg)
    {
      const Eigen::Vector3d q_leg =
        getLegQ(
          q_actual_,
          leg);

      const Eigen::Vector3d p_torso =
        kinematics_.forward(
          legs_[leg],
          q_leg);

      const Eigen::Vector3d
        p_world =
          torsoToWorld(
            p_torso);

      const double e =
        (p_world -
         foot_target_world_[leg])
        .norm();

      RCLCPP_INFO(
        get_logger(),
        "%s final foot error = %.2f mm",
        legName(leg).c_str(),
        e * 1000.0);
    }

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }

  // ============================================================
  // Utility
  // ============================================================

  static Eigen::Vector3d getLegQ(
    const Eigen::VectorXd &q,
    int leg)
  {
    return
      q.segment<3>(
        3 * leg);
  }

  static std::string legName(
    int leg)
  {
    switch (leg)
    {
      case 0:
        return "FL";

      case 1:
        return "FR";

      case 2:
        return "HL";

      case 3:
        return "HR";

      default:
        return "UNKNOWN";
    }
  }

  static std::string phaseName(
    Phase phase)
  {
    switch (phase)
    {
      case Phase::WAITING:
        return "WAITING";

      case Phase::PREPARE:
        return "PREPARE";

      case Phase::SWING_FL:
        return "SWING_FL";

      case Phase::SETTLE_FL:
        return "SETTLE_FL";

      case Phase::SWING_FR:
        return "SWING_FR";

      case Phase::SETTLE_FR:
        return "SETTLE_FR";

      case Phase::SWING_HL:
        return "SWING_HL";

      case Phase::SETTLE_HL:
        return "SETTLE_HL";

      case Phase::SWING_HR:
        return "SWING_HR";

      case Phase::SETTLE_HR:
        return "SETTLE_HR";

      case Phase::DONE:
        return "DONE";

      default:
        return "UNKNOWN";
    }
  }

  // ============================================================
  // Members
  // ============================================================

  lite3_kinematics::Lite3Kinematics
    kinematics_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    command_pub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    joint_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::TimerBase::SharedPtr
    timer_;

  std::vector<std::string>
    joint_names_;

  std::unordered_map<
    std::string,
    std::size_t>
    joint_index_;

  std::array<
    lite3_kinematics::Leg,
    4>
    legs_;

  std::array<int, 4>
    swing_order_;

  Eigen::VectorXd
    q_actual_;

  Eigen::VectorXd
    qdot_actual_;

  Eigen::VectorXd
    q_seed_;

  Eigen::VectorXd
    q_command_;

  Eigen::Vector3d
    base_position_world_ =
      Eigen::Vector3d::Zero();

  Eigen::Quaterniond
    base_orientation_world_ =
      Eigen::Quaterniond::Identity();

  std::array<
    Eigen::Vector3d,
    4>
    foot_initial_world_;

  std::array<
    Eigen::Vector3d,
    4>
    foot_target_world_;

  std::array<
    Eigen::Vector3d,
    4>
    foot_landing_world_;

  double stair_height_{0.185};
  double tread_depth_{0.315};
  double stair_start_x_{0.0};

  double front_landing_x_{0.22};
  double hind_landing_x_{0.08};

  double clearance_{0.060};
  double foot_radius_{0.022};

  double swing_duration_{3.0};
  double settle_duration_{1.0};
  double prepare_duration_{2.0};

  double riser_clearance_x_{0.050};
  double landing_margin_{0.040};

  double ik_warning_period_s_{1.0};

  double ground_surface_z_{0.0};
  double top_surface_z_{0.185};
  double top_foot_center_z_{0.207};

  Phase phase_;

  rclcpp::Time
    phase_start_time_;

  bool experiment_started_;
  bool got_joint_state_;
  bool got_odom_;
  bool targets_initialized_;
  bool done_reported_{false};
};

int main(
  int argc,
  char **argv)
{
  rclcpp::init(
    argc,
    argv);

  auto node =
    std::make_shared<
      SingleStepStairController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}