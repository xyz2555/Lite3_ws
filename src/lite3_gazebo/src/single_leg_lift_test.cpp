#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lite3_kinematics/kinematics.hpp"

using namespace std::chrono_literals;


class SingleLegLiftTest : public rclcpp::Node
{
public:

  enum class Phase
  {
    WAITING,
    PREPARE,
    LIFT,
    HOLD,
    LOWER,
    DONE
  };


  SingleLegLiftTest()
  : Node("single_leg_lift_test"),
    phase_(Phase::WAITING)
  {

    // ==========================================================
    // Parameters
    // ==========================================================

    declare_parameter<bool>(
      "start_experiment",
      false);

    declare_parameter<double>(
      "lift_height",
      0.04);

    declare_parameter<double>(
      "lift_duration",
      2.5);

    declare_parameter<double>(
      "hold_duration",
      1.0);

    declare_parameter<double>(
      "prepare_duration",
      2.0);

    declare_parameter<double>(
      "position_command_rate_hz",
      100.0);


    lift_height_ =
      get_parameter(
        "lift_height").as_double();

    lift_duration_ =
      get_parameter(
        "lift_duration").as_double();

    hold_duration_ =
      get_parameter(
        "hold_duration").as_double();

    prepare_duration_ =
      get_parameter(
        "prepare_duration").as_double();


    double rate =
      get_parameter(
        "position_command_rate_hz").as_double();


    // ==========================================================
    // Joint order
    // ==========================================================

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


    for (std::size_t i = 0;
         i < joint_names_.size();
         ++i)
    {
      joint_index_[
        joint_names_[i]] = i;
    }


    legs_ = {

      lite3_kinematics::Leg::FL,
      lite3_kinematics::Leg::FR,
      lite3_kinematics::Leg::HL,
      lite3_kinematics::Leg::HR

    };


    q_actual_ =
      Eigen::VectorXd::Zero(12);

    q_seed_ =
      Eigen::VectorXd::Zero(12);

    q_command_ =
      Eigen::VectorXd::Zero(12);


    foot_world_.fill(
      Eigen::Vector3d::Zero());


    // ==========================================================
    // Publisher
    // ==========================================================

    command_pub_ =
      create_publisher<
        std_msgs::msg::Float64MultiArray>(
          "/lite3_position_controller/commands",
          10);


    // ==========================================================
    // Joint state
    // ==========================================================

    joint_sub_ =
      create_subscription<
        sensor_msgs::msg::JointState>(
          "/joint_states",
          20,

          std::bind(
            &SingleLegLiftTest::jointStateCallback,
            this,
            std::placeholders::_1));


    // ==========================================================
    // Odometry
    // ==========================================================

    odom_sub_ =
      create_subscription<
        nav_msgs::msg::Odometry>(
          "/lite3/odometry",
          20,

          std::bind(
            &SingleLegLiftTest::odomCallback,
            this,
            std::placeholders::_1));


    // ==========================================================
    // Timer
    // ==========================================================

    timer_ =
      create_wall_timer(

        std::chrono::duration_cast<
          std::chrono::milliseconds>(
            std::chrono::duration<double>(
              1.0 / rate)),

        std::bind(
          &SingleLegLiftTest::controlLoop,
          this));



    RCLCPP_INFO(
      get_logger(),
      "==============================================");

    RCLCPP_INFO(
      get_logger(),
      "Lite3 Single Leg Lift Test");

    RCLCPP_INFO(
      get_logger(),
      "Lift height = %.1f cm",
      lift_height_ * 100.0);

    RCLCPP_INFO(
      get_logger(),
      "Lift duration = %.2f s",
      lift_duration_);

    RCLCPP_INFO(
      get_logger(),
      "Sequence: PREPARE -> LIFT -> HOLD -> LOWER");

    RCLCPP_INFO(
      get_logger(),
      "==============================================");
  }


private:


  // ============================================================
  // Joint state callback
  // ============================================================

  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {

    std::size_t matched = 0;


    for (std::size_t i = 0;
         i < msg->name.size();
         ++i)
    {

      auto it =
        joint_index_.find(
          msg->name[i]);


      if (it ==
          joint_index_.end())
        continue;


      if (i >=
          msg->position.size())
        continue;


      std::size_t idx =
        it->second;


      q_actual_(
        static_cast<int>(idx)) =
        msg->position[i];


      matched++;
    }


    got_joint_state_ =
      (matched == 12);
  }


  // ============================================================
  // Odometry callback
  // ============================================================

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    base_position_ =
      Eigen::Vector3d(

        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);


    base_orientation_ =
      Eigen::Quaterniond(

        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);


    if (base_orientation_.norm()
        < 1e-9)
    {
      got_odom_ = false;
      return;
    }


    base_orientation_.normalize();

    got_odom_ = true;
  }


  // ============================================================
  // Frame transformation
  // ============================================================

  Eigen::Vector3d torsoToWorld(
    const Eigen::Vector3d &p) const
  {

    return

      base_position_ +
      base_orientation_ * p;
  }


  Eigen::Vector3d worldToTorso(
    const Eigen::Vector3d &p) const
  {

    return

      base_orientation_.inverse() *
      (p - base_position_);
  }


  // ============================================================
  // Initialize foot positions
  // ============================================================

  bool initializeFeet()
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

      Eigen::Vector3d q_leg =
        q_actual_.segment<3>(
          3 * leg);


      Eigen::Vector3d p_torso =
        kinematics_.forward(
          legs_[leg],
          q_leg);


      foot_world_[leg] =
        torsoToWorld(
          p_torso);


      q_seed_.segment<3>(
        3 * leg) =
        q_leg;
    }


    RCLCPP_INFO(
      get_logger(),
      "Initial feet:");

    for (int leg = 0;
         leg < 4;
         ++leg)
    {

      RCLCPP_INFO(
        get_logger(),

        " %s : "
        "[%.3f %.3f %.3f]",

        legName(leg).c_str(),

        foot_world_[leg].x(),
        foot_world_[leg].y(),
        foot_world_[leg].z());
    }


    return true;
  }


  // ============================================================
  // Quintic interpolation
  // ============================================================

  static double quintic(
    double u)
  {

    u =
      std::clamp(
        u,
        0.0,
        1.0);


    return

      10.0 * std::pow(u, 3)
      -
      15.0 * std::pow(u, 4)
      +
       6.0 * std::pow(u, 5);
  }


  // ============================================================
  // IK
  // ============================================================

  bool computeIK(
    const std::array<
      Eigen::Vector3d,
      4> &targets)
  {

    q_command_ =
      q_actual_;


    for (int leg = 0;
         leg < 4;
         ++leg)
    {

      Eigen::Vector3d
        target_torso =
          worldToTorso(
            targets[leg]);


      Eigen::Vector3d q_seed =
        q_seed_.segment<3>(
          3 * leg);


      auto result =
        kinematics_.inverse(

          legs_[leg],
          target_torso,
          &q_seed);


      if (!result.success)
      {

        RCLCPP_WARN_THROTTLE(

          get_logger(),
          *get_clock(),
          2000,

          "IK failed for %s",

          legName(leg).c_str());

        return false;
      }


      q_command_.segment<3>(
        3 * leg) =
        result.q;


      q_seed_.segment<3>(
        3 * leg) =
        result.q;
    }


    return true;
  }


  // ============================================================
  // Publish command
  // ============================================================

  void publish()
  {

    std_msgs::msg::Float64MultiArray msg;

    msg.data.resize(12);


    for (int i = 0;
         i < 12;
         ++i)
    {

      msg.data[i] =
        q_command_(i);
    }


    command_pub_->publish(msg);
  }


  // ============================================================
  // Change state
  // ============================================================

  void setPhase(
    Phase p)
  {

    phase_ = p;

    phase_start_ =
      now();


    RCLCPP_INFO(
      get_logger(),
      "State -> %s",
      phaseName(p).c_str());
  }


  double phaseTime() const
  {

    return

      (now() -
       phase_start_)
      .seconds();
  }


  // ============================================================
  // Main loop
  // ============================================================

  void controlLoop()
  {

    if (!got_joint_state_ ||
        !got_odom_)
      return;


    bool start =
      get_parameter(
        "start_experiment")
      .as_bool();


    // ----------------------------------------------------------
    // Start
    // ----------------------------------------------------------

    if (!experiment_started_ &&
        start)
    {

      if (!initializeFeet())
        return;


      experiment_started_ = true;

      setPhase(
        Phase::PREPARE);
    }


    // ----------------------------------------------------------
    // Before start
    // ----------------------------------------------------------

    if (!experiment_started_)
    {

      computeIK(
        foot_world_);

      publish();

      return;
    }


    // ----------------------------------------------------------
    // Prepare
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::PREPARE)
    {

      computeIK(
        foot_world_);

      publish();


      if (phaseTime() >=
          prepare_duration_)
      {

        setPhase(
          Phase::LIFT);
      }


      return;
    }


    // ----------------------------------------------------------
    // FL lift
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::LIFT)
    {

      double u =
        std::clamp(

          phaseTime() /
          lift_duration_,

          0.0,
          1.0);


      double s =
        quintic(u);


      auto target =
        foot_world_;


      // IMPORTANT:
      //
      // FL moves ONLY in Z.
      //
      // X and Y stay exactly constant.

      target[0].z() =
        foot_world_[0].z()
        +
        lift_height_ * s;


      computeIK(target);

      publish();


      if (u >= 1.0)
      {

        setPhase(
          Phase::HOLD);
      }


      return;
    }


    // ----------------------------------------------------------
    // Hold FL in the air
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::HOLD)
    {

      auto target =
        foot_world_;


      target[0].z() =
        foot_world_[0].z()
        +
        lift_height_;


      computeIK(target);

      publish();


      if (phaseTime() >=
          hold_duration_)
      {

        setPhase(
          Phase::LOWER);
      }


      return;
    }


    // ----------------------------------------------------------
    // Lower FL
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::LOWER)
    {

      double u =
        std::clamp(

          phaseTime() /
          lift_duration_,

          0.0,
          1.0);


      double s =
        quintic(u);


      auto target =
        foot_world_;


      target[0].z() =

        foot_world_[0].z()
        +
        lift_height_
        *
        (1.0 - s);


      computeIK(target);

      publish();


      if (u >= 1.0)
      {

        setPhase(
          Phase::DONE);
      }


      return;
    }


    // ----------------------------------------------------------
    // Done
    // ----------------------------------------------------------

    if (phase_ ==
        Phase::DONE)
    {

      computeIK(
        foot_world_);

      publish();
    }
  }


  // ============================================================
  // Utility
  // ============================================================

  static std::string legName(
    int i)
  {

    switch (i)
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
    Phase p)
  {

    switch (p)
    {

      case Phase::WAITING:
        return "WAITING";

      case Phase::PREPARE:
        return "PREPARE";

      case Phase::LIFT:
        return "LIFT";

      case Phase::HOLD:
        return "HOLD";

      case Phase::LOWER:
        return "LOWER";

      case Phase::DONE:
        return "DONE";

      default:
        return "UNKNOWN";
    }
  }


  // ============================================================
  // Variables
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


  std::vector<
    std::string>
    joint_names_;


  std::unordered_map<
    std::string,
    std::size_t>
    joint_index_;


  std::array<
    lite3_kinematics::Leg,
    4>
    legs_;


  Eigen::VectorXd
    q_actual_;


  Eigen::VectorXd
    q_seed_;


  Eigen::VectorXd
    q_command_;


  std::array<
    Eigen::Vector3d,
    4>
    foot_world_;


  Eigen::Vector3d
    base_position_ =
      Eigen::Vector3d::Zero();


  Eigen::Quaterniond
    base_orientation_ =
      Eigen::Quaterniond::Identity();


  double lift_height_ = 0.04;

  double lift_duration_ = 2.5;

  double hold_duration_ = 1.0;

  double prepare_duration_ = 2.0;


  bool got_joint_state_ = false;

  bool got_odom_ = false;

  bool experiment_started_ = false;


  Phase phase_;

  rclcpp::Time
    phase_start_;
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
      SingleLegLiftTest>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}