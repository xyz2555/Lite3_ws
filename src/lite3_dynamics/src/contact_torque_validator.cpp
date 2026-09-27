#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <lite3_dynamics/dynamics.hpp>
#include <lite3_kinematics/kinematics.hpp>

using namespace std::chrono_literals;


class ContactTorqueValidator : public rclcpp::Node
{
public:

  ContactTorqueValidator()
  : Node("contact_torque_validator"),
    kinematics_(),
    dynamics_()
  {
    // ============================================================
    // Joint states
    // ============================================================

    joint_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        std::bind(
          &ContactTorqueValidator::jointCallback,
          this,
          std::placeholders::_1));


    // ============================================================
    // Floating-base pose
    //
    // This is the simulation-time reference for validation.
    // ============================================================

    base_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lite3/base_pose",
        rclcpp::QoS(20).reliable(),
        std::bind(
          &ContactTorqueValidator::baseCallback,
          this,
          std::placeholders::_1));


    // ============================================================
    // Contact sensors
    // ============================================================

    contact_subs_["FL"] =
      create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/world/flat/model/lite3/link/FL_SHANK/"
        "sensor/FL_contact/contact",
        rclcpp::QoS(10).reliable(),
        std::bind(
          &ContactTorqueValidator::contactFLCallback,
          this,
          std::placeholders::_1));


    contact_subs_["FR"] =
      create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/world/flat/model/lite3/link/FR_SHANK/"
        "sensor/FR_contact/contact",
        rclcpp::QoS(10).reliable(),
        std::bind(
          &ContactTorqueValidator::contactFRCallback,
          this,
          std::placeholders::_1));


    contact_subs_["HL"] =
      create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/world/flat/model/lite3/link/HL_SHANK/"
        "sensor/HL_contact/contact",
        rclcpp::QoS(10).reliable(),
        std::bind(
          &ContactTorqueValidator::contactHLCallback,
          this,
          std::placeholders::_1));


    contact_subs_["HR"] =
      create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/world/flat/model/lite3/link/HR_SHANK/"
        "sensor/HR_contact/contact",
        rclcpp::QoS(10).reliable(),
        std::bind(
          &ContactTorqueValidator::contactHRCallback,
          this,
          std::placeholders::_1));


    // ============================================================
    // CSV
    // ============================================================

    csv_.open(
      "/home/lexion/lite3_ws/data/contact_torque_validation.csv",
      std::ios::out | std::ios::trunc);


    if (csv_.is_open()) {
      writeCsvHeader();
    }
    else {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open contact_torque_validation.csv");
    }


    // ============================================================
    // Diagnostic timer
    // ============================================================

    diagnostic_timer_ =
      create_wall_timer(
        1s,
        std::bind(
          &ContactTorqueValidator::diagnosticCallback,
          this));


    // ============================================================
    // Startup information
    // ============================================================

    RCLCPP_INFO(
      get_logger(),
      "Contact torque validator started.");

    RCLCPP_INFO(
      get_logger(),
      "Synchronization mode: base-pose triggered.");

    RCLCPP_INFO(
      get_logger(),
      "JointState is treated as latest-state because "
      "JointState.header.stamp is zero.");

    RCLCPP_INFO(
      get_logger(),
      "Run with use_sim_time:=true");
  }


  ~ContactTorqueValidator()
  {
    if (csv_.is_open()) {
      csv_.close();
    }
  }


private:

  // ============================================================
  // Data structures
  // ============================================================

  struct TimedContact
  {
    double t = 0.0;

    Eigen::Vector3d force_world =
      Eigen::Vector3d::Zero();

    bool contact = false;
  };


  struct JointSnapshot
  {
    std::array<double, 12> q{};
    std::array<double, 12> qdot{};
    std::array<double, 12> effort{};

    bool valid = false;
  };


  struct LegResult
  {
    bool has_contact = false;

    double sync_dt = -1.0;

    Eigen::Vector3d F_world =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d F_body =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d G =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d JTF =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d tau_pred =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d tau_gz =
      Eigen::Vector3d::Zero();

    Eigen::Vector3d error =
      Eigen::Vector3d::Zero();
  };


  // ============================================================
  // Constants
  // ============================================================

  static constexpr double CONTACT_SYNC_TOL = 0.005;  // 5 ms
  static constexpr double OUTPUT_PERIOD = 0.200;     // 5 Hz


  // ============================================================
  // Diagnostics
  // ============================================================

  uint64_t joint_count_ = 0;

  uint64_t base_count_ = 0;

  uint64_t contact_count_ = 0;

  uint64_t no_joint_count_ = 0;

  uint64_t snapshot_count_ = 0;


  // ============================================================
  // Latest joint state
  //
  // JointState has no useful simulation timestamp in the
  // current configuration, therefore we store only its latest
  // state and use base pose time as the validation reference.
  // ============================================================


  // ============================================================
  // Leg utilities
  // ============================================================

  lite3_kinematics::Leg legFromName(
    const std::string & leg) const
  {
    if (leg == "FL") {
      return lite3_kinematics::Leg::FL;
    }

    if (leg == "FR") {
      return lite3_kinematics::Leg::FR;
    }

    if (leg == "HL") {
      return lite3_kinematics::Leg::HL;
    }

    return lite3_kinematics::Leg::HR;
  }


  int legOffset(
    const std::string & leg) const
  {
    if (leg == "FL") {
      return 0;
    }

    if (leg == "FR") {
      return 3;
    }

    if (leg == "HL") {
      return 6;
    }

    return 9;
  }


  // ============================================================
  // Timestamp utility
  // ============================================================

  double stampToSec(
    const builtin_interfaces::msg::Time & stamp) const
  {
    return
      static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
  }


  // ============================================================
  // Base pose callback
  //
  // IMPORTANT:
  // Base pose is now the validation trigger.
  // ============================================================

  void baseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const double base_time =
      stampToSec(msg->header.stamp);


    JointSnapshot joint_snapshot;


    {
      std::lock_guard<std::mutex> lock(mutex_);


      base_count_++;


      if (!have_joint_) {
        no_joint_count_++;
        return;
      }


      joint_snapshot =
        latest_joint_;
    }


    /*
     * The base-pose timestamp is a Gazebo simulation timestamp.
     *
     * JointState has header.stamp == 0 in the current setup,
     * therefore we do NOT attempt to synchronize using
     * JointState.header.stamp.
     *
     * Instead:
     *
     *     t_validation = t_base
     *
     * and the latest joint state is used.
     */

    snapshot_count_++;


    processBaseTriggeredSnapshot(
      joint_snapshot,
      *msg,
      base_time);
  }


  // ============================================================
  // Base-triggered snapshot
  // ============================================================

  void processBaseTriggeredSnapshot(
    const JointSnapshot & js,
    const geometry_msgs::msg::PoseStamped & base_msg,
    double base_time)
  {
    // ----------------------------------------------------------
    // Base orientation
    // ----------------------------------------------------------

    const Eigen::Matrix3d R_WB =
      rotationMatrix(
        base_msg.pose.orientation);


    /*
     * World -> body/TORSO rotation.
     */
    const Eigen::Matrix3d R_BW =
      R_WB.transpose();


    // ----------------------------------------------------------
    // Leg results
    // ----------------------------------------------------------

    std::map<
      std::string,
      LegResult
    > result;


    const std::array<std::string, 4> legs =
      {
        "FL",
        "FR",
        "HL",
        "HR"
      };


    double total_squared_error = 0.0;

    double total_abs_error = 0.0;

    int total_error_components = 0;

    double max_error = 0.0;


    // ----------------------------------------------------------
    // Process every leg
    // ----------------------------------------------------------

    for (const auto & leg_name : legs)
    {
      const int offset =
        legOffset(leg_name);


      const auto leg =
        legFromName(leg_name);


      // --------------------------------------------------------
      // Joint state
      // --------------------------------------------------------

      Eigen::Vector3d q;

      q <<
        js.q[offset + 0],
        js.q[offset + 1],
        js.q[offset + 2];


      Eigen::Vector3d qdot;

      qdot <<
        js.qdot[offset + 0],
        js.qdot[offset + 1],
        js.qdot[offset + 2];


      Eigen::Vector3d tau_gz;

      tau_gz <<
        js.effort[offset + 0],
        js.effort[offset + 1],
        js.effort[offset + 2];


      (void)qdot;


      LegResult r;


      r.tau_gz =
        tau_gz;


      // --------------------------------------------------------
      // Gravity torque
      // --------------------------------------------------------

      r.G =
        dynamics_.gravityTorque(
          leg,
          q);


      // --------------------------------------------------------
      // Contact
      // --------------------------------------------------------

      TimedContact contact;

      double contact_dt = -1.0;


      const bool found =
        nearestContact(
          leg_name,
          base_time,
          contact,
          contact_dt);


      if (found &&
          contact_dt <= CONTACT_SYNC_TOL &&
          contact.contact)
      {
        r.has_contact = true;

        r.sync_dt =
          contact_dt;


        r.F_world =
          contact.force_world;


        /*
         * Gazebo contact force is world-frame.
         *
         * Lite3 dynamics Jacobian is expressed in
         * the body / TORSO frame.
         *
         * Therefore:
         *
         *     F_body = R_BW * F_world
         */

        r.F_body =
          R_BW *
          r.F_world;
      }
      else
      {
        r.has_contact = false;

        r.sync_dt =
          found
          ? contact_dt
          : -1.0;

        r.F_world.setZero();

        r.F_body.setZero();
      }


      // --------------------------------------------------------
      // Contact torque
      //
      // tau_contact = J^T F
      // --------------------------------------------------------

      r.JTF =
        dynamics_.contactTorque(
          leg,
          q,
          r.F_body);


      // --------------------------------------------------------
      // Static equilibrium
      //
      // Established sign convention:
      //
      //     tau_pred = G - J^T F
      //
      // --------------------------------------------------------

      r.tau_pred =
        r.G -
        r.JTF;


      // --------------------------------------------------------
      // Residual
      // --------------------------------------------------------

      r.error =
        r.tau_gz -
        r.tau_pred;


      total_squared_error +=
        r.error.squaredNorm();


      total_abs_error +=
        r.error.cwiseAbs().sum();


      total_error_components += 3;


      max_error =
        std::max(
          max_error,
          r.error.cwiseAbs().maxCoeff());


      result[leg_name] =
        r;
    }


    // ==========================================================
    // CSV
    //
    // Every valid base-pose snapshot is logged.
    // ==========================================================

    const bool do_print =
      (last_print_time_ < 0.0) ||
      ((base_time - last_print_time_) >= OUTPUT_PERIOD);


    if (do_print)
    {
      last_print_time_ =
        base_time;
    }


    writeCsv(
      base_time,
      result,
      do_print);


    if (!do_print) {
      return;
    }


    // ==========================================================
    // Statistics
    // ==========================================================

    const double mae =
      total_abs_error /
      static_cast<double>(
        total_error_components);


    const double rmse =
      std::sqrt(
        total_squared_error /
        static_cast<double>(
          total_error_components));


    // ==========================================================
    // Console output
    // ==========================================================

    std::cout << "\n";


    std::cout
      << "============================================================\n";


    std::cout
      << "LITE3 CONTACT TORQUE VALIDATOR\n";


    std::cout
      << "============================================================\n";


    std::cout
      << "SIM TIME : "
      << std::fixed
      << std::setprecision(6)
      << base_time
      << " s\n";


    std::cout
      << "BASE XYZ : [ "
      << std::setprecision(6)
      << base_msg.pose.position.x
      << " "
      << base_msg.pose.position.y
      << " "
      << base_msg.pose.position.z
      << " ]\n";


    // ----------------------------------------------------------
    // Per-leg result
    // ----------------------------------------------------------

    for (const auto & leg_name : legs)
    {
      const auto & r =
        result.at(leg_name);


      std::cout
        << "\n------------------------------------------------------------\n"
        << leg_name
        << "   "
        << (
          r.has_contact
          ? "CONTACT"
          : "NO CONTACT"
        )
        << "\n"
        << "------------------------------------------------------------\n";


      if (r.sync_dt >= 0.0)
      {
        std::cout
          << "Contact dt : "
          << std::setprecision(3)
          << r.sync_dt * 1000.0
          << " ms\n";
      }
      else
      {
        std::cout
          << "Contact dt : N/A\n";
      }


      printVector(
        "F_world",
        r.F_world);


      printVector(
        "F_body",
        r.F_body);


      printVector(
        "G",
        r.G);


      printVector(
        "J^T F",
        r.JTF);


      printVector(
        "tau_pred",
        r.tau_pred);


      printVector(
        "tau_gz",
        r.tau_gz);


      printVector(
        "error",
        r.error);
    }


    // ==========================================================
    // Summary
    // ==========================================================

    std::cout
      << "\n============================================================\n"
      << "SUMMARY\n"
      << "============================================================\n";


    std::cout
      << "MAE  : "
      << std::setprecision(6)
      << mae
      << " Nm\n";


    std::cout
      << "RMSE : "
      << rmse
      << " Nm\n";


    std::cout
      << "MAX  : "
      << max_error
      << " Nm\n";


    std::cout
      << "============================================================\n";
  }


  // ============================================================
  // Contact callback: FL
  // ============================================================

  void contactFLCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    processContact("FL", msg);
  }


  // ============================================================
  // Contact callback: FR
  // ============================================================

  void contactFRCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    processContact("FR", msg);
  }


  // ============================================================
  // Contact callback: HL
  // ============================================================

  void contactHLCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    processContact("HL", msg);
  }


  // ============================================================
  // Contact callback: HR
  // ============================================================

  void contactHRCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    processContact("HR", msg);
  }


  // ============================================================
  // Process contact message
  // ============================================================

  void processContact(
    const std::string & leg,
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    const double t =
      stampToSec(msg->header.stamp);


    Eigen::Vector3d total_force =
      Eigen::Vector3d::Zero();


    bool robot_contact = false;


    for (const auto & contact : msg->contacts)
    {
      const bool robot_is_collision1 =
        contact.collision1.name.find("lite3::") !=
        std::string::npos;


      const bool robot_is_collision2 =
        contact.collision2.name.find("lite3::") !=
        std::string::npos;


      if (!robot_is_collision1 &&
          !robot_is_collision2)
      {
        continue;
      }


      robot_contact = true;


      for (const auto & wrench : contact.wrenches)
      {
        if (robot_is_collision1)
        {
          total_force.x() +=
            wrench.body_1_wrench.force.x;

          total_force.y() +=
            wrench.body_1_wrench.force.y;

          total_force.z() +=
            wrench.body_1_wrench.force.z;
        }
        else
        {
          total_force.x() +=
            wrench.body_2_wrench.force.x;

          total_force.y() +=
            wrench.body_2_wrench.force.y;

          total_force.z() +=
            wrench.body_2_wrench.force.z;
        }
      }
    }


    std::lock_guard<std::mutex> lock(mutex_);


    contact_buffer_[leg].push_back(
      TimedContact{
        t,
        total_force,
        robot_contact
      });


    while (contact_buffer_[leg].size() > 200)
    {
      contact_buffer_[leg].pop_front();
    }


    contact_count_++;
  }


  // ============================================================
  // Joint callback
  //
  // IMPORTANT:
  // This callback ONLY stores the latest valid joint state.
  // It does NOT trigger validation.
  // ============================================================

  void jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    JointSnapshot snapshot;


    int joint_received = 0;


    for (size_t i = 0;
         i < msg->name.size();
         ++i)
    {
      const int index =
        jointIndex(
          msg->name[i]);


      if (index < 0 ||
          index >= 12)
      {
        continue;
      }


      if (i < msg->position.size())
      {
        snapshot.q[index] =
          msg->position[i];
      }


      if (i < msg->velocity.size())
      {
        snapshot.qdot[index] =
          msg->velocity[i];
      }


      if (i < msg->effort.size())
      {
        snapshot.effort[index] =
          msg->effort[i];
      }


      joint_received++;
    }


    snapshot.valid =
      (joint_received == 12);


    if (snapshot.valid)
    {
      std::lock_guard<std::mutex> lock(mutex_);

      latest_joint_ =
        snapshot;

      have_joint_ =
        true;
    }


    joint_count_++;
  }


  // ============================================================
  // Joint-name mapping
  // ============================================================

  int jointIndex(
    const std::string & name) const
  {
    static const std::map<std::string, int> indices =
    {
      {"FL_HipX_joint", 0},
      {"FL_HipY_joint", 1},
      {"FL_Knee_joint", 2},

      {"FR_HipX_joint", 3},
      {"FR_HipY_joint", 4},
      {"FR_Knee_joint", 5},

      {"HL_HipX_joint", 6},
      {"HL_HipY_joint", 7},
      {"HL_Knee_joint", 8},

      {"HR_HipX_joint", 9},
      {"HR_HipY_joint", 10},
      {"HR_Knee_joint", 11}
    };


    const auto it =
      indices.find(name);


    if (it == indices.end())
    {
      return -1;
    }


    return it->second;
  }


  // ============================================================
  // Nearest contact sample
  // ============================================================

  bool nearestContact(
    const std::string & leg,
    double target_time,
    TimedContact & output,
    double & dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);


    const auto & buffer =
      contact_buffer_[leg];


    if (buffer.empty())
    {
      return false;
    }


    double best =
      std::numeric_limits<double>::max();


    bool found = false;


    for (const auto & sample : buffer)
    {
      const double error =
        std::abs(
          sample.t - target_time);


      if (error < best)
      {
        best = error;
        output = sample;
        found = true;
      }
    }


    dt = best;


    return found;
  }


  // ============================================================
  // Quaternion -> rotation matrix
  //
  // R_WB transforms vectors:
  //
  //     body -> world
  //
  // Therefore:
  //
  //     R_BW = R_WB^T
  //
  // transforms:
  //
  //     world -> body
  // ============================================================

  Eigen::Matrix3d rotationMatrix(
    const geometry_msgs::msg::Quaternion & q)
  {
    Eigen::Quaterniond quat(
      q.w,
      q.x,
      q.y,
      q.z);


    quat.normalize();


    return quat.toRotationMatrix();
  }


  // ============================================================
  // Diagnostic callback
  // ============================================================

  void diagnosticCallback()
  {
    RCLCPP_INFO(
      get_logger(),

      "joint=%llu base=%llu contact=%llu "
      "no_joint=%llu valid_snapshot=%llu",

      static_cast<unsigned long long>(
        joint_count_),

      static_cast<unsigned long long>(
        base_count_),

      static_cast<unsigned long long>(
        contact_count_),

      static_cast<unsigned long long>(
        no_joint_count_),

      static_cast<unsigned long long>(
        snapshot_count_));
  }


  // ============================================================
  // Vector printing
  // ============================================================

  void printVector(
    const std::string & label,
    const Eigen::Vector3d & v)
  {
    std::cout
      << std::left
      << std::setw(10)
      << label
      << "[ "
      << std::fixed
      << std::setprecision(5)
      << std::setw(10)
      << v.x()
      << " "
      << std::setw(10)
      << v.y()
      << " "
      << std::setw(10)
      << v.z()
      << " ]\n";
  }


  // ============================================================
  // CSV header
  // ============================================================

  void writeCsvHeader()
  {
    csv_
      << "time";


    const std::array<std::string, 4> legs =
      {
        "FL",
        "FR",
        "HL",
        "HR"
      };


    for (const auto & leg : legs)
    {
      csv_
        << ","
        << leg << "_Fx"
        << ","
        << leg << "_Fy"
        << ","
        << leg << "_Fz";


      for (const auto & joint :
           {"HipX", "HipY", "Knee"})
      {
        csv_
          << ","
          << leg
          << "_"
          << joint
          << "_tau_gz"

          << ","
          << leg
          << "_"
          << joint
          << "_tau_pred"

          << ","
          << leg
          << "_"
          << joint
          << "_error";
      }
    }


    csv_
      << "\n";


    csv_.flush();
  }


  // ============================================================
  // CSV write
  // ============================================================

  template<typename ResultMap>
  void writeCsv(
    double time,
    const ResultMap & result,
    bool flush)
  {
    if (!csv_.is_open())
    {
      return;
    }


    csv_
      << std::fixed
      << std::setprecision(9)
      << time;


    const std::array<std::string, 4> legs =
      {
        "FL",
        "FR",
        "HL",
        "HR"
      };


    for (const auto & leg : legs)
    {
      const auto & r =
        result.at(leg);


      csv_
        << ","
        << r.F_world.x()

        << ","
        << r.F_world.y()

        << ","
        << r.F_world.z();


      for (int j = 0; j < 3; ++j)
      {
        csv_
          << ","
          << r.tau_gz[j]

          << ","
          << r.tau_pred[j]

          << ","
          << r.error[j];
      }
    }


    csv_
      << "\n";


    if (flush)
    {
      csv_.flush();
    }
  }


  // ============================================================
  // Members
  // ============================================================

  lite3_kinematics::Lite3Kinematics
    kinematics_;


  lite3_dynamics::Lite3SingleLegDynamics
    dynamics_;


  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr joint_sub_;


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr base_sub_;


  std::map<
    std::string,
    rclcpp::Subscription<
      ros_gz_interfaces::msg::Contacts
    >::SharedPtr
  > contact_subs_;


  std::map<
    std::string,
    std::deque<TimedContact>
  > contact_buffer_;


  JointSnapshot latest_joint_;


  bool have_joint_ = false;


  std::mutex mutex_;


  std::ofstream csv_;


  rclcpp::TimerBase::SharedPtr
    diagnostic_timer_;


  double last_print_time_ = -1.0;
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);


  auto node =
    std::make_shared<
      ContactTorqueValidator
    >();


  rclcpp::spin(node);


  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }


  return 0;
}