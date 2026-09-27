#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <lite3_dynamics/dynamics.hpp>
#include <lite3_kinematics/kinematics.hpp>

using lite3_kinematics::Leg;
using lite3_dynamics::Lite3SingleLegDynamics;

namespace
{

constexpr std::size_t WINDOW_SIZE = 5;

// Timing limits are now expressed in SIMULATION TIME.
// The observed joint-state cadence is a few milliseconds to roughly 10 ms.
constexpr double DT_MIN_SEC = 0.001;
constexpr double DT_MAX_SEC = 0.030;
constexpr double MAX_DT_RATIO = 4.0;

// Maximum wall-time extrapolation used only to map a JointState callback
// onto the latest /clock trajectory. The derivative itself is computed
// exclusively in simulation-time units.
constexpr double CLOCK_EXTRAPOLATION_MAX_WALL_SEC = 0.050;

constexpr std::size_t PRINT_EVERY = 100;

struct JointSample
{
  double t = 0.0;  // simulation time [s]
  Eigen::Matrix<double, 12, 1> q =
    Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> qdot =
    Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> effort =
    Eigen::Matrix<double, 12, 1>::Zero();
};

struct WindowDerivative
{
  bool valid = false;
  Eigen::Matrix<double, 12, 1> qddot =
    Eigen::Matrix<double, 12, 1>::Zero();
  double center_dt = 0.0;
  double window_span = 0.0;
};

struct ClockSample
{
  double wall_t = 0.0;
  double sim_t = 0.0;
};

int jointIndex(const std::string & name)
{
  static const std::map<std::string, int> indices = {
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

  const auto it = indices.find(name);
  return (it == indices.end()) ? -1 : it->second;
}

// Estimate qddot at the center sample using a cubic least-squares fit of
// qdot against the ACTUAL, NONUNIFORM SIMULATION timestamps.
//
// Let x = (t - t_center) / scale and fit
//   qdot(x) = a0 + a1*x + a2*x^2 + a3*x^3
// Then
//   qddot(t_center) = a1 / scale.
WindowDerivative estimateQddotNonuniform(
  const std::deque<JointSample> & window)
{
  WindowDerivative result;

  if (window.size() != WINDOW_SIZE) {
    return result;
  }

  std::array<double, WINDOW_SIZE - 1> dt{};
  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = 0.0;

  for (std::size_t i = 1; i < WINDOW_SIZE; ++i) {
    dt[i - 1] = window[i].t - window[i - 1].t;

    if (!std::isfinite(dt[i - 1]) ||
        dt[i - 1] < DT_MIN_SEC ||
        dt[i - 1] > DT_MAX_SEC)
    {
      return result;
    }

    min_dt = std::min(min_dt, dt[i - 1]);
    max_dt = std::max(max_dt, dt[i - 1]);
  }

  if ((max_dt / min_dt) > MAX_DT_RATIO) {
    return result;
  }

  constexpr std::size_t c = 2;
  const double tc = window[c].t;
  const double span = window.back().t - window.front().t;

  if (!std::isfinite(span) || span <= 0.0) {
    return result;
  }

  const double scale =
    std::max(
      std::abs(window.front().t - tc),
      std::abs(window.back().t - tc));

  if (!std::isfinite(scale) || scale <= 0.0) {
    return result;
  }

  Eigen::Matrix<double, WINDOW_SIZE, 4> X;
  Eigen::Matrix<double, WINDOW_SIZE, 12> Y;

  for (std::size_t i = 0; i < WINDOW_SIZE; ++i) {
    const double x = (window[i].t - tc) / scale;
    const double x2 = x * x;
    const double x3 = x2 * x;

    X(static_cast<Eigen::Index>(i), 0) = 1.0;
    X(static_cast<Eigen::Index>(i), 1) = x;
    X(static_cast<Eigen::Index>(i), 2) = x2;
    X(static_cast<Eigen::Index>(i), 3) = x3;

    Y.row(static_cast<Eigen::Index>(i)) =
      window[i].qdot.transpose();
  }

  Eigen::ColPivHouseholderQR<
    Eigen::Matrix<double, WINDOW_SIZE, 4>> qr(X);

  if (qr.rank() < 4) {
    return result;
  }

  const Eigen::Matrix<double, 4, 12> coeff = qr.solve(Y);

  if (!coeff.allFinite()) {
    return result;
  }

  result.qddot = coeff.row(1).transpose() / scale;
  result.center_dt = 0.5 * (dt[1] + dt[2]);
  result.window_span = span;
  result.valid = result.qddot.allFinite();

  return result;
}

struct RunningStats
{
  double abs_sum = 0.0;
  double sq_sum = 0.0;
  double max_abs = 0.0;
  std::size_t count = 0;

  void add(const Eigen::Vector3d & e)
  {
    abs_sum += e.cwiseAbs().sum();
    sq_sum += e.squaredNorm();
    max_abs = std::max(max_abs, e.cwiseAbs().maxCoeff());
    count += 3;
  }

  double mae() const
  {
    return count ? abs_sum / static_cast<double>(count) : 0.0;
  }

  double rmse() const
  {
    return count
      ? std::sqrt(sq_sum / static_cast<double>(count))
      : 0.0;
  }
};

}  // namespace

class DynamicInverseDynamicsValidator : public rclcpp::Node
{
public:
  DynamicInverseDynamicsValidator()
  : Node("dynamic_inverse_dynamics_validator"),
    dynamics_()
  {
    // ------------------------------------------------------------
    // Joint states
    // ------------------------------------------------------------
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &DynamicInverseDynamicsValidator::jointCallback,
        this,
        std::placeholders::_1));

    // ------------------------------------------------------------
    // Explicit Gazebo simulation clock
    // ------------------------------------------------------------
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(50).best_effort(),
      std::bind(
        &DynamicInverseDynamicsValidator::clockCallback,
        this,
        std::placeholders::_1));

    csv_.open(
      "/home/lexion/lite3_ws/data/dynamic_inverse_dynamics_validation.csv",
      std::ios::out | std::ios::trunc);

    if (!csv_.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open dynamic_inverse_dynamics_validation.csv");
    } else {
      writeCsvHeader();
    }

    RCLCPP_INFO(
      get_logger(),
      "Dynamic inverse-dynamics validator started.");
    RCLCPP_INFO(
      get_logger(),
      "Timebase: Gazebo /clock (simulation time).");
    RCLCPP_INFO(
      get_logger(),
      "Method: 5-sample cubic LS derivative on nonuniform simulation timestamps.");
    RCLCPP_INFO(
      get_logger(),
      "Validation is evaluated at the CENTER sample of the 5-sample window.");
  }

  ~DynamicInverseDynamicsValidator() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }

    printFinalSummary();
  }

private:
  static double steadyTimeSec()
  {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
  }

  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    const double wall_t = steadyTimeSec();
    const double sim_t = rclcpp::Time(msg->clock).seconds();

    if (!std::isfinite(sim_t) || sim_t < 0.0) {
      return;
    }

    // Ignore backwards simulation time. Equal timestamps are allowed because
    // Gazebo can publish the same clock value while paused.
    if (have_latest_clock_ &&
        sim_t < latest_clock_.sim_t)
    {
      ++backward_clock_count_;
      return;
    }

    previous_clock_ = latest_clock_;
    previous_clock_valid_ = have_latest_clock_;

    latest_clock_.wall_t = wall_t;
    latest_clock_.sim_t = sim_t;
    have_latest_clock_ = true;
    ++clock_count_;
  }

  bool estimateJointSimulationTime(
    double joint_wall_t,
    double & sim_t) const
  {
    if (!have_latest_clock_) {
      return false;
    }

    // With only one /clock sample available, we cannot estimate the local
    // real-time factor safely. Wait for the second sample.
    if (!previous_clock_valid_) {
      return false;
    }

    const double wall_dt =
      latest_clock_.wall_t - previous_clock_.wall_t;
    const double sim_dt =
      latest_clock_.sim_t - previous_clock_.sim_t;

    if (!std::isfinite(wall_dt) ||
        !std::isfinite(sim_dt) ||
        wall_dt <= 0.0 ||
        sim_dt < 0.0)
    {
      return false;
    }

    const double wall_offset =
      joint_wall_t - latest_clock_.wall_t;

    if (!std::isfinite(wall_offset) ||
        std::abs(wall_offset) > CLOCK_EXTRAPOLATION_MAX_WALL_SEC)
    {
      return false;
    }

    // Linear local mapping from wall time to simulation time.
    // This allows the JointState callback to be associated with a simulation
    // timestamp slightly after the most recently received /clock message.
    const double sim_rate = sim_dt / wall_dt;

    if (!std::isfinite(sim_rate) || sim_rate < 0.0) {
      return false;
    }

    sim_t = latest_clock_.sim_t + wall_offset * sim_rate;

    if (!std::isfinite(sim_t)) {
      return false;
    }

    // Do not let the extrapolation move farther backwards than the latest
    // known simulation time because that would break monotonicity easily
    // during scheduler jitter.
    sim_t = std::max(sim_t, latest_clock_.sim_t);

    return true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    ++joint_message_count_;

    const double wall_t = steadyTimeSec();
    double sample_t = 0.0;

    // Prefer a valid JointState timestamp if the publisher ever provides
    // one. In the current Gazebo/ros2_control setup it is zero, so normally
    // the explicit /clock mapping below is used.
    const bool header_valid =
      (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);

    if (header_valid) {
      sample_t = rclcpp::Time(msg->header.stamp).seconds();
    } else {
      if (!estimateJointSimulationTime(wall_t, sample_t)) {
        ++no_sim_time_count_;
        return;
      }
    }

    JointSample sample;
    sample.t = sample_t;

    int received = 0;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      const int idx = jointIndex(msg->name[i]);
      if (idx < 0) {
        continue;
      }

      if (i < msg->position.size()) {
        sample.q(idx) = msg->position[i];
      }

      if (i < msg->velocity.size()) {
        sample.qdot(idx) = msg->velocity[i];
      }

      if (i < msg->effort.size()) {
        sample.effort(idx) = msg->effort[i];
      }

      ++received;
    }

    if (received != 12) {
      ++incomplete_joint_count_;
      return;
    }

    if (!std::isfinite(sample.t)) {
      ++invalid_time_count_;
      return;
    }

    // The /clock mapping and callback ordering should normally produce
    // monotonically increasing simulation timestamps. Reject duplicates or
    // backward samples so the derivative window remains well-conditioned.
    if (!window_.empty() && sample.t <= window_.back().t) {
      ++nonmonotonic_joint_count_;
      return;
    }

    window_.push_back(sample);
    ++accepted_joint_count_;

    while (window_.size() > WINDOW_SIZE) {
      window_.pop_front();
    }

    if (window_.size() < WINDOW_SIZE) {
      return;
    }

    const WindowDerivative deriv =
      estimateQddotNonuniform(window_);

    if (!deriv.valid) {
      ++rejected_windows_;
      return;
    }

    ++valid_windows_;
    evaluateCenterSample(window_[2], deriv);
  }

  void evaluateCenterSample(
    const JointSample & center,
    const WindowDerivative & deriv)
  {
    const std::array<Leg, 4> legs = {
      Leg::FL, Leg::FR, Leg::HL, Leg::HR
    };

    std::array<Eigen::Vector3d, 4> qddot_all;
    std::array<Eigen::Vector3d, 4> tau_id_all;
    std::array<Eigen::Vector3d, 4> tau_gz_all;
    std::array<Eigen::Vector3d, 4> err_all;

    for (std::size_t leg_i = 0; leg_i < legs.size(); ++leg_i) {
      const int offset = static_cast<int>(leg_i * 3);

      const Eigen::Vector3d q = center.q.segment<3>(offset);
      const Eigen::Vector3d qdot = center.qdot.segment<3>(offset);
      const Eigen::Vector3d qddot = deriv.qddot.segment<3>(offset);
      const Eigen::Vector3d tau_gz = center.effort.segment<3>(offset);

      const Eigen::Vector3d tau_id =
        dynamics_.inverseDynamics(
          legs[leg_i],
          q,
          qdot,
          qddot);

      const Eigen::Vector3d error = tau_gz - tau_id;

      qddot_all[leg_i] = qddot;
      tau_id_all[leg_i] = tau_id;
      tau_gz_all[leg_i] = tau_gz;
      err_all[leg_i] = error;

      stats_[leg_i].add(error);
    }

    writeCsvRow(
      center.t,
      deriv.center_dt,
      deriv.window_span,
      qddot_all,
      tau_id_all,
      tau_gz_all,
      err_all);

    if (valid_windows_ % PRINT_EVERY == 0) {
      printProgress(center, deriv);
    }
  }

  void writeCsvHeader()
  {
    csv_
      << "time_sim,dt_center_sim,window_span_sim";

    const std::array<std::string, 4> legs = {
      "FL", "FR", "HL", "HR"
    };

    for (const auto & leg : legs) {
      for (const auto & joint : {"HipX", "HipY", "Knee"}) {
        csv_
          << "," << leg << "_qddot_" << joint
          << "," << leg << "_tauID_" << joint
          << "," << leg << "_tauGz_" << joint
          << "," << leg << "_err_" << joint;
      }
    }

    csv_ << "\n";
    csv_.flush();
  }

  void writeCsvRow(
    double t,
    double dt_center,
    double window_span,
    const std::array<Eigen::Vector3d, 4> & qddot_all,
    const std::array<Eigen::Vector3d, 4> & tau_id_all,
    const std::array<Eigen::Vector3d, 4> & tau_gz_all,
    const std::array<Eigen::Vector3d, 4> & err_all)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_
      << std::fixed
      << std::setprecision(9)
      << t
      << "," << dt_center
      << "," << window_span;

    for (std::size_t leg_i = 0; leg_i < 4; ++leg_i) {
      for (int j = 0; j < 3; ++j) {
        csv_
          << "," << qddot_all[leg_i](j)
          << "," << tau_id_all[leg_i](j)
          << "," << tau_gz_all[leg_i](j)
          << "," << err_all[leg_i](j);
      }
    }

    csv_ << "\n";

    if (valid_windows_ % 50 == 0) {
      csv_.flush();
    }
  }

  void printProgress(
    const JointSample & center,
    const WindowDerivative & deriv)
  {
    RCLCPP_INFO(
      get_logger(),
      "t_sim=%.3f s | dt=%.3f ms | span=%.3f ms | "
      "clock=%zu | joint_msg=%zu accepted=%zu | "
      "valid=%zu rejected=%zu | no_time=%zu | "
      "FL MAE=%.6f Nm RMSE=%.6f Nm max=%.6f Nm",
      center.t,
      deriv.center_dt * 1000.0,
      deriv.window_span * 1000.0,
      clock_count_,
      joint_message_count_,
      accepted_joint_count_,
      valid_windows_,
      rejected_windows_,
      no_sim_time_count_,
      stats_[0].mae(),
      stats_[0].rmse(),
      stats_[0].max_abs);
  }

  void printFinalSummary() const
  {
    if (joint_message_count_ == 0) {
      return;
    }

    std::cout
      << "\n============================================================\n";
    std::cout
      << "LITE3 DYNAMIC INVERSE-DYNAMICS VALIDATOR\n";
    std::cout
      << "============================================================\n";
    std::cout
      << "Timebase            : Gazebo /clock simulation time\n";
    std::cout
      << "Joint messages      : " << joint_message_count_ << "\n";
    std::cout
      << "Accepted samples    : " << accepted_joint_count_ << "\n";
    std::cout
      << "Valid 5-point wins  : " << valid_windows_ << "\n";
    std::cout
      << "Rejected windows    : " << rejected_windows_ << "\n";
    std::cout
      << "Clock samples       : " << clock_count_ << "\n";
    std::cout
      << "No sim-time samples : " << no_sim_time_count_ << "\n";
    std::cout
      << "Incomplete joints   : " << incomplete_joint_count_ << "\n";
    std::cout
      << "Invalid time        : " << invalid_time_count_ << "\n";
    std::cout
      << "Nonmonotonic joint  : " << nonmonotonic_joint_count_ << "\n";
    std::cout
      << "Backward /clock     : " << backward_clock_count_ << "\n\n";

    const std::array<std::string, 4> legs = {
      "FL", "FR", "HL", "HR"
    };

    for (std::size_t i = 0; i < legs.size(); ++i) {
      std::cout
        << legs[i]
        << " : MAE="
        << std::fixed << std::setprecision(6)
        << stats_[i].mae()
        << " Nm, RMSE="
        << stats_[i].rmse()
        << " Nm, max="
        << stats_[i].max_abs
        << " Nm\n";
    }

    std::cout
      << "============================================================\n";
  }

  Lite3SingleLegDynamics dynamics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

  std::deque<JointSample> window_;
  std::ofstream csv_;

  std::array<RunningStats, 4> stats_{};

  ClockSample previous_clock_{};
  ClockSample latest_clock_{};

  bool previous_clock_valid_ = false;
  bool have_latest_clock_ = false;

  std::size_t clock_count_ = 0;
  std::size_t joint_message_count_ = 0;
  std::size_t accepted_joint_count_ = 0;
  std::size_t valid_windows_ = 0;
  std::size_t rejected_windows_ = 0;

  std::size_t no_sim_time_count_ = 0;
  std::size_t incomplete_joint_count_ = 0;
  std::size_t invalid_time_count_ = 0;
  std::size_t nonmonotonic_joint_count_ = 0;
  std::size_t backward_clock_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<DynamicInverseDynamicsValidator>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
