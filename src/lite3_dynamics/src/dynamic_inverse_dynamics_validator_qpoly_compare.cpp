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

// Timing limits are evaluated in Gazebo simulation time.
constexpr double DT_MIN_SEC = 0.001;
constexpr double DT_MAX_SEC = 0.030;
constexpr double MAX_DT_RATIO = 4.0;

// Maximum wall-time extrapolation used only to map a JointState callback
// onto the latest /clock trajectory. All derivatives are computed in
// simulation-time units.
constexpr double CLOCK_EXTRAPOLATION_MAX_WALL_SEC = 0.050;

constexpr std::size_t PRINT_EVERY = 100;

struct JointSample
{
  double t = 0.0;
  Eigen::Matrix<double, 12, 1> q =
    Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> qdot =
    Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> effort =
    Eigen::Matrix<double, 12, 1>::Zero();
};

struct DerivativeEstimate
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

bool validateWindowTiming(
  const std::deque<JointSample> & window,
  double & center_dt,
  double & window_span,
  double & scale)
{
  if (window.size() != WINDOW_SIZE) {
    return false;
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
      return false;
    }

    min_dt = std::min(min_dt, dt[i - 1]);
    max_dt = std::max(max_dt, dt[i - 1]);
  }

  if (min_dt <= 0.0 || (max_dt / min_dt) > MAX_DT_RATIO) {
    return false;
  }

  constexpr std::size_t c = 2;
  const double tc = window[c].t;

  window_span = window.back().t - window.front().t;
  if (!std::isfinite(window_span) || window_span <= 0.0) {
    return false;
  }

  scale = std::max(
    std::abs(window.front().t - tc),
    std::abs(window.back().t - tc));

  if (!std::isfinite(scale) || scale <= 0.0) {
    return false;
  }

  // Keep the same center-dt definition as the previous validator so the
  // results remain directly comparable.
  center_dt = 0.5 * (dt[1] + dt[2]);
  return std::isfinite(center_dt) && center_dt > 0.0;
}

// Existing baseline: cubic LS fit of qdot(t), evaluated at the center.
// qdot(x) = a0 + a1*x + a2*x^2 + a3*x^3
// x = (t-tc)/scale
// qddot(tc) = a1/scale
DerivativeEstimate estimateQddotFromQdot(
  const std::deque<JointSample> & window)
{
  DerivativeEstimate result;
  double center_dt = 0.0;
  double window_span = 0.0;
  double scale = 0.0;

  if (!validateWindowTiming(window, center_dt, window_span, scale)) {
    return result;
  }

  constexpr std::size_t c = 2;
  const double tc = window[c].t;

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
  result.center_dt = center_dt;
  result.window_span = window_span;
  result.valid = result.qddot.allFinite();
  return result;
}

// New estimator: cubic LS fit directly to q(t), then differentiate the
// fitted polynomial analytically.
//
// q(x) = a0 + a1*x + a2*x^2 + a3*x^3
// x = (t-tc)/scale
// qdot(tc) = a1/scale
// qddot(tc) = 2*a2/scale^2
//
// Using q instead of qdot avoids differentiating an already differentiated
// signal. With 5 samples and a cubic model, the fit is overdetermined and
// therefore retains a small amount of smoothing.
DerivativeEstimate estimateQddotFromQ(
  const std::deque<JointSample> & window)
{
  DerivativeEstimate result;
  double center_dt = 0.0;
  double window_span = 0.0;
  double scale = 0.0;

  if (!validateWindowTiming(window, center_dt, window_span, scale)) {
    return result;
  }

  constexpr std::size_t c = 2;
  const double tc = window[c].t;

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
      window[i].q.transpose();
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

  result.qddot = 2.0 * coeff.row(2).transpose() / (scale * scale);
  result.center_dt = center_dt;
  result.window_span = window_span;
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

struct AccelBinStats
{
  RunningStats qpoly;
  RunningStats qdot;
  std::size_t windows = 0;
};

int accelerationBin(double max_abs_accel)
{
  if (max_abs_accel < 50.0) {
    return 0;
  }
  if (max_abs_accel < 150.0) {
    return 1;
  }
  return 2;
}

const char * accelerationBinName(int bin)
{
  switch (bin) {
    case 0: return "<50 rad/s^2";
    case 1: return "50-150 rad/s^2";
    case 2: return ">=150 rad/s^2";
    default: return "unknown";
  }
}

}  // namespace

class DynamicInverseDynamicsValidatorCompare : public rclcpp::Node
{
public:
  DynamicInverseDynamicsValidatorCompare()
  : Node("dynamic_inverse_dynamics_validator_qpoly_compare"),
    dynamics_()
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &DynamicInverseDynamicsValidatorCompare::jointCallback,
        this,
        std::placeholders::_1));

    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(50).best_effort(),
      std::bind(
        &DynamicInverseDynamicsValidatorCompare::clockCallback,
        this,
        std::placeholders::_1));

    csv_.open(
      "/home/lexion/lite3_ws/data/dynamic_inverse_dynamics_validation_qpoly_compare.csv",
      std::ios::out | std::ios::trunc);

    if (!csv_.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open qpoly comparison CSV");
    } else {
      writeCsvHeader();
    }

    RCLCPP_INFO(
      get_logger(),
      "Dynamic inverse-dynamics estimator comparison started.");
    RCLCPP_INFO(
      get_logger(),
      "Timebase: Gazebo /clock simulation time.");
    RCLCPP_INFO(
      get_logger(),
      "Baseline: cubic LS qdot(t). New: cubic LS q(t) -> qddot.");
    RCLCPP_INFO(
      get_logger(),
      "Both estimators are evaluated at the center of the same 5-sample window.");
  }

  ~DynamicInverseDynamicsValidatorCompare() override
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

    if (have_latest_clock_ && sim_t < latest_clock_.sim_t) {
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
    if (!have_latest_clock_ || !previous_clock_valid_) {
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

    const double sim_rate = sim_dt / wall_dt;
    if (!std::isfinite(sim_rate) || sim_rate < 0.0) {
      return false;
    }

    sim_t = latest_clock_.sim_t + wall_offset * sim_rate;
    if (!std::isfinite(sim_t)) {
      return false;
    }

    // Preserve monotonicity against the latest known simulation time.
    sim_t = std::max(sim_t, latest_clock_.sim_t);
    return true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    ++joint_message_count_;

    const double wall_t = steadyTimeSec();
    double sample_t = 0.0;

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

    if (!sample.q.allFinite() ||
        !sample.qdot.allFinite() ||
        !sample.effort.allFinite() ||
        !std::isfinite(sample.t))
    {
      ++invalid_sample_count_;
      return;
    }

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

    const DerivativeEstimate qpoly =
      estimateQddotFromQ(window_);
    const DerivativeEstimate qdotfit =
      estimateQddotFromQdot(window_);

    if (!qpoly.valid || !qdotfit.valid) {
      ++rejected_windows_;
      return;
    }

    ++valid_windows_;
    evaluateCenterSample(window_[2], qpoly, qdotfit);
  }

  void evaluateCenterSample(
    const JointSample & center,
    const DerivativeEstimate & qpoly,
    const DerivativeEstimate & qdotfit)
  {
    const std::array<Leg, 4> legs = {
      Leg::FL, Leg::FR, Leg::HL, Leg::HR
    };

    std::array<Eigen::Vector3d, 4> qddot_qpoly;
    std::array<Eigen::Vector3d, 4> qddot_qdot;
    std::array<Eigen::Vector3d, 4> tau_id_qpoly;
    std::array<Eigen::Vector3d, 4> tau_id_qdot;
    std::array<Eigen::Vector3d, 4> tau_gz_all;
    std::array<Eigen::Vector3d, 4> err_qpoly;
    std::array<Eigen::Vector3d, 4> err_qdot;

    for (std::size_t leg_i = 0; leg_i < legs.size(); ++leg_i) {
      const int offset = static_cast<int>(leg_i * 3);

      const Eigen::Vector3d q = center.q.segment<3>(offset);
      const Eigen::Vector3d qdot = center.qdot.segment<3>(offset);
      const Eigen::Vector3d qddot_q = qpoly.qddot.segment<3>(offset);
      const Eigen::Vector3d qddot_d = qdotfit.qddot.segment<3>(offset);
      const Eigen::Vector3d tau_gz = center.effort.segment<3>(offset);

      const Eigen::Vector3d tau_q = dynamics_.inverseDynamics(
        legs[leg_i], q, qdot, qddot_q);
      const Eigen::Vector3d tau_d = dynamics_.inverseDynamics(
        legs[leg_i], q, qdot, qddot_d);

      const Eigen::Vector3d err_q = tau_gz - tau_q;
      const Eigen::Vector3d err_d = tau_gz - tau_d;

      qddot_qpoly[leg_i] = qddot_q;
      qddot_qdot[leg_i] = qddot_d;
      tau_id_qpoly[leg_i] = tau_q;
      tau_id_qdot[leg_i] = tau_d;
      tau_gz_all[leg_i] = tau_gz;
      err_qpoly[leg_i] = err_q;
      err_qdot[leg_i] = err_d;

      stats_qpoly_[leg_i].add(err_q);
      stats_qdot_[leg_i].add(err_d);
    }

    // Acceleration regime is defined by the largest absolute FL joint
    // acceleration in this validation window.
    const double fl_max_abs_accel = qpoly.qddot.segment<3>(0).cwiseAbs().maxCoeff();
    const int bin = accelerationBin(fl_max_abs_accel);
    ++accel_bin_stats_[static_cast<std::size_t>(bin)].windows;
    accel_bin_stats_[static_cast<std::size_t>(bin)].qpoly.add(err_qpoly[0]);
    accel_bin_stats_[static_cast<std::size_t>(bin)].qdot.add(err_qdot[0]);

    writeCsvRow(
      center.t,
      qpoly.center_dt,
      qpoly.window_span,
      qddot_qpoly,
      qddot_qdot,
      tau_id_qpoly,
      tau_id_qdot,
      tau_gz_all,
      err_qpoly,
      err_qdot);

    if (valid_windows_ % PRINT_EVERY == 0) {
      printProgress(center, qpoly.center_dt, fl_max_abs_accel);
    }
  }

  void writeCsvHeader()
  {
    csv_ << "time_sim,dt_center_sim,window_span_sim";

    const std::array<std::string, 4> legs = {
      "FL", "FR", "HL", "HR"
    };

    for (const auto & leg : legs) {
      for (const auto & joint : {"HipX", "HipY", "Knee"}) {
        csv_
          << "," << leg << "_qddot_qpoly_" << joint
          << "," << leg << "_qddot_qdot_" << joint
          << "," << leg << "_tauID_qpoly_" << joint
          << "," << leg << "_tauID_qdot_" << joint
          << "," << leg << "_tauGz_" << joint
          << "," << leg << "_err_qpoly_" << joint
          << "," << leg << "_err_qdot_" << joint;
      }
    }

    csv_ << "\n";
    csv_.flush();
  }

  void writeCsvRow(
    double t,
    double dt_center,
    double window_span,
    const std::array<Eigen::Vector3d, 4> & qddot_qpoly,
    const std::array<Eigen::Vector3d, 4> & qddot_qdot,
    const std::array<Eigen::Vector3d, 4> & tau_id_qpoly,
    const std::array<Eigen::Vector3d, 4> & tau_id_qdot,
    const std::array<Eigen::Vector3d, 4> & tau_gz_all,
    const std::array<Eigen::Vector3d, 4> & err_qpoly,
    const std::array<Eigen::Vector3d, 4> & err_qdot)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_ << std::fixed << std::setprecision(9)
         << t << "," << dt_center << "," << window_span;

    for (std::size_t leg_i = 0; leg_i < 4; ++leg_i) {
      for (int j = 0; j < 3; ++j) {
        csv_
          << "," << qddot_qpoly[leg_i](j)
          << "," << qddot_qdot[leg_i](j)
          << "," << tau_id_qpoly[leg_i](j)
          << "," << tau_id_qdot[leg_i](j)
          << "," << tau_gz_all[leg_i](j)
          << "," << err_qpoly[leg_i](j)
          << "," << err_qdot[leg_i](j);
      }
    }

    csv_ << "\n";

    if (valid_windows_ % 50 == 0) {
      csv_.flush();
    }
  }

  void printProgress(
    const JointSample & center,
    double center_dt,
    double fl_max_abs_accel)
  {
    RCLCPP_INFO(
      get_logger(),
      "t_sim=%.3f s | dt=%.3f ms | FL|max(qddot|)=%.2f rad/s^2 | "
      "valid=%zu rejected=%zu | "
      "Qpoly FL MAE=%.6f RMSE=%.6f max=%.6f | "
      "Qdot FL MAE=%.6f RMSE=%.6f max=%.6f",
      center.t,
      center_dt * 1000.0,
      fl_max_abs_accel,
      valid_windows_,
      rejected_windows_,
      stats_qpoly_[0].mae(),
      stats_qpoly_[0].rmse(),
      stats_qpoly_[0].max_abs,
      stats_qdot_[0].mae(),
      stats_qdot_[0].rmse(),
      stats_qdot_[0].max_abs);
  }

  void printFinalSummary() const
  {
    if (joint_message_count_ == 0) {
      return;
    }

    std::cout
      << "\n============================================================\n"
      << "LITE3 DYNAMIC INVERSE-DYNAMICS ESTIMATOR COMPARISON\n"
      << "============================================================\n"
      << "Timebase            : Gazebo /clock simulation time\n"
      << "Baseline estimator  : cubic LS fit of qdot(t)\n"
      << "New estimator       : cubic LS fit of q(t)\n"
      << "Joint messages      : " << joint_message_count_ << "\n"
      << "Accepted samples    : " << accepted_joint_count_ << "\n"
      << "Valid 5-point wins  : " << valid_windows_ << "\n"
      << "Rejected windows    : " << rejected_windows_ << "\n"
      << "Clock samples       : " << clock_count_ << "\n"
      << "No sim-time samples : " << no_sim_time_count_ << "\n"
      << "Incomplete joints   : " << incomplete_joint_count_ << "\n"
      << "Invalid samples     : " << invalid_sample_count_ << "\n"
      << "Nonmonotonic joint  : " << nonmonotonic_joint_count_ << "\n"
      << "Backward /clock     : " << backward_clock_count_ << "\n\n";

    const std::array<std::string, 4> legs = {"FL", "FR", "HL", "HR"};

    for (std::size_t i = 0; i < legs.size(); ++i) {
      std::cout
        << legs[i]
        << " | Qpoly MAE=" << std::fixed << std::setprecision(6)
        << stats_qpoly_[i].mae()
        << " RMSE=" << stats_qpoly_[i].rmse()
        << " max=" << stats_qpoly_[i].max_abs
        << " Nm | Qdot MAE=" << stats_qdot_[i].mae()
        << " RMSE=" << stats_qdot_[i].rmse()
        << " max=" << stats_qdot_[i].max_abs
        << " Nm\n";
    }

    std::cout << "\nFL acceleration regimes (based on Qpoly |qddot| max):\n";
    for (std::size_t i = 0; i < accel_bin_stats_.size(); ++i) {
      const auto & s = accel_bin_stats_[i];
      std::cout
        << "  " << accelerationBinName(static_cast<int>(i))
        << " | windows=" << s.windows
        << " | Qpoly MAE=" << s.qpoly.mae()
        << " RMSE=" << s.qpoly.rmse()
        << " max=" << s.qpoly.max_abs
        << " | Qdot MAE=" << s.qdot.mae()
        << " RMSE=" << s.qdot.rmse()
        << " max=" << s.qdot.max_abs
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

  std::array<RunningStats, 4> stats_qpoly_{};
  std::array<RunningStats, 4> stats_qdot_{};
  std::array<AccelBinStats, 3> accel_bin_stats_{};

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
  std::size_t invalid_sample_count_ = 0;
  std::size_t nonmonotonic_joint_count_ = 0;
  std::size_t backward_clock_count_ = 0;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<DynamicInverseDynamicsValidatorCompare>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
