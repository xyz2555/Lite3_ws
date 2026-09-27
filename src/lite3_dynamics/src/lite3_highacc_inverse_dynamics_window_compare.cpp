#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <lite3_dynamics/dynamics.hpp>
#include <lite3_kinematics/kinematics.hpp>

using lite3_kinematics::Leg;
using lite3_dynamics::Lite3SingleLegDynamics;

namespace
{
constexpr std::size_t NJ = 12;
constexpr double DT_MIN = 0.001;
constexpr double DT_MAX = 0.030;
constexpr double MAX_DT_RATIO = 4.0;
constexpr double ACTIVE_QDOT_THRESHOLD = 0.1;  // rad/s, based on FL Knee telemetry

const std::vector<std::size_t> WINDOWS = {5, 11, 21, 31, 41};

struct JointRow
{
  double wall_t{0.0};
  double sim_t{0.0};
  Eigen::Matrix<double, NJ, 1> q = Eigen::Matrix<double, NJ, 1>::Zero();
  Eigen::Matrix<double, NJ, 1> qdot = Eigen::Matrix<double, NJ, 1>::Zero();
  Eigen::Matrix<double, NJ, 1> effort = Eigen::Matrix<double, NJ, 1>::Zero();
};

struct ClockRow
{
  double wall_t{0.0};
  double sim_t{0.0};
};

struct Stats
{
  double abs_sum{0.0};
  double sq_sum{0.0};
  double max_abs{0.0};
  std::size_t count{0};

  void add(double e)
  {
    if (!std::isfinite(e)) return;
    const double a = std::abs(e);
    abs_sum += a;
    sq_sum += e * e;
    max_abs = std::max(max_abs, a);
    ++count;
  }

  double mae() const { return count ? abs_sum / static_cast<double>(count) : 0.0; }
  double rmse() const { return count ? std::sqrt(sq_sum / static_cast<double>(count)) : 0.0; }
};

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) out.push_back(item);
  return out;
}

bool parseDouble(const std::string & s, double & value)
{
  try {
    std::size_t pos = 0;
    value = std::stod(s, &pos);
    return pos == s.size() && std::isfinite(value);
  } catch (...) {
    return false;
  }
}

bool loadClockCsv(const std::string & path, std::vector<ClockRow> & rows)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Cannot open clock CSV: " << path << "\n";
    return false;
  }

  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    const auto p = splitCsv(line);
    if (p.size() < 3) continue;

    double wall = 0.0, sim = 0.0;
    if (!parseDouble(p[1], wall) || !parseDouble(p[2], sim)) continue;
    rows.push_back({wall, sim});
  }
  return !rows.empty();
}

bool loadJointCsv(const std::string & path, std::vector<JointRow> & rows)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Cannot open joint CSV: " << path << "\n";
    return false;
  }

  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    const auto p = splitCsv(line);
    if (p.size() < 3 + 3 * NJ) continue;

    JointRow row;
    if (!parseDouble(p[1], row.wall_t)) continue;

    bool ok = true;
    for (std::size_t j = 0; j < NJ; ++j) {
      const std::size_t base = 3 + 3 * j;
      if (!parseDouble(p[base + 0], row.q[j]) ||
          !parseDouble(p[base + 1], row.qdot[j]) ||
          !parseDouble(p[base + 2], row.effort[j])) {
        ok = false;
        break;
      }
    }
    if (ok) rows.push_back(row);
  }
  return !rows.empty();
}

bool reconstructSimulationTimes(std::vector<JointRow> & joints, const std::vector<ClockRow> & clocks)
{
  if (joints.empty() || clocks.size() < 2) return false;

  for (std::size_t i = 1; i < clocks.size(); ++i) {
    if (!(clocks[i].wall_t > clocks[i - 1].wall_t) ||
        !(clocks[i].sim_t > clocks[i - 1].sim_t)) {
      std::cerr << "Clock CSV is not strictly increasing.\n";
      return false;
    }
  }

  const double wall_min = clocks.front().wall_t;
  const double wall_max = clocks.back().wall_t;

  for (auto & row : joints) {
    if (row.wall_t < wall_min || row.wall_t > wall_max) {
      continue;
    }

    auto it = std::lower_bound(
      clocks.begin(), clocks.end(), row.wall_t,
      [](const ClockRow & c, double wall) { return c.wall_t < wall; });

    if (it == clocks.begin()) {
      row.sim_t = it->sim_t;
      continue;
    }
    if (it == clocks.end()) {
      row.sim_t = clocks.back().sim_t;
      continue;
    }

    const auto & c2 = *it;
    const auto & c1 = *(it - 1);
    const double wall_dt = c2.wall_t - c1.wall_t;
    if (wall_dt <= 0.0) return false;

    const double alpha = (row.wall_t - c1.wall_t) / wall_dt;
    row.sim_t = c1.sim_t + alpha * (c2.sim_t - c1.sim_t);
  }

  std::vector<JointRow> valid;
  valid.reserve(joints.size());
  for (const auto & j : joints) {
    if (std::isfinite(j.sim_t)) valid.push_back(j);
  }
  joints.swap(valid);
  return !joints.empty();
}

bool validateWindowTiming(const std::vector<JointRow> & w, double & center_dt, double & span)
{
  if (w.empty()) return false;

  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = 0.0;
  for (std::size_t i = 1; i < w.size(); ++i) {
    const double dt = w[i].sim_t - w[i - 1].sim_t;
    if (!(dt > 0.0)) return false;
    min_dt = std::min(min_dt, dt);
    max_dt = std::max(max_dt, dt);
  }

  if (min_dt < DT_MIN || max_dt > DT_MAX || max_dt / min_dt > MAX_DT_RATIO) {
    return false;
  }

  const std::size_t c = w.size() / 2;
  center_dt = 0.5 * ((w[c].sim_t - w[c - 1].sim_t) + (w[c + 1].sim_t - w[c].sim_t));
  span = w.back().sim_t - w.front().sim_t;
  return center_dt > 0.0 && span > 0.0 && std::isfinite(center_dt) && std::isfinite(span);
}

bool cubicFitQdot(const std::vector<JointRow> & w, Eigen::Matrix<double, NJ, 1> & qddot)
{
  const std::size_t N = w.size();
  const std::size_t c = N / 2;
  const double tc = w[c].sim_t;
  const double scale = std::max(std::abs(w.front().sim_t - tc), std::abs(w.back().sim_t - tc));
  if (!(scale > 0.0)) return false;

  Eigen::MatrixXd X(static_cast<Eigen::Index>(N), 4);
  Eigen::MatrixXd Y(static_cast<Eigen::Index>(N), static_cast<Eigen::Index>(NJ));

  for (std::size_t i = 0; i < N; ++i) {
    const double x = (w[i].sim_t - tc) / scale;
    X(static_cast<Eigen::Index>(i), 0) = 1.0;
    X(static_cast<Eigen::Index>(i), 1) = x;
    X(static_cast<Eigen::Index>(i), 2) = x * x;
    X(static_cast<Eigen::Index>(i), 3) = x * x * x;
    Y.row(static_cast<Eigen::Index>(i)) = w[i].qdot.transpose();
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(X);
  if (qr.rank() < 4) return false;
  const Eigen::MatrixXd coeff = qr.solve(Y);
  if (!coeff.allFinite()) return false;

  qddot = coeff.row(1).transpose() / scale;
  return qddot.allFinite();
}

const std::array<Leg, 4> kLegs = {Leg::FL, Leg::FR, Leg::HL, Leg::HR};
const std::array<std::string, NJ> kJointNames = {
  "FL_HipX", "FL_HipY", "FL_Knee",
  "FR_HipX", "FR_HipY", "FR_Knee",
  "HL_HipX", "HL_HipY", "HL_Knee",
  "HR_HipX", "HR_HipY", "HR_Knee"
};

struct WindowStats
{
  std::size_t valid = 0;
  std::size_t active = 0;
  std::array<Stats, NJ> overall{};
  std::array<Stats, NJ> active_stats{};
  double peak_qddot_fl_knee = 0.0;
  double peak_tau_id_fl_knee = 0.0;
};

void writeStatsHeader(std::ofstream & out)
{
  out << "window,joint,scope,count,MAE_Nm,RMSE_Nm,max_abs_error_Nm\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr
      << "Usage: " << argv[0]
      << " /path/to/lite3_highacc_joint.csv"
      << " /path/to/lite3_highacc_clock.csv\n";
    return 1;
  }

  const std::string joint_path = argv[1];
  const std::string clock_path = argv[2];

  std::vector<JointRow> joints;
  std::vector<ClockRow> clocks;

  if (!loadJointCsv(joint_path, joints)) return 2;
  if (!loadClockCsv(clock_path, clocks)) return 3;
  if (!reconstructSimulationTimes(joints, clocks)) return 4;

  std::cout << "Joint samples : " << joints.size() << "\n";
  std::cout << "Clock samples : " << clocks.size() << "\n";
  std::cout << std::fixed << std::setprecision(6)
            << "Sim time      : " << joints.front().sim_t << " -> " << joints.back().sim_t << " s\n\n";

  std::ofstream summary("lite3_highacc_inverse_dynamics_window_stats.csv");
  if (!summary.is_open()) {
    std::cerr << "Cannot open lite3_highacc_inverse_dynamics_window_stats.csv\n";
    return 5;
  }
  summary << std::fixed << std::setprecision(9);
  writeStatsHeader(summary);

  std::ofstream samples("lite3_highacc_inverse_dynamics_window_samples.csv");
  if (!samples.is_open()) {
    std::cerr << "Cannot open lite3_highacc_inverse_dynamics_window_samples.csv\n";
    return 6;
  }
  samples << std::fixed << std::setprecision(9);
  samples << "window,time_sim,active,FL_Knee_q,FL_Knee_qdot,FL_Knee_qddot,tau_model,tau_gazebo,error\n";

  Lite3SingleLegDynamics dynamics;

  for (const std::size_t N : WINDOWS) {
    WindowStats ws;
    if (N > joints.size() || (N % 2) == 0) continue;
    const std::size_t half = N / 2;

    for (std::size_t c = half; c + half < joints.size(); ++c) {
      std::vector<JointRow> w;
      w.reserve(N);
      for (std::size_t i = c - half; i <= c + half; ++i) w.push_back(joints[i]);

      double center_dt = 0.0, span = 0.0;
      if (!validateWindowTiming(w, center_dt, span)) continue;

      Eigen::Matrix<double, NJ, 1> qddot;
      if (!cubicFitQdot(w, qddot)) continue;

      const Eigen::Matrix<double, NJ, 1> q = w[half].q;
      const Eigen::Matrix<double, NJ, 1> qdot = w[half].qdot;
      const Eigen::Matrix<double, NJ, 1> tau_gz = w[half].effort;

      bool active = std::abs(qdot[2]) >= ACTIVE_QDOT_THRESHOLD;
      ++ws.valid;
      if (active) ++ws.active;

      for (std::size_t li = 0; li < 4; ++li) {
        const int off = static_cast<int>(li * 3);
        const Eigen::Vector3d tau_model = dynamics.inverseDynamics(
          kLegs[li], q.segment<3>(off), qdot.segment<3>(off), qddot.segment<3>(off));

        for (int j = 0; j < 3; ++j) {
          const std::size_t idx = li * 3 + static_cast<std::size_t>(j);
          const double e = tau_gz[idx] - tau_model[j];
          ws.overall[idx].add(e);
          if (active) ws.active_stats[idx].add(e);
        }

        if (li == 0) {
          const double qdd = qddot[2];
          const double tau = tau_model[2];
          const double e_fl_knee = tau_gz[2] - tau_model[2];

          ws.peak_qddot_fl_knee = std::max(ws.peak_qddot_fl_knee, std::abs(qdd));
          ws.peak_tau_id_fl_knee = std::max(ws.peak_tau_id_fl_knee, std::abs(tau));

          samples << N << ','
                  << w[half].sim_t << ','
                  << (active ? 1 : 0) << ','
                  << q[2] << ','
                  << qdot[2] << ','
                  << qdd << ','
                  << tau << ','
                  << tau_gz[2] << ','
                  << e_fl_knee<< '\n';
        }
      }
    }

    for (std::size_t j = 0; j < NJ; ++j) {
      summary << N << ',' << kJointNames[j] << ",overall," << ws.overall[j].count
              << ',' << ws.overall[j].mae()
              << ',' << ws.overall[j].rmse()
              << ',' << ws.overall[j].max_abs << '\n';
      summary << N << ',' << kJointNames[j] << ",active," << ws.active_stats[j].count
              << ',' << ws.active_stats[j].mae()
              << ',' << ws.active_stats[j].rmse()
              << ',' << ws.active_stats[j].max_abs << '\n';
    }

    std::cout << "Window " << N
              << " | valid=" << ws.valid
              << " active=" << ws.active
              << " | FL Knee |qdd|max=" << ws.peak_qddot_fl_knee
              << " rad/s^2"
              << " | |tauID|max=" << ws.peak_tau_id_fl_knee
              << " Nm\n";
  }

  summary.close();
  samples.close();

  std::cout << "\nOutput:\n"
            << "  lite3_highacc_inverse_dynamics_window_stats.csv\n"
            << "  lite3_highacc_inverse_dynamics_window_samples.csv\n";

  return 0;
}
