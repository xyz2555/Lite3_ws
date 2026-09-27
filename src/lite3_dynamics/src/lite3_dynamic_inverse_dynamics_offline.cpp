#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
constexpr std::size_t NW = 5;

constexpr double DT_MIN = 0.001;
constexpr double DT_MAX = 0.030;
constexpr double MAX_DT_RATIO = 4.0;

struct JointRow
{
  double wall_t{0.0};
  double sim_t{0.0};
  Eigen::Matrix<double, NJ, 1> q =
    Eigen::Matrix<double, NJ, 1>::Zero();
  Eigen::Matrix<double, NJ, 1> qdot =
    Eigen::Matrix<double, NJ, 1>::Zero();
  Eigen::Matrix<double, NJ, 1> effort =
    Eigen::Matrix<double, NJ, 1>::Zero();
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
    abs_sum += std::abs(e);
    sq_sum += e * e;
    max_abs = std::max(max_abs, std::abs(e));
    ++count;
  }

  double mae() const
  {
    return count ? abs_sum / static_cast<double>(count) : 0.0;
  }

  double rmse() const
  {
    return count ? std::sqrt(sq_sum / static_cast<double>(count)) : 0.0;
  }
};

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;

  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }

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

bool loadClockCsv(
  const std::string & path,
  std::vector<ClockRow> & rows)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Cannot open clock CSV: " << path << "\n";
    return false;
  }

  std::string line;
  std::getline(file, line);  // header

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const auto p = splitCsv(line);
    if (p.size() < 3) {
      continue;
    }

    double wall = 0.0;
    double sim = 0.0;

    if (!parseDouble(p[1], wall) ||
        !parseDouble(p[2], sim))
    {
      continue;
    }

    rows.push_back({wall, sim});
  }

  return !rows.empty();
}

bool loadJointCsv(
  const std::string & path,
  std::vector<JointRow> & rows)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Cannot open joint CSV: " << path << "\n";
    return false;
  }

  std::string line;
  std::getline(file, line);  // header

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    const auto p = splitCsv(line);

    // index, wall_t, latest_clock_sim_t + 12*(q,qdot,effort)
    if (p.size() < 3 + 3 * NJ) {
      continue;
    }

    JointRow row;

    if (!parseDouble(p[1], row.wall_t)) {
      continue;
    }

    // p[2] is retained only for comparison; the simulation time used by
    // this validator is reconstructed from the complete /clock log.

    bool ok = true;

    for (std::size_t j = 0; j < NJ; ++j) {
      const std::size_t base = 3 + 3 * j;

      if (!parseDouble(p[base + 0], row.q[j]) ||
          !parseDouble(p[base + 1], row.qdot[j]) ||
          !parseDouble(p[base + 2], row.effort[j]))
      {
        ok = false;
        break;
      }
    }

    if (ok) {
      rows.push_back(row);
    }
  }

  return !rows.empty();
}

bool reconstructSimulationTimes(
  std::vector<JointRow> & joints,
  const std::vector<ClockRow> & clocks)
{
  if (joints.empty() || clocks.size() < 2) {
    return false;
  }

  for (std::size_t i = 1; i < clocks.size(); ++i) {
    if (!(clocks[i].wall_t > clocks[i - 1].wall_t) ||
        !(clocks[i].sim_t > clocks[i - 1].sim_t))
    {
      std::cerr << "Clock CSV is not strictly increasing.\n";
      return false;
    }
  }

  const double wall_min = clocks.front().wall_t;
  const double wall_max = clocks.back().wall_t;

  for (auto & row : joints) {
    if (row.wall_t < wall_min || row.wall_t > wall_max) {
      return false;
    }

    auto it = std::lower_bound(
      clocks.begin(),
      clocks.end(),
      row.wall_t,
      [](const ClockRow & c, double wall)
      {
        return c.wall_t < wall;
      });

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

    if (wall_dt <= 0.0) {
      return false;
    }

    const double alpha =
      (row.wall_t - c1.wall_t) / wall_dt;

    row.sim_t =
      c1.sim_t +
      alpha * (c2.sim_t - c1.sim_t);
  }

  return true;
}

bool validateWindow(
  const std::array<JointRow, NW> & w,
  double & center_dt,
  double & span)
{
  std::array<double, NW - 1> dt{};

  double min_dt = std::numeric_limits<double>::infinity();
  double max_dt = 0.0;

  for (std::size_t i = 1; i < NW; ++i) {
    dt[i - 1] = w[i].sim_t - w[i - 1].sim_t;

    if (!(dt[i - 1] > 0.0)) {
      return false;
    }

    min_dt = std::min(min_dt, dt[i - 1]);
    max_dt = std::max(max_dt, dt[i - 1]);
  }

  if (min_dt < DT_MIN ||
      max_dt > DT_MAX ||
      max_dt / min_dt > MAX_DT_RATIO)
  {
    return false;
  }

  center_dt = 0.5 * (dt[1] + dt[2]);
  span = w.back().sim_t - w.front().sim_t;

  return std::isfinite(center_dt) &&
         std::isfinite(span) &&
         center_dt > 0.0 &&
         span > 0.0;
}

bool cubicFitQdot(
  const std::array<JointRow, NW> & w,
  Eigen::Matrix<double, NJ, 1> & qddot)
{
  constexpr std::size_t c = 2;
  const double tc = w[c].sim_t;

  const double scale = std::max(
    std::abs(w.front().sim_t - tc),
    std::abs(w.back().sim_t - tc));

  if (!(scale > 0.0)) {
    return false;
  }

  Eigen::Matrix<double, NW, 4> X;
  Eigen::Matrix<double, NW, NJ> Y;

  for (std::size_t i = 0; i < NW; ++i) {
    const double x =
      (w[i].sim_t - tc) / scale;

    X(static_cast<Eigen::Index>(i), 0) = 1.0;
    X(static_cast<Eigen::Index>(i), 1) = x;
    X(static_cast<Eigen::Index>(i), 2) = x * x;
    X(static_cast<Eigen::Index>(i), 3) = x * x * x;

    Y.row(static_cast<Eigen::Index>(i)) =
      w[i].qdot.transpose();
  }

  Eigen::ColPivHouseholderQR<
    Eigen::Matrix<double, NW, 4>> qr(X);

  if (qr.rank() < 4) {
    return false;
  }

  const Eigen::Matrix<double, 4, NJ> coeff =
    qr.solve(Y);

  if (!coeff.allFinite()) {
    return false;
  }

  qddot =
    coeff.row(1).transpose() / scale;

  return qddot.allFinite();
}

bool cubicFitQ(
  const std::array<JointRow, NW> & w,
  Eigen::Matrix<double, NJ, 1> & qdot,
  Eigen::Matrix<double, NJ, 1> & qddot)
{
  constexpr std::size_t c = 2;
  const double tc = w[c].sim_t;

  const double scale = std::max(
    std::abs(w.front().sim_t - tc),
    std::abs(w.back().sim_t - tc));

  if (!(scale > 0.0)) {
    return false;
  }

  Eigen::Matrix<double, NW, 4> X;
  Eigen::Matrix<double, NW, NJ> Y;

  for (std::size_t i = 0; i < NW; ++i) {
    const double x =
      (w[i].sim_t - tc) / scale;

    X(static_cast<Eigen::Index>(i), 0) = 1.0;
    X(static_cast<Eigen::Index>(i), 1) = x;
    X(static_cast<Eigen::Index>(i), 2) = x * x;
    X(static_cast<Eigen::Index>(i), 3) = x * x * x;

    Y.row(static_cast<Eigen::Index>(i)) =
      w[i].q.transpose();
  }

  Eigen::ColPivHouseholderQR<
    Eigen::Matrix<double, NW, 4>> qr(X);

  if (qr.rank() < 4) {
    return false;
  }

  const Eigen::Matrix<double, 4, NJ> coeff =
    qr.solve(Y);

  if (!coeff.allFinite()) {
    return false;
  }

  qdot =
    coeff.row(1).transpose() / scale;

  qddot =
    2.0 * coeff.row(2).transpose() /
    (scale * scale);

  return qdot.allFinite() && qddot.allFinite();
}

const std::array<Leg, 4> kLegs = {
  Leg::FL, Leg::FR, Leg::HL, Leg::HR
};

const std::array<std::string, 12> kJointNames = {
  "FL_HipX", "FL_HipY", "FL_Knee",
  "FR_HipX", "FR_HipY", "FR_Knee",
  "HL_HipX", "HL_HipY", "HL_Knee",
  "HR_HipX", "HR_HipY", "HR_Knee"
};

void writeHeader(std::ofstream & csv)
{
  csv
    << "time_sim,dt_center_sim,window_span_sim";

  for (const auto & name : kJointNames) {
    csv
      << "," << name << "_qddot_qdotfit"
      << "," << name << "_qddot_qpoly"
      << "," << name << "_tauID_qdotfit"
      << "," << name << "_tauID_qpoly"
      << "," << name << "_tauGazebo"
      << "," << name << "_err_qdotfit"
      << "," << name << "_err_qpoly";
  }

  csv << "\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr
      << "Usage:\n  "
      << argv[0]
      << " /path/to/lite3_raw_joint_log2.csv"
      << " /path/to/lite3_clock_log.csv\n";
    return 1;
  }

  const std::string joint_path = argv[1];
  const std::string clock_path = argv[2];

  std::vector<JointRow> joints;
  std::vector<ClockRow> clocks;

  if (!loadJointCsv(joint_path, joints)) {
    return 2;
  }

  if (!loadClockCsv(clock_path, clocks)) {
    return 3;
  }

  if (!reconstructSimulationTimes(joints, clocks)) {
    std::cerr
      << "Failed to reconstruct simulation time.\n";
    return 4;
  }

  std::cout
    << "Joint samples : " << joints.size() << "\n"
    << "Clock samples : " << clocks.size() << "\n"
    << "Sim time      : "
    << joints.front().sim_t
    << " -> "
    << joints.back().sim_t
    << " s\n";

  std::ofstream csv(
    "dynamic_inverse_dynamics_offline_raw.csv");

  if (!csv.is_open()) {
    std::cerr
      << "Cannot open output CSV.\n";
    return 5;
  }

  csv << std::fixed << std::setprecision(9);
  writeHeader(csv);

  std::array<Stats, NJ> stats_qdot{};
  std::array<Stats, NJ> stats_qpoly{};

  std::size_t valid_count = 0;
  std::size_t rejected_count = 0;

  for (std::size_t c = 2; c + 2 < joints.size(); ++c) {
    std::array<JointRow, NW> w = {
      joints[c - 2],
      joints[c - 1],
      joints[c],
      joints[c + 1],
      joints[c + 2]
    };

    double center_dt = 0.0;
    double span = 0.0;

    if (!validateWindow(w, center_dt, span)) {
      ++rejected_count;
      continue;
    }

    Eigen::Matrix<double, NJ, 1> qddot_qdotfit;
    Eigen::Matrix<double, NJ, 1> qdot_qpoly;
    Eigen::Matrix<double, NJ, 1> qddot_qpoly;

    if (!cubicFitQdot(w, qddot_qdotfit) ||
        !cubicFitQ(w, qdot_qpoly, qddot_qpoly))
    {
      ++rejected_count;
      continue;
    }

    std::array<Eigen::Vector3d, 4> tau_id_qdot;
    std::array<Eigen::Vector3d, 4> tau_id_qpoly;
    std::array<Eigen::Vector3d, 4> tau_gz;

    Lite3SingleLegDynamics dynamics;

    for (std::size_t li = 0; li < 4; ++li) {
      const int off = static_cast<int>(li * 3);

      const Eigen::Vector3d q =
        w[2].q.segment<3>(off);

      const Eigen::Vector3d qdot =
        w[2].qdot.segment<3>(off);

      const Eigen::Vector3d qdd1 =
        qddot_qdotfit.segment<3>(off);

      const Eigen::Vector3d qdd2 =
        qddot_qpoly.segment<3>(off);

      tau_gz[li] =
        w[2].effort.segment<3>(off);

      tau_id_qdot[li] =
        dynamics.inverseDynamics(
          kLegs[li], q, qdot, qdd1);

      tau_id_qpoly[li] =
        dynamics.inverseDynamics(
          kLegs[li], q, qdot, qdd2);

      const Eigen::Vector3d err1 =
        tau_gz[li] - tau_id_qdot[li];

      const Eigen::Vector3d err2 =
        tau_gz[li] - tau_id_qpoly[li];

      for (int j = 0; j < 3; ++j) {
        const std::size_t idx = li * 3 + j;
        stats_qdot[idx].add(err1[j]);
        stats_qpoly[idx].add(err2[j]);
      }
    }

    csv
      << w[2].sim_t
      << "," << center_dt
      << "," << span;

    for (std::size_t li = 0; li < 4; ++li) {
      for (int j = 0; j < 3; ++j) {
        const std::size_t idx = li * 3 + j;

        csv
          << "," << qddot_qdotfit[idx]
          << "," << qddot_qpoly[idx]
          << "," << tau_id_qdot[li][j]
          << "," << tau_id_qpoly[li][j]
          << "," << tau_gz[li][j]
          << "," << tau_gz[li][j] - tau_id_qdot[li][j]
          << "," << tau_gz[li][j] - tau_id_qpoly[li][j];
      }
    }

    csv << "\n";
    ++valid_count;
  }

  csv.close();

  std::cout
    << "\n============================================================\n"
    << "LITE3 OFFLINE DYNAMIC INVERSE-DYNAMICS VALIDATION\n"
    << "============================================================\n"
    << "Valid windows    : " << valid_count << "\n"
    << "Rejected windows : " << rejected_count << "\n\n";

  for (std::size_t i = 0; i < NJ; ++i) {
    std::cout
      << std::left
      << std::setw(11) << kJointNames[i]
      << " | qdot-fit MAE=" << stats_qdot[i].mae()
      << " RMSE=" << stats_qdot[i].rmse()
      << " max=" << stats_qdot[i].max_abs
      << " Nm"
      << " | q-poly MAE=" << stats_qpoly[i].mae()
      << " RMSE=" << stats_qpoly[i].rmse()
      << " max=" << stats_qpoly[i].max_abs
      << " Nm\n";
  }

  std::cout
    << "============================================================\n"
    << "Output: dynamic_inverse_dynamics_offline_raw.csv\n";

  return 0;
}
