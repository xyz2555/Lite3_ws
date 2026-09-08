#include "lite3_dynamics/dynamics.hpp"
#include <lite3_kinematics/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct TrajectorySample
{
  double t;
  Eigen::Vector3d q;
  Eigen::Vector3d qdot;
  Eigen::Vector3d qddot;
};

bool parseCSVLine(
  const std::string &line,
  TrajectorySample &sample)
{
  std::stringstream ss(line);
  std::string cell;

  double v[13];
  int index = 0;

  while (std::getline(ss, cell, ','))
  {
    if (index < 13)
    {
      try
      {
        v[index] = std::stod(cell);
      }
      catch (...)
      {
        return false;
      }
    }

    ++index;
  }

  if (index < 13)
  {
    return false;
  }

  sample.t = v[0];

  sample.q <<
    v[4], v[5], v[6];

  sample.qdot <<
    v[7], v[8], v[9];

  sample.qddot <<
    v[10], v[11], v[12];

  return true;
}

bool analyzeFile(
  const std::string &input_file,
  const std::string &output_file,
  lite3_dynamics::Lite3SingleLegDynamics &dynamics,
  Eigen::Vector3d &max_G,
  Eigen::Vector3d &max_tau_M,
  Eigen::Vector3d &max_tau_C,
  Eigen::Vector3d &max_tau_ID,
  double &max_norm_G,
  double &max_norm_tau_M,
  double &max_norm_tau_C,
  double &max_norm_tau_ID)
{
  std::ifstream input(input_file);

  if (!input.is_open())
  {
    std::cerr
      << "Cannot open: "
      << input_file
      << std::endl;

    return false;
  }

  std::ofstream output(output_file);

  if (!output.is_open())
  {
    std::cerr
      << "Cannot create: "
      << output_file
      << std::endl;

    return false;
  }

  std::string line;

  // Skip input CSV header.
  if (!std::getline(input, line))
  {
    return false;
  }

  output
    << "t,"
    << "q_hipx,q_hipy,q_knee,"
    << "qdot_hipx,qdot_hipy,qdot_knee,"
    << "qddot_hipx,qddot_hipy,qddot_knee,"
    << "tauM_hipx,tauM_hipy,tauM_knee,"
    << "tauC_hipx,tauC_hipy,tauC_knee,"
    << "tauG_hipx,tauG_hipy,tauG_knee,"
    << "tauID_hipx,tauID_hipy,tauID_knee\n";

  max_G = Eigen::Vector3d::Zero();
  max_tau_M = Eigen::Vector3d::Zero();
  max_tau_C = Eigen::Vector3d::Zero();
  max_tau_ID = Eigen::Vector3d::Zero();

  max_norm_G = 0.0;
  max_norm_tau_M = 0.0;
  max_norm_tau_C = 0.0;
  max_norm_tau_ID = 0.0;

  std::size_t sample_count = 0;

  while (std::getline(input, line))
  {
    if (line.empty())
    {
      continue;
    }

    TrajectorySample sample;

    if (!parseCSVLine(line, sample))
    {
      continue;
    }

    const auto leg =
      lite3_kinematics::Leg::FL;

    /*
     * Gravity:
     *
     * tau_G = G(q)
     */
    const Eigen::Vector3d tau_G =
      dynamics.gravityTorque(
        leg,
        sample.q);

    /*
     * Inertial term:
     *
     * tau_M = M(q) qddot
     */
    const Eigen::Matrix3d M =
      dynamics.massMatrix(
        leg,
        sample.q);

    const Eigen::Vector3d tau_M =
      M * sample.qddot;

    /*
     * Coriolis / centrifugal:
     *
     * tau_C = C(q,qdot) qdot
     */
    const Eigen::Vector3d tau_C =
      dynamics.coriolisTorque(
        leg,
        sample.q,
        sample.qdot);

    /*
     * Complete inverse dynamics.
     */
    const Eigen::Vector3d tau_ID =
      tau_M + tau_C + tau_G;

    /*
     * Write one sample.
     */
    output
      << std::setprecision(12)

      << sample.t << ","

      << sample.q(0) << ","
      << sample.q(1) << ","
      << sample.q(2) << ","

      << sample.qdot(0) << ","
      << sample.qdot(1) << ","
      << sample.qdot(2) << ","

      << sample.qddot(0) << ","
      << sample.qddot(1) << ","
      << sample.qddot(2) << ","

      << tau_M(0) << ","
      << tau_M(1) << ","
      << tau_M(2) << ","

      << tau_C(0) << ","
      << tau_C(1) << ","
      << tau_C(2) << ","

      << tau_G(0) << ","
      << tau_G(1) << ","
      << tau_G(2) << ","

      << tau_ID(0) << ","
      << tau_ID(1) << ","
      << tau_ID(2)
      << "\n";

    for (int i = 0; i < 3; ++i)
    {
      max_G(i) =
        std::max(
          max_G(i),
          std::abs(tau_G(i)));

      max_tau_M(i) =
        std::max(
          max_tau_M(i),
          std::abs(tau_M(i)));

      max_tau_C(i) =
        std::max(
          max_tau_C(i),
          std::abs(tau_C(i)));

      max_tau_ID(i) =
        std::max(
          max_tau_ID(i),
          std::abs(tau_ID(i)));
    }

    max_norm_G =
      std::max(
        max_norm_G,
        tau_G.norm());

    max_norm_tau_M =
      std::max(
        max_norm_tau_M,
        tau_M.norm());

    max_norm_tau_C =
      std::max(
        max_norm_tau_C,
        tau_C.norm());

    max_norm_tau_ID =
      std::max(
        max_norm_tau_ID,
        tau_ID.norm());

    ++sample_count;
  }

  return sample_count > 0;
}

int main()
{
  lite3_dynamics::Lite3SingleLegDynamics dynamics;

  struct Case
  {
    int height_mm;
    double duration_s;
  };

  const std::vector<Case> cases = {
    {185, 0.6},
    {185, 0.8},
    {185, 1.0},
    {185, 1.2},
    {185, 1.5},
    {185, 2.0},

    {200, 0.6},
    {200, 0.8},
    {200, 1.0},
    {200, 1.2},
    {200, 1.5},
    {200, 2.0},

    {215, 0.6},
    {215, 0.8},
    {215, 1.0},
    {215, 1.2},
    {215, 1.5},
    {215, 2.0}
  };

  std::ofstream summary(
    "lite3_stair_inverse_dynamics_summary.csv");

  if (!summary.is_open())
  {
    std::cerr
      << "Cannot create summary CSV."
      << std::endl;

    return 1;
  }

  summary
    << "height_mm,duration_s,"
    << "max_G_hipx,max_G_hipy,max_G_knee,max_G_norm,"
    << "max_tauM_hipx,max_tauM_hipy,max_tauM_knee,max_tauM_norm,"
    << "max_tauC_hipx,max_tauC_hipy,max_tauC_knee,max_tauC_norm,"
    << "max_tauID_hipx,max_tauID_hipy,max_tauID_knee,max_tauID_norm\n";

  for (const auto &c : cases)
  {
    const int duration_tenths =
      static_cast<int>(
        std::round(c.duration_s * 10.0));

    const std::string duration_tag =
      std::to_string(duration_tenths / 10)
      + "p"
      + std::to_string(duration_tenths % 10)
      + "s";

    const std::string basename =
      "lite3_FL_stair_" +
      std::to_string(c.height_mm) +
      "mm_T" +
      duration_tag;

    const std::string input_file =
      basename + ".csv";

    const std::string output_file =
      basename + "_ID.csv";

    Eigen::Vector3d max_G;
    Eigen::Vector3d max_tau_M;
    Eigen::Vector3d max_tau_C;
    Eigen::Vector3d max_tau_ID;

    double max_norm_G;
    double max_norm_tau_M;
    double max_norm_tau_C;
    double max_norm_tau_ID;

    std::cout
      << "\nAnalyzing: "
      << input_file
      << std::endl;

    const bool success =
      analyzeFile(
        input_file,
        output_file,
        dynamics,
        max_G,
        max_tau_M,
        max_tau_C,
        max_tau_ID,
        max_norm_G,
        max_norm_tau_M,
        max_norm_tau_C,
        max_norm_tau_ID);

    if (!success)
    {
      std::cerr
        << "FAILED: "
        << input_file
        << std::endl;

      continue;
    }

    summary
      << std::setprecision(12)

      << c.height_mm << ","
      << c.duration_s << ","

      << max_G(0) << ","
      << max_G(1) << ","
      << max_G(2) << ","
      << max_norm_G << ","

      << max_tau_M(0) << ","
      << max_tau_M(1) << ","
      << max_tau_M(2) << ","
      << max_norm_tau_M << ","

      << max_tau_C(0) << ","
      << max_tau_C(1) << ","
      << max_tau_C(2) << ","
      << max_norm_tau_C << ","

      << max_tau_ID(0) << ","
      << max_tau_ID(1) << ","
      << max_tau_ID(2) << ","
      << max_norm_tau_ID
      << "\n";

    std::cout
      << "  max |tau_M| = ["
      << max_tau_M(0) << ", "
      << max_tau_M(1) << ", "
      << max_tau_M(2) << "] Nm\n"

      << "  max |tau_C| = ["
      << max_tau_C(0) << ", "
      << max_tau_C(1) << ", "
      << max_tau_C(2) << "] Nm\n"

      << "  max |tau_G| = ["
      << max_G(0) << ", "
      << max_G(1) << ", "
      << max_G(2) << "] Nm\n"

      << "  max |tau_ID| = ["
      << max_tau_ID(0) << ", "
      << max_tau_ID(1) << ", "
      << max_tau_ID(2) << "] Nm\n"

      << "  max ||tau_ID|| = "
      << max_norm_tau_ID
      << " Nm"
      << std::endl;
  }

  summary.close();

  std::cout
    << "\n============================================\n"
    << "Torque decomposition complete\n"
    << "Summary: lite3_stair_inverse_dynamics_summary.csv\n"
    << "============================================\n";

  return 0;
}