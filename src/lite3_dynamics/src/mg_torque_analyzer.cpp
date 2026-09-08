#include "lite3_dynamics/dynamics.hpp"

#include <lite3_kinematics/kinematics.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>

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

  /*
   * Existing Lite3 trajectory CSV:
   *
   * 0  = t
   * 1  = x
   * 2  = y
   * 3  = z
   * 4  = qx
   * 5  = qy
   * 6  = qknee
   * 7  = qdot_x
   * 8  = qdot_y
   * 9  = qdot_knee
   * 10 = qddot_x
   * 11 = qddot_y
   * 12 = qddot_knee
   */

  double values[13];
  int index = 0;

  while (std::getline(ss, cell, ','))
  {
    if (index < 13)
    {
      try
      {
        values[index] = std::stod(cell);
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

  sample.t = values[0];

  sample.q <<
    values[4],
    values[5],
    values[6];

  sample.qdot <<
    values[7],
    values[8],
    values[9];

  sample.qddot <<
    values[10],
    values[11],
    values[12];

  return true;
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cerr
      << "Usage:\n"
      << "  ros2 run lite3_dynamics mg_torque_analyzer "
      << "<input.csv> <output.csv>\n";

    return 1;
  }

  const std::string input_file = argv[1];
  const std::string output_file = argv[2];

  std::ifstream input(input_file);

  if (!input.is_open())
  {
    std::cerr
      << "Cannot open input file: "
      << input_file
      << std::endl;

    return 1;
  }

  std::ofstream output(output_file);

  if (!output.is_open())
  {
    std::cerr
      << "Cannot open output file: "
      << output_file
      << std::endl;

    return 1;
  }

  lite3_dynamics::Lite3SingleLegDynamics dynamics;

  std::string line;

  /*
   * Skip CSV header.
   */
  if (!std::getline(input, line))
  {
    std::cerr
      << "Empty CSV file."
      << std::endl;

    return 1;
  }

  /*
   * Output:
   *
   * t
   * q
   * qdot
   * qddot
   * G
   * Cqdot
   * tau_ID
   */
  output
    << "t,"
    << "q_hipx,q_hipy,q_knee,"
    << "qdot_hipx,qdot_hipy,qdot_knee,"
    << "qddot_hipx,qddot_hipy,qddot_knee,"
    << "G_hipx,G_hipy,G_knee,"
    << "Cqdot_hipx,Cqdot_hipy,Cqdot_knee,"
    << "tauID_hipx,tauID_hipy,tauID_knee\n";

  /*
   * Maximum absolute values.
   */
  Eigen::Vector3d max_abs_G =
    Eigen::Vector3d::Zero();

  Eigen::Vector3d max_abs_Cqdot =
    Eigen::Vector3d::Zero();

  Eigen::Vector3d max_abs_tau =
    Eigen::Vector3d::Zero();

  double max_abs_tau_norm = 0.0;

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
      std::cerr
        << "Warning: skipping malformed line:\n"
        << line
        << std::endl;

      continue;
    }

    /*
     * Gravity term:
     *
     * G(q)
     */
    const Eigen::Vector3d G =
      dynamics.gravityTorque(
        lite3_kinematics::Leg::FL,
        sample.q);

    /*
     * Coriolis / centrifugal term:
     *
     * C(q,qdot) qdot
     */
    const Eigen::Vector3d Cqdot =
      dynamics.coriolisTorque(
        lite3_kinematics::Leg::FL,
        sample.q,
        sample.qdot);

    /*
     * Complete inverse dynamics:
     *
     * tau =
     *   M(q) qddot
     *   + C(q,qdot) qdot
     *   + G(q)
     */
    const Eigen::Vector3d tauID =
      dynamics.inverseDynamics(
        lite3_kinematics::Leg::FL,
        sample.q,
        sample.qdot,
        sample.qddot);

    /*
     * Write output sample.
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

      << G(0) << ","
      << G(1) << ","
      << G(2) << ","

      << Cqdot(0) << ","
      << Cqdot(1) << ","
      << Cqdot(2) << ","

      << tauID(0) << ","
      << tauID(1) << ","
      << tauID(2)
      << "\n";

    /*
     * Track maximum absolute values.
     */
    for (int i = 0; i < 3; ++i)
    {
      max_abs_G(i) =
        std::max(
          max_abs_G(i),
          std::abs(G(i)));

      max_abs_Cqdot(i) =
        std::max(
          max_abs_Cqdot(i),
          std::abs(Cqdot(i)));

      max_abs_tau(i) =
        std::max(
          max_abs_tau(i),
          std::abs(tauID(i)));
    }

    max_abs_tau_norm =
      std::max(
        max_abs_tau_norm,
        tauID.norm());

    ++sample_count;
  }

  std::cout
    << std::setprecision(10);

  std::cout
    << "\nSamples processed: "
    << sample_count
    << "\n";

  /*
   * Gravity
   */
  std::cout
    << "\nMaximum |G|:\n"
    << "HipX = "
    << max_abs_G(0)
    << " Nm\n"

    << "HipY = "
    << max_abs_G(1)
    << " Nm\n"

    << "Knee = "
    << max_abs_G(2)
    << " Nm\n";

  /*
   * Coriolis / centrifugal
   */
  std::cout
    << "\nMaximum |C qdot|:\n"
    << "HipX = "
    << max_abs_Cqdot(0)
    << " Nm\n"

    << "HipY = "
    << max_abs_Cqdot(1)
    << " Nm\n"

    << "Knee = "
    << max_abs_Cqdot(2)
    << " Nm\n";

  /*
   * Complete inverse dynamics
   */
  std::cout
    << "\nMaximum |tau_ID|:\n"
    << "HipX = "
    << max_abs_tau(0)
    << " Nm\n"

    << "HipY = "
    << max_abs_tau(1)
    << " Nm\n"

    << "Knee = "
    << max_abs_tau(2)
    << " Nm\n";

  std::cout
    << "\nMaximum ||tau_ID|| = "
    << max_abs_tau_norm
    << " Nm\n";

  std::cout
    << "\nOutput: "
    << output_file
    << std::endl;

  return 0;
}