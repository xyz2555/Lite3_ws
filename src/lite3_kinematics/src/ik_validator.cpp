#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using lite3_kinematics::IKResult;
using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

std::string legName(Leg leg)
{
  switch (leg)
  {
    case Leg::FL: return "FL";
    case Leg::FR: return "FR";
    case Leg::HL: return "HL";
    case Leg::HR: return "HR";
  }

  return "UNKNOWN";
}

void printVector(
  const std::string& name,
  const Eigen::Vector3d& v)
{
  std::cout
    << name << " = [ "
    << v(0) << ", "
    << v(1) << ", "
    << v(2) << " ]"
    << std::endl;
}

bool runKnownTest(
  const Lite3Kinematics& kin,
  Leg leg,
  const Eigen::Vector3d& q_input)
{
  const Eigen::Vector3d target =
    kin.forward(leg, q_input);

  const IKResult ik =
    kin.inverse(leg, target, &q_input);

    std::cout
  << "  input q      = ["
  << q_input(0) << ", "
  << q_input(1) << ", "
  << q_input(2) << "]\n";

std::cout
  << "  target       = ["
  << target(0) << ", "
  << target(1) << ", "
  << target(2) << "]\n";

if (ik.success)
{
  std::cout
    << "  recovered q  = ["
    << ik.q(0) << ", "
    << ik.q(1) << ", "
    << ik.q(2) << "]\n";
}

  if (!ik.success)
  {
    std::cout
      << "[FAIL] "
      << legName(leg)
      << " IK failed"
      << std::endl;

    return false;
  }

  const Eigen::Vector3d q_error =
    ik.q - q_input;

  const Eigen::Vector3d reconstructed =
    kin.forward(leg, ik.q);

  const double joint_error =
    q_error.norm();

  const double position_error =
    (reconstructed - target).norm();

  std::cout
    << "["
    << legName(leg)
    << "] ";

  std::cout
    << "joint error = "
    << std::scientific
    << joint_error
    << " rad, ";

  std::cout
    << "position error = "
    << position_error
    << " m"
    << std::endl;

  return
    joint_error < 1e-8 &&
    position_error < 1e-8;
}

}  // namespace


int main()
{
  Lite3Kinematics kin;

  std::cout << std::fixed
            << std::setprecision(10);

  std::cout
    << "==============================================\n"
    << " Lite3 IK Numerical Validator\n"
    << "==============================================\n";

  /*
   * --------------------------------------------------
   * 1. Known regression tests
   * --------------------------------------------------
   */

  std::cout
    << "\n[1] Known regression tests\n";

  std::vector<Eigen::Vector3d> tests =
  {
    Eigen::Vector3d(
      0.0,
      -0.0002152,
      1.5702284),

    Eigen::Vector3d(
      0.0,
      -0.4997368,
      1.0000532),

    Eigen::Vector3d(
      0.0995792,
      -0.4997368,
      1.0000532),

    Eigen::Vector3d(
      -0.0995792,
      -0.4997368,
      1.0000532)
  };

  bool all_pass = true;

  for (std::size_t i = 0; i < tests.size(); ++i)
  {
    std::cout
      << "Test "
      << i + 1
      << ": ";

    bool pass =
      runKnownTest(
        kin,
        Leg::FL,
        tests[i]);

    if (!pass)
    {
      all_pass = false;
    }
  }

  /*
   * --------------------------------------------------
   * 2. Random configuration test
   * --------------------------------------------------
   */

  std::cout
    << "\n[2] Random joint-space validation\n";

  const auto& lim = kin.limits();

  std::mt19937 rng(42);

  std::uniform_real_distribution<double> hipx_dist(
    lim.hipx_min,
    lim.hipx_max);

  std::uniform_real_distribution<double> hipy_dist(
    lim.hipy_min,
    lim.hipy_max);

  std::uniform_real_distribution<double> knee_dist(
    lim.knee_min,
    lim.knee_max);

  const std::vector<Leg> legs =
  {
    Leg::FL,
    Leg::FR,
    Leg::HL,
    Leg::HR
  };

  constexpr int SAMPLES_PER_LEG = 5000;

  double max_joint_error = 0.0;
  double max_position_error = 0.0;

  double sum_joint_error = 0.0;
  double sum_position_error = 0.0;

  int failures = 0;

  for (const Leg leg : legs)
  {
    for (int i = 0; i < SAMPLES_PER_LEG; ++i)
    {
      Eigen::Vector3d q;

      q <<
        hipx_dist(rng),
        hipy_dist(rng),
        knee_dist(rng);

      const Eigen::Vector3d target =
        kin.forward(leg, q);

      const IKResult ik =
        kin.inverse(leg, target, &q);

      if (!ik.success)
      {
        ++failures;
        continue;
      }

      const double joint_error =
        (ik.q - q).norm();

      const double position_error =
        (
          kin.forward(leg, ik.q) -
          target
        ).norm();

      max_joint_error =
        std::max(
          max_joint_error,
          joint_error);

      max_position_error =
        std::max(
          max_position_error,
          position_error);

      sum_joint_error += joint_error;
      sum_position_error += position_error;
    }
  }

  const int total_samples =
    static_cast<int>(legs.size()) *
    SAMPLES_PER_LEG;

  const int successful_samples =
    total_samples - failures;

  std::cout
    << "Total samples      : "
    << total_samples
    << "\n";

  std::cout
    << "Successful samples  : "
    << successful_samples
    << "\n";

  std::cout
    << "Failures            : "
    << failures
    << "\n";

  if (successful_samples > 0)
  {
    std::cout
      << "Mean joint error    : "
      << sum_joint_error /
         successful_samples
      << " rad\n";

    std::cout
      << "Max joint error     : "
      << max_joint_error
      << " rad\n";

    std::cout
      << "Mean position error : "
      << sum_position_error /
         successful_samples
      << " m\n";

    std::cout
      << "Max position error  : "
      << max_position_error
      << " m\n";
  }

  /*
   * --------------------------------------------------
   * 3. Explicit unreachable target
   * --------------------------------------------------
   */

  std::cout
    << "\n[3] Unreachable-target test\n";

  Eigen::Vector3d impossible_target;

  impossible_target <<
    1.0,
    0.0,
    -1.0;

  IKResult impossible =
    kin.inverse(
      Leg::FL,
      impossible_target,
      nullptr);

  std::cout
    << "Target [1.0, 0.0, -1.0]\n";

  std::cout
    << "IK success = "
    << (impossible.success ? "true" : "false")
    << "\n";

  if (impossible.success)
  {
    all_pass = false;
  }

  /*
   * --------------------------------------------------
   * Final result
   * --------------------------------------------------
   */

  const bool random_test_pass =
    failures == 0 &&
    max_joint_error < 1e-7 &&
    max_position_error < 1e-9;

  std::cout
    << "\n==============================================\n";

  if (all_pass && random_test_pass)
  {
    std::cout
      << " ALL TESTS PASSED\n";
  }
  else
  {
    std::cout
      << " TEST FAILURE\n";
  }

  std::cout
    << "==============================================\n";

  return
    (all_pass && random_test_pass)
      ? 0
      : 1;
}