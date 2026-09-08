#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

Eigen::Matrix3d numericalJacobian(
    const Lite3Kinematics& kin,
    Leg leg,
    const Eigen::Vector3d& q)
{
  constexpr double EPS = 1e-7;

  Eigen::Matrix3d J =
      Eigen::Matrix3d::Zero();

  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d q_plus = q;
    Eigen::Vector3d q_minus = q;

    q_plus(i) += EPS;
    q_minus(i) -= EPS;

    const Eigen::Vector3d p_plus =
        kin.forward(leg, q_plus);

    const Eigen::Vector3d p_minus =
        kin.forward(leg, q_minus);

    J.col(i) =
        (p_plus - p_minus) /
        (2.0 * EPS);
  }

  return J;
}

}  // namespace


int main()
{
  Lite3Kinematics kin;

  std::mt19937_64 rng(12345);

  const auto& lim = kin.limits();

  std::uniform_real_distribution<double> hipx_dist(
      lim.hipx_min,
      lim.hipx_max);

  std::uniform_real_distribution<double> hipy_dist(
      lim.hipy_min,
      lim.hipy_max);

  std::uniform_real_distribution<double> knee_dist(
      lim.knee_min,
      lim.knee_max);

  constexpr int SAMPLES_PER_LEG = 5000;

  double max_error = 0.0;
  double mean_error = 0.0;

  int total = 0;

  std::cout
      << "==============================================\n"
      << " Lite3 Analytical Jacobian Validator\n"
      << "==============================================\n\n";

  for (const Leg leg :
       {Leg::FL, Leg::FR, Leg::HL, Leg::HR})
  {
    double leg_max_error = 0.0;
    double leg_mean_error = 0.0;

    for (int n = 0;
         n < SAMPLES_PER_LEG;
         ++n)
    {
      Eigen::Vector3d q;

      q <<
          hipx_dist(rng),
          hipy_dist(rng),
          knee_dist(rng);

      const Eigen::Matrix3d J_analytic =
          kin.jacobian(
              leg,
              q);

      const Eigen::Matrix3d J_numeric =
          numericalJacobian(
              kin,
              leg,
              q);

      const double error =
          (J_analytic - J_numeric).norm();

      leg_max_error =
          std::max(
              leg_max_error,
              error);

      leg_mean_error += error;

      max_error =
          std::max(
              max_error,
              error);

      mean_error += error;

      ++total;
    }

    leg_mean_error /=
        static_cast<double>(
            SAMPLES_PER_LEG);

    std::cout
        << "Leg "
        << static_cast<int>(leg)
        << ":\n"
        << "  Mean error = "
        << std::scientific
        << leg_mean_error
        << "\n"
        << "  Max error  = "
        << leg_max_error
        << "\n\n";
  }

  mean_error /=
      static_cast<double>(total);

  std::cout
      << "Total samples : "
      << total
      << "\n"
      << "Mean error    : "
      << mean_error
      << "\n"
      << "Max error     : "
      << max_error
      << "\n";

  /*
   * Jacobian and numerical finite difference should
   * agree to approximately floating-point / finite-
   * difference precision.
   */
  constexpr double TOLERANCE = 1e-6;

  std::cout
      << "\nTolerance     : "
      << TOLERANCE
      << "\n";

  if (max_error < TOLERANCE)
  {
    std::cout
        << "\n==============================================\n"
        << " ALL JACOBIAN TESTS PASSED\n"
        << "==============================================\n";

    return 0;
  }

  std::cout
      << "\n==============================================\n"
      << " JACOBIAN TEST FAILURE\n"
      << "==============================================\n";

  return 1;
}