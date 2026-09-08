#include "lite3_dynamics/dynamics.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

int main()
{
  using lite3_dynamics::Lite3SingleLegDynamics;
  using lite3_kinematics::Leg;

  Lite3SingleLegDynamics dynamics;

  std::mt19937 rng(42);

  /*
   * Use conservative sampling inside joint limits.
   */
  const Eigen::Vector3d q_min(
    -0.523 + 0.05,
    -2.67  + 0.05,
     0.524 + 0.05);

  const Eigen::Vector3d q_max(
     0.523 - 0.05,
     0.314 - 0.05,
     2.792 - 0.05);

  std::uniform_real_distribution<double> d0(
    q_min(0), q_max(0));

  std::uniform_real_distribution<double> d1(
    q_min(1), q_max(1));

  std::uniform_real_distribution<double> d2(
    q_min(2), q_max(2));

  constexpr int SAMPLES_PER_LEG = 5000;
  constexpr double EPS = 1e-7;

  const Leg legs[] = {
    Leg::FL,
    Leg::FR,
    Leg::HL,
    Leg::HR
  };

  double global_max_error = 0.0;
  double global_mean_error = 0.0;
  int total_samples = 0;

  std::cout << std::setprecision(12);

  for (const auto leg : legs)
  {
    double mean_error = 0.0;
    double max_error = 0.0;

    for (int k = 0; k < SAMPLES_PER_LEG; ++k)
    {
      Eigen::Vector3d q(
        d0(rng),
        d1(rng),
        d2(rng));

      const Eigen::Vector3d G =
        dynamics.gravityTorque(leg, q);

      Eigen::Vector3d G_fd;

      for (int j = 0; j < 3; ++j)
      {
        Eigen::Vector3d qp = q;
        Eigen::Vector3d qm = q;

        qp(j) += EPS;
        qm(j) -= EPS;

        const double Vp =
          dynamics.potentialEnergy(leg, qp);

        const double Vm =
          dynamics.potentialEnergy(leg, qm);

        G_fd(j) = (Vp - Vm) / (2.0 * EPS);
      }

      const double error =
        (G - G_fd).norm();

      mean_error += error;
      max_error = std::max(max_error, error);
      global_max_error = std::max(global_max_error, error);
      global_mean_error += error;

      ++total_samples;
    }

    mean_error /= SAMPLES_PER_LEG;

    std::cout
      << "Leg ";

    switch (leg)
    {
      case Leg::FL:
        std::cout << "FL";
        break;

      case Leg::FR:
        std::cout << "FR";
        break;

      case Leg::HL:
        std::cout << "HL";
        break;

      case Leg::HR:
        std::cout << "HR";
        break;
    }

    std::cout
      << " | mean error = "
      << mean_error
      << " Nm"
      << " | max error = "
      << max_error
      << " Nm"
      << std::endl;
  }

  global_mean_error /= total_samples;

  /*
   * Standing configuration measured previously.
   */
  const Eigen::Vector3d q_standing(
    -0.02073,
    -0.67214,
     1.32366);

  const Eigen::Vector3d G_standing =
    dynamics.gravityTorque(
      Leg::FL,
      q_standing);

  std::cout << "\nFL standing configuration:\n";
  std::cout << "q = "
            << q_standing.transpose()
            << " rad\n";

  std::cout << "G(q) = "
            << G_standing.transpose()
            << " Nm\n";

  std::cout << "\nGlobal mean error = "
            << global_mean_error
            << " Nm\n";

  std::cout << "Global max error = "
            << global_max_error
            << " Nm\n";

  /*
   * Numerical differentiation should agree
   * very closely with analytical gravity torque.
   */
  if (global_max_error < 1e-6)
  {
    std::cout << "\nALL GRAVITY TESTS PASSED\n";
    return 0;
  }

  std::cout << "\nGRAVITY TEST FAILED\n";
  return 1;
}