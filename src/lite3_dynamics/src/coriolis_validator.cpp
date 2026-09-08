#include "lite3_dynamics/dynamics.hpp"

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <random>
#include <limits>

int main()
{
  using lite3_dynamics::Lite3SingleLegDynamics;
  using lite3_kinematics::Leg;

  Lite3SingleLegDynamics dynamics;

  std::mt19937 rng(42);

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

  std::uniform_real_distribution<double> dv(
    -5.0, 5.0);

  const Leg legs[] = {
    Leg::FL,
    Leg::FR,
    Leg::HL,
    Leg::HR
  };

  constexpr int SAMPLES_PER_LEG = 5000;
  constexpr double EPS = 1e-6;

  double global_max_identity_error = 0.0;

  std::cout << std::setprecision(12);

  for (const auto leg : legs)
  {
    double max_identity_error = 0.0;

    for (int sample = 0;
         sample < SAMPLES_PER_LEG;
         ++sample)
    {
      Eigen::Vector3d q(
        d0(rng),
        d1(rng),
        d2(rng));

      Eigen::Vector3d qdot(
        dv(rng),
        dv(rng),
        dv(rng));

      const Eigen::Matrix3d C =
        dynamics.coriolisMatrix(
          leg,
          q,
          qdot);

      /*
       * Numerical time derivative:
       *
       * Mdot = sum_k dM/dq_k * qdot_k
       */
      Eigen::Matrix3d Mdot =
        Eigen::Matrix3d::Zero();

      for (int k = 0; k < 3; ++k)
      {
        Eigen::Vector3d qp = q;
        Eigen::Vector3d qm = q;

        qp(k) += EPS;
        qm(k) -= EPS;

        const Eigen::Matrix3d Mp =
          dynamics.massMatrix(
            leg,
            qp);

        const Eigen::Matrix3d Mm =
          dynamics.massMatrix(
            leg,
            qm);

        const Eigen::Matrix3d dM =
          (Mp - Mm) /
          (2.0 * EPS);

        Mdot +=
          dM * qdot(k);
      }

      /*
       * Fundamental rigid-body dynamics identity:
       *
       * qdot^T (Mdot - 2C) qdot = 0
       */
      const double identity_error =
        std::abs(
          (
            qdot.transpose() *
            (Mdot - 2.0 * C) *
            qdot
          )(0)
        );

      max_identity_error =
        std::max(
          max_identity_error,
          identity_error);

      global_max_identity_error =
        std::max(
          global_max_identity_error,
          identity_error);
    }

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
      << " | max identity error = "
      << max_identity_error
      << std::endl;
  }

  /*
   * Standing configuration.
   */
  const Eigen::Vector3d q_standing(
    -0.02073,
    -0.67214,
     1.32366);

  const Eigen::Vector3d qdot_standing =
    Eigen::Vector3d::Zero();

  const Eigen::Matrix3d C_standing =
    dynamics.coriolisMatrix(
      Leg::FL,
      q_standing,
      qdot_standing);

  const Eigen::Vector3d Cqdot_standing =
    dynamics.coriolisTorque(
      Leg::FL,
      q_standing,
      qdot_standing);

  std::cout
    << "\nFL standing configuration:\n";

  std::cout
    << "C(q,0) =\n"
    << C_standing
    << "\n";

  std::cout
    << "C(q,0) qdot = "
    << Cqdot_standing.transpose()
    << " Nm\n";

  std::cout
    << "\nGlobal max identity error = "
    << global_max_identity_error
    << std::endl;

  if (global_max_identity_error < 1e-6)
  {
    std::cout
      << "\nALL CORIOLIS TESTS PASSED\n";

    return 0;
  }

  std::cout
    << "\nCORIOLIS TEST FAILED\n";

  return 1;
}