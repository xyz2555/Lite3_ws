    #include "lite3_dynamics/dynamics.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <random>

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

  double global_max_symmetry_error = 0.0;
  double global_min_eigenvalue =
    std::numeric_limits<double>::max();

  double global_max_energy_error = 0.0;

  std::cout << std::setprecision(12);

  for (const auto leg : legs)
  {
    double max_symmetry_error = 0.0;
    double min_eigenvalue =
      std::numeric_limits<double>::max();

    double max_energy_error = 0.0;

    for (int k = 0; k < SAMPLES_PER_LEG; ++k)
    {
      const Eigen::Vector3d q(
        d0(rng),
        d1(rng),
        d2(rng));

      const Eigen::Vector3d qdot(
        dv(rng),
        dv(rng),
        dv(rng));

      const Eigen::Matrix3d M =
        dynamics.massMatrix(leg, q);

      /*
       * 1. Symmetry
       */
      const double symmetry_error =
        (M - M.transpose()).norm();

      max_symmetry_error =
        std::max(
          max_symmetry_error,
          symmetry_error);

      /*
       * 2. Positive definiteness
       */
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
        solver(M);

      if (solver.info() != Eigen::Success)
      {
        std::cerr
          << "Eigenvalue solver failed."
          << std::endl;

        return 1;
      }

      const double min_eval =
        solver.eigenvalues().minCoeff();

      min_eigenvalue =
        std::min(min_eigenvalue, min_eval);

      /*
       * 3. Kinetic energy must be positive.
       *
       * Here we calculate it using M(q).
       */
      const double T =
        dynamics.kineticEnergy(
          leg,
          q,
          qdot);

      /*
       * M(q) should produce positive kinetic energy.
       */
      if (T < -1e-12)
      {
        std::cerr
          << "Negative kinetic energy!"
          << std::endl;

        return 1;
      }

      /*
       * Recompute kinetic energy directly
       * from the mass matrix expression.
       */
      const double T_direct =
        0.5 *
        qdot.transpose() *
        M *
        qdot;

      const double energy_error =
        std::abs(T - T_direct);

      max_energy_error =
        std::max(
          max_energy_error,
          energy_error);
    }

    global_max_symmetry_error =
      std::max(
        global_max_symmetry_error,
        max_symmetry_error);

    global_min_eigenvalue =
      std::min(
        global_min_eigenvalue,
        min_eigenvalue);

    global_max_energy_error =
      std::max(
        global_max_energy_error,
        max_energy_error);

    std::cout << "Leg ";

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
      << " | max symmetry error = "
      << max_symmetry_error
      << " kg m^2"

      << " | min eigenvalue = "
      << min_eigenvalue
      << " kg m^2"

      << " | max energy error = "
      << max_energy_error
      << " J"

      << std::endl;
  }

  /*
   * Standing configuration.
   */
  const Eigen::Vector3d q_standing(
    -0.02073,
    -0.67214,
     1.32366);

  const Eigen::Matrix3d M_standing =
    dynamics.massMatrix(
      Leg::FL,
      q_standing);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
    standing_solver(M_standing);

  std::cout
    << "\nFL standing configuration:\n";

  std::cout
    << "q = "
    << q_standing.transpose()
    << " rad\n";

  std::cout
    << "M(q) =\n"
    << M_standing
    << "\n";

  std::cout
    << "eigenvalues(M) = "
    << standing_solver.eigenvalues().transpose()
    << "\n";

  std::cout
    << "\nGlobal max symmetry error = "
    << global_max_symmetry_error
    << " kg m^2\n";

  std::cout
    << "Global minimum eigenvalue = "
    << global_min_eigenvalue
    << " kg m^2\n";

  std::cout
    << "Global max energy error = "
    << global_max_energy_error
    << " J\n";

  if (
    global_max_symmetry_error < 1e-12 &&
    global_min_eigenvalue > 0.0 &&
    global_max_energy_error < 1e-12)
  {
    std::cout
      << "\nALL MASS MATRIX TESTS PASSED\n";

    return 0;
  }

  std::cout
    << "\nMASS MATRIX TEST FAILED\n";

  return 1;
}