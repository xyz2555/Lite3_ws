#include "lite3_dynamics/dynamics.hpp"
#include <lite3_kinematics/kinematics.hpp>

#include <iomanip>
#include <iostream>

int main()
{
  using lite3_dynamics::Lite3SingleLegDynamics;
  using lite3_kinematics::Leg;

  Lite3SingleLegDynamics dynamics;

  /*
   * Measured total robot mass from the MJCF model:
   *
   * torso = 5.6056 kg
   * four legs = 4 * 1.583 kg
   *
   * total = 11.9376 kg
   */
  constexpr double ROBOT_MASS = 11.9376;
  constexpr double G = 9.81;

  /*
   * Nominal equal load:
   *
   * 1/4 of total robot weight.
   */
  constexpr double FZ_EQUAL =
    ROBOT_MASS * G / 4.0;

  /*
   * Additional load scenarios.
   *
   * load_factor = 1:
   *   25% robot weight
   *
   * load_factor = 2:
   *   50% robot weight
   *
   * load_factor = 3:
   *   75% robot weight
   *
   * load_factor = 4:
   *   100% robot weight
   *
   * These are load-envelope scenarios,
   * not measured Lite3 contact forces.
   */
  const double load_factors[] = {
    1.0,
    2.0,
    3.0,
    4.0
  };

  /*
   * FL standing configuration.
   */
  const Eigen::Vector3d q_standing(
    -0.02073,
    -0.67214,
     1.32366);

  std::cout << std::setprecision(10);

  std::cout
    << "============================================\n"
    << "Lite3 Static Contact Force Analysis\n"
    << "============================================\n\n";

  std::cout
    << "Robot mass = "
    << ROBOT_MASS
    << " kg\n";

  std::cout
    << "Robot weight = "
    << ROBOT_MASS * G
    << " N\n";

  std::cout
    << "Equal 4-leg load = "
    << FZ_EQUAL
    << " N\n\n";

  const Eigen::Matrix3d J =
    dynamics.footJacobian(
      Leg::FL,
      q_standing);

  const Eigen::Vector3d Gq =
    dynamics.gravityTorque(
      Leg::FL,
      q_standing);

  std::cout
    << "FL standing foot Jacobian:\n"
    << J
    << "\n\n";

  std::cout
    << "Gravity torque:\n"
    << Gq.transpose()
    << " Nm\n\n";

  for (double factor : load_factors)
  {
    const double Fz =
      FZ_EQUAL * factor;

    const Eigen::Vector3d F(
      0.0,
      0.0,
      Fz);

    const Eigen::Vector3d tau_contact =
      dynamics.contactTorque(
        Leg::FL,
        q_standing,
        F);

    /*
     * Static actuator torque:
     *
     * tau = G - J^T F
     *
     * qdot = 0
     * qddot = 0
     */
    const Eigen::Vector3d tau_static =
      Gq - tau_contact;

    std::cout
      << "Load factor = "
      << factor
      << "\n";

    std::cout
      << "  Fz = "
      << Fz
      << " N\n";

    std::cout
      << "  J^T F = "
      << tau_contact.transpose()
      << " Nm\n";

    std::cout
      << "  tau_static = "
      << tau_static.transpose()
      << " Nm\n";

    std::cout
      << "  ||tau_static|| = "
      << tau_static.norm()
      << " Nm\n\n";
  }

  return 0;
}