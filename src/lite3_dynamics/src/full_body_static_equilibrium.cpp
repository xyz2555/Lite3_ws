#include "lite3_dynamics/dynamics.hpp"
#include <lite3_kinematics/kinematics.hpp>

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>

struct LegData
{
  std::string name;
  lite3_kinematics::Leg leg;

  Eigen::Vector3d q;
  Eigen::Vector3d foot_position;
};

int main()
{
  using lite3_dynamics::Lite3SingleLegDynamics;
  using lite3_kinematics::Leg;

  Lite3SingleLegDynamics dynamics;

  constexpr double MASS = 11.9376;
  constexpr double G = 9.81;

  const double W =
    MASS * G;

  /*
   * Standing configurations obtained from the
   * measured standing joint data.
   *
   * Order:
   *   HipX, HipY, Knee
   */

  const LegData legs[] = {
    {
      "FL",
      Leg::FL,

      Eigen::Vector3d(
        -0.02073,
        -0.67214,
         1.32366),

      Eigen::Vector3d(
         0.177383,
         0.166036,
        -0.321490)
    },

    {
      "FR",
      Leg::FR,

      Eigen::Vector3d(
         0.01497,
        -0.67765,
         1.33907),

      Eigen::Vector3d(
         0.1782,
        -0.1642,
        -0.3201)
    },

    {
      "HL",
      Leg::HL,

      Eigen::Vector3d(
        -0.02465,
        -0.64953,
         1.32289),

      Eigen::Vector3d(
        -0.1644,
         0.1673,
        -0.3210)
    },

    {
      "HR",
      Leg::HR,

      Eigen::Vector3d(
         0.01714,
        -0.65116,
         1.33529),

      Eigen::Vector3d(
        -0.1629,
        -0.1649,
        -0.3202)
    }
  };

  constexpr int N = 4;

  /*
   * Force equilibrium:
   *
   * [ 1   1   1   1 ] Fz = W
   *
   * Moment equilibrium around torso:
   *
   * [ y1  y2  y3  y4 ] Fz = 0
   *
   * [-x1 -x2 -x3 -x4] Fz = 0
   *
   * Because right hand side is zero, the sign of
   * the third row does not change the solution.
   */

  Eigen::Matrix<double, 3, 4> A =
    Eigen::Matrix<double, 3, 4>::Zero();

  for (int i = 0; i < N; ++i)
  {
    const double x =
      legs[i].foot_position.x();

    const double y =
      legs[i].foot_position.y();

    A(0, i) = 1.0;
    A(1, i) = y;
    A(2, i) = -x;
  }

  Eigen::Vector3d b;

  b <<
    W,
    0.0,
    0.0;

  /*
   * Minimum-norm solution:
   *
   * F = A^T (A A^T)^-1 b
   */
  const Eigen::Vector4d Fz =
    A.transpose() *
    (A * A.transpose()).inverse() *
    b;

  std::cout
    << std::setprecision(10);

  std::cout
    << "============================================\n"
    << "Lite3 Full-Body Static Equilibrium\n"
    << "============================================\n\n";

  std::cout
    << "Robot mass = "
    << MASS
    << " kg\n";

  std::cout
    << "Robot weight = "
    << W
    << " N\n\n";

  std::cout
    << "Foot positions [m]:\n";

  for (int i = 0; i < N; ++i)
  {
    std::cout
      << "  "
      << legs[i].name
      << " = "
      << legs[i].foot_position.transpose()
      << "\n";
  }

  std::cout
    << "\nContact force distribution:\n";

  for (int i = 0; i < N; ++i)
  {
    std::cout
      << "  "
      << legs[i].name
      << ": Fz = "
      << Fz(i)
      << " N"
      << "  ("
      << 100.0 * Fz(i) / W
      << "% BW)"
      << "\n";
  }

  /*
   * Check force equilibrium.
   */
  const double sumF =
    Fz.sum();

  /*
   * Check moment equilibrium.
   */
  double Mx = 0.0;
  double My = 0.0;

  for (int i = 0; i < N; ++i)
  {
    const double x =
      legs[i].foot_position.x();

    const double y =
      legs[i].foot_position.y();

    Mx += y * Fz(i);
    My += -x * Fz(i);
  }

  std::cout
    << "\nEquilibrium check:\n"
    << "  Sum Fz = "
    << sumF
    << " N\n"
    << "  Sum Mx = "
    << Mx
    << " Nm\n"
    << "  Sum My = "
    << My
    << " Nm\n";

  /*
   * Calculate contact torque and static joint torque
   * for each leg.
   *
   * tau = G - J^T F
   */
  std::cout
    << "\nJoint torque under distributed static load:\n";

  for (int i = 0; i < N; ++i)
  {
    const Eigen::Vector3d force(
      0.0,
      0.0,
      Fz(i));

    const Eigen::Vector3d Gq =
      dynamics.gravityTorque(
        legs[i].leg,
        legs[i].q);

    const Eigen::Vector3d tau_contact =
      dynamics.contactTorque(
        legs[i].leg,
        legs[i].q,
        force);

    const Eigen::Vector3d tau_static =
      Gq - tau_contact;

    std::cout
      << "\n"
      << legs[i].name
      << "\n";

    std::cout
      << "  Fz = "
      << Fz(i)
      << " N\n";

    std::cout
      << "  G = "
      << Gq.transpose()
      << " Nm\n";

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
      << " Nm\n";
  }

  /*
   * Check for physically meaningful contact forces.
   */
  bool positive = true;

  for (int i = 0; i < N; ++i)
  {
    if (Fz(i) < 0.0)
    {
      positive = false;
    }
  }

  std::cout
    << "\nContact feasibility:\n"
    << "  All Fz >= 0 : "
    << (positive ? "YES" : "NO")
    << "\n";

  if (
    std::abs(sumF - W) < 1e-9 &&
    std::abs(Mx) < 1e-9 &&
    std::abs(My) < 1e-9 &&
    positive)
  {
    std::cout
      << "\nFULL-BODY STATIC EQUILIBRIUM PASSED\n";

    return 0;
  }

  std::cout
    << "\nFULL-BODY STATIC EQUILIBRIUM FAILED\n";

  return 1;
}