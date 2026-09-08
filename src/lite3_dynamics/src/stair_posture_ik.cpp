#include <lite3_kinematics/kinematics.hpp>

#include <iomanip>
#include <iostream>
#include <string>

struct LegStanding
{
  std::string name;
  lite3_kinematics::Leg leg;
  Eigen::Vector3d q;
  Eigen::Vector3d foot;
};

const char *ikStatusString(
  lite3_kinematics::IKStatus status)
{
  switch (status)
  {
    case lite3_kinematics::IKStatus::SUCCESS:
      return "SUCCESS";

    case lite3_kinematics::IKStatus::GEOMETRICALLY_UNREACHABLE:
      return "GEOMETRICALLY_UNREACHABLE";

    case lite3_kinematics::IKStatus::JOINT_LIMIT_VIOLATION:
      return "JOINT_LIMIT_VIOLATION";

    case lite3_kinematics::IKStatus::INVALID_TARGET:
      return "INVALID_TARGET";

    default:
      return "UNKNOWN";
  }
}

int main()
{
  using lite3_kinematics::Leg;
  using lite3_kinematics::Lite3Kinematics;

  Lite3Kinematics kinematics;

  /*
   * Measured standing configurations.
   */
  const LegStanding standing[] = {
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

  const double stair_heights[] = {
    0.185,
    0.200,
    0.215
  };

  std::cout << std::setprecision(10);

  std::cout
    << "============================================\n"
    << "Lite3 Stair Posture IK Validation\n"
    << "============================================\n";

  for (double h : stair_heights)
  {
    std::cout
      << "\n============================================\n"
      << "Stair height = "
      << h * 100.0
      << " cm\n"
      << "============================================\n";

    bool all_success = true;

    /*
     * Scenario:
     *
     * FL/FR -> upper stair
     * HL/HR -> lower stair
     *
     * We retain x/y from standing and change z
     * only for the front legs.
     */
    for (int i = 0; i < 4; ++i)
    {
      Eigen::Vector3d target =
        standing[i].foot;

      if (
        standing[i].leg == Leg::FL ||
        standing[i].leg == Leg::FR)
      {
        target.z() += h;
      }

      const auto result =
        kinematics.inverse(
          standing[i].leg,
          target,
          &standing[i].q);

      std::cout
        << "\n"
        << standing[i].name
        << "\n";

      std::cout
        << "  Standing foot = "
        << standing[i].foot.transpose()
        << "\n";

      std::cout
        << "  Target foot   = "
        << target.transpose()
        << "\n";

      std::cout
        << "  IK status     = "
        << ikStatusString(result.status)
        << "\n";

      std::cout
        << "  IK success    = "
        << (result.success ? "YES" : "NO")
        << "\n";

      if (result.success)
      {
        const Eigen::Vector3d fk =
          kinematics.forward(
            standing[i].leg,
            result.q);

        const double fk_error =
          (fk - target).norm();

        std::cout
          << "  q_stair      = "
          << result.q.transpose()
          << " rad\n";

        std::cout
          << "  FK result    = "
          << fk.transpose()
          << "\n";

        std::cout
          << "  FK error     = "
          << fk_error
          << " m\n";

        std::cout
          << "  Within limits= "
          << (
            kinematics.withinLimits(result.q)
              ? "YES"
              : "NO")
          << "\n";
      }
      else
      {
        all_success = false;
      }
    }

    std::cout
      << "\nOverall stair posture IK: "
      << (all_success ? "PASS" : "FAIL")
      << "\n";
  }

  return 0;
}