#include <iomanip>
#include <iostream>

#include "lite3_kinematics/kinematics.hpp"
#include "lite3_kinematics/single_leg_stair_analyzer.hpp"

int main() {
  using namespace lite3_kinematics;

  Lite3Kinematics kin;

  const Eigen::Vector3d q_stand(
      -0.02073,
      -0.67213,
       1.32366);

  const Eigen::Vector3d p_stand(
       0.1774,
       0.1660,
      -0.3215);

  SingleLegStairAnalyzer analyzer(
      kin,
      Leg::FL,
      p_stand,
      q_stand,
      0.05,
      25);

  StairParams stair{
      0.185,
      0.315,
      0.04,
      0.35
  };

  std::cout
      << "=== Single-Leg Stair Trajectory ===\n"
      << "Leg     : FL\n"
      << "Riser   : 18.5 cm\n"
      << "Tread   : 31.5 cm\n\n";

  const auto sweep =
      analyzer.sweepForwardDisplacement(
          stair,
          0.03,
          0.26,
          0.01);

  for (const auto& [dx, margin] : sweep) {
    std::cout
        << "dx = "
        << std::setw(5)
        << dx * 100.0
        << " cm   "

        << "worst margin = "
        << margin
        << " rad   "

        << ((margin >= 0.05)
                ? "[OK]"
                : "[FAIL]")
        << "\n";
  }

  return 0;
}