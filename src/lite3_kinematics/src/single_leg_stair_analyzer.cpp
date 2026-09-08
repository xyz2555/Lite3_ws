#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using lite3_kinematics::JointLimits;
using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

struct StairTestResult
{
  bool exact_feasible = true;
  bool safe_feasible = true;

  // Minimum distance to the exact URDF joint limits.
  double min_exact_margin =
      std::numeric_limits<double>::infinity();

  // Minimum distance after applying safety margin.
  double min_safe_margin =
      std::numeric_limits<double>::infinity();

  // Minimum margin for each individual joint.
  double min_hipx_margin =
      std::numeric_limits<double>::infinity();

  double min_hipy_margin =
      std::numeric_limits<double>::infinity();

  double min_knee_margin =
      std::numeric_limits<double>::infinity();

  // Where the limiting configuration occurred.
  std::string limiting_joint = "N/A";
  std::string limiting_segment = "N/A";

  double limiting_s = 0.0;

  double limiting_q = 0.0;

  std::size_t total_samples = 0;
  std::size_t ik_failures = 0;

  double max_position_error = 0.0;
};


struct JointMarginResult
{
  double hipx;
  double hipy;
  double knee;
};


JointMarginResult computeJointMargins(
    const Eigen::Vector3d& q,
    const JointLimits& lim)
{
  JointMarginResult result;

  result.hipx =
      std::min(
        q(0) - lim.hipx_min,
        lim.hipx_max - q(0));

  result.hipy =
      std::min(
        q(1) - lim.hipy_min,
        lim.hipy_max - q(1));

  result.knee =
      std::min(
        q(2) - lim.knee_min,
        lim.knee_max - q(2));

  return result;
}


Eigen::Vector3d interpolate(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    double s)
{
  return a + s * (b - a);
}


/*
 * Update the result with a valid IK solution.
 */
void updateResult(
    const Eigen::Vector3d& q,
    const std::string& segment_name,
    double s,
    double position_error,
    const JointLimits& lim,
    double safety_margin,
    StairTestResult& result)
{
  const JointMarginResult margin =
      computeJointMargins(q, lim);

  const double safe_hipx =
      margin.hipx - safety_margin;

  const double safe_hipy =
      margin.hipy - safety_margin;

  const double safe_knee =
      margin.knee - safety_margin;

  const double safe_overall =
      std::min({
        safe_hipx,
        safe_hipy,
        safe_knee
      });

  const double exact_overall =
      std::min({
        margin.hipx,
        margin.hipy,
        margin.knee
      });

  result.min_exact_margin =
      std::min(
        result.min_exact_margin,
        exact_overall);

  result.min_safe_margin =
      std::min(
        result.min_safe_margin,
        safe_overall);

  result.min_hipx_margin =
      std::min(
        result.min_hipx_margin,
        safe_hipx);

  result.min_hipy_margin =
      std::min(
        result.min_hipy_margin,
        safe_hipy);

  result.min_knee_margin =
      std::min(
        result.min_knee_margin,
        safe_knee);

  result.max_position_error =
      std::max(
        result.max_position_error,
        position_error);

  /*
   * Determine the limiting joint.
   */
  double local_min = safe_hipx;
  std::string local_joint = "HipX";
  double local_q = q(0);

  if (safe_hipy < local_min)
  {
    local_min = safe_hipy;
    local_joint = "HipY";
    local_q = q(1);
  }

  if (safe_knee < local_min)
  {
    local_min = safe_knee;
    local_joint = "Knee";
    local_q = q(2);
  }

  /*
   * Only update the global limiting location if this
   * is the smallest margin encountered so far.
   */
  if (local_min <= result.min_safe_margin + 1e-14)
  {
    result.limiting_joint = local_joint;
    result.limiting_segment = segment_name;
    result.limiting_s = s;
    result.limiting_q = local_q;
  }

  if (exact_overall < 0.0)
  {
    result.exact_feasible = false;
  }

  if (safe_overall < 0.0)
  {
    result.safe_feasible = false;
  }

  if (position_error > 1e-6)
  {
    result.exact_feasible = false;
    result.safe_feasible = false;
  }
}


/*
 * Test one Cartesian segment.
 */
void testSegment(
    Lite3Kinematics& kin,
    Leg leg,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const std::string& segment_name,
    int samples,
    double safety_margin,
    StairTestResult& result)
{
  const auto& lim = kin.limits();

  for (int i = 0; i <= samples; ++i)
  {
    const double s =
        static_cast<double>(i) /
        static_cast<double>(samples);

    const Eigen::Vector3d p =
        interpolate(p0, p1, s);

    ++result.total_samples;

    const auto ik =
        kin.inverse(
          leg,
          p);

    if (!ik.success)
    {
      ++result.ik_failures;

      result.exact_feasible = false;
      result.safe_feasible = false;

      continue;
    }

    updateResult(
      ik.q,
      segment_name,
      s,
      ik.position_error,
      lim,
      safety_margin,
      result);
  }
}


/*
 * Actual stair geometry.
 *
 * Coordinate convention:
 * +X = forward
 * +Z = upward
 */
StairTestResult testStair(
    Lite3Kinematics& kin,
    Leg leg,
    const Eigen::Vector3d& p_stand,
    double step_height,
    double tread_depth,
    double clearance,
    double pre_riser_distance,
    double landing_offset,
    double foot_radius,
    double safety_margin)
{
  StairTestResult result;

  /*
   * Landing must be inside the tread.
   */
  if (landing_offset <= 0.0 ||
      landing_offset >= tread_depth)
  {
    result.exact_feasible = false;
    result.safe_feasible = false;

    return result;
  }

  const double x_riser =
      p_stand.x() +
      pre_riser_distance;

  const double x_land =
      x_riser +
      landing_offset;

  /*
   * Upper tread surface.
   *
   * We know the standing foot center is approximately
   * one foot radius above the lower ground plane.
   */
  const double z_ground =
      p_stand.z() - foot_radius;

  const double z_tread =
      z_ground + step_height;

  /*
   * Center of the foot sphere while clearing the step.
   */
  const double z_clear =
      z_tread +
      foot_radius +
      clearance;

  /*
   * Center of the foot sphere during final landing.
   */
  const double z_land =
      z_tread +
      foot_radius;

  /*
   * Waypoints.
   */
  const Eigen::Vector3d P0 =
      p_stand;

  const Eigen::Vector3d P1(
      p_stand.x(),
      p_stand.y(),
      p_stand.z() + clearance);

  const Eigen::Vector3d P2(
      x_riser - foot_radius,
      p_stand.y(),
      z_clear);

  const Eigen::Vector3d P3(
      x_riser + foot_radius,
      p_stand.y(),
      z_clear);

  const Eigen::Vector3d P4(
      x_land,
      p_stand.y(),
      z_clear);

  const Eigen::Vector3d P5(
      x_land,
      p_stand.y(),
      z_land);

  constexpr int SEGMENT_SAMPLES = 100;

  testSegment(
    kin,
    leg,
    P0,
    P1,
    "STAND -> LIFT",
    SEGMENT_SAMPLES,
    safety_margin,
    result);

  testSegment(
    kin,
    leg,
    P1,
    P2,
    "LIFT -> PRE-RISER",
    SEGMENT_SAMPLES,
    safety_margin,
    result);

  testSegment(
    kin,
    leg,
    P2,
    P3,
    "CROSS RISER",
    SEGMENT_SAMPLES,
    safety_margin,
    result);

  testSegment(
    kin,
    leg,
    P3,
    P4,
    "CROSS -> LANDING",
    SEGMENT_SAMPLES,
    safety_margin,
    result);

  testSegment(
    kin,
    leg,
    P4,
    P5,
    "LOWER -> LAND",
    SEGMENT_SAMPLES,
    safety_margin,
    result);

  return result;
}


void printResultDetail(
    const StairTestResult& result)
{
  std::cout
      << "\n  Joint safe margins:\n"
      << "    HipX = "
      << result.min_hipx_margin
      << " rad\n"
      << "    HipY = "
      << result.min_hipy_margin
      << " rad\n"
      << "    Knee = "
      << result.min_knee_margin
      << " rad\n";

  std::cout
      << "\n  Limiting joint : "
      << result.limiting_joint
      << "\n"
      << "  Segment       : "
      << result.limiting_segment
      << "\n"
      << "  trajectory s  : "
      << result.limiting_s
      << "\n"
      << "  limiting q    : "
      << result.limiting_q
      << " rad\n";

  std::cout
      << "  Max IK error  : "
      << std::scientific
      << result.max_position_error
      << std::fixed
      << " m\n";

  std::cout
      << "  IK failures   : "
      << result.ik_failures
      << "\n";
}

}  // namespace


int main()
{
  Lite3Kinematics kin;

  constexpr Leg LEG = Leg::FL;

  /*
   * Measured standing pose.
   */
  const Eigen::Vector3d q_stand(
      -0.02073,
      -0.67214,
       1.32366);

  const Eigen::Vector3d p_stand =
      kin.forward(
        LEG,
        q_stand);

  /*
   * Stair parameters.
   */
  constexpr double TREAD_DEPTH =
      0.315;

  constexpr double CLEARANCE =
      0.040;

  constexpr double FOOT_RADIUS =
      0.022;

  constexpr double SAFETY_MARGIN =
      0.050;

  /*
   * This is the approach distance that produced the
   * best margin in our previous experiment.
   */
  constexpr double PRE_RISER_DISTANCE =
      0.125;

  /*
   * 5 cm inside the tread.
   */
  constexpr double LANDING_OFFSET =
      0.050;

  /*
   * Heights to investigate.
   */
  const std::vector<double> heights = {
    0.285,
    0.286,
    0.287,
    0.288,
    0.289,
    0.290,
    0.291,
    0.292,
    0.293,
    0.294,
    0.295,
    0.296,
    0.297,
    0.298,
    0.299,
    0.300
  };

  std::cout
      << "====================================================\n"
      << " Lite3 Single-Leg Stair Height Analyzer\n"
      << "====================================================\n\n";

  std::cout
      << std::fixed
      << std::setprecision(5);

  std::cout
      << "Leg             : FL\n"
      << "Tread depth     : "
      << TREAD_DEPTH
      << " m\n"
      << "Clearance       : "
      << CLEARANCE
      << " m\n"
      << "Foot radius     : "
      << FOOT_RADIUS
      << " m\n"
      << "Safety margin   : "
      << SAFETY_MARGIN
      << " rad\n"
      << "Pre-riser       : "
      << PRE_RISER_DISTANCE
      << " m\n"
      << "Landing offset  : "
      << LANDING_OFFSET
      << " m\n\n";

  std::cout
      << "Standing foot [m]\n"
      << "  X = "
      << p_stand.x()
      << "\n"
      << "  Y = "
      << p_stand.y()
      << "\n"
      << "  Z = "
      << p_stand.z()
      << "\n\n";

  std::cout
      << "====================================================\n"
      << " Height Sweep\n"
      << "====================================================\n";

  std::cout
      << "height [cm]   exact   safe   "
      << "min_safe [rad]   limiting joint\n";

  std::cout
      << "----------------------------------------------------\n";

  std::vector<StairTestResult> results;

  for (const double height : heights)
  {
    const StairTestResult result =
        testStair(
          kin,
          LEG,
          p_stand,
          height,
          TREAD_DEPTH,
          CLEARANCE,
          PRE_RISER_DISTANCE,
          LANDING_OFFSET,
          FOOT_RADIUS,
          SAFETY_MARGIN);

    results.push_back(result);

    std::cout
        << std::setw(8)
        << height * 100.0
        << "      "
        << std::setw(5)
        << (result.exact_feasible
              ? "PASS"
              : "FAIL")
        << "     "
        << std::setw(5)
        << (result.safe_feasible
              ? "PASS"
              : "FAIL")
        << "       ";

    if (std::isfinite(
          result.min_safe_margin))
    {
      std::cout
          << std::setw(10)
          << result.min_safe_margin;
    }
    else
    {
      std::cout
          << std::setw(10)
          << "N/A";
    }

    std::cout
        << "        "
        << result.limiting_joint
        << "\n";
  }

  /*
   * Detailed result for the benchmark 18.5 cm.
   */
  const std::size_t benchmark_index = 2;

  std::cout
      << "\n====================================================\n"
      << " Benchmark Detail: 18.5 cm\n"
      << "====================================================\n";

  printResultDetail(
      results[benchmark_index]);

  /*
   * Find highest SAFE-feasible height.
   */

  double first_safe_failure_height = -1.0;

  for (std::size_t i = 0;
        i < results.size();
        i++)
    {
        if (!results[i].safe_feasible)
        {
            first_safe_failure_height = heights[i];
            break;
        }
    }

  double highest_safe_height = -1.0;
  std::size_t highest_safe_index = 0;

  for (std::size_t i = 0;
       i < results.size();
       ++i)
  {
    if (results[i].safe_feasible)
    {
      highest_safe_height =
          heights[i];

      highest_safe_index =
          i;
    }
  }

  std::cout
      << "\n====================================================\n"
      << " Highest Tested SAFE Height\n"
      << "====================================================\n";

  if (highest_safe_height > 0.0)
  {
    std::cout
        << "Highest safe-tested height : "
        << highest_safe_height * 100.0
        << " cm\n";

    std::cout
        << "Minimum safe margin        : "
        << results[highest_safe_index].min_safe_margin
        << " rad\n";

    std::cout
        << "Limiting joint             : "
        << results[highest_safe_index].limiting_joint
        << "\n";

    std::cout
        << "\nFirst SAFE failure height :"
        << first_safe_failure_height * 100.00
        << " cm\n";
  }
  else
  {
    std::cout
        << "No tested height is SAFE.\n";
  }

  std::cout
      << "\n====================================================\n";

  return 0;
}