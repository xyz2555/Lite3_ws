#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lite3_kinematics
{

namespace
{

constexpr double PI = 3.14159265358979323846;

constexpr double THIGH_LENGTH = 0.20;
constexpr double SHANK_LENGTH = 0.21012;

constexpr double HIP_X_OFFSET = 0.1745;
constexpr double HIP_Y_X_OFFSET = 0.062;
constexpr double HIP_Y_LOCAL_OFFSET = 0.09735;

constexpr double EPS = 1e-10;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace


Lite3Kinematics::Lite3Kinematics()
{
  /*
   * Lite3 URDF joint limits
   */

  limits_.hipx_min = -0.523;
  limits_.hipx_max =  0.523;

  limits_.hipy_min = -2.67;
  limits_.hipy_max =  0.314;

  limits_.knee_min =  0.524;
  limits_.knee_max =  2.792;
}


Lite3Kinematics::LegGeometry
Lite3Kinematics::geometry(Leg leg) const
{
  LegGeometry g;

  g.thigh_length = THIGH_LENGTH;
  g.shank_length = SHANK_LENGTH;

  /*
   * These are the transforms from TORSO to the
   * HipX joint obtained from the Lite3 URDF.
   */

  switch (leg)
  {
    case Leg::FL:
      g.hip_x = +HIP_X_OFFSET;
      g.hip_y = +HIP_Y_X_OFFSET;
      g.hip_y_offset = +HIP_Y_LOCAL_OFFSET;
      break;

    case Leg::FR:
      g.hip_x = +HIP_X_OFFSET;
      g.hip_y = -HIP_Y_X_OFFSET;
      g.hip_y_offset = -HIP_Y_LOCAL_OFFSET;
      break;

    case Leg::HL:
      g.hip_x = -HIP_X_OFFSET;
      g.hip_y = +HIP_Y_X_OFFSET;
      g.hip_y_offset = +HIP_Y_LOCAL_OFFSET;
      break;

    case Leg::HR:
      g.hip_x = -HIP_X_OFFSET;
      g.hip_y = -HIP_Y_X_OFFSET;
      g.hip_y_offset = -HIP_Y_LOCAL_OFFSET;
      break;
  }

  return g;
}


Eigen::Vector3d
Lite3Kinematics::forward(
  Leg leg,
  const Eigen::Vector3d& q) const
{
  const auto g = geometry(leg);

  const double q_hipx = q(0);
  const double q_hipy = q(1);
  const double q_knee = q(2);

  /*
   * The FK is deliberately implemented as the same
   * transformation sequence represented by the URDF.
   *
   * TORSO
   *   -> HipX origin
   *   -> rotation around -X
   *   -> HipY origin
   *   -> rotation around -Y
   *   -> thigh translation
   *   -> rotation around -Y
   *   -> shank translation
   */

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  T.translate(
    Eigen::Vector3d(
      g.hip_x,
      g.hip_y,
      0.0));

  T.rotate(
    Eigen::AngleAxisd(
      -q_hipx,
      Eigen::Vector3d::UnitX()));

  T.translate(
    Eigen::Vector3d(
      0.0,
      g.hip_y_offset,
      0.0));

  T.rotate(
    Eigen::AngleAxisd(
      -q_hipy,
      Eigen::Vector3d::UnitY()));

  T.translate(
    Eigen::Vector3d(
      0.0,
      0.0,
      -g.thigh_length));

  T.rotate(
    Eigen::AngleAxisd(
      -q_knee,
      Eigen::Vector3d::UnitY()));

  T.translate(
    Eigen::Vector3d(
      0.0,
      0.0,
      -g.shank_length));

  return T.translation();
}


bool
Lite3Kinematics::withinLimits(
  const Eigen::Vector3d& q) const
{
  return
    q(0) >= limits_.hipx_min &&
    q(0) <= limits_.hipx_max &&

    q(1) >= limits_.hipy_min &&
    q(1) <= limits_.hipy_max &&

    q(2) >= limits_.knee_min &&
    q(2) <= limits_.knee_max;
}


double
Lite3Kinematics::wrapToPi(double angle)
{
  while (angle > PI)
  {
    angle -= 2.0 * PI;
  }

  while (angle < -PI)
  {
    angle += 2.0 * PI;
  }

  return angle;
}


IKResult
Lite3Kinematics::inverse(
  Leg leg,
  const Eigen::Vector3d& target,
  const Eigen::Vector3d* seed) const
{
  IKResult result;

  result.success = false;
  result.status = IKStatus::INVALID_TARGET;
  result.q.setZero();
  result.position_error =
    std::numeric_limits<double>::infinity();

  if (!target.allFinite())
  {
    return result;
  }

  const auto g = geometry(leg);

  /*
   * Remove the HipX/Torso offset in X.
   */
  const double a =
    target.x() - g.hip_x;

  /*
   * y_h is the position of the HipY joint after
   * the HipX origin offset is included.
   */
  // const double y_h =
  //   g.hip_y + g.hip_y_offset;

  // /*
  //  * The HipX rotation acts on the Y-Z plane.
  //  *
  //  * r^2 = y^2 + z^2
  //  *
  //  * r^2 = y_h^2 + b^2
  //  */
  // const double yz_sq =
  //   target.y() * target.y() +
  //   target.z() * target.z();

  // const double b_sq =
  //   yz_sq - y_h * y_h;

  /*
 * g.hip_y is the static TORSO -> HipX Y offset.
 *
 * g.hip_y_offset is the HipX -> HipY Y offset.
 * The latter rotates with HipX.
 *
 * Therefore the inverse problem uses:
 *
 * Y = target.y() - g.hip_y
 *
 * and
 *
 * Y = d*cos(q1) + b*sin(q1)
 * Z = -d*sin(q1) + b*cos(q1)
 */

const double Y =
    target.y() - g.hip_y;

const double d =
    g.hip_y_offset;

const double yz_sq =
    Y * Y +
    target.z() * target.z();

const double b_sq =
    yz_sq - d * d;

  if (b_sq < -EPS)
  {
    result.status =
      IKStatus::GEOMETRICALLY_UNREACHABLE;

    return result;
  }

  const double b_abs =
    std::sqrt(std::max(0.0, b_sq));

  /*
   * There are two possible b branches.
   *
   * We must test both because the complete Lite3
   * HipY joint range permits both signs of b.
   */
  const double b_candidates[2] =
  {
    -b_abs,
    +b_abs
  };

  bool found = false;
  double best_cost =
    std::numeric_limits<double>::infinity();

  Eigen::Vector3d best_q;

  for (const double b : b_candidates)
  {
    /*
     * Planar two-link geometry:
     *
     * a = L1 sin(q2) + L2 sin(q2 + q3)
     * b = -L1 cos(q2) - L2 cos(q2 + q3)
     */
    const double v = -b;

    const double D =
      (
        a * a +
        v * v -
        g.thigh_length * g.thigh_length -
        g.shank_length * g.shank_length
      )
      /
      (
        2.0 *
        g.thigh_length *
        g.shank_length
      );

    if (D < -1.0 - EPS || D > 1.0 + EPS)
    {
      continue;
    }

    const double D_clamped =
      clamp(D, -1.0, 1.0);

    /*
     * Lite3 Knee is constrained to positive angles,
     * so use the positive acos branch.
     */
    const double q3 =
      std::acos(D_clamped);

    const double q2 =
      std::atan2(a, v)
      -
      std::atan2(
        g.shank_length * std::sin(q3),
        g.thigh_length +
          g.shank_length * std::cos(q3));

    /*
     * Recover HipX from the rotation in Y-Z.
     */
    // const double q1 =
    //   wrapToPi(
    //     std::atan2(b, y_h)
    //     -
    //     std::atan2(target.z(), target.y()));

    const double q1 =
    wrapToPi(
      std::atan2(b, d)
      -
      std::atan2(target.z(), Y));

    Eigen::Vector3d candidate_q;

    candidate_q <<
      q1,
      q2,
      q3;

    if (!withinLimits(candidate_q))
    {
      continue;
    }

    /*
     * Check actual reconstruction.
     */
    const Eigen::Vector3d reconstructed =
      forward(leg, candidate_q);

    const double position_error =
      (reconstructed - target).norm();

    /*
     * If a seed is available, prefer the IK solution
     * closest to the previous configuration.
     */
    double cost = position_error;

    if (seed != nullptr)
    {
      const Eigen::Vector3d dq =
        candidate_q - *seed;

      cost += 1e-3 * dq.squaredNorm();
    }

    if (cost < best_cost)
    {
      best_cost = cost;
      best_q = candidate_q;
      result.position_error = position_error;

      found = true;
    }
  }

  if (!found)
  {
    result.status =
      IKStatus::JOINT_LIMIT_VIOLATION;

    return result;
  }

  result.success = true;
  result.status = IKStatus::SUCCESS;
  result.q = best_q;

  return result;
}

}  // namespace lite3_kinematics