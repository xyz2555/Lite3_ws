#ifndef LITE3_KINEMATICS_HPP_
#define LITE3_KINEMATICS_HPP_

#include <Eigen/Dense>

#include <string>

namespace lite3_kinematics
{

enum class Leg
{
  FL,
  FR,
  HL,
  HR
};

enum class IKStatus
{
  SUCCESS,
  GEOMETRICALLY_UNREACHABLE,
  JOINT_LIMIT_VIOLATION,
  INVALID_TARGET
};

struct JointLimits
{
  double hipx_min;
  double hipx_max;

  double hipy_min;
  double hipy_max;

  double knee_min;
  double knee_max;
};

struct IKResult
{
  bool success;
  IKStatus status;

  Eigen::Vector3d q;

  double position_error;
};

class Lite3Kinematics
{
public:

  Lite3Kinematics();

  /*
   * Forward Kinematics
   *
   * q:
   *   [HipX, HipY, Knee] in rad
   *
   * return:
   *   Foot position in TORSO frame [x,y,z] in meters
   */
  Eigen::Vector3d forward(
    Leg leg,
    const Eigen::Vector3d& q) const;

  /*
   * Inverse Kinematics
   *
   * target:
   *   desired foot position in TORSO frame
   *
   * seed:
   *   optional previous joint configuration.
   *   Used to choose the closest valid IK branch.
   */
  IKResult inverse(
    Leg leg,
    const Eigen::Vector3d& target,
    const Eigen::Vector3d* seed = nullptr) const;

  bool withinLimits(
    const Eigen::Vector3d& q) const;

  const JointLimits& limits() const
  {
    return limits_;
  }

private:

  struct LegGeometry
  {
    double hip_x;
    double hip_y;
    double hip_y_offset;

    double thigh_length;
    double shank_length;
  };

  LegGeometry geometry(Leg leg) const;

  JointLimits limits_;

  static double wrapToPi(double angle);

};

}  // namespace lite3_kinematics

#endif