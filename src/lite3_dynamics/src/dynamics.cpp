#include "lite3_dynamics/dynamics.hpp"

#include <cmath>
#include <stdexcept>

namespace lite3_dynamics
{

namespace
{

constexpr double GRAVITY = 9.81;

Eigen::Vector3d negX()
{
  return Eigen::Vector3d(-1.0, 0.0, 0.0);
}

Eigen::Vector3d negY()
{
  return Eigen::Vector3d(0.0, -1.0, 0.0);
}

}  // namespace

Lite3SingleLegDynamics::Lite3SingleLegDynamics()
{
}

LegGeometry Lite3SingleLegDynamics::getGeometry(
  lite3_kinematics::Leg leg) const
{
  constexpr double THIGH = 0.20000;
  constexpr double SHANK = 0.21012;

  switch (leg)
  {
    case lite3_kinematics::Leg::FL:
      return {
        +0.1745,
        +0.0620,
        +0.09735,
        THIGH,
        SHANK
      };

    case lite3_kinematics::Leg::FR:
      return {
        +0.1745,
        -0.0620,
        -0.09735,
        THIGH,
        SHANK
      };

    case lite3_kinematics::Leg::HL:
      return {
        -0.1745,
        +0.0620,
        +0.09735,
        THIGH,
        SHANK
      };

    case lite3_kinematics::Leg::HR:
      return {
        -0.1745,
        -0.0620,
        -0.09735,
        THIGH,
        SHANK
      };

    default:
      throw std::runtime_error("Unknown Lite3 leg");
  }
}

Eigen::Vector3d Lite3SingleLegDynamics::torqueMassGravity(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &qddot) const
{
  const Eigen::Matrix3d M =
    massMatrix(leg, q);

  const Eigen::Vector3d G =
    gravityTorque(leg, q);

  return M * qddot + G;
}

LegMassProperties Lite3SingleLegDynamics::getMassProperties(
  lite3_kinematics::Leg leg) const
{
  LegMassProperties p;

  switch (leg)
  {
    case lite3_kinematics::Leg::FL:
      p.hipx = {
        0.550,
        Eigen::Vector3d(
          -0.0060100,
          -0.0066532,
           0.00034295),
        Eigen::Matrix3d::Zero()
      };

      p.hipx.inertia.diagonal() <<
        0.0003949,
        0.0004028,
        0.0004472;

      p.hipy = {
        0.860,
        Eigen::Vector3d(
          -0.0039245,
          -0.0146320,
          -0.0251460),
        Eigen::Matrix3d::Zero()
      };

      p.hipy.inertia.diagonal() <<
        0.005736,
        0.004960,
        0.001436;

      p.knee = {
        0.153,
        Eigen::Vector3d(
           0.0064794,
          -0.0000014535,
          -0.1215700),
        Eigen::Matrix3d::Zero()
      };

      p.knee.inertia.diagonal() <<
        0.00089039,
        0.00090672,
        0.000031266;

      p.foot = {
        0.020,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Zero()
      };
      break;

    case lite3_kinematics::Leg::FR:
      p.hipx = {
        0.550,
        Eigen::Vector3d(
          -0.0105790,
           0.0113580,
           0.00048546),
        Eigen::Matrix3d::Zero()
      };

      p.hipx.inertia.diagonal() <<
        0.0003949,
        0.0004028,
        0.0004472;

      p.hipy = {
        0.860,
        Eigen::Vector3d(
          -0.0039245,
           0.0146320,
          -0.0251460),
        Eigen::Matrix3d::Zero()
      };

      p.hipy.inertia.diagonal() <<
        0.005736,
        0.004960,
        0.001436;

      p.knee = {
        0.153,
        Eigen::Vector3d(
           0.0064794,
          -0.0000014535,
          -0.1215700),
        Eigen::Matrix3d::Zero()
      };

      p.knee.inertia.diagonal() <<
        0.00089039,
        0.00090672,
        0.000031266;

      p.foot = {
        0.020,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Zero()
      };
      break;

    case lite3_kinematics::Leg::HL:
      p.hipx = {
        0.550,
        Eigen::Vector3d(
           0.0109050,
          -0.0126360,
           0.0010510),
        Eigen::Matrix3d::Zero()
      };

      p.hipx.inertia.diagonal() <<
        0.0003949,
        0.0004028,
        0.0004472;

      p.hipy = {
        0.860,
        Eigen::Vector3d(
          -0.0039245,
          -0.0146320,
          -0.0251460),
        Eigen::Matrix3d::Zero()
      };

      p.hipy.inertia.diagonal() <<
        0.005736,
        0.004960,
        0.001436;

      p.knee = {
        0.153,
        Eigen::Vector3d(
           0.0064794,
          -0.0000014535,
          -0.1215700),
        Eigen::Matrix3d::Zero()
      };

      p.knee.inertia.diagonal() <<
        0.00089039,
        0.00090672,
        0.000031266;

      p.foot = {
        0.020,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Zero()
      };
      break;

    case lite3_kinematics::Leg::HR:
      p.hipx = {
        0.550,
        Eigen::Vector3d(
           0.0103540,
           0.0114230,
           0.00049498),
        Eigen::Matrix3d::Zero()
      };

      p.hipx.inertia.diagonal() <<
        0.0003949,
        0.0004028,
        0.0004472;

      p.hipy = {
        0.860,
        Eigen::Vector3d(
          -0.0039245,
           0.0146320,
          -0.0251460),
        Eigen::Matrix3d::Zero()
      };

      p.hipy.inertia.diagonal() <<
        0.005736,
        0.004960,
        0.001436;

      p.knee = {
        0.153,
        Eigen::Vector3d(
           0.0064794,
          -0.0000014535,
          -0.1215700),
        Eigen::Matrix3d::Zero()
      };

      p.knee.inertia.diagonal() <<
        0.00089039,
        0.00090672,
        0.000031266;

      p.foot = {
        0.020,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Zero()
      };
      break;

    default:
      throw std::runtime_error("Unknown Lite3 leg");
  }

  return p;
}

Eigen::Isometry3d Lite3SingleLegDynamics::hipXTransform(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);

  Eigen::Isometry3d T =
    Eigen::Isometry3d::Identity();

  T.translate(
    Eigen::Vector3d(
      g.hip_x,
      g.hip_y,
      0.0));

  T.rotate(
    Eigen::AngleAxisd(
      -q(0),
      Eigen::Vector3d::UnitX()));

  return T;
}

Eigen::Isometry3d Lite3SingleLegDynamics::hipYTransform(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);

  Eigen::Isometry3d T =
    hipXTransform(leg, q);

  T.translate(
    Eigen::Vector3d(
      0.0,
      g.hip_y_offset,
      0.0));

  T.rotate(
    Eigen::AngleAxisd(
      -q(1),
      Eigen::Vector3d::UnitY()));

  return T;
}

Eigen::Isometry3d Lite3SingleLegDynamics::kneeTransform(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);

  Eigen::Isometry3d T =
    hipYTransform(leg, q);

  T.translate(
    Eigen::Vector3d(
      0.0,
      0.0,
      -g.thigh_length));

  T.rotate(
    Eigen::AngleAxisd(
      -q(2),
      Eigen::Vector3d::UnitY()));

  return T;
}

Eigen::Isometry3d Lite3SingleLegDynamics::footTransform(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);

  Eigen::Isometry3d T =
    kneeTransform(leg, q);

  T.translate(
    Eigen::Vector3d(
      0.0,
      0.0,
      -g.shank_length));

  return T;
}

Eigen::Vector3d Lite3SingleLegDynamics::centerOfMass(
  lite3_kinematics::Leg leg,
  Link link,
  const Eigen::Vector3d &q) const
{
  const auto p = getMassProperties(leg);

  switch (link)
  {
    case Link::HIPX:
      return hipXTransform(leg, q) *
             p.hipx.com;

    case Link::HIPY:
      return hipYTransform(leg, q) *
             p.hipy.com;

    case Link::KNEE:
      return kneeTransform(leg, q) *
             p.knee.com;

    case Link::FOOT:
      return footTransform(leg, q) *
             p.foot.com;

    default:
      throw std::runtime_error("Unknown link");
  }
}

double Lite3SingleLegDynamics::potentialEnergy(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto p = getMassProperties(leg);

  const auto com_hipx =
    centerOfMass(leg, Link::HIPX, q);

  const auto com_hipy =
    centerOfMass(leg, Link::HIPY, q);

  const auto com_knee =
    centerOfMass(leg, Link::KNEE, q);

  const auto com_foot =
    centerOfMass(leg, Link::FOOT, q);

  return GRAVITY * (
    p.hipx.mass * com_hipx.z() +
    p.hipy.mass * com_hipy.z() +
    p.knee.mass * com_knee.z() +
    p.foot.mass * com_foot.z()
  );
}

Eigen::Vector3d Lite3SingleLegDynamics::gravityTorque(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);
  const auto p = getMassProperties(leg);

  const auto T_hipx = hipXTransform(leg, q);
  const auto T_hipy = hipYTransform(leg, q);
  const auto T_knee = kneeTransform(leg, q);
  const auto T_foot = footTransform(leg, q);

  const Eigen::Vector3d origins[3] = {
    Eigen::Vector3d(g.hip_x, g.hip_y, 0.0),

    T_hipx *
      Eigen::Vector3d(
        0.0,
        g.hip_y_offset,
        0.0),

    T_hipy *
      Eigen::Vector3d(
        0.0,
        0.0,
        -g.thigh_length)
  };

  const Eigen::Vector3d axes[3] = {
    negX(),
    T_hipx.rotation() * negY(),
    T_hipy.rotation() * negY()
  };

  const Eigen::Vector3d coms[4] = {
    T_hipx * p.hipx.com,
    T_hipy * p.hipy.com,
    T_knee * p.knee.com,
    T_foot * p.foot.com
  };

  const double masses[4] = {
    p.hipx.mass,
    p.hipy.mass,
    p.knee.mass,
    p.foot.mass
  };

  Eigen::Vector3d G =
    Eigen::Vector3d::Zero();

  for (int link = 0; link < 4; ++link)
  {
    for (int joint = 0; joint < 3; ++joint)
    {
      if (joint > link && link < 3)
      {
        continue;
      }

      const Eigen::Vector3d Jv =
        axes[joint].cross(
          coms[link] - origins[joint]);

      G(joint) +=
        masses[link] * GRAVITY * Jv.z();
    }
  }

  return G;
}

Eigen::Matrix3d Lite3SingleLegDynamics::massMatrix(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);
  const auto p = getMassProperties(leg);

  const auto T_hipx = hipXTransform(leg, q);
  const auto T_hipy = hipYTransform(leg, q);
  const auto T_knee = kneeTransform(leg, q);
  const auto T_foot = footTransform(leg, q);

  /*
   * Joint origins.
   */
  const Eigen::Vector3d origins[3] = {
    Eigen::Vector3d(g.hip_x, g.hip_y, 0.0),

    T_hipx *
      Eigen::Vector3d(
        0.0,
        g.hip_y_offset,
        0.0),

    T_hipy *
      Eigen::Vector3d(
        0.0,
        0.0,
        -g.thigh_length)
  };

  /*
   * Joint axes expressed in world frame.
   */
  const Eigen::Vector3d axes[3] = {
    negX(),
    T_hipx.rotation() * negY(),
    T_hipy.rotation() * negY()
  };

  /*
   * Link COM positions.
   */
  const Eigen::Vector3d coms[4] = {
    T_hipx * p.hipx.com,
    T_hipy * p.hipy.com,
    T_knee * p.knee.com,
    T_foot * p.foot.com
  };

  const double masses[4] = {
    p.hipx.mass,
    p.hipy.mass,
    p.knee.mass,
    p.foot.mass
  };

  /*
   * Link rotations.
   *
   * Used to transform body-frame inertia
   * into the world frame.
   */
  const Eigen::Matrix3d rotations[4] = {
    T_hipx.rotation(),
    T_hipy.rotation(),
    T_knee.rotation(),
    T_foot.rotation()
  };

  const Eigen::Matrix3d inertias_body[4] = {
    p.hipx.inertia,
    p.hipy.inertia,
    p.knee.inertia,
    p.foot.inertia
  };

  Eigen::Matrix3d M =
    Eigen::Matrix3d::Zero();

  for (int link = 0; link < 4; ++link)
  {
    Eigen::Matrix<double, 3, 3> Jv =
      Eigen::Matrix<double, 3, 3>::Zero();

    Eigen::Matrix<double, 3, 3> Jw =
      Eigen::Matrix<double, 3, 3>::Zero();

    /*
     * Each link is affected only by
     * preceding joints.
     */
    for (int joint = 0; joint < 3; ++joint)
    {
      if (joint > link && link < 3)
      {
        continue;
      }

      Jv.col(joint) =
        axes[joint].cross(
          coms[link] - origins[joint]);

      Jw.col(joint) =
        axes[joint];
    }

    /*
     * Inertia about COM in world frame:
     *
     * I_world = R I_body R^T
     */
    const Eigen::Matrix3d I_world =
      rotations[link] *
      inertias_body[link] *
      rotations[link].transpose();

    M +=
      masses[link] *
      Jv.transpose() *
      Jv;

    M +=
      Jw.transpose() *
      I_world *
      Jw;
  }

  /*
   * Remove tiny numerical asymmetry.
   */
  M = 0.5 * (M + M.transpose());

  return M;
}

double Lite3SingleLegDynamics::kineticEnergy(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &qdot) const
{
  const Eigen::Matrix3d M =
    massMatrix(leg, q);

  return 0.5 *
         qdot.transpose() *
         M *
         qdot;
}

  Eigen::Matrix3d Lite3SingleLegDynamics::coriolisMatrix(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &qdot) const
{
  /*
   * Christoffel-based construction:
   *
   * C_ij =
   *   1/2 sum_k (
   *     dM_ij/dq_k
   *     + dM_ik/dq_j
   *     - dM_jk/dq_i
   *   ) qdot_k
   *
   * M derivatives are evaluated using central finite difference.
   */

  constexpr double EPS = 1e-6;

  Eigen::Matrix3d dM_dq[3];

  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d qp = q;
    Eigen::Vector3d qm = q;

    qp(k) += EPS;
    qm(k) -= EPS;

    const Eigen::Matrix3d Mp =
      massMatrix(leg, qp);

    const Eigen::Matrix3d Mm =
      massMatrix(leg, qm);

    dM_dq[k] =
      (Mp - Mm) / (2.0 * EPS);
  }

  Eigen::Matrix3d C =
    Eigen::Matrix3d::Zero();

  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      for (int k = 0; k < 3; ++k)
      {
        const double christoffel =
          0.5 *
          (
            dM_dq[k](i, j)
            +
            dM_dq[j](i, k)
            -
            dM_dq[i](j, k)
          );

        C(i, j) +=
          christoffel * qdot(k);
      }
    }
  }

  return C;
}

Eigen::Vector3d Lite3SingleLegDynamics::coriolisTorque(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &qdot) const
{
  const Eigen::Matrix3d C =
    coriolisMatrix(
      leg,
      q,
      qdot);

  return C * qdot;
}

Eigen::Vector3d Lite3SingleLegDynamics::inverseDynamics(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &qdot,
  const Eigen::Vector3d &qddot) const
{
  const Eigen::Matrix3d M =
    massMatrix(leg, q);

  const Eigen::Vector3d Cqdot =
    coriolisTorque(
      leg,
      q,
      qdot);

  const Eigen::Vector3d G =
    gravityTorque(
      leg,
      q);

  return
    M * qddot +
    Cqdot +
    G;
}

Eigen::Matrix3d Lite3SingleLegDynamics::footJacobian(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q) const
{
  const auto g = getGeometry(leg);

  const auto T_hipx =
    hipXTransform(leg, q);

  const auto T_hipy =
    hipYTransform(leg, q);

  const auto T_knee =
    kneeTransform(leg, q);

  const auto T_foot =
    footTransform(leg, q);

  /*
   * Joint origins.
   */
  const Eigen::Vector3d origins[3] = {
    Eigen::Vector3d(
      g.hip_x,
      g.hip_y,
      0.0),

    T_hipx *
      Eigen::Vector3d(
        0.0,
        g.hip_y_offset,
        0.0),

    T_hipy *
      Eigen::Vector3d(
        0.0,
        0.0,
        -g.thigh_length)
  };

  /*
   * Joint axes in world frame.
   */
  const Eigen::Vector3d axes[3] = {
    negX(),
    T_hipx.rotation() * negY(),
    T_hipy.rotation() * negY()
  };

  /*
   * Foot reference point.
   */
  const Eigen::Vector3d p_foot =
    T_foot.translation();

  Eigen::Matrix3d J =
    Eigen::Matrix3d::Zero();

  for (int joint = 0; joint < 3; ++joint)
  {
    J.col(joint) =
      axes[joint].cross(
        p_foot - origins[joint]);
  }

  return J;
}

Eigen::Vector3d Lite3SingleLegDynamics::contactTorque(
  lite3_kinematics::Leg leg,
  const Eigen::Vector3d &q,
  const Eigen::Vector3d &force) const
{
  const Eigen::Matrix3d J =
    footJacobian(leg, q);

  return J.transpose() * force;
}

}  // namespace lite3_dynamics