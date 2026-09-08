#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <fstream>

using lite3_kinematics::JointLimits;
using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

// ============================================================
// Data structures
// ============================================================

struct JointMargin
{
  double hipx = 0.0;
  double hipy = 0.0;
  double knee = 0.0;
};


struct TrajectorySample
{
  double t = 0.0;

  Eigen::Vector3d p =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d q =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d qdot =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d qdot_fd =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d qddot_fd =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d pddot =
    Eigen::Vector3d::Zero();

Eigen::Vector3d qddot_diff =
    Eigen::Vector3d::Zero();

  double sigma_min = 0.0;
  double manipulability = 0.0;

  bool ik_valid = false;
};


struct TrajectoryMetrics
{
  bool ik_feasible = true;
  bool joint_safe_feasible = true;
  bool velocity_feasible = true;
  bool continuity_feasible = true;

  double min_joint_margin =
      std::numeric_limits<double>::infinity();

  double min_hipx_margin =
      std::numeric_limits<double>::infinity();

  double min_hipy_margin =
      std::numeric_limits<double>::infinity();

  double min_knee_margin =
      std::numeric_limits<double>::infinity();

  double max_qdot = 0.0;

  double max_qddot_diff = 0.0;

  double max_hipx_velocity = 0.0;
  double max_hipy_velocity = 0.0;
  double max_knee_velocity = 0.0;

  double max_qddot = 0.0;

  double max_hipx_acceleration = 0.0;
  double max_hipy_acceleration = 0.0;
  double max_knee_acceleration = 0.0;

  double max_qdot_difference = 0.0;
  double max_delta_q = 0.0;

  double min_sigma =
      std::numeric_limits<double>::infinity();

  double min_manipulability =
      std::numeric_limits<double>::infinity();

  double max_position_error = 0.0;

  std::string limiting_joint = "N/A";
  double limiting_s = 0.0;
};


// ============================================================
// Quintic time scaling
// ============================================================

double quintic(double u)
{
  return
      10.0 * std::pow(u, 3) -
      15.0 * std::pow(u, 4) +
       6.0 * std::pow(u, 5);
}


double quinticDerivative(double u)
{
  return
      30.0 * std::pow(u, 2) -
      60.0 * std::pow(u, 3) +
      30.0 * std::pow(u, 4);
}


double quinticSecondDerivative(double u)
{
  return
      60.0 * u -
      180.0 * std::pow(u, 2) +
      120.0 * std::pow(u, 3);
}


// ============================================================
// Joint margins
// ============================================================

JointMargin computeJointMargin(
    const Eigen::Vector3d& q,
    const JointLimits& lim)
{
  JointMargin m;

  m.hipx =
      std::min(
          q(0) - lim.hipx_min,
          lim.hipx_max - q(0));

  m.hipy =
      std::min(
          q(1) - lim.hipy_min,
          lim.hipy_max - q(1));

  m.knee =
      std::min(
          q(2) - lim.knee_min,
          lim.knee_max - q(2));

  return m;
}


// ============================================================
// Pseudoinverse
// ============================================================

Eigen::Matrix3d pseudoinverse(
    const Eigen::Matrix3d& J)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      J,
      Eigen::ComputeFullU |
      Eigen::ComputeFullV);

  const Eigen::Vector3d s =
      svd.singularValues();

  Eigen::Vector3d s_inv =
      Eigen::Vector3d::Zero();

  constexpr double EPS = 1e-9;

  for (int i = 0; i < 3; ++i)
  {
    if (s(i) > EPS)
    {
      s_inv(i) = 1.0 / s(i);
    }
  }

  return
      svd.matrixV() *
      s_inv.asDiagonal() *
      svd.matrixU().transpose();
}


// ============================================================
// Update trajectory metrics
// ============================================================

void updateMetrics(
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& qdot,
    const Eigen::Matrix3d& J,
    double position_error,
    double global_s,
    const JointLimits& lim,
    double safety_margin,
    TrajectoryMetrics& metrics)
{
  const JointMargin exact =
      computeJointMargin(q, lim);

  const double hipx_safe =
      exact.hipx - safety_margin;

  const double hipy_safe =
      exact.hipy - safety_margin;

  const double knee_safe =
      exact.knee - safety_margin;

  const double safe_min =
      std::min({
          hipx_safe,
          hipy_safe,
          knee_safe
      });

  metrics.min_joint_margin =
      std::min(
          metrics.min_joint_margin,
          safe_min);

  metrics.min_hipx_margin =
      std::min(
          metrics.min_hipx_margin,
          hipx_safe);

  metrics.min_hipy_margin =
      std::min(
          metrics.min_hipy_margin,
          hipy_safe);

  metrics.min_knee_margin =
      std::min(
          metrics.min_knee_margin,
          knee_safe);

  if (safe_min < 0.0)
  {
    metrics.joint_safe_feasible = false;
  }

  // Velocity
  metrics.max_hipx_velocity =
      std::max(
          metrics.max_hipx_velocity,
          std::abs(qdot(0)));

  metrics.max_hipy_velocity =
      std::max(
          metrics.max_hipy_velocity,
          std::abs(qdot(1)));

  metrics.max_knee_velocity =
      std::max(
          metrics.max_knee_velocity,
          std::abs(qdot(2)));

  metrics.max_qdot =
      std::max({
          metrics.max_qdot,
          std::abs(qdot(0)),
          std::abs(qdot(1)),
          std::abs(qdot(2))
      });

  // Jacobian singular values
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      J,
      Eigen::ComputeFullU |
      Eigen::ComputeFullV);

  const Eigen::Vector3d singular =
      svd.singularValues();

  metrics.min_sigma =
      std::min(
          metrics.min_sigma,
          singular.minCoeff());

  metrics.min_manipulability =
      std::min(
          metrics.min_manipulability,
          singular.prod());

  metrics.max_position_error =
      std::max(
          metrics.max_position_error,
          position_error);

  // Determine limiting joint.
  double local_min = hipx_safe;
  std::string local_joint = "HipX";

  if (hipy_safe < local_min)
  {
    local_min = hipy_safe;
    local_joint = "HipY";
  }

  if (knee_safe < local_min)
  {
    local_min = knee_safe;
    local_joint = "Knee";
  }

  if (local_min <= metrics.min_joint_margin + 1e-12)
  {
    metrics.limiting_joint = local_joint;
    metrics.limiting_s = global_s;
  }
}


// ============================================================
// Generate one smooth trajectory segment
// ============================================================

bool generateSegment(
    Lite3Kinematics& kin,
    Leg leg,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double t_start,
    double duration,
    double total_duration,
    int samples,
    bool include_first,
    Eigen::Vector3d& q_seed,
    std::vector<TrajectorySample>& trajectory,
    TrajectoryMetrics& metrics)
{
  const Eigen::Vector3d dp =
      p1 - p0;

  for (int i = 0;
       i <= samples;
       ++i)
  {
    if (!include_first && i == 0)
    {
      continue;
    }

    const double u =
        static_cast<double>(i) /
        static_cast<double>(samples);

    const double t_local =
        u * duration;

    const double s =
        quintic(u);

    const double ds_du =
        quinticDerivative(u);

    const double ds_dt =
        ds_du / duration;

    const double d2s_du2 =
    quinticSecondDerivative(u);

const double d2s_dt2 =
    d2s_du2 /
    (duration * duration);

    const Eigen::Vector3d p =
        p0 + s * dp;

    const Eigen::Vector3d p_dot =
        ds_dt * dp;

    const Eigen::Vector3d p_ddot =
    d2s_dt2 * dp;

    const auto ik =
        kin.inverse(
            leg,
            p,
            &q_seed);

    TrajectorySample sample;

    sample.t =
        t_start + t_local;

    sample.p =
        p;

    sample.pddot = 
        p_ddot;

    if (!ik.success)
    {
      sample.ik_valid = false;

      metrics.ik_feasible = false;

      trajectory.push_back(sample);

      return false;
    }

    sample.ik_valid = true;

    sample.q =
        ik.q;

    q_seed =
        ik.q;

    const Eigen::Matrix3d J =
        kin.jacobian(
            leg,
            sample.q);

    const Eigen::Matrix3d J_pinv =
        pseudoinverse(J);

    sample.qdot =
        J_pinv * p_dot;

    const double global_s =
        sample.t / total_duration;

    updateMetrics(
        sample.q,
        sample.qdot,
        J,
        ik.position_error,
        global_s,
        kin.limits(),
        0.05,
        metrics);

    trajectory.push_back(
        sample);
  }

  return true;
}


// ============================================================
// Finite-difference velocity and acceleration
// ============================================================

void calculateFiniteDifferences(
    std::vector<TrajectorySample>& trajectory,
    TrajectoryMetrics& metrics)
{
  if (trajectory.size() < 3)
  {
    return;
  }

  /*
   * qdot
   */
  for (std::size_t i = 1;
       i + 1 < trajectory.size();
       ++i)
  {
    const double dt =
        trajectory[i + 1].t -
        trajectory[i - 1].t;

    if (dt <= 0.0)
    {
      continue;
    }

    trajectory[i].qdot_fd =
        (trajectory[i + 1].q -
         trajectory[i - 1].q)
        / dt;

    const Eigen::Vector3d difference =
        trajectory[i].qdot_fd -
        trajectory[i].qdot;

    metrics.max_qdot_difference =
        std::max(
            metrics.max_qdot_difference,
            difference.norm());

    const Eigen::Vector3d dq =
        trajectory[i].q -
        trajectory[i - 1].q;

    metrics.max_delta_q =
        std::max(
            metrics.max_delta_q,
            dq.cwiseAbs().maxCoeff());
  }

  /*
   * qddot
   */
  for (std::size_t i = 1;
       i + 1 < trajectory.size();
       ++i)
  {
    const double dt_forward =
        trajectory[i + 1].t -
        trajectory[i].t;

    const double dt_backward =
        trajectory[i].t -
        trajectory[i - 1].t;

    if (dt_forward <= 0.0 ||
        dt_backward <= 0.0)
    {
      continue;
    }

    /*
     * Current sampling is uniform.
     */
    if (std::abs(
          dt_forward -
          dt_backward) > 1e-9)
    {
      continue;
    }

    const double dt2 =
        dt_forward * dt_forward;

    trajectory[i].qddot_fd =
        (trajectory[i + 1].q -
         2.0 * trajectory[i].q +
         trajectory[i - 1].q)
        / dt2;

    const double hipx =
        std::abs(
            trajectory[i].qddot_fd(0));

    const double hipy =
        std::abs(
            trajectory[i].qddot_fd(1));

    const double knee =
        std::abs(
            trajectory[i].qddot_fd(2));

    metrics.max_hipx_acceleration =
        std::max(
            metrics.max_hipx_acceleration,
            hipx);

    metrics.max_hipy_acceleration =
        std::max(
            metrics.max_hipy_acceleration,
            hipy);

    metrics.max_knee_acceleration =
        std::max(
            metrics.max_knee_acceleration,
            knee);

    metrics.max_qddot =
        std::max({
            metrics.max_qddot,
            hipx,
            hipy,
            knee
        });
  }
}

void validateDifferentialAcceleration(
    std::vector<TrajectorySample>& trajectory,
    Lite3Kinematics& kin,
    Leg leg,
    TrajectoryMetrics& metrics)
{
  if (trajectory.size() < 3)
  {
    return;
  }

  for (std::size_t i = 1;
       i + 1 < trajectory.size();
       ++i)
  {
    const double dt_forward =
        trajectory[i + 1].t -
        trajectory[i].t;

    const double dt_backward =
        trajectory[i].t -
        trajectory[i - 1].t;

    /*
     * Current finite-difference implementation assumes
     * uniform time spacing.
     */
    if (dt_forward <= 0.0 ||
        dt_backward <= 0.0)
    {
      continue;
    }

    if (std::abs(
          dt_forward -
          dt_backward) > 1e-9)
    {
      continue;
    }

    const Eigen::Matrix3d J =
        kin.jacobian(
            leg,
            trajectory[i].q);

    /*
     * Numerical Jdot:
     *
     * Jdot =
     * (J[i+1] - J[i-1]) / (2 dt)
     */
    const Eigen::Matrix3d J_plus =
        kin.jacobian(
            leg,
            trajectory[i + 1].q);

    const Eigen::Matrix3d J_minus =
        kin.jacobian(
            leg,
            trajectory[i - 1].q);

    const double dt =
        dt_forward;

    const Eigen::Matrix3d Jdot =
        (J_plus - J_minus) /
        (2.0 * dt);

    /*
     * Differential kinematics:
     *
     * pddot = J qddot + Jdot qdot
     *
     * therefore:
     *
     * qddot =
     * J^+ (pddot - Jdot qdot)
     */
    const Eigen::Matrix3d J_pinv =
        pseudoinverse(J);

    const Eigen::Vector3d qddot_diff =
        J_pinv *
        (
          trajectory[i].pddot -
          Jdot *
          trajectory[i].qdot
        );

    trajectory[i].qddot_diff =
        qddot_diff;

    const Eigen::Vector3d difference =
        trajectory[i].qddot_fd -
        qddot_diff;

    metrics.max_qddot_diff =
        std::max(
            metrics.max_qddot_diff,
            difference.norm());
  }
}


// ============================================================
// Main
// ============================================================

}  // namespace

void saveTrajectoryCSV(
    const std::vector<TrajectorySample>& trajectory,
    const std::string& filename)
{
  std::ofstream file(filename);

  if (!file.is_open())
  {
    std::cerr
        << "Failed to open CSV: "
        << filename
        << "\n";

    return;
  }

  file
      << "t,"
      << "x,y,z,"
      << "qx,qy,qknee,"
      << "qdot_x,qdot_y,qdot_knee,"
      << "qddot_x,qddot_y,qddot_knee,"
      << "qddot_diff_x,qddot_diff_y,qddot_diff_knee,"
      << "pddot_x,pddot_y,pddot_z,";
    //   << "sigma_min,"
    //   << "manipulability\n";

  file << std::setprecision(12);

  for (const auto& sample :
       trajectory)
  {
    file
        << sample.t << ","

        << sample.p.x() << ","
        << sample.p.y() << ","
        << sample.p.z() << ","

        << sample.q(0) << ","
        << sample.q(1) << ","
        << sample.q(2) << ","

        << sample.qdot(0) << ","
        << sample.qdot(1) << ","
        << sample.qdot(2) << ","

        << sample.qddot_fd(0) << ","
        << sample.qddot_fd(1) << ","
        << sample.qddot_fd(2) << ","

        << sample.qddot_diff(0) << ","
        << sample.qddot_diff(1) << ","
        << sample.qddot_diff(2) << ","

        << sample.pddot.x() << ","
        << sample.pddot.y() << ","
        << sample.pddot.z() << ","

        // << sample.sigma_min << ","
        // << sample.manipulability

        << "\n";
  }

  file.close();

  std::cout
      << "Trajectory saved: "
      << filename
      << "\n";
}

int main()
{
  Lite3Kinematics kin;

  constexpr Leg LEG =
      Leg::FL;

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

  constexpr double PRE_RISER =
      0.125;

  constexpr double LANDING_OFFSET =
      0.050;

  constexpr double SAFETY_MARGIN =
      0.050;

  /*
   * Segment duration ratios.
   *
   * These correspond to the original
   * 0.20/0.30/0.20/0.30/0.20 s trajectory.
   */
  constexpr double BASE_T1 = 0.20;
  constexpr double BASE_T2 = 0.30;
  constexpr double BASE_T3 = 0.20;
  constexpr double BASE_T4 = 0.30;
  constexpr double BASE_T5 = 0.20;

  constexpr double BASE_TOTAL =
      BASE_T1 +
      BASE_T2 +
      BASE_T3 +
      BASE_T4 +
      BASE_T5;

  /*
   * Time scaling experiment.
   */
  const std::vector<double> durations = {
      0.60,
      0.80,
      1.00,
      1.20,
      1.50,
      2.00
  };

  /*
   * Stair heights of interest.
   */
  const std::vector<double> heights = {
      0.185,
      0.200,
      0.215
  };

  constexpr int SAMPLES_PER_SEGMENT =
      100;

  constexpr double HIPX_VEL_LIMIT =
      26.2;

  constexpr double HIPY_VEL_LIMIT =
      26.2;

  constexpr double KNEE_VEL_LIMIT =
      17.3;

  std::cout
      << "====================================================\n"
      << " Lite3 Smooth Stair Trajectory + Time Scaling\n"
      << "====================================================\n\n";

  std::cout
      << std::fixed
      << std::setprecision(6);

  std::cout
      << "Leg             : FL\n"
      << "Tread depth     : "
      << TREAD_DEPTH
      << " m\n"
      << "Clearance       : "
      << CLEARANCE
      << " m\n"
      << "Pre-riser       : "
      << PRE_RISER
      << " m\n"
      << "Landing offset  : "
      << LANDING_OFFSET
      << " m\n"
      << "Safety margin   : "
      << SAFETY_MARGIN
      << " rad\n\n";

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

  for (const double step_height :
       heights)
  {
    std::cout
        << "====================================================\n"
        << " Step height = "
        << step_height * 100.0
        << " cm\n"
        << "====================================================\n";

    for (const double total_duration :
         durations)
    {
      /*
       * Scale each segment proportionally.
       */
      const double scale =
          total_duration / BASE_TOTAL;

      const double T1 =
          BASE_T1 * scale;

      const double T2 =
          BASE_T2 * scale;

      const double T3 =
          BASE_T3 * scale;

      const double T4 =
          BASE_T4 * scale;

      const double T5 =
          BASE_T5 * scale;

      /*
       * Stair geometry.
       */
      const double x_riser =
          p_stand.x() +
          PRE_RISER;

      const double x_land =
          x_riser +
          LANDING_OFFSET;

      const double z_ground =
          p_stand.z() -
          FOOT_RADIUS;

      const double z_tread =
          z_ground +
          step_height;

      const double z_clear =
          z_tread +
          FOOT_RADIUS +
          CLEARANCE;

      const double z_land =
          z_tread +
          FOOT_RADIUS;

      /*
       * Waypoints.
       */
      const Eigen::Vector3d P0 =
          p_stand;

      const Eigen::Vector3d P1(
          p_stand.x(),
          p_stand.y(),
          p_stand.z() +
          CLEARANCE);

      const Eigen::Vector3d P2(
          x_riser -
          FOOT_RADIUS,
          p_stand.y(),
          z_clear);

      const Eigen::Vector3d P3(
          x_riser +
          FOOT_RADIUS,
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

      std::vector<TrajectorySample>
          trajectory;

      trajectory.reserve(
          5 *
          (SAMPLES_PER_SEGMENT + 1));

      TrajectoryMetrics metrics;

      Eigen::Vector3d q_seed =
          q_stand;

      double t0 = 0.0;

      bool success = true;

      /*
       * P0 -> P1
       */
      success &=
          generateSegment(
              kin,
              LEG,
              P0,
              P1,
              t0,
              T1,
              total_duration,
              SAMPLES_PER_SEGMENT,
              true,
              q_seed,
              trajectory,
              metrics);

      t0 += T1;

      /*
       * P1 -> P2
       */
      if (success)
      {
        success &=
            generateSegment(
                kin,
                LEG,
                P1,
                P2,
                t0,
                T2,
                total_duration,
                SAMPLES_PER_SEGMENT,
                false,
                q_seed,
                trajectory,
                metrics);
      }

      t0 += T2;

      /*
       * P2 -> P3
       */
      if (success)
      {
        success &=
            generateSegment(
                kin,
                LEG,
                P2,
                P3,
                t0,
                T3,
                total_duration,
                SAMPLES_PER_SEGMENT,
                false,
                q_seed,
                trajectory,
                metrics);
      }

      t0 += T3;

      /*
       * P3 -> P4
       */
      if (success)
      {
        success &=
            generateSegment(
                kin,
                LEG,
                P3,
                P4,
                t0,
                T4,
                total_duration,
                SAMPLES_PER_SEGMENT,
                false,
                q_seed,
                trajectory,
                metrics);
      }

      t0 += T4;

      /*
       * P4 -> P5
       */
      if (success)
      {
        success &=
            generateSegment(
                kin,
                LEG,
                P4,
                P5,
                t0,
                T5,
                total_duration,
                SAMPLES_PER_SEGMENT,
                false,
                q_seed,
                trajectory,
                metrics);
      }

      if (!success)
      {
        metrics.ik_feasible = false;
      }

      /*
       * Numerical qdot and qddot.
       */
      if (metrics.ik_feasible)
      {
        calculateFiniteDifferences(
            trajectory,
            metrics);

        validateDifferentialAcceleration(
            trajectory,
            kin,
            LEG,
            metrics
        );
      }

      /*
       * Velocity limit check.
       */
      metrics.velocity_feasible =
          metrics.max_hipx_velocity <=
              HIPX_VEL_LIMIT &&
          metrics.max_hipy_velocity <=
              HIPY_VEL_LIMIT &&
          metrics.max_knee_velocity <=
              KNEE_VEL_LIMIT;

      const bool overall =
          metrics.ik_feasible &&
          metrics.joint_safe_feasible &&
          metrics.velocity_feasible &&
          metrics.continuity_feasible;

      /*
       * ------------------------------------------------------
       * Output
       * ------------------------------------------------------
       */
      std::cout
          << "\nT = "
          << total_duration
          << " s\n";

      std::cout
          << "  IK feasible       : "
          << (metrics.ik_feasible
                ? "YES"
                : "NO")
          << "\n";

      std::cout
          << "  Joint safe        : "
          << (metrics.joint_safe_feasible
                ? "YES"
                : "NO")
          << "\n";

      std::cout
          << "  Velocity feasible : "
          << (metrics.velocity_feasible
                ? "YES"
                : "NO")
          << "\n";

      std::cout
          << "  Continuity        : "
          << (metrics.continuity_feasible
                ? "YES"
                : "NO")
          << "\n";

      std::cout
          << "  Overall           : "
          << (overall
                ? "PASS"
                : "FAIL")
          << "\n";

      std::cout
          << "  Min joint margin  : "
          << metrics.min_joint_margin
          << " rad\n";

      std::cout
          << "    HipX            : "
          << metrics.min_hipx_margin
          << " rad\n";

      std::cout
          << "    HipY            : "
          << metrics.min_hipy_margin
          << " rad\n";

      std::cout
          << "    Knee            : "
          << metrics.min_knee_margin
          << " rad\n";

      std::cout
          << "  Max |qdot|        : "
          << metrics.max_qdot
          << " rad/s\n";

      std::cout
          << "  Max |qddot|       : "
          << metrics.max_qddot
          << " rad/s^2\n";
      
      std::cout
    << "  Max qddot cross-check error : "
    << metrics.max_qddot_diff
    << " rad/s^2\n";
    
      std::cout
          << "    HipX            : "
          << metrics.max_hipx_acceleration
          << " rad/s^2\n";

      std::cout
          << "    HipY            : "
          << metrics.max_hipy_acceleration
          << " rad/s^2\n";

      std::cout
          << "    Knee            : "
          << metrics.max_knee_acceleration
          << " rad/s^2\n";

      std::cout
          << "  Max qdot error    : "
          << metrics.max_qdot_difference
          << " rad/s\n";

      std::cout
          << "  Max |dq| sample   : "
          << metrics.max_delta_q
          << " rad\n";

      std::cout
          << "  Min sigma         : "
          << metrics.min_sigma
          << "\n";

      std::cout
          << "  Min manipulability: "
          << metrics.min_manipulability
          << "\n";

      std::cout
          << "  Max IK error      : "
          << std::scientific
          << metrics.max_position_error
          << std::fixed
          << " m\n";

      {
  const int height_mm =
      static_cast<int>(
          std::round(
              step_height * 1000.0));

  const int duration_tenths =
      static_cast<int>(
          std::round(
              total_duration * 10.0));

  const int duration_seconds =
      duration_tenths / 10;

  const int duration_decimal =
      duration_tenths % 10;

  std::string duration_string;

  if (duration_decimal == 0)
  {
    duration_string =
        std::to_string(duration_seconds) + "p0s";
  }
  else
  {
    duration_string =
        std::to_string(duration_seconds) +
        "p" +
        std::to_string(duration_decimal) +
        "s";
  }

  const std::string filename =
      "lite3_FL_stair_" +
      std::to_string(height_mm) +
      "mm_T" +
      duration_string +
      ".csv";

  saveTrajectoryCSV(
      trajectory,
      filename);
}
    }
  }

  std::cout
      << "\n====================================================\n"
      << " Analysis complete\n"
      << "====================================================\n";

  return 0;
}