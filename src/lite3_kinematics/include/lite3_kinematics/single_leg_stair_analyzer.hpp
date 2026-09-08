#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lite3_kinematics/kinematics.hpp"

namespace lite3_kinematics {

struct StairParams {
  double riser_height_m;
  double tread_depth_m;
  double clearance_m = 0.04;
  double x_edge_frac = 0.35;
};

struct TrajectoryEvalResult {
  bool feasible = false;
  double worst_margin_rad = 0.0;

  Eigen::Vector3d worst_point_rel =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d worst_point_q =
      Eigen::Vector3d::Zero();

  double max_pos_err = 0.0;
};

class SingleLegStairAnalyzer {
 public:
  explicit SingleLegStairAnalyzer(
      const Lite3Kinematics& kinematics,
      Leg leg,
      const Eigen::Vector3d& p_stand,
      const Eigen::Vector3d& q_stand,
      double safety_margin_rad = 0.05,
      int samples_per_segment = 25);

  std::vector<Eigen::Vector3d> buildWaypoints(
      const StairParams& params,
      double dx) const;

  std::vector<Eigen::Vector3d> sampleTrajectory(
      const std::vector<Eigen::Vector3d>& waypoints) const;

  double jointMargin(
      const Eigen::Vector3d& q) const;

  TrajectoryEvalResult evaluate(
      const StairParams& params,
      double dx) const;

  std::vector<std::pair<double, double>>
  sweepForwardDisplacement(
      const StairParams& params,
      double dx_min,
      double dx_max,
      double dx_step) const;

  std::vector<std::pair<double, double>>
  boundaryCurve(
      double dx_min,
      double dx_max,
      double dx_step,
      double h_min,
      double h_max,
      double h_step,
      double tread_depth_m) const;

 private:
  const Lite3Kinematics& kinematics_;

  Leg leg_;

  Eigen::Vector3d p_stand_;
  Eigen::Vector3d q_stand_;

  double safety_margin_rad_;
  int samples_per_segment_;
};

}  // namespace lite3_kinematics