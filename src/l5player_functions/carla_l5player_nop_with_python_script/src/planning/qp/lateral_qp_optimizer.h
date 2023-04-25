/*

*/
#pragma once
#include "piecewise_jerk_trajectory1d.h"
#include "plan_init.h"
#include "path_points.h"

class LateralQPOptimizer {
 public:
  LateralQPOptimizer() = default;

  virtual ~LateralQPOptimizer() = default;

  virtual bool optimize(
      const std::array<double, 3>& d_state, const double delta_s,
      const std::vector<std::pair<double, double>>& d_bounds) = 0;

  virtual PiecewiseJerkTrajectory1d GetOptimalTrajectory() const;
  virtual std::vector<FrenetFramePoint> GetFrenetFramePath() const;

 protected:
  double delta_s_ = Config_.FLAGS_default_delta_s_lateral_optimization;

  std::vector<double> opt_d_;

  std::vector<double> opt_d_prime_;

  std::vector<double> opt_d_pprime_;
};