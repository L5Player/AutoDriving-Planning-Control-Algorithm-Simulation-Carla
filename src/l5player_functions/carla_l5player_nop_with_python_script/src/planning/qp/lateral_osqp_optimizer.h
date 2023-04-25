#pragma once

#include <utility>
#include <vector>
#include <osqp/osqp.h>
#include <osqp/glob_opts.h>
#include <osqp/cs.h>
#include <osqp/auxil.h>
#include <osqp/scaling.h>
#include <rclcpp/rclcpp.hpp>
#include "piecewise_jerk_trajectory1d.h"
#include "lateral_qp_optimizer.h"

class LateralOSQPOptimizer : public LateralQPOptimizer
{
public:
  LateralOSQPOptimizer() = default;

  virtual ~LateralOSQPOptimizer() = default;

  bool optimize(const std::array<double, 3> &d_state, const double delta_s,
                const std::vector<std::pair<double, double>> &d_bounds) override;

private:
  void CalculateKernel(const std::vector<std::pair<double, double>> &d_bounds, std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices, std::vector<c_int> *P_indptr);
  void CHECK_EQ(int size1, int size2);
  double delta_s_ = 0.0;
  ;
};