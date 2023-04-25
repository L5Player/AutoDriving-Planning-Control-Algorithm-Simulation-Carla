#include "lateral_qp_optimizer.h"

PiecewiseJerkTrajectory1d LateralQPOptimizer::GetOptimalTrajectory() const
{
  if(!opt_d_.empty() && !opt_d_prime_.empty() && !opt_d_pprime_.empty()){

  }

  PiecewiseJerkTrajectory1d optimal_trajectory(opt_d_.front(), opt_d_prime_.front(), opt_d_pprime_.front());

  for (size_t i = 1; i < opt_d_.size(); ++i)
  {
    double j = (opt_d_pprime_[i] - opt_d_pprime_[i - 1]) / delta_s_;
    optimal_trajectory.AppendSegment(j, delta_s_);
  }
  return optimal_trajectory;
}

std::vector<FrenetFramePoint> LateralQPOptimizer::GetFrenetFramePath()
    const
{
  std::vector<FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  for (size_t i = 0; i < opt_d_.size(); ++i)
  {
    FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s);
    frenet_frame_point.set_l(opt_d_[i]);
    frenet_frame_point.set_dl(opt_d_prime_[i]);
    frenet_frame_point.set_ddl(opt_d_pprime_[i]);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    accumulated_s += delta_s_;
  }
  return frenet_frame_path;
}
