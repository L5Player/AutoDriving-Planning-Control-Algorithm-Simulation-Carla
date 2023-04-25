#include "piecewise_braking_trajectory_generator.h"

std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(const double s_target, const double s_curr,
                                                                       const double v_target, const double v_curr,
                                                                       const double a_comfort, const double d_comfort,
                                                                       const double max_time)
{
  std::shared_ptr<PiecewiseAccelerationTrajectory1d> ptr_trajectory =
      std::make_shared<PiecewiseAccelerationTrajectory1d>(s_curr, v_curr);

  double s_dist = s_target - s_curr;

  double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);

  // if cannot stop using comfort deceleration, then brake in the beginning.
  if (comfort_stop_dist > s_dist)
  {
    double stop_d = ComputeStopDeceleration(s_dist, v_curr);
    double stop_t = v_curr / stop_d;
    ptr_trajectory->AppendSegment(-stop_d, stop_t);

    if (ptr_trajectory->ParamLength() < max_time)
    {
      ptr_trajectory->AppendSegment(0.0, max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;
  }

  // otherwise, the vehicle can stop from current speed with comfort brake.
  if (v_curr > v_target)
  {
    // 匀减速到巡航速度
    double t_cruise = (s_dist - comfort_stop_dist) / v_target;
    double t_rampdown = (v_curr - v_target) / d_comfort;
    double t_dec = v_target / d_comfort;

    ptr_trajectory->AppendSegment(-d_comfort, t_rampdown);
    ptr_trajectory->AppendSegment(0.0, t_cruise);
    ptr_trajectory->AppendSegment(-d_comfort, t_dec);

    if (ptr_trajectory->ParamLength() < max_time)
    {
      ptr_trajectory->AppendSegment(0.0, max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;
  }
  else
  {
    // 车辆现在的速度小于巡航速度
    double t_rampup = (v_target - v_curr) / a_comfort;
    double t_rampdown = (v_target - v_curr) / d_comfort;
    double s_ramp = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5;

    double s_rest = s_dist - s_ramp - comfort_stop_dist;
    if (s_rest > 0)
    {
      // 可以做匀速运动
      double t_cruise = s_rest / v_target;
      double t_dec = v_target / d_comfort;

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_rampup);
      ptr_trajectory->AppendSegment(0.0, t_cruise);
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);

      if (ptr_trajectory->ParamLength() < max_time)
      {
        ptr_trajectory->AppendSegment(0.0, max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    }
    else
    {
      double s_rampup_rampdown = s_dist - comfort_stop_dist;
      double v_max =
          std::sqrt(v_curr * v_curr + 2.0 * a_comfort * d_comfort * s_rampup_rampdown / (a_comfort + d_comfort));

      double t_acc = (v_max - v_curr) / a_comfort;
      double t_dec = v_max / d_comfort;

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_acc);
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);

      if (ptr_trajectory->ParamLength() < max_time)
      {
        ptr_trajectory->AppendSegment(0.0, max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    }
  }
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDistance(const double v, const double dec)
{
  // ACHECK(dec > 0.0);
  return v * v / dec * 0.5;
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDeceleration(const double dist, const double v)
{
  return v * v / dist * 0.5;
}