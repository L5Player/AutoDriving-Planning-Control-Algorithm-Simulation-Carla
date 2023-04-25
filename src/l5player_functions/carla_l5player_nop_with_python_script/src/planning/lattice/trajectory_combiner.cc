#include "trajectory_combiner.h"

TrajectoryCombiner::TrajectoryCombiner()
{
}
TrajectoryCombiner::~TrajectoryCombiner()
{
}
DiscretizedTrajectory TrajectoryCombiner::Combine(const std::vector<double> &accumulated_s,
                                                  const Curve1d &lon_trajectory, const Curve1d &lat_trajectory,
                                                  const std::vector<ReferencePoint> &reference_points,
                                                  const double &init_relative_time, const double &plan_start_time)
{
  DiscretizedTrajectory combined_trajectory;

  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = accumulated_s.back();
  double accumulated_trajectory_s = 0.0;
  PathPoint prev_trajectory_point;

  double last_s = -Config_.FLAGS_numerical_epsilon;
  double t_param = 0.0;
  while (t_param < Config_.FLAGS_trajectory_time_length)
  {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    if (last_s > 0.0)
    {
      s = std::max(last_s, s);
    }
    last_s = s;
    double s_dot = std::max(1e-6, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);
    if (s > s_ref_max)
    {
      break;
    }
    double relative_s = s - s0;
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    if (relative_s > lat_trajectory.ParamLength())
    {
      relative_s = lat_trajectory.ParamLength();
    }
    double d = lat_trajectory.Evaluate(0, relative_s);
    double d_prime = lat_trajectory.Evaluate(1, relative_s);
    double d_pprime = lat_trajectory.Evaluate(2, relative_s);

    ReferencePoint matched_ref_point = PathMatcher::MatchToPath(s, reference_points);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s();
    const double rx = matched_ref_point.get_x();
    const double ry = matched_ref_point.get_y();
    const double rtheta = matched_ref_point.heading();
    const double rkappa = matched_ref_point.kappa();
    const double rdkappa = matched_ref_point.dkappa();

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
    CartesianFrenetConverter::frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x,
                                                  &y, &theta, &kappa, &v, &a);

    //如果有了历史轨迹点
    if (prev_trajectory_point.has_x() && prev_trajectory_point.has_y())
    {
      double delta_x = x - prev_trajectory_point.x;
      double delta_y = y - prev_trajectory_point.y;
      double delta_s = std::hypot(delta_x, delta_y); //欧式距离
      accumulated_trajectory_s += delta_s;
    }

    TrajectoryPoint trajectory_point;

    //赋值frenet坐标系下的
    trajectory_point.set_relative_time(t_param + init_relative_time);
    trajectory_point.set_absolute_time(plan_start_time + t_param);
    // trajectory_point.set_s(accumulated_trajectory_s); //保持轨迹的第一个点为s=0
    trajectory_point.set_s(s); //不保持轨迹s
    trajectory_point.s_d = s_dot;
    trajectory_point.s_dd = s_ddot;
    trajectory_point.s_ddd = lon_trajectory.Evaluate(3, t_param);
    trajectory_point.d = d;
    trajectory_point.d_d = d_prime;
    trajectory_point.d_dd = d_pprime;
    trajectory_point.d_ddd = lat_trajectory.Evaluate(3, relative_s);
    //赋值世界坐标系下的
    trajectory_point.set_x(x);
    trajectory_point.set_y(y);
    trajectory_point.set_theta(theta);
    trajectory_point.set_kappa(kappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);

    //轨迹的时间要前进
    if (!combined_trajectory.empty())
      if (trajectory_point.relative_time < combined_trajectory.back().relative_time)
        continue;

    combined_trajectory.emplace_back(trajectory_point);

    t_param = t_param + Config_.FLAGS_trajectory_time_resolution;

    prev_trajectory_point = trajectory_point.path_point(); //存储历史轨迹点
  }
  return combined_trajectory;
}
