#include "feasible_region.h"
FeasibleRegion::FeasibleRegion(const std::array<double, 3> &init_s)
{
    init_s_ = init_s;

    double v = init_s[1];
    // CHECK_GE(v, 0.0);

    const double max_deceleration = -Config_.FLAGS_longitudinal_acceleration_lower_bound;
    // 自车从当前速度下，按照设定的最大减速度，将速度减到0所需要的时间， t = v/a
    t_at_zero_speed_ = v / max_deceleration;
    // 自车从当前速度下，按照设定的最大减速度，将速度减到0所经过的s的长度(需加上初始s)
    s_at_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);
}

double FeasibleRegion::SUpper(const double t) const
{
    if (t >= 0.0)
    {
    }
    return init_s_[0] + init_s_[1] * t +
           0.5 * Config_.FLAGS_longitudinal_acceleration_upper_bound * t * t;
}

double FeasibleRegion::SLower(const double t) const
{
    if (t < t_at_zero_speed_)
    {
        return init_s_[0] + init_s_[1] * t +
               0.5 * Config_.FLAGS_longitudinal_acceleration_lower_bound * t * t;
    }
    return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  
  return init_s_[1] + Config_.FLAGS_longitudinal_acceleration_upper_bound * t;
}

double FeasibleRegion::VLower(const double t) const {
  // v = v0 + at
  // 如果t < t_at_zero_speed_，说明速度不会减为0，则取公式计算出的速度，否则就直接取0
  return t < t_at_zero_speed_
             ? init_s_[1] + Config_.FLAGS_longitudinal_acceleration_lower_bound * t
             : 0.0;
}
 

double FeasibleRegion::TLower(const double s) const
{
    if (s >= init_s_[0])
    {
    }

    double delta_s = s - init_s_[0];
    double v = init_s_[1];
    double a = Config_.FLAGS_longitudinal_acceleration_upper_bound;
    double t = (std::sqrt(v * v + 2.0 * a * delta_s) - v) / a;
    return t;
}