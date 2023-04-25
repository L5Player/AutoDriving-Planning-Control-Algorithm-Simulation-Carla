#include "constant_jerk_trajectory1d.h"

ConstantJerkTrajectory1d::ConstantJerkTrajectory1d(const double p0, const double v0, const double a0, const double j,
                                                   const double param)
    : p0_(p0), v0_(v0), a0_(a0), param_(param), jerk_(j)
{
  // CHECK_GT(param, FLAGS_numerical_epsilon);
  p1_ = Evaluate(0, param_);
  v1_ = Evaluate(1, param_);
  a1_ = Evaluate(2, param_);
}

double ConstantJerkTrajectory1d::Evaluate(const std::uint32_t order, const double param) const
{
  // CHECK_GT(param, -FLAGS_numerical_epsilon);
  switch (order)
  {
  case 0:
  {
    return p0_ + v0_ * param + 0.5 * a0_ * param * param + jerk_ * param * param * param / 6.0;
  }
  case 1:
  {
    return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
  }
  case 2:
  {
    return a0_ + jerk_ * param;
  }
  case 3:
  {
    return jerk_;
  }
  default:
    return 0.0;
  }
}

double ConstantJerkTrajectory1d::start_position() const
{
  return p0_;
}

double ConstantJerkTrajectory1d::start_velocity() const
{
  return v0_;
}

double ConstantJerkTrajectory1d::start_acceleration() const
{
  return a0_;
}

double ConstantJerkTrajectory1d::end_position() const
{
  return p1_;
}

double ConstantJerkTrajectory1d::end_velocity() const
{
  return v1_;
}

double ConstantJerkTrajectory1d::end_acceleration() const
{
  return a1_;
}

double ConstantJerkTrajectory1d::ParamLength() const
{
  return param_;
}

std::string ConstantJerkTrajectory1d::ToString() const
{
  return "";
}

double ConstantJerkTrajectory1d::jerk() const
{
  return jerk_;
}