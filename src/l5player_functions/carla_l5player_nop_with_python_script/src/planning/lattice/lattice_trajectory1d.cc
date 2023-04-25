/*
    计算每条轨迹的轨迹点的参数
*/
#include "lattice_trajectory1d.h"

template <class T1>
void CHECK(T1 s1)
{
  if (s1 == false)
  {
    // std::cout << "false" << std::endl;
  }
}

LatticeTrajectory1d::LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d)
{
  ptr_trajectory1d_ = ptr_trajectory1d;
}

double LatticeTrajectory1d::Evaluate(const std::uint32_t order, const double param) const
{
  double param_length = ptr_trajectory1d_->ParamLength();
  if (param <= param_length)
  {
    return ptr_trajectory1d_->Evaluate(order, param);
  }

  // do constant acceleration extrapolation;
  // to align all the trajectories with time.
  double p = ptr_trajectory1d_->Evaluate(0, param_length);
  double v = ptr_trajectory1d_->Evaluate(1, param_length);
  double a = ptr_trajectory1d_->Evaluate(2, param_length);
  double t = param - param_length;

  switch (order)
  {
    case 0:
      return p + v * t + 0.5 * a * t * t;
    case 1:
      return v + a * t;
    case 2:
      return a;
    default:
      return 0.0;
  }
}

double LatticeTrajectory1d::ParamLength() const
{
  return ptr_trajectory1d_->ParamLength();
}

std::string LatticeTrajectory1d::ToString() const
{
  return ptr_trajectory1d_->ToString();
}

bool LatticeTrajectory1d::has_target_position() const
{
  return has_target_position_;
}

bool LatticeTrajectory1d::has_target_velocity() const
{
  return has_target_velocity_;
}

bool LatticeTrajectory1d::has_target_time() const
{
  return has_target_time_;
}

double LatticeTrajectory1d::target_position() const
{
  CHECK(has_target_position_);
  return target_position_;
}

double LatticeTrajectory1d::target_velocity() const
{
  CHECK(has_target_velocity_);
  return target_velocity_;
}

double LatticeTrajectory1d::target_time() const
{
  CHECK(has_target_time_);
  return target_time_;
}

void LatticeTrajectory1d::set_target_position(double target_position)
{
  target_position_ = target_position;
  has_target_position_ = true;
}

void LatticeTrajectory1d::set_target_velocity(double target_velocity)
{
  target_velocity_ = target_velocity;
  has_target_velocity_ = true;
}

void LatticeTrajectory1d::set_target_time(double target_time)
{
  target_time_ = target_time;
  has_target_time_ = true;
}
