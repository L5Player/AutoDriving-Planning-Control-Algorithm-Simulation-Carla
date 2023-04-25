#include "reference_point.h"
#include "math_utils.h"
ReferencePoint::ReferencePoint(const double kappa_, const double dkappa_, const double x_, const double y_,
                               const double heading_, const double accumulated_s_)
{
  this->kappa_ = kappa_;
  this->dkappa_ = dkappa_;
  this->x_ = x_;
  this->y_ = y_;
  this->heading_ = heading_;
  this->accumulated_s_ = accumulated_s_;
}

double ReferencePoint::kappa() const
{
  return kappa_;
}
double ReferencePoint::dkappa() const
{
  return dkappa_;
}
double ReferencePoint::heading() const
{
  return heading_;
}
double ReferencePoint::s() const
{
  return accumulated_s_;
}
double ReferencePoint::get_x() const
{
  return x_;
}
double ReferencePoint::get_y() const
{
  return y_;
}
void ReferencePoint::set_x(double x)
{
  x_ = x;
}
void ReferencePoint::set_y(double y)
{
  y_ = y;
}
void ReferencePoint::set_heading(double heading)
{
  heading_ = heading;
}
void ReferencePoint::set_kappa(double kappa)
{
  kappa_ = kappa;
}
void ReferencePoint::set_dkappa(double dkappa)
{
  dkappa_ = dkappa;
}
void ReferencePoint::set_s(double s)
{
  accumulated_s_ = s;
}

void ReferencePoint::cartesian_to_frenet(const double rs, const double rx, const double ry, const double rtheta,
                                         const double x, const double y, double* ptr_s, double* ptr_d)
{
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  *ptr_s = rs;
}

void ReferencePoint::frenet_to_cartesian(const double rs, const double rx, const double ry, const double rtheta,
                                         const double rkappa, const double rdkappa,
                                         const std::array<double, 3>& s_condition,
                                         const std::array<double, 3>& d_condition, double* const ptr_x,
                                         double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
                                         double* const ptr_v, double* const ptr_a)
{
  // ACHECK(std::abs(rs - s_condition[0]) < 1.0e-6) << "The reference point s and s_condition[0] don't match";
  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  *ptr_x = rx - sin_theta_r * d_condition[0];
  *ptr_y = ry + cos_theta_r * d_condition[0];

  const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

  const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
  const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
  const double cos_delta_theta = std::cos(delta_theta);

  *ptr_theta = common::math::NormalizeAngle(delta_theta + rtheta);

  const double kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1];
  *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) /
                    (one_minus_kappa_r_d) +
                rkappa) *
               cos_delta_theta / (one_minus_kappa_r_d);

  const double d_dot = d_condition[1] * s_condition[1];
  *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot);

  const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

  *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
           s_condition[1] * s_condition[1] / cos_delta_theta * (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}