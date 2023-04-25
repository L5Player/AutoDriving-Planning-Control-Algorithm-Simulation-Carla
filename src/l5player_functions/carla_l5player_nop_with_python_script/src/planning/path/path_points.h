#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "vec2d.h"
#include <string>

class PathPoint
{
public:
  PathPoint();

  ~PathPoint() = default;
  bool has_x() const;
  bool has_y() const;
  void set_x(const double x_);
  void set_y(const double y_);
  void set_z(const double z_);
  void set_s(const double s_);
  void set_theta(const double theta_);
  void set_kappa(const double kappa_);
  void set_dkappa(const double dkappa_);
  void set_v(const double v_);
  void set_a(const double a_);

  double get_x();
  double get_y();

  // 世界坐标系下的参数
  double x;      // x position
  double y;      // y position
  double z;      // z position
  double theta;  // yaw in rad
  double kappa;  // curvature曲率
  double dkappa; // curvature曲率导数
  double v;      // Tangential velocity
  double a;      // Tangential acceleration
  double s;      // s position along spline

private:
  bool is_x;
  bool is_y;
};

class FrenetFramePoint
{
public:
  FrenetFramePoint();
  ~FrenetFramePoint() = default;
  bool has_s() const;
  void set_s(const double s_);
  void set_ds(const double s_d_);

  void set_dds(const double s_dd_);
  void set_l(const double d_);
  void set_dl(const double d_dd_);
  void set_ddl(const double d_ddd_);

  double get_s();
  double get_l();

  // Frenet坐标系下的属性
  double relative_time; // relative_time
  double d;             // lateral offset
  double d_d;           // lateral speed
  double d_dd;          // lateral acceleration
  double d_ddd;         // lateral jerk
  double s;             // s position along spline
  double s_d;           // s speed
  double s_dd;          // s acceleration
  double s_ddd;         // s jerk

private:
  bool is_s;
};

class SpeedPoint
{
public:
  SpeedPoint();
  ~SpeedPoint() = default;
  void Clear();
  bool has_v() const;
  bool has_a() const;
  bool has_da() const;
  bool has_s() const;
  void set_s(const double s_);
  void set_t(const double t_);
  void set_v(const double v_);
  void set_a(const double a_);
  void set_da(const double da_); //加速度的导数

  double s;
  double v;
  double t;
  double a;
  double da;

private:
  bool is_s;
  bool is_v;
  bool is_t;
  bool is_a;
  bool is_da;
};

class SLPoint
{
public:
  SLPoint()
  {
  }
  SLPoint(const double _s, const double _l) : s(_s), l(_l)
  {
    t = 0;
    cost = 0;
  }
  void set_s(const double s_)
  {
    s = s_;
  }
  void set_t(const double t_)
  {
    t = t_;
  }
  void set_l(const double l_)
  {
    l = l_;
  }
  void set_cost(const double cost_)
  {
    cost = cost_;
  }
  double s;
  double l;
  double t;
  double cost;
};

class STPoint : public Vec2d
{
  // x-axis: t; y-axis: s.
public:
  STPoint() = default;
  STPoint(const double s, const double t) : Vec2d(t, s)
  {
  }

  STPoint(const Vec2d &vec2d_point) : Vec2d(vec2d_point)
  {
  }

  double x() const = delete;
  double y() const = delete;

  double s() const
  {
    return y_;
  }

  double t() const
  {
    return x_;
  }

  void set_s(const double s)
  {
    y_ = s;
  }

  void set_t(const double t)
  {
    x_ = t;
  }
};