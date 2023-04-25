/*
计算frenet坐标系下的曲线参数,
返回参考线对象
系数是数组
author：xia
date:2020.12
*/
#include "CubicSpline2D.h"

// Default constructor
CubicSpline2D::CubicSpline2D() = default;
// /输入参考线（道路中心线）的x,y坐标，构造二维三次样条函数，即sx,sy，合成x-s坐标系与y-s坐标系
CubicSpline2D::CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y, std::vector<double> s)
{
  sx = CubicSpline1D(s, x);  // x-s, filtered_points[0]是数组，包含x1,x2,,,,xn
  sy = CubicSpline1D(s, y);  // y-s，filtered_points[1]是数组，包含y1,y2,,,,yn
}

//输入参考线（道路中心线）的x,y坐标，计算曲线长度，曲线长度是一个数组，记录每一段的s长度
std::vector<double> CubicSpline2D::calc_s(const std::vector<double>& x, const std::vector<double>& y)
{
  std::vector<double> s;
  int nx = x.size();
  std::vector<double> dx(nx);
  std::vector<double> dy(nx);
  adjacent_difference(x.begin(), x.end(), dx.begin());
  adjacent_difference(y.begin(), y.end(), dy.begin());
  dx.erase(dx.begin());
  dy.erase(dy.begin());

  double cum_sum = 0.0;
  s.push_back(cum_sum);
  for (int i = 0; i < nx - 1; i++)
  {
    // cum_sum += norm(dx[i], dy[i]); //norm距离公式
    cum_sum += sqrt(pow(dx[i], 2) + pow(dy[i], 2));
    s.push_back(cum_sum);
  }
  s.erase(unique(s.begin(), s.end()), s.end());
  return s;
}

// Calculate the x position along the spline at given s，输入的是曲线长度s,也可以是t
double CubicSpline2D::calc_x(double s)
{
  return sx.calc_der0(s);
}

// Calculate the y position along the spline at given s
double CubicSpline2D::calc_y(double s)
{
  return sy.calc_der0(s);
}

// Calculate the curvature along the spline at given s
double CubicSpline2D::calc_curvature(double s)
{
  double dx = sx.calc_der1(s);
  double ddx = sx.calc_der2(s);
  double dy = sy.calc_der1(s);
  double ddy = sy.calc_der2(s);
  double k = (ddy * dx - ddx * dy) / pow(pow(dx, 2) + pow(dy, 2), 1.5);
  return k;
}

// Calculate the yaw along the spline at given s
double CubicSpline2D::calc_yaw(double s)
{
  double dx = sx.calc_der1(s);
  double dy = sy.calc_der1(s);
  double yaw = atan2(dy, dx);
  return yaw;
}

// 给定x，y位置和初始猜测s0，找到最接近的s值
//用处：
double CubicSpline2D::find_s(double x, double y, double s0, std::vector<double> s)
{
  double s_closest = s0;
  double closest = INFINITY;
  double si = s.front();
  do
  {
    double px = calc_x(si);
    double py = calc_y(si);
    double dist = sqrt(pow((x - px), 2) + pow((y - py), 2));
    if (dist < closest)
    {
      closest = dist;
      s_closest = si;
    }
    si += 0.1;
  } while (si < s.back());
  return s_closest;
}
