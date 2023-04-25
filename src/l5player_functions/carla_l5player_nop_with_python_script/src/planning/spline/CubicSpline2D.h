#ifndef CUBICSPLINE2D_H
#define CUBICSPLINE2D_H
#include "CubicSpline1D.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>

class CubicSpline2D
{
public:
  CubicSpline2D();
  CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y, std::vector<double>  s);
  double calc_x(double s);
  double calc_y(double s);
  double calc_curvature(double s);
  double calc_yaw(double s);
  double find_s(double x, double y, double s0, std::vector<double> s);
  CubicSpline1D sx, sy;
  std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y);  //计算曲线长度
};

#endif  // CUBICSPLINE2D_H
