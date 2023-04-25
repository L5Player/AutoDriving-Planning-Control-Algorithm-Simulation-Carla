/*
样条曲线y=ax^3+bx^2+cx+d
返回三次样条插值的系数ai.bi.ci.di
系数是数组
author：xia
date:2020.12
*/
#include "CubicSpline1D.h"
#include <algorithm>
#include <numeric>
#include <cmath>

// Default constructor
CubicSpline1D::CubicSpline1D() = default;
//有参构造函数,输入地图的(x,y)坐标分别用X,Y表示,nx初值X.size()，a初始化为Y，x初始化为X，y初始化为Y。这里a已经求出了，为Y
CubicSpline1D::CubicSpline1D(const std::vector<double>& X, const std::vector<double>& Y)
  : nx(X.size()), a(Y), x(X), y(Y)
{
  //计算x之间的差，为数组[0，X（2）-X（1），X（3）-X（2）。。。X（n）-X（n-1）]
  std::vector<double> h(nx);
  adjacent_difference(x.begin(), x.end(), h.begin());
  h.erase(h.begin());  //删除deltas.begin()处的一个字符，就是删除x(1)，不需要用
  // 构建矩阵
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
  matrix_a(h, A);
  vector_b(h, B);
  // 求解系数c
  // %ci=mi/2,在这里，obj.c=mi/2=A\B;因为在计算B的时候，系数是3，已经除以2，所以这里直接等于
  Eigen::MatrixXd ma_inv = A.inverse();  //.inverse()为求逆矩阵。
  Eigen::VectorXd tmp_c = ma_inv * B;
  c.resize(tmp_c.size());
  Eigen::VectorXd::Map(&c[0], tmp_c.size()) = tmp_c;  //容器tmp_c的值复制给c
  // 求解系数b,d
  for (int i = 0; i < nx - 1; i++)
  {
    d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
    b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0);
  }
}

// calculate matrix A for spline coefficient c
void CubicSpline1D::matrix_a(std::vector<double>& deltas, Eigen::MatrixXd& result)
{
  result(0, 0) = 1;
  for (int i = 0; i < nx - 1; i++)
  {
    if (i != nx - 2)
    {
      result(i + 1, i + 1) = 2.0 * (deltas[i] + deltas[i + 1]);
    }
    result(i + 1, i) = deltas[i];
    result(i, i + 1) = deltas[i];
  }

  result(0, 1) = 0.0;
  result(nx - 1, nx - 2) = 0.0;
  result(nx - 1, nx - 1) = 1.0;
}

// calculate  matrix B for spline coefficient c
void CubicSpline1D::vector_b(std::vector<double>& deltas, Eigen::VectorXd& result)
{
  for (int i = 0; i < nx - 2; i++)
  {
    result(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / deltas[i + 1] - 3.0 * (a[i + 1] - a[i]) / deltas[i];
  }
}

// 计算函数值，返回(t,result)
double CubicSpline1D::calc_der0(double t)
{
  if (t < x.front() || t >= x.back())
  {
    return NAN;
  }

  int i = search_index(t) - 1;  // t所在下标前一个点，以求差值
  double dx = t - x[i];
  return a[i] + b[i] * dx + c[i] * pow(dx, 2) + d[i] * pow(dx, 3);
}

// 计算一阶函数值
double CubicSpline1D::calc_der1(double t)
{
  if (t < x.front() || t >= x.back())
  {
    return NAN;
  }

  int i = search_index(t) - 1;
  double dx = t - x[i];

  return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow(dx, 2);
}

// 计算二阶函数值
double CubicSpline1D::calc_der2(double t)
{
  if (t < x.front() || t >= x.back())
  {
    return NAN;
  }

  int i = search_index(t) - 1;
  double dx = t - x[i];

  return 2.0 * c[i] + 6.0 * d[i] * dx;
}

// upper_bound( begin,end,num)：
//从数组的begin位置到end-1位置二分查找第一个大于num的数字，
//找到返回该数字的地址，不存在则返回end。
//通过返回的地址减去起始地址begin,得到找到数字在数组中的下标。
int CubicSpline1D::search_index(double t)
{
  return std::upper_bound(x.begin(), x.end(), t) - x.begin();
}