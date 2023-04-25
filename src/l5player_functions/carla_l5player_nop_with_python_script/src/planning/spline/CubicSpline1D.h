#ifndef CUBICSPLINE1D_H
#define CUBICSPLINE1D_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
class CubicSpline1D
{
public:
  int nx;  //计算x长度
  CubicSpline1D();
  CubicSpline1D(const std::vector<double>& X,
                const std::vector<double>& Y);  //有参构造函数,输入地图的(x,y)坐标分别用X,Y表示
  double calc_der0(double t);                   //计算函数值
  double calc_der1(double t);                   //计算一阶导数函数值
  double calc_der2(double t);                   //计算二阶导数函数值
private:
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
  std::vector<double> w;
  std::vector<double> x;
  std::vector<double> y;
  int search_index(double t);
  void matrix_a(std::vector<double>& deltas, Eigen::MatrixXd& result);
  void vector_b(std::vector<double>& deltas, Eigen::VectorXd& result);
};

#endif  // CUBICSPLINE1D_H
