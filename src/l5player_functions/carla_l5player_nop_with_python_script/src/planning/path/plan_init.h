#ifndef __plan_init__
#define __plan_init__
#include "Configs.h"
#include "curve1d.h"

const Param_Configs Config_;             //所有参数

//优先级队列，fist：存放配对的纵向和横向轨迹，second:存放总的cost

typedef std::pair<std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double> PairCost;

//防函数
struct CostComparator : public std::binary_function<const PairCost &, const PairCost &, bool>
{
  //返回true时，a的优先级低于b的优先级（a排在b的后面）
  bool operator()(const PairCost &left, const PairCost &right) const
  {
    return left.second > right.second;
  }
};


// Lattice_dynamic and EM_dynamic
struct InitialConditions
{
  double d0;   //初始的横向偏移值 [m]
  double dd0;  //初始的横向速度 [m/s]
  double ddd0; //初始的横向加速度 [m/s^2]

  double s0;   //初始的纵向值[m]
  double ds0;  //初始的纵向速度[m/s]
  double dds0; //初始的纵向加速度[m/ss]

  double init_relative_time; //规划起始点的时间

  double x_init;
  double y_init;
  double z_init;

  double v_init;
  double a_init;

  double theta_init;
  double kappa_init;
  double dkappa_init;
};

#endif