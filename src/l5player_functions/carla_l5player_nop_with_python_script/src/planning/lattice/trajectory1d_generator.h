#pragma once
#include "quintic_polynomial_curve1d.h"
#include "quartic_polynomial_curve1d.h"
#include "lattice_trajectory1d.h"

#include "PlanningTarget.h"
#include "end_condition_sampler.h"
#include "lateral_qp_optimizer.h"
#include "lateral_osqp_optimizer.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <queue>
#include <string>

class Trajectory1dGenerator
{
public:
  Trajectory1dGenerator() = default;
  Trajectory1dGenerator(const std::array<double, 3> &lon_init_state, const std::array<double, 3> &lat_init_state,
                        std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
                        std::shared_ptr<PredictionQuerier> ptr_prediction_querier);

  virtual ~Trajectory1dGenerator() = default;

  void GenerateTrajectoryBundles(const PlanningTarget &planning_target,
                                 std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle,
                                 std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle);

  void GenerateLateralTrajectoryBundle(std::vector<std::shared_ptr<Curve1d>> *ptr_lat_trajectory_bundle) const;

private:
  void GenerateSpeedProfilesForCruising(const double target_speed,
                                        std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForStopping(const double stop_point,
                                        std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const;

  void
  GenerateSpeedProfilesForPathTimeObstacles(std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const;

  void GenerateLongitudinalTrajectoryBundle(const PlanningTarget &planning_target,
                                            std::vector<std::shared_ptr<Curve1d>> *ptr_lon_trajectory_bundle) const;

  template <size_t N>
  void GenerateTrajectory1DBundle(const std::array<double, 3> &init_state,
                                  const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
                                  std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle) const;

private:
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  EndConditionSampler end_condition_sampler_;
  std::array<double, 3> init_lon_state_;
  std::array<double, 3> init_lat_state_;
  //选择二次规划
  bool FLAGS_lateral_optimization;
  ;
};

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
    const std::array<double, 3> &init_state,
    const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
    std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle) const
{
  if (ptr_trajectory_bundle != nullptr)
  {
    if (!end_conditions.empty())
    {
      ptr_trajectory_bundle->reserve(end_conditions.size());
      for (const auto &end_condition : end_conditions)
      {
        // 计算多项式的系数 a0 a1 a2 a3 a4
        auto ptr_trajectory1d =
            std::make_shared<LatticeTrajectory1d>(std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
                init_state, {end_condition.first[1], end_condition.first[2]}, end_condition.second)));

        ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
        ptr_trajectory1d->set_target_time(end_condition.second);
        ptr_trajectory_bundle->push_back(ptr_trajectory1d);
      }
    }
  }
}

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<5>(
    const std::array<double, 3> &init_state,
    const std::vector<std::pair<std::array<double, 3>, double>> &end_conditions,
    std::vector<std::shared_ptr<Curve1d>> *ptr_trajectory_bundle) const
{
  if (ptr_trajectory_bundle != nullptr)  //有线
  {
    if (!end_conditions.empty())   //有终端条件
    {
      ptr_trajectory_bundle->reserve(end_conditions.size());  //预留存储路径的空间
      for (const auto &end_condition : end_conditions)
      {
        auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(std::shared_ptr<Curve1d>(
            new QuinticPolynomialCurve1d(init_state, end_condition.first, end_condition.second)));

        ptr_trajectory1d->set_target_position(end_condition.first[0]);
        ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
        ptr_trajectory1d->set_target_time(end_condition.second);
        ptr_trajectory_bundle->push_back(ptr_trajectory1d);
      }
    }
  }
}