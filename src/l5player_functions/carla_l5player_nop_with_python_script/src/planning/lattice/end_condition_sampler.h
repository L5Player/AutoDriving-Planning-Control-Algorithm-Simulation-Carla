#pragma once
#include "plan_init.h"
#include "feasible_region.h"
#include "path_time_graph.h"
#include "prediction_querier.h"
#include <vector>
#include <cmath>
#include <array>
#include <string>

struct SamplePoint
{
  STPoint path_time_point;
  double ref_v;
};

class EndConditionSampler
{
public:
  EndConditionSampler() = default;
  EndConditionSampler(const std::array<double, 3> &init_s, const std::array<double, 3> &init_d,
                      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
                      std::shared_ptr<PredictionQuerier> ptr_prediction_querier);

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions() const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const double ref_stop_point) const;

  std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionsForPathTimePoints() const;

private:
  std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

  void QueryFollowPathTimePoints(
      const std::string &obstacle_id,
      std::vector<SamplePoint> *sample_points) const;

  void QueryOvertakePathTimePoints(
      const std::string &obstacle_id,
      std::vector<SamplePoint> *sample_points) const;

private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  FeasibleRegion feasible_region_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::shared_ptr<PredictionQuerier> ptr_prediction_querier_;
};