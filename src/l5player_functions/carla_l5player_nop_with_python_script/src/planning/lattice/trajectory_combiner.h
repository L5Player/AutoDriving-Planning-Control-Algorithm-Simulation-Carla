#ifndef TRAJECTORY_COMBINER_H
#define TRAJECTORY_COMBINER_H
#include "cartesian_frenet_conversion.h"
#include "lattice_trajectory1d.h"
#include "trajectory_evaluator.h"
#include "path_matcher.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <queue>
#include <string>

class TrajectoryCombiner
{
public:
  TrajectoryCombiner();
  ~TrajectoryCombiner();
  DiscretizedTrajectory Combine(const std::vector<double> &accumulated_s, const Curve1d &lon_trajectory,
                                const Curve1d &lat_trajectory, const std::vector<ReferencePoint> &reference_points,
                                const double &init_relative_time, const double &absolute_time);
};
#endif