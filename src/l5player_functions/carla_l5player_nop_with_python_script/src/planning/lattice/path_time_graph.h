#pragma once
#include "Obstacle.h"
// #include "path_struct.h"
#include "path_matcher.h"

class PathTimeGraph
{
public:
  PathTimeGraph() = default;
  PathTimeGraph(const std::vector<const Obstacle *> &obstacles, const std::vector<ReferencePoint> &discretized_ref_points, const double s_start, const double s_end,
                const double t_start, const double t_end, const std::array<double, 3> &init_d);
  ~PathTimeGraph() = default;
  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(const double t_start, const double t_end,
                                                                               const double t_resolution);
  SL_Boundary ComputeObstacleBoundary(
      const std::vector<Vec2d> &vertices,
      const std::vector<ReferencePoint> &discretized_ref_points) const;

  void SetupObstacles(const std::vector<const Obstacle *> &obstacles,
                      const std::vector<ReferencePoint> &discretized_ref_points);

  void SetStaticObstacle(const Obstacle *obstacle,
                         const std::vector<ReferencePoint> &discretized_ref_points);

  void SetDynamicObstacle(const Obstacle *obstacle,
                          const std::vector<ReferencePoint> &discretized_ref_points);

  std::vector<std::pair<double, double>> GetPathBlockingIntervals(const double t);

  std::vector<std::pair<double, double>> GetLateralBounds(const double s_start,
                                                          const double s_end, const double s_resolution);

  void UpdateLateralBoundsByObstacle(const SL_Boundary &sl_boundary, const std::vector<double> &discretized_path,
                                     const double s_start, const double s_end,
                                     std::vector<std::pair<double, double>> *const bounds);

  STPoint SetPathTimePoint(const std::string &obstacle_id,
                           const double s, const double t) const;

  bool IsObstacleInGraph(const std::string &obstacle_id);

  const std::vector<ST_Boundary> &GetPathTimeObstacles() const;

  std::vector<STPoint> GetObstacleSurroundingPoints(
      const std::string &obstacle_id, const double s_dist,
      const double t_min_density) const;

  std::vector<ST_Boundary> get_path_time_obstacles()
  {
    return path_time_obstacles_;
  }
  std::vector<SL_Boundary> get_static_obs_sl_boundaries()
  {
    return static_obs_sl_boundaries_;
  }

private:
  std::pair<double, double> path_range_;
  std::array<double, 3> init_d_;
  std::pair<double, double> time_range_;
  std::unordered_map<std::string, ST_Boundary> path_time_obstacle_map_;
  std::vector<ST_Boundary> path_time_obstacles_;
  std::vector<SL_Boundary> static_obs_sl_boundaries_;
  std::vector<std::vector<Box2d>> dynamic_obstacle_boxes_;
};
