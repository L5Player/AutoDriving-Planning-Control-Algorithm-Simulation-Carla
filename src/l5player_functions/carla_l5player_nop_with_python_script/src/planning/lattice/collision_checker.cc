#include "collision_checker.h"

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::CollisionChecker(const std::vector<const Obstacle *> &obstacles, const double ego_vehicle_s,
                                   const double ego_vehicle_d,
                                   const std::vector<ReferencePoint> &discretized_reference_line,
                                   const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph) : ptr_path_time_graph_(ptr_path_time_graph)
{
  BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d, discretized_reference_line);
}

CollisionChecker::~CollisionChecker()
{
}

//原理：https://zhuanlan.zhihu.com/p/146778379
bool CollisionChecker::InCollision(const DiscretizedTrajectory &discretized_trajectory)
{
  // CHECK_LE(discretized_trajectory.NumOfPoints(), predicted_bounding_rectangles_.size());
  double ego_length = Config_.FLAGS_vehicle_length;
  double ego_width = Config_.FLAGS_vehicle_width;

  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i)
  {
    const auto &trajectory_point = discretized_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    double ego_theta = trajectory_point.theta;
    Box2d ego_box({trajectory_point.x, trajectory_point.y}, ego_theta, ego_length, ego_width);

    double shift_distance = ego_length / 2.0 - Config_.back_edge_to_center; //几何中心-车辆中心
    Vec2d shift_vec{shift_distance * std::cos(ego_theta), shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    for (const auto &obstacle_box : predicted_bounding_rectangles_[i])
    {
      if (ego_box.HasOverlap(obstacle_box))
      {
        return true;
      }
    }
  }
  return false;
}

void CollisionChecker::BuildPredictedEnvironment(const std::vector<const Obstacle *> &obstacles, const double ego_vehicle_s,
                                                 const double ego_vehicle_d,
                                                 const std::vector<ReferencePoint> &discretized_reference_line)
{
  //排除不在同车道内或者在车后面的障碍物
  // If the ego vehicle is in lane,
  // then, ignore all obstacles from the same lane.

  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);

  std::vector<const Obstacle *> obstacles_considered;

  for (const Obstacle *obstacle : obstacles)
  {
    if (obstacle->IsVirtual())
    {
      continue;
    }
    //去掉了||!ptr_path_time_graph_->IsObstacleInGraph(obstacle->obstacle_id)，因为采样规划不想加入二次规划的ROI
    // if (ego_vehicle_in_lane &&
    //     (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s, discretized_reference_line)))
    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s, discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->obstacle_id)))
    {
      // 在同一个车道内，要么在主车后面要么不在ST图中
      continue;
    }

    obstacles_considered.push_back(obstacle);
  }
  // std::cout << "obstacles_considered:" << obstacles_considered.size() << "\n";

  double relative_time = 0.0;
  while (relative_time < Config_.FLAGS_trajectory_time_length)
  {
    std::vector<Box2d> predicted_env;

    for (const Obstacle *obstacle : obstacles_considered)
    {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      /*
        障碍物膨胀不难理解，为了安全，人在开车的过程中也会适当的远离附近车辆，
        但是这个膨胀方法显然太过死板了，或许百度内部早就优化了，
        只是开源没释放？所以暂时做了些许改动： 静止物体膨胀程度缩小、动态障碍物适当膨胀 。
        后期必然是需要继续完善的，进一步可借鉴物体危险程度势场方法。
        我认为纵向要膨胀大的
      */
      if (obstacle->IsStatic())
      {
        box.LongitudinalExtend(2.0 * 0.1);
        box.LateralExtend(2.0 * 0.05);
      }
      else
      {
        box.LongitudinalExtend(2.0 * Config_.FLAGS_lon_collision_buffer);
        box.LateralExtend(2.0 * Config_.FLAGS_lat_collision_buffer);
      }
      predicted_env.push_back(std::move(box));
    }

    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    relative_time += Config_.FLAGS_trajectory_time_resolution;
  }
}

bool CollisionChecker::IsEgoVehicleInLane(const double ego_vehicle_s,
                                          const double ego_vehicle_d)
{
  //道路宽度定了
  double left_width = Config_.FLAGS_default_reference_line_width * 0.5;// 5
  double right_width = Config_.FLAGS_default_reference_line_width * 0.5;

  // 跟据你当前自车的s值,来返回你距离左车道线的宽度和右车道线的宽度
  //获取道路宽度
  // ptr_reference_line_info_->reference_line().GetLaneWidth(
  //     ego_vehicle_s, &left_width, &right_width);

  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

//判断障碍物是否在自主车后便，也就是不用考虑这个障碍物了,但是用中心点判断有Bug
bool CollisionChecker::IsObstacleBehindEgoVehicle(
    const Obstacle *obstacle, const double ego_vehicle_s,
    const std::vector<ReferencePoint> &discretized_reference_line)
{
  double half_lane_width = Config_.FLAGS_default_reference_line_width * 0.5;
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);

  auto obstacle_reference_line_position = PathMatcher::GetPathFrenetCoordinate(discretized_reference_line, point.x, point.y);

  if (obstacle_reference_line_position.first < ego_vehicle_s && std::fabs(obstacle_reference_line_position.second) < half_lane_width)
  {
    // std::cout << "Ignore obstacle [" << obstacle->obstacle_id << "]";
    return true;
  }
  return false;
}