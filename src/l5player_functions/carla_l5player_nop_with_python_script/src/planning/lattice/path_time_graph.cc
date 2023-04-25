/*
动态障碍物只有ST图
静态障碍物有SL图和ST图
*/
#include "path_time_graph.h"
#include "math_utils.h"

PathTimeGraph::PathTimeGraph(const std::vector<const Obstacle *> &obstacles, const std::vector<ReferencePoint> &discretized_ref_points, const double s_start, const double s_end,
                             const double t_start, const double t_end, const std::array<double, 3> &init_d)
{
  // std::cout << "s_start: " << s_start << " s_end: " << s_end << " t_start: " << t_start
  //           << " t_end: " << t_end << std::endl;
  if (s_start >= s_end)
  {
    // ROS_ERROR("s_end is before s_start");
  }
  if (t_start >= t_end)
  {
    // ROS_ERROR("t_end is before t_start");
  }
  path_range_.first = s_start;
  path_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  init_d_ = init_d;

  SetupObstacles(obstacles, discretized_ref_points);
}

std::vector<std::vector<std::pair<double, double>>> PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                                                                            const double t_end,
                                                                                            const double t_resolution)
{
  // 每个时刻，每个障碍物的s_lower, s_upper
  // 两层vector嵌套，最里边表示同一时间所有障碍物的最大和最小的S
  // 最外边表示所有时间障碍物的信息
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution)
  {
    intervals.push_back(GetPathBlockingIntervals(t));
  }

  return intervals;
}

// 每个时刻，每个障碍物的s_lower, s_upper
std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(const double t)
{
  if (time_range_.first <= t && t <= time_range_.second)
  {
  }
  std::vector<std::pair<double, double>> intervals;
  for (auto &pt_obstacle : path_time_obstacles_)
  {
    double maxs = pt_obstacle.max_s_;
    double maxt = pt_obstacle.max_t_;
    double mins = pt_obstacle.min_s_;
    double mint = pt_obstacle.min_t_;

    // std::cout << "maxt:" << maxt << std::endl;
    // std::cout << "mint:" << mint << std::endl;
    // std::cout << "max_s:" << maxs << std::endl;
    // std::cout << "min_s:" << mins << std::endl;
    // std::cout << "///////////////////////////////" << std::endl;

    if (t > maxt || t < mint)
    {
      continue;
    }
    // lerp 线性插值 接收三个数据 a,b,t 该函数返回a,b之间以t 为百分比的值
    double s_upper = common::math::lerp(maxs, mint, maxs, maxt, t);

    double s_lower = common::math::lerp(mins, mint, mins, maxt, t);

    intervals.emplace_back(s_lower, s_upper);
  }

  return intervals;
}

//求SL图
SL_Boundary PathTimeGraph::ComputeObstacleBoundary(
    const std::vector<Vec2d> &vertices,
    const std::vector<ReferencePoint> &discretized_ref_points) const
{
  // std::cout<<"discretized_ref_points:"<<discretized_ref_points.size()<<"\n";
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  // 找到静态障碍物顶点的极大极小值
  for (const auto &point : vertices) //遍历顶点
  {
    // std::cout << "point.x():" << point.x() << ","
    //           << "point.y():" << point.y()
    //           << "\n";
    //找匹配点
    auto sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                         point.x(), point.y());
    start_s = std::fmin(start_s, sl_point.first);
    end_s = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l = std::fmax(end_l, sl_point.second);
  }

  SL_Boundary sl_boundary;

  sl_boundary.start_l_ = start_l;
  sl_boundary.end_l_ = end_l;
  sl_boundary.start_s_ = start_s;
  sl_boundary.end_s_ = end_s;

  return sl_boundary;
}

void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle *> &obstacles,
    const std::vector<ReferencePoint> &discretized_ref_points)
{
  // FLAGS_prediction_total_time其实就是PRediction模块中障碍物轨迹预测总时间，5s。
  // eval_time_interval其实就是Prediction模块中障碍物两个预测轨迹点的时间差，0.1s。
  //所以一共差不多50个位置点。最后对参考线上的每个障碍物在每个时间点设定位置标定框。
  const double total_time = Config_.FLAGS_prediction_total_time;
  uint32_t num_of_time_stamps_ = static_cast<uint32_t>(std::floor(total_time / Config_.eval_time_interval));

  //遍历每个障碍物
  for (const Obstacle *obstacle : obstacles)
  {
    if (obstacle->IsVirtual())
    {
      continue;
    }
    if (!obstacle->HasTrajectory()) //没有预测轨迹就是静态障碍物
    {
      SetStaticObstacle(obstacle, discretized_ref_points);
    }
    else
    {
      SetDynamicObstacle(obstacle, discretized_ref_points);

      // 计算每个时间点的位置，转换成标定框加入动态障碍物队列
      std::vector<Box2d> box_by_time;
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t)
      {
        TrajectoryPoint trajectory_point = obstacle->GetPointAtTime(t * Config_.eval_time_interval);

        Box2d obstacle_box = obstacle->GetBoundingBox(trajectory_point);
        static constexpr double kBuff = 0.5;
        Box2d expanded_obstacle_box = Box2d(obstacle_box.center(), obstacle_box.heading(),
                                            obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
        box_by_time.push_back(expanded_obstacle_box);
      }
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
    }
  }
  // std::cout << "static_obs_sl_boundaries_:" << static_obs_sl_boundaries_.size() << "\n";
  // std::cout << "[" << path_range_.first << "," << path_range_.second << "]"
  //           << "\n";
  // 动静态障碍物的存储
  // 静态障碍物s由近及远排序
  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(), [](const SL_Boundary &sl0, const SL_Boundary &sl1)
            { return sl0.start_s_ < sl1.start_s_; });

  for (auto &path_time_obstacle : path_time_obstacle_map_)
  {
    //
    if (path_time_obstacle.second.upper_left_point_.s() == 0 && path_time_obstacle.second.upper_right_point_.s() == 0)
    {
      continue;
    }
    path_time_obstacles_.push_back(path_time_obstacle.second); //存ST图
  }
}

//需要处理SL_Boundary和ST_Boundary两个对象
void PathTimeGraph::SetStaticObstacle(const Obstacle *obstacle,
                                      const std::vector<ReferencePoint> &discretized_ref_points)
{
  std::string obstacle_id = obstacle->obstacle_id; //赋值id

  // 计算障碍物sl的边界值
  // std::cout << "static obs polygon size = " << obstacle->polygon_points.size() << std::endl;
  SL_Boundary sl_boundary = ComputeObstacleBoundary(obstacle->polygon_points, discretized_ref_points);// 静态障碍物car的sl boundary怎么算的？

  //道路宽度定死先
  double left_width = Config_.FLAGS_default_reference_line_width * 0.5;
  double right_width = Config_.FLAGS_default_reference_line_width * 0.5;

  // SL的ROI范围
  // 静态障碍物在我的前方，在我的后方
  if (sl_boundary.start_s_ > path_range_.second ||
      sl_boundary.end_s_ < path_range_.first ||
      sl_boundary.start_l_ > left_width ||
      sl_boundary.end_l_ < -right_width)
  {
    // std::cout << obstacle_id << std::endl;
    // std::cout << "Obstacle [" << obstacle_id << "] is out of range."
    //           << "\n";
    return;
  }

  // SL障碍物基本信息
  sl_boundary.obstacle_id = obstacle->obstacle_id;
  sl_boundary.timestamp_ = obstacle->timestamp_;
  sl_boundary.obstacle_velocity = obstacle->obstacle_velocity;
  sl_boundary.obstacle_theta = obstacle->obstacle_theta;
  sl_boundary.centerpoint = obstacle->centerpoint;
  sl_boundary.obstacle_length = obstacle->obstacle_length;
  sl_boundary.obstacle_width = obstacle->obstacle_width;
  sl_boundary.obstacle_type = obstacle->obstacle_type;

  //障碍物类型
  sl_boundary.SetBoundaryType(SL_Boundary::BoundaryType::OVERTAKE);
  if (sl_boundary.obstacle_velocity < 0.1)
    sl_boundary.is_static_obstacle = true;
  else
    sl_boundary.is_static_obstacle = false;

  // ST障碍物基本信息
  path_time_obstacle_map_[obstacle_id].obstacle_id = obstacle_id;
  path_time_obstacle_map_[obstacle_id].timestamp_ = obstacle->timestamp_;// 时间戳怎么赋值
  path_time_obstacle_map_[obstacle_id].obstacle_velocity = obstacle->obstacle_velocity;
  path_time_obstacle_map_[obstacle_id].obstacle_theta = obstacle->obstacle_theta;
  path_time_obstacle_map_[obstacle_id].centerpoint = obstacle->centerpoint;
  path_time_obstacle_map_[obstacle_id].obstacle_length = obstacle->obstacle_length;
  path_time_obstacle_map_[obstacle_id].obstacle_width = obstacle->obstacle_width;
  path_time_obstacle_map_[obstacle_id].obstacle_type = obstacle->obstacle_type;
  if (path_time_obstacle_map_[obstacle_id].obstacle_velocity < 0.1)
    path_time_obstacle_map_[obstacle_id].is_static_obstacle = true;
  else
    path_time_obstacle_map_[obstacle_id].is_static_obstacle = false;

  // 构建矩形
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(SetPathTimePoint(obstacle_id, sl_boundary.start_s_, 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(obstacle_id, sl_boundary.start_s_, Config_.FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(SetPathTimePoint(obstacle_id, sl_boundary.end_s_, 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(obstacle_id, sl_boundary.end_s_, Config_.FLAGS_trajectory_time_length));

  //存储上下边界变量
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  upper_points.emplace_back(sl_boundary.end_s_, 0.0);
  upper_points.emplace_back(sl_boundary.end_s_, Config_.FLAGS_trajectory_time_length);
  lower_points.emplace_back(sl_boundary.start_s_, 0.0);
  lower_points.emplace_back(sl_boundary.start_s_, Config_.FLAGS_trajectory_time_length);
  path_time_obstacle_map_[obstacle_id].SetUpper_points(upper_points);
  path_time_obstacle_map_[obstacle_id].SetLower_points(lower_points);
  // 障碍物边界类型，测试使用
  path_time_obstacle_map_[obstacle_id].SetBoundaryType(ST_Boundary::BoundaryType::OVERTAKE);

  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));

  // std::cout << "SL-Graph mapping static obstacle: " << obstacle_id
  //           << ", start_s : " << sl_boundary.start_s_
  //           << ", end_s : " << sl_boundary.end_s_
  //           << ", start_l : " << sl_boundary.start_l_
  //           << ", end_l : " << sl_boundary.end_l_ << "\n";
}

const std::vector<ST_Boundary> &PathTimeGraph::GetPathTimeObstacles() const
{
  return path_time_obstacles_;
}

//只需要处理ST_Boundary对象
void PathTimeGraph::SetDynamicObstacle(const Obstacle *obstacle,
                                       const std::vector<ReferencePoint> &discretized_ref_points)
{
  double relative_time = time_range_.first; // 0s

  while (relative_time < time_range_.second) // 8s
  {
    // 循环遍历障碍物轨迹中的每一个点，每个点画一个ST图，最后把所有ST图组合起来，形成动态障碍物的ST图
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    // std::cout << "*relative_time1:" << relative_time << "\n";
    // std::cout << "*relative_time2:" << point.relative_time << "\n";
    // std::cout << "--------------------------"
    //           << "\n";

    // GetBoundingBox包括：障碍物预测轨迹点的位置，朝向，障碍物的长和宽
    Box2d box = obstacle->GetBoundingBox(point);

    SL_Boundary sl_boundary = ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);

    //道路宽度定死先
    double left_width = Config_.FLAGS_default_reference_line_width * 0.5;
    double right_width = Config_.FLAGS_default_reference_line_width * 0.5;

    // The obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s_ > path_range_.second ||
        sl_boundary.end_s_ < path_range_.first ||
        sl_boundary.start_l_ > left_width ||
        sl_boundary.end_l_ < -right_width)
    {
      if (path_time_obstacle_map_.find(obstacle->obstacle_id) != path_time_obstacle_map_.end()) //找得到就跳出while
      {
        break;
      }
      relative_time += Config_.FLAGS_trajectory_time_resolution;
      continue;
    }

    // std::cout << "SL-Graph mapping static obstacle: " << obstacle->obstacle_id
    //           << ", start_s : " << sl_boundary.start_s_
    //           << ", end_s : " << sl_boundary.end_s_
    //           << ", start_l : " << sl_boundary.start_l_
    //           << ", end_l : " << sl_boundary.end_l_
    //           << ", relative_time:" << relative_time
    //           << "\n";

    //找不到障碍物，也就是新出现的障碍物，则obstacle_id要给,且左边界值要给
    if (path_time_obstacle_map_.find(obstacle->obstacle_id) == path_time_obstacle_map_.end())
    {
      path_time_obstacle_map_[obstacle->obstacle_id].obstacle_id = obstacle->obstacle_id;
      path_time_obstacle_map_[obstacle->obstacle_id].set_bottom_left_point(SetPathTimePoint(obstacle->obstacle_id, sl_boundary.start_s_, relative_time));
      path_time_obstacle_map_[obstacle->obstacle_id].set_upper_left_point(SetPathTimePoint(obstacle->obstacle_id, sl_boundary.end_s_, relative_time));
    }

    // ST障碍物基本信息
    path_time_obstacle_map_[obstacle->obstacle_id].timestamp_ = obstacle->timestamp_;
    path_time_obstacle_map_[obstacle->obstacle_id].obstacle_velocity = obstacle->obstacle_velocity;
    path_time_obstacle_map_[obstacle->obstacle_id].obstacle_theta = obstacle->obstacle_theta;
    path_time_obstacle_map_[obstacle->obstacle_id].centerpoint = obstacle->centerpoint;
    path_time_obstacle_map_[obstacle->obstacle_id].obstacle_length = obstacle->obstacle_length;
    path_time_obstacle_map_[obstacle->obstacle_id].obstacle_width = obstacle->obstacle_width;
    path_time_obstacle_map_[obstacle->obstacle_id].obstacle_type = obstacle->obstacle_type;
    if (path_time_obstacle_map_[obstacle->obstacle_id].obstacle_velocity < 0.1)
      path_time_obstacle_map_[obstacle->obstacle_id].is_static_obstacle = true;
    else
      path_time_obstacle_map_[obstacle->obstacle_id].is_static_obstacle = false;

    path_time_obstacle_map_[obstacle->obstacle_id].time_start_s.emplace_back(relative_time, sl_boundary.start_s_);
    path_time_obstacle_map_[obstacle->obstacle_id].time_end_s.emplace_back(relative_time, sl_boundary.end_s_);

    // 障碍物边界类型，测试使用
    path_time_obstacle_map_[obstacle->obstacle_id].SetBoundaryType(ST_Boundary::BoundaryType::OVERTAKE);

    //
    path_time_obstacle_map_[obstacle->obstacle_id].set_bottom_right_point(SetPathTimePoint(obstacle->obstacle_id, sl_boundary.start_s_, relative_time));
    path_time_obstacle_map_[obstacle->obstacle_id].set_upper_right_point(SetPathTimePoint(obstacle->obstacle_id, sl_boundary.end_s_, relative_time));

    relative_time += Config_.FLAGS_trajectory_time_resolution;
  }

  //存储上下边界变量
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;
  upper_points.emplace_back(path_time_obstacle_map_[obstacle->obstacle_id].upper_left_point_.s(),
                            path_time_obstacle_map_[obstacle->obstacle_id].upper_left_point_.t());
  upper_points.emplace_back(path_time_obstacle_map_[obstacle->obstacle_id].upper_right_point_.s(),
                            path_time_obstacle_map_[obstacle->obstacle_id].upper_right_point_.t());
  lower_points.emplace_back(path_time_obstacle_map_[obstacle->obstacle_id].bottom_left_point_.s(),
                            path_time_obstacle_map_[obstacle->obstacle_id].bottom_left_point_.t());
  lower_points.emplace_back(path_time_obstacle_map_[obstacle->obstacle_id].bottom_right_point_.s(),
                            path_time_obstacle_map_[obstacle->obstacle_id].bottom_right_point_.t());
  path_time_obstacle_map_[obstacle->obstacle_id].SetUpper_points(upper_points);
  path_time_obstacle_map_[obstacle->obstacle_id].SetLower_points(lower_points);

  // std::cout << "ST-Graph mapping static obstacle: " << obstacle->obstacle_id << "\n"
  //           << "start_t : " << path_time_obstacle_map_[obstacle->obstacle_id].bottom_left_point_.t() << "\n"
  //           << "-start_s : " << path_time_obstacle_map_[obstacle->obstacle_id].bottom_left_point_.s()
  //           << ",end_s : " << path_time_obstacle_map_[obstacle->obstacle_id].upper_left_point_.s() << "\n"
  //           << "end_t : " << path_time_obstacle_map_[obstacle->obstacle_id].bottom_right_point_.t() << "\n"
  //           << "-start_s : " << path_time_obstacle_map_[obstacle->obstacle_id].bottom_right_point_.s()
  //           << ",end_s : " << path_time_obstacle_map_[obstacle->obstacle_id].upper_right_point_.s() << "\n";
  // std::cout << "-----------------------------------------------"
  //           << "\n";
}

//障碍物是否已经构建图
bool PathTimeGraph::IsObstacleInGraph(const std::string &obstacle_id)
{
  return path_time_obstacle_map_.find(obstacle_id) != path_time_obstacle_map_.end();
}

//-------------------------------------------根据s，求取l， 二次规划低速避开静态障碍物-------------------------------------//
std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(const double s_start,
                                                                       const double s_end, const double s_resolution)
{
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;
  double s_range = s_end - s_start;
  double s_curr = s_start;
  size_t num_bound = static_cast<size_t>(s_range / s_resolution);
  double ego_width = Config_.FLAGS_vehicle_width;

  // Initialize bounds by reference line width
  for (size_t i = 0; i < num_bound; ++i)
  {
    double left_width = Config_.FLAGS_default_reference_line_width / 2.0;
    double right_width = Config_.FLAGS_default_reference_line_width / 2.0;

    //有些地方，车比较宽，有些地方比较窄，后面再完善
    // ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width, &right_width);

    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;
    bounds.emplace_back(std::min(-right_width, ego_d_lower - Config_.FLAGS_bound_buffer),
                        std::max(left_width, ego_d_upper + Config_.FLAGS_bound_buffer));
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }

  //有静态障碍物的话，二次规划是低速避开静态障碍物
  if (static_obs_sl_boundaries_.size() > 0)
  {
    for (const SL_Boundary &static_sl_boundary : static_obs_sl_boundaries_)
    {
      UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start, s_end, &bounds);
    }
  }

  for (size_t i = 0; i < bounds.size(); ++i)
  {
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    //如果存在[0,0]车得停下来，因为开不过去了，这个还没写
    if (bounds[i].first >= bounds[i].second)
    {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }

  // for (size_t i = 0; i < bounds.size(); ++i)
  // {
  //   std::cout << "[" << bounds[i].first << "," << bounds[i].second << "]" << std::endl;
  // }
  // std::cout << "////////////////////" << bounds.size() << "///////////////////" << std::endl;

  return bounds;
}

void PathTimeGraph::UpdateLateralBoundsByObstacle(const SL_Boundary &sl_boundary, const std::vector<double> &discretized_path,
                                                  const double s_start, const double s_end,
                                                  std::vector<std::pair<double, double>> *const bounds)
{
  double end_s = sl_boundary.end_s_;
  double end_l = sl_boundary.end_l_;
  double start_s = sl_boundary.start_s_;
  double start_l = sl_boundary.start_l_;

  // std::cout << "id:" << sl_boundary.obstacle_id << std::endl;
  // std::cout << "max_l:" << end_l << std::endl;
  // std::cout << "min_l:" << start_l << std::endl;
  // std::cout << "max_s:" << end_s << std::endl;
  // std::cout << "min_s:" << start_s << std::endl;
  // std::cout << "///////////////////////////////" << std::endl;

  if (start_s > s_end || end_s < s_start)
  {
    return;
  }

  auto start_iter = std::lower_bound(discretized_path.begin(), discretized_path.end(), start_s); //返回第一个大于等于start_s的地址
  /*
    Apollo是写start_s，我觉得很奇怪
    auto end_iter = std::upper_bound(discretized_path.begin(), discretized_path.end(), start_s);
  */
  auto end_iter = std::upper_bound(discretized_path.begin(), discretized_path.end(), end_s); //返回第一个大于n的地址
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.begin();

  // std::cout << "(" << start_index << "," << end_index << ")" << std::endl;

  //障碍物在参考线中心上，需要调整宽度才有解
  if (end_l > -Config_.FLAGS_numerical_epsilon && start_l < Config_.FLAGS_numerical_epsilon)
  {
    // std::cout << "center"
    //           << "\n";
    for (size_t i = start_index; i < end_index; ++i)
    {
      bounds->operator[](i).first = -Config_.FLAGS_numerical_epsilon;
      bounds->operator[](i).second = Config_.FLAGS_numerical_epsilon;
    }
    return;
  }
  //右边有障碍物
  if (end_l < Config_.FLAGS_numerical_epsilon)
  {
    // std::cout << "right"
    //           << "\n";
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size()); ++i)
    {
      bounds->operator[](i).first = std::max(bounds->operator[](i).first, end_l + Config_.FLAGS_nudge_buffer);
    }
    return;
  }
  //左边有障碍物
  if (start_l > -Config_.FLAGS_numerical_epsilon)
  {
    // std::cout << "left"
    //           << "\n";
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size()); ++i)
    {
      bounds->operator[](i).second = std::min(bounds->operator[](i).second, start_l - Config_.FLAGS_nudge_buffer);
    }
    return;
  }
}

//--------------------------------------计算道路的宽度---------------------------------//
// bool PathTimeGraph::GetLaneWidth(const double s, double *lane_left_width, double *lane_right_width) const
// {
//     if (lane_left_width != nullptr && lane_right_width != nullptr)
//     {
//     }

//     if (s < 0.0 || s > length_) //传入参考线长度
//     {
//         return false;
//     }
//     *lane_left_width = GetSample(lane_left_width_, s);
//     *lane_right_width = GetSample(lane_right_width_, s);
//     return true;
// }

// double PathTimeGraph::GetSample(const std::vector<double> &samples, const double s) const
// {
//     if (samples.empty())
//     {
//         return 0.0;
//     }
//     if (s <= 0.0)
//     {
//         return samples[0];
//     }
//     const int idx = static_cast<int>(s / kSampleDistance);
//     double num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
//     if (idx >= num_sample_points_ - 1)
//     {
//         return samples.back();
//     }
//     const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
//     return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
// }

STPoint PathTimeGraph::SetPathTimePoint(const std::string &obstacle_id,
                                        const double s, const double t) const
{
  STPoint path_time_point(s, t);
  return path_time_point;
}

// https://blog.csdn.net/weixin_44558122/article/details/124771470
std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const std::string &obstacle_id, const double s_dist,
    const double t_min_density) const
{
  // ACHECK(t_min_density > 0.0);
  std::vector<std::pair<double, double>> bounds;
  std::vector<STPoint> pt_pairs;
  // 找不到障碍物就返回
  if (path_time_obstacle_map_.find(obstacle_id) == path_time_obstacle_map_.end())
  {
    return pt_pairs;
  }
  // 通过ID拿到障碍物相关信息
  const auto &pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

  double s0 = 0.0;
  double s1 = 0.0;

  double t0 = 0.0;
  double t1 = 0.0;

  // s_dist > 0.0的时候表示的是超车
  if (s_dist > 0.0)
  {
    s0 = pt_obstacle.upper_left_point_.s();
    s1 = pt_obstacle.upper_right_point_.s();

    t0 = pt_obstacle.upper_left_point_.t();
    t1 = pt_obstacle.upper_right_point_.t();

    bounds = pt_obstacle.time_end_s;
  }
  else // else的时候代表的是跟车的情况，跟车的话是考虑ST图的下面
  {
    // 获取ST图的左下角和右下角的s
    s0 = pt_obstacle.bottom_left_point_.s();
    s1 = pt_obstacle.bottom_right_point_.s();
    // 获取ST图的左下角和右下角的t
    t0 = pt_obstacle.bottom_left_point_.t();
    t1 = pt_obstacle.bottom_right_point_.t();

    bounds = pt_obstacle.time_start_s;
  }
  // std::cout << "s0:" << s0 << ","
  //           << "t0:" << t0 << "\n";
  // std::cout << "s1:" << s1 << ","
  //           << "t1:" << t1 << "\n";
  // 时间差
  double time_gap = t1 - t0;
  // ACHECK(time_gap > -FLAGS_numerical_epsilon);
  time_gap = std::fabs(time_gap); // 8s

  // t_min_density默认为1.0
  // num_sections表示按照设定时间间隔,在障碍物的delte_t内能取多少段. + 1是为了保证num_sections的准确度,举个例子:
  // 例如 t_min_density=2, time_gap=15, 那么time_gap / t_min_density=7.5, 那么数据类型在强转size_t的时候就
  // 取了整数,那其实就丢失了0.5的精度, 如果 +1 的话, 7.5+1=8.5, static_cast<size_t>(8.5)=8, 这样就可以保证多出
  // 来的0.5也会被考虑进去了
  size_t num_sections = static_cast<size_t>(time_gap / t_min_density + 1); // 8
  // 设定时间间隔
  double t_interval = time_gap / static_cast<double>(num_sections); // 1s

  auto comp = [](const std::pair<double, double> p, const double time)
  {
    return p.first < time;
  };

  for (size_t i = 0; i <= num_sections; ++i)
  {
    double t = t_interval * static_cast<double>(i) + t0;

    //线性插值求s，直接拉皮手术。也就回到了一开始的设定：障碍物不做变加速运动，所以其ST一定是平行四边形。（这个设定感觉有坑额..）
    // double s = common::math::lerp(s0, t0, s1, t1, t) + s_dist;   //线性插值求s，但是我不想这么求,因为动态障碍物预测没有8s，只有4s

    auto it_lower = std::lower_bound(bounds.begin(), bounds.end(), t, comp);
    double s = bounds[it_lower - bounds.begin()].second;

    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}
