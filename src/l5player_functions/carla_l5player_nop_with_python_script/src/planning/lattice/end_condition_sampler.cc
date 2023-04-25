#include "end_condition_sampler.h"

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(const State &init_s, const State &init_d,
                                         std::shared_ptr<PathTimeGraph> ptr_path_time_graph, std::shared_ptr<PredictionQuerier> ptr_prediction_querier)
    : init_s_(init_s), init_d_(init_d), feasible_region_(init_s), ptr_path_time_graph_(std::move(ptr_path_time_graph)), ptr_prediction_querier_(std::move(ptr_prediction_querier))
{
}

std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const
{
  std::vector<Condition> end_d_conditions;
  // std::array<double, 9> end_d_candidates = {-1.25, -1, -0.75, -0.5, 0, 0.5, 0.75, 1, 1.25};
  std::array<double, 3> end_d_candidates = {-3.0, 0, 3.0};
  std::array<double, 4> end_s_candidates = {10.0, 20.0, 40.0, 80.0};
  // std::array<double, 6> end_s_candidates = {5, 10, 15, 20, 25, 30}; //低速
  for (const auto &s : end_s_candidates)
  {
    for (const auto &d : end_d_candidates)
    {
      State end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(const double ref_cruise_speed) const
{
  std::vector<Condition> end_s_conditions;
  if (Config_.FLAGS_num_velocity_sample >= 1)
  {
    // 1.get采样时间
    // time interval is one second plus the last one 0.01
    // 九个时间点 0.01 1 2 3 4 5 6 7 8
    constexpr size_t num_of_time_samples = 9;
    std::array<double, num_of_time_samples> time_samples;
    for (size_t i = 1; i < num_of_time_samples; ++i)
    {
      auto ratio = static_cast<double>(i) / static_cast<double>(num_of_time_samples - 1);
      time_samples[i] = Config_.FLAGS_trajectory_time_length * ratio;
    }
    time_samples[0] = Config_.FLAGS_polynomial_minimal_param;
    // 2.get采样速度
    for (const auto &time : time_samples)
    {
      // 该时刻速度的上下限值
      double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);
      double v_lower = feasible_region_.VLower(time);

      State lower_end_s = {0.0, v_lower, 0.0};// 位移 速度 加速度
      end_s_conditions.emplace_back(lower_end_s, time);

      State upper_end_s = {0.0, v_upper, 0.0};
      end_s_conditions.emplace_back(upper_end_s, time);

      double v_range = v_upper - v_lower;
      // Number of sample velocities
      // 4.样本速度数目
      size_t num_of_mid_points;
      num_of_mid_points = std::min(static_cast<size_t>(Config_.FLAGS_num_velocity_sample - 2), // 6
                                   static_cast<size_t>(v_range / Config_.FLAGS_min_velocity_sample_gap));// 1m/s
      // std::cout << "("
      //           << "v_lower:" << v_lower << ","
      //           << "v_upper:" << v_upper << ")" << std::endl;
      if (num_of_mid_points > 0)
      {
        double velocity_seg = v_range / static_cast<double>(num_of_mid_points + 1);
        for (size_t i = 1; i <= num_of_mid_points; ++i)
        {
          State end_s = {0.0, v_lower + velocity_seg * static_cast<double>(i), 0.0};
          end_s_conditions.emplace_back(end_s, time);
        }
      }
    }
  }

  return end_s_conditions;
}

//待用
std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForStopping(const double ref_stop_point) const
{
  // time interval is one second plus the last one 0.01
  constexpr size_t num_of_time_samples = 9;
  std::array<double, num_of_time_samples> time_samples;
  for (size_t i = 1; i < num_of_time_samples; ++i)
  {
    auto ratio = static_cast<double>(i) / static_cast<double>(num_of_time_samples - 1);
    time_samples[i] = Config_.FLAGS_trajectory_time_length * ratio;
  }
  time_samples[0] = Config_.FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto &time : time_samples)
  {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForPathTimePoints() const
{
  std::vector<Condition> end_s_conditions;
  std::vector<SamplePoint> sample_points = QueryPathTimeObstacleSamplePoints(); // ST图的采样点

  for (const SamplePoint &sample_point : sample_points)
  {
    if (sample_point.path_time_point.t() < Config_.FLAGS_polynomial_minimal_param)// 0.01
    {
      continue;
    }
    double s = sample_point.path_time_point.s();
    double v = sample_point.ref_v;
    double t = sample_point.path_time_point.t();
    if (s > feasible_region_.SUpper(t) || s < feasible_region_.SLower(t))
    {
      continue;
    }
    State end_state = {s, v, 0.0};
    end_s_conditions.emplace_back(end_state, t);
  }
  return end_s_conditions;
}

std::vector<SamplePoint> EndConditionSampler::QueryPathTimeObstacleSamplePoints() const
{
  std::vector<SamplePoint> sample_points;

  for (const auto &path_time_obstacle : ptr_path_time_graph_->GetPathTimeObstacles()) //遍历障碍物的ST信息
  {
    std::string obstacle_id = path_time_obstacle.id();
    QueryFollowPathTimePoints(obstacle_id, &sample_points);   //跟随采样
    QueryOvertakePathTimePoints(obstacle_id, &sample_points); //超车采样
  }

  return sample_points;
}

//跟车采样
void EndConditionSampler::QueryFollowPathTimePoints(const std::string &obstacle_id, std::vector<SamplePoint> *const sample_points) const
{
  // 获取障碍物周围点的ST,得到采样点
  std::vector<STPoint> follow_path_time_points =
      ptr_path_time_graph_->GetObstacleSurroundingPoints(obstacle_id, -Config_.FLAGS_numerical_epsilon, Config_.FLAGS_time_min_density);// 1e-6 1s

  // for (size_t i = 0; i < follow_path_time_points.size(); i++)
  // {
  //   std::cout << " s:" << follow_path_time_points[i].s() << ","
  //             << " t:" << follow_path_time_points[i].t() << "\n";
  // }
  // std::cout << "--------------------------"
  //           << "\n";

  // for循环遍历ST下边界点
  for (const auto &path_time_point : follow_path_time_points)
  {
    // 沿参考线速度投影,求出障碍物的速度在参考线方向的分量
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
        obstacle_id, path_time_point.s(), path_time_point.t());

    // Generate candidate s
    double s_upper = path_time_point.s() - Config_.front_edge_to_center;
    double s_lower = s_upper - Config_.FLAGS_default_lon_buffer;// 5
    // CHECK_GE(FLAGS_num_sample_follow_per_timestamp, 2);

    double s_gap = Config_.FLAGS_default_lon_buffer /
                   static_cast<double>(Config_.FLAGS_num_sample_follow_per_timestamp - 1);// 3

    // 三个点，从s_lower开始，包括s_lower,每隔 s_gap m取一个点
    for (size_t i = 0; i < Config_.FLAGS_num_sample_follow_per_timestamp; ++i)
    {
      double s = s_lower + s_gap * static_cast<double>(i);
      SamplePoint sample_point;
      sample_point.path_time_point = path_time_point;
      sample_point.path_time_point.set_s(s);
      sample_point.ref_v = v;
      sample_points->push_back(std::move(sample_point));
    }
  }
}

//超车采样
void EndConditionSampler::QueryOvertakePathTimePoints(const std::string &obstacle_id, std::vector<SamplePoint> *sample_points) const
{
  std::vector<STPoint> overtake_path_time_points =
      ptr_path_time_graph_->GetObstacleSurroundingPoints(
          obstacle_id, Config_.FLAGS_numerical_epsilon, Config_.FLAGS_time_min_density);

  for (const auto &path_time_point : overtake_path_time_points)
  {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(obstacle_id, path_time_point.s(), path_time_point.t());
    SamplePoint sample_point;
    sample_point.path_time_point = path_time_point;
    sample_point.path_time_point.set_s(path_time_point.s() + Config_.FLAGS_default_lon_buffer);
    sample_point.ref_v = v;
    sample_points->push_back(std::move(sample_point));
  }
}