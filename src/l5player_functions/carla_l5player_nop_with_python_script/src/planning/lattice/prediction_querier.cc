#include "prediction_querier.h"
// #include "linear_interpolation.h"
#include "path_matcher.h"

namespace
{
    template <class Collection>
    bool InsertIfNotPresent(Collection *const collection,
                            const typename Collection::value_type &vt)
    {
        return collection->insert(vt).second;
    }
    template <class Collection>
    bool InsertIfNotPresent(
        Collection *const collection,
        const typename Collection::value_type::first_type &key,
        const typename Collection::value_type::second_type &value)
    {
        return InsertIfNotPresent(
            collection, typename Collection::value_type(key, value));
    }
}

PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle *> &obstacles,
    const std::shared_ptr<std::vector<PathPoint>> &ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line)
{
    for (const auto ptr_obstacle : obstacles)
    {
        if (InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->obstacle_id,
                               ptr_obstacle))
        {
            obstacles_.push_back(ptr_obstacle);
        }
        else
        {
            // std::cout << "Duplicated obstacle found [" << ptr_obstacle->obstacle_id << "]";
        }
    }
}

std::vector<const Obstacle *> PredictionQuerier::GetObstacles() const
{
    return obstacles_;
}

//沿参考线速度投影
double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string &obstacle_id, const double s, const double t) const
{
    // ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());
    // 获取障碍物轨迹
    const auto &trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
    // 获取轨迹点数量
    int num_traj_point = trajectory.trajectory_point_size();
    // std::cout << "num_traj_point:" << num_traj_point
    //           << "\n";
    
    // 获得轨迹点
    const std::vector<TrajectoryPoint> trajectory_points = trajectory.trajectory_point();
    // 认为是静态障碍物
    if (num_traj_point < 2)
    {
        return 0.0;
    }
    // 如果传入的t不在障碍物轨迹点的t的范围之内，说明该障碍物不会造成影响，直接返回
    if (t < trajectory_points.front().relative_time ||
        t > trajectory_points.back().relative_time)
    {
        return 0.0;
    }

    // for (size_t i = 0; i < trajectory_points.size(); i++)
    // {
    //     std::cout << " s:" << trajectory_points[i].s << ","
    //               << " relative_time:" << trajectory_points[i].relative_time << "\n";
    // }
    // std::cout << "--------------------------"
    //           << "\n";

    // 查找最靠近时间点t的轨迹点
    auto matched_it = std::lower_bound(trajectory_points.begin(),
                                       trajectory_points.end(), t,
                                       [](const TrajectoryPoint &p, const double time)
                                       {
                                           return p.relative_time < time;
                                       });
    // std::cout << "111111111111111"
    //           << "\n";
    // 获得轨迹点的v、heading
    double v = matched_it->v;
    double theta = matched_it->path_point().theta; // 这个theta是障碍物轨迹的heading
    double v_x = v * std::cos(theta);
    double v_y = v * std::sin(theta);

    // 查找该点在参考线上的对应点
    PathPoint obstacle_point_on_ref_line = PathMatcher::MatchToPath(*ptr_reference_line_, s);
    auto ref_theta = obstacle_point_on_ref_line.theta; // 这个theta是参考线在该s下的heading

    // 点乘，速度投影到参考线上
    return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}