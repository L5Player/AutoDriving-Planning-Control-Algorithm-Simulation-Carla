#include "lattice.h"

namespace l5player {
namespace planning {

std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint> &ref_points) {
    double s = 0.0;
    std::vector<PathPoint> path_points;
    for (const auto &ref_point : ref_points) {
        PathPoint path_point;
        path_point.set_x(ref_point.x_);
        path_point.set_y(ref_point.y_);
        path_point.set_theta(ref_point.heading());
        path_point.set_kappa(ref_point.kappa());
        path_point.set_dkappa(ref_point.dkappa());

        if (!path_points.empty()) {
            double dx = path_point.x - path_points.back().x;
            double dy = path_point.y - path_points.back().y;
            s += std::sqrt(dx * dx + dy * dy);
        }
        path_point.set_s(s);
        path_points.push_back(std::move(path_point));
    }
    return path_points;
}

lattice::lattice() {}

DiscretizedTrajectory lattice::plan(const TrajectoryPoint &planning_init_point, const PlanningTarget &planning_target,
                                    const std::vector<const Obstacle *> &obstacles,
                                    const std::vector<double> &accumulated_s,
                                    const std::vector<ReferencePoint> &reference_points,
                                    const bool &lateral_optimization, const double &init_relative_time,
                                    const double lon_decision_horizon, const double &plan_start_time) {
    DiscretizedTrajectory Optim_trajectory;
    // 1. compute the matched point of the init planning point on the reference line.经过投影后的匹配点
    // reference_points中有累积的s值
    ReferencePoint matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.path_point().x,
                                                            planning_init_point.path_point().y);
    std::cout << "matched_point_x = " << matched_point.get_x() << " matched_point_y = " << matched_point.get_y()
              << std::endl;

    // 2. according to the matched point, compute the init state in Frenet frame.
    std::array<double, 3> init_s;
    std::array<double, 3> init_d;
    ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
    std::cout << "init_s[0] = " << init_s[0] << " init_d[0] = " << init_d[0] << std::endl;
    // parse the decision and get the planning target.
    // 与Apollo不同，我们的前探距离不加上init_s[0]，因为我们的仿真的参考线不长。设置障碍物，存储SL ST图
    // 根据障碍物的列表算ST图
    auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_points, init_s[0],
                                                               init_s[0] + 200,    // 前瞻多少m lon_decision_horizon
                                                               0.0, Config_.FLAGS_trajectory_time_length, init_d);

    auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points));
    auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
        obstacles, ptr_reference_line);    // 将障碍物按照id 存储为map容器(key, value) 预测

    // 3.生成横纵向轨迹
    Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
    std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
    std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
    trajectory1d_generator.GenerateTrajectoryBundles(planning_target, &lon_trajectory1d_bundle,
                                                     &lat_trajectory1d_bundle);

    std::cout << "number_lon_traj = " << lon_trajectory1d_bundle.size();
    std::cout << "  number_lat_traj = " << lat_trajectory1d_bundle.size() << "\n";

    // 4.计算每条轨迹的代价,并得出优先级队列
    TrajectoryEvaluator trajectory_evaluator(planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle, init_s,
                                             ptr_path_time_graph, reference_points);
    // Get instance of collision checker and constraint checker
    CollisionChecker collision_checker(obstacles, init_s[0], init_d[0], reference_points, ptr_path_time_graph);
    // 5.轨迹拼接和最后的筛选，总是得到结合的最好的轨迹对，返回第一次无碰撞的轨迹
    while (trajectory_evaluator.has_more_trajectory_pairs()) {
        double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
        auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
        // combine two 1d trajectories to one 2d trajectory
        auto combined_trajectory =
            trajectorycombiner_.Combine(accumulated_s, *trajectory_pair.first, *trajectory_pair.second,
                                        reference_points, init_relative_time, plan_start_time);

        // 采样时候才调用，二次规划不用
        if (lateral_optimization == false) {
            // check longitudinal and lateral acceleration
            // considering trajectory curvatures
            auto result = constraintchecker_.ValidTrajectory(combined_trajectory);
            if (result != ConstraintChecker::Result::VALID) {
                switch (result) {
                    case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
                        break;
                    case ConstraintChecker::Result::VALID:
                    default:
                        // Intentional empty
                        break;
                }
                continue;
            }

            // 碰撞检测
            if (collision_checker.InCollision(combined_trajectory)) {
                continue;
            }
        }
        Optim_trajectory = std::move(combined_trajectory);
        std::cout << "Total_Trajectory_Cost = " << trajectory_pair_cost << "\n";
        break;
    }
    // ROS_WARN("trj_num :%d",Optim_trajectory.size());
    return Optim_trajectory;
}

void lattice::ComputeInitFrenetState(const ReferencePoint &matched_point, const TrajectoryPoint &cartesian_state,
                                     std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d) {
    CartesianFrenetConverter::cartesian_to_frenet(
        matched_point.accumulated_s_, matched_point.x_, matched_point.y_, matched_point.heading_, matched_point.kappa_,
        matched_point.dkappa_, cartesian_state.path_point().x, cartesian_state.path_point().y, cartesian_state.v,
        cartesian_state.a, cartesian_state.path_point().theta, cartesian_state.path_point().kappa, ptr_s, ptr_d);
}

}    // namespace planning
}    // namespace l5player