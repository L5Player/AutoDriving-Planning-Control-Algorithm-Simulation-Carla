#include "reference_line.h"

#include <iostream>
#include <vector>

std::vector<Obstacle> AllObstacle;    // 感知一帧识别到的所有障碍物

namespace l5player {
namespace reference_line {

referenceLine::referenceLine() {
    // ros parameter settings
    // rclcpp::param::get("which_smoothers", this->which_smoothers);
    // ros::param::get("apply_curvature_constraint", this->apply_curvature_constraint);
    // ros::param::get("use_lateral_optimization", this->FLAGS_lateral_optimization);
    // ros::param::get("which_planners", this->which_planners);
    // subscriber
    // routing_sub_ = n_.subscribe("/routing", 10, &referenceLine::routingCallback, this);
    // gps_sub_ = n_.subscribe("/gps", 10, &referenceLine::gpsCallback, this);

    // publisher
    // trajectory_pub_ = n_.advertise<msg_gen::trajectory>("/trajectory_waypoints", 10);          //发布局部轨迹
    // rviz_pub_ = n_.advertise<nav_msgs::Path>("/rviz_trajectory_waypoints", 10);                //发布局部轨迹
    // rviz_obstacle_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/rviz_obstacles", 10); // 在rviz中显示障碍物

    // ROS_INFO("which_planners = %i",which_planners);
    which_planners = 0;
    switch (which_planners) {
        case 0:
            std::cout << "lattice planning!!!" << std::endl;
            planning_base_ = std::make_shared<lattice>();
            break;
        case 1:

            break;
        default:
            std::cout << "default:lattice planning!!!" << std::endl;
            planning_base_ = std::make_shared<lattice>();
            break;
    }
}

void referenceLine::routingCallback(const geometry_msgs::msg::PoseArray& routing) {
    routing_waypoints_ = Eigen::MatrixXd::Zero(routing.poses.size(), 3);
    // 确保一开始只订阅一次
    if (routing_waypoint_flag_.size() < 1) {
        routing_waypoint_flag_.push_back(routing.poses[0].position.x);
        for (int i = 0; i < routing.poses.size(); ++i) {
            routing_waypoints_(i, 0) = routing.poses[i].position.x;
            routing_waypoints_(i, 1) = routing.poses[i].position.y;
            routing_waypoints_(i, 2) = routing.poses[i].position.z;
        }
        // 参考线的平滑计算只计算一次
        referenceLine_split(routing_waypoints_);
    }
}

void referenceLine::gpsCallback(const l5player_nop_msgs::msg::Gps& pGps) {
    gps_ = pGps;
    gps_flag_ = {1};
}

// 非referenceLine成员函数
// 已知顶点和中心点，将任何障碍物变成方形的二维平面，计算长和宽,并存储于Marker
void Turn_obstacles_into_squares(visualization_msgs::msg::Marker& marker, const Obstacle* Primitive_obstacle,
                                 const int id) {
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY((double)0.0, (double)0.0, (double)(Primitive_obstacle->obstacle_theta));
    marker.pose.orientation.set__w(myQuaternion.getW());
    marker.pose.orientation.set__x(myQuaternion.getX());
    marker.pose.orientation.set__y(myQuaternion.getY());
    marker.pose.orientation.set__z(myQuaternion.getZ());

    marker.header.frame_id = "world";
    // marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = Primitive_obstacle->centerpoint.position.x;
    marker.pose.position.y = Primitive_obstacle->centerpoint.position.y;
    marker.pose.position.z = 0;
    marker.scale.x = Primitive_obstacle->obstacle_length;
    marker.scale.y = Primitive_obstacle->obstacle_width;
    marker.scale.z = Primitive_obstacle->obstacle_height;
    marker.color.r = 255.0f;
    marker.color.g = 140.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // marker.lifetime = ros::Duration();
}

void referenceLine::run() {
    // 订阅障碍物
    Obstacle ob(true);    // 实例化类，构造函数订阅障碍物
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        if (referenceline_.poses.size() > 0) {
            referencePointsCalc(referenceline_);    // 计算参考点的kappa、theta，添加flag确保只计算一次，减少重复计算
        }

        // 障碍物存储
        std::vector<const Obstacle*> obstacles;    // Apollo是这种类型
        for (size_t i = 0; i < AllObstacle.size(); ++i) {
            obstacles.emplace_back(&AllObstacle[i]);
        }

        // 障碍物发布rviz
        if (obstacles.size() > 0) {
            visualization_msgs::msg::MarkerArray obstacle_MarkerArray;
            for (int i = 0; i < obstacles.size(); ++i) {
                visualization_msgs::msg::Marker marker;
                Turn_obstacles_into_squares(marker, obstacles[i], i);
                obstacle_MarkerArray.markers.push_back(marker);
            }
            // rviz_obstacle_pub_.publish(obstacle_MarkerArray);
        }
        // 确保gps和reference_points收到数据之后再进行
        if (reference_points.size() > 0 && gps_flag_.size() > 0) {
            // double current_time = node_refline_->get_clock()->now().seconds();
            double current_time;
            std::vector<TrajectoryPoint> stitching_trajectory = plan_start_point(current_time);
            // 轨迹生成
            PlanningTarget planning_target(Config_.default_cruise_speed, accumulated_s);    // 目标m/s
            lon_decision_horizon = accumulated_s[accumulated_s.size() - 1];
            best_path_ = planning_base_->plan(stitching_trajectory.back(), planning_target, obstacles, accumulated_s,
                                              reference_points, FLAGS_lateral_optimization, init_relative_time,
                                              lon_decision_horizon, plan_start_time);
            // ROS_INFO("current_time %f", current_time);
            // ROS_INFO("plan_start_time = %f", plan_start_time);
            pre_trajectory_.clear();
            pre_trajectory_.insert(
                pre_trajectory_.end(), stitching_trajectory.begin(),
                stitching_trajectory.end() - 1);    // 规划起点被同时包含在了stitching和best_path中，因此需要减1
            // ROS_INFO("stitch_.size = %i", stitching_trajectory.size());
            // ROS_INFO("stitch_pre_trajectory_.size = %i", pre_trajectory_.size());
            pre_trajectory_.insert(pre_trajectory_.end(), best_path_.begin(), best_path_.end());
            // ROS_INFO("best_stitch_pre_trajectory_.size = %i", pre_trajectory_.size());

            // for (int i = 0; i < pre_trajectory_.size();++i){
            //   ROS_INFO("relative_time_%i = %f absolute_time = %f", i, pre_trajectory_[i].relative_time,
            //   pre_trajectory_[i].absolute_time);
            // }

            // 发布轨迹
            l5player_nop_msgs::msg::Trajectory trajectory_d;
            trajectory_d.trajectorypoint.clear();
            trajectory_d.pointsize = pre_trajectory_.size();
            for (int i = 0; i < pre_trajectory_.size(); ++i) {
                l5player_nop_msgs::msg::TrajectoryPoint TrajectoryPoint_d;
                TrajectoryPoint_d.x = pre_trajectory_[i].x;
                TrajectoryPoint_d.y = pre_trajectory_[i].y;
                TrajectoryPoint_d.z = pre_trajectory_[i].z;
                TrajectoryPoint_d.theta = pre_trajectory_[i].theta;
                TrajectoryPoint_d.kappa = pre_trajectory_[i].kappa;
                TrajectoryPoint_d.dkappa = pre_trajectory_[i].dkappa;
                TrajectoryPoint_d.v = pre_trajectory_[i].v;
                TrajectoryPoint_d.a = pre_trajectory_[i].a;
                TrajectoryPoint_d.relative_time = pre_trajectory_[i].relative_time;
                TrajectoryPoint_d.absolute_time = pre_trajectory_[i].absolute_time;
                TrajectoryPoint_d.d = pre_trajectory_[i].d;
                TrajectoryPoint_d.d_d = pre_trajectory_[i].d_d;
                TrajectoryPoint_d.d_dd = pre_trajectory_[i].d_dd;
                TrajectoryPoint_d.s = pre_trajectory_[i].s;
                TrajectoryPoint_d.s_d = pre_trajectory_[i].s_d;
                TrajectoryPoint_d.s_dd = pre_trajectory_[i].s_dd;
                TrajectoryPoint_d.s_ddd = pre_trajectory_[i].s_ddd;
                TrajectoryPoint_d.d_ddd = pre_trajectory_[i].d_ddd;
                trajectory_d.trajectorypoint.emplace_back(TrajectoryPoint_d);
            }

            traj_points_.poses.clear();
            traj_points_.header.frame_id = "world";
            traj_points_.header.stamp = steady_clock_.now();
            for (int i = 0; i < pre_trajectory_.size(); ++i) {
                geometry_msgs::msg::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = "world";
                pose_stamp.header.stamp = steady_clock_.now();
                pose_stamp.pose.position.x = pre_trajectory_[i].x;
                pose_stamp.pose.position.y = pre_trajectory_[i].y;
                pose_stamp.pose.position.z = 0;
                traj_points_.poses.push_back(pose_stamp);
            }

            // trajectory_pub_.publish(trajectory_d);
            // rviz_pub_.publish(traj_points_);    // rviz可视化
        }

        // rclcpp::spin_once();
        loop_rate.sleep();
    }
}

void referenceLine::referenceLine_split(Eigen::MatrixXd& hdmap_way_points) {
    average_interpolation(hdmap_way_points, path_point_after_interpolation_, 0.2, 0.6);
    Smooth(path_point_after_interpolation_);
}

void referenceLine::average_interpolation(Eigen::MatrixXd& input, Eigen::MatrixXd& output, double interval_dis,
                                          double distance)    // 0.2 0.6
{
    // 1.定义一个容器，类型为Point3d_s,即（x,y,z）
    std::vector<Point3d_s> vec_3d;
    std::vector<Point3d_s> n_vec;
    Point3d_s p;
    // 2.遍历
    // std::cout << " input.rows()" << input.rows() << std::endl;
    for (int i = 0; i < input.rows() - 1; i++) {
        double dis = (input.row(i + 1) - input.row(i)).norm();    // 求两点的距离，前一行和这一行坐标的距离
        // std::cout << "dis " << dis << std::endl;
        // 两点距离太长的话就进行插点
        if (dis >= distance) {
            // 计算(x,y)两点的距离
            double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
                                   (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
            // 计算角度
            double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
            double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
            // 两点之间要插值的插值点的数量
            int num = dis / interval_dis;    // 分割了一下
            // std::cout << "num " << num << std::endl;
            // 插入点
            for (int j = 0; j < num; j++) {
                // i=0,j=0的时候其实是插入起点
                p.x = input(i, 0) + j * interval_dis * cos_a;
                p.y = input(i, 1) + j * interval_dis * sin_a;
                p.z = input(i, 2);
                vec_3d.push_back(p);
            }
        }
        // 3.有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
        else if (dis < distance) {
            p.x = input(i, 0);
            p.y = input(i, 1);
            p.z = input(i, 2);
            vec_3d.push_back(p);
        }
    }
    // 4.漏了终点，需要加上
    p.x = input(input.rows() - 1, 0);
    p.y = input(input.rows() - 1, 1);
    p.z = input(input.rows() - 1, 2);
    vec_3d.push_back(p);

    // 传给输出矩阵output
    output = Eigen::MatrixXd::Zero(vec_3d.size(), 3);
    int j = 0;
    for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++) {
        output(j, 0) = (*it).x;
        output(j, 1) = (*it).y;
        output(j, 2) = (*it).z;
        j++;
    }
}

void referenceLine::Smooth(Eigen::MatrixXd& path_point_after_interpolation_) {
    referenceline_.poses.clear();
    referenceline_.header.frame_id = "world";
    referenceline_.header.stamp = steady_clock_.now();
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "world";
    pose_stamp.header.stamp = steady_clock_.now();

    //////////////优化参考路径点///////////////
    bool status = false;                                   // 是否平滑成功标志
    std::vector<std::pair<double, double>> raw_point2d;    // 参考路径点
    std::vector<double> box_bounds;                        // 边界条件
    // 获取原始参考路径点和边界值
    for (int i = 0; i < path_point_after_interpolation_.rows(); i++) {
        raw_point2d.emplace_back(path_point_after_interpolation_(i, 0), path_point_after_interpolation_(i, 1));
        box_bounds.emplace_back(0.25);
    }
    // fix front and back points to avoid end states deviate from the center of road
    box_bounds.front() = 0.0;
    box_bounds.back() = 0.0;

    // 标准化路径点坐标，以第一个点为基准
    NormalizePoints(&raw_point2d);

    // box contraints on pos are used in cos theta smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    std::vector<double> bounds = box_bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto& bound : bounds) {
        bound *= box_ratio;
    }

    std::vector<std::pair<double, double>> smoothed_point2d;
    std::vector<double> opt_x;
    std::vector<double> opt_y;
    // 使用CosThetaSmoother
    if (which_smoothers == true) {
        // ROS_WARN("CosThetaSmoother");
        status = CosThetaSmooth(raw_point2d, bounds, &opt_x, &opt_y);
    }
    // 使用FemPosSmooth
    else if (which_smoothers == false)    // 默认是false
    {
        // ROS_WARN("FemPosDeviationSmoother");
        status = FemPosSmooth(raw_point2d, bounds, &opt_x, &opt_y);
    }
    if (status == false)    // 求解失败
    {
        // ROS_WARN("reference line smoothing failed!");
    } else    // 求解成功
    {
        for (size_t i = 0; i < opt_x.size(); i++) {
            smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
        }
        DeNormalizePoints(&smoothed_point2d);

        // 发布
        for (int i = 0; i < smoothed_point2d.size(); i++) {
            pose_stamp.pose.position.x = smoothed_point2d[i].first;
            pose_stamp.pose.position.y = smoothed_point2d[i].second;
            pose_stamp.pose.position.z = 0;
            referenceline_.poses.push_back(pose_stamp);
        }
    }
}

void referenceLine::NormalizePoints(std::vector<std::pair<double, double>>* xy_points) {
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double>& point) {
        auto curr_x = point.first;
        auto curr_y = point.second;
        std::pair<double, double> xy(curr_x - zero_x_, curr_y - zero_y_);
        point = std::move(xy);
    });
}

void referenceLine::DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points) {
    std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double>& point) {
        auto curr_x = point.first;
        auto curr_y = point.second;
        std::pair<double, double> xy(curr_x + zero_x_, curr_y + zero_y_);
        point = std::move(xy);
    });
}

void referenceLine::referencePointsCalc(const nav_msgs::msg::Path& path_point) {
    // 计算参考点的kappa、theta，只需要计算一次
    if (path_point.poses.size() > 0 && path_point_flag_.size() < 1) {
        reference_path.first.clear();
        reference_path.second.clear();
        accumulated_s.clear();
        reference_points.clear();

        path_point_flag_.push_back(path_point.poses[0].pose.position.x);
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<std::pair<double, double>> xy_points;

        // auto beforeTime = std::chrono::steady_clock::now(); //计时开始
        for (size_t i = 0; i < path_point.poses.size(); ++i) {
            reference_path.first.push_back(path_point.poses[i].pose.position.x);
            reference_path.second.push_back(path_point.poses[i].pose.position.y);
            xy_points.emplace_back(path_point.poses[i].pose.position.x, path_point.poses[i].pose.position.y);
        }

        if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
            // ROS_WARN("rerferenceline generate failed!");
            std::cout << "rerferenceline generate failed!" << std::endl;
        } else {
            for (size_t i = 0; i < xy_points.size(); ++i) {
                // 创建ReferencePoint类
                ReferencePoint reference_point(kappas[i], dkappas[i], xy_points[i].first, xy_points[i].second,
                                               headings[i], accumulated_s[i]);
                reference_points.emplace_back(reference_point);
            }
        }
    }
}

std::vector<TrajectoryPoint> referenceLine::plan_start_point(double& current_time) {
    if (this->is_first_run) {
        // 第一次运行
        std::cout << "第1次运行" << std::endl;
        this->is_first_run = false;
        double x_init = gps_.posx;    // 定位
        double y_init = gps_.posy;
        double z_init = 0;
        double v_init = 0;
        double a_init = 0;
        double theta_init = gps_.oriz;
        double kappa_init = 0;
        double dkappa_init = 0;
        double s0 = 0;
        this->init_relative_time = 0.0;
        plan_start_time = current_time + 0.1;    // start point absolute time

        TrajectoryPoint init_point;
        init_point.set_s(0.0);
        init_point.set_x(x_init);
        init_point.set_y(y_init);
        init_point.set_z(z_init);
        init_point.set_theta(theta_init);
        init_point.set_kappa(kappa_init);
        init_point.set_v(v_init);
        init_point.set_a(a_init);
        init_point.set_relative_time(0.0);
        return std::vector<TrajectoryPoint>(1, init_point);
    } else {
        // 非第一次运行
        std::cout << "非第一次运行且有历史轨迹" << std::endl;
        double x_cur = gps_.posx;    // 定位
        double y_cur = gps_.posy;
        double theta_cur = gps_.oriz;
        double kappa_cur = 0;
        double vx_cur = gps_.velx;
        double vy_cur = gps_.vely;
        double ax_cur = gps_.accelx;
        double ay_cur = gps_.accely;
        double dt = 0.1;
        int index = 0;

        for (int i = 0; i < pre_trajectory_.size() - 1; ++i) {
            if (pre_trajectory_[i].absolute_time <= current_time &&
                pre_trajectory_[i + 1].absolute_time > current_time) {
                index = i;
                break;
            }
        }
        // 上一周期规划的本周期车辆应该在的位置
        double pre_x_desire = pre_trajectory_[index].x;
        double pre_y_desire = pre_trajectory_[index].y;
        double pre_theta_desire = pre_trajectory_[index].theta;
        // 计算横纵向误差
        Eigen::Matrix<double, 2, 1> tor;
        tor << cos(pre_theta_desire), sin(pre_theta_desire);
        Eigen::Matrix<double, 2, 1> nor;
        nor << -sin(pre_theta_desire), cos(pre_theta_desire);
        // 误差向量
        Eigen::Matrix<double, 2, 1> d_err;
        d_err << x_cur - pre_x_desire, y_cur - pre_y_desire;
        double lon_err = abs(d_err.transpose() * tor);
        double lat_err = abs(d_err.transpose() * nor);

        std::size_t position_matched_index = pre_trajectory_.QueryNearestPoint({x_cur, y_cur});
        std::cout << "pre_trajectory_ size : " << pre_trajectory_.size() << std::endl;
        std::cout << "poisition match point index: " << position_matched_index << std::endl;
        std::cout << "time match point index: " << index << std::endl;
        printf("pre_x_desire = %f m\n", pre_x_desire);    // 局部路径
        printf("pre_y_desire = %f m\n", pre_y_desire);
        printf("lon_err = %f m\n", lon_err);
        printf("lat_err = %f m\n", lat_err);
        // 如果纵向误差大于2.5或者横向误差大于0.5认为控制没跟上
        if (lon_err > 2.5 || lat_err > 0.5) {
            // ROS_INFO("out of control, use dynamic to deduce");
            std::cout << "ax_cur = " << ax_cur << " ay_cur = " << ay_cur << std::endl;
            double x_init = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
            double y_init = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
            double z_init = 0;
            double v_init = std::sqrt(std::pow(vy_cur + ay_cur * dt, 2) + std::pow(vx_cur + ax_cur * dt, 2));
            double a_init = std::sqrt(ax_cur * ax_cur + ay_cur * ay_cur);
            double theta_init = atan2(vy_cur + ay_cur * dt, vx_cur + ax_cur * dt);
            double kappa_init = kappa_cur;
            double dkappa_init = 0;
            double s0 = 0;
            this->init_relative_time = 0.0;

            plan_start_time = current_time + 0.1;

            TrajectoryPoint init_point;
            init_point.set_s(0.0);
            init_point.set_x(x_init);
            init_point.set_y(y_init);
            init_point.set_z(z_init);
            init_point.set_theta(theta_init);
            init_point.set_kappa(kappa_init);
            init_point.set_v(v_init);
            init_point.set_a(a_init);
            init_point.set_relative_time(0.0);
            return std::vector<TrajectoryPoint>(1, init_point);
        } else {
            // ROS_INFO("good control");
            int index_good = 0;
            for (int i = index; i < pre_trajectory_.size() - 1; ++i) {
                if (pre_trajectory_[i].absolute_time <= current_time + 0.1 &&
                    pre_trajectory_[i + 1].absolute_time > current_time + 0.1) {
                    index_good = i;
                    std::cout << "forward time match point index:: " << i << std::endl;
                    break;
                }
            }
            double x_init = pre_trajectory_[index_good].x;
            double y_init = pre_trajectory_[index_good].y;
            double z_init = 0;
            double v_init = pre_trajectory_[index_good].v;
            double a_init = pre_trajectory_[index_good].a;
            double theta_init = pre_trajectory_[index_good].theta;
            double kappa_init = pre_trajectory_[index_good].kappa;
            double dkappa_init = pre_trajectory_[index_good].dkappa;
            // double s0 = pre_trajectory_[index_good].s;
            // double ds0 = pre_trajectory_[index_good].s_d;
            // double dd0 = pre_trajectory_[index_good].d_d;
            // double d0 = pre_trajectory_[index_good].d;
            // double dds0 = pre_trajectory_[index_good].s_dd;
            // double ddd0 = pre_trajectory_[index_good].d_dd;
            plan_start_time = pre_trajectory_[index_good].absolute_time;

            TrajectoryPoint init_point;
            init_point.set_s(0.0);
            init_point.set_x(x_init);
            init_point.set_y(y_init);
            init_point.set_z(z_init);
            init_point.set_theta(theta_init);
            init_point.set_kappa(kappa_init);
            init_point.set_v(v_init);
            init_point.set_a(a_init);
            init_point.set_relative_time(0.0);
            return std::vector<TrajectoryPoint>(1, init_point);

            // 轨迹拼接

            // auto matched_index = std::min(index, static_cast<int>(position_matched_index));//
            // 时间匹配点和位置匹配点index最小的 std::vector<TrajectoryPoint>
            // stitching_trajectory(pre_trajectory_.begin() + matched_index, pre_trajectory_.begin() + index_good);
            // std::vector<TrajectoryPoint> stitching_trajectory;
            // stitching_trajectory.clear();
            // if (matched_index > 20)
            // {
            //   stitching_trajectory.insert(stitching_trajectory.end(), pre_trajectory_.begin() + matched_index - 19,
            //                                                           pre_trajectory_.begin() + matched_index);//
            //                                                           取不到右端点因此需要加1

            // }
            // else
            // {
            //   stitching_trajectory.insert(stitching_trajectory.end(), pre_trajectory_.begin(),
            //                                                           pre_trajectory_.begin() + matched_index);//
            //                                                           取不到右端点因此需要加1
            // }

            // // auto time_matched_point = pre_trajectory_.TrajectoryPointAt(index_good);
            // const double zero_s = stitching_trajectory[0].path_point().s;
            // const double zero_time = stitching_trajectory[0].relative_time;
            // for (auto &tp : stitching_trajectory)
            // {
            //   // if (!tp.has_path_point()) {
            //   //   return ComputeReinitStitchingTrajectory(vehicle_state);
            //   // }
            //   tp.set_relative_time(tp.relative_time - zero_time);
            //   // tp.set_absolute_time(tp.absolute_time -
            //   //                      (pre_trajectory_[std::max(0, static_cast<int>(matched_index -
            //   1))].absolute_time + pre_trajectory_[0].absolute_time)); tp.set_s(tp.path_point().s - zero_s);
            //   std::cout << "tp.s = " << tp.s << "  tp.relative_time = " << tp.relative_time << std::endl;
            // }
            // this->init_relative_time = stitching_trajectory[stitching_trajectory.size() - 1].relative_time;
            // std::cout << "this->init_relative_time = " << this->init_relative_time << std::endl;
            // return stitching_trajectory;
        }
    }
}

bool referenceLine::is_update_dynamic(nav_msgs::msg::Path& trj_point_array, int size) {
    bool is_update;
    is_update = false;
    // 车的当前位置
    double pp_x = gps_.posx;
    double pp_y = gps_.posy;
    double xx = trj_point_array.poses[size].pose.position.x;
    double yy = trj_point_array.poses[size].pose.position.y;
    double distance = sqrt(pow(pp_x - xx, 2) + pow(pp_y - yy, 2));
    if (distance < 2)    // 接近了
    {
        is_update = true;
    }
    return is_update;
}
bool referenceLine::CosThetaSmooth(const std::vector<std::pair<double, double>>& raw_point2d,
                                   const std::vector<double>& bounds, std::vector<double>* opt_x,
                                   std::vector<double>* opt_y) {
    const double weight_cos_included_angle = 10000.0;
    const double weight_anchor_points = 1.0;
    const double weight_length = 1.0;
    const size_t print_level = 0;
    const size_t max_num_of_iterations = 500;
    const size_t acceptable_num_of_iterations = 15;
    const double tol = 1e-8;
    const double acceptable_tol = 1e-1;
    const bool use_automatic_differentiation = true;

    CosThetaIpoptInterface* smoother = new CosThetaIpoptInterface(raw_point2d, bounds);

    smoother->set_weight_cos_included_angle(weight_cos_included_angle);
    smoother->set_weight_anchor_points(weight_anchor_points);
    smoother->set_weight_length(weight_length);
    smoother->set_automatic_differentiation_flag(use_automatic_differentiation);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue("print_level", static_cast<int>(print_level));
    app->Options()->SetIntegerValue("max_iter", static_cast<int>(max_num_of_iterations));
    app->Options()->SetIntegerValue("acceptable_iter", static_cast<int>(acceptable_num_of_iterations));
    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", acceptable_tol);

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        std::cout << "*** Error during initialization!\n";
        return false;
    }

    status = app->OptimizeTNLP(problem);    // 还没写！

    if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level) {
        // Retrieve some statistics about the solve
        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        // std::cout << "*** The problem solved in " << iter_count << " iterations!\n";
    } else {
        std::cout << "Solver fails with return code: " << static_cast<int>(status) << std::endl;
        return false;
    }
    smoother->get_optimization_results(opt_x, opt_y);
    return true;
}

bool referenceLine::FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d,
                                 const std::vector<double>& bounds, std::vector<double>* opt_x,
                                 std::vector<double>* opt_y) {
    if (apply_curvature_constraint) {
        if (use_sqp) {
            return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
        } else {
            return NlpWithIpopt(raw_point2d, bounds, opt_x, opt_y);
        }
    } else {
        return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }
    return true;
}

bool referenceLine::QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                               const std::vector<double>& bounds, std::vector<double>* opt_x,
                               std::vector<double>* opt_y) {
    if (opt_x == nullptr || opt_y == nullptr) {
        std::cout << "opt_x or opt_y is nullptr";
        return false;
    }

    FemPosDeviationOsqpInterface solver;

    double weight_fem_pos_deviation = 1.0e10;
    double weight_ref_deviation = 1.0;
    double weight_path_length = 1.0;

    // osqp settings
    int max_iter = 500;
    // time_limit set to be 0.0 meaning no time limit
    double time_limit = 0.0;
    bool verbose = false;
    bool scaled_termination = true;
    bool warm_start = true;

    solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    solver.set_weight_path_length(weight_path_length);
    solver.set_weight_ref_deviation(weight_ref_deviation);

    solver.set_max_iter(max_iter);
    solver.set_time_limit(time_limit);
    solver.set_verbose(verbose);
    solver.set_scaled_termination(scaled_termination);
    solver.set_warm_start(warm_start);

    solver.set_ref_points(raw_point2d);
    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
        return false;
    }

    *opt_x = solver.opt_x();
    *opt_y = solver.opt_y();
    return true;
}

bool referenceLine::SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                                const std::vector<double>& bounds, std::vector<double>* opt_x,
                                std::vector<double>* opt_y) {
    if (opt_x == nullptr || opt_y == nullptr) {
        std::cout << "opt_x or opt_y is nullptr";
        return false;
    }

    FemPosDeviationSqpOsqpInterface solver;

    double weight_fem_pos_deviation = 1.0e10;
    double weight_ref_deviation = 1.0;
    double weight_path_length = 1.0;
    double weight_curvature_constraint_slack_var = 1.0e2;
    double curvature_constraint = 0.2;
    double sqp_ftol = 1e-4;
    double sqp_ctol = 1e-3;
    int sqp_pen_max_iter = 10;
    int sqp_sub_max_iter = 100;

    // osqp settings
    int max_iter = 500;
    // time_limit set to be 0.0 meaning no time limit
    double time_limit = 0.0;
    bool verbose = false;
    bool scaled_termination = true;
    bool warm_start = true;

    solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    solver.set_weight_path_length(weight_path_length);
    solver.set_weight_ref_deviation(weight_ref_deviation);
    solver.set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);

    solver.set_curvature_constraint(curvature_constraint);

    solver.set_sqp_sub_max_iter(sqp_sub_max_iter);
    solver.set_sqp_ftol(sqp_ftol);
    solver.set_sqp_pen_max_iter(sqp_pen_max_iter);
    solver.set_sqp_ctol(sqp_ctol);

    solver.set_max_iter(max_iter);
    solver.set_time_limit(time_limit);
    solver.set_verbose(verbose);
    solver.set_scaled_termination(scaled_termination);
    solver.set_warm_start(warm_start);

    solver.set_ref_points(raw_point2d);
    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
        return false;
    }

    std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

    // TODO(Jinyun): unify output data container
    opt_x->resize(opt_xy.size());
    opt_y->resize(opt_xy.size());
    for (size_t i = 0; i < opt_xy.size(); ++i) {
        (*opt_x)[i] = opt_xy[i].first;
        (*opt_y)[i] = opt_xy[i].second;
    }
    return true;
}

bool referenceLine::NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d,
                                 const std::vector<double>& bounds, std::vector<double>* opt_x,
                                 std::vector<double>* opt_y) {
    if (opt_x == nullptr || opt_y == nullptr) {
        std::cout << "opt_x or opt_y is nullptr";
        return false;
    }

    double weight_fem_pos_deviation = 1.0e10;
    double weight_ref_deviation = 1.0;
    double weight_path_length = 1.0;
    double weight_curvature_constraint_slack_var = 1.0e2;
    double curvature_constraint = 0.2;
    // ipopt settings
    int print_level = 0;
    int max_num_of_iterations = 500;
    int acceptable_num_of_iterations = 15;
    double tol = 1e-8;
    double acceptable_tol = 1e-1;

    FemPosDeviationIpoptInterface* smoother = new FemPosDeviationIpoptInterface(raw_point2d, bounds);

    smoother->set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    smoother->set_weight_path_length(weight_path_length);
    smoother->set_weight_ref_deviation(weight_ref_deviation);
    smoother->set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);
    smoother->set_curvature_constraint(curvature_constraint);

    Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

    // Create an instance of the IpoptApplication
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue("print_level", static_cast<int>(print_level));
    app->Options()->SetIntegerValue("max_iter", static_cast<int>(max_num_of_iterations));
    app->Options()->SetIntegerValue("acceptable_iter", static_cast<int>(acceptable_num_of_iterations));
    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", acceptable_tol);

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        std::cout << "*** Error during initialization!";
        return false;
    }

    status = app->OptimizeTNLP(problem);

    if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level) {
        // Retrieve some statistics about the solve
        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        std::cout << "*** The problem solved in " << iter_count << " iterations!";
    } else {
        std::cout << "Solver fails with return code: " << static_cast<int>(status);
        return false;
    }
    smoother->get_optimization_results(opt_x, opt_y);
    return true;
}

}    // namespace reference_line
}    // namespace l5player
