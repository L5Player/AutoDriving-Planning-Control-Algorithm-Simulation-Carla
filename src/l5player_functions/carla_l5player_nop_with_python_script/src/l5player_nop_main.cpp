#include <math.h>

#include <fstream>

#include "l5player_nop/common.h"
#include "l5player_nop/nop_function_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

// using namespace l5player::planning;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("carla_l5player_pid_new_controller_publisher");

// Controller
l5player::control::PIDController yaw_pid_controller(0.5, 0.3, 0.1);    // 转向角pid
// l5player::control::PIDController speed_pid_controller(0.206, 0.0206, 0.515);    // 速度pid Kp Ki Kd
// l5player::control::PIDController speed_pid_controller(0.16, 0.02, 0.01);    // 速度pid Kp Ki Kd

NopFunctionNode::NopFunctionNode()
    : Node("carla_l5player_nop_with_python_script")
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    V_set_ = 30;    // km/h
    first_record_ = true;
    cnt = 0;
    qos = 10;

    // controler
    pid_controller_longitudinal_ = std::make_unique<l5player::control::PIDController>(0.16, 0.02, 0.01);
    lqr_controller_lateral_ = std::make_unique<l5player::control::LqrController>();
    lqr_controller_lateral_->LoadControlConf();
    lqr_controller_lateral_->Init();
    stanely_controller_lateral_ = std::make_unique<l5player::control::StanleyController>();
    stanely_controller_lateral_->LoadControlConf();

    switch (which_planners) {
        case 0:
            std::cout << "lattice planning!!!" << std::endl;
            planning_base_ = std::make_shared<l5player::planning::lattice>();
            break;
        case 1:

            break;
        default:
            std::cout << "default:lattice planning!!!" << std::endl;
            planning_base_ = std::make_shared<l5player::planning::lattice>();
            break;
    }

    // RCLCPP_INFO(LOGGER, "NopFunctionNode test 1");
    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", qos, std::bind(&NopFunctionNode::odomCallback, this, _1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/carla/ego_vehicle/imu", qos, std::bind(&NopFunctionNode::ImuCallback, this, _1));

    control_cmd.header.stamp = this->now();
    control_cmd.gear = 1;
    control_cmd.manual_gear_shift = false;
    control_cmd.reverse = false;
    control_cmd.hand_brake = false;

    auto time_node_start = this->now();
    vehicle_control_target_velocity_publisher =
        this->create_publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>("/carla/ego_vehicle/target_velocity", qos);
    vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = 0.0;

    carla_status_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
        "/carla/ego_vehicle/vehicle_status", qos, std::bind(&NopFunctionNode::VehicleStatusCallback, this, _1));

    carla_vehicle_object_subscriber = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
        "/carla/ego_vehicle/objects", qos, std::bind(&NopFunctionNode::SetObjectArrayCallback, this, _1));

    // RCLCPP_INFO(LOGGER, "NopFunctionNode test 2");

    // 读取参考线路径
    std::ifstream infile(
        "/home/bea20/l5player_premium/auto-driving-planning-control-algorithm-simulation-carla_premium/src/"
        "l5player_functions/"
        "carla_l5player_nop_with_python_script/data/"
        "2022_09_29_16_27_08_ins_data_map_after_preprocess.csv",
        std::ios::in);           // 将文件流对象与文件连接起来
    assert(infile.is_open());    // 若失败,则输出错误消息,并终止程序运行

    while (getline(infile, _line)) {
        // std::cout << _line << std::endl;
        // 解析每行的数据
        std::stringstream ss(_line);
        std::string _sub;
        std::vector<std::string> subArray;
        // 按照逗号分隔
        while (getline(ss, _sub, ',')) {
            subArray.push_back(_sub);
        }
        double pt_x = std::atof(subArray[2].c_str());
        double pt_y = std::atof(subArray[3].c_str());

        // 设置巡航车速
        double pt_v = static_cast<double>(AEB_TEST_CRUISE_SPEED);

        v_points.push_back(pt_v);
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }
    infile.close();

    // 平滑整个导航路径的参考线
    routing_waypoints_ = Eigen::MatrixXd::Zero(xy_points.size(), 3);
    for (size_t i = 0; i < xy_points.size(); ++i) {
        routing_waypoints_(i, 0) = xy_points.at(i).first;
        routing_waypoints_(i, 1) = xy_points.at(i).second;
        routing_waypoints_(i, 2) = (double)0.0;
    }
    rl_.referenceLine_split(routing_waypoints_);

    auto referenceline_ptr = rl_.get_referenceline();
    rl_.referencePointsCalc(*referenceline_ptr);    // 计算参考点的kappa、theta，确保只计算一次，减少重复计算

    acceleration_cmd = 0.0;
    steer_cmd = 0.0;

    vehicle_control_publisher =
        this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", qos);
    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_reference_path", 2);
    nop_planed_traj_publisher = this->create_publisher<nav_msgs::msg::Path>("/l5player_nop_planed_path", 2);
    rviz_obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/l5player_nop/obstacle", 2);
    nop_rl_points_publisher_ = this->create_publisher<nav_msgs::msg::Path>("l5player_nop_rl_points", 2);
    nop_debug_info_publisher_ =
        this->create_publisher<l5player_nop_msgs::msg::NopFunctionDebugInfo>("l5player_nop_debuginfo", 2);

    global_path_publish_timer =
        this->create_wall_timer(500ms, std::bind(&NopFunctionNode::GlobalPathPublishCallback, this));

    // timer for control
    // vehicle_control_iteration_timer_ =
    //     this->create_wall_timer(50ms, std::bind(&NopFunctionNode::VehicleControlIterationCallback, this));

    // Initialize the transform broadcaster
    tf_broadcaster_gps_vehicle = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void NopFunctionNode::NopRunOnce() {
    // 障碍物存储
    std::cout << "==========NopRunOnce==========" << std::endl;
    std::vector<const Obstacle *> obstacles;    // Apollo是这种类型
    for (size_t i = 0; i < AllObstacle.size(); ++i) {
        obstacles.emplace_back(&AllObstacle.at(i));
    }

    // 障碍物发布rviz
    if (obstacles.size() > 0) {
        visualization_msgs::msg::MarkerArray obstacle_MarkerArray;
        for (int i = 0; i < obstacles.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            Turn_obstacles_into_squares(marker, obstacles[i], i);
            obstacle_MarkerArray.markers.push_back(marker);
        }
        rviz_obstacle_pub_->publish(obstacle_MarkerArray);
    }

    auto reference_points_ptr = rl_.GetReferencePoints();
    nav_msgs::msg::Path pub_rl_points_rviz;
    pub_rl_points_rviz.poses.clear();
    pub_rl_points_rviz.header.frame_id = "gps";
    pub_rl_points_rviz.header.stamp = this->get_clock()->now();

    for (int i = 0; i < reference_points_ptr->size(); ++i) {
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = "gps";
        pose_stamp.header.stamp = this->get_clock()->now();
        pose_stamp.pose.position.x = reference_points_ptr->at(i).get_x();
        pose_stamp.pose.position.y = reference_points_ptr->at(i).get_y();
        pose_stamp.pose.position.z = 0;
        pub_rl_points_rviz.poses.push_back(pose_stamp);
    }
    nop_rl_points_publisher_->publish(pub_rl_points_rviz);

    // plan
    if (reference_points_ptr->size() > 0) {
        double current_time = this->get_clock()->now().seconds();
        std::vector<TrajectoryPoint> stitching_trajectory = rl_.plan_start_point(current_time);
        std::cout << "planning point start point x: " << stitching_trajectory.at(0).x
                  << ", start point y: " << stitching_trajectory.at(0).y << std::endl;

        PlanningTarget planning_target(Config_.default_cruise_speed, rl_.accumulated_s);    // 目标m/s
        rl_.lon_decision_horizon = rl_.accumulated_s.back();
        rl_.best_path_ = planning_base_->plan(stitching_trajectory.back(), planning_target, obstacles,
                                              rl_.accumulated_s, *reference_points_ptr, rl_.FLAGS_lateral_optimization,
                                              rl_.init_relative_time, rl_.lon_decision_horizon, rl_.plan_start_time);

        // ROS_INFO("current_time %f", current_time);
        // ROS_INFO("plan_start_time = %f", plan_start_time);
        std::cout << "cruise speed: " << Config_.default_cruise_speed << std::endl;

        std::cout << "current_time: " << std::fixed << current_time << "  plan_start_time: " << std::fixed
                  << rl_.plan_start_time << std::endl;

        rl_.pre_trajectory_.clear();
        rl_.pre_trajectory_.insert(
            rl_.pre_trajectory_.end(), stitching_trajectory.begin(),
            stitching_trajectory.end() - 1);    // 规划起点被同时包含在了stitching和best_path中，因此需要减1
        // ROS_INFO("stitch_.size = %i", stitching_trajectory.size());
        // ROS_INFO("stitch_pre_trajectory_.size = %i", pre_trajectory_.size());
        rl_.pre_trajectory_.insert(rl_.pre_trajectory_.end(), rl_.best_path_.begin(), rl_.best_path_.end());
        // ROS_INFO("best_stitch_pre_trajectory_.size = %i", pre_trajectory_.size());

        // for (int i = 0; i < pre_trajectory_.size();++i){
        //   ROS_INFO("relative_time_%i = %f absolute_time = %f", i, pre_trajectory_[i].relative_time,
        //   pre_trajectory_[i].absolute_time);
        // }

        DiscretizedTrajectory &pre_trajectory_ = rl_.pre_trajectory_;

        // 发布plan生成轨迹
        // rl_.traj_points_.poses.clear();
        // rl_.traj_points_.header.frame_id = "gps";
        // rl_.traj_points_.header.stamp = this->get_clock()->now();
        // for (int i = 0; i < pre_trajectory_.size(); ++i) {
        //     geometry_msgs::msg::PoseStamped pose_stamp;
        //     pose_stamp.header.frame_id = "gps";
        //     pose_stamp.header.stamp = this->get_clock()->now();
        //     pose_stamp.pose.position.x = pre_trajectory_[i].x;
        //     pose_stamp.pose.position.y = pre_trajectory_[i].y;
        //     pose_stamp.pose.position.z = 0;
        //     rl_.traj_points_.poses.push_back(pose_stamp);
        // }
        // nop_planed_traj_publisher->publish(rl_.traj_points_);

        // Construct the reference_line path profile
        std::vector<double> headings;
        std::vector<double> accumulated_s;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<std::pair<double, double>> xy_points_temp;
        v_points.clear();

        for (size_t i = 0; i < pre_trajectory_.size(); i++) {
            headings.emplace_back(pre_trajectory_.at(i).theta);
            accumulated_s.emplace_back(pre_trajectory_.at(i).s);
            kappas.emplace_back(pre_trajectory_.at(i).kappa);
            dkappas.emplace_back(pre_trajectory_.at(i).dkappa);
            v_points.emplace_back(pre_trajectory_.at(i).v);
            xy_points_temp.emplace_back(std::make_pair(pre_trajectory_.at(i).x, pre_trajectory_.at(i).y));
        }

        size_t _count_points = headings.size();
        size_t _stop_begin_point = ceil(_count_points * 0.85);
        size_t _stop_point = ceil(_count_points * 0.95);
        std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

        int _index_before_stop = 0;
        planning_published_trajectory.trajectory_points.clear();
        for (size_t i = 0; i < headings.size(); i++) {
            TrajectoryPointOri trajectory_pt;
            trajectory_pt.x = xy_points_temp[i].first;
            trajectory_pt.y = xy_points_temp[i].second;
            if (i < _stop_begin_point) {
                trajectory_pt.v = v_points[i];
                _index_before_stop++;
            } else {
                if (trajectory_pt.v > 1.0) {
                    trajectory_pt.v = v_points[_index_before_stop] *
                                      ((double)i / ((double)_stop_begin_point - (double)_stop_point) -
                                       (double)_stop_point / ((double)_stop_begin_point - (double)_stop_point));
                } else {
                    trajectory_pt.v = 0;
                }
            }
            trajectory_pt.a = 0.0;
            trajectory_pt.heading = headings[i];
            trajectory_pt.kappa = kappas[i];

            planning_published_trajectory.trajectory_points.push_back(trajectory_pt);
        }

        trajectory_points_.clear();
        if (Config_.FLAGS_if_use_local_planner) {
            trajectory_points_ = planning_published_trajectory.trajectory_points;
            // 发布最终control使用轨迹
            nav_msgs::msg::Path pub_planed_traj_rviz;
            pub_planed_traj_rviz.poses.clear();
            pub_planed_traj_rviz.header.frame_id = "gps";
            pub_planed_traj_rviz.header.stamp = this->get_clock()->now();
            for (int i = 0; i < trajectory_points_.size(); ++i) {
                geometry_msgs::msg::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = "gps";
                pose_stamp.header.stamp = this->get_clock()->now();
                pose_stamp.pose.position.x = trajectory_points_[i].x;
                pose_stamp.pose.position.y = trajectory_points_[i].y;
                pose_stamp.pose.position.z = 0;
                pub_planed_traj_rviz.poses.push_back(pose_stamp);
            }
            nop_planed_traj_publisher->publish(pub_planed_traj_rviz);
        } else {
            auto reference_points_ptr = rl_.GetReferencePoints();
            if (reference_points_ptr->size() == static_cast<size_t>(0)) {
                std::cout << "dont have reference_points , return false ! " << std::endl;
                return;
            }
            auto index = QueryNearestPointIndexByPositionOfRefLine(vehicle_state_.x, vehicle_state_.y);
            std::cout << "control relline index is: " << index
                      << " , x is : " << reference_points_ptr->at(index).get_x()
                      << " , y is : " << reference_points_ptr->at(index).get_y() << std::endl;

            nav_msgs::msg::Path pub_planed_traj_rviz;
            pub_planed_traj_rviz.poses.clear();
            pub_planed_traj_rviz.header.frame_id = "gps";
            pub_planed_traj_rviz.header.stamp = this->get_clock()->now();
            std::size_t max_index = reference_points_ptr->size() > static_cast<size_t>(max_index + 500)
                                        ? static_cast<size_t>(max_index + 500)
                                        : reference_points_ptr->size();
            for (std::size_t i = index; i < max_index; ++i) {
                // 生成control使用的轨迹
                TrajectoryPointOri traj_point{0};
                traj_point.a = 0.0;
                traj_point.heading = reference_points_ptr->at(i).heading();
                traj_point.kappa = reference_points_ptr->at(i).kappa();
                traj_point.v = Config_.default_cruise_speed;
                traj_point.x = reference_points_ptr->at(i).get_x();
                traj_point.y = reference_points_ptr->at(i).get_y();
                trajectory_points_.emplace_back(traj_point);
                // 发布最终control使用轨迹
                geometry_msgs::msg::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = "gps";
                pose_stamp.header.stamp = this->get_clock()->now();
                pose_stamp.pose.position.x = reference_points_ptr->at(i).get_x();
                pose_stamp.pose.position.y = reference_points_ptr->at(i).get_y();
                pose_stamp.pose.position.z = 0;
                pub_planed_traj_rviz.poses.push_back(pose_stamp);
            }
            nop_planed_traj_publisher->publish(pub_planed_traj_rviz);
        }

        if (trajectory_points_.size() > static_cast<size_t>(0)) {
            std::cout << "first trajectory_points !!!!! : "
                      << " x: " << trajectory_points_.at(0).x << " y: " << trajectory_points_.at(0).y
                      << " v: " << trajectory_points_.at(0).v << std::endl;
        } else {
            std::cout << "first trajectory_points_size is 0 " << std::endl;
        }
    }

    // control
    VehicleControlIterationCallback();

    nop_debug_info_publisher_->publish(nop_debug_info_);
}

void NopFunctionNode::SetObjectArrayCallback(derived_object_msgs::msg::ObjectArray::SharedPtr msg) {
    const auto &objects = msg->objects;
    // 存到AllObstacle
    AllObstacle.clear();
    if (objects.size() > (size_t)0) {
        for (size_t i = 0; i < objects.size(); ++i) {
            Obstacle obs;
            /*-------------------------------------每种形状都有的基本信息----------------------------------------*/
            // 中心点
            obs.centerpoint.position.x = objects.at(i).pose.position.x;
            obs.centerpoint.position.y = objects.at(i).pose.position.y;
            obs.centerpoint.position.z = (double)0.0;            // 压缩二维

            obs.obstacle_id = objects.at(i).id;                  // id
            obs.obstacle_type = objects.at(i).classification;    // 类型

            // 角度
            geometry_msgs::msg::Quaternion obs_quaternion = objects.at(i).pose.orientation;
            obs.obstacle_theta = tf2::getYaw(obs_quaternion);
            obs.obstacle_velocity = objects.at(i).twist.linear.x;
            if (obs.obstacle_velocity > 0.2)    // 动态障碍物
            {
                auto oba = obs.GetOba();
                Prediction::Ob_Trajectory ob_tray =
                    oba->Generater_Trajectory(obs.centerpoint, 4, obs.obstacle_theta, obs.obstacle_velocity);
                obs.SetTrajectory(ob_tray);
            }

            /*---------------------------------------过滤不在有效区域的障碍物-------------------------------------------*/
            // PPoint centerpoint(obs.centerpoint.position.x, obs.centerpoint.position.y);
            // if (oba.Inside_rectangle(front_left, back_left, back_right, front_right, centerpoint) == false)
            // {
            //   continue; //跳过这个障碍物，进入下一个
            // }

            /*--------------------------------------不同形状有差别的信息-----------------------------------------*/
            // std::cout << "shape:" << msgs->objects[i].shape.type << std::endl;

            if (obs.obstacle_type == 6)        // Car
            {
                obs.pinnacle.poses.clear();    // 顶点暂时为空
                obs.obstacle_radius =
                    sqrt(pow(objects.at(i).shape.dimensions.at(0), 2) + pow(objects.at(i).shape.dimensions.at(1), 2)) /
                    2;
                // 长和宽
                obs.obstacle_length = objects.at(i).shape.dimensions.at(0);
                obs.obstacle_width = objects.at(i).shape.dimensions.at(1);
                obs.obstacle_height = objects.at(i).shape.dimensions.at(2);
            } else if (obs.obstacle_type == 4)                                 // 圆形：Pedestrian
            {
                obs.obstacle_radius = objects.at(i).shape.dimensions.at(0);    // 半径
                // 顶点
                obs.pinnacle.poses.clear();
                PPoint ob_center(obs.centerpoint.position.x, obs.centerpoint.position.y);
                auto oba = obs.GetOba();
                // 求障碍物顶点
                PPoint ob_left_front;
                PPoint ob_left_buttom;
                PPoint ob_right_front;
                PPoint ob_right_buttom;
                oba->CalculateCarBoundaryPoint(1, 1, ob_center, obs.obstacle_theta, ob_left_front, ob_left_buttom,
                                               ob_right_buttom, ob_right_front);
                // oba->visualization_points(ob_left_front, ob_left_buttom, ob_right_buttom,
                //                           ob_right_front);    // 显示障碍物顶点

                std::vector<PPoint> ob_vector;
                ob_vector.emplace_back(ob_right_front);     // 右下角
                ob_vector.emplace_back(ob_left_buttom);     // 右上角
                ob_vector.emplace_back(ob_left_front);      // 左上角
                ob_vector.emplace_back(ob_right_buttom);    // 左下角
                for (size_t j = 0; j < ob_vector.size(); ++j) {
                    geometry_msgs::msg::Pose sds;
                    sds.position.x = ob_vector[j].x;
                    sds.position.y = ob_vector[j].y;
                    sds.position.z = 0;    // 压缩二维
                    obs.pinnacle.poses.push_back(sds);
                    obs.polygon_points.push_back(Vec2d(ob_vector[j].x, ob_vector[j].y));
                }
                // 长和宽（假设）
                obs.obstacle_length = objects.at(i).shape.dimensions.at(0);
                obs.obstacle_width = objects.at(i).shape.dimensions.at(1);
                obs.obstacle_height = objects.at(i).shape.dimensions.at(2);
            }
            // else if (msgs->objects[i].shape.type == 2) //多边形:未知
            // {
            //   //顶点
            //   obs.pinnacle.poses.clear();
            //   for (size_t j = 0; j < msgs->objects[i].shape.footprint.points.size(); j++)
            //   {
            //     geometry_msgs::Pose sds;
            //     sds.position.x = msgs->objects[i].shape.footprint.points[j].x;
            //     sds.position.y = msgs->objects[i].shape.footprint.points[j].y;
            //     sds.position.z = 0; //压缩二维
            //     obs.pinnacle.poses.push_back(sds);
            //     obs.polygon_points.push_back(Vec2d(msgs->objects[i].shape.footprint.points[j].x,
            //     msgs->objects[i].shape.footprint.points[j].y));
            //   }

            //   //半径为0
            //   obs.obstacle_radius = 0;
            //   //长和宽（假设）
            //   obs.obstacle_length = msgs->objects[i].shape.dimensions.x;
            //   obs.obstacle_width = msgs->objects[i].shape.dimensions.y;
            //   obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
            // }
            AllObstacle.emplace_back(obs);
            std::cout << "pos_x: " << obs.centerpoint.position.x << "  pos_y: " << obs.centerpoint.position.y
                      << "  heading: " << obs.obstacle_theta << "  width: " << obs.obstacle_width
                      << "  length: " << obs.obstacle_length << "  velocity: " << obs.obstacle_velocity << std::endl;
        }
    }
    std::cout << "AllObstacle size: " << AllObstacle.size() << " !!!!!!!" << std::endl;
}

void NopFunctionNode::ObjectArrayCallback(derived_object_msgs::msg::ObjectArray::SharedPtr msg) {
    const auto &objects = msg->objects;
    if (objects.size() != (size_t)0) {
        // std::cout << "find object" << std::endl;
    } else {
        std::cout << "not find object" << std::endl;
        return;
    }
    const double object_x = objects.at(0).pose.position.x;
    const double object_y = objects.at(0).pose.position.y;

    // std::cout << "object x : " << objects.at(0).pose.position.x << std::endl;
    // std::cout << "object y : " << objects.at(0).pose.position.y << std::endl;
    // std::cout << "vehicle_state_.x : " << vehicle_state_.x << std::endl;
    // std::cout << "vehicle_state_.y : " << vehicle_state_.y << std::endl;
    // std::cout << "vehicle_state_.v : " << vehicle_state_.v << std::endl;
    // std::cout << "vehicle_state_.acceleration : " << vehicle_state_.acceleration << std::endl;

    double dist_x = abs(object_x - vehicle_state_.x);
    double dist_y = abs(object_y - vehicle_state_.y);
    const double object_dist = sqrt(pow(dist_x, (double)2.0) + pow(dist_y, (double)2.0));
    double reaction_time = AEB_BRAKE_REACTION_TIME;
    double safe_dist = (vehicle_state_.v / (double)3.6) * reaction_time +
                       (double)0.5 * vehicle_state_.acceleration * pow(AEB_BRAKE_REACTION_TIME, (double)2.0);

    if (safe_dist > object_dist) {
        if_aeb_active_ = true;
        std::cout << "AEB DECELERATION(m/s2) : " << -control_cmd.brake << std::endl;
    }
    if (if_aeb_active_ && abs(vehicle_state_.v - (double)0.0) < (double)0.001) {
        std::cout << "AEB stop distance: " << object_dist - (double)1.5 << std::endl;
    }
}

void NopFunctionNode::GlobalPathPublishCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // publish for orininal points for rviz
    // global_path.poses.clear();
    // for (const auto &point : xy_points) {
    //     this_pose_stamped.header.frame_id = "gps";
    //     this_pose_stamped.header.stamp = this->get_clock()->now();
    //     this_pose_stamped.pose.position.x = point.first;
    //     this_pose_stamped.pose.position.y = point.second;
    //     this_pose_stamped.pose.position.z = 0;
    //     this_pose_stamped.pose.orientation.x = 0;
    //     this_pose_stamped.pose.orientation.y = 0;
    //     this_pose_stamped.pose.orientation.z = 0;
    //     this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

    //     global_path.poses.push_back(this_pose_stamped);
    // }

    // publish for orininal points for rviz
    global_path.poses.clear();
    for (const auto &point : rl_.reference_points) {
        this_pose_stamped.header.frame_id = "gps";
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = point.get_x();
        this_pose_stamped.pose.position.y = point.get_y();
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

        global_path.poses.push_back(this_pose_stamped);
    }

    global_path.header.frame_id = "gps";
    global_path.header.stamp = this->get_clock()->now();
    global_path_publisher_->publish(global_path);
}

NopFunctionNode::~NopFunctionNode() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/

void NopFunctionNode::VehicleStatusCallback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 为了在rqt里面，一个plot里面查看目标速度和实际速度，需要两个速度有关的消息都使用
**************************************************************************************'''*/
{
    vehicle_control_target_velocity.header.stamp = msg->header.stamp;
    vehicle_state_.acceleration = msg->acceleration.linear.x;
}

double NopFunctionNode::PointDistanceSquare(const TrajectoryPointOri &point, const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : 两点之间的距离
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

TrajectoryPointOri NopFunctionNode::QueryNearestPointByPosition(const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    std::cout << "index: " << index_min << " ";
    std::cout << "vehicle.x: " << x << " "
              << "vehicle.y: " << y << std::endl;
    std::cout << "trajectory_points.x: " << trajectory_points_[index_min].x << " "
              << "trajectory_points.y: " << trajectory_points_[index_min].y;

    std::cout << std::endl;

    return trajectory_points_[index_min];
}

double NopFunctionNode::PointDistanceSquareOfRefLine(const ReferencePoint &point, const double x, const double y) {
    double dx = point.get_x() - x;
    double dy = point.get_y() - y;
    return dx * dx + dy * dy;
}

size_t NopFunctionNode::QueryNearestPointIndexByPositionOfRefLine(const double x, const double y) {
    auto reference_points_ptr = rl_.GetReferencePoints();
    double d_min = PointDistanceSquareOfRefLine(reference_points_ptr->at(0), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < reference_points_ptr->size(); ++i) {
        double d_temp = PointDistanceSquareOfRefLine(reference_points_ptr->at(i), x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    // std::cout << "index: " << index_min << " ";

    return index_min;
}

void NopFunctionNode::ImuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
    vehicle_state_.ax = msg->linear_acceleration.x;
    vehicle_state_.ay = msg->linear_acceleration.y;
    rl_.gps_.accelx = vehicle_state_.ax;
    rl_.gps_.accely = vehicle_state_.ay;
}

void NopFunctionNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->pose.pose.position.x);
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_state_.roll, vehicle_state_.pitch, vehicle_state_.yaw);

    if (first_record_) {
        vehicle_state_.start_point_x = msg->pose.pose.position.x;
        vehicle_state_.start_point_y = msg->pose.pose.position.y;
        vehicle_state_.start_heading = -M_PI / 2;
        first_record_ = false;
    }
    vehicle_state_.x = msg->pose.pose.position.x;
    vehicle_state_.y = msg->pose.pose.position.y;
    vehicle_state_.vx = msg->twist.twist.linear.x;
    vehicle_state_.vy = msg->twist.twist.linear.y;
    vehicle_state_.vz = msg->twist.twist.linear.z;
    vehicle_state_.v = std::sqrt(vehicle_state_.vx * vehicle_state_.vx + vehicle_state_.vy * vehicle_state_.vy +
                                 vehicle_state_.vz * vehicle_state_.vz) *
                       3.6;                         // 本车速度
    vehicle_state_.heading = vehicle_state_.yaw;    // pose.orientation是四元数

    // 将自车位置信息传给reference line
    //    double x_cur = gps_.posx;    // 定位
    // double y_cur = gps_.posy;
    // double theta_cur = gps_.oriz;
    // double kappa_cur = 0;
    // double vx_cur = gps_.velx;
    // double vy_cur = gps_.vely;
    // double ax_cur = gps_.accelx;
    // double ay_cur = gps_.accely;
    rl_.gps_.posx = vehicle_state_.x;
    rl_.gps_.posy = vehicle_state_.y;
    rl_.gps_.oriz = vehicle_state_.heading;
    rl_.gps_.velx = vehicle_state_.vx;
    rl_.gps_.vely = vehicle_state_.vy;

    /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
    // history_path.header.stamp = this->get_clock()->now();
    // history_path.header.frame_id = "gps";

    // history_path_points.header.stamp = this->get_clock()->now();
    // history_path_points.header.frame_id = "gps";
    // history_path_points.pose.position.x = vehicle_state_.x;
    // history_path_points.pose.position.y = vehicle_state_.y;
    // history_path_points.pose.position.z = 0;
    // history_path_points.pose.orientation = msg->pose.pose.orientation;
    // history_path.poses.push_back(history_path_points);

    // if (history_path.poses.size() > 2000) {
    //     vector<geometry_msgs::msg::PoseStamped>::iterator k = history_path.poses.begin();
    //     history_path.poses.erase(k);
    // }
    // nop_planed_traj_publisher->publish(history_path);

    // 将世界坐标系和车辆坐标系的位置关系广播出来
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "gps";
    transformStamped.child_frame_id = "vehicle_odometry";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;

    transformStamped.transform.rotation.x = quat_tf.x();
    transformStamped.transform.rotation.y = quat_tf.y();
    transformStamped.transform.rotation.z = quat_tf.z();
    transformStamped.transform.rotation.w = quat_tf.w();

    tf_broadcaster_gps_vehicle->sendTransform(transformStamped);
}

// void NopFunctionNode::VehicleControlIterationCallback(carla_msgs::msg::CarlaStatus::SharedPtr msg)
void NopFunctionNode::VehicleControlIterationCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    TrajectoryPointOri target_point_;

    target_point_ = this->QueryNearestPointByPosition(vehicle_state_.x, vehicle_state_.y);

    std::cout << "CONTROL_INFO , target_point_.v_mps : " << target_point_.v
              << "  vehicle_state_.v_kph : " << vehicle_state_.v << " x: " << target_point_.x
              << " y: " << target_point_.y << std::endl;

    double v_err = target_point_.v - (vehicle_state_.v / (double)3.6);    // 速度误差
    double yaw_err = vehicle_state_.heading - target_point_.heading;      // 横摆角误差

    if (yaw_err > M_PI / 6) {
        yaw_err = M_PI / 6;
    } else if (yaw_err < -M_PI / 6) {
        yaw_err = -M_PI / 6;
    }

    if (cnt % 1 == 0) {
        // cout << "start_heading: " << vehicle_state_.start_heading << endl;
        // cout << "heading: " << vehicle_state_.heading << endl;
        // cout << "~~ vehicle_state_.v: " << vehicle_state_.v * 3.6 << ", target_point_.v: " << target_point_.v
        //      << ", v_err: " << v_err << endl;
        // cout << "yaw_err: " << yaw_err << endl;
        // cout << "control_cmd.target_wheel_angle: " << control_cmd.target_wheel_angle << endl;
    }
    TrajectoryData planning_published_trajectory;
    planning_published_trajectory.trajectory_points.clear();
    for (size_t i = 0; i < trajectory_points_.size(); i++) {
        TrajectoryPointOri point = trajectory_points_.at(i);
        planning_published_trajectory.trajectory_points.emplace_back(point);
    }

    vehicle_state_.velocity = vehicle_state_.v;
    ControlCmd cmd;

    // use lqr lat
    if (true) {
        lqr_controller_lateral_->ComputeControlCommand(this->vehicle_state_, planning_published_trajectory, cmd);
    }
    // use stanely lat
    if (false) {
        stanely_controller_lateral_->ComputeControlCmd(this->vehicle_state_, this->planning_published_trajectory, cmd);
    }

    acceleration_cmd = pid_controller_longitudinal_->Control(v_err, 0.05);
    // steer_cmd = yaw_pid_controller.Control(yaw_err, 0.01);
    steer_cmd = 0.0;

    std::cout << "acceleration_cmd : " << acceleration_cmd << std::endl;

    control_cmd.header.stamp = this->now();

    if (acceleration_cmd >= 1.0) {
        acceleration_cmd = 1.0;
    }
    if (acceleration_cmd <= -1) {
        acceleration_cmd = -1.0;
    }

    if (acceleration_cmd <= 0) {
        control_cmd.brake = -acceleration_cmd;
        control_cmd.throttle = 0;
    } else {
        control_cmd.throttle = acceleration_cmd;
        control_cmd.brake = 0;
    }

    if (isnan(cmd.steer_target)) {
        control_cmd.steer = 0;
    } else {
        control_cmd.steer = cmd.steer_target;
    }
    std::cout << "steer_target : " << control_cmd.steer << std::endl;
    // std::cout << "acceleration_cmd: " << acceleration_cmd << std::endl;
    control_cmd.gear = 1;
    control_cmd.reverse = false;
    control_cmd.hand_brake = false;
    control_cmd.manual_gear_shift = false;

    // if (if_aeb_active_ == true) {
    //     control_cmd.brake = -AEB_BRAKE_DECELERATION;
    //     control_cmd.throttle = 0;
    // }

    vehicle_control_publisher->publish(control_cmd);

    // vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = target_point_.v / 3.6;
    vehicle_control_target_velocity_publisher->publish(vehicle_control_target_velocity);
    cnt++;
}

// 非referenceLine成员函数
// 已知顶点和中心点，将任何障碍物变成方形的二维平面，计算长和宽,并存储于Marker
void NopFunctionNode::Turn_obstacles_into_squares(visualization_msgs::msg::Marker &marker,
                                                  const Obstacle *Primitive_obstacle, const int id) {
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

// int main(int argc, char **argv)
// /*'''**************************************************************************************
// - FunctionName: None
// - Function    : None
// - Inputs      : None
// - Outputs     : None
// - Comments    : None
// **************************************************************************************'''*/
// {
//     // RCLCPP_INFO(LOGGER, "Initialize node");

//     rclcpp::init(argc, argv);

//     auto n = std::make_shared<NopFunctionNode>();

//     rclcpp::spin(n);

//     rclcpp::shutdown();

//     return 0;
// }
