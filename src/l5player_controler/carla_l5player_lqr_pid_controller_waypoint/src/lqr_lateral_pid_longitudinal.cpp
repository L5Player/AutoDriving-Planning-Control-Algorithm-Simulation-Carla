#include "carla_l5player_lqr_pid_controller/lqr_lateral_pid_longitudinal.h"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lqr_lateral_pid_longitudinal");

LQRControllerNode::LQRControllerNode()
    : Node("lqr_lateral_pid_longitudinal")
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    this->declare_parameter<std::string>("roadmap_path", roadmap_path);    // 读取路网文件名
    this->declare_parameter<double>("target_speed", target_speed);         // 读取目标速度
    this->declare_parameter<double>("goal_tolerance", goalTolerance_);     // 读取目标速度
    this->declare_parameter<double>("speed_P", speed_P);                   // 读取PID参数
    this->declare_parameter<double>("speed_I", speed_I);
    this->declare_parameter<double>("speed_D", speed_D);

    this->get_parameter<std::string>("roadmap_path", roadmap_path);    // 读取路网文件名
    this->get_parameter<double>("target_speed", target_speed);         // 读取目标速度
    this->get_parameter<double>("goal_tolerance", goalTolerance_);     // 读取目标速度
    this->get_parameter<double>("speed_P", speed_P);                   // 读取PID参数
    this->get_parameter<double>("speed_I", speed_I);
    this->get_parameter<double>("speed_D", speed_D);

    // 加载路网文件
    std::cout << "roadmap_path: " << roadmap_path << "  " << target_speed << std::endl;
    // loadRoadmap(roadmap_path);

    pid_controller_longitudinal = std::make_unique<l5player::control::PIDController>(speed_P, speed_I, speed_D);

    lqr_controller_lateral = std::make_unique<l5player::control::LqrController>();
    lqr_controller_lateral->LoadControlConf();
    lqr_controller_lateral->Init();

    route_waypoint_subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "/carla/ego_vehicle/waypoints", 10, std::bind(&LQRControllerNode::RouteWaypointCallback, this, _1));
    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", 10, std::bind(&LQRControllerNode::OdomCallback, this, _1));
    lacalization_data_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/carla/ego_vehicle/imu", 10, std::bind(&LQRControllerNode::IMUCallback, this, _1));

    vehicle_control_publisher =
        this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
    control_cmd.header.stamp = this->now();
    control_cmd.gear = 1;
    control_cmd.manual_gear_shift = false;
    control_cmd.reverse = false;
    control_cmd.hand_brake = false;

    auto time_node_start = this->now();
    vehicle_control_target_velocity_publisher =
        this->create_publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>("/carla/ego_vehicle/target_velocity", 10);
    vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = 0.0;

    carla_status_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
        "/carla/ego_vehicle/vehicle_status", 10, std::bind(&LQRControllerNode::VehicleStatusCallback, this, _1));

    // used for rviz
    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_reference_path", 2);
    history_path_visualization_publisher = this->create_publisher<nav_msgs::msg::Path>("/history_path", 2);

    // Initialize the transform broadcaster
    tf_broadcaster_gps_vehicle = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    global_path_publish_timer =
        this->create_wall_timer(500ms, std::bind(&LQRControllerNode::GlobalPathPublishCallback, this));

    RCLCPP_INFO(LOGGER, "lqr_control_node init finish!");
}

LQRControllerNode::~LQRControllerNode()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{}

void LQRControllerNode::RouteWaypointCallback(nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Got Route Waypoint data!!!");
    if_get_waypoint = true;

    std::cout << "pose size is : " << msg->poses.size() << std::endl;

    global_path.poses.clear();

    for (int i = 0; i < msg->poses.size(); i++) {
        double pt_x = msg->poses[i].pose.position.x;
        double pt_y = msg->poses[i].pose.position.y;
        v_points.push_back(20.0);
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<l5player::control::ReferenceLine> reference_line =
        std::make_unique<l5player::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) {
        // std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i]
        //           << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    }

    size_t _count_points = headings.size();
    size_t _stop_begin_point = ceil(_count_points * 0.85);
    size_t _stop_point = ceil(_count_points * 0.95);
    // std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

    int _index_before_stop = 0;
    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
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

        this_pose_stamped.header.frame_id = "gps";
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = xy_points[i].first;
        this_pose_stamped.pose.position.y = xy_points[i].second;
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

        global_path.poses.push_back(this_pose_stamped);
        global_path.header.frame_id = "gps";
    }

    goal_point = planning_published_trajectory.trajectory_points.back();

    trajectory_points_ = planning_published_trajectory.trajectory_points;

    vehicle_control_iteration_timer =
        this->create_wall_timer(10ms, std::bind(&LQRControllerNode::VehicleControllerIterationCallback, this));
}

void LQRControllerNode::OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicleState_.roll, vehicleState_.pitch, vehicleState_.yaw);

    if (firstRecord_) {
        vehicleState_.start_point_x = msg->pose.pose.position.x;
        vehicleState_.start_point_y = msg->pose.pose.position.y;
        firstRecord_ = false;
    }
    vehicleState_.x = msg->pose.pose.position.x;    //
    vehicleState_.y = msg->pose.pose.position.y;
    vehicleState_.vx = msg->twist.twist.linear.x;
    vehicleState_.vy = msg->twist.twist.linear.y;
    vehicleState_.vz = msg->twist.twist.linear.z;
    vehicleState_.velocity = std::sqrt(vehicleState_.vx * vehicleState_.vx + vehicleState_.vy * vehicleState_.vy +
                                       vehicleState_.vz * vehicleState_.vz) *
                             3.6;    // 本车速度
    vehicleState_.heading = vehicleState_.yaw;
    // cout << "vehicleState_.heading: " << vehicleState_.heading << endl;

    /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
    history_path.header.stamp = this->get_clock()->now();
    history_path.header.frame_id = "gps";

    history_path_points.header.stamp = this->get_clock()->now();
    history_path_points.header.frame_id = "gps";
    history_path_points.pose.position.x = vehicleState_.x;
    history_path_points.pose.position.y = vehicleState_.y;
    history_path_points.pose.position.z = 0;
    history_path_points.pose.orientation = msg->pose.pose.orientation;
    history_path.poses.push_back(history_path_points);

    if (history_path.poses.size() > 2000) {
        vector<geometry_msgs::msg::PoseStamped>::iterator k = history_path.poses.begin();
        history_path.poses.erase(k);
    }

    history_path_visualization_publisher->publish(history_path);

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
void LQRControllerNode::IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // RCLCPP_INFO(LOGGER, "Got IMU data!!!");
    vehicleState_.angular_velocity = msg->angular_velocity.z;    // 平面角速度(绕z轴转动的角速度)
    vehicleState_.acceleration = sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
                                      msg->linear_acceleration.y * msg->linear_acceleration.y);    // 加速度
}
void LQRControllerNode::loadRoadmap(const std::string& roadmap_path)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // 读取参考线路径
    std::ifstream infile(roadmap_path, ios::in);    // 将文件流对象与文件连接起来
    // assert(infile.is_open());                       // 若失败,则输出错误消息,并终止程序运行

    while (getline(infile, _line)) {
        // std::cout << _line << std::endl;
        // 解析每行的数据
        stringstream ss(_line);
        string _sub;
        vector<string> subArray;
        // 按照逗号分隔
        while (getline(ss, _sub, ',')) {
            subArray.push_back(_sub);
        }
        double pt_x = std::atof(subArray[2].c_str());
        double pt_y = std::atof(subArray[3].c_str());
        // double pt_v = std::atof(subArray[6].c_str());

        v_points.push_back(20.0);
        xy_points.push_back(std::make_pair(pt_x, pt_y));
    }
    infile.close();

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<l5player::control::ReferenceLine> reference_line =
        std::make_unique<l5player::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) {
        // std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i]
        //           << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    }

    size_t _count_points = headings.size();
    size_t _stop_begin_point = ceil(_count_points * 0.85);
    size_t _stop_point = ceil(_count_points * 0.95);
    // std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

    int _index_before_stop = 0;
    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
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

        this_pose_stamped.header.frame_id = "gps";
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = xy_points[i].first;
        this_pose_stamped.pose.position.y = xy_points[i].second;
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

        global_path.poses.push_back(this_pose_stamped);
        global_path.header.frame_id = "gps";
    }

    goal_point = planning_published_trajectory.trajectory_points.back();

    trajectory_points_ = planning_published_trajectory.trajectory_points;
}

void LQRControllerNode::GlobalPathPublishCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    global_path.header.stamp = this->get_clock()->now();
    global_path_publisher_->publish(global_path);
}
void LQRControllerNode::VehicleStatusCallback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 为了在rqt里面，一个plot里面查看目标速度和实际速度，需要两个速度有关的消息都使用
**************************************************************************************'''*/
{
    vehicle_control_target_velocity.header.stamp = msg->header.stamp;
}

double LQRControllerNode::PointDistanceSquare(const TrajectoryPoint& point, const double x, const double y)
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

TrajectoryPoint LQRControllerNode::QueryNearestPointByPosition(const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double d_min = this->PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = this->PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return trajectory_points_[index_min];
}

void LQRControllerNode::VehicleControllerIterationCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    ControlCmd cmd;
    if (!firstRecord_) {    // 有定位数据开始控制
        // 小于容忍距离，车辆速度设置为0
        if (pointDistance(goal_point, vehicleState_.x, vehicleState_.y) < goalTolerance_) {
            targetSpeed_ = 0;
            isReachGoal_ = true;
        }
        if (!isReachGoal_) {
            lqr_controller_lateral->ComputeControlCommand(this->vehicleState_, this->planning_published_trajectory,
                                                          cmd);
        }

        TrajectoryPoint target_point_;
        target_point_ = this->QueryNearestPointByPosition(vehicleState_.x, vehicleState_.y);
        double v_err = target_point_.v - vehicleState_.velocity;           // 速度误差
        double yaw_err = vehicleState_.heading - target_point_.heading;    // 横摆角误差
        // cout << "v_err: " << v_err << endl;
        double acceleration_cmd = pid_controller_longitudinal->Control(v_err, 0.01);

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
        // control_cmd.steer = 0;
        control_cmd.gear = 1;
        control_cmd.reverse = false;
        control_cmd.hand_brake = false;
        control_cmd.manual_gear_shift = false;

        vehicle_control_publisher->publish(control_cmd);

        vehicle_control_target_velocity.velocity = target_point_.v / 3.6;
        vehicle_control_target_velocity_publisher->publish(vehicle_control_target_velocity);
        // cout << "control_cmd.steer: " << control_cmd.steer << endl;
        // cout << "~~ vehicleState_.v: " << vehicleState_.velocity * 3.6 << ", target_point_.v: " << target_point_.v
        //      << ", v_err: " << v_err << endl;
        // cout << "yaw_err: " << yaw_err << endl;
    }
}
int main(int argc, char** argv) {
    RCLCPP_INFO(LOGGER, "Initializa Node~");
    std::cout << argv[0] << std::endl;
    rclcpp::init(argc, argv);
    auto n = std::make_shared<LQRControllerNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}