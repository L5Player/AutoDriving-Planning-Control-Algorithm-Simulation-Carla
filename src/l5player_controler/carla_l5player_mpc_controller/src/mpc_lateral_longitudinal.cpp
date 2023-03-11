#include "carla_l5player_mpc_controller/mpc_lateral_longitudinal.h"

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "Eigen/LU"
#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mpc_lateral_longitudinal");

MPCControllerNode::MPCControllerNode()
    : Node("mpc_lateral_longitudinal")
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    this->declare_parameter<std::string>("roadmap_path", roadmap_path);    //读取路网文件名
    this->declare_parameter<double>("target_speed", target_speed);         //读取目标速度
    this->declare_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->declare_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->declare_parameter<double>("speed_I", speed_I);
    this->declare_parameter<double>("speed_D", speed_D);

    this->get_parameter<std::string>("roadmap_path", roadmap_path);    //读取路网文件名
    this->get_parameter<double>("target_speed", target_speed);         //读取目标速度
    this->get_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->get_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->get_parameter<double>("speed_I", speed_I);
    this->get_parameter<double>("speed_D", speed_D);

    // Declare and initialize a parameter, return the effective value.
    this->declare_parameter<double>("vehicle_ref_v", target_v);
    this->declare_parameter<double>("vehicle_steering_ratio_double", steering_ratio);
    this->declare_parameter<double>("vehicle_Lf_double", kinamatic_para_Lf);
    this->declare_parameter<int>("mpc_control_horizon_length_int", mpc_control_horizon_length);
    this->declare_parameter<double>("mpc_control_step_length_double", mpc_control_step_length);
    this->declare_parameter<bool>("mpc_tracking_enable_bool", mpc_tracking_enable_bool);
    this->declare_parameter<int>("mpc_cte_weight_int", mpc_cte_weight_int);
    this->declare_parameter<int>("mpc_epsi_weight_int", epsi_weight);
    this->declare_parameter<int>("mpc_v_weight_int", v_weight);
    this->declare_parameter<int>("mpc_steer_actuator_cost_weight_int", steer_actuator_cost_weight);
    this->declare_parameter<int>("mpc_acc_actuator_cost_weight_int", acc_actuator_cost_weight);
    this->declare_parameter<int>("mpc_change_steer_cost_weight_int", change_steer_cost_weight);
    this->declare_parameter<int>("mpc_change_accel_cost_weight_int", change_accel_cost_weight);
    this->declare_parameter<double>("mpc_reference_path_length", reference_path_length);
    this->declare_parameter<double>("mpc_controller_delay_compensation", controller_delay_compensation);
    this->declare_parameter<double>("mpc_point_distance_of_reference_line_visualization", point_distance_of_reference_line_visualization);
    this->declare_parameter<int>("mpc_points_number_of_reference_line_visualization", points_number_of_reference_line_visualization);
    this->declare_parameter<int>("mpc_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("mpc_working_mode", working_mode);
    this->declare_parameter<int>("mpc_with_planner_flag", with_planner_flag);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<double>("vehicle_ref_v", this->target_v);
    this->get_parameter<double>("vehicle_steering_ratio_double", this->steering_ratio);
    this->get_parameter<double>("vehicle_Lf_double", this->kinamatic_para_Lf);
    this->get_parameter<int>("mpc_control_horizon_length_int", this->mpc_control_horizon_length);
    this->get_parameter<double>("mpc_control_step_length_double", this->mpc_control_step_length);
    this->get_parameter<bool>("mpc_tracking_enable_bool", this->mpc_enable_signal);
    this->get_parameter<int>("mpc_cte_weight_int", this->cte_weight);
    this->get_parameter<int>("mpc_epsi_weight_int", this->epsi_weight);
    this->get_parameter<int>("mpc_v_weight_int", this->v_weight);
    this->get_parameter<int>("mpc_steer_actuator_cost_weight_int", this->steer_actuator_cost_weight);
    this->get_parameter<int>("mpc_acc_actuator_cost_weight_int", this->acc_actuator_cost_weight);
    this->get_parameter<int>("mpc_change_steer_cost_weight_int", this->change_steer_cost_weight);
    this->get_parameter<int>("mpc_change_accel_cost_weight_int", this->change_accel_cost_weight);
    this->get_parameter<double>("mpc_reference_path_length", this->reference_path_length);
    this->get_parameter<double>("mpc_controller_delay_compensation", this->controller_delay_compensation);
    this->get_parameter<double>("mpc_point_distance_of_reference_line_visualization", this->point_distance_of_reference_line_visualization);
    this->get_parameter<int>("mpc_points_number_of_reference_line_visualization", this->points_number_of_reference_line_visualization);
    this->get_parameter<int>("mpc_former_point_of_current_position", this->former_point_of_current_position);
    this->get_parameter<int>("mpc_working_mode", this->working_mode);
    this->get_parameter<int>("mpc_with_planner_flag", this->with_planner_flag);

    RCLCPP_INFO(this->get_logger(), "target_v %f", this->target_v);
    RCLCPP_INFO(this->get_logger(), "vehicle_steering_ratio_double %f", this->steering_ratio);
    RCLCPP_INFO(this->get_logger(), "vehicle_Lf_double %f", this->kinamatic_para_Lf);
    RCLCPP_INFO(this->get_logger(), "mpc_control_horizon_length_int %d", this->mpc_control_horizon_length);
    RCLCPP_INFO(this->get_logger(), "mpc_control_step_length_double %f", this->mpc_control_step_length);
    RCLCPP_INFO(this->get_logger(), "mpc_tracking_enable_bool %d", int(this->mpc_enable_signal));
    RCLCPP_INFO(this->get_logger(), "mpc_cte_weight_int %d", this->cte_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_epsi_weight_int %d", this->epsi_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_v_weight_int %d", this->v_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_steer_actuator_cost_weight_int %d", this->steer_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_acc_actuator_cost_weight_int %d", this->acc_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_change_steer_cost_weight_int %d", this->change_steer_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_change_accel_cost_weight_int %d", this->change_accel_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_reference_path_length %f", this->reference_path_length);
    RCLCPP_INFO(this->get_logger(), "mpc_controller_delay_compensation %f", this->controller_delay_compensation);
    RCLCPP_INFO(this->get_logger(), "mpc_point_distance_of_reference_line_visualization %f", this->point_distance_of_reference_line_visualization);
    RCLCPP_INFO(this->get_logger(), "mpc_points_number_of_reference_line_visualization %d", this->points_number_of_reference_line_visualization);
    RCLCPP_INFO(this->get_logger(), "mpc_former_point_of_current_position %d", this->former_point_of_current_position);
    RCLCPP_INFO(this->get_logger(), "mpc_working_mode %d", this->working_mode);
    RCLCPP_INFO(this->get_logger(), "mpc_with_planner_flag %d", this->with_planner_flag);

    //加载路网文件
    std::cout << "roadmap_path: " << roadmap_path << "  " << target_speed << std::endl;
    loadRoadmap(roadmap_path);

    // pid_controller_longitudinal = std::make_unique<l5player::control::PIDController>(speed_P, speed_I, speed_D);

    // mpc_controller_lateral = std::make_unique<l5player::control::MPCController>();
    // mpc_controller_lateral->LoadControlConf();
    // mpc_controller_lateral->Init();

    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&MPCControllerNode::OdomCallback, this, _1));
    lacalization_data_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("/carla/ego_vehicle/imu", 10, std::bind(&MPCControllerNode::IMUCallback, this, _1));

    vehicle_control_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
    control_cmd.header.stamp = this->now();
    control_cmd.gear = 1;
    control_cmd.manual_gear_shift = false;
    control_cmd.reverse = false;
    control_cmd.hand_brake = false;

    auto time_node_start = this->now();
    vehicle_control_target_velocity_publisher = this->create_publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>("/carla/ego_vehicle/target_velocity", 10);
    vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = 0.0;

    carla_status_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10, std::bind(&MPCControllerNode::VehicleStatusCallback, this, _1));

    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_reference_path", 2);
    history_path_visualization_publisher = this->create_publisher<nav_msgs::msg::Path>("/history_path", 2);

    // Initialize the transform broadcaster
    tf_broadcaster_gps_vehicle = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // mpc 求解出来的未来一段时间的路径，以及mpc使用的未来一段时间的参考轨迹点
    mpc_reference_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("mpc_reference_path", 10);
    mpc_output_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("mpc_output_path", 10);
    mpc_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("mpc_iteration_duration", 10);    // 用于统计MPC求解时间的广播器

    vehicle_control_iteration_timer = this->create_wall_timer(50ms, std::bind(&MPCControllerNode::VehicleControllerIterationCallback, this));
    global_path_publish_timer = this->create_wall_timer(500ms, std::bind(&MPCControllerNode::GlobalPathPublishCallback, this));

    RCLCPP_INFO(LOGGER, "mpc_control_node init finish!");
}

MPCControllerNode::~MPCControllerNode()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{}

void MPCControllerNode::OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : the x direction of msg is longitudinal
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
    vehicleState_.x = msg->pose.pose.position.x;
    vehicleState_.y = msg->pose.pose.position.y;
    vehicleState_.vx = msg->twist.twist.linear.x;
    vehicleState_.vy = msg->twist.twist.linear.y;
    vehicleState_.vz = msg->twist.twist.linear.z;
    vehicleState_.velocity = std::sqrt(vehicleState_.vx * vehicleState_.vx + vehicleState_.vy * vehicleState_.vy + vehicleState_.vz * vehicleState_.vz) * 3.6;    // 本车速度
    vehicleState_.heading = vehicleState_.yaw;
    // cout << "vehicleState_.heading: " << vehicleState_.heading << endl;

    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    psi = vehicleState_.yaw;
    v_longitudinal = msg->twist.twist.linear.x;
    v_lateral = msg->twist.twist.linear.y;

    // cout << "v_longitudinal: " << v_longitudinal << " , v_lateral" << v_lateral << endl;

    /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
    history_path.header.stamp = this->get_clock()->now();
    history_path.header.frame_id = "map";

    history_path_points.header.stamp = this->get_clock()->now();
    history_path_points.header.frame_id = "map";
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
    transformStamped.header.frame_id = "map";
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
void MPCControllerNode::IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    RCLCPP_INFO(LOGGER, "Got IMU data!!!");
    vehicleState_.angular_velocity = msg->angular_velocity.z;                                                                                                // 平面角速度(绕z轴转动的角速度)
    vehicleState_.acceleration = sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x + msg->linear_acceleration.y * msg->linear_acceleration.y);    // 加速度

    a_longitudinal = msg->linear_acceleration.x;
    a_lateral = msg->linear_acceleration.y;
    yaw_rate = msg->angular_velocity.z;
}
void MPCControllerNode::loadRoadmap(const std::string& roadmap_path)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // 读取参考线路径
    std::ifstream infile(roadmap_path, std::ios::in);    //将文件流对象与文件连接起来
    assert(infile.is_open());                            //若失败,则输出错误消息,并终止程序运行

    while (getline(infile, _line)) {
        std::cout << _line << std::endl;
        //解析每行的数据
        std::stringstream ss(_line);
        string _sub;
        vector<string> subArray;
        //按照逗号分隔
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
    std::unique_ptr<l5player::control::ReferenceLine> reference_line = std::make_unique<l5player::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    for (size_t i = 0; i < headings.size(); i++) {
        std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    }

    size_t _count_points = headings.size();
    size_t _stop_begin_point = ceil(_count_points * 0.85);
    size_t _stop_point = ceil(_count_points * 0.95);
    std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

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
                trajectory_pt.v = v_points[_index_before_stop] * ((double)i / ((double)_stop_begin_point - (double)_stop_point) - (double)_stop_point / ((double)_stop_begin_point - (double)_stop_point));
            } else {
                trajectory_pt.v = 0;
            }
        }
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];

        planning_published_trajectory.trajectory_points.push_back(trajectory_pt);

        this_pose_stamped.header.frame_id = "map";
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = xy_points[i].first;
        this_pose_stamped.pose.position.y = xy_points[i].second;
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 0;    // 这里实际上是放的frenet坐标系的S

        global_path.poses.push_back(this_pose_stamped);
        global_path.header.frame_id = "map";
    }

    goal_point = planning_published_trajectory.trajectory_points.back();

    trajectory_points_ = planning_published_trajectory.trajectory_points;
}

void MPCControllerNode::GlobalPathPublishCallback()
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
void MPCControllerNode::VehicleStatusCallback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 为了在rqt里面，一个plot里面查看目标速度和实际速度，需要两个速度有关的消息都使用
**************************************************************************************'''*/
{
    vehicle_control_target_velocity.header.stamp = msg->header.stamp;
    delta = msg->control.steer * 24;    // [-1, 1] from carla
}

double MPCControllerNode::PointDistanceSquare(const TrajectoryPoint& point, const double x, const double y)
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

TrajectoryPoint MPCControllerNode::QueryNearestPointByPosition(const double x, const double y)
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

void MPCControllerNode::VehicleControllerIterationCallback()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    rclcpp::Time start_mpc;
    rclcpp::Time end_mpc;
    start_mpc = this->now();
    double iteration_time_length;

    // ControlCmd cmd;

    if (!firstRecord_) {    //有定位数据开始控制

        reference_path_length = 16;    // 20 points
        global_path_remap_x.clear();
        global_path_remap_y.clear();

        for (size_t i = 0; i < xy_points.size() - 1; i++) {
            double shift_x = xy_points[i].first - px;
            double shift_y = xy_points[i].second - py;
            global_path_remap_x.push_back(shift_x * cos(psi) + shift_y * sin(psi));
            global_path_remap_y.push_back(-shift_x * sin(psi) + shift_y * cos(psi));
        }
        // 从全局路径中，找到距离当前位置最近的前方的点。
        for (size_t i = former_point_of_current_position; i < global_path_remap_x.size(); i++) {
            if (global_path_remap_x[i] > 0.0) {
                former_point_of_current_position = i;
                break;
            }
        }
        VectorXd coeffs;
        double cte;
        /* Convert to Eigen::VectorXd */
        double* ptrx = &global_path_remap_x[former_point_of_current_position];
        Eigen::Map<VectorXd> ptsx_transform(ptrx, reference_path_length);
        double* ptry = &global_path_remap_y[former_point_of_current_position];
        Eigen::Map<VectorXd> ptsy_transform(ptry, reference_path_length);
        /* Fit coefficients of fifth order polynomial*/
        coeffs = polyfit(ptsx_transform, ptsy_transform, 5);
        cte = polyeval(coeffs, 0);    // 在车辆坐标系下，当前时刻的位置偏差就是期望轨迹在车辆坐标系中的截距
        double epsi = -atan(coeffs[1]);

        // std::cout << "cte: " << cte << " , epsi: " << epsi << std::endl;

        /*Latency for predicting time at actuation*/
        double dt = controller_delay_compensation;
        // double dt = 0;

        double Lf = kinamatic_para_Lf;

        /* Predict future state (take latency into account) x, y and psi are all zero in the new reference system */
        double pred_px = 0.0 + v_longitudinal * dt;    // psi is 0, cos(0) = 1, can leave out
        double pred_py = 0.0 + v_lateral * 0;          // sin(0) = 0
        double pred_psi = 0.0 - v_longitudinal * delta / Lf * 0;
        double pred_v_longitudinal = v_longitudinal + a_longitudinal * 0;
        double pred_v_lateral = v_lateral + 0.0 * 0;
        double pred_omega = yaw_rate;
        double pred_cte = cte + v_longitudinal * tan(epsi) * 0;
        double pred_epsi = epsi - v_longitudinal * delta / Lf * 0;
        // cout << pred_v_longitudinal << " *********** v_longitudinal" << target_v << "target_v" << endl;
        /* Feed in the predicted state values.  这里传入的是车辆坐标系下的控制器时延模型*/
        Eigen::VectorXd state(8);
        state << pred_px, pred_py, pred_psi, pred_v_longitudinal, pred_v_lateral, pred_omega, pred_cte, pred_epsi;

        auto vars =
            mpc.Solve(state, coeffs,
                      target_v,    // m/s
                      cte_weight, epsi_weight, v_weight, steer_actuator_cost_weight, acc_actuator_cost_weight, change_steer_cost_weight, change_accel_cost_weight, mpc_control_horizon_length, mpc_control_step_length, kinamatic_para_Lf, a_lateral, old_steer_value, old_throttle_value, steering_ratio);
        old_steer_value = vars[0]; // * (1 / (24 * M_PI / 180));    // carla里面的横向控制信号 [-1,1]，但是模型计算的时候使用的是弧度单位的前轮转角，当前轮最大转角为24°的时候，通过这个公式进行转换
        old_throttle_value = vars[1];

        control_cmd.header.stamp = this->now();

        if (vars[0] >= 1.0) {
            vars[0] = 1.0;
        }
        if (vars[0] <= -1) {
            vars[0] = -1.0;
        }

        if (vars[1] >= 1.0) {
            vars[1] = 1.0;
        }
        if (vars[1] <= -1) {
            vars[1] = -1.0;
        }
        if (vars[1] <= 0) {
            control_cmd.brake = -vars[1];
            control_cmd.throttle = 0;
        } else {
            control_cmd.throttle = vars[1];
            control_cmd.brake = 0;
        }
        if (isnan(old_steer_value)) {
            control_cmd.steer = 0;
        } else {
            control_cmd.steer = old_steer_value;
        }
        // control_cmd.steer = 0;
        // control_cmd.throttle = 0.2;
        control_cmd.brake = 0;
        control_cmd.gear = 1;
        control_cmd.reverse = false;
        control_cmd.hand_brake = false;
        control_cmd.manual_gear_shift = false;

        vehicle_control_publisher->publish(control_cmd);
        /* 这里的路径和全局路径可视化是重合的，消息发出来，但是不一定要用的 */
        /* Display the waypoints / reference line. */
        vector<double> next_x_vals;
        vector<double> next_y_vals;

        next_x_vals.clear();
        next_y_vals.clear();

        /* 可视化里面,这里覆盖了从yaml配置文件传过来的可视化长度 */
        double poly_inc = 0.6;
        size_t num_points = reference_path_length; /* how many point "int the future" to be plotted. */

        for (size_t i = 0; i < num_points; i++) {
            double future_x;
            future_x = poly_inc * i;
            double future_y = polyeval(coeffs, future_x);
            next_x_vals.push_back(future_x);
            next_y_vals.push_back(future_y);
        }

        reference_path_id++;
        if (reference_path_id > 10000) {
            reference_path_id = 100;
        }
        reference_path.id = reference_path_id;
        reference_path.header.frame_id = "vehicle_odometry";
        reference_path.header.stamp = this->get_clock()->now();
        reference_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        reference_path.action = visualization_msgs::msg::Marker::ADD;
        reference_path.lifetime = rclcpp::Duration(20ms);    // 如果不想可视化所有的，就设置为 1ms，想可视化所有的就设置为 0ns，可视化一定时间长度的可以自定义时间
        reference_path.scale.x = 0.04;
        reference_path.scale.y = 0.04;
        reference_path.scale.z = 0.04;
        reference_path.color.b = 1.0;
        reference_path.color.a = 1.0;

        reference_path.points.clear();
        geometry_msgs::msg::Point p;
        // 可视化拟合后反求的结果
        for (uint i = 0; i < next_x_vals.size(); i++) {
            if (i % 2 == 0) {
                p.x = next_x_vals[i];
            } else {
                p.y = next_y_vals[i];
                reference_path.points.push_back(p);
            }
        }

        // mpc_output_path;
        mpc_output_path.id = reference_path_id;
        mpc_output_path.header.frame_id = "vehicle_odometry";
        mpc_output_path.header.stamp = this->get_clock()->now();
        mpc_output_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        mpc_output_path.action = visualization_msgs::msg::Marker::ADD;
        mpc_output_path.lifetime = rclcpp::Duration(200ms);
        mpc_output_path.scale.x = 0.04;
        mpc_output_path.scale.y = 0.04;
        mpc_output_path.scale.z = 0.04;
        mpc_output_path.color.r = 1.0;
        mpc_output_path.color.a = 1.0;

        mpc_output_path.points.clear();

        geometry_msgs::msg::Point pp;
        // 可视化原始点
        for (uint i = 2; i < vars.size(); i++) {
            if (i % 2 == 0) {
                pp.x = vars[i];
            } else {
                pp.y = vars[i];
                mpc_output_path.points.push_back(pp);
                // std::cout << pp.x << "  mpc output  " << pp.y << std::endl;
            }
        }
        // cout << "global_path_remap_y.size() " << global_path_remap_y.size() << endl;
        // for (uint i = 0; i < uint(global_path_remap_y.size()); i++) {
        //     pp.x = global_path_remap_x[i];
        //     pp.y = global_path_remap_y[i];
        //     mpc_output_path.points.push_back(pp);
        //     std::cout << "iiiiii: " << i << endl;
        //     // std::cout << pp.x << "  mpc output  " << pp.y << std::endl;
        // }

        vehicle_control_target_velocity.velocity = target_v;
        vehicle_control_target_velocity_publisher->publish(vehicle_control_target_velocity);
        // cout << "control_cmd.steer: " << control_cmd.steer << endl;
        // cout << "~~ vehicleState_.v: " << vehicleState_.velocity * 3.6 << ", target_point_.v: " << target_point_.v << ", v_err: " << v_err << endl;
        // cout << "yaw_err: " << yaw_err << endl;

        end_mpc = this->now();
        iteration_time_length = (end_mpc - start_mpc).nanoseconds();
        mpc_iteration_duration_msg.data = iteration_time_length / 1000000;
        mpc_iteration_time_publisher->publish(mpc_iteration_duration_msg);
        RCLCPP_INFO(this->get_logger(), "mpc iteration time: %f ms", iteration_time_length / 1000000);

        mpc_reference_path_publisher->publish(reference_path);
        mpc_output_path_publisher->publish(mpc_output_path);
    }
}

int main(int argc, char** argv) {
    RCLCPP_INFO(LOGGER, "Initializa Node~");
    std::cout << argv[0] << std::endl;
    rclcpp::init(argc, argv);
    auto n = std::make_shared<MPCControllerNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}