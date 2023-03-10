#ifndef MPC_LATERAL_LONGITUDINAL_H_
#define MPC_LATERAL_LONGITUDINAL_H_

#include <stdint.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "carla_l5player_mpc_controller/mpc_controller.h"
#include "carla_l5player_mpc_controller/pid_controller.h"

#include "carla_l5player_mpc_controller/helpers.h"
#include "carla_l5player_mpc_controller/spline.h"
#include "carla_l5player_mpc_controller/coordinate_transform.h"

#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_status.hpp"
#include <std_msgs/msg/float32.hpp>
#include "carla_msgs/msg/carla_vehicle_target_velocity.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "rclcpp/rate.hpp"
#include <cstdio>
#include <thread>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iomanip>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/unsupported/Eigen/Splines>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>


using namespace l5player::control;
// using namespace std;

using namespace std::chrono_literals; // 表示时间长度的命名空间
//调用嵌套空间std::chrono_literals下的函数
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;

using CppAD::AD;
using Eigen::VectorXd;

class MPCControllerNode : public rclcpp::Node {
   public:
    MPCControllerNode();
    ~MPCControllerNode();
    bool init();

    // 声明定时器
    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer;
    void VehicleControllerIterationCallback();   

    rclcpp::TimerBase::SharedPtr global_path_publish_timer;
    void GlobalPathPublishCallback();

    // 声明广播器，订阅器以及订阅回调函数和广播消息类型

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;
    void OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lacalization_data_imu_subscriber;
    void IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_visualization_publisher;
    nav_msgs::msg::Path history_path;
    geometry_msgs::msg::PoseStamped history_path_points;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    nav_msgs::msg::Path global_path;
    geometry_msgs::msg::PoseStamped this_pose_stamped;

    // 声明类内其他函数
    void loadRoadmap(const std::string &roadmap_path);

    //计算两点之间的距离
    double pointDistance(const TrajectoryPoint &point, const double x, const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const double x1, const double y1, const double x, const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    }

    rclcpp::Publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>::SharedPtr vehicle_control_target_velocity_publisher;
    carla_msgs::msg::CarlaVehicleTargetVelocity vehicle_control_target_velocity;

    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr carla_status_subscriber;
    void VehicleStatusCallback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
    
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr vehicle_control_publisher;
    carla_msgs::msg::CarlaEgoVehicleControl control_cmd;

    TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);
    double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y);

   private:
    double targetSpeed_ = 5;
    double controlFrequency_ = 100;                 //控制频率
    double goalTolerance_ = 0.5;                    //到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;
    int cnt;

    std::string _line;
    std::vector<std::pair<double, double>> xy_points;
    std::vector<double> v_points;

    TrajectoryData planning_published_trajectory;
    TrajectoryPoint goal_point;
    std::vector<TrajectoryPoint> trajectory_points_;

    std::string roadmap_path;
    double speed_P, speed_I, speed_D, target_speed;

    VehicleState vehicleState_;
    // TrajectoryData planningPublishedTrajectory_;    //跟踪的轨迹
    // TrajectoryPoint goalPoint_;                     //终点

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;
    // std::unique_ptr<l5player::control::PIDController> pid_controller_longitudinal;
    // std::unique_ptr<l5player::control::MPCController> mpc_controller_lateral;

    double delta; // the current steering angle from carla
    double a_longitudinal; // the acceleration 
    double a_lateral;

    double steer_value;
    double throttle_value;

    double px;
    double py;
    double psi;
    double yaw_rate;
    double v_longitudinal;
    double v_lateral;

    vector<double> global_path_remap_x;
    vector<double> global_path_remap_y;

    int former_point_of_current_position;
    double old_steer_value;
    double old_throttle_value;

    visualization_msgs::msg::Marker reference_path;
    visualization_msgs::msg::Marker mpc_output_path;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_reference_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_output_path_publisher;
    int reference_path_id = 101;

    int mpc_cte_weight_int;
    int mpc_epsi_weight_int;
    int mpc_v_weight_int;
    int mpc_steer_actuator_cost_weight_int;
    int mpc_acc_actuator_cost_weight_int;
    int mpc_change_steer_cost_weight_int;
    int mpc_change_accel_cost_weight_int;

    double ref_v;

    double vehicle_steering_ratio_double;
    double vehicle_Lf_double;

    int mpc_control_horizon_length_int;
    double mpc_control_step_length_double;

    bool mpc_tracking_enable_bool;

    mpc_controller mpc;

    double target_v; // km/h

    // 考虑道路曲率的车辆运动学模型里面可以设置的参数
    double steering_ratio;
    double kinamatic_para_Lf;

    // MPC预测模型里可以调节的参数
    int mpc_control_horizon_length; // MPC求解的时候,一次求解里面 MPC 预测的步数,乘以步长,就是 MPC 一次求解考虑的未来的范围大小.
    double mpc_control_step_length; //Original 0.1 不是控制步长,而是模型离散化后进行预测时的预测步长

    // MPC 目标函数里面可以调节的权重
    int cte_weight; //Original2000
    int epsi_weight;
    int v_weight;
    int steer_actuator_cost_weight; //Original 6000
    int acc_actuator_cost_weight; //Original 6000
    int change_steer_cost_weight;
    int change_accel_cost_weight;

    double ins_delay;

    // MPC 里面拟合前方参考路径的时候使用的路点的数量，这个意义更新为使用的前方多少距离内的点
    double reference_path_length;

    int reference_path_points_number;

    double controller_delay_compensation; // 控制信号延时补偿

    // 在 RVIZ 里面可视化根据参考路径拟合得到的多项式表达的曲线时,显示的点的数量和点的距离参数
    double point_distance_of_reference_line_visualization; // step on x
    int points_number_of_reference_line_visualization;     /* how many point "int the future" to be plotted. */
    bool mpc_enable_signal = true;
    int working_mode;
    int with_planner_flag; // MPC做全局纯跟踪还是与规划算法上下贯通的标志 为 0 做纯跟踪，为 1 与规划算法上下贯通
    std_msgs::msg::Float32 mpc_iteration_duration_msg = std_msgs::msg::Float32();
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mpc_iteration_time_publisher;


};
#endif /* __MPC_CONTROLLER_NODE_H__ */
