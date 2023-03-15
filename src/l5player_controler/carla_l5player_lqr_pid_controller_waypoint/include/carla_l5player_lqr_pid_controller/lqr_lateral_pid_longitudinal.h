#ifndef LQR_LATERAL_PID_LONGITUDINAL_H_
#define LQR_LATERAL_PID_LONGITUDINAL_H_

#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

#include "carla_l5player_lqr_pid_controller/lqr_controller.h"
#include "carla_l5player_lqr_pid_controller/pid_controller.h"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "carla_msgs/msg/carla_status.hpp"
#include "carla_msgs/msg/carla_vehicle_target_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace l5player::control;
using namespace std;

class LQRControllerNode : public rclcpp::Node {
   public:
    LQRControllerNode();
    ~LQRControllerNode();
    bool init();

    // 声明定时器
    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer;
    void VehicleControllerIterationCallback();

    rclcpp::TimerBase::SharedPtr global_path_publish_timer;
    void GlobalPathPublishCallback();

    // 声明广播器，订阅器以及订阅回调函数和广播消息类型

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_waypoint_subscriber;
    void RouteWaypointCallback(nav_msgs::msg::Path::SharedPtr msg);

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

    // 计算两点之间的距离
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
    double controlFrequency_ = 100;    // 控制频率
    double goalTolerance_ = 0.5;       // 到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;
    int cnt;
    bool if_get_waypoint = false;

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
    std::unique_ptr<l5player::control::PIDController> pid_controller_longitudinal;
    std::unique_ptr<l5player::control::LqrController> lqr_controller_lateral;
};
#endif /* __LQR_CONTROLLER_NODE_H__ */
