#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <cmath>
// #include <tf/tf.h>
#include <tf2/utils.h>
#include <boost/thread.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Dense>

// #include "CubicSpline2D.h"
#include "trajectoryPoint.h"
#include "Ob_prediction_trajectory.h"

//坐标点
struct PPoint
{
  double x; //坐标点x坐标
  double y; //坐标点y坐标
  PPoint(double _x, double _y) : x(_x), y(_y){};
  PPoint() : x(0), y(0){};
  PPoint(const PPoint &other) : x(other.x), y(other.y){};
  const PPoint operator=(const PPoint &p)
  {
    x = p.x;
    y = p.y;
    return *this;
  }
  bool operator==(const PPoint &p)
  {
    return (abs(x - p.x) < 0.0001 && abs(y - p.y) < 0.0001);
  }
};

class Obstacle_avoid
{
private:
  Eigen::Vector3d origin_mgrs_point_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Pinnacle_Visualization;
  // rclcpp::Publisher Pinnacle_Visualization;
  // PPoint p1, p2, p3, p4;
  PPoint p;
  nav_msgs::msg::Path referenceline;
  // rclcpp::Publisher referenceline_pub;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr Points_Visualization;
  // std::shared_ptr<rclcpp::Node> node_obstacle_;

public:
  Obstacle_avoid();
  Obstacle_avoid(Eigen::Vector3d &origin_mgrs_point);
  ~Obstacle_avoid();
  void CalculateCarBoundaryPoint(double car_length, double car_width, PPoint &center, double angle, PPoint &front_left,
                                 PPoint &back_left, PPoint &back_right, PPoint &front_right);
  void visualization(PPoint center, double ego_heading, double length, double width);
  void visualization_points(PPoint ob_left_front, PPoint ob_left_buttom, PPoint ob_right_front, PPoint ob_right_buttom);
  void Publish(std::pair<std::vector<double>, std::vector<double>> &path_point_after_interpolation_);
  double getCross(PPoint p1, PPoint p2, PPoint p);
  bool Inside_rectangle(PPoint p1, PPoint p2, PPoint p3, PPoint p4, PPoint p);

  // bool GetClosestLanelet(geometry_msgs::Pose &search_pose, lanelet::LaneletMapPtr &lanelet_map_ptr_,
  //                        lanelet::Lanelet *closest_lanelet, double distance_thresh);

  struct Point3d_s
  {
    double x;
    double y;
    double z;
  };

  //用此函数模拟动态障碍物的预测轨迹，即：前探距离=当前位置*预测距离，然后线性插值，来得到预测轨迹点
  Prediction::Ob_Trajectory Generater_Trajectory(geometry_msgs::msg::Pose ob_pos, double pre_time, double obstacle_theta, double obstacle_velocity);
  void average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double pre_time);
};
