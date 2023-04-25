#pragma once
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <nav_msgs/msg/path.h>
#include "path_points.h"
#include "polygon2d.h"

// #include "auxiliary_function.h"
#include "Obstacle_avoid.h"

// #include <lanelet2_extension/io/autoware_osm_parser.h>
// #include <lanelet2_extension/projection/mgrs_projector.h>
// #include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/utilities.h>
// #include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>

class SL_Boundary
{
public:
  SL_Boundary() = default;
  ~SL_Boundary() = default;
  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType
  {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  std::string TypeName() const
  {
    if (boundary_type_ == BoundaryType::FOLLOW)
    {
      return "FOLLOW";
    }
    else if (boundary_type_ == BoundaryType::KEEP_CLEAR)
    {
      return "KEEP_CLEAR";
    }
    else if (boundary_type_ == BoundaryType::OVERTAKE)
    {
      return "OVERTAKE";
    }
    else if (boundary_type_ == BoundaryType::STOP)
    {
      return "STOP";
    }
    else if (boundary_type_ == BoundaryType::YIELD)
    {
      return "YIELD";
    }
    else if (boundary_type_ == BoundaryType::UNKNOWN)
    {
      return "UNKNOWN";
    }
    return "";
  }

  BoundaryType boundary_type() const
  {
    return boundary_type_;
  }
  void SetBoundaryType(const BoundaryType &boundary_type)
  {
    boundary_type_ = boundary_type;
  }
  const std::string &id() const
  {
    return obstacle_id;
  }

  void add_boundary_point(SLPoint boundary_points_)
  {
    boundary_points.push_back(boundary_points_);
  }
  std::vector<SLPoint> boundary_point() const
  {
    return boundary_points;
  }

private:
  BoundaryType boundary_type_; //障碍物边界类型
  std::vector<SLPoint> boundary_points;

public:
  std::string obstacle_id;
  //障碍物左右位置
  std::string obstacle_position;
  //障碍物的速度
  double obstacle_velocity;
  //障碍物朝向
  double obstacle_theta;
  //中心坐标
  geometry_msgs::msg::Pose centerpoint;
  //障碍物的长宽
  double obstacle_length;
  double obstacle_width;
  //是不是静态障碍物
  bool is_static_obstacle;
  //障碍物的类型
  int obstacle_type;

  //数值边界
  double start_l_;
  double end_l_;
  double start_s_;
  double end_s_;

  //时间戳
  rclcpp::Time timestamp_;
};

class ST_Boundary : public common::math::Polygon2d
{
public:
  ST_Boundary() = default;
  ST_Boundary(const std::vector<std::pair<STPoint, STPoint>> &point_pairs, bool is_accurate_boundary);
  ~ST_Boundary() = default;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType
  {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  std::string TypeName() const
  {
    if (boundary_type_ == BoundaryType::FOLLOW)
    {
      return "FOLLOW";
    }
    else if (boundary_type_ == BoundaryType::KEEP_CLEAR)
    {
      return "KEEP_CLEAR";
    }
    else if (boundary_type_ == BoundaryType::OVERTAKE)
    {
      return "OVERTAKE";
      return "STOP";
    }
    else if (boundary_type_ == BoundaryType::YIELD)
    {
      return "YIELD";
    }
    else if (boundary_type_ == BoundaryType::UNKNOWN)
    {
      return "UNKNOWN";
    }
    return "";
  }
  BoundaryType boundary_type() const
  {
    return boundary_type_;
  }
  void SetBoundaryType(const BoundaryType &boundary_type)
  {
    boundary_type_ = boundary_type;
  }
  const std::string &id() const
  {
    return obstacle_id;
  }
  double characteristic_length() const
  {
    // return characteristic_length_;//啥意思
    return obstacle_length;
  }
  bool IsPointInBoundary(const STPoint &st_point) const;
  bool GetBoundarySRange(const double curr_time, double *s_upper, double *s_lower) const;
  bool GetIndexRange(const std::vector<STPoint> &points, const double t, size_t *left, size_t *right) const;
  bool IsPointNear(const LineSegment2d &seg, const Vec2d &point, const double max_dist);
  void RemoveRedundantPoints(std::vector<std::pair<STPoint, STPoint>> *point_pairs);
  bool GetUnblockSRange(const double curr_time, double *s_upper, double *s_lower) const;
  bool GetBoundarySlopes(const double curr_time, double *ds_upper,
                         double *ds_lower) const;

  void SetUpper_points(const std::vector<STPoint> &upper_points)
  {
    upper_points_ = std::move(upper_points);
  }
  void SetLower_points(const std::vector<STPoint> &lower_points)
  {
    lower_points_ = std::move(lower_points);
  }

  std::vector<STPoint> getUpper_points() const
  {
    return upper_points_;
  }
  std::vector<STPoint> getetLower_points() const
  {
    return lower_points_;
  }

  void set_upper_left_point(STPoint st_point)
  {
    upper_left_point_ = std::move(st_point);
  }

  void set_upper_right_point(STPoint st_point)
  {
    upper_right_point_ = std::move(st_point);
    max_s_ = upper_right_point_.s();
    max_t_ = upper_right_point_.t();
  }

  void set_bottom_left_point(STPoint st_point)
  {
    bottom_left_point_ = std::move(st_point);
    min_s_ = bottom_left_point_.s();
    min_t_ = bottom_left_point_.t();
  }

  void set_bottom_right_point(STPoint st_point)
  {
    bottom_right_point_ = std::move(st_point);
  }

private:
  BoundaryType boundary_type_; //障碍物边界类型
  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  double characteristic_length_;

public:
  std::string obstacle_id;

  //顶点
  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;

  //存储T和S的对应值
  std::vector<std::pair<double, double>> time_start_s;
  std::vector<std::pair<double, double>> time_end_s;

  //障碍物左右位置
  std::string obstacle_position;
  //障碍物的速度
  double obstacle_velocity;
  //障碍物朝向
  double obstacle_theta;
  //中心坐标
  geometry_msgs::msg::Pose centerpoint;
  //障碍物的长宽
  double obstacle_length;
  double obstacle_width;
  //是不是静态障碍物
  bool is_static_obstacle;
  //障碍物的类型
  int obstacle_type;

  //数值边界
  double min_s_;
  double max_s_;
  double max_t_;
  double min_t_;

  //时间戳
  rclcpp::Time timestamp_;
};