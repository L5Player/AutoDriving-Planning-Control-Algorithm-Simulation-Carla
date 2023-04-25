#pragma once
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string>
#include <vector>
#include "path_points.h"
#include "plan_init.h"

class FrenetPath
{
public:
  // Frenet坐标系下的属性
  std::vector<double> t;     // time
  std::vector<double> d;     // lateral offset
  std::vector<double> d_d;   // lateral speed
  std::vector<double> d_dd;  // lateral acceleration
  std::vector<double> d_ddd; // lateral jerk
  std::vector<double> s;     // s position along spline
  std::vector<double> s_d;   // s speed
  std::vector<double> s_dd;  // s acceleration
  std::vector<double> s_ddd; // s jerk

  // 世界坐标系下的参数
  std::vector<double> x;     // x position
  std::vector<double> y;     // y position
  std::vector<double> theta; // yaw in rad
  double kappa;              // curvature曲率
  std::vector<double> dL;    // Running length / arc length
  std::vector<double> v;     // Tangential velocity
  std::vector<double> a;     // Tangential acceleration

  // Debug
  std::vector<double> ix;
  std::vector<double> iy;
  std::vector<double> iyaw;

  //横向代价属性
  double c_lateral_deviation;
  double c_lateral_velocity;
  double c_lateral_acceleration;
  double c_lateral_jerk;
  double Jd; // Lateral cost
  //纵向代价属性
  double c_longitudinal_acceleration;
  double c_longitudinal_dist;
  double c_longitudinal_jerk;
  double c_time_taken;
  double c_end_speed_deviation;
  double Js; // Longitudinal cost
  //总的代价函数
  double J; // Total cost
};

//--------------------TrajectoryPoint:轨迹点-----------------------//
class TrajectoryPoint
{
public:
  TrajectoryPoint();
  TrajectoryPoint(const InitialConditions &Initial);
  ~TrajectoryPoint() = default;

public:
  // 世界坐标系下的参数
  double x;      // x position
  double y;      // y position
  double z;      // z position
  double theta;  // yaw in rad
  double kappa;  // curvature曲率
  double dkappa; // curvature曲率导数
  double v;      // Tangential velocity
  double a;      // Tangential acceleration
  // Frenet坐标系下的属性
  double relative_time; // relative_time
  double absolute_time; // 轨迹的绝对时间
  double d;             // lateral offset
  double d_d;           // lateral speed
  double d_dd;          // lateral acceleration
  double s;             // s position along spline
  double s_d;           // s speed
  double s_dd;          // s acceleration

  double s_ddd;
  double d_ddd;
  PathPoint pathpoint_;

public:
  PathPoint path_point() const;
  bool has_path_point() const;
  void CopyFrom(PathPoint &path_point, FrenetFramePoint &frenet_point);

  void set_x(double x_);
  void set_y(double y_);
  void set_z(double z_);
  void set_theta(double theta_);
  void set_s(double s_);
  void set_kappa(double kappa_);
  void set_dkappa(double dkappa_);
  void set_v(double v_);
  void set_a(double a_);
  void set_relative_time(double relative_time_);
  void set_absolute_time(double absolute_time_);

private:
  bool has_path_point_;
};

class DiscretizedTrajectory : public std::vector<TrajectoryPoint>
{
public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint> &trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<TrajectoryPoint> &trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time) const;

  virtual size_t QueryNearestPoint(const Vec2d &position) const;

  size_t QueryNearestPointWithBuffer(const Vec2d &position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const TrajectoryPoint &trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<TrajectoryPoint> &trajectory_points)
  {
    if (!empty() && trajectory_points.size() > 1)
    {
      // CHECK(trajectory_points.back().relative_time < front().relative_time);
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const TrajectoryPoint &TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();
};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }
