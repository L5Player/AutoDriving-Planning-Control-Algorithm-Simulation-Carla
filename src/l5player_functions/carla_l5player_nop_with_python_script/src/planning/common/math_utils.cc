#include "math_utils.h"
#include <cmath>
#include <utility>

namespace common
{
  namespace math
  {
    double Sigmoid(const double x)
    {
      return 1.0 / (1.0 + std::exp(-x));
    }

    double NormalizeAngle(const double angle)
    {
      double a = std::fmod(angle + M_PI, 2.0 * M_PI);// 取余
      if (a < 0.0)
      {
        a += (2.0 * M_PI);
      }
      return a - M_PI;
    }

    double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1, const Vec2d &end_point_2)
    {
      return (end_point_1 - start_point).CrossProdVec(end_point_2 - start_point);
    }

    double WrapAngle(const double angle)
    {
      const double new_angle = std::fmod(angle, M_PI * 2.0);
      return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
    }

    std::pair<double, double> Cartesian2Polar(double x, double y)
    {
      double r = std::sqrt(x * x + y * y);
      double theta = std::atan2(y, x);
      return std::make_pair(r, theta);
    }

    double slerp(const double a0, const double t0, const double a1, const double t1, const double t)
    {
      if (std::abs(t1 - t0) <= 1e-10)
      {
        // ROS_INFO("input time difference is too small");
        return NormalizeAngle(a0);
      }
      const double a0_n = NormalizeAngle(a0);
      const double a1_n = NormalizeAngle(a1);
      double d = a1_n - a0_n;
      if (d > M_PI)
      {
        d = d - 2 * M_PI;
      }
      else if (d < -M_PI)
      {
        d = d + 2 * M_PI;
      }

      const double r = (t - t0) / (t1 - t0);
      const double a = a0_n + d * r;// 线性插值
      return NormalizeAngle(a);
    }

    SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0, const SLPoint &p1, const double w)
    {
      // CHECK_GE(w, 0.0);

      SLPoint p;
      p.set_s((1 - w) * p0.s + w * p1.s);
      p.set_l((1 - w) * p0.l + w * p1.l);
      return p;
    }

    PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0, const PathPoint &p1, const double s)
    {
      double s0 = p0.s;
      double s1 = p1.s;

      PathPoint path_point;
      double weight = (s - s0) / (s1 - s0);
      double x = (1 - weight) * p0.x + weight * p1.x;
      double y = (1 - weight) * p0.y + weight * p1.y;
      double theta = common::math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
      double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
      double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
      path_point.set_x(x);
      path_point.set_y(y);
      path_point.set_theta(theta);
      path_point.set_kappa(kappa);
      path_point.set_dkappa(dkappa);
      path_point.set_s(s);
      return path_point;
    }

    TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0, const TrajectoryPoint &tp1,
                                                        const double t)
    {
      if (!tp0.has_path_point() || !tp1.has_path_point())
      {
        std::cout << "tp0 and tp1 has no path point"
                  << "\n";
        TrajectoryPoint p;
        return p;
      }
      const PathPoint pp0 = tp0.path_point();
      const PathPoint pp1 = tp1.path_point();
      double t0 = tp0.relative_time;
      double t1 = tp1.relative_time;

      TrajectoryPoint tp;
      tp.set_v(lerp(tp0.v, t0, tp1.v, t1, t));
      tp.set_a(lerp(tp0.a, t0, tp1.a, t1, t));
      tp.set_relative_time(t);
      tp.set_x(lerp(pp0.x, t0, pp1.x, t1, t));
      tp.set_y(lerp(pp0.y, t0, pp1.y, t1, t));
      tp.set_theta(slerp(pp0.theta, t0, pp1.theta, t1, t));
      tp.set_kappa(lerp(pp0.kappa, t0, pp1.kappa, t1, t));
      tp.set_dkappa(lerp(pp0.dkappa, t0, pp1.dkappa, t1, t));
      tp.set_s(lerp(pp0.s, t0, pp1.s, t1, t));

      return tp;
    }
  } // namespace math
} // namespace common
