#include "constraint_checker.h"

namespace
{
  template <typename T>
  bool WithinRange(const T v, const T lower, const T upper)
  {
    return lower <= v && v <= upper;
  }
} // namespace

ConstraintChecker::Result ConstraintChecker::ValidTrajectory(const DiscretizedTrajectory &trajectory)
{
  const double kMaxCheckRelativeTime = Config_.FLAGS_trajectory_time_length;
  for (const auto &p : trajectory)
  {
    double t = p.relative_time;
    if (t > kMaxCheckRelativeTime)
    {
      // ROS_INFO("relative time is bigger than 8s");
      break;
    }
    double lon_v = p.v;
    if (!WithinRange(lon_v, Config_.FLAGS_speed_lower_bound, Config_.FLAGS_speed_upper_bound))
    {
      // std::cout << "Velocity at relative time " << t << " exceeds bound, value: " << lon_v << ", bound ["
      //           << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound << "]."
      //           << "\n";
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }

    double lon_a = p.a;
    if (!WithinRange(lon_a, Config_.FLAGS_longitudinal_acceleration_lower_bound, Config_.FLAGS_longitudinal_acceleration_upper_bound))
    {
      // std::cout << "Longitudinal acceleration at relative time " << t << " exceeds bound, value: " << lon_a
      //           << ", bound [" << FLAGS_longitudinal_acceleration_lower_bound << ", "
      //           << FLAGS_longitudinal_acceleration_upper_bound << "]."
      //           << "\n";
      return Result::LON_ACCELERATION_OUT_OF_BOUND;
    }

    double kappa = p.kappa;
    if (!WithinRange(kappa, -Config_.FLAGS_kappa_bound, Config_.FLAGS_kappa_bound))
    {
      // std::cout << "Kappa at relative time " << t << " exceeds bound, value: " << kappa << ", bound ["
      //           << -Config_.FLAGS_kappa_bound << ", " << Config_.FLAGS_kappa_bound << "]."
      //           << "\n";
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }

  for (size_t i = 1; i < trajectory.size(); ++i)
  {
    const auto &p0 = trajectory[static_cast<uint32_t>(i - 1)];
    const auto &p1 = trajectory[static_cast<uint32_t>(i)];

    if (p1.relative_time > kMaxCheckRelativeTime)
    {
      break;
    }

    // lon_jerk和lat_a
    double t = p0.relative_time;

    double dt = p1.relative_time - p0.relative_time;
    double d_lon_a = p1.a - p0.a;
    double lon_jerk = d_lon_a / dt;
    if (!WithinRange(lon_jerk, Config_.FLAGS_longitudinal_jerk_lower_bound, Config_.FLAGS_longitudinal_jerk_upper_bound))
    {
      // std::cout << "Longitudinal jerk at relative time " << t << " exceeds bound, value: " << lon_jerk << ", bound ["
      //           << Config_.FLAGS_longitudinal_jerk_lower_bound << ", " << Config_.FLAGS_longitudinal_jerk_upper_bound << "].";
      return Result::LON_JERK_OUT_OF_BOUND;
    }

    double lat_a = p1.v * p1.v * p1.kappa;
    if (!WithinRange(lat_a, -Config_.FLAGS_lateral_acceleration_bound, Config_.FLAGS_lateral_acceleration_bound))
    {
      // std::cout << "Lateral acceleration at relative time " << t << " exceeds bound, value: " << lat_a << ",bound["
      //           << -FLAGS_lateral_acceleration_bound << ", " << FLAGS_lateral_acceleration_bound << "].";
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }

    // TODO(zhangyajia): this is temporarily disabled
    // due to low quality reference line.
    /*
        double d_lat_a = p1.v * p1.v * p1.kappa - p0.v * p0.v * p0.kappa;
        double lat_jerk = d_lat_a / dt;
        if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound))
        {
          // std::cout << "Lateral jerk at relative time " << t << " exceeds bound, value: " << lat_jerk << ", bound
          ["
          //           << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound << "].";
          return Result::LAT_JERK_OUT_OF_BOUND;
        }
      */
  }

  return Result::VALID;
}