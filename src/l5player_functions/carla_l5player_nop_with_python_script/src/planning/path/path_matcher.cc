#include "path_matcher.h"
#include "math_utils.h"

//找匹配点ReferencePoint,这个加了插值InterpolateUsingLinearApproximation，也不是常函数
ReferencePoint PathMatcher::MatchToPath(const double s, const std::vector<ReferencePoint> &reference_points)
{
  auto comp = [](const ReferencePoint &point, const double &s)
  { return point.accumulated_s_ < s; };
  auto it_lower = std::lower_bound(reference_points.begin(), reference_points.end(), s, comp);
  if (it_lower == reference_points.begin())
  {
    return reference_points.front();
  }
  else if (it_lower == reference_points.end())
  {
    return reference_points.back();
  }
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

ReferencePoint PathMatcher::MatchToPath(const std::vector<ReferencePoint> &reference_points,
                                        const double x, const double y)
{
  // CHECK_GT(reference_line.size(), 0);

  auto func_distance_square = [](const ReferencePoint &point, const double x,
                                 const double y)
  {
    double dx = point.x_ - x;
    double dy = point.y_ - y;
    return dx * dx + dy * dy;
  };

  double distance_min = func_distance_square(reference_points.front(), x, y);
  std::size_t index_min = 0;

  for (std::size_t i = 1; i < reference_points.size(); ++i)
  {
    double distance_temp = func_distance_square(reference_points[i], x, y);
    if (distance_temp < distance_min)
    {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      (index_min + 1 == reference_points.size()) ? index_min : index_min + 1;

  if (index_start == index_end)
  {
    return reference_points[index_start];
  }

  return FindProjectionPoint(reference_points[index_start],
                             reference_points[index_end], x, y);
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint> &reference_line,
                                   const double s)
{
  auto comp = [](const PathPoint &point, const double s)
  {
    return point.s < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
  if (it_lower == reference_line.begin())
  {
    return reference_line.front();
  }
  else if (it_lower == reference_line.end())
  {
    return reference_line.back();
  }

  // interpolate between it_lower - 1 and it_lower
  // return interpolate(*(it_lower - 1), *it_lower, s);
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(
    const std::vector<ReferencePoint> &reference_points, const double x,
    const double y)
{
  auto matched_path_point = MatchToPath(reference_points, x, y);
  double rtheta = matched_path_point.heading_;
  double rx = matched_path_point.x_;
  double ry = matched_path_point.y_;
  double delta_x = x - rx;
  double delta_y = y - ry;
  double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
  std::pair<double, double> relative_coordinate;
  relative_coordinate.first = matched_path_point.accumulated_s_;
  relative_coordinate.second = std::copysign(std::hypot(delta_x, delta_y), side);
  return relative_coordinate;
}

ReferencePoint PathMatcher::FindProjectionPoint(const ReferencePoint &p0,
                                                const ReferencePoint &p1, const double x,
                                                const double y)
{
  double v0x = x - p0.x_;
  double v0y = y - p0.y_;

  double v1x = p1.x_ - p0.x_;
  double v1y = p1.y_ - p0.y_;

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  double delta_s = dot / v1_norm;
  return InterpolateUsingLinearApproximation(p0, p1, p0.accumulated_s_ + delta_s);
}

ReferencePoint PathMatcher::InterpolateUsingLinearApproximation(const ReferencePoint &p0, const ReferencePoint &p1,
                                                                const double s)
{
  double s0 = p0.s();
  double s1 = p1.s();

  ReferencePoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.get_x() + weight * p1.get_x();
  double y = (1 - weight) * p0.get_y() + weight * p1.get_y();
  double theta = common::math::slerp(p0.heading(), p0.s(), p1.heading(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_heading(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_s(s);
  return path_point;
}

PathPoint PathMatcher::InterpolateUsingLinearApproximation(const PathPoint &p0,
                                                           const PathPoint &p1,
                                                           const double s)
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

//计算参考路径点的 headings accumulated_s kappas dkappas
bool PathMatcher::ComputePathProfile(const std::vector<std::pair<double, double>> &xy_points,
                                     std::vector<double> *headings, std::vector<double> *accumulated_s,
                                     std::vector<double> *kappas, std::vector<double> *dkappas)
{
  // CHECK_NOTNULL(headings);
  // CHECK_NOTNULL(kappas);
  // CHECK_NOTNULL(dkappas);
  headings->clear();
  kappas->clear();
  dkappas->clear();

  if (xy_points.size() < 2)
  {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i)
  {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0)
    {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    }
    else if (i == points_size - 1)
    {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    }
    else
    {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i)
  {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points[0].first;
  double fy = xy_points[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i)
  {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    double end_segment_s = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i)
  {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0)
    {
      xds = (xy_points[i + 1].first - xy_points[i].first) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
    }
    else if (i == points_size - 1)
    {
      xds = (xy_points[i].first - xy_points[i - 1].first) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
    }
    else
    {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i)
  {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0)
    {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i));
    }
    else if (i == points_size - 1)
    {
      xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i) - accumulated_s->at(i - 1));
    }
    else
    {
      xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i)
  {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i)
  {
    double dkappa = 0.0;
    if (i == 0)
    {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
    }
    else if (i == points_size - 1)
    {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
    }
    else
    {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

// //移除重叠的参考路径点
// void PathMatcher::RemoveDuplicates(std::vector<ReferencePoint> *points)
// {
//   // CHECK_NOTNULL(points);
//   // int count = 0;
//   // const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
//   // for (size_t i = 0; i < points->size(); ++i)
//   // {
//   //   if (count == 0 || (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit)
//   //   {
//   //     (*points)[count++] = (*points)[i];
//   //   }
//   //   else
//   //   {
//   //     (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
//   //   }
//   // }
//   // points->resize(count);
// }
