#include "FrenetPath.h"
#include "math_utils.h"

FrenetFramePath::FrenetFramePath(std::vector<FrenetFramePoint> points)
    : std::vector<FrenetFramePoint>(std::move(points))
{
}

double FrenetFramePath::Length() const
{
  if (empty())
  {
    return 0.0;
  }
  return back().s - front().s;
}
double FrenetFramePath::Size() const
{
  return size();
}

FrenetFramePoint FrenetFramePath::EvaluateByS(const double s) const
{
  //   CHECK_GT(size(), 1);
  auto it_lower = std::lower_bound(begin(), end(), s, LowerBoundComparator);
  if (it_lower == begin())
  {
    return front();
  }
  else if (it_lower == end())
  {
    return back();
  }
  const auto &p0 = *(it_lower - 1);
  const auto s0 = p0.s;
  const auto &p1 = *it_lower;
  const auto s1 = p1.s;

  FrenetFramePoint p;
  p.set_s(s);
  p.set_l(common::math::lerp(p0.d, s0, p1.d, s1, s));
  p.set_dl(common::math::lerp(p0.d_d, s0, p1.d_d, s1, s));
  p.set_ddl(common::math::lerp(p0.d_dd, s0, p1.d_dd, s1, s));
  return p;
}

DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points) : std::vector<PathPoint>(std::move(path_points))
{
}

double DiscretizedPath::Length() const
{
  if (empty())
  {
    return 0.0;
  }
  return back().s - front().s;
}
double DiscretizedPath::Size() const
{
  return size();
}

PathPoint DiscretizedPath::Evaluate(const double path_s) const
{
  //   ACHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin())
  {
    return front();
  }
  if (it_lower == end())
  {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const
{
  auto func = [](const PathPoint &tp, const double path_s)
  {
    return tp.s < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const
{
  //   ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin())
  {
    return front();
  }
  if (it_upper == end())
  {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_upper - 1), *it_upper, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(const double path_s) const
{
  auto func = [](const double path_s, const PathPoint &tp)
  { return tp.s < path_s; };
  return std::upper_bound(begin(), end(), path_s, func);
}
