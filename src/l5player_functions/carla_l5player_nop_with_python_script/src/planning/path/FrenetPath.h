#ifndef FRENET_H
#define FRENET_H
#include "trajectoryPoint.h"

class DiscretizedPath : public std::vector<PathPoint>
{
public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);
  double Length() const;
  double Size() const;
  PathPoint Evaluate(const double path_s) const;

  PathPoint EvaluateReverse(const double path_s) const;

protected:
  std::vector<PathPoint>::const_iterator QueryLowerBound(const double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(const double path_s) const;
};

class FrenetFramePath : public std::vector<FrenetFramePoint>
{
public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<FrenetFramePoint> points);

  double Size() const;
  double Length() const;
  FrenetFramePoint EvaluateByS(const double s) const;
  // /**
  //  * @brief Get the FrenetFramePoint that is within  SL_Boundary, or the one with
  //  * smallest l() in  SL_Boundary's s range [start_s(), end_s()]
  //  */
  // FrenetFramePoint GetNearestPoint(const  SL_Boundary& sl) const;

private:
  static bool LowerBoundComparator(const FrenetFramePoint &p, const double s)
  {
    return p.s < s;
  }
  static bool UpperBoundComparator(const double s, const FrenetFramePoint &p)
  {
    return s < p.s;
  }
};

#endif // FRENET_H
