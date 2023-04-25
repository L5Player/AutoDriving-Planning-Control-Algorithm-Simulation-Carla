#pragma once
#include <iostream>
#include <vector>
#include "vec2d.h"
#include "reference_point.h"
#include "path_points.h"

//参考线
class PathMatcher
{
public:
  PathMatcher() = default;
  ~PathMatcher() = default;
  static ReferencePoint MatchToPath(const double s, const std::vector<ReferencePoint> &reference_points);

  static ReferencePoint MatchToPath(const std::vector<ReferencePoint> &reference_points,
                                    const double x, const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint> &reference_line,
                               const double s);

  static std::pair<double, double> GetPathFrenetCoordinate(const std::vector<ReferencePoint> &reference_points, const double x,
                                                           const double y);

  static ReferencePoint FindProjectionPoint(const ReferencePoint &p0,
                                            const ReferencePoint &p1, const double x,
                                            const double y);

  static bool ComputePathProfile(const std::vector<std::pair<double, double>> &xy_points, std::vector<double> *headings,
                                 std::vector<double> *accumulated_s, std::vector<double> *kappas,
                                 std::vector<double> *dkappas);
  // static void RemoveDuplicates(std::vector<ReferencePoint> *points);

  static ReferencePoint InterpolateUsingLinearApproximation(const ReferencePoint &p0, const ReferencePoint &p1,
                                                            const double s);
                                                            
  static PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                                       const PathPoint &p1,
                                                       const double s);

private:
  const double kDuplicatedPointsEpsilon = 1e-7;
  // std::shared_ptr<Auxiliary> aux;
};
