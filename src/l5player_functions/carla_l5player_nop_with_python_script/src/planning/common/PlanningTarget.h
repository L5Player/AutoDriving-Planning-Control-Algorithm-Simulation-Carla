#ifndef PLANNINGTARGET_H
#define PLANNINGTARGET_H
#include <vector>
#include <iostream>
class PlanningTarget
{
public:
  PlanningTarget();
  PlanningTarget(const double speed, const std::vector<double>& reference_s);
  ~PlanningTarget();
  double cruise_speed() const;
  bool has_stop_point() const;
  double stop_point() const;

private:
  std::vector<double> reference_s_;
  double speed_;
};
#endif