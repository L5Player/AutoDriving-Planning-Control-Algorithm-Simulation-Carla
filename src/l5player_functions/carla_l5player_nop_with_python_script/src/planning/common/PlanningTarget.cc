/**
 * @file PlanningTarget.cpp 输入：巡航速度，是否有停止点，停止点的位置
 * @author feifei (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "PlanningTarget.h"
PlanningTarget::PlanningTarget()
{
}
PlanningTarget::~PlanningTarget()
{
}
PlanningTarget::PlanningTarget(const double speed, const std::vector<double> &reference_s)
    : speed_(speed), reference_s_(reference_s)
{
}
double PlanningTarget::cruise_speed() const
{
  return speed_;
}
double PlanningTarget::stop_point() const
{
  return reference_s_.back();
}
bool PlanningTarget::has_stop_point() const
{
  return false;
}