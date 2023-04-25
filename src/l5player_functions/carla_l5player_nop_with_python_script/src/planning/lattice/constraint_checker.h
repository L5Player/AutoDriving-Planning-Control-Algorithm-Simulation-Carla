#pragma once
#include <memory>
#include <vector>
#include "plan_init.h"
#include "FrenetPath.h"

class ConstraintChecker
{
public:
  enum class Result
  {
    VALID,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    LAT_VELOCITY_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND,
  };
  ConstraintChecker() = default;
  ~ConstraintChecker() = default;
  Result ValidTrajectory(const DiscretizedTrajectory &trajectory);

private:
  ;
};