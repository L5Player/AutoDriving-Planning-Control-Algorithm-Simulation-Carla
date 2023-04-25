/**
 * @file planning_base.h
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 20223-02-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __PlanningBase__
#define __PlanningBase__
#pragma once

// #include "lattice/lattice.h"

namespace l5player {
namespace planning{
    
class PlanningBase {

public:
    PlanningBase() = default;

    virtual ~PlanningBase()=default;

    virtual DiscretizedTrajectory plan(const TrajectoryPoint &planning_init_point,
      const PlanningTarget &planning_target,
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<double> &accumulated_s,
      const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
      const double &init_relative_time, const double lon_decision_horizon, const double &absolute_time) = 0;
};

// PlanningBase::PlanningBase(){}

} // l5player
} // planning 

#endif // !__lattice__
