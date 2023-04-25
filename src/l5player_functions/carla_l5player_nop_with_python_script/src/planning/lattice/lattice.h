/**
 * @file lattice.h 主要是订阅topic /referenceLine_smoothed，为参考线计算heading和kappa生成reference_point，之后进行速度规划和路径规划
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-12-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __lattice__
#define __lattice__
#pragma once

#include<iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include "reference_point.h"
#include "trajectoryPoint.h"

#include "l5player_nop_msgs/msg/gps.hpp"
#include "l5player_nop_msgs/msg/trajectory.h"
#include "l5player_nop_msgs/msg/trajectory_point.h"

#include "PlanningTarget.h"
#include "Obstacle.h"
#include "cartesian_frenet_conversion.h"
#include "trajectory1d_generator.h"
#include "trajectory_evaluator.h"
#include "trajectory_combiner.h"
#include "constraint_checker.h"
#include "collision_checker.h"
#include "planning_base.h"

namespace l5player{
namespace planning{

class lattice : public PlanningBase
{
public:
    lattice();
    ~lattice() = default;
    
    /**
     * @brief lattice主函数入口
     * 
     * @param planning_init_point 
     * @param planning_target 
     * @param obstacles 
     * @param accumulated_s 
     * @param reference_points 
     * @param lateral_optimization 
     * @param init_relative_time 
     * @param lon_decision_horizon 
     * @return DiscretizedTrajectory 
     */
    DiscretizedTrajectory plan(
      const TrajectoryPoint &planning_init_point,
      const PlanningTarget &planning_target,
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<double> &accumulated_s,
      const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
      const double &init_relative_time, const double lon_decision_horizon, const double &absolute_time) override;
    
    /**
     * @brief 规划起点frenet和cartesian坐标转换
     * 
     * @param matched_point 
     * @param cartesian_state 
     * @param ptr_s 
     * @param ptr_d 
     */
    void ComputeInitFrenetState(const ReferencePoint &matched_point, const TrajectoryPoint &cartesian_state,
                                std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d);
  

private:
    // handle
    // ros::NodeHandle n_;
    // // publisher
  	// ros::Publisher trajectory_pub_;
  	// // subscriber
  	// ros::Subscriber referenceLine_subscriber_, gps_sub_;

    // class
    TrajectoryCombiner trajectorycombiner_;
    ConstraintChecker constraintchecker_;

};


} // namespace planning
} // namespace l5player

#endif // !__lattice__

