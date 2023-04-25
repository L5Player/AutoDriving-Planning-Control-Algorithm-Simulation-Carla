#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include "trajectoryPoint.h"

namespace Prediction
{
    class Ob_Trajectory
    {
    private:
        std::vector<TrajectoryPoint> trajectory_points;

    public:
        Ob_Trajectory();
        ~Ob_Trajectory() = default;

        int trajectory_point_size() const
        {
            return trajectory_points.size();
        }
        std::vector<TrajectoryPoint> trajectory_point() const
        {
            return trajectory_points;
        }
        void Set_Trajectory(std::vector<TrajectoryPoint> trajectory_points_)
        {
            trajectory_points = std::move(trajectory_points_);
        }
    };

} // namespace prediction
