#ifndef LQR_CONTROLLER_H_
#define LQR_CONTROLLER_H_
#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include <eigen3/Eigen/Core>
#include "l5player_nop/common.h"

namespace l5player {
namespace control {

using Matrix = Eigen::MatrixXd;

class LqrController {
   public:
    LqrController();
    ~LqrController();

    void LoadControlConf();
    void Init();

    bool ComputeControlCommand(const VehicleState &localization, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd);

   protected:
    void UpdateState(const VehicleState &vehicle_state);

    void UpdateMatrix(const VehicleState &vehicle_state);

    double ComputeFeedForward(const VehicleState &localization, double ref_curvature);

    void ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, LateralControlErrorPtr &lat_con_err);

    TrajectoryPointOri QueryNearestPointByPosition(const double x, const double y); 

    std::shared_ptr<TrajectoryData> QueryNearestPointByPositionWithPreview(const double x, const double y, const int preview_points_number);

    void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K);

    std::vector<TrajectoryPointOri> trajectory_points_;

    // the following parameters are vehicle physics related.
    // control time interval
    double ts_ = 0.0;
    // corner stiffness; front
    double cf_ = 0.0;
    // corner stiffness; rear
    double cr_ = 0.0;
    // distance between front and rear wheel center
    double wheelbase_ = 0.0;
    // mass of the vehicle
    double mass_ = 0.0;
    // distance from front wheel center to COM
    double lf_ = 0.0;
    // distance from rear wheel center to COM
    double lr_ = 0.0;
    // rotational inertia
    double iz_ = 0.0;
    // the ratio between the turn of the steering wheel and the turn of the wheels
    double steer_ratio_ = 0.0;
    // the maximum turn of steer
    double steer_single_direction_max_degree_ = 0.0;

    // number of states without previews, includes
    // lateral error, lateral error rate, heading error, heading error rate
    const int basic_state_size_ = 4;
    // vehicle state matrix
    Eigen::MatrixXd matrix_a_;
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd matrix_ad_;
    // control matrix
    Eigen::MatrixXd matrix_b_;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd_;
    // gain matrix
    Eigen::MatrixXd matrix_k_;
    // control authority weighting matrix
    Eigen::MatrixXd matrix_r_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;
    // updated state weighting matrix
    Eigen::MatrixXd matrix_q_updated_;
    // vehicle state matrix coefficients
    Eigen::MatrixXd matrix_a_coeff_;
    // 4 by 1 matrix; state matrix
    Eigen::MatrixXd matrix_state_;

    // parameters for lqr solver; number of iterations
    int lqr_max_iteration_ = 0;
    // parameters for lqr solver; threshold for computation
    double lqr_eps_ = 0.0;

    // Look-ahead controller
    bool enable_look_ahead_back_control_ = false;

    // for compute the differential valute to estimate acceleration/lon_jerk
    double previous_lateral_acceleration_ = 0.0;

    double previous_heading_rate_ = 0.0;
    double previous_ref_heading_rate_ = 0.0;

    double previous_heading_acceleration_ = 0.0;
    double previous_ref_heading_acceleration_ = 0.0;

    // for logging purpose
    std::ofstream steer_log_file_;

    const std::string name_;

    double query_relative_time_;

    double pre_steer_angle_ = 0.0;

    double pre_steering_position_ = 0.0;

    double minimum_speed_protection_ = 0.1;

    double current_trajectory_timestamp_ = -1.0;

    double init_vehicle_x_ = 0.0;

    double init_vehicle_y_ = 0.0;

    double init_vehicle_heading_ = 0.0;

    double low_speed_bound_ = 0.0;

    double low_speed_window_ = 0.0;

    double driving_orientation_ = 0.0;

    double steer_offset_ = 0.0;

    // added
    double ref_curv_;

   public:
    double ref_curv_front_;
};

}    // namespace control
}    // namespace l5player

#endif