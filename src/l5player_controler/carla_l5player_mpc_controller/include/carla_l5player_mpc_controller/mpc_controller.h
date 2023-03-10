#ifndef MPC_CONTROLLER_H_
#define MPC_CONTROLLER_H_
#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common.h"
#include <cppad/cppad.hpp>
#define HAVE_CSTDDEF
#include <cppad/ipopt/solve.hpp>
#undef HAVE_CSTDDEF

using CppAD::AD;
using Eigen::VectorXd;

using Matrix = Eigen::MatrixXd;

class mpc_controller {
   private:
    /*Indexes on the 1-D vector for readability，
    These indexes are used for "vars"*/
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_longitudinal_start;
    size_t v_lateral_start;
    size_t yaw_rate_start;
    size_t cte_start;
    size_t epsi_start;
    size_t front_wheel_angle_start;
    size_t longitudinal_acceleration_start;    //控制时域长度为25的时候，控制量一共有24个。
    size_t front_wheel_angle_increment_start;
    size_t longitudinal_acceleration_increment_start;
    size_t Np;
    size_t Nc;
    double dt;
    double a_lateral;

   public:
    mpc_controller(/* args */);
    virtual ~mpc_controller();

    // Solve the model given an initial state and polynomial coefficients
    // Retrun the first actuators
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs, const double &target_v, const int &cte_weight, const int &epsi_weight, const int &v_weight, const int &steer_actuator_cost_weight, const int &acc_actuator_cost_weight, const int &change_steer_cost_weight,
                              const int &change_accel_cost_weight, const int &mpc_control_horizon_length, const double &mpc_control_step_length, const double &kinamatic_para_Lf, const double &a_lateral, const double &old_steer_value, const double &old_throttle_value, const double &steer_ratio);
};

class FG_eval {
   public:
    /*Define weights for different terms of objective*/
    // double yaw_rate;
    double cte_weight;
    double epsi_weight;
    double v_weight;
    double steer_actuator_cost_weight_fg;
    double acc_actuator_cost_weight_fg;
    double change_steer_cost_weight;
    double change_accel_cost_weight;
    double target_v;    // km/h
    size_t mpc_prediction_horizon_length;
    double mpc_control_step_length;
    double kinamatic_para_Lf;
    /*Reference values that will be include in objective function*/
    double ref_cte = 0;
    double ref_epsi = 0;
    double ref_v;    // m/s
    /*Set the timestep length and duration*/
    size_t Np;
    size_t Nc;
    double dt;
    double a_lateral;
    double steer_ratio;
    AD<double> old_steer_value;
    AD<double> old_throttle_value;

    /*
    This value assumes the model presented in the classroom is used.
    It was obtained by measuring the radius formed by running the vehicle in the
    simulator around in a circle with a constant steering angle and velocity on
    a flat terrain.
    Lf was tuned until the the radius formed by the simulating the model
    presented in the classroom matched the previous radius.
    This is the length from front to CoG that has a similar radius.
    */
    double Lf;

    /*Indexes on the 1-D vector for readability，
    These indexes are used for "vars"*/
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_longitudinal_start;
    size_t v_lateral_start;
    size_t yaw_rate_start;
    size_t cte_start;
    size_t epsi_start;
    size_t front_wheel_angle_start;
    size_t longitudinal_acceleration_start;    //控制时域长度为25的时候，控制量一共有24个。

    size_t front_wheel_angle_increment_start;
    size_t longitudinal_acceleration_increment_start;

    // AD<double> Ccf = 46900; // 前轮轮胎侧偏刚度
    // AD<double> Ccr = 42700; // 后轮轮胎侧偏刚度
    AD<double> Cf = 46000;    // 前轮轮胎纵向刚度
    AD<double> Cr = 46000;    // 后轮轮胎纵向刚度
    // AD<double> Sf = 0.2;    // 前轮滑移率
    // AD<double> Sr = 0.0;    // 后轮滑移率
    AD<double> m = 1000;
    // AD<double> v_lateral;
    // AD<double> v_longitudinal;
    AD<double> l = 1.60;
    AD<double> I = 3575;
    AD<double> lf = 0.95;
    AD<double> lr = 1.6 - lf;

    double x;
    double y;
    double psi;
    double v_longitudinal;
    double v_lateral;
    double yaw_rate;
    double cte;
    double epsi;

   public:
    VectorXd coeffs;    // Fitted polynomial coefficients
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

   public:
    /* coeffs are the coefficients of the fitted polynomial,
       will be used by the cross track error and heading error equations.*/
    FG_eval(const Eigen::VectorXd &state, VectorXd coeffs, const double &target_v, const int &cte_weight, const int &epsi_weight, const int &v_weight, const int &steer_actuator_cost_weight_fg, const int &acc_actuator_cost_weight, const int &change_steer_cost_weight,
            const int &change_accel_cost_weight, const int &mpc_prediction_horizon_length, const int &mpc_control_horizon_length, const double &mpc_control_step_length, const double &kinamatic_para_Lf, const double &a_lateral, const double &old_steer_value, const double &old_throttle_value,
            const double &steer_ratio);
    ~FG_eval();
    // 重载函数调用运算符，创建一个可以传递任意数目参数的运算符函数
    // 函数对象,可以将像函数调用一样使用函数对象
    void operator()(ADvector &fg, ADvector &vars);
};    // namespace l5player

#endif