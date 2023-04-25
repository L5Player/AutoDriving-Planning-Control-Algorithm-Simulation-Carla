/**
 * @file reference_line.h
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-12-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <utility>
#include <vector>

#include "cos_theta_ipopt_interface.h"
#include "fem_pos_deviation_ipopt_interface.h"
#include "fem_pos_deviation_osqp_interface.h"
#include "fem_pos_deviation_sqp_osqp_interface.h"
#include "lattice.h"
// #include "path_matcher.h"
// #include "reference_point.h"
// #include "trajectoryPoint.h"
#include "l5player_nop_msgs/msg/gps.hpp"
#include "l5player_nop_msgs/msg/trajectory.hpp"
#include "planning_base.h"

using namespace l5player::planning;

namespace l5player {
namespace reference_line {

// std::vector<RefPoint> targetPath_;
struct Point3d_s {
    double x;
    double y;
    double z;
};

class referenceLine {
    friend class VehicleControlPublisher;

   public:
    referenceLine();
    ~referenceLine() = default;

    /**
     * @brief 全局路径的回调函数
     *
     * @param routing
     */
    void routingCallback(const geometry_msgs::msg::PoseArray& routing);
    /**
     * @brief gps的回调函数
     *
     * @param pGps
     */
    void gpsCallback(const l5player_nop_msgs::msg::Gps& pGps);
    /**
     * @brief planning的入口函数
     *
     */
    void run();
    /**
     * @brief 为planning模块单独创建一个线程
     *
     */
    void referenceLine_thread();

    /**
     * @brief 对参考线进行操作
     *
     * @param hdmap_way_points
     */
    void referenceLine_split(Eigen::MatrixXd& hdmap_way_points);
    /**
     * @brief 将参考线以话题 发布
     *
     * @param path_point_after_interpolation_
     */
    void Smooth(Eigen::MatrixXd& path_point_after_interpolation_);
    void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);
    void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

    /**
     * @brief 将输入的全局路径插值后输出到output
     *
     * @param input
     * @param output
     * @param interval_dis
     * @param distance
     */
    void average_interpolation(Eigen::MatrixXd& input, Eigen::MatrixXd& output, double interval_dis, double distance);
    /**
     * @brief 计算平滑后的参考线的kappa、theta，赋值到成员变量reference_points中
     *
     * @param path_point
     */
    void referencePointsCalc(const nav_msgs::msg::Path& path_point);

    /**
     * @brief
     * 计算规划起点:第一次运行，规划起点就是定位点，轨迹的plan_start_time=current_time；之后每次运行先判断控制是否跟上
     * 控制跟上：规划起点是轨迹绝对时间上往后0.1s的轨迹点，plan_start_time是在上一帧轨迹中找到距current_time+0.1最接近的轨迹点的索引
     * 控制没跟上：规划起点根据车辆动力学合理外推，plan_start_time=current_time+0.1;
     *
     */
    std::vector<TrajectoryPoint> plan_start_point(double& tt);

    bool is_update_dynamic(nav_msgs::msg::Path& trj_point_array, int size);

    const nav_msgs::msg::Path* const get_referenceline() { return &referenceline_; }

    const std::vector<ReferencePoint>* const GetReferencePoints() { return &reference_points; }

   private:
    bool CosThetaSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                        std::vector<double>* opt_x, std::vector<double>* opt_y);
    /**
     * @brief femSmooth方法
     *
     * @param raw_point2d
     * @param bounds
     * @param opt_x
     * @param opt_y
     * @return true
     * @return false
     */
    bool FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                      std::vector<double>* opt_x, std::vector<double>* opt_y);

    bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                    std::vector<double>* opt_x, std::vector<double>* opt_y);

    bool NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                      std::vector<double>* opt_x, std::vector<double>* opt_y);

    bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                     std::vector<double>* opt_x, std::vector<double>* opt_y);

   public:
    // handle
    // ros::NodeHandle n_;
    // std::shared_ptr<rclcpp::Node> node_refline_;
    // publisher
    // ros::Publisher trajectory_pub_, rviz_pub_, rviz_obstacle_pub_;
    // // subscriber
    // ros::Subscriber routing_sub_, gps_sub_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    std::shared_ptr<PlanningBase> planning_base_;

    // param
    Eigen::MatrixXd routing_waypoints_;                 // 中心点坐标
    Eigen::MatrixXd path_point_after_interpolation_;    // 插值后的轨迹点
    double zero_x_ = 0.0;
    double zero_y_ = 0.0;
    nav_msgs::msg::Path referenceline_;

    std::pair<std::vector<double>, std::vector<double>> reference_path;    // 参考路径点位置（x,y）
    std::vector<double> accumulated_s;                                     // 纵向距离
    std::vector<ReferencePoint> reference_points;                          // 参考路径点参数
    nav_msgs::msg::Path traj_points_;                                      // 局部规划的轨迹,nav_msgs类型
    l5player_nop_msgs::msg::Gps gps_;

    // flag
    std::vector<double> gps_flag_;
    bool is_first_run = true;
    bool FLAGS_lateral_optimization;    // 选择二次规划
    bool which_smoothers;               // 选择参考线平滑方式
    int which_planners;
    /*
    如果考虑参考线的曲率约束，其优化问题是非线性的，
    可以使用ipopt非线性求解器求解（内点法），也可以使用osqp二次规划求解器来用SQP方法求解；
    如果不考虑曲率约束，则直接用osqp求解二次规划问题。
    */
    bool apply_curvature_constraint;               // 是否使用曲率约束
    bool use_sqp = false;                          // 是否使用SQP方法求解
    std::vector<double> routing_waypoint_flag_;    // 添加flag，确保参考线平滑只计算一次
    std::vector<double> path_point_flag_;          // 添加flag，确保参考线的kappa、theta等只计算一次

    double d0;                    // 初始的横向偏移值 [m]
    double dd0;                   // 初始的横向速度 [m/s]
    double ddd0;                  // 初始的横向加速度 [m/s^2]
    double s0;                    // 初始的纵向值[m]
    double ds0;                   // 初始的纵向速度[m/s]
    double dds0;                  // 初始的纵向加速度[m/ss]
    double init_relative_time;    // 规划起始点的时间
    double plan_start_time;       // 规划开始的时间
    double x_init;
    double y_init;
    double z_init;
    double v_init;
    double a_init;
    double theta_init;
    // double theta_end;
    double kappa_init;
    double dkappa_init;

    double lon_decision_horizon = 0;    // 前视距离

    // class
    DiscretizedTrajectory best_path_;         // 最佳路径
    DiscretizedTrajectory pre_trajectory_;    // 上一帧的轨迹
};

}    // namespace reference_line
}    // namespace l5player

#endif    // !__reference_line__
