#pragma once
#include <rclcpp/rclcpp.hpp>

class Param_Configs {
   public:
    // general project config
    bool FLAGS_if_use_local_planner;    // 是否使用规划器

    // cost
    double FLAGS_trajectory_time_resolution;       // Trajectory time resolution in planning
    double FLAGS_weight_target_speed;              // Weight of target speed cost
    double FLAGS_weight_dist_travelled;            // Weight of travelled distance cost
    double FLAGS_longitudinal_jerk_lower_bound;    // The lower bound of longitudinal jerk.
    double FLAGS_longitudinal_jerk_upper_bound;    // The upper bound of longitudinal jerk
    double FLAGS_lon_collision_cost_std;           // The standard deviation of longitudinal collision cost function
    double FLAGS_lon_collision_yield_buffer;       // Longitudinal collision buffer for yield
    double FLAGS_lon_collision_overtake_buffer;    // Longitudinal collision buffer for overtake

    double FLAGS_weight_lon_objective;
    double FLAGS_weight_lon_jerk;
    double FLAGS_weight_lon_collision;
    double FLAGS_weight_centripetal_acceleration;
    double FLAGS_weight_lat_offset;
    double FLAGS_weight_lat_comfort;

    double FLAGS_speed_lon_decision_horizon;               // Longitudinal horizon for speed decision making (meter)
    double FLAGS_trajectory_space_resolution;              // Trajectory space resolution in planning
    double FLAGS_lat_offset_bound;                         // The bound of lateral offset
    double FLAGS_weight_opposite_side_offset;              // Weight of opposite side lateral offset cost
    double FLAGS_weight_same_side_offset;                  // Weight of same side lateral offset cost

    double FLAGS_speed_lower_bound;                        // The lowest speed allowed
    double FLAGS_speed_upper_bound;                        // The highest speed allowed
    double FLAGS_longitudinal_acceleration_lower_bound;    // The lowest longitudinal acceleration allowed
    double FLAGS_longitudinal_acceleration_upper_bound;    // The highest longitudinal acceleration allowed
    double FLAGS_comfort_acceleration_factor;              // Factor for comfort acceleration
    double FLAGS_Max_curvature;                            // 轨迹的最大曲率
    double FLAGS_kappa_bound;
    double FLAGS_dkappa_bound;
    double FLAGS_lateral_acceleration_bound;
    double FLAGS_lateral_jerk_bound;

    // 纵向规划
    double FLAGS_trajectory_time_length;      // 最大预测时间 [s]，8s
    double FLAGS_polynomial_minimal_param;    // Minimal time parameter in polynomials
    double FLAGS_num_velocity_sample;         // The number of velocity samples in end condition sampler
    double FLAGS_min_velocity_sample_gap;     // Minimal sampling gap for velocity

    // 二次规划下调整的参数
    bool FLAGS_enable_osqp_debug;
    double FLAGS_weight_lateral_offset;
    double FLAGS_weight_lateral_derivative;
    double FLAGS_weight_lateral_second_order_derivative;
    double FLAGS_lateral_third_order_derivative_max;
    double FLAGS_weight_lateral_obstacle_distance;
    double FLAGS_default_delta_s_lateral_optimization;
    double FLAGS_max_s_lateral_optimization;
    double FLAGS_bound_buffer;
    double FLAGS_numerical_epsilon;    // 横向优化的边界缓冲
    double FLAGS_nudge_buffer;         // 用于横向优化的微移缓冲
    double kSampleDistance;
    double FLAGS_lon_collision_buffer;
    double FLAGS_lat_collision_buffer;

    // VehicleParam
    double min_turn_radius;
    double max_acceleration;
    double min_deceleration;
    double max_deceleration;
    double preferred_max_acceleration;
    double preferred_min_deceleration;
    double wheel_rolling_radius;
    double brake_deadzone;
    double throttle_deadzone;

    double FLAGS_default_lon_buffer;                 //"Default longitudinal buffer to sample path-time points."
    double FLAGS_time_min_density;                   //"Minimal time density to search sample points."
    double FLAGS_num_sample_follow_per_timestamp;    // "The number of sample points for each timestamp to follow"

    double FLAGS_vehicle_width;                      // 车的宽度,我们的车小
    double FLAGS_vehicle_length;                     // 车的长度,我们的车小
                                                     // 改车的长度宽度之后，这个一定要改
    double front_edge_to_center;
    double back_edge_to_center;
    double left_edge_to_center;
    double right_edge_to_center;

    double wheel_base;              // 原来是2.8448,wheel_base
    double max_steer_angle;         // 8.20304748437
    double max_steer_angle_rate;    // 8.55211
    double steer_ratio;             // 16
    double max_abs_speed_when_stopped;

    // -------------------------------SQP solver---------------------------//
    bool FLAGS_enable_sqp_solver;                 // "True to enable SQP solver."
    double default_active_set_eps_num;            //"qpOases wrapper error control numerator"
    double default_active_set_eps_den;            //"qpOases wrapper error control denominator"
    double default_active_set_eps_iter_ref;       //"qpOases wrapper early termination tolerance for iterative
                                                  // refinement"// NOLINT
    double default_qp_smoothing_eps_num;          //"qpOases wrapper error control numerator"
    double default_qp_smoothing_eps_den;          //"qpOases wrapper error control denominator"
    double default_qp_smoothing_eps_iter_ref;     //"qpOases wrapper early termination tolerance for iterative
                                                  // refinement" // NOLINT
    bool default_enable_active_set_debug_info;    //"Enable print information"
    uint32_t default_qp_iteration_num;            //"Default qp oases iteration time"
                                          // "look forward time times adc speed to calculate this distance when creating
                                          // reference line from routing"
    double look_forward_time_sec;
    double PathLength;    // 预计的轨迹长度，即路径DP采样的前探距离，
                          // 规划的参考线要比PathLength这个长,长度要和目标速度搭配调,速度越大合成的轨迹长度越长
    double step_length_max;                       // [default = 15.0];
    double step_length_min;                       //[default = 8.0];
    double uturn_speed_limit;                     // U形大弯
    double default_cruise_speed;                  // EM 巡航速度
    double planning_upper_speed_limit;            // Maximum speed (m/s) in planning.
    double max_spline_length;                     // 每一段sqp优化曲线长度
    double max_constraint_interval;
    double change_lane_speed_relax_percentage;    //"The percentage of change lane speed relaxation."
    double reference_line_weight;                 // 路径qp优化参考线权重
    double history_path_weight;                   // 路径qp优化历史轨迹(即输入的Dp路径)权重
    double derivative_weight;
    double first_spline_weight_factor;
    double second_derivative_weight;
    double third_derivative_weight;
    double lane_change_mid_l;

    double FLAGS_dl_bound;    //"The bound for derivative l in s-l coordinate system."
    uint32_t spline_order;
    double time_resolution;
    uint32_t num_output;    //
    double point_constraint_s_position;
    double cross_lane_lateral_extension;
    double FLAGS_default_reference_line_width;

    // DpStSpeedOptimizerConfig 和 STDrivableBoundary
    double unit_t;
    double safe_distance;
    double total_path_length;      // 可调整,什么意义，没想出来
    double total_time;
    uint32_t dense_dimension_s;    // 可调整影响速度DP中s/t图采样效果
    double dense_unit_s;           // 可调整影响速度DP中s/t图采样效果
    double sparse_unit_s;          // 可调整影响速度DP中s/t图采样效果
    uint32_t matrix_dimension_s;
    double matrix_dimension_t;
    double obstacle_weight;
    double default_obstacle_cost;
    double spatial_potential_penalty;
    double reference_weight;
    double keep_clear_low_speed_penalty;
    double default_speed_cost;
    double exceed_speed_penalty;
    double low_speed_penalty;
    double reference_speed_penalty;
    double accel_penalty;
    double decel_penalty;
    double positive_jerk_coeff;
    double negative_jerk_coeff;

    bool FLAGS_use_st_drivable_boundary;
    bool FLAGS_enable_dp_reference_speed;

    // -----------------------EM_planner需要调节的参数-------------------------//
    bool FLAGS_use_navigation_mode;
    bool IsChangeLanePath;               // 如果可以变道
    bool IsClearToChangeLane;            // 可以换车道了,也就是换道安全
    int sample_points_num_each_level;    //[default = 9];
    int navigator_sample_num_each_level;
    double lateral_sample_offset;        //[default = 0.5];
    double lateral_adjust_coeff;         //[default = 0.5];
    double sidepass_distance;

    // Trajectory Cost Config
    double eval_time_interval;             // [default = 0.1];
    double path_resolution;                // [default = 0.1];
    double obstacle_ignore_distance;       // [default = 20.0];
    double obstacle_collision_distance;    // [default = 0.2];
    double obstacle_risk_distance;         // [default = 2.0];
    double obstacle_collision_cost;        // [default = 1e3];
    double path_l_cost;
    double path_dl_cost;
    double path_ddl_cost;
    double path_l_cost_param_l0;
    double path_l_cost_param_b;
    double path_l_cost_param_k;
    double path_out_lane_cost;
    double path_end_l_cost;
    double FLAGS_prediction_total_time;    // 预测
    double FLAGS_lateral_ignore_buffer;    // EM planner重要参数，调节边界缓冲值
    double FLAGS_slowdown_profile_deceleration;
    double FLAGS_trajectory_time_min_interval;
    double FLAGS_trajectory_time_max_interval;        //"(seconds) Trajectory time interval when publish. The is the max
                                                      // value."
    double
        FLAGS_trajectory_time_high_density_period;    //(seconds) Keep high density in the next this amount of seconds.

    // QP过程的参数
    // skip all path tasks and use trimmed previous path
    bool FLAGS_enable_skip_path_tasks;
    // If using front axe center in path planning, the path can be more agile
    bool FLAGS_use_front_axe_center_in_path_planning;
    // enable force_pull_over_open_space_parking_test
    bool FLAGS_enable_force_pull_over_open_space_parking_test;
    // the default value for lateral derivative bound.
    double FLAGS_lateral_derivative_bound_default;
    double path_reference_l_weight;    //[default = 0.0];

    // piecewise_jerk_speed_optimizer_config
    double acc_weight;
    double jerk_weight;
    double kappa_penalty_weight;
    double ref_s_weight;
    double ref_v_weight;
    double fallback_total_time;
    double fallback_time_unit;
    double qp_delta_s;
    double min_look_ahead_time;
    double min_look_ahead_distance;
    double lateral_buffer;
    double path_output_resolution;

    // QpStSpeedConfig
    bool FLAGS_use_osqp_optimizer_for_qp_st;
    bool FLAGS_enable_follow_accel_constraint;    // "Enable follow acceleration  raint."
    double number_of_discrete_graph_t;
    double accel_kernel_weight;
    double jerk_kernel_weight;
    double stop_weight;
    double cruise_weight;
    double follow_weight;
    double regularization_weight;
    double follow_drag_distance;
    double dp_st_reference_weight;    // 速度规划qp，设置原始速度的影响
    double init_jerk_kernel_weight;
    double yield_weight;
    double yield_drag_distance;
    double number_of_evaluated_graph_t;
    // StBoundaryConfig
    double boundary_buffer;
    double high_speed_centric_acceleration_limit;
    double low_speed_centric_acceleration_limit;
    double high_speed_threshold;
    double low_speed_threshold;
    double minimal_kappa;
    double point_extension;
    double lowest_speed;    // 速度限制里面的最低速度
    double num_points_to_avg_kappa;
    double static_obs_nudge_speed_ratio;
    double dynamic_obs_nudge_speed_ratio;
    double centri_jerk_speed_coeff;
    bool FLAGS_enable_nudge_slowdown;

    // PlannerOpenSpaceConfig
    double xy_grid_resolution;
    double phi_grid_resolution;
    double grid_a_star_xy_resolution;
    double node_radius;
    double step_size;
    bool FLAGS_enable_parallel_hybrid_a;

    double next_node_num;
    double delta_t;
    double traj_forward_penalty;
    double traj_back_penalty;
    double traj_gear_switch_penalty;
    double traj_steer_penalty;
    double traj_steer_change_penalty;
    bool FLAGS_use_s_curve_speed_smooth;

   public:
    Param_Configs();
    ~Param_Configs();
};
