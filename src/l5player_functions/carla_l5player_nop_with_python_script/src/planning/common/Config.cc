#include "Configs.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>

Param_Configs::Param_Configs()
{
  YAML::Node config = YAML::LoadFile("src/l5player_functions/carla_l5player_nop_with_python_script/src/planning/config/configs.yaml");

  FLAGS_trajectory_time_resolution = config["FLAGS_trajectory_time_resolution"].as<double>();  
  FLAGS_weight_target_speed = config["FLAGS_weight_target_speed"].as<double>();
  FLAGS_weight_dist_travelled = config["FLAGS_weight_dist_travelled"].as<double>();
  FLAGS_longitudinal_jerk_lower_bound = config["FLAGS_longitudinal_jerk_lower_bound"].as<double>();
  FLAGS_longitudinal_jerk_upper_bound = config["FLAGS_longitudinal_jerk_upper_bound"].as<double>();
  FLAGS_lon_collision_cost_std = config["FLAGS_lon_collision_cost_std"].as<double>();
  FLAGS_lon_collision_yield_buffer = config["FLAGS_lon_collision_yield_buffer"].as<double>();
  FLAGS_lon_collision_overtake_buffer = config["FLAGS_lon_collision_overtake_buffer"].as<double>();

  FLAGS_weight_lon_objective = config["FLAGS_weight_lon_objective"].as<double>();
  FLAGS_weight_lon_jerk = config["FLAGS_weight_lon_jerk"].as<double>();
  FLAGS_weight_lon_collision = config["FLAGS_weight_lon_collision"].as<double>();
  FLAGS_weight_centripetal_acceleration = config["FLAGS_weight_centripetal_acceleration"].as<double>();
  FLAGS_weight_lat_offset = config["FLAGS_weight_lat_offset"].as<double>();
  FLAGS_weight_lat_comfort = config["FLAGS_weight_lat_comfort"].as<double>();

  FLAGS_speed_lon_decision_horizon = config["FLAGS_speed_lon_decision_horizon"].as<double>();
  FLAGS_trajectory_space_resolution = config["FLAGS_trajectory_space_resolution"].as<double>();
  FLAGS_lat_offset_bound = config["FLAGS_lat_offset_bound"].as<double>();
  FLAGS_weight_opposite_side_offset = config["FLAGS_weight_opposite_side_offset"].as<double>();
  FLAGS_weight_same_side_offset = config["FLAGS_weight_same_side_offset"].as<double>();

  FLAGS_speed_lower_bound = config["FLAGS_speed_lower_bound"].as<double>();
  FLAGS_speed_upper_bound = config["FLAGS_speed_upper_bound"].as<double>();
  FLAGS_longitudinal_acceleration_lower_bound = config["FLAGS_longitudinal_acceleration_lower_bound"].as<double>();
  FLAGS_longitudinal_acceleration_upper_bound = config["FLAGS_longitudinal_acceleration_upper_bound"].as<double>();
  FLAGS_comfort_acceleration_factor = config["FLAGS_comfort_acceleration_factor"].as<double>();
  FLAGS_Max_curvature = config["FLAGS_Max_curvature"].as<double>();
  FLAGS_kappa_bound = config["FLAGS_kappa_bound"].as<double>();
  FLAGS_dkappa_bound = config["FLAGS_dkappa_bound"].as<double>();
  FLAGS_lateral_acceleration_bound = config["FLAGS_lateral_acceleration_bound"].as<double>();
  FLAGS_lateral_jerk_bound = config["FLAGS_lateral_jerk_bound"].as<double>();

  FLAGS_trajectory_time_length = config["FLAGS_trajectory_time_length"].as<double>();
  FLAGS_polynomial_minimal_param = config["FLAGS_polynomial_minimal_param"].as<double>();
  FLAGS_num_velocity_sample = config["FLAGS_num_velocity_sample"].as<double>();
  FLAGS_min_velocity_sample_gap = config["FLAGS_min_velocity_sample_gap"].as<double>();

  FLAGS_enable_osqp_debug = config["FLAGS_enable_osqp_debug"].as<bool>();
  FLAGS_weight_lateral_offset = config["FLAGS_weight_lateral_offset"].as<double>();
  FLAGS_weight_lateral_derivative = config["FLAGS_weight_lateral_derivative"].as<double>();
  FLAGS_weight_lateral_second_order_derivative = config["FLAGS_weight_lateral_second_order_derivative"].as<double>();
  FLAGS_lateral_third_order_derivative_max = config["FLAGS_lateral_third_order_derivative_max"].as<double>();
  FLAGS_weight_lateral_obstacle_distance = config["FLAGS_weight_lateral_obstacle_distance"].as<double>();
  FLAGS_default_delta_s_lateral_optimization = config["FLAGS_default_delta_s_lateral_optimization"].as<double>();
  FLAGS_max_s_lateral_optimization = config["FLAGS_max_s_lateral_optimization"].as<double>();
  FLAGS_bound_buffer = config["FLAGS_bound_buffer"].as<double>();
  FLAGS_numerical_epsilon = config["FLAGS_numerical_epsilon"].as<double>();
  FLAGS_nudge_buffer = config["FLAGS_nudge_buffer"].as<double>();
  kSampleDistance = config["kSampleDistance"].as<double>();
  FLAGS_lon_collision_buffer = config["FLAGS_lon_collision_buffer"].as<double>();
  FLAGS_lat_collision_buffer = config["FLAGS_lat_collision_buffer"].as<double>();

  min_turn_radius = config["min_turn_radius"].as<double>();
  max_acceleration = config["max_acceleration"].as<double>();
  min_deceleration = config["min_deceleration"].as<double>();
  max_deceleration = config["max_deceleration"].as<double>();
  preferred_max_acceleration = config["preferred_max_acceleration"].as<double>();
  preferred_min_deceleration = config["preferred_min_deceleration"].as<double>();
  wheel_rolling_radius = config["wheel_rolling_radius"].as<double>();
  brake_deadzone = config["brake_deadzone"].as<double>();
  throttle_deadzone = config["throttle_deadzone"].as<double>();

  FLAGS_default_lon_buffer = config["FLAGS_default_lon_buffer"].as<double>();
  FLAGS_time_min_density = config["FLAGS_time_min_density"].as<double>();
  FLAGS_num_sample_follow_per_timestamp = config["FLAGS_num_sample_follow_per_timestamp"].as<double>();
  FLAGS_vehicle_width = config["FLAGS_vehicle_width"].as<double>();
  FLAGS_vehicle_length = config["FLAGS_vehicle_length"].as<double>();

  front_edge_to_center = config["front_edge_to_center"].as<double>();
  back_edge_to_center = config["back_edge_to_center"].as<double>();
  left_edge_to_center = config["left_edge_to_center"].as<double>();
  right_edge_to_center = config["right_edge_to_center"].as<double>();

  wheel_base = config["wheel_base"].as<double>();
  max_steer_angle = config["max_steer_angle"].as<double>();
  max_steer_angle_rate = config["max_steer_angle_rate"].as<double>();
  steer_ratio = config["steer_ratio"].as<double>();
  max_abs_speed_when_stopped = config["max_abs_speed_when_stopped"].as<double>();

  FLAGS_enable_sqp_solver = config["FLAGS_enable_sqp_solver"].as<bool>();
  default_active_set_eps_num = config["default_active_set_eps_num"].as<double>();
  default_active_set_eps_den = config["default_active_set_eps_den"].as<double>();
  default_active_set_eps_iter_ref = config["default_active_set_eps_iter_ref"].as<double>();
  default_qp_smoothing_eps_num = config["default_qp_smoothing_eps_num"].as<double>();
  default_qp_smoothing_eps_den = config["default_qp_smoothing_eps_den"].as<double>();
  default_qp_smoothing_eps_iter_ref = config["default_qp_smoothing_eps_iter_ref"].as<double>();
  default_enable_active_set_debug_info = config["default_enable_active_set_debug_info"].as<bool>();
  default_qp_iteration_num = config["default_qp_iteration_num"].as<uint32_t>();

  look_forward_time_sec = config["look_forward_time_sec"].as<double>();
  PathLength = config["PathLength"].as<double>();
  step_length_max = config["step_length_max"].as<double>();
  step_length_min = config["step_length_min"].as<double>();
  uturn_speed_limit = config["uturn_speed_limit"].as<double>();
  default_cruise_speed = config["default_cruise_speed"].as<double>();
  planning_upper_speed_limit = config["planning_upper_speed_limit"].as<double>();
  max_spline_length = config["max_spline_length"].as<double>();
  max_constraint_interval = config["max_constraint_interval"].as<double>();
  change_lane_speed_relax_percentage = config["change_lane_speed_relax_percentage"].as<double>();
  reference_line_weight = config["reference_line_weight"].as<double>();
  history_path_weight = config["history_path_weight"].as<double>();
  derivative_weight = config["derivative_weight"].as<double>();
  first_spline_weight_factor = config["first_spline_weight_factor"].as<double>();
  second_derivative_weight = config["second_derivative_weight"].as<double>();
  third_derivative_weight = config["third_derivative_weight"].as<double>();
  lane_change_mid_l = config["lane_change_mid_l"].as<double>();

  FLAGS_dl_bound = config["FLAGS_dl_bound"].as<double>();
  spline_order = config["spline_order"].as<uint32_t>();
  time_resolution = config["time_resolution"].as<double>();
  num_output = config["num_output"].as<uint32_t>();
  point_constraint_s_position = config["point_constraint_s_position"].as<double>();
  cross_lane_lateral_extension = config["cross_lane_lateral_extension"].as<double>();
  FLAGS_default_reference_line_width = config["FLAGS_default_reference_line_width"].as<double>();

  unit_t = config["unit_t"].as<double>();
  safe_distance = config["safe_distance"].as<double>();
  total_path_length = config["total_path_length"].as<double>();
  total_time = config["total_time"].as<double>();
  dense_dimension_s = config["dense_dimension_s"].as<uint32_t>();
  dense_unit_s = config["dense_unit_s"].as<double>();
  sparse_unit_s = config["sparse_unit_s"].as<double>();
  matrix_dimension_s = config["matrix_dimension_s"].as<uint32_t>();
  matrix_dimension_t = config["matrix_dimension_t"].as<double>();
  obstacle_weight = config["obstacle_weight"].as<double>();
  default_obstacle_cost = config["default_obstacle_cost"].as<double>();
  spatial_potential_penalty = config["spatial_potential_penalty"].as<double>();

  reference_weight = config["reference_weight"].as<double>();
  keep_clear_low_speed_penalty = config["keep_clear_low_speed_penalty"].as<double>();
  default_speed_cost = config["default_speed_cost"].as<double>();
  exceed_speed_penalty = config["exceed_speed_penalty"].as<double>();
  low_speed_penalty = config["low_speed_penalty"].as<double>();
  reference_speed_penalty = config["reference_speed_penalty"].as<double>();
  accel_penalty = config["accel_penalty"].as<double>();
  decel_penalty = config["decel_penalty"].as<double>();
  positive_jerk_coeff = config["positive_jerk_coeff"].as<double>();
  negative_jerk_coeff = config["negative_jerk_coeff"].as<double>();
  FLAGS_use_st_drivable_boundary = config["FLAGS_use_st_drivable_boundary"].as<bool>();
  FLAGS_enable_dp_reference_speed = config["FLAGS_enable_dp_reference_speed"].as<bool>();

  FLAGS_use_navigation_mode = config["FLAGS_use_navigation_mode"].as<bool>();
  IsChangeLanePath = config["IsChangeLanePath"].as<bool>();
  IsClearToChangeLane = config["IsClearToChangeLane"].as<bool>();
  sample_points_num_each_level = config["sample_points_num_each_level"].as<int>();
  navigator_sample_num_each_level = config["navigator_sample_num_each_level"].as<int>();
  lateral_sample_offset = config["lateral_sample_offset"].as<double>();
  lateral_adjust_coeff = config["lateral_adjust_coeff"].as<double>();
  sidepass_distance = config["sidepass_distance"].as<double>();

  eval_time_interval = config["eval_time_interval"].as<double>();
  path_resolution = config["path_resolution"].as<double>();
  obstacle_ignore_distance = config["obstacle_ignore_distance"].as<double>();
  obstacle_collision_distance = config["obstacle_collision_distance"].as<double>();
  obstacle_risk_distance = config["obstacle_risk_distance"].as<double>();
  obstacle_collision_cost = config["obstacle_collision_cost"].as<double>();
  path_l_cost = config["path_l_cost"].as<double>();
  path_dl_cost = config["path_dl_cost"].as<double>();
  path_ddl_cost = config["path_ddl_cost"].as<double>();
  path_l_cost_param_l0 = config["path_l_cost_param_l0"].as<double>();
  path_l_cost_param_b = config["path_l_cost_param_b"].as<double>();
  path_l_cost_param_k = config["path_l_cost_param_k"].as<double>();
  path_out_lane_cost = config["path_out_lane_cost"].as<double>();
  path_end_l_cost = config["path_end_l_cost"].as<double>();
  FLAGS_prediction_total_time = config["FLAGS_prediction_total_time"].as<double>();
  FLAGS_lateral_ignore_buffer = config["FLAGS_lateral_ignore_buffer"].as<double>();
  FLAGS_slowdown_profile_deceleration = config["FLAGS_slowdown_profile_deceleration"].as<double>();
  FLAGS_trajectory_time_min_interval = config["FLAGS_trajectory_time_min_interval"].as<double>();
  FLAGS_trajectory_time_max_interval = config["FLAGS_trajectory_time_max_interval"].as<double>();
  FLAGS_trajectory_time_high_density_period = config["FLAGS_trajectory_time_high_density_period"].as<double>();

  FLAGS_enable_skip_path_tasks = config["FLAGS_enable_skip_path_tasks"].as<bool>();
  FLAGS_use_front_axe_center_in_path_planning = config["FLAGS_use_front_axe_center_in_path_planning"].as<bool>();
  FLAGS_enable_force_pull_over_open_space_parking_test = config["FLAGS_enable_force_pull_over_open_space_parking_test"].as<bool>();
  FLAGS_lateral_derivative_bound_default = config["FLAGS_lateral_derivative_bound_default"].as<double>();
  path_reference_l_weight = config["path_reference_l_weight"].as<double>();

  acc_weight = config["acc_weight"].as<double>();
  jerk_weight = config["jerk_weight"].as<double>();
  kappa_penalty_weight = config["kappa_penalty_weight"].as<double>();
  ref_s_weight = config["ref_s_weight"].as<double>();
  ref_v_weight = config["ref_v_weight"].as<double>();
  fallback_total_time = config["fallback_total_time"].as<double>();
  fallback_time_unit = config["fallback_time_unit"].as<double>();
  qp_delta_s = config["qp_delta_s"].as<double>();
  min_look_ahead_time = config["min_look_ahead_time"].as<double>();
  min_look_ahead_distance = config["min_look_ahead_distance"].as<double>();
  lateral_buffer = config["lateral_buffer"].as<double>();
  path_output_resolution = config["path_output_resolution"].as<double>();

  FLAGS_use_osqp_optimizer_for_qp_st = config["FLAGS_use_osqp_optimizer_for_qp_st"].as<bool>();
  FLAGS_enable_follow_accel_constraint = config["FLAGS_enable_follow_accel_constraint"].as<bool>();
  number_of_discrete_graph_t = config["number_of_discrete_graph_t"].as<double>();
  accel_kernel_weight = config["accel_kernel_weight"].as<double>();
  jerk_kernel_weight = config["jerk_kernel_weight"].as<double>();
  stop_weight = config["stop_weight"].as<double>();
  cruise_weight = config["cruise_weight"].as<double>();
  follow_weight = config["follow_weight"].as<double>();
  regularization_weight = config["regularization_weight"].as<double>();
  follow_drag_distance = config["follow_drag_distance"].as<double>();
  dp_st_reference_weight = config["dp_st_reference_weight"].as<double>();
  init_jerk_kernel_weight = config["init_jerk_kernel_weight"].as<double>();
  yield_weight = config["yield_weight"].as<double>();
  yield_drag_distance = config["yield_drag_distance"].as<double>();
  number_of_evaluated_graph_t = config["number_of_evaluated_graph_t"].as<double>();

  boundary_buffer = config["boundary_buffer"].as<double>();
  high_speed_centric_acceleration_limit = config["high_speed_centric_acceleration_limit"].as<double>();
  low_speed_centric_acceleration_limit = config["low_speed_centric_acceleration_limit"].as<double>();
  high_speed_threshold = config["high_speed_threshold"].as<double>();
  low_speed_threshold = config["low_speed_threshold"].as<double>();
  minimal_kappa = config["minimal_kappa"].as<double>();
  point_extension = config["point_extension"].as<double>();
  lowest_speed = config["lowest_speed"].as<double>();
  num_points_to_avg_kappa = config["num_points_to_avg_kappa"].as<double>();
  static_obs_nudge_speed_ratio = config["static_obs_nudge_speed_ratio"].as<double>();
  dynamic_obs_nudge_speed_ratio = config["dynamic_obs_nudge_speed_ratio"].as<double>();
  centri_jerk_speed_coeff = config["centri_jerk_speed_coeff"].as<double>();
  FLAGS_enable_nudge_slowdown = config["FLAGS_enable_nudge_slowdown"].as<bool>();

  xy_grid_resolution = config["xy_grid_resolution"].as<double>();
  phi_grid_resolution = config["phi_grid_resolution"].as<double>();
  grid_a_star_xy_resolution = config["grid_a_star_xy_resolution"].as<double>();
  node_radius = config["node_radius"].as<double>();
  step_size = config["step_size"].as<double>();
  FLAGS_enable_parallel_hybrid_a = config["FLAGS_enable_parallel_hybrid_a"].as<bool>(); 
  
  next_node_num = config["next_node_num"].as<double>();
  delta_t = config["delta_t"].as<double>();
  traj_forward_penalty = config["traj_forward_penalty"].as<double>();
  traj_back_penalty = config["traj_back_penalty"].as<double>();
  traj_gear_switch_penalty = config["traj_gear_switch_penalty"].as<double>();
  traj_steer_penalty = config["traj_steer_penalty"].as<double>(); 
  traj_steer_change_penalty = config["traj_steer_change_penalty"].as<double>();
  FLAGS_use_s_curve_speed_smooth = config["FLAGS_use_s_curve_speed_smooth"].as<bool>(); 
}

Param_Configs::~Param_Configs()
{
}
