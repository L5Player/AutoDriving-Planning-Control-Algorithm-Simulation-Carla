cmake_minimum_required(VERSION 3.8)
project(carla_l5player_nop_with_python_script)

set(vehicle_control_MAJOR_VERSION 0)
set(vehicle_control_MINOR_VERSION 0)
set(vehicle_control_PATCH_VERSION 0)
set(vehicle_control_VERSION ${vehicle_control_MAJOR_VERSION}.${vehicle_control_MINOR_VERSION}.${vehicle_control_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(l5player_nop_msgs REQUIRED)

find_package(Eigen3 QUIET)
find_package(OsqpEigen QUIET)

add_executable(${PROJECT_NAME}_node 
                src/l5player_nop_node.cpp
                src/l5player_nop_main.cpp 
                src/pid_controller.cpp 
                src/reference_line.cpp
                src/planning/common/box2d.cc
                src/planning/common/Config.cc
                src/planning/common/line_segment2d.cc
                src/planning/common/math_utils.cc
                src/planning/common/PlanningTarget.cc
                src/planning/common/polygon2d.cc
                src/planning/common/vec2d.cc
                src/planning/path/FrenetPath.cc
                src/planning/path/path_matcher.cc
                src/planning/path/path_points.cc
                src/planning/path/trajectoryPoint.cc
                src/planning/obstacle/boundarys.cc
                src/planning/obstacle/Ob_prediction_trajectory.cc
                src/planning/obstacle/Obstacle_avoid.cc
                src/planning/obstacle/Obstacle.cc
                src/planning/reference_line/cartesian_frenet_conversion.cc
                src/planning/reference_line/reference_line.cc
                src/planning/reference_line/cos_theta_ipopt_interface.cc
                src/planning/reference_line/fem_pos_deviation_ipopt_interface.cc
                src/planning/reference_line/fem_pos_deviation_osqp_interface.cc
                src/planning/reference_line/fem_pos_deviation_sqp_osqp_interface.cc
                src/planning/reference_line/reference_point.cc
                src/planning/spline/CubicSpline1D.cc
                src/planning/spline/CubicSpline2D.cc
                src/planning/polynomial/quartic_polynomial_curve1d.cc
                src/planning/polynomial/quintic_polynomial_curve1d.cc
                src/planning/qp/constant_jerk_trajectory1d.cc
                src/planning/qp/lateral_osqp_optimizer.cc
                src/planning/qp/lateral_qp_optimizer.cc
                src/planning/qp/piecewise_jerk_trajectory1d.cc

                src/planning/lattice/collision_checker.cc
                src/planning/lattice/constraint_checker.cc
                src/planning/lattice/end_condition_sampler.cc
                src/planning/lattice/feasible_region.cc
                src/planning/lattice/lattice_trajectory1d.cc
                src/planning/lattice/lattice.cc
                src/planning/lattice/path_time_graph.cc
                src/planning/lattice/piecewise_acceleration_trajectory1d.cc
                src/planning/lattice/piecewise_braking_trajectory_generator.cc
                src/planning/lattice/prediction_querier.cc
                src/planning/lattice/trajectory_combiner.cc
                src/planning/lattice/trajectory_evaluator.cc
                src/planning/lattice/trajectory1d_generator.cc


                )

ament_target_dependencies(${PROJECT_NAME}_node std_msgs derived_object_msgs 
rclcpp geometry_msgs nav_msgs carla_msgs tf2 tf2_ros yaml-cpp visualization_msgs l5player_nop_msgs )# ipopt z ssl uv uWS

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories( ${PROJECT_NAME}_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                            $<INSTALL_INTERFACE:include>
                            $<INSTALL_INTERFACE:src>
                            src/planning/path
                            src/planning/common
                            src/planning/polynomial
                            src/planning/reference_line
                            src/planning/lattice
                            src/planning/obstacle
                            src/planning/qp
                            src/planning/spline
                            src/planning/
                          )

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node   ${YAMLCPP_LIBRARIES} adolc osqp ipopt)

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
config
rviz
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()