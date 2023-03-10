cmake_minimum_required(VERSION 3.8)
project(carla_l5player_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mpc_lateral_longitudinal 
                src/mpc_lateral_longitudinal.cpp
                src/pid_controller.cpp
                src/reference_line.cpp
                src/mpc_controller.cpp)
target_include_directories(mpc_lateral_longitudinal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_lateral_longitudinal ipopt)
target_compile_features(mpc_lateral_longitudinal PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(mpc_lateral_longitudinal "std_msgs" "rclcpp" "nav_msgs" "sensor_msgs" "visualization_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs" "carla_msgs")# ipopt z ssl uv uWS

install(TARGETS mpc_lateral_longitudinal
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
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
