cmake_minimum_required(VERSION 3.8)
project(carla_l5player_pid_new_controller)

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

add_executable(${PROJECT_NAME}_node 
                src/vehicle_longitudinal_controller_pid.cpp 
                src/pid_controller.cpp 
                src/reference_line.cpp)

ament_target_dependencies(${PROJECT_NAME}_node std_msgs rclcpp geometry_msgs nav_msgs carla_msgs tf2 tf2_ros)# ipopt z ssl uv uWS

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories( ${PROJECT_NAME}_node PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

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