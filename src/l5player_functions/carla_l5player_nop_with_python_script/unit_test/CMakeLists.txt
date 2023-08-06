cmake_minimum_required(VERSION 3.8)

file(GLOB_RECURSE TEST_SRCS ./*_test.cc)

ament_add_gtest(${PROJECT_NAME}_test ${TEST_SRCS})

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
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
find_package(GTest REQUIRED)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_test
)

ament_target_dependencies(${PROJECT_NAME}_test std_msgs derived_object_msgs sensor_msgs
rclcpp geometry_msgs nav_msgs carla_msgs tf2 tf2_ros yaml-cpp visualization_msgs l5player_nop_msgs GTest)# ipopt z ssl uv uWS

target_include_directories( ${PROJECT_NAME}_test PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
                            $<INSTALL_INTERFACE:include>
                            $<INSTALL_INTERFACE:src>
                            ../src/planning/path
                            ../src/planning/common
                            ../src/planning/polynomial
                            ../src/planning/reference_line
                            ../src/planning/lattice
                            ../src/planning/obstacle
                            ../src/planning/qp
                            ../src/planning/spline
                            ../src/planning/
                          )


# Install
install(
    DIRECTORY data/
    DESTINATION share/${PROJECT_NAME}/
)

install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
)