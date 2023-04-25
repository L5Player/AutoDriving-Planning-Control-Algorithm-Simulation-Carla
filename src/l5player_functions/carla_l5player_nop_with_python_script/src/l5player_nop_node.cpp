#include "carla_l5player_pid_controller/vehicle_longitudinal_controller_pid.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    // RCLCPP_INFO(LOGGER, "Initialize node");

    rclcpp::init(argc, argv);

    auto n = std::make_shared<VehicleControlPublisher>();

    rclcpp::spin(n);

    rclcpp::shutdown();

    return 0;
}
