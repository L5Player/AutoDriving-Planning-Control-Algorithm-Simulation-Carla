#include "l5player_nop/nop_function_node.h"
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

    auto nop_node = std::make_shared<NopFunctionNode>();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(nop_node);

        nop_node->NopRunOnce();

        loop_rate.sleep();
    }

    // rclcpp::spin(n);

    rclcpp::shutdown();

    return 0;
}
