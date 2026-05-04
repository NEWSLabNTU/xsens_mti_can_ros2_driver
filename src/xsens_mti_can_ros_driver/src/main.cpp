#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "xscaninterface.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("xsens_mti_can_ros_node");

    XsCanInterface canInterface(node);
    if (!canInterface.initialize())
    {
        RCLCPP_FATAL(node->get_logger(),
                     "Driver initialization failed; exiting non-zero.");
        rclcpp::shutdown();
        return 1;
    }
    canInterface.registerPublishers();
    canInterface.start();

    rclcpp::spin(node);

    canInterface.stop();
    rclcpp::shutdown();
    return 0;
}
