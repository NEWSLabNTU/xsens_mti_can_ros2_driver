#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "xscaninterface.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("xsens_mti_can_ros_node");

    XsCanInterface canInterface(node);
    canInterface.initialize();
    canInterface.registerPublishers();
    canInterface.start();

    rclcpp::spin(node);

    canInterface.stop();
    rclcpp::shutdown();
    return 0;
}
