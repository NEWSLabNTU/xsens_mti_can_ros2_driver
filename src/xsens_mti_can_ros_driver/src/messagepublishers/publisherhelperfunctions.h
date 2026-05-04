#ifndef PUBLISHER_HELPER_FUNCTION_H
#define PUBLISHER_HELPER_FUNCTION_H

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Returns true when stddev was provided and applied; false when the param is
// empty or wrong-size. Caller uses the return to decide whether to publish a
// real covariance or the "unknown" sentinel (covariance[0] = -1) per
// sensor_msgs/Imu conventions.
inline bool variance_from_stddev_param(const rclcpp::Node::SharedPtr &node,
                                       const std::string &param,
                                       double *variance_out)
{
    std::vector<double> stddev;
    if (node->get_parameter(param, stddev) && stddev.size() == 3)
    {
        auto squared = [](double x) { return x * x; };
        std::transform(stddev.begin(), stddev.end(), variance_out, squared);
        return true;
    }

    if (!stddev.empty() && stddev.size() != 3)
    {
        RCLCPP_WARN(node->get_logger(),
                    "Wrong size of param: %s, must be of size 3",
                    param.c_str());
    }
    std::memset(variance_out, 0, 3 * sizeof(double));
    return false;
}

#endif
