#ifndef PACKETCALLBACK_H
#define PACKETCALLBACK_H

#include <rclcpp/rclcpp.hpp>

#include "xsens_parser.h"

inline constexpr const char *DEFAULT_FRAME_ID = "imu_link";

class PacketCallback
{
public:
    virtual ~PacketCallback() = default;
    virtual void operator()(const XsDataPacket &, rclcpp::Time) = 0;
};

#endif
