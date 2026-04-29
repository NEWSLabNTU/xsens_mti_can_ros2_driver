// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef UTCTIMEPUBLISHER_H
#define UTCTIMEPUBLISHER_H

#include <ctime>

#include <sensor_msgs/msg/time_reference.hpp>

#include "packetcallback.h"

struct UTCTimePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub;

    explicit UTCTimePublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::TimeReference>("imu/utctime", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsUtcTime)
        {
            return;
        }

        const XsUtcTime &utcTime = packet.utc_time;

        struct tm timeinfo = {};
        timeinfo.tm_year = utcTime.year - 1900;
        timeinfo.tm_mon = utcTime.month - 1;
        timeinfo.tm_mday = utcTime.day;
        timeinfo.tm_hour = utcTime.hour;
        timeinfo.tm_min = utcTime.minute;
        timeinfo.tm_sec = utcTime.second;

        time_t epochSeconds = timegm(&timeinfo);

        // 1 tenth of a millisecond = 100000 nanoseconds
        rclcpp::Time rosUtcTime(static_cast<int64_t>(epochSeconds),
                                utcTime.tenthms * 100000u,
                                RCL_ROS_TIME);

        sensor_msgs::msg::TimeReference msg;
        msg.header.stamp = timestamp;
        msg.time_ref = rosUtcTime;

        pub->publish(msg);
    }
};

#endif
