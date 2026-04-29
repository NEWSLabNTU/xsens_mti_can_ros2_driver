// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef TIMEREFERENCEPUBLISHER_H
#define TIMEREFERENCEPUBLISHER_H

#include <sensor_msgs/msg/time_reference.hpp>

#include "packetcallback.h"

struct TimeReferencePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub;

    explicit TimeReferencePublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::TimeReference>("imu/time_ref", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsSampleTimeFine)
        {
            return;
        }

        constexpr uint32_t SAMPLE_TIME_FINE_HZ = 10000UL;
        constexpr uint32_t ONE_GHZ = 1000000000UL;

        const uint32_t t_fine = packet.sample_time_fine.timestamp;
        const uint32_t sec = t_fine / SAMPLE_TIME_FINE_HZ;
        const uint32_t nsec = (t_fine % SAMPLE_TIME_FINE_HZ) * (ONE_GHZ / SAMPLE_TIME_FINE_HZ);

        sensor_msgs::msg::TimeReference msg;
        msg.header.stamp = timestamp;
        msg.time_ref = rclcpp::Time(static_cast<int64_t>(sec), nsec, RCL_ROS_TIME);

        pub->publish(msg);
    }
};

#endif
