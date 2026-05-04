// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef TEMPERATUREPUBLISHER_H
#define TEMPERATUREPUBLISHER_H

#include <sensor_msgs/msg/temperature.hpp>

#include "packetcallback.h"

struct TemperaturePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit TemperaturePublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::Temperature>("temperature", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsTemperature)
        {
            return;
        }

        sensor_msgs::msg::Temperature msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;
        msg.temperature = packet.temperature.temperature;
        msg.variance = 0;

        pub->publish(msg);
    }
};

#endif
