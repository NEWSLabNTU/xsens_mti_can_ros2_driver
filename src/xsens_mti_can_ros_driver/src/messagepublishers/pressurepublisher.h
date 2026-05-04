// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef PRESSUREPUBLISHER_H
#define PRESSUREPUBLISHER_H

#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "packetcallback.h"

struct PressurePublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit PressurePublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::FluidPressure>("pressure", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsBarometricPressure)
        {
            return;
        }

        sensor_msgs::msg::FluidPressure msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;
        msg.fluid_pressure = packet.baro_pressure.pressure;
        msg.variance = 0;

        pub->publish(msg);
    }
};

#endif
