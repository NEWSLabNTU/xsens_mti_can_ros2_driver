// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef FREEACCELERATIONPUBLISHER_H
#define FREEACCELERATIONPUBLISHER_H

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "packetcallback.h"

struct FreeAccelerationPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit FreeAccelerationPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("filter/free_acceleration", pub_queue_size);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsFreeAcceleration)
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsFreeAcceleration &accel = packet.free_acceleration;
        msg.vector.x = accel.x;
        msg.vector.y = accel.y;
        msg.vector.z = accel.z;

        pub->publish(msg);
    }
};

#endif
