// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef ANGULARVELOCITYPUBLISHER_H
#define ANGULARVELOCITYPUBLISHER_H

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "packetcallback.h"

struct AngularVelocityPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit AngularVelocityPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/angular_velocity", pub_queue_size);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsRateOfTurn)
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsRateOfTurn &gyro = packet.rate_of_turn;
        msg.vector.x = gyro.x;
        msg.vector.y = gyro.y;
        msg.vector.z = gyro.z;

        pub->publish(msg);
    }
};

#endif
