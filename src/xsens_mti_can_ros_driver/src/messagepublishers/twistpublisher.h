// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef TWISTPUBLISHER_H
#define TWISTPUBLISHER_H

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "packetcallback.h"

struct TwistPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit TwistPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("filter/twist", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!(packet.containsXsVelocity && packet.containsXsRateOfTurn))
        {
            return;
        }

        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsVelocity &v = packet.velocity;
        msg.twist.linear.x = v.x;
        msg.twist.linear.y = v.y;
        msg.twist.linear.z = v.z;

        const XsRateOfTurn &gyro = packet.rate_of_turn;
        msg.twist.angular.x = gyro.x;
        msg.twist.angular.y = gyro.y;
        msg.twist.angular.z = gyro.z;

        pub->publish(msg);
    }
};

#endif
