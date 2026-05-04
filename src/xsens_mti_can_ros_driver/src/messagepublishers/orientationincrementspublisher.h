// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef ORIENTATIONINCREMENTSPUBLISHER_H
#define ORIENTATIONINCREMENTSPUBLISHER_H

#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "packetcallback.h"

struct OrientationIncrementsPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit OrientationIncrementsPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("imu/dq", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsDeltaQ)
        {
            return;
        }

        geometry_msgs::msg::QuaternionStamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsQuaternion &dq = packet.delta_q;
        msg.quaternion.w = dq.q0;
        msg.quaternion.x = dq.q1;
        msg.quaternion.y = dq.q2;
        msg.quaternion.z = dq.q3;

        pub->publish(msg);
    }
};

#endif
