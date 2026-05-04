// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef ORIENTATIONPUBLISHER_H
#define ORIENTATIONPUBLISHER_H

#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "packetcallback.h"

struct OrientationPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit OrientationPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("filter/quaternion", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsQuaternion)
        {
            return;
        }

        geometry_msgs::msg::QuaternionStamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsQuaternion &q = packet.quaternion;
        msg.quaternion.w = q.q0;
        msg.quaternion.x = q.q1;
        msg.quaternion.y = q.q2;
        msg.quaternion.z = q.q3;

        pub->publish(msg);
    }
};

#endif
