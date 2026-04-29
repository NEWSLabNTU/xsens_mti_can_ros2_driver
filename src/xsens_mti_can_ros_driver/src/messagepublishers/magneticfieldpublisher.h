// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef MAGNETICFIELDPUBLISHER_H
#define MAGNETICFIELDPUBLISHER_H

#include <sensor_msgs/msg/magnetic_field.hpp>

#include "packetcallback.h"
#include "publisherhelperfunctions.h"

struct MagneticFieldPublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    double magnetic_field_variance[3] = {0, 0, 0};

    explicit MagneticFieldPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", pub_queue_size);
        variance_from_stddev_param(node, "magnetic_field_stddev", magnetic_field_variance);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsMagneticField)
        {
            return;
        }

        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsMagneticField &mag = packet.magnetic_field;
        msg.magnetic_field.x = mag.x;
        msg.magnetic_field.y = mag.y;
        msg.magnetic_field.z = mag.z;

        msg.magnetic_field_covariance[0] = magnetic_field_variance[0];
        msg.magnetic_field_covariance[4] = magnetic_field_variance[1];
        msg.magnetic_field_covariance[8] = magnetic_field_variance[2];

        pub->publish(msg);
    }
};

#endif
