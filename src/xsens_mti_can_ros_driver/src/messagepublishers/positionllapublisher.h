// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef POSITIONLLAPUBLISHER_H
#define POSITIONLLAPUBLISHER_H

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "packetcallback.h"

struct PositionLLAPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit PositionLLAPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("filter/positionlla", pub_queue_size);
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!(packet.containsXsLatLon && packet.containsXsAltitudeEllipsoid))
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        msg.vector.x = packet.lat_lon.latitude;
        msg.vector.y = packet.lat_lon.longitude;
        msg.vector.z = packet.altitude.alt_ellipsoid;

        pub->publish(msg);
    }
};

#endif
