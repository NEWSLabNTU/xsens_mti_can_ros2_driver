// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef GNSSPOSEPUBLISHER_H
#define GNSSPOSEPUBLISHER_H

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "packetcallback.h"

struct GNSSPOSEPublisher : public PacketCallback
{
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit GNSSPOSEPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::SensorDataQoS().keep_last(pub_queue_size));
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!(packet.containsXsLatLon && packet.containsXsAltitudeEllipsoid && packet.containsXsQuaternion))
        {
            return;
        }

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        msg.pose.position.x = packet.lat_lon.latitude;
        msg.pose.position.y = packet.lat_lon.longitude;
        msg.pose.position.z = packet.altitude.alt_ellipsoid;

        const XsQuaternion &q = packet.quaternion;
        msg.pose.orientation.w = q.q0;
        msg.pose.orientation.x = q.q1;
        msg.pose.orientation.y = q.q2;
        msg.pose.orientation.z = q.q3;

        pub->publish(msg);
    }
};

#endif
