// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef TRANSFORMPUBLISHER_H
#define TRANSFORMPUBLISHER_H

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "packetcallback.h"

struct TransformPublisher : public PacketCallback
{
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit TransformPublisher(rclcpp::Node::SharedPtr node)
        : tf_broadcaster(node)
    {
        node->get_parameter("frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsXsQuaternion)
        {
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = timestamp;
        tf.header.frame_id = "world";
        tf.child_frame_id = frame_id;
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;

        const XsQuaternion &q = packet.quaternion;
        tf.transform.rotation.x = q.q1;
        tf.transform.rotation.y = q.q2;
        tf.transform.rotation.z = q.q3;
        tf.transform.rotation.w = q.q0;

        tf_broadcaster.sendTransform(tf);
    }
};

#endif
