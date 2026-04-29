// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef IMUPUBLISHER_H
#define IMUPUBLISHER_H

#include <sensor_msgs/msg/imu.hpp>

#include "packetcallback.h"
#include "publisherhelperfunctions.h"

struct ImuPublisher : public PacketCallback
{
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    double orientation_variance[3];
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];

    explicit ImuPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", pub_queue_size);
        node->get_parameter("frame_id", frame_id);

        // REP 145: Conventions for IMU Sensor Drivers
        variance_from_stddev_param(node, "orientation_stddev", orientation_variance);
        variance_from_stddev_param(node, "angular_velocity_stddev", angular_velocity_variance);
        variance_from_stddev_param(node, "linear_acceleration_stddev", linear_acceleration_variance);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        bool quaternion_available = packet.containsXsQuaternion;
        bool accel_available = packet.containsXsAcceleration;
        bool gyro_available = packet.containsXsRateOfTurn;

        if (!(quaternion_available || accel_available || gyro_available))
        {
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        if (quaternion_available)
        {
            const XsQuaternion &q = packet.quaternion;
            msg.orientation.w = q.q0;
            msg.orientation.x = q.q1;
            msg.orientation.y = q.q2;
            msg.orientation.z = q.q3;
            msg.orientation_covariance[0] = orientation_variance[0];
            msg.orientation_covariance[4] = orientation_variance[1];
            msg.orientation_covariance[8] = orientation_variance[2];
        }
        else
        {
            msg.orientation_covariance[0] = -1;
        }

        if (gyro_available)
        {
            const XsRateOfTurn &g = packet.rate_of_turn;
            msg.angular_velocity.x = g.x;
            msg.angular_velocity.y = g.y;
            msg.angular_velocity.z = g.z;
            msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
            msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
            msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
        }
        else
        {
            msg.angular_velocity_covariance[0] = -1;
        }

        if (accel_available)
        {
            const XsAcceleration &a = packet.acceleration;
            msg.linear_acceleration.x = a.x;
            msg.linear_acceleration.y = a.y;
            msg.linear_acceleration.z = a.z;
            msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
            msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
            msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
        }
        else
        {
            msg.linear_acceleration_covariance[0] = -1;
        }

        pub->publish(msg);
    }
};

#endif
