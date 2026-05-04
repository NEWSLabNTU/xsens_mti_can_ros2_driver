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
    bool has_orientation_stddev = false;
    bool has_angular_velocity_stddev = false;
    bool has_linear_acceleration_stddev = false;

    explicit ImuPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        // SensorDataQoS = BEST_EFFORT, KeepLast(N). Matches Autoware sensor
        // consumer convention; mismatched (Reliable vs BestEffort) QoS would
        // silently drop all messages.
        auto qos = rclcpp::SensorDataQoS().keep_last(pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        node->get_parameter("frame_id", frame_id);

        // REP 145: Conventions for IMU Sensor Drivers. If a stddev parameter
        // isn't supplied we publish covariance[0]=-1 ("unknown") rather than
        // fabricating zero variance — downstream filters must not be told
        // the data is perfectly known when it isn't.
        has_orientation_stddev =
            variance_from_stddev_param(node, "orientation_stddev", orientation_variance);
        has_angular_velocity_stddev =
            variance_from_stddev_param(node, "angular_velocity_stddev", angular_velocity_variance);
        has_linear_acceleration_stddev = variance_from_stddev_param(
            node, "linear_acceleration_stddev", linear_acceleration_variance);
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
            if (has_orientation_stddev)
            {
                msg.orientation_covariance[0] = orientation_variance[0];
                msg.orientation_covariance[4] = orientation_variance[1];
                msg.orientation_covariance[8] = orientation_variance[2];
            }
            else
            {
                msg.orientation_covariance[0] = -1;
            }
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
            if (has_angular_velocity_stddev)
            {
                msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
                msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
                msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
            }
            else
            {
                msg.angular_velocity_covariance[0] = -1;
            }
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
            if (has_linear_acceleration_stddev)
            {
                msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
                msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
                msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
            }
            else
            {
                msg.linear_acceleration_covariance[0] = -1;
            }
        }
        else
        {
            msg.linear_acceleration_covariance[0] = -1;
        }

        pub->publish(msg);
    }
};

#endif
