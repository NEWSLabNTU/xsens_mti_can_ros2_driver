// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "packetcallback.h"
#include "xsens_mti_can_ros_driver/msg/xs_status_word.hpp"

struct StatusPublisher : public PacketCallback
{
    rclcpp::Publisher<xsens_mti_can_ros_driver::msg::XsStatusWord>::SharedPtr pub;

    explicit StatusPublisher(rclcpp::Node::SharedPtr node)
    {
        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<xsens_mti_can_ros_driver::msg::XsStatusWord>("status", pub_queue_size);
    }

    static void assignMessage(xsens_mti_can_ros_driver::msg::XsStatusWord &msg,
                              const XsStatusWord &status)
    {
        msg.selftest = status.selftest;
        msg.filter_valid = status.filter_valid;
        msg.gnss_fix = status.gnss_fix;
        msg.no_rotation_update_status = status.no_rotation_update_status;
        msg.representative_motion = status.representative_motion;
        msg.clock_bias_estimation = status.clock_bias_estimation;
        msg.clipflag_acc_x = status.clipflag_acc_x;
        msg.clipflag_acc_y = status.clipflag_acc_y;
        msg.clipflag_acc_z = status.clipflag_acc_z;
        msg.clipflag_gyr_x = status.clipflag_gyr_x;
        msg.clipflag_gyr_y = status.clipflag_gyr_y;
        msg.clipflag_gyr_z = status.clipflag_gyr_z;
        msg.clipflag_mag_x = status.clipflag_mag_x;
        msg.clipflag_mag_y = status.clipflag_mag_y;
        msg.clipflag_mag_z = status.clipflag_mag_z;
        msg.clipping_indication = status.clipping_indication;
        msg.syncin_marker = status.syncin_marker;
        msg.syncout_marker = status.syncout_marker;
        msg.filter_mode = status.filter_mode;
        msg.have_gnss_time_pulse = status.have_gnss_time_pulse;
        msg.rtk_status = status.rtk_status;
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time /*timestamp*/) override
    {
        if (!packet.containsXsStatusWord)
        {
            return;
        }

        xsens_mti_can_ros_driver::msg::XsStatusWord msg;
        assignMessage(msg, packet.status_word);
        pub->publish(msg);
    }
};

#endif
