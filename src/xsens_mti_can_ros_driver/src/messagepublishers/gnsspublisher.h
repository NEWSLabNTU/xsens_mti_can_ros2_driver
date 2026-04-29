// Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
// SPDX-License-Identifier: BSD-3-Clause
//
// Reference implementation kept commented out — depends on raw GNSS PVT fields
// the CAN parser does not yet decode. Re-enable once those become available.
#ifndef GNSSPUBLISHER_H
#define GNSSPUBLISHER_H

// #include <sensor_msgs/msg/nav_sat_fix.hpp>
//
// #include "packetcallback.h"
//
// #define FIX_TYPE_2D_FIX (2)
// #define FIX_TYPE_3D_FIX (3)
// #define FIX_TYPE_GNSS_AND_DEAD_RECKONING (4)
//
// struct GnssPublisher : public PacketCallback
// {
//     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub;
//     std::string frame_id = DEFAULT_FRAME_ID;
//
//     explicit GnssPublisher(rclcpp::Node::SharedPtr node)
//     {
//         int pub_queue_size = 5;
//         node->get_parameter("publisher_queue_size", pub_queue_size);
//         pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("gnss", pub_queue_size);
//         node->get_parameter("frame_id", frame_id);
//     }
//
//     void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
//     {
//         if (!(packet.containsXsGnssReceiverStatus && packet.containsXsGnssReceiverDop))
//         {
//             return;
//         }
//         sensor_msgs::msg::NavSatFix msg;
//         msg.header.stamp = timestamp;
//         msg.header.frame_id = frame_id;
//         const XsGnssReceiverStatus status = packet.gnss_receiver_status;
//         switch (status.fix_type)
//         {
//         case FIX_TYPE_2D_FIX:
//         case FIX_TYPE_3D_FIX:
//         case FIX_TYPE_GNSS_AND_DEAD_RECKONING:
//             msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
//             break;
//         default:
//             msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
//         }
//         msg.status.service = 0;
//         pub->publish(msg);
//     }
// };

#endif
