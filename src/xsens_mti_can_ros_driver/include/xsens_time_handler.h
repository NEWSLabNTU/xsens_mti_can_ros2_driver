#ifndef XSENS_TIME_HANDLER_H
#define XSENS_TIME_HANDLER_H

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "xsens_parser.h"

class XsensTimeHandler
{
public:
    explicit XsensTimeHandler(rclcpp::Clock::SharedPtr clock);
    rclcpp::Time convertUtcTimeToRosTime(const XsDataPacket &packet);
    void setTimeOption(const std::string &option);
    void setRollover(const uint32_t &rollOver);

private:
    rclcpp::Clock::SharedPtr clock_;
    std::string time_option;
    rclcpp::Time firstUTCTimestamp;
    uint32_t prevSampleTimeFine;
    bool isFirstFrame;
    uint32_t m_RollOver = 0xFFFFFFFF;  // 2^32-1
    mutable std::mutex m_mutex;
};

#endif // XSENS_TIME_HANDLER_H
