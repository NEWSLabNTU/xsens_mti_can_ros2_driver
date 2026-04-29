#include "xsens_time_handler.h"

#include <ctime>

XsensTimeHandler::XsensTimeHandler(rclcpp::Clock::SharedPtr clock)
    : clock_(std::move(clock)),
      time_option(""),
      firstUTCTimestamp(0, 0, RCL_ROS_TIME),
      prevSampleTimeFine(0),
      isFirstFrame(true)
{
}

rclcpp::Time XsensTimeHandler::convertUtcTimeToRosTime(const XsDataPacket &packet)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (time_option == "mti_utc" && packet.containsXsUtcTime)
    {
        struct tm timeinfo = {};

        timeinfo.tm_year = packet.utc_time.year - 1900;
        timeinfo.tm_mon = packet.utc_time.month - 1;
        timeinfo.tm_mday = packet.utc_time.day;
        timeinfo.tm_hour = packet.utc_time.hour;
        timeinfo.tm_min = packet.utc_time.minute;
        timeinfo.tm_sec = packet.utc_time.second;

        time_t epochSeconds = timegm(&timeinfo);

        // 1 tenth of a millisecond = 100000 nanoseconds
        return rclcpp::Time(static_cast<int64_t>(epochSeconds),
                            100000u * packet.utc_time.tenthms,
                            RCL_ROS_TIME);
    }
    else if (time_option == "mti_sampletime" && packet.containsXsSampleTimeFine)
    {
        uint32_t currentSampleTimeFine = packet.sample_time_fine.timestamp;

        if (isFirstFrame)
        {
            isFirstFrame = false;
            firstUTCTimestamp = clock_->now();
            prevSampleTimeFine = currentSampleTimeFine;
            return firstUTCTimestamp;
        }

        int64_t timeDiff = static_cast<int64_t>(currentSampleTimeFine) -
                           static_cast<int64_t>(prevSampleTimeFine);

        // Wraparound detection: only adjust when the jump backwards is large
        // enough that an actual rollover is far more likely than late frames.
        if (timeDiff < 0 && timeDiff < -static_cast<int64_t>(m_RollOver / 2))
        {
            timeDiff += m_RollOver;
        }

        // SampleTimeFine ticks at 10 kHz -> seconds = ticks * 1e-4
        rclcpp::Duration deltaTime = rclcpp::Duration::from_seconds(timeDiff * 0.0001);
        firstUTCTimestamp = firstUTCTimestamp + deltaTime;

        prevSampleTimeFine = currentSampleTimeFine;
        return firstUTCTimestamp;
    }

    return clock_->now();
}

void XsensTimeHandler::setTimeOption(const std::string &option)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    time_option = option;
}

void XsensTimeHandler::setRollover(const uint32_t &rollOver)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_RollOver = rollOver;
}
