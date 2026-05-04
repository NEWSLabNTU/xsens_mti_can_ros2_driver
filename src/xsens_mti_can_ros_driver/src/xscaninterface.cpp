#include "xscaninterface.h"

#include <cerrno>
#include <cstring>
#include <utility>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnssposepublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/packetcallback.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/statuspublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/utctimepublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/velocitypublisher.h"

namespace
{
template <typename PubT>
void register_if_enabled(rclcpp::Node::SharedPtr node, const std::string &param,
                         std::list<std::unique_ptr<PacketCallback>> &out)
{
    bool enabled = false;
    if (node->get_parameter(param, enabled) && enabled)
    {
        out.push_back(std::make_unique<PubT>(node));
    }
}
}  // namespace

XsCanInterface::XsCanInterface(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      socket_(-1),
      m_startframeid(XSENS_SAMPLE_TIME_FRAME_ID),
      lastFrameId_(0xFFFFFFFFu),
      m_timeHandler(node_->get_clock()),
      running_(false),
      prev_counter_valid_(false),
      prev_counter_value_(0),
      last_read_error_log_(0, 0, RCL_ROS_TIME),
      read_timeout_ms_(100),
      reopen_after_errors_(20)
{
}

XsCanInterface::~XsCanInterface()
{
    stop();
}

bool XsCanInterface::initialize()
{
    can_interface_ = node_->declare_parameter<std::string>("can_interface", "can0");

    const int start_frame_id = node_->declare_parameter<int>(
        "start_frame_id", static_cast<int>(XSENS_SAMPLE_TIME_FRAME_ID));
    if (start_frame_id < 0 || start_frame_id > 0x7FF)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "start_frame_id %d out of standard CAN range [0, 0x7FF]",
                     start_frame_id);
        return false;
    }
    m_startframeid = static_cast<uint32_t>(start_frame_id);

    const std::string time_option =
        node_->declare_parameter<std::string>("time_option", "mti_utc");
    m_timeHandler.setTimeOption(time_option);

    const int queue_size = node_->declare_parameter<int>("publisher_queue_size", 5);
    if (queue_size <= 0)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "publisher_queue_size must be positive, got %d", queue_size);
        return false;
    }

    node_->declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);

    read_timeout_ms_ = node_->declare_parameter<int>("read_timeout_ms", 100);
    reopen_after_errors_ =
        node_->declare_parameter<int>("reopen_after_errors", 20);

    // Optional [x,y,z] stddev arrays. Empty = absent → publishers default to
    // covariance[0]=-1 ("unknown") so downstream filters don't trust fabricated
    // zero variance.
    const std::vector<double> empty_stddev;
    node_->declare_parameter<std::vector<double>>("orientation_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("angular_velocity_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("linear_acceleration_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("magnetic_field_stddev", empty_stddev);

    for (const auto &name : {
             "pub_imu", "pub_quaternion", "pub_acceleration",
             "pub_angular_velocity", "pub_mag", "pub_dq",
             "pub_dv", "pub_sampletime", "pub_temperature",
             "pub_pressure", "pub_twist", "pub_free_acceleration",
             "pub_transform", "pub_positionLLA", "pub_velocity",
             "pub_status", "pub_gnsspose", "pub_utctime",
             "pub_gnss", "pub_nmea"})
    {
        node_->declare_parameter<bool>(name, false);
    }

    RCLCPP_INFO(node_->get_logger(),
                "XsCanInterface::initialize, can_interface: %s, start_frame_id: 0x%x, time_option: %s",
                can_interface_.c_str(), m_startframeid, time_option.c_str());

    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    diag_updater_->setHardwareID("xsens_mti_can");
    diag_updater_->add("xsens_mti_can/interface", this, &XsCanInterface::diagInterface);

    if (!openSocket())
    {
        return false;
    }

    return true;
}

bool XsCanInterface::openSocket()
{
    closeSocket();

    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: %s",
                     ::strerror(errno));
        return false;
    }

    // Read timeout lets the loop check `running_` between reads and avoids
    // depending solely on shutdown() to unblock on stop.
    struct timeval tv = {};
    tv.tv_sec = read_timeout_ms_ / 1000;
    tv.tv_usec = (read_timeout_ms_ % 1000) * 1000;
    if (::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "setsockopt(SO_RCVTIMEO) failed: %s — relying on shutdown() for unblock",
                    ::strerror(errno));
    }

    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "ioctl SIOCGIFINDEX on %s failed: %s",
                     can_interface_.c_str(), ::strerror(errno));
        closeSocket();
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "bind() on %s failed: %s",
                     can_interface_.c_str(), ::strerror(errno));
        closeSocket();
        return false;
    }

    socket_healthy_.store(true);
    return true;
}

void XsCanInterface::closeSocket()
{
    if (socket_ >= 0)
    {
        ::shutdown(socket_, SHUT_RDWR);
        ::close(socket_);
        socket_ = -1;
    }
    socket_healthy_.store(false);
}

void XsCanInterface::registerPublishers()
{
    register_if_enabled<ImuPublisher>(node_, "pub_imu", m_callbacks);
    register_if_enabled<OrientationPublisher>(node_, "pub_quaternion", m_callbacks);
    register_if_enabled<AccelerationPublisher>(node_, "pub_acceleration", m_callbacks);
    register_if_enabled<AngularVelocityPublisher>(node_, "pub_angular_velocity", m_callbacks);
    register_if_enabled<MagneticFieldPublisher>(node_, "pub_mag", m_callbacks);
    register_if_enabled<OrientationIncrementsPublisher>(node_, "pub_dq", m_callbacks);
    register_if_enabled<VelocityIncrementPublisher>(node_, "pub_dv", m_callbacks);
    register_if_enabled<TimeReferencePublisher>(node_, "pub_sampletime", m_callbacks);
    register_if_enabled<TemperaturePublisher>(node_, "pub_temperature", m_callbacks);
    register_if_enabled<PressurePublisher>(node_, "pub_pressure", m_callbacks);
    register_if_enabled<TwistPublisher>(node_, "pub_twist", m_callbacks);
    register_if_enabled<FreeAccelerationPublisher>(node_, "pub_free_acceleration", m_callbacks);
    register_if_enabled<TransformPublisher>(node_, "pub_transform", m_callbacks);
    register_if_enabled<PositionLLAPublisher>(node_, "pub_positionLLA", m_callbacks);
    register_if_enabled<VelocityPublisher>(node_, "pub_velocity", m_callbacks);
    register_if_enabled<StatusPublisher>(node_, "pub_status", m_callbacks);
    register_if_enabled<GNSSPOSEPublisher>(node_, "pub_gnsspose", m_callbacks);
    register_if_enabled<UTCTimePublisher>(node_, "pub_utctime", m_callbacks);
}

void XsCanInterface::start()
{
    if (reader_thread_.joinable())
    {
        // Already running — guard against accidental double-start.
        return;
    }
    if (socket_ < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "CAN socket not open; cannot start read loop");
        return;
    }
    running_ = true;
    reader_thread_ = std::thread(&XsCanInterface::readLoop, this);
}

void XsCanInterface::stop()
{
    running_ = false;
    // Unblocks any read() blocked past SO_RCVTIMEO.
    if (socket_ >= 0)
    {
        ::shutdown(socket_, SHUT_RDWR);
    }
    if (reader_thread_.joinable())
    {
        reader_thread_.join();
    }
    closeSocket();
}

void XsCanInterface::readLoop()
{
    int consecutive_errors = 0;
    while (running_ && rclcpp::ok())
    {
        struct can_frame frame;
        ssize_t nbytes = ::read(socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            // Read timeout — let the loop re-check running_ and continue.
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }

            read_errors_.fetch_add(1, std::memory_order_relaxed);
            consecutive_errors++;

            // Throttle error logs to once per second so a sustained failure
            // doesn't flood the log.
            const auto now = node_->get_clock()->now();
            if ((now - last_read_error_log_).seconds() >= 1.0)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "CAN read error (consecutive=%d): %s",
                             consecutive_errors, ::strerror(errno));
                last_read_error_log_ = now;
            }

            if (consecutive_errors >= reopen_after_errors_)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Reopening CAN socket on %s after %d errors",
                            can_interface_.c_str(), consecutive_errors);
                if (openSocket())
                {
                    consecutive_errors = 0;
                }
                else
                {
                    // Backoff to avoid tight reopen loop.
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
            continue;
        }

        // SOCK_RAW always delivers full frames; partial reads aren't a thing
        // for this socket type. Treat any non-full read as a kernel weirdness
        // and skip.
        if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame)))
        {
            bad_frames_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        consecutive_errors = 0;
        good_frames_.fetch_add(1, std::memory_order_relaxed);

        // CAN error frame (controller fault, bus-off, etc.). The kernel
        // delivers these with CAN_ERR_FLAG set; the payload is an error class
        // bitmask. Don't try to parse as a data frame.
        if (frame.can_id & CAN_ERR_FLAG)
        {
            can_err_frames_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "CAN bus error frame (err_class=0x%x)",
                                 frame.can_id & CAN_ERR_MASK);
            continue;
        }

        // Mask off any flag bits — frame.can_id might carry CAN_RTR_FLAG or
        // CAN_EFF_FLAG. We only consume standard data frames for Xsens.
        if (frame.can_id & CAN_RTR_FLAG)
        {
            continue;
        }
        const uint32_t currentFrameId =
            (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                          : (frame.can_id & CAN_SFF_MASK);
        // Patch frame.can_id in place so saveToPacket sees a clean ID.
        frame.can_id = currentFrameId;

        if (currentFrameId == m_startframeid)
        {
            if (lastFrameId_ != 0xFFFFFFFFu)
            {
                dispatchPacket();
            }

            packet_ = XsDataPacket();
            saveToPacket(frame, packet_);
        }
        else
        {
            saveToPacket(frame, packet_);
        }

        lastFrameId_ = currentFrameId;
    }
}

void XsCanInterface::processCANMessages()
{
    // Kept as a thin wrapper for backward compatibility; readLoop now drives
    // the socket directly so it can handle reopen + error frames in one place.
}

void XsCanInterface::dispatchPacket()
{
    // Group-counter integrity: a missing tick means we lost frames between
    // dispatches; drop the sample rather than emitting a Frankensteined
    // packet. First valid counter just primes the reference.
    if (packet_.containsXsGroupCounter)
    {
        const uint16_t counter = packet_.counter.counter;
        if (prev_counter_valid_)
        {
            const uint16_t expected = static_cast<uint16_t>(prev_counter_value_ + 1);
            if (counter != expected)
            {
                dropped_samples_.fetch_add(1, std::memory_order_relaxed);
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "GroupCounter gap: expected %u got %u — dropping sample",
                                     expected, counter);
                prev_counter_value_ = counter;
                return;
            }
        }
        prev_counter_value_ = counter;
        prev_counter_valid_ = true;
    }

    // Time-option fallback observability.
    if (packet_.containsXsError)
    {
        error_latched_.store(true);
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                              "MTi reports XsError (output buffer overflow=%d)",
                              packet_.error.CEI_OutputBufferOverflow);
    }
    if (packet_.containsXsWarning)
    {
        warning_latched_.store(true);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "MTi reports XsWarning (code=0x%x)",
                             packet_.warning.warning_code);
    }

    rclcpp::Time now = m_timeHandler.convertUtcTimeToRosTime(packet_);
    for (auto &cb : m_callbacks)
    {
        (*cb)(packet_, now);
    }
}

void XsCanInterface::saveToPacket(const can_frame &frame, XsDataPacket &packet)
{
    bool parsed_ok = true;
    switch (frame.can_id)
    {
    case XSENS_ERROR_FRAME_ID:
        if (xserror_unpack(packet.error, frame))
        {
            packet.containsXsError = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_WARNING_FRAME_ID:
        if (xswarning_unpack(packet.warning, frame))
        {
            packet.containsXsWarning = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_SAMPLE_TIME_FRAME_ID:
        if (xssampletime_unpack(packet.sample_time_fine, frame))
        {
            packet.containsXsSampleTimeFine = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_GROUP_COUNTER_FRAME_ID:
        if (xsgroupcounter_unpack(packet.counter, frame))
        {
            packet.containsXsGroupCounter = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_UTC_FRAME_ID:
        if (xsutctime_unpack(packet.utc_time, frame))
        {
            packet.containsXsUtcTime = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_STATUS_WORD_FRAME_ID:
        if (xsstatusword_unpack(packet.status_word, frame))
        {
            packet.containsXsStatusWord = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_QUATERNION_FRAME_ID:
        if (xsquaternion_unpack(packet.quaternion, frame))
        {
            packet.containsXsQuaternion = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_EULER_ANGLES_FRAME_ID:
        if (xseuler_unpack(packet.euler, frame))
        {
            packet.containsXsEuler = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_DELTA_V_FRAME_ID:
        if (xsdeltavelocity_unpack(packet.delta_velocity, frame))
        {
            packet.containsXsDeltaVelocity = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_RATE_OF_TURN_FRAME_ID:
        if (xsrateofturn_unpack(packet.rate_of_turn, frame))
        {
            packet.containsXsRateOfTurn = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_DELTA_Q_FRAME_ID:
        if (xsquaternion_unpack(packet.delta_q, frame))
        {
            packet.containsXsDeltaQ = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_ACCELERATION_FRAME_ID:
        if (xsacceleration_unpack(packet.acceleration, frame))
        {
            packet.containsXsAcceleration = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_FREE_ACCELERATION_FRAME_ID:
        if (xsacceleration_unpack(packet.free_acceleration, frame))
        {
            packet.containsXsFreeAcceleration = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_MAGNETIC_FIELD_FRAME_ID:
        if (xsmagneticfield_unpack(packet.magnetic_field, frame))
        {
            packet.containsXsMagneticField = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_TEMPERATURE_FRAME_ID:
        if (xstemperature_unpack(packet.temperature, frame))
        {
            packet.containsXsTemperature = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_BAROMETRIC_PRESSURE_FRAME_ID:
        if (xsbaropressure_unpack(packet.baro_pressure, frame))
        {
            packet.containsXsBarometricPressure = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_RATE_OF_TURN_HR_FRAME_ID:
        if (xsrateofturn_unpack(packet.rate_of_turn_hr, frame))
        {
            packet.containsXsRateOfTurnHR = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_ACCELERATION_HR_FRAME_ID:
        if (xsacceleration_unpack(packet.acceleration_hr, frame))
        {
            packet.containsXsAccelerationHR = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_LAT_LON_FRAME_ID:
        if (xslatlon_unpack(packet.lat_lon, frame))
        {
            packet.containsXsLatLon = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_ALTITUDE_ELLIPSOID_FRAME_ID:
        if (xsaltellipsoid_unpack(packet.altitude, frame))
        {
            packet.containsXsAltitudeEllipsoid = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_POSITION_ECEF_X_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.x, frame))
        {
            packet.containsXsPositionEcefX = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_POSITION_ECEF_Y_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.y, frame))
        {
            packet.containsXsPositionEcefY = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_POSITION_ECEF_Z_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.z, frame))
        {
            packet.containsXsPositionEcefZ = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_VELOCITY_FRAME_ID:
        if (xsvelocity_unpack(packet.velocity, frame))
        {
            packet.containsXsVelocity = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_GNSS_RECEIVER_STATUS_FRAME_ID:
        if (xsgnsssreceiverstatus_unpack(packet.gnss_receiver_status, frame))
        {
            packet.containsXsGnssReceiverStatus = true;
        }
        else parsed_ok = false;
        break;
    case XSENS_GNSS_RECEIVER_DOP_FRAME_ID:
        if (xsgnsssreceiverdop_unpack(packet.gnss_receiver_dop, frame))
        {
            packet.containsXsGnssReceiverDop = true;
        }
        else parsed_ok = false;
        break;
    default:
        unknown_frames_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Unknown CAN ID: 0x%x", frame.can_id);
        return;
    }

    if (!parsed_ok)
    {
        bad_frames_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Short payload for CAN ID 0x%x (dlc=%u)",
                             frame.can_id, frame.can_dlc);
    }
}

void XsCanInterface::registerCallback(std::unique_ptr<PacketCallback> cb)
{
    m_callbacks.push_back(std::move(cb));
}

void XsCanInterface::diagInterface(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    using LV = diagnostic_msgs::msg::DiagnosticStatus;
    const bool healthy = socket_healthy_.load();
    const bool err = error_latched_.load();
    const bool warn = warning_latched_.load();

    if (!healthy)
    {
        stat.summary(LV::ERROR, "CAN socket unhealthy");
    }
    else if (err)
    {
        stat.summary(LV::ERROR, "MTi reported XsError");
    }
    else if (warn)
    {
        stat.summary(LV::WARN, "MTi reported XsWarning");
    }
    else
    {
        stat.summary(LV::OK, "ok");
    }

    stat.add("socket_healthy", healthy);
    stat.add("can_interface", can_interface_);
    stat.add("good_frames", good_frames_.load());
    stat.add("bad_frames", bad_frames_.load());
    stat.add("can_err_frames", can_err_frames_.load());
    stat.add("unknown_frames", unknown_frames_.load());
    stat.add("dropped_samples", dropped_samples_.load());
    stat.add("read_errors", read_errors_.load());
    stat.add("xs_error_latched", err);
    stat.add("xs_warning_latched", warn);
}
