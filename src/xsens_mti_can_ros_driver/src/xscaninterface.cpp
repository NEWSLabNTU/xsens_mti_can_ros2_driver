#include "xscaninterface.h"

#include <cstring>
#include <utility>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
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
      running_(false)
{
}

XsCanInterface::~XsCanInterface()
{
    stop();
}

void XsCanInterface::initialize()
{
    // Declare all parameters up front so publishers can simply get_parameter().
    can_interface_ = node_->declare_parameter<std::string>("can_interface", "can0");
    const int start_frame_id = node_->declare_parameter<int>(
        "start_frame_id", static_cast<int>(XSENS_SAMPLE_TIME_FRAME_ID));
    m_startframeid = static_cast<uint32_t>(start_frame_id);

    const std::string time_option =
        node_->declare_parameter<std::string>("time_option", "mti_utc");
    m_timeHandler.setTimeOption(time_option);

    node_->declare_parameter<int>("publisher_queue_size", 5);
    node_->declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);

    // Optional [x,y,z] stddev arrays for IMU/MagneticField covariances.
    const std::vector<double> empty_stddev;
    node_->declare_parameter<std::vector<double>>("orientation_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("angular_velocity_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("linear_acceleration_stddev", empty_stddev);
    node_->declare_parameter<std::vector<double>>("magnetic_field_stddev", empty_stddev);

    // Per-topic enable flags.
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
                "XsCanInterface::initialize, can_interface: %s, start_frame_id: %u, time_option: %s",
                can_interface_.c_str(), m_startframeid, time_option.c_str());

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open SocketCAN socket: %s", strerror(errno));
        return;
    }

    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "ioctl SIOCGIFINDEX on %s failed: %s",
                     can_interface_.c_str(), strerror(errno));
        ::close(socket_);
        socket_ = -1;
        return;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "bind() on %s failed: %s",
                     can_interface_.c_str(), strerror(errno));
        ::close(socket_);
        socket_ = -1;
    }
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
    if (socket_ >= 0)
    {
        // shutdown() unblocks any in-flight read() in the reader thread.
        ::shutdown(socket_, SHUT_RDWR);
    }
    if (reader_thread_.joinable())
    {
        reader_thread_.join();
    }
    if (socket_ >= 0)
    {
        ::close(socket_);
        socket_ = -1;
    }
}

void XsCanInterface::readLoop()
{
    while (running_ && rclcpp::ok())
    {
        processCANMessages();
    }
}

void XsCanInterface::processCANMessages()
{
    struct can_frame frame;
    ssize_t nbytes = read(socket_, &frame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
        if (running_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error while reading CAN frame: %s", strerror(errno));
        }
        return;
    }

    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Incomplete CAN frame read");
        return;
    }

    const uint32_t currentFrameId = frame.can_id;

    if (currentFrameId == m_startframeid)
    {
        if (lastFrameId_ != 0xFFFFFFFFu)
        {
            rclcpp::Time now = m_timeHandler.convertUtcTimeToRosTime(packet_);
            for (auto &cb : m_callbacks)
            {
                (*cb)(packet_, now);
            }
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

void XsCanInterface::saveToPacket(const can_frame &frame, XsDataPacket &packet)
{
    switch (frame.can_id)
    {
    case XSENS_ERROR_FRAME_ID:
        if (xserror_unpack(packet.error, frame))
        {
            packet.containsXsError = true;
        }
        break;
    case XSENS_WARNING_FRAME_ID:
        if (xswarning_unpack(packet.warning, frame))
        {
            packet.containsXsWarning = true;
        }
        break;
    case XSENS_SAMPLE_TIME_FRAME_ID:
        if (xssampletime_unpack(packet.sample_time_fine, frame))
        {
            packet.containsXsSampleTimeFine = true;
        }
        break;
    case XSENS_GROUP_COUNTER_FRAME_ID:
        if (xsgroupcounter_unpack(packet.counter, frame))
        {
            packet.containsXsGroupCounter = true;
        }
        break;
    case XSENS_UTC_FRAME_ID:
        if (xsutctime_unpack(packet.utc_time, frame))
        {
            packet.containsXsUtcTime = true;
        }
        break;
    case XSENS_STATUS_WORD_FRAME_ID:
        if (xsstatusword_unpack(packet.status_word, frame))
        {
            packet.containsXsStatusWord = true;
        }
        break;
    case XSENS_QUATERNION_FRAME_ID:
        if (xsquaternion_unpack(packet.quaternion, frame))
        {
            packet.containsXsQuaternion = true;
        }
        break;
    case XSENS_EULER_ANGLES_FRAME_ID:
        if (xseuler_unpack(packet.euler, frame))
        {
            packet.containsXsEuler = true;
        }
        break;
    case XSENS_DELTA_V_FRAME_ID:
        if (xsdeltavelocity_unpack(packet.delta_velocity, frame))
        {
            packet.containsXsDeltaVelocity = true;
        }
        break;
    case XSENS_RATE_OF_TURN_FRAME_ID:
        if (xsrateofturn_unpack(packet.rate_of_turn, frame))
        {
            packet.containsXsRateOfTurn = true;
        }
        break;
    case XSENS_DELTA_Q_FRAME_ID:
        if (xsquaternion_unpack(packet.delta_q, frame))
        {
            packet.containsXsDeltaQ = true;
        }
        break;
    case XSENS_ACCELERATION_FRAME_ID:
        if (xsacceleration_unpack(packet.acceleration, frame))
        {
            packet.containsXsAcceleration = true;
        }
        break;
    case XSENS_FREE_ACCELERATION_FRAME_ID:
        if (xsacceleration_unpack(packet.free_acceleration, frame))
        {
            packet.containsXsFreeAcceleration = true;
        }
        break;
    case XSENS_MAGNETIC_FIELD_FRAME_ID:
        if (xsmagneticfield_unpack(packet.magnetic_field, frame))
        {
            packet.containsXsMagneticField = true;
        }
        break;
    case XSENS_TEMPERATURE_FRAME_ID:
        if (xstemperature_unpack(packet.temperature, frame))
        {
            packet.containsXsTemperature = true;
        }
        break;
    case XSENS_BAROMETRIC_PRESSURE_FRAME_ID:
        if (xsbaropressure_unpack(packet.baro_pressure, frame))
        {
            packet.containsXsBarometricPressure = true;
        }
        break;
    case XSENS_RATE_OF_TURN_HR_FRAME_ID:
        if (xsrateofturn_unpack(packet.rate_of_turn_hr, frame))
        {
            packet.containsXsRateOfTurnHR = true;
        }
        break;
    case XSENS_ACCELERATION_HR_FRAME_ID:
        if (xsacceleration_unpack(packet.acceleration_hr, frame))
        {
            packet.containsXsAccelerationHR = true;
        }
        break;
    case XSENS_LAT_LON_FRAME_ID:
        if (xslatlon_unpack(packet.lat_lon, frame))
        {
            packet.containsXsLatLon = true;
        }
        break;
    case XSENS_ALTITUDE_ELLIPSOID_FRAME_ID:
        if (xsaltellipsoid_unpack(packet.altitude, frame))
        {
            packet.containsXsAltitudeEllipsoid = true;
        }
        break;
    case XSENS_POSITION_ECEF_X_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.x, frame))
        {
            packet.containsXsPositionEcefX = true;
        }
        break;
    case XSENS_POSITION_ECEF_Y_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.y, frame))
        {
            packet.containsXsPositionEcefY = true;
        }
        break;
    case XSENS_POSITION_ECEF_Z_FRAME_ID:
        if (xspositionecefX_unpack(packet.position_ecef.z, frame))
        {
            packet.containsXsPositionEcefZ = true;
        }
        break;
    case XSENS_VELOCITY_FRAME_ID:
        if (xsvelocity_unpack(packet.velocity, frame))
        {
            packet.containsXsVelocity = true;
        }
        break;
    case XSENS_GNSS_RECEIVER_STATUS_FRAME_ID:
        if (xsgnsssreceiverstatus_unpack(packet.gnss_receiver_status, frame))
        {
            packet.containsXsGnssReceiverStatus = true;
        }
        break;
    case XSENS_GNSS_RECEIVER_DOP_FRAME_ID:
        if (xsgnsssreceiverdop_unpack(packet.gnss_receiver_dop, frame))
        {
            packet.containsXsGnssReceiverDop = true;
        }
        break;
    default:
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Unknown CAN ID: 0x%x", frame.can_id);
        break;
    }
}

void XsCanInterface::registerCallback(std::unique_ptr<PacketCallback> cb)
{
    m_callbacks.push_back(std::move(cb));
}
