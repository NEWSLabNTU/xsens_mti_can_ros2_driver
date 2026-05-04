#include "xsens_parser.h"

#include <cstdint>

namespace
{
// All multi-byte fields in the Xsens CAN protocol are big-endian. Cast bytes
// to unsigned before shifting so the operation is defined for any payload
// (signed-int shifts overflow into the sign bit on values with the MSB set,
// which is undefined behavior in C++).
inline uint16_t be_u16(const uint8_t *p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) << 8 |
                                 static_cast<uint16_t>(p[1]));
}

inline uint32_t be_u32(const uint8_t *p)
{
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

inline int16_t be_i16(const uint8_t *p)
{
    return static_cast<int16_t>(be_u16(p));
}

inline int32_t be_i32(const uint8_t *p)
{
    return static_cast<int32_t>(be_u32(p));
}
}  // namespace

bool xserror_unpack(XsError &e, const struct can_frame &frame)
{
    if (frame.can_dlc < 1)
    {
        return false;
    }

    e.CEI_OutputBufferOverflow = (frame.data[0] == 0x01);

    return true;
}

bool xswarning_unpack(XsWarning &w, const struct can_frame &frame)
{
    if (frame.can_dlc < 1)
    {
        return false;
    }

    w.warning_code = frame.data[0];

    return true;
}

bool xssampletime_unpack(XsSampleTimeFine &sampleTime, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        return false;
    }

    sampleTime.timestamp = be_u32(frame.data);
    return true;
}

bool xsgroupcounter_unpack(XsGroupCounter &groupCounter, const struct can_frame &frame)
{
    if (frame.can_dlc < 2)
    {
        return false;
    }

    groupCounter.counter = be_u16(frame.data);
    return true;
}

bool xsutctime_unpack(XsUtcTime &utctime, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        return false;
    }
    uint8_t year = frame.data[0];
    // Convert raw uint8 (e.g. 23) to full year. Valid until 2069 — the < 70
    // heuristic flips to 1900-base after that. Acceptable for vehicle service
    // life.
    if (year < 70)
    {
        utctime.year = static_cast<uint16_t>(2000 + year);
    }
    else
    {
        utctime.year = static_cast<uint16_t>(1900 + year);
    }
    utctime.month = frame.data[1];
    utctime.day = frame.data[2];
    utctime.hour = frame.data[3];
    utctime.minute = frame.data[4];
    utctime.second = frame.data[5];
    utctime.tenthms = be_u16(frame.data + 6);

    return true;
}

bool xsquaternion_unpack(XsQuaternion &q, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        return false;
    }

    const double scale = 1.0 / ((1 << 15) - 1);
    float *q_arr[] = {&q.q0, &q.q1, &q.q2, &q.q3};

    for (size_t i = 0; i < 4; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *q_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xseuler_unpack(XsEuler &euler, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 7); // 0.0078125
    float *euler_arr[] = {&euler.roll, &euler.pitch, &euler.yaw};

    for (size_t i = 0; i < 3; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *euler_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsacceleration_unpack(XsAcceleration &acc, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 8); // 0.00390625
    float *acc_arr[] = {&acc.x, &acc.y, &acc.z};

    for (size_t i = 0; i < 3; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *acc_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsrateofturn_unpack(XsRateOfTurn &gyro, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 9); // 0.001953125
    float *gyro_arr[] = {&gyro.x, &gyro.y, &gyro.z};

    for (size_t i = 0; i < 3; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *gyro_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsdeltavelocity_unpack(XsDeltaVelocity &dv, const struct can_frame &frame)
{
    if (frame.can_dlc < 7)
    {
        return false;
    }

    // Exponent comes from the wire — reject values that would make `1 <<
    // exponent` undefined (≥ 31 on int) or numerically meaningless. The
    // protocol spec uses small, fixed exponents; anything large is corruption.
    const uint8_t exponent = frame.data[6];
    if (exponent >= 31)
    {
        return false;
    }
    const double scale = 1.0 / static_cast<double>(1u << exponent);
    float *dv_arr[] = {&dv.x, &dv.y, &dv.z};

    for (size_t i = 0; i < 3; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *dv_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xsmagneticfield_unpack(XsMagneticField &mag, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 10); // 0.0009765625
    float *mag_arr[] = {&mag.x, &mag.y, &mag.z};

    for (size_t i = 0; i < 3; ++i)
    {
        const int16_t value = be_i16(frame.data + 2 * i);
        *mag_arr[i] = static_cast<float>(value * scale);
    }

    return true;
}

bool xslatlon_unpack(XsLatLon &latlon, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        return false;
    }

    const double scale_lat = 1.0 / (1 << 24); // 5.9604644775e-08
    const double scale_lon = 1.0 / (1 << 23); // 1.1920928955e-07

    const int32_t latitude = be_i32(frame.data);
    const int32_t longitude = be_i32(frame.data + 4);

    latlon.latitude = static_cast<double>(latitude) * scale_lat;
    latlon.longitude = static_cast<double>(longitude) * scale_lon;

    return true;
}

bool xsaltellipsoid_unpack(XsAltitudeEllipsoid &alt, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 15); // 3.0517578125e-05
    const int32_t alt_ellipsoid = be_i32(frame.data);

    alt.alt_ellipsoid = static_cast<double>(alt_ellipsoid) * scale;
    return true;
}

bool xspositionecefX_unpack(double &pos, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 8); // 0.00390625
    const int32_t x = be_i32(frame.data);
    pos = static_cast<double>(x) * scale;
    return true;
}

bool xsvelocity_unpack(XsVelocity &vel, const struct can_frame &frame)
{
    if (frame.can_dlc < 6)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 6); // 0.015625

    vel.x = static_cast<float>(be_i16(frame.data) * scale);
    vel.y = static_cast<float>(be_i16(frame.data + 2) * scale);
    vel.z = static_cast<float>(be_i16(frame.data + 4) * scale);

    return true;
}

bool xsstatusword_unpack(XsStatusWord &status, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        return false;
    }

    const uint32_t statusWord = be_u32(frame.data);

    status.selftest = statusWord & (1u << 0);
    status.filter_valid = statusWord & (1u << 1);
    status.gnss_fix = statusWord & (1u << 2);
    status.no_rotation_update_status = (statusWord >> 3) & 0x3u;
    status.representative_motion = statusWord & (1u << 5);
    status.clock_bias_estimation = statusWord & (1u << 6);
    status.clipflag_acc_x = statusWord & (1u << 8);
    status.clipflag_acc_y = statusWord & (1u << 9);
    status.clipflag_acc_z = statusWord & (1u << 10);
    status.clipflag_gyr_x = statusWord & (1u << 11);
    status.clipflag_gyr_y = statusWord & (1u << 12);
    status.clipflag_gyr_z = statusWord & (1u << 13);
    status.clipflag_mag_x = statusWord & (1u << 14);
    status.clipflag_mag_y = statusWord & (1u << 15);
    status.clipflag_mag_z = statusWord & (1u << 16);
    status.clipping_indication = statusWord & (1u << 19);
    status.syncin_marker = statusWord & (1u << 21);
    status.syncout_marker = statusWord & (1u << 22);
    status.filter_mode = static_cast<uint8_t>((statusWord >> 23) & 0x7u);
    status.have_gnss_time_pulse = statusWord & (1u << 26);
    status.rtk_status = static_cast<uint8_t>((statusWord >> 27) & 0x3u);

    return true;
}

bool xstemperature_unpack(XsTemperature &temp, const struct can_frame &frame)
{
    if (frame.can_dlc < 2)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 8);
    const uint16_t temperature = be_u16(frame.data);
    temp.temperature = static_cast<float>(temperature * scale);
    return true;
}

bool xsbaropressure_unpack(XsBarometricPressure &pressure, const struct can_frame &frame)
{
    if (frame.can_dlc < 4)
    {
        return false;
    }

    const double scale = 1.0 / (1 << 15);
    const uint32_t temp_pressure = be_u32(frame.data);
    pressure.pressure = static_cast<float>(temp_pressure * scale);
    return true;
}

bool xsgnsssreceiverstatus_unpack(XsGnssReceiverStatus &gnssStatus, const struct can_frame &frame)
{
    if (frame.can_dlc < 5)
    {
        return false;
    }

    gnssStatus.fix_type = frame.data[0];
    gnssStatus.num_sv = frame.data[1];
    gnssStatus.flags = frame.data[2];
    gnssStatus.valid = frame.data[3];
    gnssStatus.num_svs = frame.data[4];
    return true;
}

bool xsgnsssreceiverdop_unpack(XsGnssReceiverDop &gnssDop, const struct can_frame &frame)
{
    if (frame.can_dlc < 8)
    {
        return false;
    }

    const double scale = 0.01;

    const uint16_t pdop = be_u16(frame.data);
    const uint16_t tdop = be_u16(frame.data + 2);
    const uint16_t vdop = be_u16(frame.data + 4);
    const uint16_t hdop = be_u16(frame.data + 6);

    gnssDop.pdop = static_cast<float>(pdop * scale);
    gnssDop.tdop = static_cast<float>(tdop * scale);
    gnssDop.vdop = static_cast<float>(vdop * scale);
    gnssDop.hdop = static_cast<float>(hdop * scale);

    return true;
}
