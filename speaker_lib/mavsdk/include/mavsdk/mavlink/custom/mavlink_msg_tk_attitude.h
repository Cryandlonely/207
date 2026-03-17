#pragma once
// MESSAGE TK_ATTITUDE PACKING

#define MAVLINK_MSG_ID_TK_ATTITUDE 51006


typedef struct __mavlink_tk_attitude_t {
 float pitchAngle; /*< [°] Pitch, unit degree.*/
 float rollAngle; /*< [°] Roll, unit degree.*/
 float yawAngle; /*< [°] Yaw, unit degree.*/
 float angular_velocity_x; /*< [°/s] Angular velocity X-axis, unit degree per second.*/
 float angular_velocity_y; /*< [°/s] Angular velocity Y-axis, unit degree per second.*/
 float angular_velocity_z; /*< [°/s] Angular velocity Z-axis, unit degree per second.*/
 float acceleration_x; /*< [m/s] Acceleration X-axis, unit m/s.*/
 float acceleration_y; /*< [m/s] Acceleration Y-axis, unit m/s.*/
 float acceleration_z; /*< [m/s] Acceleration Z-axis, unit m/s.*/
 uint8_t mode; /*<  Current mode.*/
} mavlink_tk_attitude_t;

#define MAVLINK_MSG_ID_TK_ATTITUDE_LEN 37
#define MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN 37
#define MAVLINK_MSG_ID_51006_LEN 37
#define MAVLINK_MSG_ID_51006_MIN_LEN 37

#define MAVLINK_MSG_ID_TK_ATTITUDE_CRC 117
#define MAVLINK_MSG_ID_51006_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_ATTITUDE { \
    51006, \
    "TK_ATTITUDE", \
    10, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_tk_attitude_t, mode) }, \
         { "pitchAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_attitude_t, pitchAngle) }, \
         { "rollAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_attitude_t, rollAngle) }, \
         { "yawAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_attitude_t, yawAngle) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tk_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tk_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tk_attitude_t, angular_velocity_z) }, \
         { "acceleration_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tk_attitude_t, acceleration_x) }, \
         { "acceleration_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_tk_attitude_t, acceleration_y) }, \
         { "acceleration_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_tk_attitude_t, acceleration_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_ATTITUDE { \
    "TK_ATTITUDE", \
    10, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_tk_attitude_t, mode) }, \
         { "pitchAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_attitude_t, pitchAngle) }, \
         { "rollAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_attitude_t, rollAngle) }, \
         { "yawAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_attitude_t, yawAngle) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tk_attitude_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tk_attitude_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tk_attitude_t, angular_velocity_z) }, \
         { "acceleration_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tk_attitude_t, acceleration_x) }, \
         { "acceleration_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_tk_attitude_t, acceleration_y) }, \
         { "acceleration_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_tk_attitude_t, acceleration_z) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Current mode.
 * @param pitchAngle [°] Pitch, unit degree.
 * @param rollAngle [°] Roll, unit degree.
 * @param yawAngle [°] Yaw, unit degree.
 * @param angular_velocity_x [°/s] Angular velocity X-axis, unit degree per second.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis, unit degree per second.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis, unit degree per second.
 * @param acceleration_x [m/s] Acceleration X-axis, unit m/s.
 * @param acceleration_y [m/s] Acceleration Y-axis, unit m/s.
 * @param acceleration_z [m/s] Acceleration Z-axis, unit m/s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode, float pitchAngle, float rollAngle, float yawAngle, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitchAngle);
    _mav_put_float(buf, 4, rollAngle);
    _mav_put_float(buf, 8, yawAngle);
    _mav_put_float(buf, 12, angular_velocity_x);
    _mav_put_float(buf, 16, angular_velocity_y);
    _mav_put_float(buf, 20, angular_velocity_z);
    _mav_put_float(buf, 24, acceleration_x);
    _mav_put_float(buf, 28, acceleration_y);
    _mav_put_float(buf, 32, acceleration_z);
    _mav_put_uint8_t(buf, 36, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#else
    mavlink_tk_attitude_t packet;
    packet.pitchAngle = pitchAngle;
    packet.rollAngle = rollAngle;
    packet.yawAngle = yawAngle;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
}

/**
 * @brief Pack a tk_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Current mode.
 * @param pitchAngle [°] Pitch, unit degree.
 * @param rollAngle [°] Roll, unit degree.
 * @param yawAngle [°] Yaw, unit degree.
 * @param angular_velocity_x [°/s] Angular velocity X-axis, unit degree per second.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis, unit degree per second.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis, unit degree per second.
 * @param acceleration_x [m/s] Acceleration X-axis, unit m/s.
 * @param acceleration_y [m/s] Acceleration Y-axis, unit m/s.
 * @param acceleration_z [m/s] Acceleration Z-axis, unit m/s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_attitude_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t mode, float pitchAngle, float rollAngle, float yawAngle, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitchAngle);
    _mav_put_float(buf, 4, rollAngle);
    _mav_put_float(buf, 8, yawAngle);
    _mav_put_float(buf, 12, angular_velocity_x);
    _mav_put_float(buf, 16, angular_velocity_y);
    _mav_put_float(buf, 20, angular_velocity_z);
    _mav_put_float(buf, 24, acceleration_x);
    _mav_put_float(buf, 28, acceleration_y);
    _mav_put_float(buf, 32, acceleration_z);
    _mav_put_uint8_t(buf, 36, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#else
    mavlink_tk_attitude_t packet;
    packet.pitchAngle = pitchAngle;
    packet.rollAngle = rollAngle;
    packet.yawAngle = yawAngle;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a tk_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  Current mode.
 * @param pitchAngle [°] Pitch, unit degree.
 * @param rollAngle [°] Roll, unit degree.
 * @param yawAngle [°] Yaw, unit degree.
 * @param angular_velocity_x [°/s] Angular velocity X-axis, unit degree per second.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis, unit degree per second.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis, unit degree per second.
 * @param acceleration_x [m/s] Acceleration X-axis, unit m/s.
 * @param acceleration_y [m/s] Acceleration Y-axis, unit m/s.
 * @param acceleration_z [m/s] Acceleration Z-axis, unit m/s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode,float pitchAngle,float rollAngle,float yawAngle,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z,float acceleration_x,float acceleration_y,float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitchAngle);
    _mav_put_float(buf, 4, rollAngle);
    _mav_put_float(buf, 8, yawAngle);
    _mav_put_float(buf, 12, angular_velocity_x);
    _mav_put_float(buf, 16, angular_velocity_y);
    _mav_put_float(buf, 20, angular_velocity_z);
    _mav_put_float(buf, 24, acceleration_x);
    _mav_put_float(buf, 28, acceleration_y);
    _mav_put_float(buf, 32, acceleration_z);
    _mav_put_uint8_t(buf, 36, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#else
    mavlink_tk_attitude_t packet;
    packet.pitchAngle = pitchAngle;
    packet.rollAngle = rollAngle;
    packet.yawAngle = yawAngle;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
}

/**
 * @brief Encode a tk_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_attitude_t* tk_attitude)
{
    return mavlink_msg_tk_attitude_pack(system_id, component_id, msg, tk_attitude->mode, tk_attitude->pitchAngle, tk_attitude->rollAngle, tk_attitude->yawAngle, tk_attitude->angular_velocity_x, tk_attitude->angular_velocity_y, tk_attitude->angular_velocity_z, tk_attitude->acceleration_x, tk_attitude->acceleration_y, tk_attitude->acceleration_z);
}

/**
 * @brief Encode a tk_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_attitude_t* tk_attitude)
{
    return mavlink_msg_tk_attitude_pack_chan(system_id, component_id, chan, msg, tk_attitude->mode, tk_attitude->pitchAngle, tk_attitude->rollAngle, tk_attitude->yawAngle, tk_attitude->angular_velocity_x, tk_attitude->angular_velocity_y, tk_attitude->angular_velocity_z, tk_attitude->acceleration_x, tk_attitude->acceleration_y, tk_attitude->acceleration_z);
}

/**
 * @brief Encode a tk_attitude struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_attitude_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_attitude_t* tk_attitude)
{
    return mavlink_msg_tk_attitude_pack_status(system_id, component_id, _status, msg,  tk_attitude->mode, tk_attitude->pitchAngle, tk_attitude->rollAngle, tk_attitude->yawAngle, tk_attitude->angular_velocity_x, tk_attitude->angular_velocity_y, tk_attitude->angular_velocity_z, tk_attitude->acceleration_x, tk_attitude->acceleration_y, tk_attitude->acceleration_z);
}

/**
 * @brief Send a tk_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  Current mode.
 * @param pitchAngle [°] Pitch, unit degree.
 * @param rollAngle [°] Roll, unit degree.
 * @param yawAngle [°] Yaw, unit degree.
 * @param angular_velocity_x [°/s] Angular velocity X-axis, unit degree per second.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis, unit degree per second.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis, unit degree per second.
 * @param acceleration_x [m/s] Acceleration X-axis, unit m/s.
 * @param acceleration_y [m/s] Acceleration Y-axis, unit m/s.
 * @param acceleration_z [m/s] Acceleration Z-axis, unit m/s.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_attitude_send(mavlink_channel_t chan, uint8_t mode, float pitchAngle, float rollAngle, float yawAngle, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitchAngle);
    _mav_put_float(buf, 4, rollAngle);
    _mav_put_float(buf, 8, yawAngle);
    _mav_put_float(buf, 12, angular_velocity_x);
    _mav_put_float(buf, 16, angular_velocity_y);
    _mav_put_float(buf, 20, angular_velocity_z);
    _mav_put_float(buf, 24, acceleration_x);
    _mav_put_float(buf, 28, acceleration_y);
    _mav_put_float(buf, 32, acceleration_z);
    _mav_put_uint8_t(buf, 36, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_ATTITUDE, buf, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#else
    mavlink_tk_attitude_t packet;
    packet.pitchAngle = pitchAngle;
    packet.rollAngle = rollAngle;
    packet.yawAngle = yawAngle;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a tk_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_attitude_send_struct(mavlink_channel_t chan, const mavlink_tk_attitude_t* tk_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_attitude_send(chan, tk_attitude->mode, tk_attitude->pitchAngle, tk_attitude->rollAngle, tk_attitude->yawAngle, tk_attitude->angular_velocity_x, tk_attitude->angular_velocity_y, tk_attitude->angular_velocity_z, tk_attitude->acceleration_x, tk_attitude->acceleration_y, tk_attitude->acceleration_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_ATTITUDE, (const char *)tk_attitude, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, float pitchAngle, float rollAngle, float yawAngle, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitchAngle);
    _mav_put_float(buf, 4, rollAngle);
    _mav_put_float(buf, 8, yawAngle);
    _mav_put_float(buf, 12, angular_velocity_x);
    _mav_put_float(buf, 16, angular_velocity_y);
    _mav_put_float(buf, 20, angular_velocity_z);
    _mav_put_float(buf, 24, acceleration_x);
    _mav_put_float(buf, 28, acceleration_y);
    _mav_put_float(buf, 32, acceleration_z);
    _mav_put_uint8_t(buf, 36, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_ATTITUDE, buf, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#else
    mavlink_tk_attitude_t *packet = (mavlink_tk_attitude_t *)msgbuf;
    packet->pitchAngle = pitchAngle;
    packet->rollAngle = rollAngle;
    packet->yawAngle = yawAngle;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->acceleration_x = acceleration_x;
    packet->acceleration_y = acceleration_y;
    packet->acceleration_z = acceleration_z;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_LEN, MAVLINK_MSG_ID_TK_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_ATTITUDE UNPACKING


/**
 * @brief Get field mode from tk_attitude message
 *
 * @return  Current mode.
 */
static inline uint8_t mavlink_msg_tk_attitude_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field pitchAngle from tk_attitude message
 *
 * @return [°] Pitch, unit degree.
 */
static inline float mavlink_msg_tk_attitude_get_pitchAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rollAngle from tk_attitude message
 *
 * @return [°] Roll, unit degree.
 */
static inline float mavlink_msg_tk_attitude_get_rollAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yawAngle from tk_attitude message
 *
 * @return [°] Yaw, unit degree.
 */
static inline float mavlink_msg_tk_attitude_get_yawAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field angular_velocity_x from tk_attitude message
 *
 * @return [°/s] Angular velocity X-axis, unit degree per second.
 */
static inline float mavlink_msg_tk_attitude_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field angular_velocity_y from tk_attitude message
 *
 * @return [°/s] Angular velocity Y-axis, unit degree per second.
 */
static inline float mavlink_msg_tk_attitude_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field angular_velocity_z from tk_attitude message
 *
 * @return [°/s] Angular velocity Z-axis, unit degree per second.
 */
static inline float mavlink_msg_tk_attitude_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field acceleration_x from tk_attitude message
 *
 * @return [m/s] Acceleration X-axis, unit m/s.
 */
static inline float mavlink_msg_tk_attitude_get_acceleration_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field acceleration_y from tk_attitude message
 *
 * @return [m/s] Acceleration Y-axis, unit m/s.
 */
static inline float mavlink_msg_tk_attitude_get_acceleration_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field acceleration_z from tk_attitude message
 *
 * @return [m/s] Acceleration Z-axis, unit m/s.
 */
static inline float mavlink_msg_tk_attitude_get_acceleration_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a tk_attitude message into a struct
 *
 * @param msg The message to decode
 * @param tk_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_attitude_decode(const mavlink_message_t* msg, mavlink_tk_attitude_t* tk_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_attitude->pitchAngle = mavlink_msg_tk_attitude_get_pitchAngle(msg);
    tk_attitude->rollAngle = mavlink_msg_tk_attitude_get_rollAngle(msg);
    tk_attitude->yawAngle = mavlink_msg_tk_attitude_get_yawAngle(msg);
    tk_attitude->angular_velocity_x = mavlink_msg_tk_attitude_get_angular_velocity_x(msg);
    tk_attitude->angular_velocity_y = mavlink_msg_tk_attitude_get_angular_velocity_y(msg);
    tk_attitude->angular_velocity_z = mavlink_msg_tk_attitude_get_angular_velocity_z(msg);
    tk_attitude->acceleration_x = mavlink_msg_tk_attitude_get_acceleration_x(msg);
    tk_attitude->acceleration_y = mavlink_msg_tk_attitude_get_acceleration_y(msg);
    tk_attitude->acceleration_z = mavlink_msg_tk_attitude_get_acceleration_z(msg);
    tk_attitude->mode = mavlink_msg_tk_attitude_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_TK_ATTITUDE_LEN;
        memset(tk_attitude, 0, MAVLINK_MSG_ID_TK_ATTITUDE_LEN);
    memcpy(tk_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
