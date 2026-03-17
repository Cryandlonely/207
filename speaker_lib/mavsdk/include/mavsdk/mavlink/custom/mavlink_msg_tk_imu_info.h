#pragma once
// MESSAGE TK_IMU_INFO PACKING

#define MAVLINK_MSG_ID_TK_IMU_INFO 51004


typedef struct __mavlink_tk_imu_info_t {
 float temperature; /*< [°c] Temperature.*/
 float angular_velocity_x; /*< [°/s] Angular velocity X-axis.*/
 float angular_velocity_y; /*< [°/s] Angular velocity Y-axis.*/
 float angular_velocity_z; /*< [°/s] Angular velocity Z-axis.*/
 float acceleration_x; /*< [m/s] Acceleration X-axis.*/
 float acceleration_y; /*< [m/s] Acceleration Y-axis.*/
 float acceleration_z; /*< [m/s] Acceleration Z-axis.*/
} mavlink_tk_imu_info_t;

#define MAVLINK_MSG_ID_TK_IMU_INFO_LEN 28
#define MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN 28
#define MAVLINK_MSG_ID_51004_LEN 28
#define MAVLINK_MSG_ID_51004_MIN_LEN 28

#define MAVLINK_MSG_ID_TK_IMU_INFO_CRC 146
#define MAVLINK_MSG_ID_51004_CRC 146



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_IMU_INFO { \
    51004, \
    "TK_IMU_INFO", \
    7, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_imu_info_t, temperature) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_imu_info_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_imu_info_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tk_imu_info_t, angular_velocity_z) }, \
         { "acceleration_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tk_imu_info_t, acceleration_x) }, \
         { "acceleration_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tk_imu_info_t, acceleration_y) }, \
         { "acceleration_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tk_imu_info_t, acceleration_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_IMU_INFO { \
    "TK_IMU_INFO", \
    7, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_imu_info_t, temperature) }, \
         { "angular_velocity_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_imu_info_t, angular_velocity_x) }, \
         { "angular_velocity_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_imu_info_t, angular_velocity_y) }, \
         { "angular_velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tk_imu_info_t, angular_velocity_z) }, \
         { "acceleration_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tk_imu_info_t, acceleration_x) }, \
         { "acceleration_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_tk_imu_info_t, acceleration_y) }, \
         { "acceleration_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_tk_imu_info_t, acceleration_z) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_imu_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature [°c] Temperature.
 * @param angular_velocity_x [°/s] Angular velocity X-axis.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis.
 * @param acceleration_x [m/s] Acceleration X-axis.
 * @param acceleration_y [m/s] Acceleration Y-axis.
 * @param acceleration_z [m/s] Acceleration Z-axis.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_imu_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temperature, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, angular_velocity_x);
    _mav_put_float(buf, 8, angular_velocity_y);
    _mav_put_float(buf, 12, angular_velocity_z);
    _mav_put_float(buf, 16, acceleration_x);
    _mav_put_float(buf, 20, acceleration_y);
    _mav_put_float(buf, 24, acceleration_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#else
    mavlink_tk_imu_info_t packet;
    packet.temperature = temperature;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_IMU_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
}

/**
 * @brief Pack a tk_imu_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature [°c] Temperature.
 * @param angular_velocity_x [°/s] Angular velocity X-axis.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis.
 * @param acceleration_x [m/s] Acceleration X-axis.
 * @param acceleration_y [m/s] Acceleration Y-axis.
 * @param acceleration_z [m/s] Acceleration Z-axis.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_imu_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float temperature, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, angular_velocity_x);
    _mav_put_float(buf, 8, angular_velocity_y);
    _mav_put_float(buf, 12, angular_velocity_z);
    _mav_put_float(buf, 16, acceleration_x);
    _mav_put_float(buf, 20, acceleration_y);
    _mav_put_float(buf, 24, acceleration_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#else
    mavlink_tk_imu_info_t packet;
    packet.temperature = temperature;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_IMU_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#endif
}

/**
 * @brief Pack a tk_imu_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature [°c] Temperature.
 * @param angular_velocity_x [°/s] Angular velocity X-axis.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis.
 * @param acceleration_x [m/s] Acceleration X-axis.
 * @param acceleration_y [m/s] Acceleration Y-axis.
 * @param acceleration_z [m/s] Acceleration Z-axis.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_imu_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temperature,float angular_velocity_x,float angular_velocity_y,float angular_velocity_z,float acceleration_x,float acceleration_y,float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, angular_velocity_x);
    _mav_put_float(buf, 8, angular_velocity_y);
    _mav_put_float(buf, 12, angular_velocity_z);
    _mav_put_float(buf, 16, acceleration_x);
    _mav_put_float(buf, 20, acceleration_y);
    _mav_put_float(buf, 24, acceleration_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#else
    mavlink_tk_imu_info_t packet;
    packet.temperature = temperature;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_IMU_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
}

/**
 * @brief Encode a tk_imu_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_imu_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_imu_info_t* tk_imu_info)
{
    return mavlink_msg_tk_imu_info_pack(system_id, component_id, msg, tk_imu_info->temperature, tk_imu_info->angular_velocity_x, tk_imu_info->angular_velocity_y, tk_imu_info->angular_velocity_z, tk_imu_info->acceleration_x, tk_imu_info->acceleration_y, tk_imu_info->acceleration_z);
}

/**
 * @brief Encode a tk_imu_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_imu_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_imu_info_t* tk_imu_info)
{
    return mavlink_msg_tk_imu_info_pack_chan(system_id, component_id, chan, msg, tk_imu_info->temperature, tk_imu_info->angular_velocity_x, tk_imu_info->angular_velocity_y, tk_imu_info->angular_velocity_z, tk_imu_info->acceleration_x, tk_imu_info->acceleration_y, tk_imu_info->acceleration_z);
}

/**
 * @brief Encode a tk_imu_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_imu_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_imu_info_t* tk_imu_info)
{
    return mavlink_msg_tk_imu_info_pack_status(system_id, component_id, _status, msg,  tk_imu_info->temperature, tk_imu_info->angular_velocity_x, tk_imu_info->angular_velocity_y, tk_imu_info->angular_velocity_z, tk_imu_info->acceleration_x, tk_imu_info->acceleration_y, tk_imu_info->acceleration_z);
}

/**
 * @brief Send a tk_imu_info message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature [°c] Temperature.
 * @param angular_velocity_x [°/s] Angular velocity X-axis.
 * @param angular_velocity_y [°/s] Angular velocity Y-axis.
 * @param angular_velocity_z [°/s] Angular velocity Z-axis.
 * @param acceleration_x [m/s] Acceleration X-axis.
 * @param acceleration_y [m/s] Acceleration Y-axis.
 * @param acceleration_z [m/s] Acceleration Z-axis.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_imu_info_send(mavlink_channel_t chan, float temperature, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, angular_velocity_x);
    _mav_put_float(buf, 8, angular_velocity_y);
    _mav_put_float(buf, 12, angular_velocity_z);
    _mav_put_float(buf, 16, acceleration_x);
    _mav_put_float(buf, 20, acceleration_y);
    _mav_put_float(buf, 24, acceleration_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_IMU_INFO, buf, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#else
    mavlink_tk_imu_info_t packet;
    packet.temperature = temperature;
    packet.angular_velocity_x = angular_velocity_x;
    packet.angular_velocity_y = angular_velocity_y;
    packet.angular_velocity_z = angular_velocity_z;
    packet.acceleration_x = acceleration_x;
    packet.acceleration_y = acceleration_y;
    packet.acceleration_z = acceleration_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_IMU_INFO, (const char *)&packet, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#endif
}

/**
 * @brief Send a tk_imu_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_imu_info_send_struct(mavlink_channel_t chan, const mavlink_tk_imu_info_t* tk_imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_imu_info_send(chan, tk_imu_info->temperature, tk_imu_info->angular_velocity_x, tk_imu_info->angular_velocity_y, tk_imu_info->angular_velocity_z, tk_imu_info->acceleration_x, tk_imu_info->acceleration_y, tk_imu_info->acceleration_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_IMU_INFO, (const char *)tk_imu_info, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_IMU_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_imu_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temperature, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, float acceleration_x, float acceleration_y, float acceleration_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature);
    _mav_put_float(buf, 4, angular_velocity_x);
    _mav_put_float(buf, 8, angular_velocity_y);
    _mav_put_float(buf, 12, angular_velocity_z);
    _mav_put_float(buf, 16, acceleration_x);
    _mav_put_float(buf, 20, acceleration_y);
    _mav_put_float(buf, 24, acceleration_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_IMU_INFO, buf, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#else
    mavlink_tk_imu_info_t *packet = (mavlink_tk_imu_info_t *)msgbuf;
    packet->temperature = temperature;
    packet->angular_velocity_x = angular_velocity_x;
    packet->angular_velocity_y = angular_velocity_y;
    packet->angular_velocity_z = angular_velocity_z;
    packet->acceleration_x = acceleration_x;
    packet->acceleration_y = acceleration_y;
    packet->acceleration_z = acceleration_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_IMU_INFO, (const char *)packet, MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_LEN, MAVLINK_MSG_ID_TK_IMU_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_IMU_INFO UNPACKING


/**
 * @brief Get field temperature from tk_imu_info message
 *
 * @return [°c] Temperature.
 */
static inline float mavlink_msg_tk_imu_info_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field angular_velocity_x from tk_imu_info message
 *
 * @return [°/s] Angular velocity X-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_angular_velocity_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field angular_velocity_y from tk_imu_info message
 *
 * @return [°/s] Angular velocity Y-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_angular_velocity_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field angular_velocity_z from tk_imu_info message
 *
 * @return [°/s] Angular velocity Z-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_angular_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field acceleration_x from tk_imu_info message
 *
 * @return [m/s] Acceleration X-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_acceleration_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field acceleration_y from tk_imu_info message
 *
 * @return [m/s] Acceleration Y-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_acceleration_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field acceleration_z from tk_imu_info message
 *
 * @return [m/s] Acceleration Z-axis.
 */
static inline float mavlink_msg_tk_imu_info_get_acceleration_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a tk_imu_info message into a struct
 *
 * @param msg The message to decode
 * @param tk_imu_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_imu_info_decode(const mavlink_message_t* msg, mavlink_tk_imu_info_t* tk_imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_imu_info->temperature = mavlink_msg_tk_imu_info_get_temperature(msg);
    tk_imu_info->angular_velocity_x = mavlink_msg_tk_imu_info_get_angular_velocity_x(msg);
    tk_imu_info->angular_velocity_y = mavlink_msg_tk_imu_info_get_angular_velocity_y(msg);
    tk_imu_info->angular_velocity_z = mavlink_msg_tk_imu_info_get_angular_velocity_z(msg);
    tk_imu_info->acceleration_x = mavlink_msg_tk_imu_info_get_acceleration_x(msg);
    tk_imu_info->acceleration_y = mavlink_msg_tk_imu_info_get_acceleration_y(msg);
    tk_imu_info->acceleration_z = mavlink_msg_tk_imu_info_get_acceleration_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_IMU_INFO_LEN? msg->len : MAVLINK_MSG_ID_TK_IMU_INFO_LEN;
        memset(tk_imu_info, 0, MAVLINK_MSG_ID_TK_IMU_INFO_LEN);
    memcpy(tk_imu_info, _MAV_PAYLOAD(msg), len);
#endif
}
