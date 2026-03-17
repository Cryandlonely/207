#pragma once
// MESSAGE TK_INFRARED_TEMPERATURE PACKING

#define MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE 51008


typedef struct __mavlink_tk_infrared_temperature_t {
 float infrared_temperature_max; /*<   Maximum infrared temperature.*/
 float infrared_temperature_min; /*<   Minimum infrared temperature.*/
 float infrared_temperature_avg; /*<   Average infrared temperature.*/
} mavlink_tk_infrared_temperature_t;

#define MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN 12
#define MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN 12
#define MAVLINK_MSG_ID_51008_LEN 12
#define MAVLINK_MSG_ID_51008_MIN_LEN 12

#define MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC 253
#define MAVLINK_MSG_ID_51008_CRC 253



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_INFRARED_TEMPERATURE { \
    51008, \
    "TK_INFRARED_TEMPERATURE", \
    3, \
    {  { "infrared_temperature_max", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_max) }, \
         { "infrared_temperature_min", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_min) }, \
         { "infrared_temperature_avg", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_avg) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_INFRARED_TEMPERATURE { \
    "TK_INFRARED_TEMPERATURE", \
    3, \
    {  { "infrared_temperature_max", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_max) }, \
         { "infrared_temperature_min", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_min) }, \
         { "infrared_temperature_avg", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tk_infrared_temperature_t, infrared_temperature_avg) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_infrared_temperature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param infrared_temperature_max   Maximum infrared temperature.
 * @param infrared_temperature_min   Minimum infrared temperature.
 * @param infrared_temperature_avg   Average infrared temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float infrared_temperature_max, float infrared_temperature_min, float infrared_temperature_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN];
    _mav_put_float(buf, 0, infrared_temperature_max);
    _mav_put_float(buf, 4, infrared_temperature_min);
    _mav_put_float(buf, 8, infrared_temperature_avg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#else
    mavlink_tk_infrared_temperature_t packet;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
}

/**
 * @brief Pack a tk_infrared_temperature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param infrared_temperature_max   Maximum infrared temperature.
 * @param infrared_temperature_min   Minimum infrared temperature.
 * @param infrared_temperature_avg   Average infrared temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float infrared_temperature_max, float infrared_temperature_min, float infrared_temperature_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN];
    _mav_put_float(buf, 0, infrared_temperature_max);
    _mav_put_float(buf, 4, infrared_temperature_min);
    _mav_put_float(buf, 8, infrared_temperature_avg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#else
    mavlink_tk_infrared_temperature_t packet;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#endif
}

/**
 * @brief Pack a tk_infrared_temperature message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param infrared_temperature_max   Maximum infrared temperature.
 * @param infrared_temperature_min   Minimum infrared temperature.
 * @param infrared_temperature_avg   Average infrared temperature.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float infrared_temperature_max,float infrared_temperature_min,float infrared_temperature_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN];
    _mav_put_float(buf, 0, infrared_temperature_max);
    _mav_put_float(buf, 4, infrared_temperature_min);
    _mav_put_float(buf, 8, infrared_temperature_avg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#else
    mavlink_tk_infrared_temperature_t packet;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
}

/**
 * @brief Encode a tk_infrared_temperature struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_infrared_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_infrared_temperature_t* tk_infrared_temperature)
{
    return mavlink_msg_tk_infrared_temperature_pack(system_id, component_id, msg, tk_infrared_temperature->infrared_temperature_max, tk_infrared_temperature->infrared_temperature_min, tk_infrared_temperature->infrared_temperature_avg);
}

/**
 * @brief Encode a tk_infrared_temperature struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_infrared_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_infrared_temperature_t* tk_infrared_temperature)
{
    return mavlink_msg_tk_infrared_temperature_pack_chan(system_id, component_id, chan, msg, tk_infrared_temperature->infrared_temperature_max, tk_infrared_temperature->infrared_temperature_min, tk_infrared_temperature->infrared_temperature_avg);
}

/**
 * @brief Encode a tk_infrared_temperature struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_infrared_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_infrared_temperature_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_infrared_temperature_t* tk_infrared_temperature)
{
    return mavlink_msg_tk_infrared_temperature_pack_status(system_id, component_id, _status, msg,  tk_infrared_temperature->infrared_temperature_max, tk_infrared_temperature->infrared_temperature_min, tk_infrared_temperature->infrared_temperature_avg);
}

/**
 * @brief Send a tk_infrared_temperature message
 * @param chan MAVLink channel to send the message
 *
 * @param infrared_temperature_max   Maximum infrared temperature.
 * @param infrared_temperature_min   Minimum infrared temperature.
 * @param infrared_temperature_avg   Average infrared temperature.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_infrared_temperature_send(mavlink_channel_t chan, float infrared_temperature_max, float infrared_temperature_min, float infrared_temperature_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN];
    _mav_put_float(buf, 0, infrared_temperature_max);
    _mav_put_float(buf, 4, infrared_temperature_min);
    _mav_put_float(buf, 8, infrared_temperature_avg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE, buf, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#else
    mavlink_tk_infrared_temperature_t packet;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE, (const char *)&packet, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#endif
}

/**
 * @brief Send a tk_infrared_temperature message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_infrared_temperature_send_struct(mavlink_channel_t chan, const mavlink_tk_infrared_temperature_t* tk_infrared_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_infrared_temperature_send(chan, tk_infrared_temperature->infrared_temperature_max, tk_infrared_temperature->infrared_temperature_min, tk_infrared_temperature->infrared_temperature_avg);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE, (const char *)tk_infrared_temperature, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_infrared_temperature_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float infrared_temperature_max, float infrared_temperature_min, float infrared_temperature_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, infrared_temperature_max);
    _mav_put_float(buf, 4, infrared_temperature_min);
    _mav_put_float(buf, 8, infrared_temperature_avg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE, buf, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#else
    mavlink_tk_infrared_temperature_t *packet = (mavlink_tk_infrared_temperature_t *)msgbuf;
    packet->infrared_temperature_max = infrared_temperature_max;
    packet->infrared_temperature_min = infrared_temperature_min;
    packet->infrared_temperature_avg = infrared_temperature_avg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE, (const char *)packet, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_INFRARED_TEMPERATURE UNPACKING


/**
 * @brief Get field infrared_temperature_max from tk_infrared_temperature message
 *
 * @return   Maximum infrared temperature.
 */
static inline float mavlink_msg_tk_infrared_temperature_get_infrared_temperature_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field infrared_temperature_min from tk_infrared_temperature message
 *
 * @return   Minimum infrared temperature.
 */
static inline float mavlink_msg_tk_infrared_temperature_get_infrared_temperature_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field infrared_temperature_avg from tk_infrared_temperature message
 *
 * @return   Average infrared temperature.
 */
static inline float mavlink_msg_tk_infrared_temperature_get_infrared_temperature_avg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a tk_infrared_temperature message into a struct
 *
 * @param msg The message to decode
 * @param tk_infrared_temperature C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_infrared_temperature_decode(const mavlink_message_t* msg, mavlink_tk_infrared_temperature_t* tk_infrared_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_infrared_temperature->infrared_temperature_max = mavlink_msg_tk_infrared_temperature_get_infrared_temperature_max(msg);
    tk_infrared_temperature->infrared_temperature_min = mavlink_msg_tk_infrared_temperature_get_infrared_temperature_min(msg);
    tk_infrared_temperature->infrared_temperature_avg = mavlink_msg_tk_infrared_temperature_get_infrared_temperature_avg(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN? msg->len : MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN;
        memset(tk_infrared_temperature, 0, MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_LEN);
    memcpy(tk_infrared_temperature, _MAV_PAYLOAD(msg), len);
#endif
}
