#pragma once
// MESSAGE MOTOR_PARAMETER_INFO PACKING

#define MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO 51503


typedef struct __mavlink_motor_parameter_info_t {
 float param1; /*<  param1.*/
 float param2; /*<  param2.*/
 float param3; /*<  param3.*/
 float param4; /*<  param4.*/
 float param5; /*<  param5.*/
 float param6; /*<  param6.*/
 uint8_t controlParamID; /*<   Parameter ID.*/
} mavlink_motor_parameter_info_t;

#define MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN 25
#define MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN 25
#define MAVLINK_MSG_ID_51503_LEN 25
#define MAVLINK_MSG_ID_51503_MIN_LEN 25

#define MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC 216
#define MAVLINK_MSG_ID_51503_CRC 216



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_PARAMETER_INFO { \
    51503, \
    "MOTOR_PARAMETER_INFO", \
    7, \
    {  { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_parameter_info_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_parameter_info_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_parameter_info_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_parameter_info_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_parameter_info_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_parameter_info_t, param6) }, \
         { "controlParamID", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_motor_parameter_info_t, controlParamID) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_PARAMETER_INFO { \
    "MOTOR_PARAMETER_INFO", \
    7, \
    {  { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_parameter_info_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_parameter_info_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_parameter_info_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_parameter_info_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_parameter_info_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_motor_parameter_info_t, param6) }, \
         { "controlParamID", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_motor_parameter_info_t, controlParamID) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_parameter_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @param param4  param4.
 * @param param5  param5.
 * @param param6  param6.
 * @param controlParamID   Parameter ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_parameter_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float param1, float param2, float param3, float param4, float param5, float param6, uint8_t controlParamID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_float(buf, 12, param4);
    _mav_put_float(buf, 16, param5);
    _mav_put_float(buf, 20, param6);
    _mav_put_uint8_t(buf, 24, controlParamID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#else
    mavlink_motor_parameter_info_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.controlParamID = controlParamID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
}

/**
 * @brief Pack a motor_parameter_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @param param4  param4.
 * @param param5  param5.
 * @param param6  param6.
 * @param controlParamID   Parameter ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_parameter_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float param1, float param2, float param3, float param4, float param5, float param6, uint8_t controlParamID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_float(buf, 12, param4);
    _mav_put_float(buf, 16, param5);
    _mav_put_float(buf, 20, param6);
    _mav_put_uint8_t(buf, 24, controlParamID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#else
    mavlink_motor_parameter_info_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.controlParamID = controlParamID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#endif
}

/**
 * @brief Pack a motor_parameter_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @param param4  param4.
 * @param param5  param5.
 * @param param6  param6.
 * @param controlParamID   Parameter ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_parameter_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float param1,float param2,float param3,float param4,float param5,float param6,uint8_t controlParamID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_float(buf, 12, param4);
    _mav_put_float(buf, 16, param5);
    _mav_put_float(buf, 20, param6);
    _mav_put_uint8_t(buf, 24, controlParamID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#else
    mavlink_motor_parameter_info_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.controlParamID = controlParamID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
}

/**
 * @brief Encode a motor_parameter_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_parameter_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_parameter_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_parameter_info_t* motor_parameter_info)
{
    return mavlink_msg_motor_parameter_info_pack(system_id, component_id, msg, motor_parameter_info->param1, motor_parameter_info->param2, motor_parameter_info->param3, motor_parameter_info->param4, motor_parameter_info->param5, motor_parameter_info->param6, motor_parameter_info->controlParamID);
}

/**
 * @brief Encode a motor_parameter_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_parameter_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_parameter_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_parameter_info_t* motor_parameter_info)
{
    return mavlink_msg_motor_parameter_info_pack_chan(system_id, component_id, chan, msg, motor_parameter_info->param1, motor_parameter_info->param2, motor_parameter_info->param3, motor_parameter_info->param4, motor_parameter_info->param5, motor_parameter_info->param6, motor_parameter_info->controlParamID);
}

/**
 * @brief Encode a motor_parameter_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_parameter_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_parameter_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_parameter_info_t* motor_parameter_info)
{
    return mavlink_msg_motor_parameter_info_pack_status(system_id, component_id, _status, msg,  motor_parameter_info->param1, motor_parameter_info->param2, motor_parameter_info->param3, motor_parameter_info->param4, motor_parameter_info->param5, motor_parameter_info->param6, motor_parameter_info->controlParamID);
}

/**
 * @brief Send a motor_parameter_info message
 * @param chan MAVLink channel to send the message
 *
 * @param param1  param1.
 * @param param2  param2.
 * @param param3  param3.
 * @param param4  param4.
 * @param param5  param5.
 * @param param6  param6.
 * @param controlParamID   Parameter ID.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_parameter_info_send(mavlink_channel_t chan, float param1, float param2, float param3, float param4, float param5, float param6, uint8_t controlParamID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN];
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_float(buf, 12, param4);
    _mav_put_float(buf, 16, param5);
    _mav_put_float(buf, 20, param6);
    _mav_put_uint8_t(buf, 24, controlParamID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO, buf, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#else
    mavlink_motor_parameter_info_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.controlParamID = controlParamID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#endif
}

/**
 * @brief Send a motor_parameter_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_parameter_info_send_struct(mavlink_channel_t chan, const mavlink_motor_parameter_info_t* motor_parameter_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_parameter_info_send(chan, motor_parameter_info->param1, motor_parameter_info->param2, motor_parameter_info->param3, motor_parameter_info->param4, motor_parameter_info->param5, motor_parameter_info->param6, motor_parameter_info->controlParamID);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO, (const char *)motor_parameter_info, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_parameter_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float param1, float param2, float param3, float param4, float param5, float param6, uint8_t controlParamID)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, param1);
    _mav_put_float(buf, 4, param2);
    _mav_put_float(buf, 8, param3);
    _mav_put_float(buf, 12, param4);
    _mav_put_float(buf, 16, param5);
    _mav_put_float(buf, 20, param6);
    _mav_put_uint8_t(buf, 24, controlParamID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO, buf, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#else
    mavlink_motor_parameter_info_t *packet = (mavlink_motor_parameter_info_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->controlParamID = controlParamID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO, (const char *)packet, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_PARAMETER_INFO UNPACKING


/**
 * @brief Get field param1 from motor_parameter_info message
 *
 * @return  param1.
 */
static inline float mavlink_msg_motor_parameter_info_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from motor_parameter_info message
 *
 * @return  param2.
 */
static inline float mavlink_msg_motor_parameter_info_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from motor_parameter_info message
 *
 * @return  param3.
 */
static inline float mavlink_msg_motor_parameter_info_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from motor_parameter_info message
 *
 * @return  param4.
 */
static inline float mavlink_msg_motor_parameter_info_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field param5 from motor_parameter_info message
 *
 * @return  param5.
 */
static inline float mavlink_msg_motor_parameter_info_get_param5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field param6 from motor_parameter_info message
 *
 * @return  param6.
 */
static inline float mavlink_msg_motor_parameter_info_get_param6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field controlParamID from motor_parameter_info message
 *
 * @return   Parameter ID.
 */
static inline uint8_t mavlink_msg_motor_parameter_info_get_controlParamID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a motor_parameter_info message into a struct
 *
 * @param msg The message to decode
 * @param motor_parameter_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_parameter_info_decode(const mavlink_message_t* msg, mavlink_motor_parameter_info_t* motor_parameter_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_parameter_info->param1 = mavlink_msg_motor_parameter_info_get_param1(msg);
    motor_parameter_info->param2 = mavlink_msg_motor_parameter_info_get_param2(msg);
    motor_parameter_info->param3 = mavlink_msg_motor_parameter_info_get_param3(msg);
    motor_parameter_info->param4 = mavlink_msg_motor_parameter_info_get_param4(msg);
    motor_parameter_info->param5 = mavlink_msg_motor_parameter_info_get_param5(msg);
    motor_parameter_info->param6 = mavlink_msg_motor_parameter_info_get_param6(msg);
    motor_parameter_info->controlParamID = mavlink_msg_motor_parameter_info_get_controlParamID(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN;
        memset(motor_parameter_info, 0, MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_LEN);
    memcpy(motor_parameter_info, _MAV_PAYLOAD(msg), len);
#endif
}
