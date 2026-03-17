#pragma once
// MESSAGE MOTOR_MULTITURN_ANGLE_INFO PACKING

#define MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO 51506


typedef struct __mavlink_motor_multiturn_angle_info_t {
 float motorAngle; /*<  Angle position low. data type float ,units:0.01°/LSB */
} mavlink_motor_multiturn_angle_info_t;

#define MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN 4
#define MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN 4
#define MAVLINK_MSG_ID_51506_LEN 4
#define MAVLINK_MSG_ID_51506_MIN_LEN 4

#define MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC 112
#define MAVLINK_MSG_ID_51506_CRC 112



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_MULTITURN_ANGLE_INFO { \
    51506, \
    "MOTOR_MULTITURN_ANGLE_INFO", \
    1, \
    {  { "motorAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_multiturn_angle_info_t, motorAngle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_MULTITURN_ANGLE_INFO { \
    "MOTOR_MULTITURN_ANGLE_INFO", \
    1, \
    {  { "motorAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_multiturn_angle_info_t, motorAngle) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_multiturn_angle_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motorAngle  Angle position low. data type float ,units:0.01°/LSB 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float motorAngle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN];
    _mav_put_float(buf, 0, motorAngle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#else
    mavlink_motor_multiturn_angle_info_t packet;
    packet.motorAngle = motorAngle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
}

/**
 * @brief Pack a motor_multiturn_angle_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motorAngle  Angle position low. data type float ,units:0.01°/LSB 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float motorAngle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN];
    _mav_put_float(buf, 0, motorAngle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#else
    mavlink_motor_multiturn_angle_info_t packet;
    packet.motorAngle = motorAngle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#endif
}

/**
 * @brief Pack a motor_multiturn_angle_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motorAngle  Angle position low. data type float ,units:0.01°/LSB 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float motorAngle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN];
    _mav_put_float(buf, 0, motorAngle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#else
    mavlink_motor_multiturn_angle_info_t packet;
    packet.motorAngle = motorAngle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
}

/**
 * @brief Encode a motor_multiturn_angle_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_multiturn_angle_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_multiturn_angle_info_t* motor_multiturn_angle_info)
{
    return mavlink_msg_motor_multiturn_angle_info_pack(system_id, component_id, msg, motor_multiturn_angle_info->motorAngle);
}

/**
 * @brief Encode a motor_multiturn_angle_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_multiturn_angle_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_multiturn_angle_info_t* motor_multiturn_angle_info)
{
    return mavlink_msg_motor_multiturn_angle_info_pack_chan(system_id, component_id, chan, msg, motor_multiturn_angle_info->motorAngle);
}

/**
 * @brief Encode a motor_multiturn_angle_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_multiturn_angle_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_multiturn_angle_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_multiturn_angle_info_t* motor_multiturn_angle_info)
{
    return mavlink_msg_motor_multiturn_angle_info_pack_status(system_id, component_id, _status, msg,  motor_multiturn_angle_info->motorAngle);
}

/**
 * @brief Send a motor_multiturn_angle_info message
 * @param chan MAVLink channel to send the message
 *
 * @param motorAngle  Angle position low. data type float ,units:0.01°/LSB 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_multiturn_angle_info_send(mavlink_channel_t chan, float motorAngle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN];
    _mav_put_float(buf, 0, motorAngle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO, buf, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#else
    mavlink_motor_multiturn_angle_info_t packet;
    packet.motorAngle = motorAngle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#endif
}

/**
 * @brief Send a motor_multiturn_angle_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_multiturn_angle_info_send_struct(mavlink_channel_t chan, const mavlink_motor_multiturn_angle_info_t* motor_multiturn_angle_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_multiturn_angle_info_send(chan, motor_multiturn_angle_info->motorAngle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO, (const char *)motor_multiturn_angle_info, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_multiturn_angle_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float motorAngle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, motorAngle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO, buf, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#else
    mavlink_motor_multiturn_angle_info_t *packet = (mavlink_motor_multiturn_angle_info_t *)msgbuf;
    packet->motorAngle = motorAngle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO, (const char *)packet, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_MULTITURN_ANGLE_INFO UNPACKING


/**
 * @brief Get field motorAngle from motor_multiturn_angle_info message
 *
 * @return  Angle position low. data type float ,units:0.01°/LSB 
 */
static inline float mavlink_msg_motor_multiturn_angle_info_get_motorAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a motor_multiturn_angle_info message into a struct
 *
 * @param msg The message to decode
 * @param motor_multiturn_angle_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_multiturn_angle_info_decode(const mavlink_message_t* msg, mavlink_motor_multiturn_angle_info_t* motor_multiturn_angle_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_multiturn_angle_info->motorAngle = mavlink_msg_motor_multiturn_angle_info_get_motorAngle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN;
        memset(motor_multiturn_angle_info, 0, MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_LEN);
    memcpy(motor_multiturn_angle_info, _MAV_PAYLOAD(msg), len);
#endif
}
