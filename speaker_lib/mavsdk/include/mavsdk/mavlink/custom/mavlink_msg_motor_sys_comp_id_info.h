#pragma once
// MESSAGE MOTOR_SYS_COMP_ID_INFO PACKING

#define MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO 51508


typedef struct __mavlink_motor_sys_comp_id_info_t {
 uint8_t sysId; /*<  System ID.*/
 uint8_t compId; /*<  Component ID.*/
} mavlink_motor_sys_comp_id_info_t;

#define MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN 2
#define MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN 2
#define MAVLINK_MSG_ID_51508_LEN 2
#define MAVLINK_MSG_ID_51508_MIN_LEN 2

#define MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC 92
#define MAVLINK_MSG_ID_51508_CRC 92



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_SYS_COMP_ID_INFO { \
    51508, \
    "MOTOR_SYS_COMP_ID_INFO", \
    2, \
    {  { "sysId", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_motor_sys_comp_id_info_t, sysId) }, \
         { "compId", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_motor_sys_comp_id_info_t, compId) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_SYS_COMP_ID_INFO { \
    "MOTOR_SYS_COMP_ID_INFO", \
    2, \
    {  { "sysId", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_motor_sys_comp_id_info_t, sysId) }, \
         { "compId", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_motor_sys_comp_id_info_t, compId) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_sys_comp_id_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sysId  System ID.
 * @param compId  Component ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sysId, uint8_t compId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN];
    _mav_put_uint8_t(buf, 0, sysId);
    _mav_put_uint8_t(buf, 1, compId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#else
    mavlink_motor_sys_comp_id_info_t packet;
    packet.sysId = sysId;
    packet.compId = compId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
}

/**
 * @brief Pack a motor_sys_comp_id_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param sysId  System ID.
 * @param compId  Component ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t sysId, uint8_t compId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN];
    _mav_put_uint8_t(buf, 0, sysId);
    _mav_put_uint8_t(buf, 1, compId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#else
    mavlink_motor_sys_comp_id_info_t packet;
    packet.sysId = sysId;
    packet.compId = compId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#endif
}

/**
 * @brief Pack a motor_sys_comp_id_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sysId  System ID.
 * @param compId  Component ID.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sysId,uint8_t compId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN];
    _mav_put_uint8_t(buf, 0, sysId);
    _mav_put_uint8_t(buf, 1, compId);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#else
    mavlink_motor_sys_comp_id_info_t packet;
    packet.sysId = sysId;
    packet.compId = compId;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
}

/**
 * @brief Encode a motor_sys_comp_id_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_sys_comp_id_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_sys_comp_id_info_t* motor_sys_comp_id_info)
{
    return mavlink_msg_motor_sys_comp_id_info_pack(system_id, component_id, msg, motor_sys_comp_id_info->sysId, motor_sys_comp_id_info->compId);
}

/**
 * @brief Encode a motor_sys_comp_id_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_sys_comp_id_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_sys_comp_id_info_t* motor_sys_comp_id_info)
{
    return mavlink_msg_motor_sys_comp_id_info_pack_chan(system_id, component_id, chan, msg, motor_sys_comp_id_info->sysId, motor_sys_comp_id_info->compId);
}

/**
 * @brief Encode a motor_sys_comp_id_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_sys_comp_id_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_sys_comp_id_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_sys_comp_id_info_t* motor_sys_comp_id_info)
{
    return mavlink_msg_motor_sys_comp_id_info_pack_status(system_id, component_id, _status, msg,  motor_sys_comp_id_info->sysId, motor_sys_comp_id_info->compId);
}

/**
 * @brief Send a motor_sys_comp_id_info message
 * @param chan MAVLink channel to send the message
 *
 * @param sysId  System ID.
 * @param compId  Component ID.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_sys_comp_id_info_send(mavlink_channel_t chan, uint8_t sysId, uint8_t compId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN];
    _mav_put_uint8_t(buf, 0, sysId);
    _mav_put_uint8_t(buf, 1, compId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO, buf, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#else
    mavlink_motor_sys_comp_id_info_t packet;
    packet.sysId = sysId;
    packet.compId = compId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#endif
}

/**
 * @brief Send a motor_sys_comp_id_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_sys_comp_id_info_send_struct(mavlink_channel_t chan, const mavlink_motor_sys_comp_id_info_t* motor_sys_comp_id_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_sys_comp_id_info_send(chan, motor_sys_comp_id_info->sysId, motor_sys_comp_id_info->compId);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO, (const char *)motor_sys_comp_id_info, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_sys_comp_id_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sysId, uint8_t compId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, sysId);
    _mav_put_uint8_t(buf, 1, compId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO, buf, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#else
    mavlink_motor_sys_comp_id_info_t *packet = (mavlink_motor_sys_comp_id_info_t *)msgbuf;
    packet->sysId = sysId;
    packet->compId = compId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO, (const char *)packet, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_SYS_COMP_ID_INFO UNPACKING


/**
 * @brief Get field sysId from motor_sys_comp_id_info message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_motor_sys_comp_id_info_get_sysId(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field compId from motor_sys_comp_id_info message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_motor_sys_comp_id_info_get_compId(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a motor_sys_comp_id_info message into a struct
 *
 * @param msg The message to decode
 * @param motor_sys_comp_id_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_sys_comp_id_info_decode(const mavlink_message_t* msg, mavlink_motor_sys_comp_id_info_t* motor_sys_comp_id_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_sys_comp_id_info->sysId = mavlink_msg_motor_sys_comp_id_info_get_sysId(msg);
    motor_sys_comp_id_info->compId = mavlink_msg_motor_sys_comp_id_info_get_compId(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN;
        memset(motor_sys_comp_id_info, 0, MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_LEN);
    memcpy(motor_sys_comp_id_info, _MAV_PAYLOAD(msg), len);
#endif
}
