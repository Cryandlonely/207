#pragma once
// MESSAGE TK_LOG_INFO PACKING

#define MAVLINK_MSG_ID_TK_LOG_INFO 51704


typedef struct __mavlink_tk_log_info_t {
 uint8_t type; /*<  Log type.*/
 uint8_t level; /*<  Log level.*/
 char message[128]; /*<  Log message.*/
} mavlink_tk_log_info_t;

#define MAVLINK_MSG_ID_TK_LOG_INFO_LEN 130
#define MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN 130
#define MAVLINK_MSG_ID_51704_LEN 130
#define MAVLINK_MSG_ID_51704_MIN_LEN 130

#define MAVLINK_MSG_ID_TK_LOG_INFO_CRC 33
#define MAVLINK_MSG_ID_51704_CRC 33

#define MAVLINK_MSG_TK_LOG_INFO_FIELD_MESSAGE_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_LOG_INFO { \
    51704, \
    "TK_LOG_INFO", \
    3, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_tk_log_info_t, type) }, \
         { "level", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_tk_log_info_t, level) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 128, 2, offsetof(mavlink_tk_log_info_t, message) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_LOG_INFO { \
    "TK_LOG_INFO", \
    3, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_tk_log_info_t, type) }, \
         { "level", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_tk_log_info_t, level) }, \
         { "message", NULL, MAVLINK_TYPE_CHAR, 128, 2, offsetof(mavlink_tk_log_info_t, message) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_log_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  Log type.
 * @param level  Log level.
 * @param message  Log message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_log_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t level, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_LOG_INFO_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, level);
    _mav_put_char_array(buf, 2, message, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#else
    mavlink_tk_log_info_t packet;
    packet.type = type;
    packet.level = level;
    mav_array_memcpy(packet.message, message, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_LOG_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
}

/**
 * @brief Pack a tk_log_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  Log type.
 * @param level  Log level.
 * @param message  Log message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_log_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t type, uint8_t level, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_LOG_INFO_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, level);
    _mav_put_char_array(buf, 2, message, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#else
    mavlink_tk_log_info_t packet;
    packet.type = type;
    packet.level = level;
    mav_array_memcpy(packet.message, message, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_LOG_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#endif
}

/**
 * @brief Pack a tk_log_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type  Log type.
 * @param level  Log level.
 * @param message  Log message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_log_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type,uint8_t level,const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_LOG_INFO_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, level);
    _mav_put_char_array(buf, 2, message, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#else
    mavlink_tk_log_info_t packet;
    packet.type = type;
    packet.level = level;
    mav_array_memcpy(packet.message, message, sizeof(char)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_LOG_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
}

/**
 * @brief Encode a tk_log_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_log_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_log_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_log_info_t* tk_log_info)
{
    return mavlink_msg_tk_log_info_pack(system_id, component_id, msg, tk_log_info->type, tk_log_info->level, tk_log_info->message);
}

/**
 * @brief Encode a tk_log_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_log_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_log_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_log_info_t* tk_log_info)
{
    return mavlink_msg_tk_log_info_pack_chan(system_id, component_id, chan, msg, tk_log_info->type, tk_log_info->level, tk_log_info->message);
}

/**
 * @brief Encode a tk_log_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_log_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_log_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_log_info_t* tk_log_info)
{
    return mavlink_msg_tk_log_info_pack_status(system_id, component_id, _status, msg,  tk_log_info->type, tk_log_info->level, tk_log_info->message);
}

/**
 * @brief Send a tk_log_info message
 * @param chan MAVLink channel to send the message
 *
 * @param type  Log type.
 * @param level  Log level.
 * @param message  Log message.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_log_info_send(mavlink_channel_t chan, uint8_t type, uint8_t level, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_LOG_INFO_LEN];
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, level);
    _mav_put_char_array(buf, 2, message, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_LOG_INFO, buf, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#else
    mavlink_tk_log_info_t packet;
    packet.type = type;
    packet.level = level;
    mav_array_memcpy(packet.message, message, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_LOG_INFO, (const char *)&packet, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#endif
}

/**
 * @brief Send a tk_log_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_log_info_send_struct(mavlink_channel_t chan, const mavlink_tk_log_info_t* tk_log_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_log_info_send(chan, tk_log_info->type, tk_log_info->level, tk_log_info->message);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_LOG_INFO, (const char *)tk_log_info, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_LOG_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_log_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t level, const char *message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, type);
    _mav_put_uint8_t(buf, 1, level);
    _mav_put_char_array(buf, 2, message, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_LOG_INFO, buf, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#else
    mavlink_tk_log_info_t *packet = (mavlink_tk_log_info_t *)msgbuf;
    packet->type = type;
    packet->level = level;
    mav_array_memcpy(packet->message, message, sizeof(char)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_LOG_INFO, (const char *)packet, MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_LEN, MAVLINK_MSG_ID_TK_LOG_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_LOG_INFO UNPACKING


/**
 * @brief Get field type from tk_log_info message
 *
 * @return  Log type.
 */
static inline uint8_t mavlink_msg_tk_log_info_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field level from tk_log_info message
 *
 * @return  Log level.
 */
static inline uint8_t mavlink_msg_tk_log_info_get_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field message from tk_log_info message
 *
 * @return  Log message.
 */
static inline uint16_t mavlink_msg_tk_log_info_get_message(const mavlink_message_t* msg, char *message)
{
    return _MAV_RETURN_char_array(msg, message, 128,  2);
}

/**
 * @brief Decode a tk_log_info message into a struct
 *
 * @param msg The message to decode
 * @param tk_log_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_log_info_decode(const mavlink_message_t* msg, mavlink_tk_log_info_t* tk_log_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_log_info->type = mavlink_msg_tk_log_info_get_type(msg);
    tk_log_info->level = mavlink_msg_tk_log_info_get_level(msg);
    mavlink_msg_tk_log_info_get_message(msg, tk_log_info->message);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_LOG_INFO_LEN? msg->len : MAVLINK_MSG_ID_TK_LOG_INFO_LEN;
        memset(tk_log_info, 0, MAVLINK_MSG_ID_TK_LOG_INFO_LEN);
    memcpy(tk_log_info, _MAV_PAYLOAD(msg), len);
#endif
}
