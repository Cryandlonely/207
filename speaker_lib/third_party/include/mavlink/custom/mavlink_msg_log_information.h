#pragma once
// MESSAGE LOG_INFORMATION PACKING

#define MAVLINK_MSG_ID_LOG_INFORMATION 20001


typedef struct __mavlink_log_information_t {
 int32_t log_level; /*<  log level*/
 char text[200]; /*<  information text*/
} mavlink_log_information_t;

#define MAVLINK_MSG_ID_LOG_INFORMATION_LEN 204
#define MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN 204
#define MAVLINK_MSG_ID_20001_LEN 204
#define MAVLINK_MSG_ID_20001_MIN_LEN 204

#define MAVLINK_MSG_ID_LOG_INFORMATION_CRC 167
#define MAVLINK_MSG_ID_20001_CRC 167

#define MAVLINK_MSG_LOG_INFORMATION_FIELD_TEXT_LEN 200

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOG_INFORMATION { \
    20001, \
    "LOG_INFORMATION", \
    2, \
    {  { "text", NULL, MAVLINK_TYPE_CHAR, 200, 4, offsetof(mavlink_log_information_t, text) }, \
         { "log_level", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_log_information_t, log_level) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOG_INFORMATION { \
    "LOG_INFORMATION", \
    2, \
    {  { "text", NULL, MAVLINK_TYPE_CHAR, 200, 4, offsetof(mavlink_log_information_t, text) }, \
         { "log_level", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_log_information_t, log_level) }, \
         } \
}
#endif

/**
 * @brief Pack a log_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param text  information text
 * @param log_level  log level
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *text, int32_t log_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, log_level);
    _mav_put_char_array(buf, 4, text, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#else
    mavlink_log_information_t packet;
    packet.log_level = log_level;
    mav_array_memcpy(packet.text, text, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOG_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
}

/**
 * @brief Pack a log_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param text  information text
 * @param log_level  log level
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *text, int32_t log_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, log_level);
    _mav_put_char_array(buf, 4, text, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#else
    mavlink_log_information_t packet;
    packet.log_level = log_level;
    mav_array_memcpy(packet.text, text, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOG_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a log_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param text  information text
 * @param log_level  log level
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *text,int32_t log_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, log_level);
    _mav_put_char_array(buf, 4, text, 200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#else
    mavlink_log_information_t packet;
    packet.log_level = log_level;
    mav_array_memcpy(packet.text, text, sizeof(char)*200);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOG_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
}

/**
 * @brief Encode a log_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param log_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_log_information_t* log_information)
{
    return mavlink_msg_log_information_pack(system_id, component_id, msg, log_information->text, log_information->log_level);
}

/**
 * @brief Encode a log_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param log_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_log_information_t* log_information)
{
    return mavlink_msg_log_information_pack_chan(system_id, component_id, chan, msg, log_information->text, log_information->log_level);
}

/**
 * @brief Encode a log_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param log_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_log_information_t* log_information)
{
    return mavlink_msg_log_information_pack_status(system_id, component_id, _status, msg,  log_information->text, log_information->log_level);
}

/**
 * @brief Send a log_information message
 * @param chan MAVLink channel to send the message
 *
 * @param text  information text
 * @param log_level  log level
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_log_information_send(mavlink_channel_t chan, const char *text, int32_t log_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, log_level);
    _mav_put_char_array(buf, 4, text, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_INFORMATION, buf, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#else
    mavlink_log_information_t packet;
    packet.log_level = log_level;
    mav_array_memcpy(packet.text, text, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a log_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_log_information_send_struct(mavlink_channel_t chan, const mavlink_log_information_t* log_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_log_information_send(chan, log_information->text, log_information->log_level);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_INFORMATION, (const char *)log_information, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOG_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_log_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *text, int32_t log_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, log_level);
    _mav_put_char_array(buf, 4, text, 200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_INFORMATION, buf, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#else
    mavlink_log_information_t *packet = (mavlink_log_information_t *)msgbuf;
    packet->log_level = log_level;
    mav_array_memcpy(packet->text, text, sizeof(char)*200);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_LEN, MAVLINK_MSG_ID_LOG_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE LOG_INFORMATION UNPACKING


/**
 * @brief Get field text from log_information message
 *
 * @return  information text
 */
static inline uint16_t mavlink_msg_log_information_get_text(const mavlink_message_t* msg, char *text)
{
    return _MAV_RETURN_char_array(msg, text, 200,  4);
}

/**
 * @brief Get field log_level from log_information message
 *
 * @return  log level
 */
static inline int32_t mavlink_msg_log_information_get_log_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a log_information message into a struct
 *
 * @param msg The message to decode
 * @param log_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_log_information_decode(const mavlink_message_t* msg, mavlink_log_information_t* log_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    log_information->log_level = mavlink_msg_log_information_get_log_level(msg);
    mavlink_msg_log_information_get_text(msg, log_information->text);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOG_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_LOG_INFORMATION_LEN;
        memset(log_information, 0, MAVLINK_MSG_ID_LOG_INFORMATION_LEN);
    memcpy(log_information, _MAV_PAYLOAD(msg), len);
#endif
}
