#pragma once
// MESSAGE SPEAKER_AUDIO_INFO PACKING

#define MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO 20302


typedef struct __mavlink_speaker_audio_info_t {
 uint32_t index; /*<  Zero-based index of this audio since vehicle was armed.*/
 float length; /*<  The length of audio, (in seconds).*/
 char name[64]; /*<  The name of audio file.*/
 char format[16]; /*<  The format of audio file.*/
} mavlink_speaker_audio_info_t;

#define MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN 88
#define MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN 88
#define MAVLINK_MSG_ID_20302_LEN 88
#define MAVLINK_MSG_ID_20302_MIN_LEN 88

#define MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC 104
#define MAVLINK_MSG_ID_20302_CRC 104

#define MAVLINK_MSG_SPEAKER_AUDIO_INFO_FIELD_NAME_LEN 64
#define MAVLINK_MSG_SPEAKER_AUDIO_INFO_FIELD_FORMAT_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPEAKER_AUDIO_INFO { \
    20302, \
    "SPEAKER_AUDIO_INFO", \
    4, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 64, 8, offsetof(mavlink_speaker_audio_info_t, name) }, \
         { "index", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_speaker_audio_info_t, index) }, \
         { "format", NULL, MAVLINK_TYPE_CHAR, 16, 72, offsetof(mavlink_speaker_audio_info_t, format) }, \
         { "length", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speaker_audio_info_t, length) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPEAKER_AUDIO_INFO { \
    "SPEAKER_AUDIO_INFO", \
    4, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 64, 8, offsetof(mavlink_speaker_audio_info_t, name) }, \
         { "index", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_speaker_audio_info_t, index) }, \
         { "format", NULL, MAVLINK_TYPE_CHAR, 16, 72, offsetof(mavlink_speaker_audio_info_t, format) }, \
         { "length", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speaker_audio_info_t, length) }, \
         } \
}
#endif

/**
 * @brief Pack a speaker_audio_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @param format  The format of audio file.
 * @param length  The length of audio, (in seconds).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_audio_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *name, uint32_t index, const char *format, float length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_float(buf, 4, length);
    _mav_put_char_array(buf, 8, name, 64);
    _mav_put_char_array(buf, 72, format, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#else
    mavlink_speaker_audio_info_t packet;
    packet.index = index;
    packet.length = length;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
    mav_array_memcpy(packet.format, format, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
}

/**
 * @brief Pack a speaker_audio_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @param format  The format of audio file.
 * @param length  The length of audio, (in seconds).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_audio_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *name, uint32_t index, const char *format, float length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_float(buf, 4, length);
    _mav_put_char_array(buf, 8, name, 64);
    _mav_put_char_array(buf, 72, format, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#else
    mavlink_speaker_audio_info_t packet;
    packet.index = index;
    packet.length = length;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
    mav_array_memcpy(packet.format, format, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#endif
}

/**
 * @brief Pack a speaker_audio_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @param format  The format of audio file.
 * @param length  The length of audio, (in seconds).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_audio_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *name,uint32_t index,const char *format,float length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_float(buf, 4, length);
    _mav_put_char_array(buf, 8, name, 64);
    _mav_put_char_array(buf, 72, format, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#else
    mavlink_speaker_audio_info_t packet;
    packet.index = index;
    packet.length = length;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
    mav_array_memcpy(packet.format, format, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
}

/**
 * @brief Encode a speaker_audio_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param speaker_audio_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_audio_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_speaker_audio_info_t* speaker_audio_info)
{
    return mavlink_msg_speaker_audio_info_pack(system_id, component_id, msg, speaker_audio_info->name, speaker_audio_info->index, speaker_audio_info->format, speaker_audio_info->length);
}

/**
 * @brief Encode a speaker_audio_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speaker_audio_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_audio_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_speaker_audio_info_t* speaker_audio_info)
{
    return mavlink_msg_speaker_audio_info_pack_chan(system_id, component_id, chan, msg, speaker_audio_info->name, speaker_audio_info->index, speaker_audio_info->format, speaker_audio_info->length);
}

/**
 * @brief Encode a speaker_audio_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param speaker_audio_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_audio_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_speaker_audio_info_t* speaker_audio_info)
{
    return mavlink_msg_speaker_audio_info_pack_status(system_id, component_id, _status, msg,  speaker_audio_info->name, speaker_audio_info->index, speaker_audio_info->format, speaker_audio_info->length);
}

/**
 * @brief Send a speaker_audio_info message
 * @param chan MAVLink channel to send the message
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @param format  The format of audio file.
 * @param length  The length of audio, (in seconds).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_speaker_audio_info_send(mavlink_channel_t chan, const char *name, uint32_t index, const char *format, float length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_float(buf, 4, length);
    _mav_put_char_array(buf, 8, name, 64);
    _mav_put_char_array(buf, 72, format, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO, buf, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#else
    mavlink_speaker_audio_info_t packet;
    packet.index = index;
    packet.length = length;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
    mav_array_memcpy(packet.format, format, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO, (const char *)&packet, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#endif
}

/**
 * @brief Send a speaker_audio_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_speaker_audio_info_send_struct(mavlink_channel_t chan, const mavlink_speaker_audio_info_t* speaker_audio_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_speaker_audio_info_send(chan, speaker_audio_info->name, speaker_audio_info->index, speaker_audio_info->format, speaker_audio_info->length);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO, (const char *)speaker_audio_info, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_speaker_audio_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *name, uint32_t index, const char *format, float length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_float(buf, 4, length);
    _mav_put_char_array(buf, 8, name, 64);
    _mav_put_char_array(buf, 72, format, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO, buf, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#else
    mavlink_speaker_audio_info_t *packet = (mavlink_speaker_audio_info_t *)msgbuf;
    packet->index = index;
    packet->length = length;
    mav_array_memcpy(packet->name, name, sizeof(char)*64);
    mav_array_memcpy(packet->format, format, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO, (const char *)packet, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE SPEAKER_AUDIO_INFO UNPACKING


/**
 * @brief Get field name from speaker_audio_info message
 *
 * @return  The name of audio file.
 */
static inline uint16_t mavlink_msg_speaker_audio_info_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 64,  8);
}

/**
 * @brief Get field index from speaker_audio_info message
 *
 * @return  Zero-based index of this audio since vehicle was armed.
 */
static inline uint32_t mavlink_msg_speaker_audio_info_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field format from speaker_audio_info message
 *
 * @return  The format of audio file.
 */
static inline uint16_t mavlink_msg_speaker_audio_info_get_format(const mavlink_message_t* msg, char *format)
{
    return _MAV_RETURN_char_array(msg, format, 16,  72);
}

/**
 * @brief Get field length from speaker_audio_info message
 *
 * @return  The length of audio, (in seconds).
 */
static inline float mavlink_msg_speaker_audio_info_get_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a speaker_audio_info message into a struct
 *
 * @param msg The message to decode
 * @param speaker_audio_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_speaker_audio_info_decode(const mavlink_message_t* msg, mavlink_speaker_audio_info_t* speaker_audio_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    speaker_audio_info->index = mavlink_msg_speaker_audio_info_get_index(msg);
    speaker_audio_info->length = mavlink_msg_speaker_audio_info_get_length(msg);
    mavlink_msg_speaker_audio_info_get_name(msg, speaker_audio_info->name);
    mavlink_msg_speaker_audio_info_get_format(msg, speaker_audio_info->format);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN? msg->len : MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN;
        memset(speaker_audio_info, 0, MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_LEN);
    memcpy(speaker_audio_info, _MAV_PAYLOAD(msg), len);
#endif
}
