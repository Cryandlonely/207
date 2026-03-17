#pragma once
// MESSAGE SKYLINK_FILE_INFORMATION PACKING

#define MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION 20401


typedef struct __mavlink_skylink_file_information_t {
 uint32_t index; /*<  Zero-based index of this audio since vehicle was armed.*/
 char name[64]; /*<  The name of audio file.*/
} mavlink_skylink_file_information_t;

#define MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN 68
#define MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN 68
#define MAVLINK_MSG_ID_20401_LEN 68
#define MAVLINK_MSG_ID_20401_MIN_LEN 68

#define MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC 220
#define MAVLINK_MSG_ID_20401_CRC 220

#define MAVLINK_MSG_SKYLINK_FILE_INFORMATION_FIELD_NAME_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SKYLINK_FILE_INFORMATION { \
    20401, \
    "SKYLINK_FILE_INFORMATION", \
    2, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 64, 4, offsetof(mavlink_skylink_file_information_t, name) }, \
         { "index", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_skylink_file_information_t, index) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SKYLINK_FILE_INFORMATION { \
    "SKYLINK_FILE_INFORMATION", \
    2, \
    {  { "name", NULL, MAVLINK_TYPE_CHAR, 64, 4, offsetof(mavlink_skylink_file_information_t, name) }, \
         { "index", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_skylink_file_information_t, index) }, \
         } \
}
#endif

/**
 * @brief Pack a skylink_file_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_skylink_file_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *name, uint32_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_char_array(buf, 4, name, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#else
    mavlink_skylink_file_information_t packet;
    packet.index = index;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
}

/**
 * @brief Pack a skylink_file_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_skylink_file_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *name, uint32_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_char_array(buf, 4, name, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#else
    mavlink_skylink_file_information_t packet;
    packet.index = index;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a skylink_file_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_skylink_file_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *name,uint32_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_char_array(buf, 4, name, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#else
    mavlink_skylink_file_information_t packet;
    packet.index = index;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
}

/**
 * @brief Encode a skylink_file_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param skylink_file_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_skylink_file_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_skylink_file_information_t* skylink_file_information)
{
    return mavlink_msg_skylink_file_information_pack(system_id, component_id, msg, skylink_file_information->name, skylink_file_information->index);
}

/**
 * @brief Encode a skylink_file_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param skylink_file_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_skylink_file_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_skylink_file_information_t* skylink_file_information)
{
    return mavlink_msg_skylink_file_information_pack_chan(system_id, component_id, chan, msg, skylink_file_information->name, skylink_file_information->index);
}

/**
 * @brief Encode a skylink_file_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param skylink_file_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_skylink_file_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_skylink_file_information_t* skylink_file_information)
{
    return mavlink_msg_skylink_file_information_pack_status(system_id, component_id, _status, msg,  skylink_file_information->name, skylink_file_information->index);
}

/**
 * @brief Send a skylink_file_information message
 * @param chan MAVLink channel to send the message
 *
 * @param name  The name of audio file.
 * @param index  Zero-based index of this audio since vehicle was armed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_skylink_file_information_send(mavlink_channel_t chan, const char *name, uint32_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_char_array(buf, 4, name, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION, buf, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#else
    mavlink_skylink_file_information_t packet;
    packet.index = index;
    mav_array_memcpy(packet.name, name, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a skylink_file_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_skylink_file_information_send_struct(mavlink_channel_t chan, const mavlink_skylink_file_information_t* skylink_file_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_skylink_file_information_send(chan, skylink_file_information->name, skylink_file_information->index);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION, (const char *)skylink_file_information, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_skylink_file_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *name, uint32_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, index);
    _mav_put_char_array(buf, 4, name, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION, buf, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#else
    mavlink_skylink_file_information_t *packet = (mavlink_skylink_file_information_t *)msgbuf;
    packet->index = index;
    mav_array_memcpy(packet->name, name, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE SKYLINK_FILE_INFORMATION UNPACKING


/**
 * @brief Get field name from skylink_file_information message
 *
 * @return  The name of audio file.
 */
static inline uint16_t mavlink_msg_skylink_file_information_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 64,  4);
}

/**
 * @brief Get field index from skylink_file_information message
 *
 * @return  Zero-based index of this audio since vehicle was armed.
 */
static inline uint32_t mavlink_msg_skylink_file_information_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a skylink_file_information message into a struct
 *
 * @param msg The message to decode
 * @param skylink_file_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_skylink_file_information_decode(const mavlink_message_t* msg, mavlink_skylink_file_information_t* skylink_file_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    skylink_file_information->index = mavlink_msg_skylink_file_information_get_index(msg);
    mavlink_msg_skylink_file_information_get_name(msg, skylink_file_information->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN;
        memset(skylink_file_information, 0, MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_LEN);
    memcpy(skylink_file_information, _MAV_PAYLOAD(msg), len);
#endif
}
