#pragma once
// MESSAGE TK_CUSTOM_DATA PACKING

#define MAVLINK_MSG_ID_TK_CUSTOM_DATA 51000


typedef struct __mavlink_tk_custom_data_t {
 uint16_t msg_id; /*<  The name of audio file.*/
 uint8_t len; /*<  Data length.*/
 char data[252]; /*<  Custom data area.*/
} mavlink_tk_custom_data_t;

#define MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN 255
#define MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN 255
#define MAVLINK_MSG_ID_51000_LEN 255
#define MAVLINK_MSG_ID_51000_MIN_LEN 255

#define MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC 160
#define MAVLINK_MSG_ID_51000_CRC 160

#define MAVLINK_MSG_TK_CUSTOM_DATA_FIELD_DATA_LEN 252

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_CUSTOM_DATA { \
    51000, \
    "TK_CUSTOM_DATA", \
    3, \
    {  { "msg_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_tk_custom_data_t, msg_id) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_tk_custom_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_CHAR, 252, 3, offsetof(mavlink_tk_custom_data_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_CUSTOM_DATA { \
    "TK_CUSTOM_DATA", \
    3, \
    {  { "msg_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_tk_custom_data_t, msg_id) }, \
         { "len", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_tk_custom_data_t, len) }, \
         { "data", NULL, MAVLINK_TYPE_CHAR, 252, 3, offsetof(mavlink_tk_custom_data_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_custom_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msg_id  The name of audio file.
 * @param len  Data length.
 * @param data  Custom data area.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_custom_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t msg_id, uint8_t len, const char *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN];
    _mav_put_uint16_t(buf, 0, msg_id);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_char_array(buf, 3, data, 252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#else
    mavlink_tk_custom_data_t packet;
    packet.msg_id = msg_id;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(char)*252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_CUSTOM_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
}

/**
 * @brief Pack a tk_custom_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param msg_id  The name of audio file.
 * @param len  Data length.
 * @param data  Custom data area.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_custom_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t msg_id, uint8_t len, const char *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN];
    _mav_put_uint16_t(buf, 0, msg_id);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_char_array(buf, 3, data, 252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#else
    mavlink_tk_custom_data_t packet;
    packet.msg_id = msg_id;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(char)*252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_CUSTOM_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#endif
}

/**
 * @brief Pack a tk_custom_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param msg_id  The name of audio file.
 * @param len  Data length.
 * @param data  Custom data area.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_custom_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t msg_id,uint8_t len,const char *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN];
    _mav_put_uint16_t(buf, 0, msg_id);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_char_array(buf, 3, data, 252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#else
    mavlink_tk_custom_data_t packet;
    packet.msg_id = msg_id;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(char)*252);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_CUSTOM_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
}

/**
 * @brief Encode a tk_custom_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_custom_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_custom_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_custom_data_t* tk_custom_data)
{
    return mavlink_msg_tk_custom_data_pack(system_id, component_id, msg, tk_custom_data->msg_id, tk_custom_data->len, tk_custom_data->data);
}

/**
 * @brief Encode a tk_custom_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_custom_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_custom_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_custom_data_t* tk_custom_data)
{
    return mavlink_msg_tk_custom_data_pack_chan(system_id, component_id, chan, msg, tk_custom_data->msg_id, tk_custom_data->len, tk_custom_data->data);
}

/**
 * @brief Encode a tk_custom_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_custom_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_custom_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_custom_data_t* tk_custom_data)
{
    return mavlink_msg_tk_custom_data_pack_status(system_id, component_id, _status, msg,  tk_custom_data->msg_id, tk_custom_data->len, tk_custom_data->data);
}

/**
 * @brief Send a tk_custom_data message
 * @param chan MAVLink channel to send the message
 *
 * @param msg_id  The name of audio file.
 * @param len  Data length.
 * @param data  Custom data area.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_custom_data_send(mavlink_channel_t chan, uint16_t msg_id, uint8_t len, const char *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN];
    _mav_put_uint16_t(buf, 0, msg_id);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_char_array(buf, 3, data, 252);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA, buf, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#else
    mavlink_tk_custom_data_t packet;
    packet.msg_id = msg_id;
    packet.len = len;
    mav_array_memcpy(packet.data, data, sizeof(char)*252);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA, (const char *)&packet, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#endif
}

/**
 * @brief Send a tk_custom_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_custom_data_send_struct(mavlink_channel_t chan, const mavlink_tk_custom_data_t* tk_custom_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_custom_data_send(chan, tk_custom_data->msg_id, tk_custom_data->len, tk_custom_data->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA, (const char *)tk_custom_data, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_custom_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t msg_id, uint8_t len, const char *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, msg_id);
    _mav_put_uint8_t(buf, 2, len);
    _mav_put_char_array(buf, 3, data, 252);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA, buf, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#else
    mavlink_tk_custom_data_t *packet = (mavlink_tk_custom_data_t *)msgbuf;
    packet->msg_id = msg_id;
    packet->len = len;
    mav_array_memcpy(packet->data, data, sizeof(char)*252);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_CUSTOM_DATA, (const char *)packet, MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN, MAVLINK_MSG_ID_TK_CUSTOM_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_CUSTOM_DATA UNPACKING


/**
 * @brief Get field msg_id from tk_custom_data message
 *
 * @return  The name of audio file.
 */
static inline uint16_t mavlink_msg_tk_custom_data_get_msg_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field len from tk_custom_data message
 *
 * @return  Data length.
 */
static inline uint8_t mavlink_msg_tk_custom_data_get_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field data from tk_custom_data message
 *
 * @return  Custom data area.
 */
static inline uint16_t mavlink_msg_tk_custom_data_get_data(const mavlink_message_t* msg, char *data)
{
    return _MAV_RETURN_char_array(msg, data, 252,  3);
}

/**
 * @brief Decode a tk_custom_data message into a struct
 *
 * @param msg The message to decode
 * @param tk_custom_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_custom_data_decode(const mavlink_message_t* msg, mavlink_tk_custom_data_t* tk_custom_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_custom_data->msg_id = mavlink_msg_tk_custom_data_get_msg_id(msg);
    tk_custom_data->len = mavlink_msg_tk_custom_data_get_len(msg);
    mavlink_msg_tk_custom_data_get_data(msg, tk_custom_data->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN? msg->len : MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN;
        memset(tk_custom_data, 0, MAVLINK_MSG_ID_TK_CUSTOM_DATA_LEN);
    memcpy(tk_custom_data, _MAV_PAYLOAD(msg), len);
#endif
}
