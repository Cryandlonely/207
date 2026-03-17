#pragma once
// MESSAGE LIGHT_STATUS PACKING

#define MAVLINK_MSG_ID_LIGHT_STATUS 20502


typedef struct __mavlink_light_status_t {
 uint32_t id; /*<  The light id.*/
 float brightness; /*<  Brightness of light.*/
 uint8_t on_off; /*<  On/off of light.*/
 uint8_t mode; /*<  Mode of light.*/
} mavlink_light_status_t;

#define MAVLINK_MSG_ID_LIGHT_STATUS_LEN 10
#define MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_20502_LEN 10
#define MAVLINK_MSG_ID_20502_MIN_LEN 10

#define MAVLINK_MSG_ID_LIGHT_STATUS_CRC 139
#define MAVLINK_MSG_ID_20502_CRC 139



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LIGHT_STATUS { \
    20502, \
    "LIGHT_STATUS", \
    4, \
    {  { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_light_status_t, id) }, \
         { "on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_light_status_t, on_off) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_light_status_t, mode) }, \
         { "brightness", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_light_status_t, brightness) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LIGHT_STATUS { \
    "LIGHT_STATUS", \
    4, \
    {  { "id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_light_status_t, id) }, \
         { "on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_light_status_t, on_off) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_light_status_t, mode) }, \
         { "brightness", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_light_status_t, brightness) }, \
         } \
}
#endif

/**
 * @brief Pack a light_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  The light id.
 * @param on_off  On/off of light.
 * @param mode  Mode of light.
 * @param brightness  Brightness of light.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t id, uint8_t on_off, uint8_t mode, float brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_float(buf, 4, brightness);
    _mav_put_uint8_t(buf, 8, on_off);
    _mav_put_uint8_t(buf, 9, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#else
    mavlink_light_status_t packet;
    packet.id = id;
    packet.brightness = brightness;
    packet.on_off = on_off;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
}

/**
 * @brief Pack a light_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  The light id.
 * @param on_off  On/off of light.
 * @param mode  Mode of light.
 * @param brightness  Brightness of light.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t id, uint8_t on_off, uint8_t mode, float brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_float(buf, 4, brightness);
    _mav_put_uint8_t(buf, 8, on_off);
    _mav_put_uint8_t(buf, 9, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#else
    mavlink_light_status_t packet;
    packet.id = id;
    packet.brightness = brightness;
    packet.on_off = on_off;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a light_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  The light id.
 * @param on_off  On/off of light.
 * @param mode  Mode of light.
 * @param brightness  Brightness of light.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t id,uint8_t on_off,uint8_t mode,float brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_float(buf, 4, brightness);
    _mav_put_uint8_t(buf, 8, on_off);
    _mav_put_uint8_t(buf, 9, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#else
    mavlink_light_status_t packet;
    packet.id = id;
    packet.brightness = brightness;
    packet.on_off = on_off;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
}

/**
 * @brief Encode a light_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param light_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_light_status_t* light_status)
{
    return mavlink_msg_light_status_pack(system_id, component_id, msg, light_status->id, light_status->on_off, light_status->mode, light_status->brightness);
}

/**
 * @brief Encode a light_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param light_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_light_status_t* light_status)
{
    return mavlink_msg_light_status_pack_chan(system_id, component_id, chan, msg, light_status->id, light_status->on_off, light_status->mode, light_status->brightness);
}

/**
 * @brief Encode a light_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param light_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_light_status_t* light_status)
{
    return mavlink_msg_light_status_pack_status(system_id, component_id, _status, msg,  light_status->id, light_status->on_off, light_status->mode, light_status->brightness);
}

/**
 * @brief Send a light_status message
 * @param chan MAVLink channel to send the message
 *
 * @param id  The light id.
 * @param on_off  On/off of light.
 * @param mode  Mode of light.
 * @param brightness  Brightness of light.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_light_status_send(mavlink_channel_t chan, uint32_t id, uint8_t on_off, uint8_t mode, float brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_float(buf, 4, brightness);
    _mav_put_uint8_t(buf, 8, on_off);
    _mav_put_uint8_t(buf, 9, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_STATUS, buf, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#else
    mavlink_light_status_t packet;
    packet.id = id;
    packet.brightness = brightness;
    packet.on_off = on_off;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#endif
}

/**
 * @brief Send a light_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_light_status_send_struct(mavlink_channel_t chan, const mavlink_light_status_t* light_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_light_status_send(chan, light_status->id, light_status->on_off, light_status->mode, light_status->brightness);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_STATUS, (const char *)light_status, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_LIGHT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_light_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t id, uint8_t on_off, uint8_t mode, float brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, id);
    _mav_put_float(buf, 4, brightness);
    _mav_put_uint8_t(buf, 8, on_off);
    _mav_put_uint8_t(buf, 9, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_STATUS, buf, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#else
    mavlink_light_status_t *packet = (mavlink_light_status_t *)msgbuf;
    packet->id = id;
    packet->brightness = brightness;
    packet->on_off = on_off;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_STATUS, (const char *)packet, MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_LEN, MAVLINK_MSG_ID_LIGHT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE LIGHT_STATUS UNPACKING


/**
 * @brief Get field id from light_status message
 *
 * @return  The light id.
 */
static inline uint32_t mavlink_msg_light_status_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field on_off from light_status message
 *
 * @return  On/off of light.
 */
static inline uint8_t mavlink_msg_light_status_get_on_off(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field mode from light_status message
 *
 * @return  Mode of light.
 */
static inline uint8_t mavlink_msg_light_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field brightness from light_status message
 *
 * @return  Brightness of light.
 */
static inline float mavlink_msg_light_status_get_brightness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a light_status message into a struct
 *
 * @param msg The message to decode
 * @param light_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_light_status_decode(const mavlink_message_t* msg, mavlink_light_status_t* light_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    light_status->id = mavlink_msg_light_status_get_id(msg);
    light_status->brightness = mavlink_msg_light_status_get_brightness(msg);
    light_status->on_off = mavlink_msg_light_status_get_on_off(msg);
    light_status->mode = mavlink_msg_light_status_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LIGHT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_LIGHT_STATUS_LEN;
        memset(light_status, 0, MAVLINK_MSG_ID_LIGHT_STATUS_LEN);
    memcpy(light_status, _MAV_PAYLOAD(msg), len);
#endif
}
