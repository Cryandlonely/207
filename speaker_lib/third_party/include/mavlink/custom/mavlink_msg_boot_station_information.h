#pragma once
// MESSAGE BOOT_STATION_INFORMATION PACKING

#define MAVLINK_MSG_ID_BOOT_STATION_INFORMATION 51700


typedef struct __mavlink_boot_station_information_t {
 uint8_t station_type; /*<  station type.*/
} mavlink_boot_station_information_t;

#define MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN 1
#define MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN 1
#define MAVLINK_MSG_ID_51700_LEN 1
#define MAVLINK_MSG_ID_51700_MIN_LEN 1

#define MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC 193
#define MAVLINK_MSG_ID_51700_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BOOT_STATION_INFORMATION { \
    51700, \
    "BOOT_STATION_INFORMATION", \
    1, \
    {  { "station_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_boot_station_information_t, station_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BOOT_STATION_INFORMATION { \
    "BOOT_STATION_INFORMATION", \
    1, \
    {  { "station_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_boot_station_information_t, station_type) }, \
         } \
}
#endif

/**
 * @brief Pack a boot_station_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param station_type  station type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_station_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t station_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN];
    _mav_put_uint8_t(buf, 0, station_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#else
    mavlink_boot_station_information_t packet;
    packet.station_type = station_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT_STATION_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
}

/**
 * @brief Pack a boot_station_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param station_type  station type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_station_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t station_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN];
    _mav_put_uint8_t(buf, 0, station_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#else
    mavlink_boot_station_information_t packet;
    packet.station_type = station_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT_STATION_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a boot_station_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param station_type  station type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_station_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t station_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN];
    _mav_put_uint8_t(buf, 0, station_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#else
    mavlink_boot_station_information_t packet;
    packet.station_type = station_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT_STATION_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
}

/**
 * @brief Encode a boot_station_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boot_station_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_station_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boot_station_information_t* boot_station_information)
{
    return mavlink_msg_boot_station_information_pack(system_id, component_id, msg, boot_station_information->station_type);
}

/**
 * @brief Encode a boot_station_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot_station_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_station_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_boot_station_information_t* boot_station_information)
{
    return mavlink_msg_boot_station_information_pack_chan(system_id, component_id, chan, msg, boot_station_information->station_type);
}

/**
 * @brief Encode a boot_station_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param boot_station_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_station_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_boot_station_information_t* boot_station_information)
{
    return mavlink_msg_boot_station_information_pack_status(system_id, component_id, _status, msg,  boot_station_information->station_type);
}

/**
 * @brief Send a boot_station_information message
 * @param chan MAVLink channel to send the message
 *
 * @param station_type  station type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_boot_station_information_send(mavlink_channel_t chan, uint8_t station_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN];
    _mav_put_uint8_t(buf, 0, station_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION, buf, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#else
    mavlink_boot_station_information_t packet;
    packet.station_type = station_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a boot_station_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_boot_station_information_send_struct(mavlink_channel_t chan, const mavlink_boot_station_information_t* boot_station_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_boot_station_information_send(chan, boot_station_information->station_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION, (const char *)boot_station_information, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_boot_station_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t station_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, station_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION, buf, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#else
    mavlink_boot_station_information_t *packet = (mavlink_boot_station_information_t *)msgbuf;
    packet->station_type = station_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE BOOT_STATION_INFORMATION UNPACKING


/**
 * @brief Get field station_type from boot_station_information message
 *
 * @return  station type.
 */
static inline uint8_t mavlink_msg_boot_station_information_get_station_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a boot_station_information message into a struct
 *
 * @param msg The message to decode
 * @param boot_station_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_boot_station_information_decode(const mavlink_message_t* msg, mavlink_boot_station_information_t* boot_station_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    boot_station_information->station_type = mavlink_msg_boot_station_information_get_station_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN;
        memset(boot_station_information, 0, MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_LEN);
    memcpy(boot_station_information, _MAV_PAYLOAD(msg), len);
#endif
}
