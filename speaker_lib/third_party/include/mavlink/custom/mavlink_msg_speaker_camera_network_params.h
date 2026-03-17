#pragma once
// MESSAGE SPEAKER_CAMERA_NETWORK_PARAMS PACKING

#define MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS 20304


typedef struct __mavlink_speaker_camera_network_params_t {
 uint8_t ip_byte_0; /*<  IP byte 0*/
 uint8_t ip_byte_1; /*<  IP byte 1*/
 uint8_t ip_byte_2; /*<  IP byte 2*/
 uint8_t ip_byte_3; /*<  IP byte 3*/
 uint8_t bitrate; /*<  Bitrate*/
 uint8_t resolution; /*<  Resolution*/
 uint8_t encoding; /*<  Encoding*/
} mavlink_speaker_camera_network_params_t;

#define MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN 7
#define MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN 7
#define MAVLINK_MSG_ID_20304_LEN 7
#define MAVLINK_MSG_ID_20304_MIN_LEN 7

#define MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC 38
#define MAVLINK_MSG_ID_20304_CRC 38



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPEAKER_CAMERA_NETWORK_PARAMS { \
    20304, \
    "SPEAKER_CAMERA_NETWORK_PARAMS", \
    7, \
    {  { "ip_byte_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_0) }, \
         { "ip_byte_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_1) }, \
         { "ip_byte_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_2) }, \
         { "ip_byte_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_3) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_speaker_camera_network_params_t, bitrate) }, \
         { "resolution", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_speaker_camera_network_params_t, resolution) }, \
         { "encoding", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_speaker_camera_network_params_t, encoding) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPEAKER_CAMERA_NETWORK_PARAMS { \
    "SPEAKER_CAMERA_NETWORK_PARAMS", \
    7, \
    {  { "ip_byte_0", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_0) }, \
         { "ip_byte_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_1) }, \
         { "ip_byte_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_2) }, \
         { "ip_byte_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_speaker_camera_network_params_t, ip_byte_3) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_speaker_camera_network_params_t, bitrate) }, \
         { "resolution", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_speaker_camera_network_params_t, resolution) }, \
         { "encoding", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_speaker_camera_network_params_t, encoding) }, \
         } \
}
#endif

/**
 * @brief Pack a speaker_camera_network_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ip_byte_0  IP byte 0
 * @param ip_byte_1  IP byte 1
 * @param ip_byte_2  IP byte 2
 * @param ip_byte_3  IP byte 3
 * @param bitrate  Bitrate
 * @param resolution  Resolution
 * @param encoding  Encoding
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t ip_byte_0, uint8_t ip_byte_1, uint8_t ip_byte_2, uint8_t ip_byte_3, uint8_t bitrate, uint8_t resolution, uint8_t encoding)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN];
    _mav_put_uint8_t(buf, 0, ip_byte_0);
    _mav_put_uint8_t(buf, 1, ip_byte_1);
    _mav_put_uint8_t(buf, 2, ip_byte_2);
    _mav_put_uint8_t(buf, 3, ip_byte_3);
    _mav_put_uint8_t(buf, 4, bitrate);
    _mav_put_uint8_t(buf, 5, resolution);
    _mav_put_uint8_t(buf, 6, encoding);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#else
    mavlink_speaker_camera_network_params_t packet;
    packet.ip_byte_0 = ip_byte_0;
    packet.ip_byte_1 = ip_byte_1;
    packet.ip_byte_2 = ip_byte_2;
    packet.ip_byte_3 = ip_byte_3;
    packet.bitrate = bitrate;
    packet.resolution = resolution;
    packet.encoding = encoding;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
}

/**
 * @brief Pack a speaker_camera_network_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ip_byte_0  IP byte 0
 * @param ip_byte_1  IP byte 1
 * @param ip_byte_2  IP byte 2
 * @param ip_byte_3  IP byte 3
 * @param bitrate  Bitrate
 * @param resolution  Resolution
 * @param encoding  Encoding
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t ip_byte_0, uint8_t ip_byte_1, uint8_t ip_byte_2, uint8_t ip_byte_3, uint8_t bitrate, uint8_t resolution, uint8_t encoding)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN];
    _mav_put_uint8_t(buf, 0, ip_byte_0);
    _mav_put_uint8_t(buf, 1, ip_byte_1);
    _mav_put_uint8_t(buf, 2, ip_byte_2);
    _mav_put_uint8_t(buf, 3, ip_byte_3);
    _mav_put_uint8_t(buf, 4, bitrate);
    _mav_put_uint8_t(buf, 5, resolution);
    _mav_put_uint8_t(buf, 6, encoding);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#else
    mavlink_speaker_camera_network_params_t packet;
    packet.ip_byte_0 = ip_byte_0;
    packet.ip_byte_1 = ip_byte_1;
    packet.ip_byte_2 = ip_byte_2;
    packet.ip_byte_3 = ip_byte_3;
    packet.bitrate = bitrate;
    packet.resolution = resolution;
    packet.encoding = encoding;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#endif
}

/**
 * @brief Pack a speaker_camera_network_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ip_byte_0  IP byte 0
 * @param ip_byte_1  IP byte 1
 * @param ip_byte_2  IP byte 2
 * @param ip_byte_3  IP byte 3
 * @param bitrate  Bitrate
 * @param resolution  Resolution
 * @param encoding  Encoding
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t ip_byte_0,uint8_t ip_byte_1,uint8_t ip_byte_2,uint8_t ip_byte_3,uint8_t bitrate,uint8_t resolution,uint8_t encoding)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN];
    _mav_put_uint8_t(buf, 0, ip_byte_0);
    _mav_put_uint8_t(buf, 1, ip_byte_1);
    _mav_put_uint8_t(buf, 2, ip_byte_2);
    _mav_put_uint8_t(buf, 3, ip_byte_3);
    _mav_put_uint8_t(buf, 4, bitrate);
    _mav_put_uint8_t(buf, 5, resolution);
    _mav_put_uint8_t(buf, 6, encoding);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#else
    mavlink_speaker_camera_network_params_t packet;
    packet.ip_byte_0 = ip_byte_0;
    packet.ip_byte_1 = ip_byte_1;
    packet.ip_byte_2 = ip_byte_2;
    packet.ip_byte_3 = ip_byte_3;
    packet.bitrate = bitrate;
    packet.resolution = resolution;
    packet.encoding = encoding;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
}

/**
 * @brief Encode a speaker_camera_network_params struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param speaker_camera_network_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_speaker_camera_network_params_t* speaker_camera_network_params)
{
    return mavlink_msg_speaker_camera_network_params_pack(system_id, component_id, msg, speaker_camera_network_params->ip_byte_0, speaker_camera_network_params->ip_byte_1, speaker_camera_network_params->ip_byte_2, speaker_camera_network_params->ip_byte_3, speaker_camera_network_params->bitrate, speaker_camera_network_params->resolution, speaker_camera_network_params->encoding);
}

/**
 * @brief Encode a speaker_camera_network_params struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speaker_camera_network_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_speaker_camera_network_params_t* speaker_camera_network_params)
{
    return mavlink_msg_speaker_camera_network_params_pack_chan(system_id, component_id, chan, msg, speaker_camera_network_params->ip_byte_0, speaker_camera_network_params->ip_byte_1, speaker_camera_network_params->ip_byte_2, speaker_camera_network_params->ip_byte_3, speaker_camera_network_params->bitrate, speaker_camera_network_params->resolution, speaker_camera_network_params->encoding);
}

/**
 * @brief Encode a speaker_camera_network_params struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param speaker_camera_network_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_camera_network_params_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_speaker_camera_network_params_t* speaker_camera_network_params)
{
    return mavlink_msg_speaker_camera_network_params_pack_status(system_id, component_id, _status, msg,  speaker_camera_network_params->ip_byte_0, speaker_camera_network_params->ip_byte_1, speaker_camera_network_params->ip_byte_2, speaker_camera_network_params->ip_byte_3, speaker_camera_network_params->bitrate, speaker_camera_network_params->resolution, speaker_camera_network_params->encoding);
}

/**
 * @brief Send a speaker_camera_network_params message
 * @param chan MAVLink channel to send the message
 *
 * @param ip_byte_0  IP byte 0
 * @param ip_byte_1  IP byte 1
 * @param ip_byte_2  IP byte 2
 * @param ip_byte_3  IP byte 3
 * @param bitrate  Bitrate
 * @param resolution  Resolution
 * @param encoding  Encoding
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_speaker_camera_network_params_send(mavlink_channel_t chan, uint8_t ip_byte_0, uint8_t ip_byte_1, uint8_t ip_byte_2, uint8_t ip_byte_3, uint8_t bitrate, uint8_t resolution, uint8_t encoding)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN];
    _mav_put_uint8_t(buf, 0, ip_byte_0);
    _mav_put_uint8_t(buf, 1, ip_byte_1);
    _mav_put_uint8_t(buf, 2, ip_byte_2);
    _mav_put_uint8_t(buf, 3, ip_byte_3);
    _mav_put_uint8_t(buf, 4, bitrate);
    _mav_put_uint8_t(buf, 5, resolution);
    _mav_put_uint8_t(buf, 6, encoding);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS, buf, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#else
    mavlink_speaker_camera_network_params_t packet;
    packet.ip_byte_0 = ip_byte_0;
    packet.ip_byte_1 = ip_byte_1;
    packet.ip_byte_2 = ip_byte_2;
    packet.ip_byte_3 = ip_byte_3;
    packet.bitrate = bitrate;
    packet.resolution = resolution;
    packet.encoding = encoding;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#endif
}

/**
 * @brief Send a speaker_camera_network_params message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_speaker_camera_network_params_send_struct(mavlink_channel_t chan, const mavlink_speaker_camera_network_params_t* speaker_camera_network_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_speaker_camera_network_params_send(chan, speaker_camera_network_params->ip_byte_0, speaker_camera_network_params->ip_byte_1, speaker_camera_network_params->ip_byte_2, speaker_camera_network_params->ip_byte_3, speaker_camera_network_params->bitrate, speaker_camera_network_params->resolution, speaker_camera_network_params->encoding);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS, (const char *)speaker_camera_network_params, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_speaker_camera_network_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t ip_byte_0, uint8_t ip_byte_1, uint8_t ip_byte_2, uint8_t ip_byte_3, uint8_t bitrate, uint8_t resolution, uint8_t encoding)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, ip_byte_0);
    _mav_put_uint8_t(buf, 1, ip_byte_1);
    _mav_put_uint8_t(buf, 2, ip_byte_2);
    _mav_put_uint8_t(buf, 3, ip_byte_3);
    _mav_put_uint8_t(buf, 4, bitrate);
    _mav_put_uint8_t(buf, 5, resolution);
    _mav_put_uint8_t(buf, 6, encoding);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS, buf, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#else
    mavlink_speaker_camera_network_params_t *packet = (mavlink_speaker_camera_network_params_t *)msgbuf;
    packet->ip_byte_0 = ip_byte_0;
    packet->ip_byte_1 = ip_byte_1;
    packet->ip_byte_2 = ip_byte_2;
    packet->ip_byte_3 = ip_byte_3;
    packet->bitrate = bitrate;
    packet->resolution = resolution;
    packet->encoding = encoding;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS, (const char *)packet, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_CRC);
#endif
}
#endif

#endif

// MESSAGE SPEAKER_CAMERA_NETWORK_PARAMS UNPACKING


/**
 * @brief Get field ip_byte_0 from speaker_camera_network_params message
 *
 * @return  IP byte 0
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_ip_byte_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ip_byte_1 from speaker_camera_network_params message
 *
 * @return  IP byte 1
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_ip_byte_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ip_byte_2 from speaker_camera_network_params message
 *
 * @return  IP byte 2
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_ip_byte_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ip_byte_3 from speaker_camera_network_params message
 *
 * @return  IP byte 3
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_ip_byte_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field bitrate from speaker_camera_network_params message
 *
 * @return  Bitrate
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field resolution from speaker_camera_network_params message
 *
 * @return  Resolution
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_resolution(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field encoding from speaker_camera_network_params message
 *
 * @return  Encoding
 */
static inline uint8_t mavlink_msg_speaker_camera_network_params_get_encoding(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a speaker_camera_network_params message into a struct
 *
 * @param msg The message to decode
 * @param speaker_camera_network_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_speaker_camera_network_params_decode(const mavlink_message_t* msg, mavlink_speaker_camera_network_params_t* speaker_camera_network_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    speaker_camera_network_params->ip_byte_0 = mavlink_msg_speaker_camera_network_params_get_ip_byte_0(msg);
    speaker_camera_network_params->ip_byte_1 = mavlink_msg_speaker_camera_network_params_get_ip_byte_1(msg);
    speaker_camera_network_params->ip_byte_2 = mavlink_msg_speaker_camera_network_params_get_ip_byte_2(msg);
    speaker_camera_network_params->ip_byte_3 = mavlink_msg_speaker_camera_network_params_get_ip_byte_3(msg);
    speaker_camera_network_params->bitrate = mavlink_msg_speaker_camera_network_params_get_bitrate(msg);
    speaker_camera_network_params->resolution = mavlink_msg_speaker_camera_network_params_get_resolution(msg);
    speaker_camera_network_params->encoding = mavlink_msg_speaker_camera_network_params_get_encoding(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN? msg->len : MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN;
        memset(speaker_camera_network_params, 0, MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_LEN);
    memcpy(speaker_camera_network_params, _MAV_PAYLOAD(msg), len);
#endif
}
