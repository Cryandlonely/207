#pragma once
// MESSAGE IP_ADDRESS_INFO PACKING

#define MAVLINK_MSG_ID_IP_ADDRESS_INFO 51703


typedef struct __mavlink_ip_address_info_t {
 uint32_t ipAddress; /*<  IP address.*/
 uint16_t ipPort; /*<  IP port.*/
} mavlink_ip_address_info_t;

#define MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN 6
#define MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN 6
#define MAVLINK_MSG_ID_51703_LEN 6
#define MAVLINK_MSG_ID_51703_MIN_LEN 6

#define MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC 140
#define MAVLINK_MSG_ID_51703_CRC 140



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IP_ADDRESS_INFO { \
    51703, \
    "IP_ADDRESS_INFO", \
    2, \
    {  { "ipAddress", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ip_address_info_t, ipAddress) }, \
         { "ipPort", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ip_address_info_t, ipPort) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IP_ADDRESS_INFO { \
    "IP_ADDRESS_INFO", \
    2, \
    {  { "ipAddress", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ip_address_info_t, ipAddress) }, \
         { "ipPort", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ip_address_info_t, ipPort) }, \
         } \
}
#endif

/**
 * @brief Pack a ip_address_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ipAddress  IP address.
 * @param ipPort  IP port.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ip_address_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t ipAddress, uint16_t ipPort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN];
    _mav_put_uint32_t(buf, 0, ipAddress);
    _mav_put_uint16_t(buf, 4, ipPort);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#else
    mavlink_ip_address_info_t packet;
    packet.ipAddress = ipAddress;
    packet.ipPort = ipPort;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IP_ADDRESS_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
}

/**
 * @brief Pack a ip_address_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ipAddress  IP address.
 * @param ipPort  IP port.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ip_address_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t ipAddress, uint16_t ipPort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN];
    _mav_put_uint32_t(buf, 0, ipAddress);
    _mav_put_uint16_t(buf, 4, ipPort);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#else
    mavlink_ip_address_info_t packet;
    packet.ipAddress = ipAddress;
    packet.ipPort = ipPort;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IP_ADDRESS_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#endif
}

/**
 * @brief Pack a ip_address_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ipAddress  IP address.
 * @param ipPort  IP port.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ip_address_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t ipAddress,uint16_t ipPort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN];
    _mav_put_uint32_t(buf, 0, ipAddress);
    _mav_put_uint16_t(buf, 4, ipPort);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#else
    mavlink_ip_address_info_t packet;
    packet.ipAddress = ipAddress;
    packet.ipPort = ipPort;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IP_ADDRESS_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
}

/**
 * @brief Encode a ip_address_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ip_address_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ip_address_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ip_address_info_t* ip_address_info)
{
    return mavlink_msg_ip_address_info_pack(system_id, component_id, msg, ip_address_info->ipAddress, ip_address_info->ipPort);
}

/**
 * @brief Encode a ip_address_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ip_address_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ip_address_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ip_address_info_t* ip_address_info)
{
    return mavlink_msg_ip_address_info_pack_chan(system_id, component_id, chan, msg, ip_address_info->ipAddress, ip_address_info->ipPort);
}

/**
 * @brief Encode a ip_address_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ip_address_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ip_address_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ip_address_info_t* ip_address_info)
{
    return mavlink_msg_ip_address_info_pack_status(system_id, component_id, _status, msg,  ip_address_info->ipAddress, ip_address_info->ipPort);
}

/**
 * @brief Send a ip_address_info message
 * @param chan MAVLink channel to send the message
 *
 * @param ipAddress  IP address.
 * @param ipPort  IP port.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ip_address_info_send(mavlink_channel_t chan, uint32_t ipAddress, uint16_t ipPort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN];
    _mav_put_uint32_t(buf, 0, ipAddress);
    _mav_put_uint16_t(buf, 4, ipPort);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO, buf, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#else
    mavlink_ip_address_info_t packet;
    packet.ipAddress = ipAddress;
    packet.ipPort = ipPort;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO, (const char *)&packet, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#endif
}

/**
 * @brief Send a ip_address_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ip_address_info_send_struct(mavlink_channel_t chan, const mavlink_ip_address_info_t* ip_address_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ip_address_info_send(chan, ip_address_info->ipAddress, ip_address_info->ipPort);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO, (const char *)ip_address_info, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ip_address_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t ipAddress, uint16_t ipPort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, ipAddress);
    _mav_put_uint16_t(buf, 4, ipPort);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO, buf, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#else
    mavlink_ip_address_info_t *packet = (mavlink_ip_address_info_t *)msgbuf;
    packet->ipAddress = ipAddress;
    packet->ipPort = ipPort;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IP_ADDRESS_INFO, (const char *)packet, MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN, MAVLINK_MSG_ID_IP_ADDRESS_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE IP_ADDRESS_INFO UNPACKING


/**
 * @brief Get field ipAddress from ip_address_info message
 *
 * @return  IP address.
 */
static inline uint32_t mavlink_msg_ip_address_info_get_ipAddress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ipPort from ip_address_info message
 *
 * @return  IP port.
 */
static inline uint16_t mavlink_msg_ip_address_info_get_ipPort(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a ip_address_info message into a struct
 *
 * @param msg The message to decode
 * @param ip_address_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_ip_address_info_decode(const mavlink_message_t* msg, mavlink_ip_address_info_t* ip_address_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ip_address_info->ipAddress = mavlink_msg_ip_address_info_get_ipAddress(msg);
    ip_address_info->ipPort = mavlink_msg_ip_address_info_get_ipPort(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN? msg->len : MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN;
        memset(ip_address_info, 0, MAVLINK_MSG_ID_IP_ADDRESS_INFO_LEN);
    memcpy(ip_address_info, _MAV_PAYLOAD(msg), len);
#endif
}
