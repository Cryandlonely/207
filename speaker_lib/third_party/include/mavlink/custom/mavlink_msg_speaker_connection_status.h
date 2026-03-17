#pragma once
// MESSAGE SPEAKER_CONNECTION_STATUS PACKING

#define MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS 20303


typedef struct __mavlink_speaker_connection_status_t {
 uint64_t connection_id; /*<  The connection id.*/
 uint8_t operation_ack; /*<  Connection operation type.*/
 uint8_t connection_amount; /*<  The connection amount.*/
 uint8_t connection_index; /*<  The connection index.*/
 uint8_t connection_status; /*<  The connection status.*/
} mavlink_speaker_connection_status_t;

#define MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN 12
#define MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_20303_LEN 12
#define MAVLINK_MSG_ID_20303_MIN_LEN 12

#define MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC 15
#define MAVLINK_MSG_ID_20303_CRC 15



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPEAKER_CONNECTION_STATUS { \
    20303, \
    "SPEAKER_CONNECTION_STATUS", \
    5, \
    {  { "operation_ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_speaker_connection_status_t, operation_ack) }, \
         { "connection_amount", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_speaker_connection_status_t, connection_amount) }, \
         { "connection_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_speaker_connection_status_t, connection_index) }, \
         { "connection_id", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_speaker_connection_status_t, connection_id) }, \
         { "connection_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_speaker_connection_status_t, connection_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPEAKER_CONNECTION_STATUS { \
    "SPEAKER_CONNECTION_STATUS", \
    5, \
    {  { "operation_ack", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_speaker_connection_status_t, operation_ack) }, \
         { "connection_amount", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_speaker_connection_status_t, connection_amount) }, \
         { "connection_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_speaker_connection_status_t, connection_index) }, \
         { "connection_id", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_speaker_connection_status_t, connection_id) }, \
         { "connection_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_speaker_connection_status_t, connection_status) }, \
         } \
}
#endif

/**
 * @brief Pack a speaker_connection_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param operation_ack  Connection operation type.
 * @param connection_amount  The connection amount.
 * @param connection_index  The connection index.
 * @param connection_id  The connection id.
 * @param connection_status  The connection status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_connection_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t operation_ack, uint8_t connection_amount, uint8_t connection_index, uint64_t connection_id, uint8_t connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, connection_id);
    _mav_put_uint8_t(buf, 8, operation_ack);
    _mav_put_uint8_t(buf, 9, connection_amount);
    _mav_put_uint8_t(buf, 10, connection_index);
    _mav_put_uint8_t(buf, 11, connection_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#else
    mavlink_speaker_connection_status_t packet;
    packet.connection_id = connection_id;
    packet.operation_ack = operation_ack;
    packet.connection_amount = connection_amount;
    packet.connection_index = connection_index;
    packet.connection_status = connection_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
}

/**
 * @brief Pack a speaker_connection_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param operation_ack  Connection operation type.
 * @param connection_amount  The connection amount.
 * @param connection_index  The connection index.
 * @param connection_id  The connection id.
 * @param connection_status  The connection status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_connection_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t operation_ack, uint8_t connection_amount, uint8_t connection_index, uint64_t connection_id, uint8_t connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, connection_id);
    _mav_put_uint8_t(buf, 8, operation_ack);
    _mav_put_uint8_t(buf, 9, connection_amount);
    _mav_put_uint8_t(buf, 10, connection_index);
    _mav_put_uint8_t(buf, 11, connection_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#else
    mavlink_speaker_connection_status_t packet;
    packet.connection_id = connection_id;
    packet.operation_ack = operation_ack;
    packet.connection_amount = connection_amount;
    packet.connection_index = connection_index;
    packet.connection_status = connection_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#endif
}

/**
 * @brief Pack a speaker_connection_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param operation_ack  Connection operation type.
 * @param connection_amount  The connection amount.
 * @param connection_index  The connection index.
 * @param connection_id  The connection id.
 * @param connection_status  The connection status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_connection_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t operation_ack,uint8_t connection_amount,uint8_t connection_index,uint64_t connection_id,uint8_t connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, connection_id);
    _mav_put_uint8_t(buf, 8, operation_ack);
    _mav_put_uint8_t(buf, 9, connection_amount);
    _mav_put_uint8_t(buf, 10, connection_index);
    _mav_put_uint8_t(buf, 11, connection_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#else
    mavlink_speaker_connection_status_t packet;
    packet.connection_id = connection_id;
    packet.operation_ack = operation_ack;
    packet.connection_amount = connection_amount;
    packet.connection_index = connection_index;
    packet.connection_status = connection_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
}

/**
 * @brief Encode a speaker_connection_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param speaker_connection_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_connection_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_speaker_connection_status_t* speaker_connection_status)
{
    return mavlink_msg_speaker_connection_status_pack(system_id, component_id, msg, speaker_connection_status->operation_ack, speaker_connection_status->connection_amount, speaker_connection_status->connection_index, speaker_connection_status->connection_id, speaker_connection_status->connection_status);
}

/**
 * @brief Encode a speaker_connection_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speaker_connection_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_connection_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_speaker_connection_status_t* speaker_connection_status)
{
    return mavlink_msg_speaker_connection_status_pack_chan(system_id, component_id, chan, msg, speaker_connection_status->operation_ack, speaker_connection_status->connection_amount, speaker_connection_status->connection_index, speaker_connection_status->connection_id, speaker_connection_status->connection_status);
}

/**
 * @brief Encode a speaker_connection_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param speaker_connection_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_connection_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_speaker_connection_status_t* speaker_connection_status)
{
    return mavlink_msg_speaker_connection_status_pack_status(system_id, component_id, _status, msg,  speaker_connection_status->operation_ack, speaker_connection_status->connection_amount, speaker_connection_status->connection_index, speaker_connection_status->connection_id, speaker_connection_status->connection_status);
}

/**
 * @brief Send a speaker_connection_status message
 * @param chan MAVLink channel to send the message
 *
 * @param operation_ack  Connection operation type.
 * @param connection_amount  The connection amount.
 * @param connection_index  The connection index.
 * @param connection_id  The connection id.
 * @param connection_status  The connection status.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_speaker_connection_status_send(mavlink_channel_t chan, uint8_t operation_ack, uint8_t connection_amount, uint8_t connection_index, uint64_t connection_id, uint8_t connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, connection_id);
    _mav_put_uint8_t(buf, 8, operation_ack);
    _mav_put_uint8_t(buf, 9, connection_amount);
    _mav_put_uint8_t(buf, 10, connection_index);
    _mav_put_uint8_t(buf, 11, connection_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS, buf, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#else
    mavlink_speaker_connection_status_t packet;
    packet.connection_id = connection_id;
    packet.operation_ack = operation_ack;
    packet.connection_amount = connection_amount;
    packet.connection_index = connection_index;
    packet.connection_status = connection_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#endif
}

/**
 * @brief Send a speaker_connection_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_speaker_connection_status_send_struct(mavlink_channel_t chan, const mavlink_speaker_connection_status_t* speaker_connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_speaker_connection_status_send(chan, speaker_connection_status->operation_ack, speaker_connection_status->connection_amount, speaker_connection_status->connection_index, speaker_connection_status->connection_id, speaker_connection_status->connection_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS, (const char *)speaker_connection_status, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_speaker_connection_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t operation_ack, uint8_t connection_amount, uint8_t connection_index, uint64_t connection_id, uint8_t connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, connection_id);
    _mav_put_uint8_t(buf, 8, operation_ack);
    _mav_put_uint8_t(buf, 9, connection_amount);
    _mav_put_uint8_t(buf, 10, connection_index);
    _mav_put_uint8_t(buf, 11, connection_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS, buf, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#else
    mavlink_speaker_connection_status_t *packet = (mavlink_speaker_connection_status_t *)msgbuf;
    packet->connection_id = connection_id;
    packet->operation_ack = operation_ack;
    packet->connection_amount = connection_amount;
    packet->connection_index = connection_index;
    packet->connection_status = connection_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS, (const char *)packet, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SPEAKER_CONNECTION_STATUS UNPACKING


/**
 * @brief Get field operation_ack from speaker_connection_status message
 *
 * @return  Connection operation type.
 */
static inline uint8_t mavlink_msg_speaker_connection_status_get_operation_ack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field connection_amount from speaker_connection_status message
 *
 * @return  The connection amount.
 */
static inline uint8_t mavlink_msg_speaker_connection_status_get_connection_amount(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field connection_index from speaker_connection_status message
 *
 * @return  The connection index.
 */
static inline uint8_t mavlink_msg_speaker_connection_status_get_connection_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field connection_id from speaker_connection_status message
 *
 * @return  The connection id.
 */
static inline uint64_t mavlink_msg_speaker_connection_status_get_connection_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field connection_status from speaker_connection_status message
 *
 * @return  The connection status.
 */
static inline uint8_t mavlink_msg_speaker_connection_status_get_connection_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Decode a speaker_connection_status message into a struct
 *
 * @param msg The message to decode
 * @param speaker_connection_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_speaker_connection_status_decode(const mavlink_message_t* msg, mavlink_speaker_connection_status_t* speaker_connection_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    speaker_connection_status->connection_id = mavlink_msg_speaker_connection_status_get_connection_id(msg);
    speaker_connection_status->operation_ack = mavlink_msg_speaker_connection_status_get_operation_ack(msg);
    speaker_connection_status->connection_amount = mavlink_msg_speaker_connection_status_get_connection_amount(msg);
    speaker_connection_status->connection_index = mavlink_msg_speaker_connection_status_get_connection_index(msg);
    speaker_connection_status->connection_status = mavlink_msg_speaker_connection_status_get_connection_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN;
        memset(speaker_connection_status, 0, MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_LEN);
    memcpy(speaker_connection_status, _MAV_PAYLOAD(msg), len);
#endif
}
