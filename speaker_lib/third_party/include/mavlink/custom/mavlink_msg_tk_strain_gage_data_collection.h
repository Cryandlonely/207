#pragma once
// MESSAGE TK_STRAIN_GAGE_DATA_COLLECTION PACKING

#define MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION 51705


typedef struct __mavlink_tk_strain_gage_data_collection_t {
 uint8_t data[230]; /*<  Strain gauge data.*/
} mavlink_tk_strain_gage_data_collection_t;

#define MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN 230
#define MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN 230
#define MAVLINK_MSG_ID_51705_LEN 230
#define MAVLINK_MSG_ID_51705_MIN_LEN 230

#define MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC 230
#define MAVLINK_MSG_ID_51705_CRC 230

#define MAVLINK_MSG_TK_STRAIN_GAGE_DATA_COLLECTION_FIELD_DATA_LEN 230

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_STRAIN_GAGE_DATA_COLLECTION { \
    51705, \
    "TK_STRAIN_GAGE_DATA_COLLECTION", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_UINT8_T, 230, 0, offsetof(mavlink_tk_strain_gage_data_collection_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_STRAIN_GAGE_DATA_COLLECTION { \
    "TK_STRAIN_GAGE_DATA_COLLECTION", \
    1, \
    {  { "data", NULL, MAVLINK_TYPE_UINT8_T, 230, 0, offsetof(mavlink_tk_strain_gage_data_collection_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_strain_gage_data_collection message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  Strain gauge data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN];

    _mav_put_uint8_t_array(buf, 0, data, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#else
    mavlink_tk_strain_gage_data_collection_t packet;

    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
}

/**
 * @brief Pack a tk_strain_gage_data_collection message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param data  Strain gauge data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN];

    _mav_put_uint8_t_array(buf, 0, data, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#else
    mavlink_tk_strain_gage_data_collection_t packet;

    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#endif
}

/**
 * @brief Pack a tk_strain_gage_data_collection message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data  Strain gauge data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN];

    _mav_put_uint8_t_array(buf, 0, data, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#else
    mavlink_tk_strain_gage_data_collection_t packet;

    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
}

/**
 * @brief Encode a tk_strain_gage_data_collection struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_strain_gage_data_collection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_strain_gage_data_collection_t* tk_strain_gage_data_collection)
{
    return mavlink_msg_tk_strain_gage_data_collection_pack(system_id, component_id, msg, tk_strain_gage_data_collection->data);
}

/**
 * @brief Encode a tk_strain_gage_data_collection struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_strain_gage_data_collection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_strain_gage_data_collection_t* tk_strain_gage_data_collection)
{
    return mavlink_msg_tk_strain_gage_data_collection_pack_chan(system_id, component_id, chan, msg, tk_strain_gage_data_collection->data);
}

/**
 * @brief Encode a tk_strain_gage_data_collection struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_strain_gage_data_collection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_strain_gage_data_collection_t* tk_strain_gage_data_collection)
{
    return mavlink_msg_tk_strain_gage_data_collection_pack_status(system_id, component_id, _status, msg,  tk_strain_gage_data_collection->data);
}

/**
 * @brief Send a tk_strain_gage_data_collection message
 * @param chan MAVLink channel to send the message
 *
 * @param data  Strain gauge data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_strain_gage_data_collection_send(mavlink_channel_t chan, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN];

    _mav_put_uint8_t_array(buf, 0, data, 230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION, buf, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#else
    mavlink_tk_strain_gage_data_collection_t packet;

    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION, (const char *)&packet, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#endif
}

/**
 * @brief Send a tk_strain_gage_data_collection message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_strain_gage_data_collection_send_struct(mavlink_channel_t chan, const mavlink_tk_strain_gage_data_collection_t* tk_strain_gage_data_collection)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_strain_gage_data_collection_send(chan, tk_strain_gage_data_collection->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION, (const char *)tk_strain_gage_data_collection, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_strain_gage_data_collection_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint8_t_array(buf, 0, data, 230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION, buf, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#else
    mavlink_tk_strain_gage_data_collection_t *packet = (mavlink_tk_strain_gage_data_collection_t *)msgbuf;

    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION, (const char *)packet, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_STRAIN_GAGE_DATA_COLLECTION UNPACKING


/**
 * @brief Get field data from tk_strain_gage_data_collection message
 *
 * @return  Strain gauge data.
 */
static inline uint16_t mavlink_msg_tk_strain_gage_data_collection_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 230,  0);
}

/**
 * @brief Decode a tk_strain_gage_data_collection message into a struct
 *
 * @param msg The message to decode
 * @param tk_strain_gage_data_collection C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_strain_gage_data_collection_decode(const mavlink_message_t* msg, mavlink_tk_strain_gage_data_collection_t* tk_strain_gage_data_collection)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_strain_gage_data_collection_get_data(msg, tk_strain_gage_data_collection->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN? msg->len : MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN;
        memset(tk_strain_gage_data_collection, 0, MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_LEN);
    memcpy(tk_strain_gage_data_collection, _MAV_PAYLOAD(msg), len);
#endif
}
