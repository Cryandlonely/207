#pragma once
// MESSAGE TK_TRACKING_STATUS PACKING

#define MAVLINK_MSG_ID_TK_TRACKING_STATUS 51007


typedef struct __mavlink_tk_tracking_status_t {
 int16_t x_deviation_quantity; /*< [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).*/
 int16_t y_deviation_quantity; /*< [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).*/
 uint8_t tracking_status; /*<  Tracking status, area see GIMBAL_TRACKING_STATUS.*/
} mavlink_tk_tracking_status_t;

#define MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN 5
#define MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN 5
#define MAVLINK_MSG_ID_51007_LEN 5
#define MAVLINK_MSG_ID_51007_MIN_LEN 5

#define MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC 132
#define MAVLINK_MSG_ID_51007_CRC 132



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_TRACKING_STATUS { \
    51007, \
    "TK_TRACKING_STATUS", \
    3, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_tk_tracking_status_t, tracking_status) }, \
         { "x_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tk_tracking_status_t, x_deviation_quantity) }, \
         { "y_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tk_tracking_status_t, y_deviation_quantity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_TRACKING_STATUS { \
    "TK_TRACKING_STATUS", \
    3, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_tk_tracking_status_t, tracking_status) }, \
         { "x_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tk_tracking_status_t, x_deviation_quantity) }, \
         { "y_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tk_tracking_status_t, y_deviation_quantity) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_tracking_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Tracking status, area see GIMBAL_TRACKING_STATUS.
 * @param x_deviation_quantity [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_tracking_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t tracking_status, int16_t x_deviation_quantity, int16_t y_deviation_quantity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN];
    _mav_put_int16_t(buf, 0, x_deviation_quantity);
    _mav_put_int16_t(buf, 2, y_deviation_quantity);
    _mav_put_uint8_t(buf, 4, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#else
    mavlink_tk_tracking_status_t packet;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_TRACKING_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
}

/**
 * @brief Pack a tk_tracking_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Tracking status, area see GIMBAL_TRACKING_STATUS.
 * @param x_deviation_quantity [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_tracking_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t tracking_status, int16_t x_deviation_quantity, int16_t y_deviation_quantity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN];
    _mav_put_int16_t(buf, 0, x_deviation_quantity);
    _mav_put_int16_t(buf, 2, y_deviation_quantity);
    _mav_put_uint8_t(buf, 4, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#else
    mavlink_tk_tracking_status_t packet;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_TRACKING_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#endif
}

/**
 * @brief Pack a tk_tracking_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_status  Tracking status, area see GIMBAL_TRACKING_STATUS.
 * @param x_deviation_quantity [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_tracking_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t tracking_status,int16_t x_deviation_quantity,int16_t y_deviation_quantity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN];
    _mav_put_int16_t(buf, 0, x_deviation_quantity);
    _mav_put_int16_t(buf, 2, y_deviation_quantity);
    _mav_put_uint8_t(buf, 4, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#else
    mavlink_tk_tracking_status_t packet;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_TRACKING_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
}

/**
 * @brief Encode a tk_tracking_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_tracking_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_tracking_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_tracking_status_t* tk_tracking_status)
{
    return mavlink_msg_tk_tracking_status_pack(system_id, component_id, msg, tk_tracking_status->tracking_status, tk_tracking_status->x_deviation_quantity, tk_tracking_status->y_deviation_quantity);
}

/**
 * @brief Encode a tk_tracking_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_tracking_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_tracking_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_tracking_status_t* tk_tracking_status)
{
    return mavlink_msg_tk_tracking_status_pack_chan(system_id, component_id, chan, msg, tk_tracking_status->tracking_status, tk_tracking_status->x_deviation_quantity, tk_tracking_status->y_deviation_quantity);
}

/**
 * @brief Encode a tk_tracking_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_tracking_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_tracking_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_tracking_status_t* tk_tracking_status)
{
    return mavlink_msg_tk_tracking_status_pack_status(system_id, component_id, _status, msg,  tk_tracking_status->tracking_status, tk_tracking_status->x_deviation_quantity, tk_tracking_status->y_deviation_quantity);
}

/**
 * @brief Send a tk_tracking_status message
 * @param chan MAVLink channel to send the message
 *
 * @param tracking_status  Tracking status, area see GIMBAL_TRACKING_STATUS.
 * @param x_deviation_quantity [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_tracking_status_send(mavlink_channel_t chan, uint8_t tracking_status, int16_t x_deviation_quantity, int16_t y_deviation_quantity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN];
    _mav_put_int16_t(buf, 0, x_deviation_quantity);
    _mav_put_int16_t(buf, 2, y_deviation_quantity);
    _mav_put_uint8_t(buf, 4, tracking_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#else
    mavlink_tk_tracking_status_t packet;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.tracking_status = tracking_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS, (const char *)&packet, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#endif
}

/**
 * @brief Send a tk_tracking_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_tracking_status_send_struct(mavlink_channel_t chan, const mavlink_tk_tracking_status_t* tk_tracking_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_tracking_status_send(chan, tk_tracking_status->tracking_status, tk_tracking_status->x_deviation_quantity, tk_tracking_status->y_deviation_quantity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS, (const char *)tk_tracking_status, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_tracking_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t tracking_status, int16_t x_deviation_quantity, int16_t y_deviation_quantity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, x_deviation_quantity);
    _mav_put_int16_t(buf, 2, y_deviation_quantity);
    _mav_put_uint8_t(buf, 4, tracking_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS, buf, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#else
    mavlink_tk_tracking_status_t *packet = (mavlink_tk_tracking_status_t *)msgbuf;
    packet->x_deviation_quantity = x_deviation_quantity;
    packet->y_deviation_quantity = y_deviation_quantity;
    packet->tracking_status = tracking_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_TRACKING_STATUS, (const char *)packet, MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN, MAVLINK_MSG_ID_TK_TRACKING_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_TRACKING_STATUS UNPACKING


/**
 * @brief Get field tracking_status from tk_tracking_status message
 *
 * @return  Tracking status, area see GIMBAL_TRACKING_STATUS.
 */
static inline uint8_t mavlink_msg_tk_tracking_status_get_tracking_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field x_deviation_quantity from tk_tracking_status message
 *
 * @return [ Pixels]  x-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 */
static inline int16_t mavlink_msg_tk_tracking_status_get_x_deviation_quantity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field y_deviation_quantity from tk_tracking_status message
 *
 * @return [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 */
static inline int16_t mavlink_msg_tk_tracking_status_get_y_deviation_quantity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a tk_tracking_status message into a struct
 *
 * @param msg The message to decode
 * @param tk_tracking_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_tracking_status_decode(const mavlink_message_t* msg, mavlink_tk_tracking_status_t* tk_tracking_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_tracking_status->x_deviation_quantity = mavlink_msg_tk_tracking_status_get_x_deviation_quantity(msg);
    tk_tracking_status->y_deviation_quantity = mavlink_msg_tk_tracking_status_get_y_deviation_quantity(msg);
    tk_tracking_status->tracking_status = mavlink_msg_tk_tracking_status_get_tracking_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN? msg->len : MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN;
        memset(tk_tracking_status, 0, MAVLINK_MSG_ID_TK_TRACKING_STATUS_LEN);
    memcpy(tk_tracking_status, _MAV_PAYLOAD(msg), len);
#endif
}
