#pragma once
// MESSAGE TK_PERIPHERAL_DEVICE_STATUS PACKING

#define MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS 51009


typedef struct __mavlink_tk_peripheral_device_status_t {
 uint32_t status_flag; /*<  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.*/
 uint32_t reserved; /*<  Reserved.*/
 uint16_t camera_zoom; /*<  camera zoom.*/
 uint16_t laser_distance; /*< [dm]  Laser ranging distance, unit dm.*/
 uint16_t photos_cnt; /*<   Photo count.*/
} mavlink_tk_peripheral_device_status_t;

#define MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN 14
#define MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_51009_LEN 14
#define MAVLINK_MSG_ID_51009_MIN_LEN 14

#define MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC 4
#define MAVLINK_MSG_ID_51009_CRC 4



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_PERIPHERAL_DEVICE_STATUS { \
    51009, \
    "TK_PERIPHERAL_DEVICE_STATUS", \
    5, \
    {  { "camera_zoom", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_tk_peripheral_device_status_t, camera_zoom) }, \
         { "laser_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_tk_peripheral_device_status_t, laser_distance) }, \
         { "photos_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tk_peripheral_device_status_t, photos_cnt) }, \
         { "status_flag", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_tk_peripheral_device_status_t, status_flag) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_tk_peripheral_device_status_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_PERIPHERAL_DEVICE_STATUS { \
    "TK_PERIPHERAL_DEVICE_STATUS", \
    5, \
    {  { "camera_zoom", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_tk_peripheral_device_status_t, camera_zoom) }, \
         { "laser_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_tk_peripheral_device_status_t, laser_distance) }, \
         { "photos_cnt", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tk_peripheral_device_status_t, photos_cnt) }, \
         { "status_flag", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_tk_peripheral_device_status_t, status_flag) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_tk_peripheral_device_status_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_peripheral_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param camera_zoom  camera zoom.
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param photos_cnt   Photo count.
 * @param status_flag  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.
 * @param reserved  Reserved.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t camera_zoom, uint16_t laser_distance, uint16_t photos_cnt, uint32_t status_flag, uint32_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, status_flag);
    _mav_put_uint32_t(buf, 4, reserved);
    _mav_put_uint16_t(buf, 8, camera_zoom);
    _mav_put_uint16_t(buf, 10, laser_distance);
    _mav_put_uint16_t(buf, 12, photos_cnt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#else
    mavlink_tk_peripheral_device_status_t packet;
    packet.status_flag = status_flag;
    packet.reserved = reserved;
    packet.camera_zoom = camera_zoom;
    packet.laser_distance = laser_distance;
    packet.photos_cnt = photos_cnt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
}

/**
 * @brief Pack a tk_peripheral_device_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param camera_zoom  camera zoom.
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param photos_cnt   Photo count.
 * @param status_flag  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.
 * @param reserved  Reserved.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t camera_zoom, uint16_t laser_distance, uint16_t photos_cnt, uint32_t status_flag, uint32_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, status_flag);
    _mav_put_uint32_t(buf, 4, reserved);
    _mav_put_uint16_t(buf, 8, camera_zoom);
    _mav_put_uint16_t(buf, 10, laser_distance);
    _mav_put_uint16_t(buf, 12, photos_cnt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#else
    mavlink_tk_peripheral_device_status_t packet;
    packet.status_flag = status_flag;
    packet.reserved = reserved;
    packet.camera_zoom = camera_zoom;
    packet.laser_distance = laser_distance;
    packet.photos_cnt = photos_cnt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a tk_peripheral_device_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_zoom  camera zoom.
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param photos_cnt   Photo count.
 * @param status_flag  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.
 * @param reserved  Reserved.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t camera_zoom,uint16_t laser_distance,uint16_t photos_cnt,uint32_t status_flag,uint32_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, status_flag);
    _mav_put_uint32_t(buf, 4, reserved);
    _mav_put_uint16_t(buf, 8, camera_zoom);
    _mav_put_uint16_t(buf, 10, laser_distance);
    _mav_put_uint16_t(buf, 12, photos_cnt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#else
    mavlink_tk_peripheral_device_status_t packet;
    packet.status_flag = status_flag;
    packet.reserved = reserved;
    packet.camera_zoom = camera_zoom;
    packet.laser_distance = laser_distance;
    packet.photos_cnt = photos_cnt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
}

/**
 * @brief Encode a tk_peripheral_device_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_peripheral_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_peripheral_device_status_t* tk_peripheral_device_status)
{
    return mavlink_msg_tk_peripheral_device_status_pack(system_id, component_id, msg, tk_peripheral_device_status->camera_zoom, tk_peripheral_device_status->laser_distance, tk_peripheral_device_status->photos_cnt, tk_peripheral_device_status->status_flag, tk_peripheral_device_status->reserved);
}

/**
 * @brief Encode a tk_peripheral_device_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_peripheral_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_peripheral_device_status_t* tk_peripheral_device_status)
{
    return mavlink_msg_tk_peripheral_device_status_pack_chan(system_id, component_id, chan, msg, tk_peripheral_device_status->camera_zoom, tk_peripheral_device_status->laser_distance, tk_peripheral_device_status->photos_cnt, tk_peripheral_device_status->status_flag, tk_peripheral_device_status->reserved);
}

/**
 * @brief Encode a tk_peripheral_device_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_peripheral_device_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_peripheral_device_status_t* tk_peripheral_device_status)
{
    return mavlink_msg_tk_peripheral_device_status_pack_status(system_id, component_id, _status, msg,  tk_peripheral_device_status->camera_zoom, tk_peripheral_device_status->laser_distance, tk_peripheral_device_status->photos_cnt, tk_peripheral_device_status->status_flag, tk_peripheral_device_status->reserved);
}

/**
 * @brief Send a tk_peripheral_device_status message
 * @param chan MAVLink channel to send the message
 *
 * @param camera_zoom  camera zoom.
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param photos_cnt   Photo count.
 * @param status_flag  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.
 * @param reserved  Reserved.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_peripheral_device_status_send(mavlink_channel_t chan, uint16_t camera_zoom, uint16_t laser_distance, uint16_t photos_cnt, uint32_t status_flag, uint32_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, status_flag);
    _mav_put_uint32_t(buf, 4, reserved);
    _mav_put_uint16_t(buf, 8, camera_zoom);
    _mav_put_uint16_t(buf, 10, laser_distance);
    _mav_put_uint16_t(buf, 12, photos_cnt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS, buf, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#else
    mavlink_tk_peripheral_device_status_t packet;
    packet.status_flag = status_flag;
    packet.reserved = reserved;
    packet.camera_zoom = camera_zoom;
    packet.laser_distance = laser_distance;
    packet.photos_cnt = photos_cnt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#endif
}

/**
 * @brief Send a tk_peripheral_device_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_peripheral_device_status_send_struct(mavlink_channel_t chan, const mavlink_tk_peripheral_device_status_t* tk_peripheral_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_peripheral_device_status_send(chan, tk_peripheral_device_status->camera_zoom, tk_peripheral_device_status->laser_distance, tk_peripheral_device_status->photos_cnt, tk_peripheral_device_status->status_flag, tk_peripheral_device_status->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS, (const char *)tk_peripheral_device_status, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_peripheral_device_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t camera_zoom, uint16_t laser_distance, uint16_t photos_cnt, uint32_t status_flag, uint32_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, status_flag);
    _mav_put_uint32_t(buf, 4, reserved);
    _mav_put_uint16_t(buf, 8, camera_zoom);
    _mav_put_uint16_t(buf, 10, laser_distance);
    _mav_put_uint16_t(buf, 12, photos_cnt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS, buf, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#else
    mavlink_tk_peripheral_device_status_t *packet = (mavlink_tk_peripheral_device_status_t *)msgbuf;
    packet->status_flag = status_flag;
    packet->reserved = reserved;
    packet->camera_zoom = camera_zoom;
    packet->laser_distance = laser_distance;
    packet->photos_cnt = photos_cnt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS, (const char *)packet, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_PERIPHERAL_DEVICE_STATUS UNPACKING


/**
 * @brief Get field camera_zoom from tk_peripheral_device_status message
 *
 * @return  camera zoom.
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_get_camera_zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field laser_distance from tk_peripheral_device_status message
 *
 * @return [dm]  Laser ranging distance, unit dm.
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_get_laser_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field photos_cnt from tk_peripheral_device_status message
 *
 * @return   Photo count.
 */
static inline uint16_t mavlink_msg_tk_peripheral_device_status_get_photos_cnt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field status_flag from tk_peripheral_device_status message
 *
 * @return  Peripheral device status flag, area see TK_PERIPHERAL_DEVICE_STATUS_FLAG.
 */
static inline uint32_t mavlink_msg_tk_peripheral_device_status_get_status_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field reserved from tk_peripheral_device_status message
 *
 * @return  Reserved.
 */
static inline uint32_t mavlink_msg_tk_peripheral_device_status_get_reserved(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a tk_peripheral_device_status message into a struct
 *
 * @param msg The message to decode
 * @param tk_peripheral_device_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_peripheral_device_status_decode(const mavlink_message_t* msg, mavlink_tk_peripheral_device_status_t* tk_peripheral_device_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_peripheral_device_status->status_flag = mavlink_msg_tk_peripheral_device_status_get_status_flag(msg);
    tk_peripheral_device_status->reserved = mavlink_msg_tk_peripheral_device_status_get_reserved(msg);
    tk_peripheral_device_status->camera_zoom = mavlink_msg_tk_peripheral_device_status_get_camera_zoom(msg);
    tk_peripheral_device_status->laser_distance = mavlink_msg_tk_peripheral_device_status_get_laser_distance(msg);
    tk_peripheral_device_status->photos_cnt = mavlink_msg_tk_peripheral_device_status_get_photos_cnt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN;
        memset(tk_peripheral_device_status, 0, MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_LEN);
    memcpy(tk_peripheral_device_status, _MAV_PAYLOAD(msg), len);
#endif
}
