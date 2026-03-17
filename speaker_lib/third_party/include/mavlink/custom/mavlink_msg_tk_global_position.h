#pragma once
// MESSAGE TK_GLOBAL_POSITION PACKING

#define MAVLINK_MSG_ID_TK_GLOBAL_POSITION 51005


typedef struct __mavlink_tk_global_position_t {
 int32_t longitude; /*< [degE7] Longitude, unit [degE7].*/
 int32_t latitude; /*< [degE7] Latitude, unit [degE7].*/
 int32_t altitude; /*< [mm] Altitude, unit [mm].*/
 int32_t relative_altitude; /*< [mm] Relative altitude, unit [mm].*/
 int32_t target_longitude; /*< [degE7] Target longitude, unit [degE7].*/
 int32_t target_latitude; /*< [degE7] Target latitude, unit [degE7].*/
 int32_t target_altitude; /*< [mm] Target altitude, unit [mm].*/
 uint16_t heading; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint16_t target_distance; /*< [dm]  Target horizontal distance, unit [dm].*/
} mavlink_tk_global_position_t;

#define MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN 32
#define MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN 32
#define MAVLINK_MSG_ID_51005_LEN 32
#define MAVLINK_MSG_ID_51005_MIN_LEN 32

#define MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC 83
#define MAVLINK_MSG_ID_51005_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_GLOBAL_POSITION { \
    51005, \
    "TK_GLOBAL_POSITION", \
    9, \
    {  { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_tk_global_position_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tk_global_position_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tk_global_position_t, altitude) }, \
         { "relative_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tk_global_position_t, relative_altitude) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_tk_global_position_t, heading) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_tk_global_position_t, target_longitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_tk_global_position_t, target_latitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_tk_global_position_t, target_altitude) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_tk_global_position_t, target_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_GLOBAL_POSITION { \
    "TK_GLOBAL_POSITION", \
    9, \
    {  { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_tk_global_position_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tk_global_position_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tk_global_position_t, altitude) }, \
         { "relative_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tk_global_position_t, relative_altitude) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_tk_global_position_t, heading) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_tk_global_position_t, target_longitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_tk_global_position_t, target_latitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_tk_global_position_t, target_altitude) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_tk_global_position_t, target_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_global_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param longitude [degE7] Longitude, unit [degE7].
 * @param latitude [degE7] Latitude, unit [degE7].
 * @param altitude [mm] Altitude, unit [mm].
 * @param relative_altitude [mm] Relative altitude, unit [mm].
 * @param heading [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param target_longitude [degE7] Target longitude, unit [degE7].
 * @param target_latitude [degE7] Target latitude, unit [degE7].
 * @param target_altitude [mm] Target altitude, unit [mm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_global_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_altitude, uint16_t heading, int32_t target_longitude, int32_t target_latitude, int32_t target_altitude, uint16_t target_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN];
    _mav_put_int32_t(buf, 0, longitude);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int32_t(buf, 12, relative_altitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_latitude);
    _mav_put_int32_t(buf, 24, target_altitude);
    _mav_put_uint16_t(buf, 28, heading);
    _mav_put_uint16_t(buf, 30, target_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#else
    mavlink_tk_global_position_t packet;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.target_altitude = target_altitude;
    packet.heading = heading;
    packet.target_distance = target_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GLOBAL_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
}

/**
 * @brief Pack a tk_global_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param longitude [degE7] Longitude, unit [degE7].
 * @param latitude [degE7] Latitude, unit [degE7].
 * @param altitude [mm] Altitude, unit [mm].
 * @param relative_altitude [mm] Relative altitude, unit [mm].
 * @param heading [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param target_longitude [degE7] Target longitude, unit [degE7].
 * @param target_latitude [degE7] Target latitude, unit [degE7].
 * @param target_altitude [mm] Target altitude, unit [mm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_global_position_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_altitude, uint16_t heading, int32_t target_longitude, int32_t target_latitude, int32_t target_altitude, uint16_t target_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN];
    _mav_put_int32_t(buf, 0, longitude);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int32_t(buf, 12, relative_altitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_latitude);
    _mav_put_int32_t(buf, 24, target_altitude);
    _mav_put_uint16_t(buf, 28, heading);
    _mav_put_uint16_t(buf, 30, target_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#else
    mavlink_tk_global_position_t packet;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.target_altitude = target_altitude;
    packet.heading = heading;
    packet.target_distance = target_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GLOBAL_POSITION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#endif
}

/**
 * @brief Pack a tk_global_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param longitude [degE7] Longitude, unit [degE7].
 * @param latitude [degE7] Latitude, unit [degE7].
 * @param altitude [mm] Altitude, unit [mm].
 * @param relative_altitude [mm] Relative altitude, unit [mm].
 * @param heading [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param target_longitude [degE7] Target longitude, unit [degE7].
 * @param target_latitude [degE7] Target latitude, unit [degE7].
 * @param target_altitude [mm] Target altitude, unit [mm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_global_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t longitude,int32_t latitude,int32_t altitude,int32_t relative_altitude,uint16_t heading,int32_t target_longitude,int32_t target_latitude,int32_t target_altitude,uint16_t target_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN];
    _mav_put_int32_t(buf, 0, longitude);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int32_t(buf, 12, relative_altitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_latitude);
    _mav_put_int32_t(buf, 24, target_altitude);
    _mav_put_uint16_t(buf, 28, heading);
    _mav_put_uint16_t(buf, 30, target_distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#else
    mavlink_tk_global_position_t packet;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.target_altitude = target_altitude;
    packet.heading = heading;
    packet.target_distance = target_distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GLOBAL_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
}

/**
 * @brief Encode a tk_global_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_global_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_global_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_global_position_t* tk_global_position)
{
    return mavlink_msg_tk_global_position_pack(system_id, component_id, msg, tk_global_position->longitude, tk_global_position->latitude, tk_global_position->altitude, tk_global_position->relative_altitude, tk_global_position->heading, tk_global_position->target_longitude, tk_global_position->target_latitude, tk_global_position->target_altitude, tk_global_position->target_distance);
}

/**
 * @brief Encode a tk_global_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_global_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_global_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_global_position_t* tk_global_position)
{
    return mavlink_msg_tk_global_position_pack_chan(system_id, component_id, chan, msg, tk_global_position->longitude, tk_global_position->latitude, tk_global_position->altitude, tk_global_position->relative_altitude, tk_global_position->heading, tk_global_position->target_longitude, tk_global_position->target_latitude, tk_global_position->target_altitude, tk_global_position->target_distance);
}

/**
 * @brief Encode a tk_global_position struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_global_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_global_position_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_global_position_t* tk_global_position)
{
    return mavlink_msg_tk_global_position_pack_status(system_id, component_id, _status, msg,  tk_global_position->longitude, tk_global_position->latitude, tk_global_position->altitude, tk_global_position->relative_altitude, tk_global_position->heading, tk_global_position->target_longitude, tk_global_position->target_latitude, tk_global_position->target_altitude, tk_global_position->target_distance);
}

/**
 * @brief Send a tk_global_position message
 * @param chan MAVLink channel to send the message
 *
 * @param longitude [degE7] Longitude, unit [degE7].
 * @param latitude [degE7] Latitude, unit [degE7].
 * @param altitude [mm] Altitude, unit [mm].
 * @param relative_altitude [mm] Relative altitude, unit [mm].
 * @param heading [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param target_longitude [degE7] Target longitude, unit [degE7].
 * @param target_latitude [degE7] Target latitude, unit [degE7].
 * @param target_altitude [mm] Target altitude, unit [mm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_global_position_send(mavlink_channel_t chan, int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_altitude, uint16_t heading, int32_t target_longitude, int32_t target_latitude, int32_t target_altitude, uint16_t target_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN];
    _mav_put_int32_t(buf, 0, longitude);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int32_t(buf, 12, relative_altitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_latitude);
    _mav_put_int32_t(buf, 24, target_altitude);
    _mav_put_uint16_t(buf, 28, heading);
    _mav_put_uint16_t(buf, 30, target_distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION, buf, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#else
    mavlink_tk_global_position_t packet;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.altitude = altitude;
    packet.relative_altitude = relative_altitude;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.target_altitude = target_altitude;
    packet.heading = heading;
    packet.target_distance = target_distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION, (const char *)&packet, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#endif
}

/**
 * @brief Send a tk_global_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_global_position_send_struct(mavlink_channel_t chan, const mavlink_tk_global_position_t* tk_global_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_global_position_send(chan, tk_global_position->longitude, tk_global_position->latitude, tk_global_position->altitude, tk_global_position->relative_altitude, tk_global_position->heading, tk_global_position->target_longitude, tk_global_position->target_latitude, tk_global_position->target_altitude, tk_global_position->target_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION, (const char *)tk_global_position, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_global_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_altitude, uint16_t heading, int32_t target_longitude, int32_t target_latitude, int32_t target_altitude, uint16_t target_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, longitude);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, altitude);
    _mav_put_int32_t(buf, 12, relative_altitude);
    _mav_put_int32_t(buf, 16, target_longitude);
    _mav_put_int32_t(buf, 20, target_latitude);
    _mav_put_int32_t(buf, 24, target_altitude);
    _mav_put_uint16_t(buf, 28, heading);
    _mav_put_uint16_t(buf, 30, target_distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION, buf, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#else
    mavlink_tk_global_position_t *packet = (mavlink_tk_global_position_t *)msgbuf;
    packet->longitude = longitude;
    packet->latitude = latitude;
    packet->altitude = altitude;
    packet->relative_altitude = relative_altitude;
    packet->target_longitude = target_longitude;
    packet->target_latitude = target_latitude;
    packet->target_altitude = target_altitude;
    packet->heading = heading;
    packet->target_distance = target_distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GLOBAL_POSITION, (const char *)packet, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_GLOBAL_POSITION UNPACKING


/**
 * @brief Get field longitude from tk_global_position message
 *
 * @return [degE7] Longitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_global_position_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field latitude from tk_global_position message
 *
 * @return [degE7] Latitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_global_position_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from tk_global_position message
 *
 * @return [mm] Altitude, unit [mm].
 */
static inline int32_t mavlink_msg_tk_global_position_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field relative_altitude from tk_global_position message
 *
 * @return [mm] Relative altitude, unit [mm].
 */
static inline int32_t mavlink_msg_tk_global_position_get_relative_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field heading from tk_global_position message
 *
 * @return [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_tk_global_position_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field target_longitude from tk_global_position message
 *
 * @return [degE7] Target longitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_global_position_get_target_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field target_latitude from tk_global_position message
 *
 * @return [degE7] Target latitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_global_position_get_target_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field target_altitude from tk_global_position message
 *
 * @return [mm] Target altitude, unit [mm].
 */
static inline int32_t mavlink_msg_tk_global_position_get_target_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field target_distance from tk_global_position message
 *
 * @return [dm]  Target horizontal distance, unit [dm].
 */
static inline uint16_t mavlink_msg_tk_global_position_get_target_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Decode a tk_global_position message into a struct
 *
 * @param msg The message to decode
 * @param tk_global_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_global_position_decode(const mavlink_message_t* msg, mavlink_tk_global_position_t* tk_global_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_global_position->longitude = mavlink_msg_tk_global_position_get_longitude(msg);
    tk_global_position->latitude = mavlink_msg_tk_global_position_get_latitude(msg);
    tk_global_position->altitude = mavlink_msg_tk_global_position_get_altitude(msg);
    tk_global_position->relative_altitude = mavlink_msg_tk_global_position_get_relative_altitude(msg);
    tk_global_position->target_longitude = mavlink_msg_tk_global_position_get_target_longitude(msg);
    tk_global_position->target_latitude = mavlink_msg_tk_global_position_get_target_latitude(msg);
    tk_global_position->target_altitude = mavlink_msg_tk_global_position_get_target_altitude(msg);
    tk_global_position->heading = mavlink_msg_tk_global_position_get_heading(msg);
    tk_global_position->target_distance = mavlink_msg_tk_global_position_get_target_distance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN? msg->len : MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN;
        memset(tk_global_position, 0, MAVLINK_MSG_ID_TK_GLOBAL_POSITION_LEN);
    memcpy(tk_global_position, _MAV_PAYLOAD(msg), len);
#endif
}
