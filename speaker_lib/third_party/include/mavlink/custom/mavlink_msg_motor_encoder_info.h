#pragma once
// MESSAGE MOTOR_ENCODER_INFO PACKING

#define MAVLINK_MSG_ID_MOTOR_ENCODER_INFO 51504


typedef struct __mavlink_motor_encoder_info_t {
 uint16_t encoder; /*<  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.*/
 uint16_t encoderRaw; /*<  Raw encoder position (uint16_t type, value range depends on encoder resolution).*/
 uint16_t encoderOffset; /*<  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.*/
} mavlink_motor_encoder_info_t;

#define MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN 6
#define MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN 6
#define MAVLINK_MSG_ID_51504_LEN 6
#define MAVLINK_MSG_ID_51504_MIN_LEN 6

#define MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC 57
#define MAVLINK_MSG_ID_51504_CRC 57



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_ENCODER_INFO { \
    51504, \
    "MOTOR_ENCODER_INFO", \
    3, \
    {  { "encoder", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_motor_encoder_info_t, encoder) }, \
         { "encoderRaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_motor_encoder_info_t, encoderRaw) }, \
         { "encoderOffset", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_motor_encoder_info_t, encoderOffset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_ENCODER_INFO { \
    "MOTOR_ENCODER_INFO", \
    3, \
    {  { "encoder", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_motor_encoder_info_t, encoder) }, \
         { "encoderRaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_motor_encoder_info_t, encoderRaw) }, \
         { "encoderOffset", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_motor_encoder_info_t, encoderOffset) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_encoder_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.
 * @param encoderRaw  Raw encoder position (uint16_t type, value range depends on encoder resolution).
 * @param encoderOffset  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_encoder_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t encoder, uint16_t encoderRaw, uint16_t encoderOffset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN];
    _mav_put_uint16_t(buf, 0, encoder);
    _mav_put_uint16_t(buf, 2, encoderRaw);
    _mav_put_uint16_t(buf, 4, encoderOffset);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#else
    mavlink_motor_encoder_info_t packet;
    packet.encoder = encoder;
    packet.encoderRaw = encoderRaw;
    packet.encoderOffset = encoderOffset;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_ENCODER_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
}

/**
 * @brief Pack a motor_encoder_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.
 * @param encoderRaw  Raw encoder position (uint16_t type, value range depends on encoder resolution).
 * @param encoderOffset  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_encoder_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t encoder, uint16_t encoderRaw, uint16_t encoderOffset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN];
    _mav_put_uint16_t(buf, 0, encoder);
    _mav_put_uint16_t(buf, 2, encoderRaw);
    _mav_put_uint16_t(buf, 4, encoderOffset);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#else
    mavlink_motor_encoder_info_t packet;
    packet.encoder = encoder;
    packet.encoderRaw = encoderRaw;
    packet.encoderOffset = encoderOffset;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_ENCODER_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#endif
}

/**
 * @brief Pack a motor_encoder_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.
 * @param encoderRaw  Raw encoder position (uint16_t type, value range depends on encoder resolution).
 * @param encoderOffset  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_encoder_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t encoder,uint16_t encoderRaw,uint16_t encoderOffset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN];
    _mav_put_uint16_t(buf, 0, encoder);
    _mav_put_uint16_t(buf, 2, encoderRaw);
    _mav_put_uint16_t(buf, 4, encoderOffset);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#else
    mavlink_motor_encoder_info_t packet;
    packet.encoder = encoder;
    packet.encoderRaw = encoderRaw;
    packet.encoderOffset = encoderOffset;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_ENCODER_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
}

/**
 * @brief Encode a motor_encoder_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_encoder_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_encoder_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_encoder_info_t* motor_encoder_info)
{
    return mavlink_msg_motor_encoder_info_pack(system_id, component_id, msg, motor_encoder_info->encoder, motor_encoder_info->encoderRaw, motor_encoder_info->encoderOffset);
}

/**
 * @brief Encode a motor_encoder_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_encoder_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_encoder_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_encoder_info_t* motor_encoder_info)
{
    return mavlink_msg_motor_encoder_info_pack_chan(system_id, component_id, chan, msg, motor_encoder_info->encoder, motor_encoder_info->encoderRaw, motor_encoder_info->encoderOffset);
}

/**
 * @brief Encode a motor_encoder_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_encoder_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_encoder_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_encoder_info_t* motor_encoder_info)
{
    return mavlink_msg_motor_encoder_info_pack_status(system_id, component_id, _status, msg,  motor_encoder_info->encoder, motor_encoder_info->encoderRaw, motor_encoder_info->encoderOffset);
}

/**
 * @brief Send a motor_encoder_info message
 * @param chan MAVLink channel to send the message
 *
 * @param encoder  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.
 * @param encoderRaw  Raw encoder position (uint16_t type, value range depends on encoder resolution).
 * @param encoderOffset  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_encoder_info_send(mavlink_channel_t chan, uint16_t encoder, uint16_t encoderRaw, uint16_t encoderOffset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN];
    _mav_put_uint16_t(buf, 0, encoder);
    _mav_put_uint16_t(buf, 2, encoderRaw);
    _mav_put_uint16_t(buf, 4, encoderOffset);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO, buf, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#else
    mavlink_motor_encoder_info_t packet;
    packet.encoder = encoder;
    packet.encoderRaw = encoderRaw;
    packet.encoderOffset = encoderOffset;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#endif
}

/**
 * @brief Send a motor_encoder_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_encoder_info_send_struct(mavlink_channel_t chan, const mavlink_motor_encoder_info_t* motor_encoder_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_encoder_info_send(chan, motor_encoder_info->encoder, motor_encoder_info->encoderRaw, motor_encoder_info->encoderOffset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO, (const char *)motor_encoder_info, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_encoder_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t encoder, uint16_t encoderRaw, uint16_t encoderOffset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, encoder);
    _mav_put_uint16_t(buf, 2, encoderRaw);
    _mav_put_uint16_t(buf, 4, encoderOffset);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO, buf, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#else
    mavlink_motor_encoder_info_t *packet = (mavlink_motor_encoder_info_t *)msgbuf;
    packet->encoder = encoder;
    packet->encoderRaw = encoderRaw;
    packet->encoderOffset = encoderOffset;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO, (const char *)packet, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_ENCODER_INFO UNPACKING


/**
 * @brief Get field encoder from motor_encoder_info message
 *
 * @return  Encoder position (uint16_t type, value range depends on encoder resolution), which is the raw encoder position minus the encoder offset.
 */
static inline uint16_t mavlink_msg_motor_encoder_info_get_encoder(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field encoderRaw from motor_encoder_info message
 *
 * @return  Raw encoder position (uint16_t type, value range depends on encoder resolution).
 */
static inline uint16_t mavlink_msg_motor_encoder_info_get_encoderRaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field encoderOffset from motor_encoder_info message
 *
 * @return  Encoder offset (uint16_t type, value range depends on encoder resolution), this point is the initial zero position after motor power-on.
 */
static inline uint16_t mavlink_msg_motor_encoder_info_get_encoderOffset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a motor_encoder_info message into a struct
 *
 * @param msg The message to decode
 * @param motor_encoder_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_encoder_info_decode(const mavlink_message_t* msg, mavlink_motor_encoder_info_t* motor_encoder_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_encoder_info->encoder = mavlink_msg_motor_encoder_info_get_encoder(msg);
    motor_encoder_info->encoderRaw = mavlink_msg_motor_encoder_info_get_encoderRaw(msg);
    motor_encoder_info->encoderOffset = mavlink_msg_motor_encoder_info_get_encoderOffset(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN;
        memset(motor_encoder_info, 0, MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_LEN);
    memcpy(motor_encoder_info, _MAV_PAYLOAD(msg), len);
#endif
}
