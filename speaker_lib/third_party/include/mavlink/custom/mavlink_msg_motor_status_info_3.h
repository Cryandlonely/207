#pragma once
// MESSAGE MOTOR_STATUS_INFO_3 PACKING

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3 51502


typedef struct __mavlink_motor_status_info_3_t {
 float motor_temperature; /*<  Motor temperature, unit: 1°C.*/
 float phase_current_a; /*<  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.*/
 float phase_current_b; /*<   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.*/
 float phase_current_c; /*<   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.*/
} mavlink_motor_status_info_3_t;

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN 16
#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN 16
#define MAVLINK_MSG_ID_51502_LEN 16
#define MAVLINK_MSG_ID_51502_MIN_LEN 16

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC 162
#define MAVLINK_MSG_ID_51502_CRC 162



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_3 { \
    51502, \
    "MOTOR_STATUS_INFO_3", \
    4, \
    {  { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_3_t, motor_temperature) }, \
         { "phase_current_a", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_3_t, phase_current_a) }, \
         { "phase_current_b", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_3_t, phase_current_b) }, \
         { "phase_current_c", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_status_info_3_t, phase_current_c) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_3 { \
    "MOTOR_STATUS_INFO_3", \
    4, \
    {  { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_3_t, motor_temperature) }, \
         { "phase_current_a", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_3_t, phase_current_a) }, \
         { "phase_current_b", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_3_t, phase_current_b) }, \
         { "phase_current_c", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_status_info_3_t, phase_current_c) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_status_info_3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param phase_current_a  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_b   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_c   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float motor_temperature, float phase_current_a, float phase_current_b, float phase_current_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, phase_current_a);
    _mav_put_float(buf, 8, phase_current_b);
    _mav_put_float(buf, 12, phase_current_c);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#else
    mavlink_motor_status_info_3_t packet;
    packet.motor_temperature = motor_temperature;
    packet.phase_current_a = phase_current_a;
    packet.phase_current_b = phase_current_b;
    packet.phase_current_c = phase_current_c;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
}

/**
 * @brief Pack a motor_status_info_3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param phase_current_a  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_b   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_c   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_3_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float motor_temperature, float phase_current_a, float phase_current_b, float phase_current_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, phase_current_a);
    _mav_put_float(buf, 8, phase_current_b);
    _mav_put_float(buf, 12, phase_current_c);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#else
    mavlink_motor_status_info_3_t packet;
    packet.motor_temperature = motor_temperature;
    packet.phase_current_a = phase_current_a;
    packet.phase_current_b = phase_current_b;
    packet.phase_current_c = phase_current_c;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#endif
}

/**
 * @brief Pack a motor_status_info_3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param phase_current_a  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_b   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_c   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float motor_temperature,float phase_current_a,float phase_current_b,float phase_current_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, phase_current_a);
    _mav_put_float(buf, 8, phase_current_b);
    _mav_put_float(buf, 12, phase_current_c);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#else
    mavlink_motor_status_info_3_t packet;
    packet.motor_temperature = motor_temperature;
    packet.phase_current_a = phase_current_a;
    packet.phase_current_b = phase_current_b;
    packet.phase_current_c = phase_current_c;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
}

/**
 * @brief Encode a motor_status_info_3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_status_info_3_t* motor_status_info_3)
{
    return mavlink_msg_motor_status_info_3_pack(system_id, component_id, msg, motor_status_info_3->motor_temperature, motor_status_info_3->phase_current_a, motor_status_info_3->phase_current_b, motor_status_info_3->phase_current_c);
}

/**
 * @brief Encode a motor_status_info_3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_status_info_3_t* motor_status_info_3)
{
    return mavlink_msg_motor_status_info_3_pack_chan(system_id, component_id, chan, msg, motor_status_info_3->motor_temperature, motor_status_info_3->phase_current_a, motor_status_info_3->phase_current_b, motor_status_info_3->phase_current_c);
}

/**
 * @brief Encode a motor_status_info_3 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_3_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_status_info_3_t* motor_status_info_3)
{
    return mavlink_msg_motor_status_info_3_pack_status(system_id, component_id, _status, msg,  motor_status_info_3->motor_temperature, motor_status_info_3->phase_current_a, motor_status_info_3->phase_current_b, motor_status_info_3->phase_current_c);
}

/**
 * @brief Send a motor_status_info_3 message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param phase_current_a  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_b   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 * @param phase_current_c   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_status_info_3_send(mavlink_channel_t chan, float motor_temperature, float phase_current_a, float phase_current_b, float phase_current_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, phase_current_a);
    _mav_put_float(buf, 8, phase_current_b);
    _mav_put_float(buf, 12, phase_current_c);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#else
    mavlink_motor_status_info_3_t packet;
    packet.motor_temperature = motor_temperature;
    packet.phase_current_a = phase_current_a;
    packet.phase_current_b = phase_current_b;
    packet.phase_current_c = phase_current_c;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#endif
}

/**
 * @brief Send a motor_status_info_3 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_status_info_3_send_struct(mavlink_channel_t chan, const mavlink_motor_status_info_3_t* motor_status_info_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_status_info_3_send(chan, motor_status_info_3->motor_temperature, motor_status_info_3->phase_current_a, motor_status_info_3->phase_current_b, motor_status_info_3->phase_current_c);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3, (const char *)motor_status_info_3, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_status_info_3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float motor_temperature, float phase_current_a, float phase_current_b, float phase_current_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, phase_current_a);
    _mav_put_float(buf, 8, phase_current_b);
    _mav_put_float(buf, 12, phase_current_c);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#else
    mavlink_motor_status_info_3_t *packet = (mavlink_motor_status_info_3_t *)msgbuf;
    packet->motor_temperature = motor_temperature;
    packet->phase_current_a = phase_current_a;
    packet->phase_current_b = phase_current_b;
    packet->phase_current_c = phase_current_c;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3, (const char *)packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_STATUS_INFO_3 UNPACKING


/**
 * @brief Get field motor_temperature from motor_status_info_3 message
 *
 * @return  Motor temperature, unit: 1°C.
 */
static inline float mavlink_msg_motor_status_info_3_get_motor_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field phase_current_a from motor_status_info_3 message
 *
 * @return  For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 */
static inline float mavlink_msg_motor_status_info_3_get_phase_current_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field phase_current_b from motor_status_info_3 message
 *
 * @return   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 */
static inline float mavlink_msg_motor_status_info_3_get_phase_current_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field phase_current_c from motor_status_info_3 message
 *
 * @return   For MG motor phase current resolution is (66/4096 A)/LSB; For MF motor phase current resolution is (33/4096 A)/LSB.
 */
static inline float mavlink_msg_motor_status_info_3_get_phase_current_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a motor_status_info_3 message into a struct
 *
 * @param msg The message to decode
 * @param motor_status_info_3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_status_info_3_decode(const mavlink_message_t* msg, mavlink_motor_status_info_3_t* motor_status_info_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_status_info_3->motor_temperature = mavlink_msg_motor_status_info_3_get_motor_temperature(msg);
    motor_status_info_3->phase_current_a = mavlink_msg_motor_status_info_3_get_phase_current_a(msg);
    motor_status_info_3->phase_current_b = mavlink_msg_motor_status_info_3_get_phase_current_b(msg);
    motor_status_info_3->phase_current_c = mavlink_msg_motor_status_info_3_get_phase_current_c(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN;
        memset(motor_status_info_3, 0, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_LEN);
    memcpy(motor_status_info_3, _MAV_PAYLOAD(msg), len);
#endif
}
