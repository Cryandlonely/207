#pragma once
// MESSAGE MOTOR_STATUS_INFO_2 PACKING

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2 51501


typedef struct __mavlink_motor_status_info_2_t {
 float motor_temperature; /*<  Motor temperature, unit: 1°C.*/
 float torque_current; /*<    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.*/
 float output_power; /*<   Output power,range:-1000~1000.*/
 float motor_speed; /*<  Motor speed, unit: 1dps/LSB.*/
 float motor_angle; /*<  Motor angle.*/
 uint16_t encoder_position; /*<  Encoder position.*/
} mavlink_motor_status_info_2_t;

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN 22
#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN 22
#define MAVLINK_MSG_ID_51501_LEN 22
#define MAVLINK_MSG_ID_51501_MIN_LEN 22

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC 10
#define MAVLINK_MSG_ID_51501_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_2 { \
    51501, \
    "MOTOR_STATUS_INFO_2", \
    6, \
    {  { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_2_t, motor_temperature) }, \
         { "torque_current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_2_t, torque_current) }, \
         { "output_power", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_2_t, output_power) }, \
         { "motor_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_status_info_2_t, motor_speed) }, \
         { "motor_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_status_info_2_t, motor_angle) }, \
         { "encoder_position", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_motor_status_info_2_t, encoder_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_2 { \
    "MOTOR_STATUS_INFO_2", \
    6, \
    {  { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_2_t, motor_temperature) }, \
         { "torque_current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_2_t, torque_current) }, \
         { "output_power", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_2_t, output_power) }, \
         { "motor_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_motor_status_info_2_t, motor_speed) }, \
         { "motor_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_motor_status_info_2_t, motor_angle) }, \
         { "encoder_position", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_motor_status_info_2_t, encoder_position) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_status_info_2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param torque_current    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.
 * @param output_power   Output power,range:-1000~1000.
 * @param motor_speed  Motor speed, unit: 1dps/LSB.
 * @param motor_angle  Motor angle.
 * @param encoder_position  Encoder position.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float motor_temperature, float torque_current, float output_power, float motor_speed, float motor_angle, uint16_t encoder_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, torque_current);
    _mav_put_float(buf, 8, output_power);
    _mav_put_float(buf, 12, motor_speed);
    _mav_put_float(buf, 16, motor_angle);
    _mav_put_uint16_t(buf, 20, encoder_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#else
    mavlink_motor_status_info_2_t packet;
    packet.motor_temperature = motor_temperature;
    packet.torque_current = torque_current;
    packet.output_power = output_power;
    packet.motor_speed = motor_speed;
    packet.motor_angle = motor_angle;
    packet.encoder_position = encoder_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
}

/**
 * @brief Pack a motor_status_info_2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param torque_current    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.
 * @param output_power   Output power,range:-1000~1000.
 * @param motor_speed  Motor speed, unit: 1dps/LSB.
 * @param motor_angle  Motor angle.
 * @param encoder_position  Encoder position.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_2_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float motor_temperature, float torque_current, float output_power, float motor_speed, float motor_angle, uint16_t encoder_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, torque_current);
    _mav_put_float(buf, 8, output_power);
    _mav_put_float(buf, 12, motor_speed);
    _mav_put_float(buf, 16, motor_angle);
    _mav_put_uint16_t(buf, 20, encoder_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#else
    mavlink_motor_status_info_2_t packet;
    packet.motor_temperature = motor_temperature;
    packet.torque_current = torque_current;
    packet.output_power = output_power;
    packet.motor_speed = motor_speed;
    packet.motor_angle = motor_angle;
    packet.encoder_position = encoder_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#endif
}

/**
 * @brief Pack a motor_status_info_2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param torque_current    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.
 * @param output_power   Output power,range:-1000~1000.
 * @param motor_speed  Motor speed, unit: 1dps/LSB.
 * @param motor_angle  Motor angle.
 * @param encoder_position  Encoder position.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float motor_temperature,float torque_current,float output_power,float motor_speed,float motor_angle,uint16_t encoder_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, torque_current);
    _mav_put_float(buf, 8, output_power);
    _mav_put_float(buf, 12, motor_speed);
    _mav_put_float(buf, 16, motor_angle);
    _mav_put_uint16_t(buf, 20, encoder_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#else
    mavlink_motor_status_info_2_t packet;
    packet.motor_temperature = motor_temperature;
    packet.torque_current = torque_current;
    packet.output_power = output_power;
    packet.motor_speed = motor_speed;
    packet.motor_angle = motor_angle;
    packet.encoder_position = encoder_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
}

/**
 * @brief Encode a motor_status_info_2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_status_info_2_t* motor_status_info_2)
{
    return mavlink_msg_motor_status_info_2_pack(system_id, component_id, msg, motor_status_info_2->motor_temperature, motor_status_info_2->torque_current, motor_status_info_2->output_power, motor_status_info_2->motor_speed, motor_status_info_2->motor_angle, motor_status_info_2->encoder_position);
}

/**
 * @brief Encode a motor_status_info_2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_status_info_2_t* motor_status_info_2)
{
    return mavlink_msg_motor_status_info_2_pack_chan(system_id, component_id, chan, msg, motor_status_info_2->motor_temperature, motor_status_info_2->torque_current, motor_status_info_2->output_power, motor_status_info_2->motor_speed, motor_status_info_2->motor_angle, motor_status_info_2->encoder_position);
}

/**
 * @brief Encode a motor_status_info_2 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_2_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_status_info_2_t* motor_status_info_2)
{
    return mavlink_msg_motor_status_info_2_pack_status(system_id, component_id, _status, msg,  motor_status_info_2->motor_temperature, motor_status_info_2->torque_current, motor_status_info_2->output_power, motor_status_info_2->motor_speed, motor_status_info_2->motor_angle, motor_status_info_2->encoder_position);
}

/**
 * @brief Send a motor_status_info_2 message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_temperature  Motor temperature, unit: 1°C.
 * @param torque_current    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.
 * @param output_power   Output power,range:-1000~1000.
 * @param motor_speed  Motor speed, unit: 1dps/LSB.
 * @param motor_angle  Motor angle.
 * @param encoder_position  Encoder position.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_status_info_2_send(mavlink_channel_t chan, float motor_temperature, float torque_current, float output_power, float motor_speed, float motor_angle, uint16_t encoder_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN];
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, torque_current);
    _mav_put_float(buf, 8, output_power);
    _mav_put_float(buf, 12, motor_speed);
    _mav_put_float(buf, 16, motor_angle);
    _mav_put_uint16_t(buf, 20, encoder_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#else
    mavlink_motor_status_info_2_t packet;
    packet.motor_temperature = motor_temperature;
    packet.torque_current = torque_current;
    packet.output_power = output_power;
    packet.motor_speed = motor_speed;
    packet.motor_angle = motor_angle;
    packet.encoder_position = encoder_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#endif
}

/**
 * @brief Send a motor_status_info_2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_status_info_2_send_struct(mavlink_channel_t chan, const mavlink_motor_status_info_2_t* motor_status_info_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_status_info_2_send(chan, motor_status_info_2->motor_temperature, motor_status_info_2->torque_current, motor_status_info_2->output_power, motor_status_info_2->motor_speed, motor_status_info_2->motor_angle, motor_status_info_2->encoder_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2, (const char *)motor_status_info_2, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_status_info_2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float motor_temperature, float torque_current, float output_power, float motor_speed, float motor_angle, uint16_t encoder_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, motor_temperature);
    _mav_put_float(buf, 4, torque_current);
    _mav_put_float(buf, 8, output_power);
    _mav_put_float(buf, 12, motor_speed);
    _mav_put_float(buf, 16, motor_angle);
    _mav_put_uint16_t(buf, 20, encoder_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#else
    mavlink_motor_status_info_2_t *packet = (mavlink_motor_status_info_2_t *)msgbuf;
    packet->motor_temperature = motor_temperature;
    packet->torque_current = torque_current;
    packet->output_power = output_power;
    packet->motor_speed = motor_speed;
    packet->motor_angle = motor_angle;
    packet->encoder_position = encoder_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2, (const char *)packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_STATUS_INFO_2 UNPACKING


/**
 * @brief Get field motor_temperature from motor_status_info_2 message
 *
 * @return  Motor temperature, unit: 1°C.
 */
static inline float mavlink_msg_motor_status_info_2_get_motor_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field torque_current from motor_status_info_2 message
 *
 * @return    For MG motor iq resolution is (66/4096 A)/LSB; For MF motor iq resolution is (33/4096 A)/LSB.
 */
static inline float mavlink_msg_motor_status_info_2_get_torque_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field output_power from motor_status_info_2 message
 *
 * @return   Output power,range:-1000~1000.
 */
static inline float mavlink_msg_motor_status_info_2_get_output_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor_speed from motor_status_info_2 message
 *
 * @return  Motor speed, unit: 1dps/LSB.
 */
static inline float mavlink_msg_motor_status_info_2_get_motor_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field motor_angle from motor_status_info_2 message
 *
 * @return  Motor angle.
 */
static inline float mavlink_msg_motor_status_info_2_get_motor_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field encoder_position from motor_status_info_2 message
 *
 * @return  Encoder position.
 */
static inline uint16_t mavlink_msg_motor_status_info_2_get_encoder_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a motor_status_info_2 message into a struct
 *
 * @param msg The message to decode
 * @param motor_status_info_2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_status_info_2_decode(const mavlink_message_t* msg, mavlink_motor_status_info_2_t* motor_status_info_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_status_info_2->motor_temperature = mavlink_msg_motor_status_info_2_get_motor_temperature(msg);
    motor_status_info_2->torque_current = mavlink_msg_motor_status_info_2_get_torque_current(msg);
    motor_status_info_2->output_power = mavlink_msg_motor_status_info_2_get_output_power(msg);
    motor_status_info_2->motor_speed = mavlink_msg_motor_status_info_2_get_motor_speed(msg);
    motor_status_info_2->motor_angle = mavlink_msg_motor_status_info_2_get_motor_angle(msg);
    motor_status_info_2->encoder_position = mavlink_msg_motor_status_info_2_get_encoder_position(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN;
        memset(motor_status_info_2, 0, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_LEN);
    memcpy(motor_status_info_2, _MAV_PAYLOAD(msg), len);
#endif
}
