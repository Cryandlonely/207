#pragma once
// MESSAGE MOTOR_STATUS_INFO_1 PACKING

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1 51500


typedef struct __mavlink_motor_status_info_1_t {
 float bus_voltage; /*<    Bus voltage, unit: 0.01V/LSB.*/
 float bus_current; /*<  Bus current, unit: 0.01A/LSB.*/
 float motor_temperature; /*<  Motor temperature, unit: 1°C/LSB.*/
 uint8_t motor_status; /*<  Custom data ,area see MOTOR_STATUS.*/
 uint8_t error_status; /*<  Custom data area,see MOTOR_ERR_STATUS.*/
} mavlink_motor_status_info_1_t;

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN 14
#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN 14
#define MAVLINK_MSG_ID_51500_LEN 14
#define MAVLINK_MSG_ID_51500_MIN_LEN 14

#define MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC 254
#define MAVLINK_MSG_ID_51500_CRC 254



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_1 { \
    51500, \
    "MOTOR_STATUS_INFO_1", \
    5, \
    {  { "bus_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_1_t, bus_voltage) }, \
         { "bus_current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_1_t, bus_current) }, \
         { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_1_t, motor_temperature) }, \
         { "motor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_motor_status_info_1_t, motor_status) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_motor_status_info_1_t, error_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_STATUS_INFO_1 { \
    "MOTOR_STATUS_INFO_1", \
    5, \
    {  { "bus_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_status_info_1_t, bus_voltage) }, \
         { "bus_current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motor_status_info_1_t, bus_current) }, \
         { "motor_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motor_status_info_1_t, motor_temperature) }, \
         { "motor_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_motor_status_info_1_t, motor_status) }, \
         { "error_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_motor_status_info_1_t, error_status) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_status_info_1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param bus_voltage    Bus voltage, unit: 0.01V/LSB.
 * @param bus_current  Bus current, unit: 0.01A/LSB.
 * @param motor_temperature  Motor temperature, unit: 1°C/LSB.
 * @param motor_status  Custom data ,area see MOTOR_STATUS.
 * @param error_status  Custom data area,see MOTOR_ERR_STATUS.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float bus_voltage, float bus_current, float motor_temperature, uint8_t motor_status, uint8_t error_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN];
    _mav_put_float(buf, 0, bus_voltage);
    _mav_put_float(buf, 4, bus_current);
    _mav_put_float(buf, 8, motor_temperature);
    _mav_put_uint8_t(buf, 12, motor_status);
    _mav_put_uint8_t(buf, 13, error_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#else
    mavlink_motor_status_info_1_t packet;
    packet.bus_voltage = bus_voltage;
    packet.bus_current = bus_current;
    packet.motor_temperature = motor_temperature;
    packet.motor_status = motor_status;
    packet.error_status = error_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
}

/**
 * @brief Pack a motor_status_info_1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param bus_voltage    Bus voltage, unit: 0.01V/LSB.
 * @param bus_current  Bus current, unit: 0.01A/LSB.
 * @param motor_temperature  Motor temperature, unit: 1°C/LSB.
 * @param motor_status  Custom data ,area see MOTOR_STATUS.
 * @param error_status  Custom data area,see MOTOR_ERR_STATUS.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_1_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float bus_voltage, float bus_current, float motor_temperature, uint8_t motor_status, uint8_t error_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN];
    _mav_put_float(buf, 0, bus_voltage);
    _mav_put_float(buf, 4, bus_current);
    _mav_put_float(buf, 8, motor_temperature);
    _mav_put_uint8_t(buf, 12, motor_status);
    _mav_put_uint8_t(buf, 13, error_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#else
    mavlink_motor_status_info_1_t packet;
    packet.bus_voltage = bus_voltage;
    packet.bus_current = bus_current;
    packet.motor_temperature = motor_temperature;
    packet.motor_status = motor_status;
    packet.error_status = error_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#endif
}

/**
 * @brief Pack a motor_status_info_1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bus_voltage    Bus voltage, unit: 0.01V/LSB.
 * @param bus_current  Bus current, unit: 0.01A/LSB.
 * @param motor_temperature  Motor temperature, unit: 1°C/LSB.
 * @param motor_status  Custom data ,area see MOTOR_STATUS.
 * @param error_status  Custom data area,see MOTOR_ERR_STATUS.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_status_info_1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float bus_voltage,float bus_current,float motor_temperature,uint8_t motor_status,uint8_t error_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN];
    _mav_put_float(buf, 0, bus_voltage);
    _mav_put_float(buf, 4, bus_current);
    _mav_put_float(buf, 8, motor_temperature);
    _mav_put_uint8_t(buf, 12, motor_status);
    _mav_put_uint8_t(buf, 13, error_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#else
    mavlink_motor_status_info_1_t packet;
    packet.bus_voltage = bus_voltage;
    packet.bus_current = bus_current;
    packet.motor_temperature = motor_temperature;
    packet.motor_status = motor_status;
    packet.error_status = error_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
}

/**
 * @brief Encode a motor_status_info_1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_status_info_1_t* motor_status_info_1)
{
    return mavlink_msg_motor_status_info_1_pack(system_id, component_id, msg, motor_status_info_1->bus_voltage, motor_status_info_1->bus_current, motor_status_info_1->motor_temperature, motor_status_info_1->motor_status, motor_status_info_1->error_status);
}

/**
 * @brief Encode a motor_status_info_1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_status_info_1_t* motor_status_info_1)
{
    return mavlink_msg_motor_status_info_1_pack_chan(system_id, component_id, chan, msg, motor_status_info_1->bus_voltage, motor_status_info_1->bus_current, motor_status_info_1->motor_temperature, motor_status_info_1->motor_status, motor_status_info_1->error_status);
}

/**
 * @brief Encode a motor_status_info_1 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_status_info_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_status_info_1_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_status_info_1_t* motor_status_info_1)
{
    return mavlink_msg_motor_status_info_1_pack_status(system_id, component_id, _status, msg,  motor_status_info_1->bus_voltage, motor_status_info_1->bus_current, motor_status_info_1->motor_temperature, motor_status_info_1->motor_status, motor_status_info_1->error_status);
}

/**
 * @brief Send a motor_status_info_1 message
 * @param chan MAVLink channel to send the message
 *
 * @param bus_voltage    Bus voltage, unit: 0.01V/LSB.
 * @param bus_current  Bus current, unit: 0.01A/LSB.
 * @param motor_temperature  Motor temperature, unit: 1°C/LSB.
 * @param motor_status  Custom data ,area see MOTOR_STATUS.
 * @param error_status  Custom data area,see MOTOR_ERR_STATUS.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_status_info_1_send(mavlink_channel_t chan, float bus_voltage, float bus_current, float motor_temperature, uint8_t motor_status, uint8_t error_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN];
    _mav_put_float(buf, 0, bus_voltage);
    _mav_put_float(buf, 4, bus_current);
    _mav_put_float(buf, 8, motor_temperature);
    _mav_put_uint8_t(buf, 12, motor_status);
    _mav_put_uint8_t(buf, 13, error_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#else
    mavlink_motor_status_info_1_t packet;
    packet.bus_voltage = bus_voltage;
    packet.bus_current = bus_current;
    packet.motor_temperature = motor_temperature;
    packet.motor_status = motor_status;
    packet.error_status = error_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#endif
}

/**
 * @brief Send a motor_status_info_1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_status_info_1_send_struct(mavlink_channel_t chan, const mavlink_motor_status_info_1_t* motor_status_info_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_status_info_1_send(chan, motor_status_info_1->bus_voltage, motor_status_info_1->bus_current, motor_status_info_1->motor_temperature, motor_status_info_1->motor_status, motor_status_info_1->error_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1, (const char *)motor_status_info_1, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_status_info_1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float bus_voltage, float bus_current, float motor_temperature, uint8_t motor_status, uint8_t error_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, bus_voltage);
    _mav_put_float(buf, 4, bus_current);
    _mav_put_float(buf, 8, motor_temperature);
    _mav_put_uint8_t(buf, 12, motor_status);
    _mav_put_uint8_t(buf, 13, error_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1, buf, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#else
    mavlink_motor_status_info_1_t *packet = (mavlink_motor_status_info_1_t *)msgbuf;
    packet->bus_voltage = bus_voltage;
    packet->bus_current = bus_current;
    packet->motor_temperature = motor_temperature;
    packet->motor_status = motor_status;
    packet->error_status = error_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1, (const char *)packet, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_STATUS_INFO_1 UNPACKING


/**
 * @brief Get field bus_voltage from motor_status_info_1 message
 *
 * @return    Bus voltage, unit: 0.01V/LSB.
 */
static inline float mavlink_msg_motor_status_info_1_get_bus_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field bus_current from motor_status_info_1 message
 *
 * @return  Bus current, unit: 0.01A/LSB.
 */
static inline float mavlink_msg_motor_status_info_1_get_bus_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field motor_temperature from motor_status_info_1 message
 *
 * @return  Motor temperature, unit: 1°C/LSB.
 */
static inline float mavlink_msg_motor_status_info_1_get_motor_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor_status from motor_status_info_1 message
 *
 * @return  Custom data ,area see MOTOR_STATUS.
 */
static inline uint8_t mavlink_msg_motor_status_info_1_get_motor_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field error_status from motor_status_info_1 message
 *
 * @return  Custom data area,see MOTOR_ERR_STATUS.
 */
static inline uint8_t mavlink_msg_motor_status_info_1_get_error_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a motor_status_info_1 message into a struct
 *
 * @param msg The message to decode
 * @param motor_status_info_1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_status_info_1_decode(const mavlink_message_t* msg, mavlink_motor_status_info_1_t* motor_status_info_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_status_info_1->bus_voltage = mavlink_msg_motor_status_info_1_get_bus_voltage(msg);
    motor_status_info_1->bus_current = mavlink_msg_motor_status_info_1_get_bus_current(msg);
    motor_status_info_1->motor_temperature = mavlink_msg_motor_status_info_1_get_motor_temperature(msg);
    motor_status_info_1->motor_status = mavlink_msg_motor_status_info_1_get_motor_status(msg);
    motor_status_info_1->error_status = mavlink_msg_motor_status_info_1_get_error_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN;
        memset(motor_status_info_1, 0, MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_LEN);
    memcpy(motor_status_info_1, _MAV_PAYLOAD(msg), len);
#endif
}
