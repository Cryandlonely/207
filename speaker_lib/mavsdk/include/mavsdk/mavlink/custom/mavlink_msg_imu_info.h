#pragma once
// MESSAGE IMU_INFO PACKING

#define MAVLINK_MSG_ID_IMU_INFO 51702


typedef struct __mavlink_imu_info_t {
 float temperature; /*<  IMU temperature in Celsius.*/
 float gyro_offset[3]; /*<  Gyroscope offset [X, Y, Z].*/
 float acc_norm[3]; /*<  Accelerometer offset [X, Y, Z].*/
 float acc_matrix[9]; /*<  3x3 accelerometer calibration matrix (row-major).*/
 uint8_t flag; /*<  Calibration complete flag (1 = done).*/
} mavlink_imu_info_t;

#define MAVLINK_MSG_ID_IMU_INFO_LEN 65
#define MAVLINK_MSG_ID_IMU_INFO_MIN_LEN 65
#define MAVLINK_MSG_ID_51702_LEN 65
#define MAVLINK_MSG_ID_51702_MIN_LEN 65

#define MAVLINK_MSG_ID_IMU_INFO_CRC 115
#define MAVLINK_MSG_ID_51702_CRC 115

#define MAVLINK_MSG_IMU_INFO_FIELD_GYRO_OFFSET_LEN 3
#define MAVLINK_MSG_IMU_INFO_FIELD_ACC_NORM_LEN 3
#define MAVLINK_MSG_IMU_INFO_FIELD_ACC_MATRIX_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU_INFO { \
    51702, \
    "IMU_INFO", \
    5, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_info_t, temperature) }, \
         { "gyro_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 4, offsetof(mavlink_imu_info_t, gyro_offset) }, \
         { "acc_norm", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_imu_info_t, acc_norm) }, \
         { "acc_matrix", NULL, MAVLINK_TYPE_FLOAT, 9, 28, offsetof(mavlink_imu_info_t, acc_matrix) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_imu_info_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU_INFO { \
    "IMU_INFO", \
    5, \
    {  { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_info_t, temperature) }, \
         { "gyro_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 4, offsetof(mavlink_imu_info_t, gyro_offset) }, \
         { "acc_norm", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_imu_info_t, acc_norm) }, \
         { "acc_matrix", NULL, MAVLINK_TYPE_FLOAT, 9, 28, offsetof(mavlink_imu_info_t, acc_matrix) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_imu_info_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a imu_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature  IMU temperature in Celsius.
 * @param gyro_offset  Gyroscope offset [X, Y, Z].
 * @param acc_norm  Accelerometer offset [X, Y, Z].
 * @param acc_matrix  3x3 accelerometer calibration matrix (row-major).
 * @param flag  Calibration complete flag (1 = done).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float temperature, const float *gyro_offset, const float *acc_norm, const float *acc_matrix, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_uint8_t(buf, 64, flag);
    _mav_put_float_array(buf, 4, gyro_offset, 3);
    _mav_put_float_array(buf, 16, acc_norm, 3);
    _mav_put_float_array(buf, 28, acc_matrix, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_INFO_LEN);
#else
    mavlink_imu_info_t packet;
    packet.temperature = temperature;
    packet.flag = flag;
    mav_array_memcpy(packet.gyro_offset, gyro_offset, sizeof(float)*3);
    mav_array_memcpy(packet.acc_norm, acc_norm, sizeof(float)*3);
    mav_array_memcpy(packet.acc_matrix, acc_matrix, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
}

/**
 * @brief Pack a imu_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature  IMU temperature in Celsius.
 * @param gyro_offset  Gyroscope offset [X, Y, Z].
 * @param acc_norm  Accelerometer offset [X, Y, Z].
 * @param acc_matrix  3x3 accelerometer calibration matrix (row-major).
 * @param flag  Calibration complete flag (1 = done).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float temperature, const float *gyro_offset, const float *acc_norm, const float *acc_matrix, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_uint8_t(buf, 64, flag);
    _mav_put_float_array(buf, 4, gyro_offset, 3);
    _mav_put_float_array(buf, 16, acc_norm, 3);
    _mav_put_float_array(buf, 28, acc_matrix, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_INFO_LEN);
#else
    mavlink_imu_info_t packet;
    packet.temperature = temperature;
    packet.flag = flag;
    mav_array_memcpy(packet.gyro_offset, gyro_offset, sizeof(float)*3);
    mav_array_memcpy(packet.acc_norm, acc_norm, sizeof(float)*3);
    mav_array_memcpy(packet.acc_matrix, acc_matrix, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN);
#endif
}

/**
 * @brief Pack a imu_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature  IMU temperature in Celsius.
 * @param gyro_offset  Gyroscope offset [X, Y, Z].
 * @param acc_norm  Accelerometer offset [X, Y, Z].
 * @param acc_matrix  3x3 accelerometer calibration matrix (row-major).
 * @param flag  Calibration complete flag (1 = done).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float temperature,const float *gyro_offset,const float *acc_norm,const float *acc_matrix,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_uint8_t(buf, 64, flag);
    _mav_put_float_array(buf, 4, gyro_offset, 3);
    _mav_put_float_array(buf, 16, acc_norm, 3);
    _mav_put_float_array(buf, 28, acc_matrix, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_INFO_LEN);
#else
    mavlink_imu_info_t packet;
    packet.temperature = temperature;
    packet.flag = flag;
    mav_array_memcpy(packet.gyro_offset, gyro_offset, sizeof(float)*3);
    mav_array_memcpy(packet.acc_norm, acc_norm, sizeof(float)*3);
    mav_array_memcpy(packet.acc_matrix, acc_matrix, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
}

/**
 * @brief Encode a imu_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_info_t* imu_info)
{
    return mavlink_msg_imu_info_pack(system_id, component_id, msg, imu_info->temperature, imu_info->gyro_offset, imu_info->acc_norm, imu_info->acc_matrix, imu_info->flag);
}

/**
 * @brief Encode a imu_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_info_t* imu_info)
{
    return mavlink_msg_imu_info_pack_chan(system_id, component_id, chan, msg, imu_info->temperature, imu_info->gyro_offset, imu_info->acc_norm, imu_info->acc_matrix, imu_info->flag);
}

/**
 * @brief Encode a imu_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param imu_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_imu_info_t* imu_info)
{
    return mavlink_msg_imu_info_pack_status(system_id, component_id, _status, msg,  imu_info->temperature, imu_info->gyro_offset, imu_info->acc_norm, imu_info->acc_matrix, imu_info->flag);
}

/**
 * @brief Send a imu_info message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature  IMU temperature in Celsius.
 * @param gyro_offset  Gyroscope offset [X, Y, Z].
 * @param acc_norm  Accelerometer offset [X, Y, Z].
 * @param acc_matrix  3x3 accelerometer calibration matrix (row-major).
 * @param flag  Calibration complete flag (1 = done).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_info_send(mavlink_channel_t chan, float temperature, const float *gyro_offset, const float *acc_norm, const float *acc_matrix, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_INFO_LEN];
    _mav_put_float(buf, 0, temperature);
    _mav_put_uint8_t(buf, 64, flag);
    _mav_put_float_array(buf, 4, gyro_offset, 3);
    _mav_put_float_array(buf, 16, acc_norm, 3);
    _mav_put_float_array(buf, 28, acc_matrix, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_INFO, buf, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#else
    mavlink_imu_info_t packet;
    packet.temperature = temperature;
    packet.flag = flag;
    mav_array_memcpy(packet.gyro_offset, gyro_offset, sizeof(float)*3);
    mav_array_memcpy(packet.acc_norm, acc_norm, sizeof(float)*3);
    mav_array_memcpy(packet.acc_matrix, acc_matrix, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_INFO, (const char *)&packet, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#endif
}

/**
 * @brief Send a imu_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_info_send_struct(mavlink_channel_t chan, const mavlink_imu_info_t* imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_info_send(chan, imu_info->temperature, imu_info->gyro_offset, imu_info->acc_norm, imu_info->acc_matrix, imu_info->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_INFO, (const char *)imu_info, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float temperature, const float *gyro_offset, const float *acc_norm, const float *acc_matrix, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, temperature);
    _mav_put_uint8_t(buf, 64, flag);
    _mav_put_float_array(buf, 4, gyro_offset, 3);
    _mav_put_float_array(buf, 16, acc_norm, 3);
    _mav_put_float_array(buf, 28, acc_matrix, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_INFO, buf, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#else
    mavlink_imu_info_t *packet = (mavlink_imu_info_t *)msgbuf;
    packet->temperature = temperature;
    packet->flag = flag;
    mav_array_memcpy(packet->gyro_offset, gyro_offset, sizeof(float)*3);
    mav_array_memcpy(packet->acc_norm, acc_norm, sizeof(float)*3);
    mav_array_memcpy(packet->acc_matrix, acc_matrix, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_INFO, (const char *)packet, MAVLINK_MSG_ID_IMU_INFO_MIN_LEN, MAVLINK_MSG_ID_IMU_INFO_LEN, MAVLINK_MSG_ID_IMU_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU_INFO UNPACKING


/**
 * @brief Get field temperature from imu_info message
 *
 * @return  IMU temperature in Celsius.
 */
static inline float mavlink_msg_imu_info_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field gyro_offset from imu_info message
 *
 * @return  Gyroscope offset [X, Y, Z].
 */
static inline uint16_t mavlink_msg_imu_info_get_gyro_offset(const mavlink_message_t* msg, float *gyro_offset)
{
    return _MAV_RETURN_float_array(msg, gyro_offset, 3,  4);
}

/**
 * @brief Get field acc_norm from imu_info message
 *
 * @return  Accelerometer offset [X, Y, Z].
 */
static inline uint16_t mavlink_msg_imu_info_get_acc_norm(const mavlink_message_t* msg, float *acc_norm)
{
    return _MAV_RETURN_float_array(msg, acc_norm, 3,  16);
}

/**
 * @brief Get field acc_matrix from imu_info message
 *
 * @return  3x3 accelerometer calibration matrix (row-major).
 */
static inline uint16_t mavlink_msg_imu_info_get_acc_matrix(const mavlink_message_t* msg, float *acc_matrix)
{
    return _MAV_RETURN_float_array(msg, acc_matrix, 9,  28);
}

/**
 * @brief Get field flag from imu_info message
 *
 * @return  Calibration complete flag (1 = done).
 */
static inline uint8_t mavlink_msg_imu_info_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Decode a imu_info message into a struct
 *
 * @param msg The message to decode
 * @param imu_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_info_decode(const mavlink_message_t* msg, mavlink_imu_info_t* imu_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu_info->temperature = mavlink_msg_imu_info_get_temperature(msg);
    mavlink_msg_imu_info_get_gyro_offset(msg, imu_info->gyro_offset);
    mavlink_msg_imu_info_get_acc_norm(msg, imu_info->acc_norm);
    mavlink_msg_imu_info_get_acc_matrix(msg, imu_info->acc_matrix);
    imu_info->flag = mavlink_msg_imu_info_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_INFO_LEN? msg->len : MAVLINK_MSG_ID_IMU_INFO_LEN;
        memset(imu_info, 0, MAVLINK_MSG_ID_IMU_INFO_LEN);
    memcpy(imu_info, _MAV_PAYLOAD(msg), len);
#endif
}
