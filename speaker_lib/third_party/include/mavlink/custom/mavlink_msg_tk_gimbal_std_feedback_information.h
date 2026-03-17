#pragma once
// MESSAGE TK_GIMBAL_STD_FEEDBACK_INFORMATION PACKING

#define MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION 51003


typedef struct __mavlink_tk_gimbal_std_feedback_information_t {
 int32_t target_longitude; /*< [degE7]  Target longitude, unit [degE7].*/
 int32_t target_latitude; /*< [degE7]  Target latitude, unit [degE7].*/
 int32_t longitude; /*< [degE7]  Aircraft longitude, unit [degE7].*/
 int32_t latitude; /*< [degE7]  Aircraft latitude, unit [degE7].*/
 int16_t pitch_frame_angle; /*< [0.1X]  Pitch frame angle, unit 0.1 degree.*/
 int16_t yaw_frame_angle; /*< [0.1X]  Yaw frame angle, unit 0.1 degree.*/
 int16_t roll_frame_angle; /*< [0.1X]  Roll frame angle, unit 0.1 degree.*/
 int16_t x_deviation_quantity; /*< [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).*/
 int16_t y_deviation_quantity; /*< [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).*/
 uint16_t laser_distance; /*< [dm]  Laser ranging distance, unit dm.*/
 uint16_t zoom; /*< [0.1X]  Zoom ratio, unit 0.1X.*/
 int16_t target_altitude; /*< [dm]  Target altitude, unit [dm].*/
 uint16_t target_distance; /*< [dm]  Target horizontal distance, unit [dm].*/
 uint16_t altitude; /*< [dm]  Aircraft altitude, unit [dm].*/
 int16_t infrared_temperature_max; /*< [0.1X]  Maximum infrared temperature, unit 0.1X.*/
 int16_t infrared_temperature_min; /*< [0.1X]  Minimum infrared temperature, unit 0.1X.*/
 int16_t infrared_temperature_avg; /*< [0.1X]  Average infrared temperature, unit 0.1X.*/
 int16_t x_Motor_angular_velocity; /*< [0.1X]  X-axis motor angular velocity, unit 0.1X.*/
 int16_t y_Motor_angular_velocity; /*< [0.1X]  Y-axis motor angular velocity, unit 0.1X.*/
 int16_t z_Motor_angular_velocity; /*< [0.1X]  Z-axis motor angular velocity, unit 0.1X.*/
 uint8_t gimbal_status_one; /*<   .*/
 uint8_t gimbal_status_two; /*<   .*/
 uint8_t gimbal_status_three; /*<   .*/
} mavlink_tk_gimbal_std_feedback_information_t;

#define MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN 51
#define MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN 51
#define MAVLINK_MSG_ID_51003_LEN 51
#define MAVLINK_MSG_ID_51003_MIN_LEN 51

#define MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC 249
#define MAVLINK_MSG_ID_51003_CRC 249



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_GIMBAL_STD_FEEDBACK_INFORMATION { \
    51003, \
    "TK_GIMBAL_STD_FEEDBACK_INFORMATION", \
    23, \
    {  { "gimbal_status_one", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_one) }, \
         { "gimbal_status_two", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_two) }, \
         { "gimbal_status_three", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_three) }, \
         { "pitch_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_tk_gimbal_std_feedback_information_t, pitch_frame_angle) }, \
         { "yaw_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_tk_gimbal_std_feedback_information_t, yaw_frame_angle) }, \
         { "roll_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_tk_gimbal_std_feedback_information_t, roll_frame_angle) }, \
         { "x_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_tk_gimbal_std_feedback_information_t, x_deviation_quantity) }, \
         { "y_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_tk_gimbal_std_feedback_information_t, y_deviation_quantity) }, \
         { "laser_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_tk_gimbal_std_feedback_information_t, laser_distance) }, \
         { "zoom", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_tk_gimbal_std_feedback_information_t, zoom) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_longitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_latitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_altitude) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_distance) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tk_gimbal_std_feedback_information_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tk_gimbal_std_feedback_information_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_tk_gimbal_std_feedback_information_t, altitude) }, \
         { "infrared_temperature_max", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_max) }, \
         { "infrared_temperature_min", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_min) }, \
         { "infrared_temperature_avg", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_avg) }, \
         { "x_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_tk_gimbal_std_feedback_information_t, x_Motor_angular_velocity) }, \
         { "y_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_tk_gimbal_std_feedback_information_t, y_Motor_angular_velocity) }, \
         { "z_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_tk_gimbal_std_feedback_information_t, z_Motor_angular_velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_GIMBAL_STD_FEEDBACK_INFORMATION { \
    "TK_GIMBAL_STD_FEEDBACK_INFORMATION", \
    23, \
    {  { "gimbal_status_one", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_one) }, \
         { "gimbal_status_two", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_two) }, \
         { "gimbal_status_three", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_tk_gimbal_std_feedback_information_t, gimbal_status_three) }, \
         { "pitch_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_tk_gimbal_std_feedback_information_t, pitch_frame_angle) }, \
         { "yaw_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_tk_gimbal_std_feedback_information_t, yaw_frame_angle) }, \
         { "roll_frame_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_tk_gimbal_std_feedback_information_t, roll_frame_angle) }, \
         { "x_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_tk_gimbal_std_feedback_information_t, x_deviation_quantity) }, \
         { "y_deviation_quantity", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_tk_gimbal_std_feedback_information_t, y_deviation_quantity) }, \
         { "laser_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_tk_gimbal_std_feedback_information_t, laser_distance) }, \
         { "zoom", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_tk_gimbal_std_feedback_information_t, zoom) }, \
         { "target_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_longitude) }, \
         { "target_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_latitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_altitude) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_tk_gimbal_std_feedback_information_t, target_distance) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tk_gimbal_std_feedback_information_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tk_gimbal_std_feedback_information_t, latitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_tk_gimbal_std_feedback_information_t, altitude) }, \
         { "infrared_temperature_max", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_max) }, \
         { "infrared_temperature_min", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_min) }, \
         { "infrared_temperature_avg", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_tk_gimbal_std_feedback_information_t, infrared_temperature_avg) }, \
         { "x_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_tk_gimbal_std_feedback_information_t, x_Motor_angular_velocity) }, \
         { "y_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_tk_gimbal_std_feedback_information_t, y_Motor_angular_velocity) }, \
         { "z_Motor_angular_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_tk_gimbal_std_feedback_information_t, z_Motor_angular_velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_gimbal_std_feedback_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_status_one   .
 * @param gimbal_status_two   .
 * @param gimbal_status_three   .
 * @param pitch_frame_angle [0.1X]  Pitch frame angle, unit 0.1 degree.
 * @param yaw_frame_angle [0.1X]  Yaw frame angle, unit 0.1 degree.
 * @param roll_frame_angle [0.1X]  Roll frame angle, unit 0.1 degree.
 * @param x_deviation_quantity [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param zoom [0.1X]  Zoom ratio, unit 0.1X.
 * @param target_longitude [degE7]  Target longitude, unit [degE7].
 * @param target_latitude [degE7]  Target latitude, unit [degE7].
 * @param target_altitude [dm]  Target altitude, unit [dm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @param longitude [degE7]  Aircraft longitude, unit [degE7].
 * @param latitude [degE7]  Aircraft latitude, unit [degE7].
 * @param altitude [dm]  Aircraft altitude, unit [dm].
 * @param infrared_temperature_max [0.1X]  Maximum infrared temperature, unit 0.1X.
 * @param infrared_temperature_min [0.1X]  Minimum infrared temperature, unit 0.1X.
 * @param infrared_temperature_avg [0.1X]  Average infrared temperature, unit 0.1X.
 * @param x_Motor_angular_velocity [0.1X]  X-axis motor angular velocity, unit 0.1X.
 * @param y_Motor_angular_velocity [0.1X]  Y-axis motor angular velocity, unit 0.1X.
 * @param z_Motor_angular_velocity [0.1X]  Z-axis motor angular velocity, unit 0.1X.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gimbal_status_one, uint8_t gimbal_status_two, uint8_t gimbal_status_three, int16_t pitch_frame_angle, int16_t yaw_frame_angle, int16_t roll_frame_angle, int16_t x_deviation_quantity, int16_t y_deviation_quantity, uint16_t laser_distance, uint16_t zoom, int32_t target_longitude, int32_t target_latitude, int16_t target_altitude, uint16_t target_distance, int32_t longitude, int32_t latitude, uint16_t altitude, int16_t infrared_temperature_max, int16_t infrared_temperature_min, int16_t infrared_temperature_avg, int16_t x_Motor_angular_velocity, int16_t y_Motor_angular_velocity, int16_t z_Motor_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, target_longitude);
    _mav_put_int32_t(buf, 4, target_latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, latitude);
    _mav_put_int16_t(buf, 16, pitch_frame_angle);
    _mav_put_int16_t(buf, 18, yaw_frame_angle);
    _mav_put_int16_t(buf, 20, roll_frame_angle);
    _mav_put_int16_t(buf, 22, x_deviation_quantity);
    _mav_put_int16_t(buf, 24, y_deviation_quantity);
    _mav_put_uint16_t(buf, 26, laser_distance);
    _mav_put_uint16_t(buf, 28, zoom);
    _mav_put_int16_t(buf, 30, target_altitude);
    _mav_put_uint16_t(buf, 32, target_distance);
    _mav_put_uint16_t(buf, 34, altitude);
    _mav_put_int16_t(buf, 36, infrared_temperature_max);
    _mav_put_int16_t(buf, 38, infrared_temperature_min);
    _mav_put_int16_t(buf, 40, infrared_temperature_avg);
    _mav_put_int16_t(buf, 42, x_Motor_angular_velocity);
    _mav_put_int16_t(buf, 44, y_Motor_angular_velocity);
    _mav_put_int16_t(buf, 46, z_Motor_angular_velocity);
    _mav_put_uint8_t(buf, 48, gimbal_status_one);
    _mav_put_uint8_t(buf, 49, gimbal_status_two);
    _mav_put_uint8_t(buf, 50, gimbal_status_three);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_std_feedback_information_t packet;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.pitch_frame_angle = pitch_frame_angle;
    packet.yaw_frame_angle = yaw_frame_angle;
    packet.roll_frame_angle = roll_frame_angle;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.laser_distance = laser_distance;
    packet.zoom = zoom;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.altitude = altitude;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;
    packet.x_Motor_angular_velocity = x_Motor_angular_velocity;
    packet.y_Motor_angular_velocity = y_Motor_angular_velocity;
    packet.z_Motor_angular_velocity = z_Motor_angular_velocity;
    packet.gimbal_status_one = gimbal_status_one;
    packet.gimbal_status_two = gimbal_status_two;
    packet.gimbal_status_three = gimbal_status_three;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
}

/**
 * @brief Pack a tk_gimbal_std_feedback_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param gimbal_status_one   .
 * @param gimbal_status_two   .
 * @param gimbal_status_three   .
 * @param pitch_frame_angle [0.1X]  Pitch frame angle, unit 0.1 degree.
 * @param yaw_frame_angle [0.1X]  Yaw frame angle, unit 0.1 degree.
 * @param roll_frame_angle [0.1X]  Roll frame angle, unit 0.1 degree.
 * @param x_deviation_quantity [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param zoom [0.1X]  Zoom ratio, unit 0.1X.
 * @param target_longitude [degE7]  Target longitude, unit [degE7].
 * @param target_latitude [degE7]  Target latitude, unit [degE7].
 * @param target_altitude [dm]  Target altitude, unit [dm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @param longitude [degE7]  Aircraft longitude, unit [degE7].
 * @param latitude [degE7]  Aircraft latitude, unit [degE7].
 * @param altitude [dm]  Aircraft altitude, unit [dm].
 * @param infrared_temperature_max [0.1X]  Maximum infrared temperature, unit 0.1X.
 * @param infrared_temperature_min [0.1X]  Minimum infrared temperature, unit 0.1X.
 * @param infrared_temperature_avg [0.1X]  Average infrared temperature, unit 0.1X.
 * @param x_Motor_angular_velocity [0.1X]  X-axis motor angular velocity, unit 0.1X.
 * @param y_Motor_angular_velocity [0.1X]  Y-axis motor angular velocity, unit 0.1X.
 * @param z_Motor_angular_velocity [0.1X]  Z-axis motor angular velocity, unit 0.1X.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t gimbal_status_one, uint8_t gimbal_status_two, uint8_t gimbal_status_three, int16_t pitch_frame_angle, int16_t yaw_frame_angle, int16_t roll_frame_angle, int16_t x_deviation_quantity, int16_t y_deviation_quantity, uint16_t laser_distance, uint16_t zoom, int32_t target_longitude, int32_t target_latitude, int16_t target_altitude, uint16_t target_distance, int32_t longitude, int32_t latitude, uint16_t altitude, int16_t infrared_temperature_max, int16_t infrared_temperature_min, int16_t infrared_temperature_avg, int16_t x_Motor_angular_velocity, int16_t y_Motor_angular_velocity, int16_t z_Motor_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, target_longitude);
    _mav_put_int32_t(buf, 4, target_latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, latitude);
    _mav_put_int16_t(buf, 16, pitch_frame_angle);
    _mav_put_int16_t(buf, 18, yaw_frame_angle);
    _mav_put_int16_t(buf, 20, roll_frame_angle);
    _mav_put_int16_t(buf, 22, x_deviation_quantity);
    _mav_put_int16_t(buf, 24, y_deviation_quantity);
    _mav_put_uint16_t(buf, 26, laser_distance);
    _mav_put_uint16_t(buf, 28, zoom);
    _mav_put_int16_t(buf, 30, target_altitude);
    _mav_put_uint16_t(buf, 32, target_distance);
    _mav_put_uint16_t(buf, 34, altitude);
    _mav_put_int16_t(buf, 36, infrared_temperature_max);
    _mav_put_int16_t(buf, 38, infrared_temperature_min);
    _mav_put_int16_t(buf, 40, infrared_temperature_avg);
    _mav_put_int16_t(buf, 42, x_Motor_angular_velocity);
    _mav_put_int16_t(buf, 44, y_Motor_angular_velocity);
    _mav_put_int16_t(buf, 46, z_Motor_angular_velocity);
    _mav_put_uint8_t(buf, 48, gimbal_status_one);
    _mav_put_uint8_t(buf, 49, gimbal_status_two);
    _mav_put_uint8_t(buf, 50, gimbal_status_three);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_std_feedback_information_t packet;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.pitch_frame_angle = pitch_frame_angle;
    packet.yaw_frame_angle = yaw_frame_angle;
    packet.roll_frame_angle = roll_frame_angle;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.laser_distance = laser_distance;
    packet.zoom = zoom;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.altitude = altitude;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;
    packet.x_Motor_angular_velocity = x_Motor_angular_velocity;
    packet.y_Motor_angular_velocity = y_Motor_angular_velocity;
    packet.z_Motor_angular_velocity = z_Motor_angular_velocity;
    packet.gimbal_status_one = gimbal_status_one;
    packet.gimbal_status_two = gimbal_status_two;
    packet.gimbal_status_three = gimbal_status_three;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a tk_gimbal_std_feedback_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_status_one   .
 * @param gimbal_status_two   .
 * @param gimbal_status_three   .
 * @param pitch_frame_angle [0.1X]  Pitch frame angle, unit 0.1 degree.
 * @param yaw_frame_angle [0.1X]  Yaw frame angle, unit 0.1 degree.
 * @param roll_frame_angle [0.1X]  Roll frame angle, unit 0.1 degree.
 * @param x_deviation_quantity [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param zoom [0.1X]  Zoom ratio, unit 0.1X.
 * @param target_longitude [degE7]  Target longitude, unit [degE7].
 * @param target_latitude [degE7]  Target latitude, unit [degE7].
 * @param target_altitude [dm]  Target altitude, unit [dm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @param longitude [degE7]  Aircraft longitude, unit [degE7].
 * @param latitude [degE7]  Aircraft latitude, unit [degE7].
 * @param altitude [dm]  Aircraft altitude, unit [dm].
 * @param infrared_temperature_max [0.1X]  Maximum infrared temperature, unit 0.1X.
 * @param infrared_temperature_min [0.1X]  Minimum infrared temperature, unit 0.1X.
 * @param infrared_temperature_avg [0.1X]  Average infrared temperature, unit 0.1X.
 * @param x_Motor_angular_velocity [0.1X]  X-axis motor angular velocity, unit 0.1X.
 * @param y_Motor_angular_velocity [0.1X]  Y-axis motor angular velocity, unit 0.1X.
 * @param z_Motor_angular_velocity [0.1X]  Z-axis motor angular velocity, unit 0.1X.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gimbal_status_one,uint8_t gimbal_status_two,uint8_t gimbal_status_three,int16_t pitch_frame_angle,int16_t yaw_frame_angle,int16_t roll_frame_angle,int16_t x_deviation_quantity,int16_t y_deviation_quantity,uint16_t laser_distance,uint16_t zoom,int32_t target_longitude,int32_t target_latitude,int16_t target_altitude,uint16_t target_distance,int32_t longitude,int32_t latitude,uint16_t altitude,int16_t infrared_temperature_max,int16_t infrared_temperature_min,int16_t infrared_temperature_avg,int16_t x_Motor_angular_velocity,int16_t y_Motor_angular_velocity,int16_t z_Motor_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, target_longitude);
    _mav_put_int32_t(buf, 4, target_latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, latitude);
    _mav_put_int16_t(buf, 16, pitch_frame_angle);
    _mav_put_int16_t(buf, 18, yaw_frame_angle);
    _mav_put_int16_t(buf, 20, roll_frame_angle);
    _mav_put_int16_t(buf, 22, x_deviation_quantity);
    _mav_put_int16_t(buf, 24, y_deviation_quantity);
    _mav_put_uint16_t(buf, 26, laser_distance);
    _mav_put_uint16_t(buf, 28, zoom);
    _mav_put_int16_t(buf, 30, target_altitude);
    _mav_put_uint16_t(buf, 32, target_distance);
    _mav_put_uint16_t(buf, 34, altitude);
    _mav_put_int16_t(buf, 36, infrared_temperature_max);
    _mav_put_int16_t(buf, 38, infrared_temperature_min);
    _mav_put_int16_t(buf, 40, infrared_temperature_avg);
    _mav_put_int16_t(buf, 42, x_Motor_angular_velocity);
    _mav_put_int16_t(buf, 44, y_Motor_angular_velocity);
    _mav_put_int16_t(buf, 46, z_Motor_angular_velocity);
    _mav_put_uint8_t(buf, 48, gimbal_status_one);
    _mav_put_uint8_t(buf, 49, gimbal_status_two);
    _mav_put_uint8_t(buf, 50, gimbal_status_three);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_std_feedback_information_t packet;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.pitch_frame_angle = pitch_frame_angle;
    packet.yaw_frame_angle = yaw_frame_angle;
    packet.roll_frame_angle = roll_frame_angle;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.laser_distance = laser_distance;
    packet.zoom = zoom;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.altitude = altitude;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;
    packet.x_Motor_angular_velocity = x_Motor_angular_velocity;
    packet.y_Motor_angular_velocity = y_Motor_angular_velocity;
    packet.z_Motor_angular_velocity = z_Motor_angular_velocity;
    packet.gimbal_status_one = gimbal_status_one;
    packet.gimbal_status_two = gimbal_status_two;
    packet.gimbal_status_three = gimbal_status_three;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
}

/**
 * @brief Encode a tk_gimbal_std_feedback_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_std_feedback_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_gimbal_std_feedback_information_t* tk_gimbal_std_feedback_information)
{
    return mavlink_msg_tk_gimbal_std_feedback_information_pack(system_id, component_id, msg, tk_gimbal_std_feedback_information->gimbal_status_one, tk_gimbal_std_feedback_information->gimbal_status_two, tk_gimbal_std_feedback_information->gimbal_status_three, tk_gimbal_std_feedback_information->pitch_frame_angle, tk_gimbal_std_feedback_information->yaw_frame_angle, tk_gimbal_std_feedback_information->roll_frame_angle, tk_gimbal_std_feedback_information->x_deviation_quantity, tk_gimbal_std_feedback_information->y_deviation_quantity, tk_gimbal_std_feedback_information->laser_distance, tk_gimbal_std_feedback_information->zoom, tk_gimbal_std_feedback_information->target_longitude, tk_gimbal_std_feedback_information->target_latitude, tk_gimbal_std_feedback_information->target_altitude, tk_gimbal_std_feedback_information->target_distance, tk_gimbal_std_feedback_information->longitude, tk_gimbal_std_feedback_information->latitude, tk_gimbal_std_feedback_information->altitude, tk_gimbal_std_feedback_information->infrared_temperature_max, tk_gimbal_std_feedback_information->infrared_temperature_min, tk_gimbal_std_feedback_information->infrared_temperature_avg, tk_gimbal_std_feedback_information->x_Motor_angular_velocity, tk_gimbal_std_feedback_information->y_Motor_angular_velocity, tk_gimbal_std_feedback_information->z_Motor_angular_velocity);
}

/**
 * @brief Encode a tk_gimbal_std_feedback_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_std_feedback_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_gimbal_std_feedback_information_t* tk_gimbal_std_feedback_information)
{
    return mavlink_msg_tk_gimbal_std_feedback_information_pack_chan(system_id, component_id, chan, msg, tk_gimbal_std_feedback_information->gimbal_status_one, tk_gimbal_std_feedback_information->gimbal_status_two, tk_gimbal_std_feedback_information->gimbal_status_three, tk_gimbal_std_feedback_information->pitch_frame_angle, tk_gimbal_std_feedback_information->yaw_frame_angle, tk_gimbal_std_feedback_information->roll_frame_angle, tk_gimbal_std_feedback_information->x_deviation_quantity, tk_gimbal_std_feedback_information->y_deviation_quantity, tk_gimbal_std_feedback_information->laser_distance, tk_gimbal_std_feedback_information->zoom, tk_gimbal_std_feedback_information->target_longitude, tk_gimbal_std_feedback_information->target_latitude, tk_gimbal_std_feedback_information->target_altitude, tk_gimbal_std_feedback_information->target_distance, tk_gimbal_std_feedback_information->longitude, tk_gimbal_std_feedback_information->latitude, tk_gimbal_std_feedback_information->altitude, tk_gimbal_std_feedback_information->infrared_temperature_max, tk_gimbal_std_feedback_information->infrared_temperature_min, tk_gimbal_std_feedback_information->infrared_temperature_avg, tk_gimbal_std_feedback_information->x_Motor_angular_velocity, tk_gimbal_std_feedback_information->y_Motor_angular_velocity, tk_gimbal_std_feedback_information->z_Motor_angular_velocity);
}

/**
 * @brief Encode a tk_gimbal_std_feedback_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_std_feedback_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_gimbal_std_feedback_information_t* tk_gimbal_std_feedback_information)
{
    return mavlink_msg_tk_gimbal_std_feedback_information_pack_status(system_id, component_id, _status, msg,  tk_gimbal_std_feedback_information->gimbal_status_one, tk_gimbal_std_feedback_information->gimbal_status_two, tk_gimbal_std_feedback_information->gimbal_status_three, tk_gimbal_std_feedback_information->pitch_frame_angle, tk_gimbal_std_feedback_information->yaw_frame_angle, tk_gimbal_std_feedback_information->roll_frame_angle, tk_gimbal_std_feedback_information->x_deviation_quantity, tk_gimbal_std_feedback_information->y_deviation_quantity, tk_gimbal_std_feedback_information->laser_distance, tk_gimbal_std_feedback_information->zoom, tk_gimbal_std_feedback_information->target_longitude, tk_gimbal_std_feedback_information->target_latitude, tk_gimbal_std_feedback_information->target_altitude, tk_gimbal_std_feedback_information->target_distance, tk_gimbal_std_feedback_information->longitude, tk_gimbal_std_feedback_information->latitude, tk_gimbal_std_feedback_information->altitude, tk_gimbal_std_feedback_information->infrared_temperature_max, tk_gimbal_std_feedback_information->infrared_temperature_min, tk_gimbal_std_feedback_information->infrared_temperature_avg, tk_gimbal_std_feedback_information->x_Motor_angular_velocity, tk_gimbal_std_feedback_information->y_Motor_angular_velocity, tk_gimbal_std_feedback_information->z_Motor_angular_velocity);
}

/**
 * @brief Send a tk_gimbal_std_feedback_information message
 * @param chan MAVLink channel to send the message
 *
 * @param gimbal_status_one   .
 * @param gimbal_status_two   .
 * @param gimbal_status_three   .
 * @param pitch_frame_angle [0.1X]  Pitch frame angle, unit 0.1 degree.
 * @param yaw_frame_angle [0.1X]  Yaw frame angle, unit 0.1 degree.
 * @param roll_frame_angle [0.1X]  Roll frame angle, unit 0.1 degree.
 * @param x_deviation_quantity [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 * @param y_deviation_quantity [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 * @param laser_distance [dm]  Laser ranging distance, unit dm.
 * @param zoom [0.1X]  Zoom ratio, unit 0.1X.
 * @param target_longitude [degE7]  Target longitude, unit [degE7].
 * @param target_latitude [degE7]  Target latitude, unit [degE7].
 * @param target_altitude [dm]  Target altitude, unit [dm].
 * @param target_distance [dm]  Target horizontal distance, unit [dm].
 * @param longitude [degE7]  Aircraft longitude, unit [degE7].
 * @param latitude [degE7]  Aircraft latitude, unit [degE7].
 * @param altitude [dm]  Aircraft altitude, unit [dm].
 * @param infrared_temperature_max [0.1X]  Maximum infrared temperature, unit 0.1X.
 * @param infrared_temperature_min [0.1X]  Minimum infrared temperature, unit 0.1X.
 * @param infrared_temperature_avg [0.1X]  Average infrared temperature, unit 0.1X.
 * @param x_Motor_angular_velocity [0.1X]  X-axis motor angular velocity, unit 0.1X.
 * @param y_Motor_angular_velocity [0.1X]  Y-axis motor angular velocity, unit 0.1X.
 * @param z_Motor_angular_velocity [0.1X]  Z-axis motor angular velocity, unit 0.1X.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_gimbal_std_feedback_information_send(mavlink_channel_t chan, uint8_t gimbal_status_one, uint8_t gimbal_status_two, uint8_t gimbal_status_three, int16_t pitch_frame_angle, int16_t yaw_frame_angle, int16_t roll_frame_angle, int16_t x_deviation_quantity, int16_t y_deviation_quantity, uint16_t laser_distance, uint16_t zoom, int32_t target_longitude, int32_t target_latitude, int16_t target_altitude, uint16_t target_distance, int32_t longitude, int32_t latitude, uint16_t altitude, int16_t infrared_temperature_max, int16_t infrared_temperature_min, int16_t infrared_temperature_avg, int16_t x_Motor_angular_velocity, int16_t y_Motor_angular_velocity, int16_t z_Motor_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN];
    _mav_put_int32_t(buf, 0, target_longitude);
    _mav_put_int32_t(buf, 4, target_latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, latitude);
    _mav_put_int16_t(buf, 16, pitch_frame_angle);
    _mav_put_int16_t(buf, 18, yaw_frame_angle);
    _mav_put_int16_t(buf, 20, roll_frame_angle);
    _mav_put_int16_t(buf, 22, x_deviation_quantity);
    _mav_put_int16_t(buf, 24, y_deviation_quantity);
    _mav_put_uint16_t(buf, 26, laser_distance);
    _mav_put_uint16_t(buf, 28, zoom);
    _mav_put_int16_t(buf, 30, target_altitude);
    _mav_put_uint16_t(buf, 32, target_distance);
    _mav_put_uint16_t(buf, 34, altitude);
    _mav_put_int16_t(buf, 36, infrared_temperature_max);
    _mav_put_int16_t(buf, 38, infrared_temperature_min);
    _mav_put_int16_t(buf, 40, infrared_temperature_avg);
    _mav_put_int16_t(buf, 42, x_Motor_angular_velocity);
    _mav_put_int16_t(buf, 44, y_Motor_angular_velocity);
    _mav_put_int16_t(buf, 46, z_Motor_angular_velocity);
    _mav_put_uint8_t(buf, 48, gimbal_status_one);
    _mav_put_uint8_t(buf, 49, gimbal_status_two);
    _mav_put_uint8_t(buf, 50, gimbal_status_three);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION, buf, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#else
    mavlink_tk_gimbal_std_feedback_information_t packet;
    packet.target_longitude = target_longitude;
    packet.target_latitude = target_latitude;
    packet.longitude = longitude;
    packet.latitude = latitude;
    packet.pitch_frame_angle = pitch_frame_angle;
    packet.yaw_frame_angle = yaw_frame_angle;
    packet.roll_frame_angle = roll_frame_angle;
    packet.x_deviation_quantity = x_deviation_quantity;
    packet.y_deviation_quantity = y_deviation_quantity;
    packet.laser_distance = laser_distance;
    packet.zoom = zoom;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.altitude = altitude;
    packet.infrared_temperature_max = infrared_temperature_max;
    packet.infrared_temperature_min = infrared_temperature_min;
    packet.infrared_temperature_avg = infrared_temperature_avg;
    packet.x_Motor_angular_velocity = x_Motor_angular_velocity;
    packet.y_Motor_angular_velocity = y_Motor_angular_velocity;
    packet.z_Motor_angular_velocity = z_Motor_angular_velocity;
    packet.gimbal_status_one = gimbal_status_one;
    packet.gimbal_status_two = gimbal_status_two;
    packet.gimbal_status_three = gimbal_status_three;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a tk_gimbal_std_feedback_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_gimbal_std_feedback_information_send_struct(mavlink_channel_t chan, const mavlink_tk_gimbal_std_feedback_information_t* tk_gimbal_std_feedback_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_gimbal_std_feedback_information_send(chan, tk_gimbal_std_feedback_information->gimbal_status_one, tk_gimbal_std_feedback_information->gimbal_status_two, tk_gimbal_std_feedback_information->gimbal_status_three, tk_gimbal_std_feedback_information->pitch_frame_angle, tk_gimbal_std_feedback_information->yaw_frame_angle, tk_gimbal_std_feedback_information->roll_frame_angle, tk_gimbal_std_feedback_information->x_deviation_quantity, tk_gimbal_std_feedback_information->y_deviation_quantity, tk_gimbal_std_feedback_information->laser_distance, tk_gimbal_std_feedback_information->zoom, tk_gimbal_std_feedback_information->target_longitude, tk_gimbal_std_feedback_information->target_latitude, tk_gimbal_std_feedback_information->target_altitude, tk_gimbal_std_feedback_information->target_distance, tk_gimbal_std_feedback_information->longitude, tk_gimbal_std_feedback_information->latitude, tk_gimbal_std_feedback_information->altitude, tk_gimbal_std_feedback_information->infrared_temperature_max, tk_gimbal_std_feedback_information->infrared_temperature_min, tk_gimbal_std_feedback_information->infrared_temperature_avg, tk_gimbal_std_feedback_information->x_Motor_angular_velocity, tk_gimbal_std_feedback_information->y_Motor_angular_velocity, tk_gimbal_std_feedback_information->z_Motor_angular_velocity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION, (const char *)tk_gimbal_std_feedback_information, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_gimbal_std_feedback_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gimbal_status_one, uint8_t gimbal_status_two, uint8_t gimbal_status_three, int16_t pitch_frame_angle, int16_t yaw_frame_angle, int16_t roll_frame_angle, int16_t x_deviation_quantity, int16_t y_deviation_quantity, uint16_t laser_distance, uint16_t zoom, int32_t target_longitude, int32_t target_latitude, int16_t target_altitude, uint16_t target_distance, int32_t longitude, int32_t latitude, uint16_t altitude, int16_t infrared_temperature_max, int16_t infrared_temperature_min, int16_t infrared_temperature_avg, int16_t x_Motor_angular_velocity, int16_t y_Motor_angular_velocity, int16_t z_Motor_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, target_longitude);
    _mav_put_int32_t(buf, 4, target_latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int32_t(buf, 12, latitude);
    _mav_put_int16_t(buf, 16, pitch_frame_angle);
    _mav_put_int16_t(buf, 18, yaw_frame_angle);
    _mav_put_int16_t(buf, 20, roll_frame_angle);
    _mav_put_int16_t(buf, 22, x_deviation_quantity);
    _mav_put_int16_t(buf, 24, y_deviation_quantity);
    _mav_put_uint16_t(buf, 26, laser_distance);
    _mav_put_uint16_t(buf, 28, zoom);
    _mav_put_int16_t(buf, 30, target_altitude);
    _mav_put_uint16_t(buf, 32, target_distance);
    _mav_put_uint16_t(buf, 34, altitude);
    _mav_put_int16_t(buf, 36, infrared_temperature_max);
    _mav_put_int16_t(buf, 38, infrared_temperature_min);
    _mav_put_int16_t(buf, 40, infrared_temperature_avg);
    _mav_put_int16_t(buf, 42, x_Motor_angular_velocity);
    _mav_put_int16_t(buf, 44, y_Motor_angular_velocity);
    _mav_put_int16_t(buf, 46, z_Motor_angular_velocity);
    _mav_put_uint8_t(buf, 48, gimbal_status_one);
    _mav_put_uint8_t(buf, 49, gimbal_status_two);
    _mav_put_uint8_t(buf, 50, gimbal_status_three);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION, buf, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#else
    mavlink_tk_gimbal_std_feedback_information_t *packet = (mavlink_tk_gimbal_std_feedback_information_t *)msgbuf;
    packet->target_longitude = target_longitude;
    packet->target_latitude = target_latitude;
    packet->longitude = longitude;
    packet->latitude = latitude;
    packet->pitch_frame_angle = pitch_frame_angle;
    packet->yaw_frame_angle = yaw_frame_angle;
    packet->roll_frame_angle = roll_frame_angle;
    packet->x_deviation_quantity = x_deviation_quantity;
    packet->y_deviation_quantity = y_deviation_quantity;
    packet->laser_distance = laser_distance;
    packet->zoom = zoom;
    packet->target_altitude = target_altitude;
    packet->target_distance = target_distance;
    packet->altitude = altitude;
    packet->infrared_temperature_max = infrared_temperature_max;
    packet->infrared_temperature_min = infrared_temperature_min;
    packet->infrared_temperature_avg = infrared_temperature_avg;
    packet->x_Motor_angular_velocity = x_Motor_angular_velocity;
    packet->y_Motor_angular_velocity = y_Motor_angular_velocity;
    packet->z_Motor_angular_velocity = z_Motor_angular_velocity;
    packet->gimbal_status_one = gimbal_status_one;
    packet->gimbal_status_two = gimbal_status_two;
    packet->gimbal_status_three = gimbal_status_three;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_GIMBAL_STD_FEEDBACK_INFORMATION UNPACKING


/**
 * @brief Get field gimbal_status_one from tk_gimbal_std_feedback_information message
 *
 * @return   .
 */
static inline uint8_t mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_one(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field gimbal_status_two from tk_gimbal_std_feedback_information message
 *
 * @return   .
 */
static inline uint8_t mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_two(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field gimbal_status_three from tk_gimbal_std_feedback_information message
 *
 * @return   .
 */
static inline uint8_t mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_three(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field pitch_frame_angle from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Pitch frame angle, unit 0.1 degree.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_pitch_frame_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field yaw_frame_angle from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Yaw frame angle, unit 0.1 degree.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_yaw_frame_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field roll_frame_angle from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Roll frame angle, unit 0.1 degree.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_roll_frame_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field x_deviation_quantity from tk_gimbal_std_feedback_information message
 *
 * @return [ Pixels]  X-axis deviation amount, unit pixels, plus or minus 960 (image center as origin).
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_x_deviation_quantity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field y_deviation_quantity from tk_gimbal_std_feedback_information message
 *
 * @return [ Pixels]  Y-axis deviation amount, unit pixels, plus or minus 540 (image center as origin).
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_y_deviation_quantity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field laser_distance from tk_gimbal_std_feedback_information message
 *
 * @return [dm]  Laser ranging distance, unit dm.
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_get_laser_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field zoom from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Zoom ratio, unit 0.1X.
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_get_zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field target_longitude from tk_gimbal_std_feedback_information message
 *
 * @return [degE7]  Target longitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_gimbal_std_feedback_information_get_target_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field target_latitude from tk_gimbal_std_feedback_information message
 *
 * @return [degE7]  Target latitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_gimbal_std_feedback_information_get_target_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field target_altitude from tk_gimbal_std_feedback_information message
 *
 * @return [dm]  Target altitude, unit [dm].
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_target_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field target_distance from tk_gimbal_std_feedback_information message
 *
 * @return [dm]  Target horizontal distance, unit [dm].
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_get_target_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field longitude from tk_gimbal_std_feedback_information message
 *
 * @return [degE7]  Aircraft longitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_gimbal_std_feedback_information_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field latitude from tk_gimbal_std_feedback_information message
 *
 * @return [degE7]  Aircraft latitude, unit [degE7].
 */
static inline int32_t mavlink_msg_tk_gimbal_std_feedback_information_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field altitude from tk_gimbal_std_feedback_information message
 *
 * @return [dm]  Aircraft altitude, unit [dm].
 */
static inline uint16_t mavlink_msg_tk_gimbal_std_feedback_information_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field infrared_temperature_max from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Maximum infrared temperature, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field infrared_temperature_min from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Minimum infrared temperature, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field infrared_temperature_avg from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Average infrared temperature, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_avg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field x_Motor_angular_velocity from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  X-axis motor angular velocity, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_x_Motor_angular_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field y_Motor_angular_velocity from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Y-axis motor angular velocity, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_y_Motor_angular_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field z_Motor_angular_velocity from tk_gimbal_std_feedback_information message
 *
 * @return [0.1X]  Z-axis motor angular velocity, unit 0.1X.
 */
static inline int16_t mavlink_msg_tk_gimbal_std_feedback_information_get_z_Motor_angular_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Decode a tk_gimbal_std_feedback_information message into a struct
 *
 * @param msg The message to decode
 * @param tk_gimbal_std_feedback_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_gimbal_std_feedback_information_decode(const mavlink_message_t* msg, mavlink_tk_gimbal_std_feedback_information_t* tk_gimbal_std_feedback_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tk_gimbal_std_feedback_information->target_longitude = mavlink_msg_tk_gimbal_std_feedback_information_get_target_longitude(msg);
    tk_gimbal_std_feedback_information->target_latitude = mavlink_msg_tk_gimbal_std_feedback_information_get_target_latitude(msg);
    tk_gimbal_std_feedback_information->longitude = mavlink_msg_tk_gimbal_std_feedback_information_get_longitude(msg);
    tk_gimbal_std_feedback_information->latitude = mavlink_msg_tk_gimbal_std_feedback_information_get_latitude(msg);
    tk_gimbal_std_feedback_information->pitch_frame_angle = mavlink_msg_tk_gimbal_std_feedback_information_get_pitch_frame_angle(msg);
    tk_gimbal_std_feedback_information->yaw_frame_angle = mavlink_msg_tk_gimbal_std_feedback_information_get_yaw_frame_angle(msg);
    tk_gimbal_std_feedback_information->roll_frame_angle = mavlink_msg_tk_gimbal_std_feedback_information_get_roll_frame_angle(msg);
    tk_gimbal_std_feedback_information->x_deviation_quantity = mavlink_msg_tk_gimbal_std_feedback_information_get_x_deviation_quantity(msg);
    tk_gimbal_std_feedback_information->y_deviation_quantity = mavlink_msg_tk_gimbal_std_feedback_information_get_y_deviation_quantity(msg);
    tk_gimbal_std_feedback_information->laser_distance = mavlink_msg_tk_gimbal_std_feedback_information_get_laser_distance(msg);
    tk_gimbal_std_feedback_information->zoom = mavlink_msg_tk_gimbal_std_feedback_information_get_zoom(msg);
    tk_gimbal_std_feedback_information->target_altitude = mavlink_msg_tk_gimbal_std_feedback_information_get_target_altitude(msg);
    tk_gimbal_std_feedback_information->target_distance = mavlink_msg_tk_gimbal_std_feedback_information_get_target_distance(msg);
    tk_gimbal_std_feedback_information->altitude = mavlink_msg_tk_gimbal_std_feedback_information_get_altitude(msg);
    tk_gimbal_std_feedback_information->infrared_temperature_max = mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_max(msg);
    tk_gimbal_std_feedback_information->infrared_temperature_min = mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_min(msg);
    tk_gimbal_std_feedback_information->infrared_temperature_avg = mavlink_msg_tk_gimbal_std_feedback_information_get_infrared_temperature_avg(msg);
    tk_gimbal_std_feedback_information->x_Motor_angular_velocity = mavlink_msg_tk_gimbal_std_feedback_information_get_x_Motor_angular_velocity(msg);
    tk_gimbal_std_feedback_information->y_Motor_angular_velocity = mavlink_msg_tk_gimbal_std_feedback_information_get_y_Motor_angular_velocity(msg);
    tk_gimbal_std_feedback_information->z_Motor_angular_velocity = mavlink_msg_tk_gimbal_std_feedback_information_get_z_Motor_angular_velocity(msg);
    tk_gimbal_std_feedback_information->gimbal_status_one = mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_one(msg);
    tk_gimbal_std_feedback_information->gimbal_status_two = mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_two(msg);
    tk_gimbal_std_feedback_information->gimbal_status_three = mavlink_msg_tk_gimbal_std_feedback_information_get_gimbal_status_three(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN;
        memset(tk_gimbal_std_feedback_information, 0, MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_LEN);
    memcpy(tk_gimbal_std_feedback_information, _MAV_PAYLOAD(msg), len);
#endif
}
