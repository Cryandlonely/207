#pragma once
// MESSAGE LIGHT_INFORMATION PACKING

#define MAVLINK_MSG_ID_LIGHT_INFORMATION 20500


typedef struct __mavlink_light_information_t {
 char vendor_name[32]; /*<  Name of the speaker vendor.*/
 char model_name[32]; /*<  Name of the speaker model.*/
 char serial_number[32]; /*<  Serial number of the speaker.*/
 char firmware_version[32]; /*<  Version of the speaker firmware.*/
} mavlink_light_information_t;

#define MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN 128
#define MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN 128
#define MAVLINK_MSG_ID_20500_LEN 128
#define MAVLINK_MSG_ID_20500_MIN_LEN 128

#define MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC 174
#define MAVLINK_MSG_ID_20500_CRC 174

#define MAVLINK_MSG_LIGHT_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_LIGHT_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_LIGHT_INFORMATION_FIELD_SERIAL_NUMBER_LEN 32
#define MAVLINK_MSG_LIGHT_INFORMATION_FIELD_FIRMWARE_VERSION_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LIGHT_INFORMATION { \
    20500, \
    "LIGHT_INFORMATION", \
    4, \
    {  { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_light_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 32, offsetof(mavlink_light_information_t, model_name) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 64, offsetof(mavlink_light_information_t, serial_number) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 96, offsetof(mavlink_light_information_t, firmware_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LIGHT_INFORMATION { \
    "LIGHT_INFORMATION", \
    4, \
    {  { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_light_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 32, offsetof(mavlink_light_information_t, model_name) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 64, offsetof(mavlink_light_information_t, serial_number) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 96, offsetof(mavlink_light_information_t, firmware_version) }, \
         } \
}
#endif

/**
 * @brief Pack a light_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vendor_name  Name of the speaker vendor.
 * @param model_name  Name of the speaker model.
 * @param serial_number  Serial number of the speaker.
 * @param firmware_version  Version of the speaker firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#else
    mavlink_light_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
}

/**
 * @brief Pack a light_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vendor_name  Name of the speaker vendor.
 * @param model_name  Name of the speaker model.
 * @param serial_number  Serial number of the speaker.
 * @param firmware_version  Version of the speaker firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#else
    mavlink_light_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a light_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vendor_name  Name of the speaker vendor.
 * @param model_name  Name of the speaker model.
 * @param serial_number  Serial number of the speaker.
 * @param firmware_version  Version of the speaker firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_light_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *vendor_name,const char *model_name,const char *serial_number,const char *firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#else
    mavlink_light_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LIGHT_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
}

/**
 * @brief Encode a light_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param light_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_light_information_t* light_information)
{
    return mavlink_msg_light_information_pack(system_id, component_id, msg, light_information->vendor_name, light_information->model_name, light_information->serial_number, light_information->firmware_version);
}

/**
 * @brief Encode a light_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param light_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_light_information_t* light_information)
{
    return mavlink_msg_light_information_pack_chan(system_id, component_id, chan, msg, light_information->vendor_name, light_information->model_name, light_information->serial_number, light_information->firmware_version);
}

/**
 * @brief Encode a light_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param light_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_light_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_light_information_t* light_information)
{
    return mavlink_msg_light_information_pack_status(system_id, component_id, _status, msg,  light_information->vendor_name, light_information->model_name, light_information->serial_number, light_information->firmware_version);
}

/**
 * @brief Send a light_information message
 * @param chan MAVLink channel to send the message
 *
 * @param vendor_name  Name of the speaker vendor.
 * @param model_name  Name of the speaker model.
 * @param serial_number  Serial number of the speaker.
 * @param firmware_version  Version of the speaker firmware.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_light_information_send(mavlink_channel_t chan, const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_INFORMATION, buf, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#else
    mavlink_light_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a light_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_light_information_send_struct(mavlink_channel_t chan, const mavlink_light_information_t* light_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_light_information_send(chan, light_information->vendor_name, light_information->model_name, light_information->serial_number, light_information->firmware_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_INFORMATION, (const char *)light_information, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_light_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_INFORMATION, buf, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#else
    mavlink_light_information_t *packet = (mavlink_light_information_t *)msgbuf;

    mav_array_memcpy(packet->vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet->model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet->serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet->firmware_version, firmware_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LIGHT_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_LIGHT_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE LIGHT_INFORMATION UNPACKING


/**
 * @brief Get field vendor_name from light_information message
 *
 * @return  Name of the speaker vendor.
 */
static inline uint16_t mavlink_msg_light_information_get_vendor_name(const mavlink_message_t* msg, char *vendor_name)
{
    return _MAV_RETURN_char_array(msg, vendor_name, 32,  0);
}

/**
 * @brief Get field model_name from light_information message
 *
 * @return  Name of the speaker model.
 */
static inline uint16_t mavlink_msg_light_information_get_model_name(const mavlink_message_t* msg, char *model_name)
{
    return _MAV_RETURN_char_array(msg, model_name, 32,  32);
}

/**
 * @brief Get field serial_number from light_information message
 *
 * @return  Serial number of the speaker.
 */
static inline uint16_t mavlink_msg_light_information_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 32,  64);
}

/**
 * @brief Get field firmware_version from light_information message
 *
 * @return  Version of the speaker firmware.
 */
static inline uint16_t mavlink_msg_light_information_get_firmware_version(const mavlink_message_t* msg, char *firmware_version)
{
    return _MAV_RETURN_char_array(msg, firmware_version, 32,  96);
}

/**
 * @brief Decode a light_information message into a struct
 *
 * @param msg The message to decode
 * @param light_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_light_information_decode(const mavlink_message_t* msg, mavlink_light_information_t* light_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_light_information_get_vendor_name(msg, light_information->vendor_name);
    mavlink_msg_light_information_get_model_name(msg, light_information->model_name);
    mavlink_msg_light_information_get_serial_number(msg, light_information->serial_number);
    mavlink_msg_light_information_get_firmware_version(msg, light_information->firmware_version);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN;
        memset(light_information, 0, MAVLINK_MSG_ID_LIGHT_INFORMATION_LEN);
    memcpy(light_information, _MAV_PAYLOAD(msg), len);
#endif
}
