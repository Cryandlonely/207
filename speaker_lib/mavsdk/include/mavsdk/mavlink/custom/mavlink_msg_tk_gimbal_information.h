#pragma once
// MESSAGE TK_GIMBAL_INFORMATION PACKING

#define MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION 51001


typedef struct __mavlink_tk_gimbal_information_t {
 char vendor_name[32]; /*<  Name of the gimbal vendor.*/
 char model_name[32]; /*<  Name of the gimbal model.*/
 char serial_number[32]; /*<  Serial number of the gimbal.*/
 char firmware_version[32]; /*<  Version of the gimbal firmware.*/
 char protocol_version[32]; /*<  Version of the protocol firmware.*/
} mavlink_tk_gimbal_information_t;

#define MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN 160
#define MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN 160
#define MAVLINK_MSG_ID_51001_LEN 160
#define MAVLINK_MSG_ID_51001_MIN_LEN 160

#define MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC 24
#define MAVLINK_MSG_ID_51001_CRC 24

#define MAVLINK_MSG_TK_GIMBAL_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_TK_GIMBAL_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_TK_GIMBAL_INFORMATION_FIELD_SERIAL_NUMBER_LEN 32
#define MAVLINK_MSG_TK_GIMBAL_INFORMATION_FIELD_FIRMWARE_VERSION_LEN 32
#define MAVLINK_MSG_TK_GIMBAL_INFORMATION_FIELD_PROTOCOL_VERSION_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TK_GIMBAL_INFORMATION { \
    51001, \
    "TK_GIMBAL_INFORMATION", \
    5, \
    {  { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_tk_gimbal_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 32, offsetof(mavlink_tk_gimbal_information_t, model_name) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 64, offsetof(mavlink_tk_gimbal_information_t, serial_number) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 96, offsetof(mavlink_tk_gimbal_information_t, firmware_version) }, \
         { "protocol_version", NULL, MAVLINK_TYPE_CHAR, 32, 128, offsetof(mavlink_tk_gimbal_information_t, protocol_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TK_GIMBAL_INFORMATION { \
    "TK_GIMBAL_INFORMATION", \
    5, \
    {  { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_tk_gimbal_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 32, offsetof(mavlink_tk_gimbal_information_t, model_name) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 32, 64, offsetof(mavlink_tk_gimbal_information_t, serial_number) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 96, offsetof(mavlink_tk_gimbal_information_t, firmware_version) }, \
         { "protocol_version", NULL, MAVLINK_TYPE_CHAR, 32, 128, offsetof(mavlink_tk_gimbal_information_t, protocol_version) }, \
         } \
}
#endif

/**
 * @brief Pack a tk_gimbal_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param serial_number  Serial number of the gimbal.
 * @param firmware_version  Version of the gimbal firmware.
 * @param protocol_version  Version of the protocol firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version, const char *protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_put_char_array(buf, 128, protocol_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.protocol_version, protocol_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
}

/**
 * @brief Pack a tk_gimbal_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param serial_number  Serial number of the gimbal.
 * @param firmware_version  Version of the gimbal firmware.
 * @param protocol_version  Version of the protocol firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version, const char *protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_put_char_array(buf, 128, protocol_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.protocol_version, protocol_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a tk_gimbal_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param serial_number  Serial number of the gimbal.
 * @param firmware_version  Version of the gimbal firmware.
 * @param protocol_version  Version of the protocol firmware.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *vendor_name,const char *model_name,const char *serial_number,const char *firmware_version,const char *protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_put_char_array(buf, 128, protocol_version, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#else
    mavlink_tk_gimbal_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.protocol_version, protocol_version, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
}

/**
 * @brief Encode a tk_gimbal_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tk_gimbal_information_t* tk_gimbal_information)
{
    return mavlink_msg_tk_gimbal_information_pack(system_id, component_id, msg, tk_gimbal_information->vendor_name, tk_gimbal_information->model_name, tk_gimbal_information->serial_number, tk_gimbal_information->firmware_version, tk_gimbal_information->protocol_version);
}

/**
 * @brief Encode a tk_gimbal_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tk_gimbal_information_t* tk_gimbal_information)
{
    return mavlink_msg_tk_gimbal_information_pack_chan(system_id, component_id, chan, msg, tk_gimbal_information->vendor_name, tk_gimbal_information->model_name, tk_gimbal_information->serial_number, tk_gimbal_information->firmware_version, tk_gimbal_information->protocol_version);
}

/**
 * @brief Encode a tk_gimbal_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tk_gimbal_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tk_gimbal_information_t* tk_gimbal_information)
{
    return mavlink_msg_tk_gimbal_information_pack_status(system_id, component_id, _status, msg,  tk_gimbal_information->vendor_name, tk_gimbal_information->model_name, tk_gimbal_information->serial_number, tk_gimbal_information->firmware_version, tk_gimbal_information->protocol_version);
}

/**
 * @brief Send a tk_gimbal_information message
 * @param chan MAVLink channel to send the message
 *
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param serial_number  Serial number of the gimbal.
 * @param firmware_version  Version of the gimbal firmware.
 * @param protocol_version  Version of the protocol firmware.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tk_gimbal_information_send(mavlink_channel_t chan, const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version, const char *protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN];

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_put_char_array(buf, 128, protocol_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION, buf, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#else
    mavlink_tk_gimbal_information_t packet;

    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.protocol_version, protocol_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a tk_gimbal_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tk_gimbal_information_send_struct(mavlink_channel_t chan, const mavlink_tk_gimbal_information_t* tk_gimbal_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_gimbal_information_send(chan, tk_gimbal_information->vendor_name, tk_gimbal_information->model_name, tk_gimbal_information->serial_number, tk_gimbal_information->firmware_version, tk_gimbal_information->protocol_version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION, (const char *)tk_gimbal_information, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tk_gimbal_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *vendor_name, const char *model_name, const char *serial_number, const char *firmware_version, const char *protocol_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, vendor_name, 32);
    _mav_put_char_array(buf, 32, model_name, 32);
    _mav_put_char_array(buf, 64, serial_number, 32);
    _mav_put_char_array(buf, 96, firmware_version, 32);
    _mav_put_char_array(buf, 128, protocol_version, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION, buf, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#else
    mavlink_tk_gimbal_information_t *packet = (mavlink_tk_gimbal_information_t *)msgbuf;

    mav_array_memcpy(packet->vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet->model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet->serial_number, serial_number, sizeof(char)*32);
    mav_array_memcpy(packet->firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet->protocol_version, protocol_version, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE TK_GIMBAL_INFORMATION UNPACKING


/**
 * @brief Get field vendor_name from tk_gimbal_information message
 *
 * @return  Name of the gimbal vendor.
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_get_vendor_name(const mavlink_message_t* msg, char *vendor_name)
{
    return _MAV_RETURN_char_array(msg, vendor_name, 32,  0);
}

/**
 * @brief Get field model_name from tk_gimbal_information message
 *
 * @return  Name of the gimbal model.
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_get_model_name(const mavlink_message_t* msg, char *model_name)
{
    return _MAV_RETURN_char_array(msg, model_name, 32,  32);
}

/**
 * @brief Get field serial_number from tk_gimbal_information message
 *
 * @return  Serial number of the gimbal.
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 32,  64);
}

/**
 * @brief Get field firmware_version from tk_gimbal_information message
 *
 * @return  Version of the gimbal firmware.
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_get_firmware_version(const mavlink_message_t* msg, char *firmware_version)
{
    return _MAV_RETURN_char_array(msg, firmware_version, 32,  96);
}

/**
 * @brief Get field protocol_version from tk_gimbal_information message
 *
 * @return  Version of the protocol firmware.
 */
static inline uint16_t mavlink_msg_tk_gimbal_information_get_protocol_version(const mavlink_message_t* msg, char *protocol_version)
{
    return _MAV_RETURN_char_array(msg, protocol_version, 32,  128);
}

/**
 * @brief Decode a tk_gimbal_information message into a struct
 *
 * @param msg The message to decode
 * @param tk_gimbal_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_tk_gimbal_information_decode(const mavlink_message_t* msg, mavlink_tk_gimbal_information_t* tk_gimbal_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tk_gimbal_information_get_vendor_name(msg, tk_gimbal_information->vendor_name);
    mavlink_msg_tk_gimbal_information_get_model_name(msg, tk_gimbal_information->model_name);
    mavlink_msg_tk_gimbal_information_get_serial_number(msg, tk_gimbal_information->serial_number);
    mavlink_msg_tk_gimbal_information_get_firmware_version(msg, tk_gimbal_information->firmware_version);
    mavlink_msg_tk_gimbal_information_get_protocol_version(msg, tk_gimbal_information->protocol_version);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN;
        memset(tk_gimbal_information, 0, MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_LEN);
    memcpy(tk_gimbal_information, _MAV_PAYLOAD(msg), len);
#endif
}
