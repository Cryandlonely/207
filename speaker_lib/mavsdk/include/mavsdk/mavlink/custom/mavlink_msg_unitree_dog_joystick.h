#pragma once
// MESSAGE UNITREE_DOG_JOYSTICK PACKING

#define MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK 51800


typedef struct __mavlink_unitree_dog_joystick_t {
 float Lx; /*<  Joystick channel values.*/
 float Ly; /*<  Joystick channel values.*/
 float Rx; /*<  Joystick channel values.*/
 float Ry; /*<  Joystick channel values.*/
} mavlink_unitree_dog_joystick_t;

#define MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN 16
#define MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN 16
#define MAVLINK_MSG_ID_51800_LEN 16
#define MAVLINK_MSG_ID_51800_MIN_LEN 16

#define MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC 240
#define MAVLINK_MSG_ID_51800_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UNITREE_DOG_JOYSTICK { \
    51800, \
    "UNITREE_DOG_JOYSTICK", \
    4, \
    {  { "Lx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_unitree_dog_joystick_t, Lx) }, \
         { "Ly", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_unitree_dog_joystick_t, Ly) }, \
         { "Rx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_unitree_dog_joystick_t, Rx) }, \
         { "Ry", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_unitree_dog_joystick_t, Ry) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UNITREE_DOG_JOYSTICK { \
    "UNITREE_DOG_JOYSTICK", \
    4, \
    {  { "Lx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_unitree_dog_joystick_t, Lx) }, \
         { "Ly", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_unitree_dog_joystick_t, Ly) }, \
         { "Rx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_unitree_dog_joystick_t, Rx) }, \
         { "Ry", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_unitree_dog_joystick_t, Ry) }, \
         } \
}
#endif

/**
 * @brief Pack a unitree_dog_joystick message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Lx  Joystick channel values.
 * @param Ly  Joystick channel values.
 * @param Rx  Joystick channel values.
 * @param Ry  Joystick channel values.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float Lx, float Ly, float Rx, float Ry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN];
    _mav_put_float(buf, 0, Lx);
    _mav_put_float(buf, 4, Ly);
    _mav_put_float(buf, 8, Rx);
    _mav_put_float(buf, 12, Ry);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#else
    mavlink_unitree_dog_joystick_t packet;
    packet.Lx = Lx;
    packet.Ly = Ly;
    packet.Rx = Rx;
    packet.Ry = Ry;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
}

/**
 * @brief Pack a unitree_dog_joystick message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param Lx  Joystick channel values.
 * @param Ly  Joystick channel values.
 * @param Rx  Joystick channel values.
 * @param Ry  Joystick channel values.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float Lx, float Ly, float Rx, float Ry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN];
    _mav_put_float(buf, 0, Lx);
    _mav_put_float(buf, 4, Ly);
    _mav_put_float(buf, 8, Rx);
    _mav_put_float(buf, 12, Ry);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#else
    mavlink_unitree_dog_joystick_t packet;
    packet.Lx = Lx;
    packet.Ly = Ly;
    packet.Rx = Rx;
    packet.Ry = Ry;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#endif
}

/**
 * @brief Pack a unitree_dog_joystick message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Lx  Joystick channel values.
 * @param Ly  Joystick channel values.
 * @param Rx  Joystick channel values.
 * @param Ry  Joystick channel values.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float Lx,float Ly,float Rx,float Ry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN];
    _mav_put_float(buf, 0, Lx);
    _mav_put_float(buf, 4, Ly);
    _mav_put_float(buf, 8, Rx);
    _mav_put_float(buf, 12, Ry);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#else
    mavlink_unitree_dog_joystick_t packet;
    packet.Lx = Lx;
    packet.Ly = Ly;
    packet.Rx = Rx;
    packet.Ry = Ry;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
}

/**
 * @brief Encode a unitree_dog_joystick struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param unitree_dog_joystick C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_unitree_dog_joystick_t* unitree_dog_joystick)
{
    return mavlink_msg_unitree_dog_joystick_pack(system_id, component_id, msg, unitree_dog_joystick->Lx, unitree_dog_joystick->Ly, unitree_dog_joystick->Rx, unitree_dog_joystick->Ry);
}

/**
 * @brief Encode a unitree_dog_joystick struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param unitree_dog_joystick C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_unitree_dog_joystick_t* unitree_dog_joystick)
{
    return mavlink_msg_unitree_dog_joystick_pack_chan(system_id, component_id, chan, msg, unitree_dog_joystick->Lx, unitree_dog_joystick->Ly, unitree_dog_joystick->Rx, unitree_dog_joystick->Ry);
}

/**
 * @brief Encode a unitree_dog_joystick struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param unitree_dog_joystick C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_unitree_dog_joystick_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_unitree_dog_joystick_t* unitree_dog_joystick)
{
    return mavlink_msg_unitree_dog_joystick_pack_status(system_id, component_id, _status, msg,  unitree_dog_joystick->Lx, unitree_dog_joystick->Ly, unitree_dog_joystick->Rx, unitree_dog_joystick->Ry);
}

/**
 * @brief Send a unitree_dog_joystick message
 * @param chan MAVLink channel to send the message
 *
 * @param Lx  Joystick channel values.
 * @param Ly  Joystick channel values.
 * @param Rx  Joystick channel values.
 * @param Ry  Joystick channel values.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_unitree_dog_joystick_send(mavlink_channel_t chan, float Lx, float Ly, float Rx, float Ry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN];
    _mav_put_float(buf, 0, Lx);
    _mav_put_float(buf, 4, Ly);
    _mav_put_float(buf, 8, Rx);
    _mav_put_float(buf, 12, Ry);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK, buf, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#else
    mavlink_unitree_dog_joystick_t packet;
    packet.Lx = Lx;
    packet.Ly = Ly;
    packet.Rx = Rx;
    packet.Ry = Ry;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK, (const char *)&packet, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#endif
}

/**
 * @brief Send a unitree_dog_joystick message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_unitree_dog_joystick_send_struct(mavlink_channel_t chan, const mavlink_unitree_dog_joystick_t* unitree_dog_joystick)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_unitree_dog_joystick_send(chan, unitree_dog_joystick->Lx, unitree_dog_joystick->Ly, unitree_dog_joystick->Rx, unitree_dog_joystick->Ry);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK, (const char *)unitree_dog_joystick, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#endif
}

#if MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_unitree_dog_joystick_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float Lx, float Ly, float Rx, float Ry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Lx);
    _mav_put_float(buf, 4, Ly);
    _mav_put_float(buf, 8, Rx);
    _mav_put_float(buf, 12, Ry);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK, buf, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#else
    mavlink_unitree_dog_joystick_t *packet = (mavlink_unitree_dog_joystick_t *)msgbuf;
    packet->Lx = Lx;
    packet->Ly = Ly;
    packet->Rx = Rx;
    packet->Ry = Ry;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK, (const char *)packet, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_CRC);
#endif
}
#endif

#endif

// MESSAGE UNITREE_DOG_JOYSTICK UNPACKING


/**
 * @brief Get field Lx from unitree_dog_joystick message
 *
 * @return  Joystick channel values.
 */
static inline float mavlink_msg_unitree_dog_joystick_get_Lx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Ly from unitree_dog_joystick message
 *
 * @return  Joystick channel values.
 */
static inline float mavlink_msg_unitree_dog_joystick_get_Ly(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Rx from unitree_dog_joystick message
 *
 * @return  Joystick channel values.
 */
static inline float mavlink_msg_unitree_dog_joystick_get_Rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Ry from unitree_dog_joystick message
 *
 * @return  Joystick channel values.
 */
static inline float mavlink_msg_unitree_dog_joystick_get_Ry(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a unitree_dog_joystick message into a struct
 *
 * @param msg The message to decode
 * @param unitree_dog_joystick C-struct to decode the message contents into
 */
static inline void mavlink_msg_unitree_dog_joystick_decode(const mavlink_message_t* msg, mavlink_unitree_dog_joystick_t* unitree_dog_joystick)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    unitree_dog_joystick->Lx = mavlink_msg_unitree_dog_joystick_get_Lx(msg);
    unitree_dog_joystick->Ly = mavlink_msg_unitree_dog_joystick_get_Ly(msg);
    unitree_dog_joystick->Rx = mavlink_msg_unitree_dog_joystick_get_Rx(msg);
    unitree_dog_joystick->Ry = mavlink_msg_unitree_dog_joystick_get_Ry(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN? msg->len : MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN;
        memset(unitree_dog_joystick, 0, MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_LEN);
    memcpy(unitree_dog_joystick, _MAV_PAYLOAD(msg), len);
#endif
}
