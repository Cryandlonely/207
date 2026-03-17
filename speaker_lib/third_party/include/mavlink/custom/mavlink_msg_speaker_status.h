#pragma once
// MESSAGE SPEAKER_STATUS PACKING

#define MAVLINK_MSG_ID_SPEAKER_STATUS 20301


typedef struct __mavlink_speaker_status_t {
 float playing_time_s; /*<  Elapsed time since starting the audio playing (in seconds).*/
 float current_volume; /*<  Current volume level (in persentage).*/
 int32_t network_module_signal_quality; /*<  Current network signal quality.*/
 float network_module_latency; /*<  Current network latency (in milliseconds).*/
 float roll; /*<  Value of speaker roll.*/
 float pitch; /*<  Value of speaker pitch.*/
 float yaw; /*<  Value of speaker yaw.*/
 uint8_t play_mode; /*<  Play audio mode.*/
 uint8_t play_status; /*<  Whether audio playing is currently in process.*/
 uint8_t light_mode; /*<  Mode of speaker light.*/
 uint8_t light_on_off; /*<  On/off of speaker light.*/
 uint8_t real_time_listen_status; /*<  Status of real time listen.*/
 uint8_t real_time_speak_status; /*<  Status of real time speak.*/
} mavlink_speaker_status_t;

#define MAVLINK_MSG_ID_SPEAKER_STATUS_LEN 34
#define MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN 34
#define MAVLINK_MSG_ID_20301_LEN 34
#define MAVLINK_MSG_ID_20301_MIN_LEN 34

#define MAVLINK_MSG_ID_SPEAKER_STATUS_CRC 121
#define MAVLINK_MSG_ID_20301_CRC 121



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPEAKER_STATUS { \
    20301, \
    "SPEAKER_STATUS", \
    13, \
    {  { "play_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_speaker_status_t, play_mode) }, \
         { "play_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_speaker_status_t, play_status) }, \
         { "playing_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_speaker_status_t, playing_time_s) }, \
         { "current_volume", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speaker_status_t, current_volume) }, \
         { "network_module_signal_quality", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_speaker_status_t, network_module_signal_quality) }, \
         { "network_module_latency", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_speaker_status_t, network_module_latency) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_speaker_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_speaker_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_speaker_status_t, yaw) }, \
         { "light_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_speaker_status_t, light_mode) }, \
         { "light_on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_speaker_status_t, light_on_off) }, \
         { "real_time_listen_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_speaker_status_t, real_time_listen_status) }, \
         { "real_time_speak_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_speaker_status_t, real_time_speak_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPEAKER_STATUS { \
    "SPEAKER_STATUS", \
    13, \
    {  { "play_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_speaker_status_t, play_mode) }, \
         { "play_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_speaker_status_t, play_status) }, \
         { "playing_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_speaker_status_t, playing_time_s) }, \
         { "current_volume", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speaker_status_t, current_volume) }, \
         { "network_module_signal_quality", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_speaker_status_t, network_module_signal_quality) }, \
         { "network_module_latency", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_speaker_status_t, network_module_latency) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_speaker_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_speaker_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_speaker_status_t, yaw) }, \
         { "light_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_speaker_status_t, light_mode) }, \
         { "light_on_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_speaker_status_t, light_on_off) }, \
         { "real_time_listen_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_speaker_status_t, real_time_listen_status) }, \
         { "real_time_speak_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_speaker_status_t, real_time_speak_status) }, \
         } \
}
#endif

/**
 * @brief Pack a speaker_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param play_mode  Play audio mode.
 * @param play_status  Whether audio playing is currently in process.
 * @param playing_time_s  Elapsed time since starting the audio playing (in seconds).
 * @param current_volume  Current volume level (in persentage).
 * @param network_module_signal_quality  Current network signal quality.
 * @param network_module_latency  Current network latency (in milliseconds).
 * @param roll  Value of speaker roll.
 * @param pitch  Value of speaker pitch.
 * @param yaw  Value of speaker yaw.
 * @param light_mode  Mode of speaker light.
 * @param light_on_off  On/off of speaker light.
 * @param real_time_listen_status  Status of real time listen.
 * @param real_time_speak_status  Status of real time speak.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t play_mode, uint8_t play_status, float playing_time_s, float current_volume, int32_t network_module_signal_quality, float network_module_latency, float roll, float pitch, float yaw, uint8_t light_mode, uint8_t light_on_off, uint8_t real_time_listen_status, uint8_t real_time_speak_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_STATUS_LEN];
    _mav_put_float(buf, 0, playing_time_s);
    _mav_put_float(buf, 4, current_volume);
    _mav_put_int32_t(buf, 8, network_module_signal_quality);
    _mav_put_float(buf, 12, network_module_latency);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, play_mode);
    _mav_put_uint8_t(buf, 29, play_status);
    _mav_put_uint8_t(buf, 30, light_mode);
    _mav_put_uint8_t(buf, 31, light_on_off);
    _mav_put_uint8_t(buf, 32, real_time_listen_status);
    _mav_put_uint8_t(buf, 33, real_time_speak_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#else
    mavlink_speaker_status_t packet;
    packet.playing_time_s = playing_time_s;
    packet.current_volume = current_volume;
    packet.network_module_signal_quality = network_module_signal_quality;
    packet.network_module_latency = network_module_latency;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.play_mode = play_mode;
    packet.play_status = play_status;
    packet.light_mode = light_mode;
    packet.light_on_off = light_on_off;
    packet.real_time_listen_status = real_time_listen_status;
    packet.real_time_speak_status = real_time_speak_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
}

/**
 * @brief Pack a speaker_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param play_mode  Play audio mode.
 * @param play_status  Whether audio playing is currently in process.
 * @param playing_time_s  Elapsed time since starting the audio playing (in seconds).
 * @param current_volume  Current volume level (in persentage).
 * @param network_module_signal_quality  Current network signal quality.
 * @param network_module_latency  Current network latency (in milliseconds).
 * @param roll  Value of speaker roll.
 * @param pitch  Value of speaker pitch.
 * @param yaw  Value of speaker yaw.
 * @param light_mode  Mode of speaker light.
 * @param light_on_off  On/off of speaker light.
 * @param real_time_listen_status  Status of real time listen.
 * @param real_time_speak_status  Status of real time speak.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t play_mode, uint8_t play_status, float playing_time_s, float current_volume, int32_t network_module_signal_quality, float network_module_latency, float roll, float pitch, float yaw, uint8_t light_mode, uint8_t light_on_off, uint8_t real_time_listen_status, uint8_t real_time_speak_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_STATUS_LEN];
    _mav_put_float(buf, 0, playing_time_s);
    _mav_put_float(buf, 4, current_volume);
    _mav_put_int32_t(buf, 8, network_module_signal_quality);
    _mav_put_float(buf, 12, network_module_latency);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, play_mode);
    _mav_put_uint8_t(buf, 29, play_status);
    _mav_put_uint8_t(buf, 30, light_mode);
    _mav_put_uint8_t(buf, 31, light_on_off);
    _mav_put_uint8_t(buf, 32, real_time_listen_status);
    _mav_put_uint8_t(buf, 33, real_time_speak_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#else
    mavlink_speaker_status_t packet;
    packet.playing_time_s = playing_time_s;
    packet.current_volume = current_volume;
    packet.network_module_signal_quality = network_module_signal_quality;
    packet.network_module_latency = network_module_latency;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.play_mode = play_mode;
    packet.play_status = play_status;
    packet.light_mode = light_mode;
    packet.light_on_off = light_on_off;
    packet.real_time_listen_status = real_time_listen_status;
    packet.real_time_speak_status = real_time_speak_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a speaker_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param play_mode  Play audio mode.
 * @param play_status  Whether audio playing is currently in process.
 * @param playing_time_s  Elapsed time since starting the audio playing (in seconds).
 * @param current_volume  Current volume level (in persentage).
 * @param network_module_signal_quality  Current network signal quality.
 * @param network_module_latency  Current network latency (in milliseconds).
 * @param roll  Value of speaker roll.
 * @param pitch  Value of speaker pitch.
 * @param yaw  Value of speaker yaw.
 * @param light_mode  Mode of speaker light.
 * @param light_on_off  On/off of speaker light.
 * @param real_time_listen_status  Status of real time listen.
 * @param real_time_speak_status  Status of real time speak.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speaker_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t play_mode,uint8_t play_status,float playing_time_s,float current_volume,int32_t network_module_signal_quality,float network_module_latency,float roll,float pitch,float yaw,uint8_t light_mode,uint8_t light_on_off,uint8_t real_time_listen_status,uint8_t real_time_speak_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_STATUS_LEN];
    _mav_put_float(buf, 0, playing_time_s);
    _mav_put_float(buf, 4, current_volume);
    _mav_put_int32_t(buf, 8, network_module_signal_quality);
    _mav_put_float(buf, 12, network_module_latency);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, play_mode);
    _mav_put_uint8_t(buf, 29, play_status);
    _mav_put_uint8_t(buf, 30, light_mode);
    _mav_put_uint8_t(buf, 31, light_on_off);
    _mav_put_uint8_t(buf, 32, real_time_listen_status);
    _mav_put_uint8_t(buf, 33, real_time_speak_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#else
    mavlink_speaker_status_t packet;
    packet.playing_time_s = playing_time_s;
    packet.current_volume = current_volume;
    packet.network_module_signal_quality = network_module_signal_quality;
    packet.network_module_latency = network_module_latency;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.play_mode = play_mode;
    packet.play_status = play_status;
    packet.light_mode = light_mode;
    packet.light_on_off = light_on_off;
    packet.real_time_listen_status = real_time_listen_status;
    packet.real_time_speak_status = real_time_speak_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEAKER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
}

/**
 * @brief Encode a speaker_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param speaker_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_speaker_status_t* speaker_status)
{
    return mavlink_msg_speaker_status_pack(system_id, component_id, msg, speaker_status->play_mode, speaker_status->play_status, speaker_status->playing_time_s, speaker_status->current_volume, speaker_status->network_module_signal_quality, speaker_status->network_module_latency, speaker_status->roll, speaker_status->pitch, speaker_status->yaw, speaker_status->light_mode, speaker_status->light_on_off, speaker_status->real_time_listen_status, speaker_status->real_time_speak_status);
}

/**
 * @brief Encode a speaker_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speaker_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_speaker_status_t* speaker_status)
{
    return mavlink_msg_speaker_status_pack_chan(system_id, component_id, chan, msg, speaker_status->play_mode, speaker_status->play_status, speaker_status->playing_time_s, speaker_status->current_volume, speaker_status->network_module_signal_quality, speaker_status->network_module_latency, speaker_status->roll, speaker_status->pitch, speaker_status->yaw, speaker_status->light_mode, speaker_status->light_on_off, speaker_status->real_time_listen_status, speaker_status->real_time_speak_status);
}

/**
 * @brief Encode a speaker_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param speaker_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speaker_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_speaker_status_t* speaker_status)
{
    return mavlink_msg_speaker_status_pack_status(system_id, component_id, _status, msg,  speaker_status->play_mode, speaker_status->play_status, speaker_status->playing_time_s, speaker_status->current_volume, speaker_status->network_module_signal_quality, speaker_status->network_module_latency, speaker_status->roll, speaker_status->pitch, speaker_status->yaw, speaker_status->light_mode, speaker_status->light_on_off, speaker_status->real_time_listen_status, speaker_status->real_time_speak_status);
}

/**
 * @brief Send a speaker_status message
 * @param chan MAVLink channel to send the message
 *
 * @param play_mode  Play audio mode.
 * @param play_status  Whether audio playing is currently in process.
 * @param playing_time_s  Elapsed time since starting the audio playing (in seconds).
 * @param current_volume  Current volume level (in persentage).
 * @param network_module_signal_quality  Current network signal quality.
 * @param network_module_latency  Current network latency (in milliseconds).
 * @param roll  Value of speaker roll.
 * @param pitch  Value of speaker pitch.
 * @param yaw  Value of speaker yaw.
 * @param light_mode  Mode of speaker light.
 * @param light_on_off  On/off of speaker light.
 * @param real_time_listen_status  Status of real time listen.
 * @param real_time_speak_status  Status of real time speak.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_speaker_status_send(mavlink_channel_t chan, uint8_t play_mode, uint8_t play_status, float playing_time_s, float current_volume, int32_t network_module_signal_quality, float network_module_latency, float roll, float pitch, float yaw, uint8_t light_mode, uint8_t light_on_off, uint8_t real_time_listen_status, uint8_t real_time_speak_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEAKER_STATUS_LEN];
    _mav_put_float(buf, 0, playing_time_s);
    _mav_put_float(buf, 4, current_volume);
    _mav_put_int32_t(buf, 8, network_module_signal_quality);
    _mav_put_float(buf, 12, network_module_latency);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, play_mode);
    _mav_put_uint8_t(buf, 29, play_status);
    _mav_put_uint8_t(buf, 30, light_mode);
    _mav_put_uint8_t(buf, 31, light_on_off);
    _mav_put_uint8_t(buf, 32, real_time_listen_status);
    _mav_put_uint8_t(buf, 33, real_time_speak_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_STATUS, buf, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#else
    mavlink_speaker_status_t packet;
    packet.playing_time_s = playing_time_s;
    packet.current_volume = current_volume;
    packet.network_module_signal_quality = network_module_signal_quality;
    packet.network_module_latency = network_module_latency;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.play_mode = play_mode;
    packet.play_status = play_status;
    packet.light_mode = light_mode;
    packet.light_on_off = light_on_off;
    packet.real_time_listen_status = real_time_listen_status;
    packet.real_time_speak_status = real_time_speak_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#endif
}

/**
 * @brief Send a speaker_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_speaker_status_send_struct(mavlink_channel_t chan, const mavlink_speaker_status_t* speaker_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_speaker_status_send(chan, speaker_status->play_mode, speaker_status->play_status, speaker_status->playing_time_s, speaker_status->current_volume, speaker_status->network_module_signal_quality, speaker_status->network_module_latency, speaker_status->roll, speaker_status->pitch, speaker_status->yaw, speaker_status->light_mode, speaker_status->light_on_off, speaker_status->real_time_listen_status, speaker_status->real_time_speak_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_STATUS, (const char *)speaker_status, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPEAKER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_speaker_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t play_mode, uint8_t play_status, float playing_time_s, float current_volume, int32_t network_module_signal_quality, float network_module_latency, float roll, float pitch, float yaw, uint8_t light_mode, uint8_t light_on_off, uint8_t real_time_listen_status, uint8_t real_time_speak_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, playing_time_s);
    _mav_put_float(buf, 4, current_volume);
    _mav_put_int32_t(buf, 8, network_module_signal_quality);
    _mav_put_float(buf, 12, network_module_latency);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, play_mode);
    _mav_put_uint8_t(buf, 29, play_status);
    _mav_put_uint8_t(buf, 30, light_mode);
    _mav_put_uint8_t(buf, 31, light_on_off);
    _mav_put_uint8_t(buf, 32, real_time_listen_status);
    _mav_put_uint8_t(buf, 33, real_time_speak_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_STATUS, buf, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#else
    mavlink_speaker_status_t *packet = (mavlink_speaker_status_t *)msgbuf;
    packet->playing_time_s = playing_time_s;
    packet->current_volume = current_volume;
    packet->network_module_signal_quality = network_module_signal_quality;
    packet->network_module_latency = network_module_latency;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->play_mode = play_mode;
    packet->play_status = play_status;
    packet->light_mode = light_mode;
    packet->light_on_off = light_on_off;
    packet->real_time_listen_status = real_time_listen_status;
    packet->real_time_speak_status = real_time_speak_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEAKER_STATUS, (const char *)packet, MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN, MAVLINK_MSG_ID_SPEAKER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SPEAKER_STATUS UNPACKING


/**
 * @brief Get field play_mode from speaker_status message
 *
 * @return  Play audio mode.
 */
static inline uint8_t mavlink_msg_speaker_status_get_play_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field play_status from speaker_status message
 *
 * @return  Whether audio playing is currently in process.
 */
static inline uint8_t mavlink_msg_speaker_status_get_play_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field playing_time_s from speaker_status message
 *
 * @return  Elapsed time since starting the audio playing (in seconds).
 */
static inline float mavlink_msg_speaker_status_get_playing_time_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_volume from speaker_status message
 *
 * @return  Current volume level (in persentage).
 */
static inline float mavlink_msg_speaker_status_get_current_volume(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field network_module_signal_quality from speaker_status message
 *
 * @return  Current network signal quality.
 */
static inline int32_t mavlink_msg_speaker_status_get_network_module_signal_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field network_module_latency from speaker_status message
 *
 * @return  Current network latency (in milliseconds).
 */
static inline float mavlink_msg_speaker_status_get_network_module_latency(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from speaker_status message
 *
 * @return  Value of speaker roll.
 */
static inline float mavlink_msg_speaker_status_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from speaker_status message
 *
 * @return  Value of speaker pitch.
 */
static inline float mavlink_msg_speaker_status_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from speaker_status message
 *
 * @return  Value of speaker yaw.
 */
static inline float mavlink_msg_speaker_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field light_mode from speaker_status message
 *
 * @return  Mode of speaker light.
 */
static inline uint8_t mavlink_msg_speaker_status_get_light_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field light_on_off from speaker_status message
 *
 * @return  On/off of speaker light.
 */
static inline uint8_t mavlink_msg_speaker_status_get_light_on_off(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field real_time_listen_status from speaker_status message
 *
 * @return  Status of real time listen.
 */
static inline uint8_t mavlink_msg_speaker_status_get_real_time_listen_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field real_time_speak_status from speaker_status message
 *
 * @return  Status of real time speak.
 */
static inline uint8_t mavlink_msg_speaker_status_get_real_time_speak_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Decode a speaker_status message into a struct
 *
 * @param msg The message to decode
 * @param speaker_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_speaker_status_decode(const mavlink_message_t* msg, mavlink_speaker_status_t* speaker_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    speaker_status->playing_time_s = mavlink_msg_speaker_status_get_playing_time_s(msg);
    speaker_status->current_volume = mavlink_msg_speaker_status_get_current_volume(msg);
    speaker_status->network_module_signal_quality = mavlink_msg_speaker_status_get_network_module_signal_quality(msg);
    speaker_status->network_module_latency = mavlink_msg_speaker_status_get_network_module_latency(msg);
    speaker_status->roll = mavlink_msg_speaker_status_get_roll(msg);
    speaker_status->pitch = mavlink_msg_speaker_status_get_pitch(msg);
    speaker_status->yaw = mavlink_msg_speaker_status_get_yaw(msg);
    speaker_status->play_mode = mavlink_msg_speaker_status_get_play_mode(msg);
    speaker_status->play_status = mavlink_msg_speaker_status_get_play_status(msg);
    speaker_status->light_mode = mavlink_msg_speaker_status_get_light_mode(msg);
    speaker_status->light_on_off = mavlink_msg_speaker_status_get_light_on_off(msg);
    speaker_status->real_time_listen_status = mavlink_msg_speaker_status_get_real_time_listen_status(msg);
    speaker_status->real_time_speak_status = mavlink_msg_speaker_status_get_real_time_speak_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPEAKER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SPEAKER_STATUS_LEN;
        memset(speaker_status, 0, MAVLINK_MSG_ID_SPEAKER_STATUS_LEN);
    memcpy(speaker_status, _MAV_PAYLOAD(msg), len);
#endif
}
