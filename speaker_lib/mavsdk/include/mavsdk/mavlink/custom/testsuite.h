/** @file
 *    @brief MAVLink comm protocol testsuite generated from custom.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CUSTOM_TESTSUITE_H
#define CUSTOM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_custom(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_custom(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_file_information_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FILE_INFORMATION_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_file_information_request_t packet_in = {
        963497464,"EFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO"
    };
    mavlink_file_information_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.reserved = packet_in.reserved;
        
        mav_array_memcpy(packet1.file_name, packet_in.file_name, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FILE_INFORMATION_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FILE_INFORMATION_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_information_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_file_information_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_information_request_pack(system_id, component_id, &msg , packet1.file_name , packet1.reserved );
    mavlink_msg_file_information_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_information_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.file_name , packet1.reserved );
    mavlink_msg_file_information_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_file_information_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_information_request_send(MAVLINK_COMM_1 , packet1.file_name , packet1.reserved );
    mavlink_msg_file_information_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FILE_INFORMATION_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FILE_INFORMATION_REQUEST) != NULL);
#endif
}

static void mavlink_test_log_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOG_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_log_information_t packet_in = {
        963497464,"EFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTU"
    };
    mavlink_log_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.log_level = packet_in.log_level;
        
        mav_array_memcpy(packet1.text, packet_in.text, sizeof(char)*200);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOG_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_log_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_information_pack(system_id, component_id, &msg , packet1.text , packet1.log_level );
    mavlink_msg_log_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.text , packet1.log_level );
    mavlink_msg_log_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_log_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_log_information_send(MAVLINK_COMM_1 , packet1.text , packet1.log_level );
    mavlink_msg_log_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOG_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOG_INFORMATION) != NULL);
#endif
}

static void mavlink_test_speaker_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEAKER_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speaker_information_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK","MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ","STUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
    mavlink_speaker_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(char)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(char)*32);
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEAKER_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEAKER_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speaker_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_information_pack(system_id, component_id, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_speaker_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_speaker_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speaker_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_information_send(MAVLINK_COMM_1 , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_speaker_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEAKER_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEAKER_INFORMATION) != NULL);
#endif
}

static void mavlink_test_speaker_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEAKER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speaker_status_t packet_in = {
        17.0,45.0,963497880,101.0,129.0,157.0,185.0,89,156,223,34,101,168
    };
    mavlink_speaker_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.playing_time_s = packet_in.playing_time_s;
        packet1.current_volume = packet_in.current_volume;
        packet1.network_module_signal_quality = packet_in.network_module_signal_quality;
        packet1.network_module_latency = packet_in.network_module_latency;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.play_mode = packet_in.play_mode;
        packet1.play_status = packet_in.play_status;
        packet1.light_mode = packet_in.light_mode;
        packet1.light_on_off = packet_in.light_on_off;
        packet1.real_time_listen_status = packet_in.real_time_listen_status;
        packet1.real_time_speak_status = packet_in.real_time_speak_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEAKER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speaker_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_status_pack(system_id, component_id, &msg , packet1.play_mode , packet1.play_status , packet1.playing_time_s , packet1.current_volume , packet1.network_module_signal_quality , packet1.network_module_latency , packet1.roll , packet1.pitch , packet1.yaw , packet1.light_mode , packet1.light_on_off , packet1.real_time_listen_status , packet1.real_time_speak_status );
    mavlink_msg_speaker_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.play_mode , packet1.play_status , packet1.playing_time_s , packet1.current_volume , packet1.network_module_signal_quality , packet1.network_module_latency , packet1.roll , packet1.pitch , packet1.yaw , packet1.light_mode , packet1.light_on_off , packet1.real_time_listen_status , packet1.real_time_speak_status );
    mavlink_msg_speaker_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speaker_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_status_send(MAVLINK_COMM_1 , packet1.play_mode , packet1.play_status , packet1.playing_time_s , packet1.current_volume , packet1.network_module_signal_quality , packet1.network_module_latency , packet1.roll , packet1.pitch , packet1.yaw , packet1.light_mode , packet1.light_on_off , packet1.real_time_listen_status , packet1.real_time_speak_status );
    mavlink_msg_speaker_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEAKER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEAKER_STATUS) != NULL);
#endif
}

static void mavlink_test_speaker_audio_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speaker_audio_info_t packet_in = {
        963497464,45.0,"IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRS","UVWXYZABCDEFGHI"
    };
    mavlink_speaker_audio_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.index = packet_in.index;
        packet1.length = packet_in.length;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*64);
        mav_array_memcpy(packet1.format, packet_in.format, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_audio_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speaker_audio_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_audio_info_pack(system_id, component_id, &msg , packet1.name , packet1.index , packet1.format , packet1.length );
    mavlink_msg_speaker_audio_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_audio_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.index , packet1.format , packet1.length );
    mavlink_msg_speaker_audio_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speaker_audio_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_audio_info_send(MAVLINK_COMM_1 , packet1.name , packet1.index , packet1.format , packet1.length );
    mavlink_msg_speaker_audio_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEAKER_AUDIO_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEAKER_AUDIO_INFO) != NULL);
#endif
}

static void mavlink_test_speaker_connection_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speaker_connection_status_t packet_in = {
        93372036854775807ULL,29,96,163,230
    };
    mavlink_speaker_connection_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.connection_id = packet_in.connection_id;
        packet1.operation_ack = packet_in.operation_ack;
        packet1.connection_amount = packet_in.connection_amount;
        packet1.connection_index = packet_in.connection_index;
        packet1.connection_status = packet_in.connection_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_connection_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speaker_connection_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_connection_status_pack(system_id, component_id, &msg , packet1.operation_ack , packet1.connection_amount , packet1.connection_index , packet1.connection_id , packet1.connection_status );
    mavlink_msg_speaker_connection_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_connection_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.operation_ack , packet1.connection_amount , packet1.connection_index , packet1.connection_id , packet1.connection_status );
    mavlink_msg_speaker_connection_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speaker_connection_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_connection_status_send(MAVLINK_COMM_1 , packet1.operation_ack , packet1.connection_amount , packet1.connection_index , packet1.connection_id , packet1.connection_status );
    mavlink_msg_speaker_connection_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEAKER_CONNECTION_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEAKER_CONNECTION_STATUS) != NULL);
#endif
}

static void mavlink_test_speaker_camera_network_params(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speaker_camera_network_params_t packet_in = {
        5,72,139,206,17,84,151
    };
    mavlink_speaker_camera_network_params_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ip_byte_0 = packet_in.ip_byte_0;
        packet1.ip_byte_1 = packet_in.ip_byte_1;
        packet1.ip_byte_2 = packet_in.ip_byte_2;
        packet1.ip_byte_3 = packet_in.ip_byte_3;
        packet1.bitrate = packet_in.bitrate;
        packet1.resolution = packet_in.resolution;
        packet1.encoding = packet_in.encoding;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_camera_network_params_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speaker_camera_network_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_camera_network_params_pack(system_id, component_id, &msg , packet1.ip_byte_0 , packet1.ip_byte_1 , packet1.ip_byte_2 , packet1.ip_byte_3 , packet1.bitrate , packet1.resolution , packet1.encoding );
    mavlink_msg_speaker_camera_network_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_camera_network_params_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ip_byte_0 , packet1.ip_byte_1 , packet1.ip_byte_2 , packet1.ip_byte_3 , packet1.bitrate , packet1.resolution , packet1.encoding );
    mavlink_msg_speaker_camera_network_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speaker_camera_network_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speaker_camera_network_params_send(MAVLINK_COMM_1 , packet1.ip_byte_0 , packet1.ip_byte_1 , packet1.ip_byte_2 , packet1.ip_byte_3 , packet1.bitrate , packet1.resolution , packet1.encoding );
    mavlink_msg_speaker_camera_network_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEAKER_CAMERA_NETWORK_PARAMS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEAKER_CAMERA_NETWORK_PARAMS) != NULL);
#endif
}

static void mavlink_test_skylink_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SKYLINK_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_skylink_information_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK","MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ","STUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
    mavlink_skylink_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(char)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(char)*32);
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SKYLINK_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SKYLINK_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_skylink_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_information_pack(system_id, component_id, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_skylink_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_skylink_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_skylink_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_information_send(MAVLINK_COMM_1 , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_skylink_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SKYLINK_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SKYLINK_INFORMATION) != NULL);
#endif
}

static void mavlink_test_skylink_file_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_skylink_file_information_t packet_in = {
        963497464,"EFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO"
    };
    mavlink_skylink_file_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.index = packet_in.index;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_file_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_skylink_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_file_information_pack(system_id, component_id, &msg , packet1.name , packet1.index );
    mavlink_msg_skylink_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_file_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.index );
    mavlink_msg_skylink_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_skylink_file_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skylink_file_information_send(MAVLINK_COMM_1 , packet1.name , packet1.index );
    mavlink_msg_skylink_file_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SKYLINK_FILE_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SKYLINK_FILE_INFORMATION) != NULL);
#endif
}

static void mavlink_test_light_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LIGHT_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_light_information_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK","MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ","STUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
    mavlink_light_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(char)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(char)*32);
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LIGHT_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_light_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_information_pack(system_id, component_id, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_light_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_light_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_light_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_information_send(MAVLINK_COMM_1 , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version );
    mavlink_msg_light_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LIGHT_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LIGHT_INFORMATION) != NULL);
#endif
}

static void mavlink_test_light_file_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LIGHT_FILE_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_light_file_information_t packet_in = {
        963497464,"EFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNO"
    };
    mavlink_light_file_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.index = packet_in.index;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LIGHT_FILE_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LIGHT_FILE_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_file_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_light_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_file_information_pack(system_id, component_id, &msg , packet1.name , packet1.index );
    mavlink_msg_light_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_file_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.index );
    mavlink_msg_light_file_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_light_file_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_file_information_send(MAVLINK_COMM_1 , packet1.name , packet1.index );
    mavlink_msg_light_file_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LIGHT_FILE_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LIGHT_FILE_INFORMATION) != NULL);
#endif
}

static void mavlink_test_light_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LIGHT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_light_status_t packet_in = {
        963497464,45.0,29,96
    };
    mavlink_light_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        packet1.brightness = packet_in.brightness;
        packet1.on_off = packet_in.on_off;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LIGHT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_light_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_status_pack(system_id, component_id, &msg , packet1.id , packet1.on_off , packet1.mode , packet1.brightness );
    mavlink_msg_light_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.on_off , packet1.mode , packet1.brightness );
    mavlink_msg_light_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_light_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_light_status_send(MAVLINK_COMM_1 , packet1.id , packet1.on_off , packet1.mode , packet1.brightness );
    mavlink_msg_light_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LIGHT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LIGHT_STATUS) != NULL);
#endif
}

static void mavlink_test_tk_custom_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_CUSTOM_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_custom_data_t packet_in = {
        17235,139,"DEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST"
    };
    mavlink_tk_custom_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.msg_id = packet_in.msg_id;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(char)*252);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_CUSTOM_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_custom_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_custom_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_custom_data_pack(system_id, component_id, &msg , packet1.msg_id , packet1.len , packet1.data );
    mavlink_msg_tk_custom_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_custom_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.msg_id , packet1.len , packet1.data );
    mavlink_msg_tk_custom_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_custom_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_custom_data_send(MAVLINK_COMM_1 , packet1.msg_id , packet1.len , packet1.data );
    mavlink_msg_tk_custom_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_CUSTOM_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_CUSTOM_DATA) != NULL);
#endif
}

static void mavlink_test_tk_gimbal_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_gimbal_information_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJK","MNOPQRSTUVWXYZABCDEFGHIJKLMNOPQ","STUVWXYZABCDEFGHIJKLMNOPQRSTUVW","YZABCDEFGHIJKLMNOPQRSTUVWXYZABC"
    };
    mavlink_tk_gimbal_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(char)*32);
        mav_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(char)*32);
        mav_array_memcpy(packet1.serial_number, packet_in.serial_number, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        mav_array_memcpy(packet1.protocol_version, packet_in.protocol_version, sizeof(char)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_gimbal_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_information_pack(system_id, component_id, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version , packet1.protocol_version );
    mavlink_msg_tk_gimbal_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version , packet1.protocol_version );
    mavlink_msg_tk_gimbal_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_gimbal_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_information_send(MAVLINK_COMM_1 , packet1.vendor_name , packet1.model_name , packet1.serial_number , packet1.firmware_version , packet1.protocol_version );
    mavlink_msg_tk_gimbal_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_GIMBAL_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_GIMBAL_INFORMATION) != NULL);
#endif
}

static void mavlink_test_tk_rc_channels(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_RC_CHANNELS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_rc_channels_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003
    };
    mavlink_tk_rc_channels_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.chan1_raw = packet_in.chan1_raw;
        packet1.chan2_raw = packet_in.chan2_raw;
        packet1.chan3_raw = packet_in.chan3_raw;
        packet1.chan4_raw = packet_in.chan4_raw;
        packet1.chan5_raw = packet_in.chan5_raw;
        packet1.chan6_raw = packet_in.chan6_raw;
        packet1.chan7_raw = packet_in.chan7_raw;
        packet1.chan8_raw = packet_in.chan8_raw;
        packet1.chan9_raw = packet_in.chan9_raw;
        packet1.chan10_raw = packet_in.chan10_raw;
        packet1.chan11_raw = packet_in.chan11_raw;
        packet1.chan12_raw = packet_in.chan12_raw;
        packet1.chan13_raw = packet_in.chan13_raw;
        packet1.chan14_raw = packet_in.chan14_raw;
        packet1.chan15_raw = packet_in.chan15_raw;
        packet1.chan16_raw = packet_in.chan16_raw;
        packet1.chan17_raw = packet_in.chan17_raw;
        packet1.chan18_raw = packet_in.chan18_raw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_RC_CHANNELS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_RC_CHANNELS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_rc_channels_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_rc_channels_pack(system_id, component_id, &msg , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_tk_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_rc_channels_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_tk_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_rc_channels_send(MAVLINK_COMM_1 , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw );
    mavlink_msg_tk_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_RC_CHANNELS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_RC_CHANNELS) != NULL);
#endif
}

static void mavlink_test_tk_gimbal_std_feedback_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_gimbal_std_feedback_information_t packet_in = {
        963497464,963497672,963497880,963498088,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315,19419,19523,19627,149,216,27
    };
    mavlink_tk_gimbal_std_feedback_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_longitude = packet_in.target_longitude;
        packet1.target_latitude = packet_in.target_latitude;
        packet1.longitude = packet_in.longitude;
        packet1.latitude = packet_in.latitude;
        packet1.pitch_frame_angle = packet_in.pitch_frame_angle;
        packet1.yaw_frame_angle = packet_in.yaw_frame_angle;
        packet1.roll_frame_angle = packet_in.roll_frame_angle;
        packet1.x_deviation_quantity = packet_in.x_deviation_quantity;
        packet1.y_deviation_quantity = packet_in.y_deviation_quantity;
        packet1.laser_distance = packet_in.laser_distance;
        packet1.zoom = packet_in.zoom;
        packet1.target_altitude = packet_in.target_altitude;
        packet1.target_distance = packet_in.target_distance;
        packet1.altitude = packet_in.altitude;
        packet1.infrared_temperature_max = packet_in.infrared_temperature_max;
        packet1.infrared_temperature_min = packet_in.infrared_temperature_min;
        packet1.infrared_temperature_avg = packet_in.infrared_temperature_avg;
        packet1.x_Motor_angular_velocity = packet_in.x_Motor_angular_velocity;
        packet1.y_Motor_angular_velocity = packet_in.y_Motor_angular_velocity;
        packet1.z_Motor_angular_velocity = packet_in.z_Motor_angular_velocity;
        packet1.gimbal_status_one = packet_in.gimbal_status_one;
        packet1.gimbal_status_two = packet_in.gimbal_status_two;
        packet1.gimbal_status_three = packet_in.gimbal_status_three;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_std_feedback_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_gimbal_std_feedback_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_std_feedback_information_pack(system_id, component_id, &msg , packet1.gimbal_status_one , packet1.gimbal_status_two , packet1.gimbal_status_three , packet1.pitch_frame_angle , packet1.yaw_frame_angle , packet1.roll_frame_angle , packet1.x_deviation_quantity , packet1.y_deviation_quantity , packet1.laser_distance , packet1.zoom , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance , packet1.longitude , packet1.latitude , packet1.altitude , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg , packet1.x_Motor_angular_velocity , packet1.y_Motor_angular_velocity , packet1.z_Motor_angular_velocity );
    mavlink_msg_tk_gimbal_std_feedback_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_std_feedback_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gimbal_status_one , packet1.gimbal_status_two , packet1.gimbal_status_three , packet1.pitch_frame_angle , packet1.yaw_frame_angle , packet1.roll_frame_angle , packet1.x_deviation_quantity , packet1.y_deviation_quantity , packet1.laser_distance , packet1.zoom , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance , packet1.longitude , packet1.latitude , packet1.altitude , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg , packet1.x_Motor_angular_velocity , packet1.y_Motor_angular_velocity , packet1.z_Motor_angular_velocity );
    mavlink_msg_tk_gimbal_std_feedback_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_gimbal_std_feedback_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_gimbal_std_feedback_information_send(MAVLINK_COMM_1 , packet1.gimbal_status_one , packet1.gimbal_status_two , packet1.gimbal_status_three , packet1.pitch_frame_angle , packet1.yaw_frame_angle , packet1.roll_frame_angle , packet1.x_deviation_quantity , packet1.y_deviation_quantity , packet1.laser_distance , packet1.zoom , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance , packet1.longitude , packet1.latitude , packet1.altitude , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg , packet1.x_Motor_angular_velocity , packet1.y_Motor_angular_velocity , packet1.z_Motor_angular_velocity );
    mavlink_msg_tk_gimbal_std_feedback_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_GIMBAL_STD_FEEDBACK_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_GIMBAL_STD_FEEDBACK_INFORMATION) != NULL);
#endif
}

static void mavlink_test_tk_imu_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_IMU_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_imu_info_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_tk_imu_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.acceleration_x = packet_in.acceleration_x;
        packet1.acceleration_y = packet_in.acceleration_y;
        packet1.acceleration_z = packet_in.acceleration_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_IMU_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_imu_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_imu_info_pack(system_id, component_id, &msg , packet1.temperature , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_imu_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_imu_info_send(MAVLINK_COMM_1 , packet1.temperature , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_IMU_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_IMU_INFO) != NULL);
#endif
}

static void mavlink_test_tk_global_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_GLOBAL_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_global_position_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,18691,18795
    };
    mavlink_tk_global_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.longitude = packet_in.longitude;
        packet1.latitude = packet_in.latitude;
        packet1.altitude = packet_in.altitude;
        packet1.relative_altitude = packet_in.relative_altitude;
        packet1.target_longitude = packet_in.target_longitude;
        packet1.target_latitude = packet_in.target_latitude;
        packet1.target_altitude = packet_in.target_altitude;
        packet1.heading = packet_in.heading;
        packet1.target_distance = packet_in.target_distance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_GLOBAL_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_global_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_global_position_pack(system_id, component_id, &msg , packet1.longitude , packet1.latitude , packet1.altitude , packet1.relative_altitude , packet1.heading , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance );
    mavlink_msg_tk_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_global_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.longitude , packet1.latitude , packet1.altitude , packet1.relative_altitude , packet1.heading , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance );
    mavlink_msg_tk_global_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_global_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_global_position_send(MAVLINK_COMM_1 , packet1.longitude , packet1.latitude , packet1.altitude , packet1.relative_altitude , packet1.heading , packet1.target_longitude , packet1.target_latitude , packet1.target_altitude , packet1.target_distance );
    mavlink_msg_tk_global_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_GLOBAL_POSITION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_GLOBAL_POSITION) != NULL);
#endif
}

static void mavlink_test_tk_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_ATTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_attitude_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113
    };
    mavlink_tk_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pitchAngle = packet_in.pitchAngle;
        packet1.rollAngle = packet_in.rollAngle;
        packet1.yawAngle = packet_in.yawAngle;
        packet1.angular_velocity_x = packet_in.angular_velocity_x;
        packet1.angular_velocity_y = packet_in.angular_velocity_y;
        packet1.angular_velocity_z = packet_in.angular_velocity_z;
        packet1.acceleration_x = packet_in.acceleration_x;
        packet1.acceleration_y = packet_in.acceleration_y;
        packet1.acceleration_z = packet_in.acceleration_z;
        packet1.mode = packet_in.mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_attitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_attitude_pack(system_id, component_id, &msg , packet1.mode , packet1.pitchAngle , packet1.rollAngle , packet1.yawAngle , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.pitchAngle , packet1.rollAngle , packet1.yawAngle , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_attitude_send(MAVLINK_COMM_1 , packet1.mode , packet1.pitchAngle , packet1.rollAngle , packet1.yawAngle , packet1.angular_velocity_x , packet1.angular_velocity_y , packet1.angular_velocity_z , packet1.acceleration_x , packet1.acceleration_y , packet1.acceleration_z );
    mavlink_msg_tk_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_ATTITUDE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_ATTITUDE) != NULL);
#endif
}

static void mavlink_test_tk_tracking_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_TRACKING_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_tracking_status_t packet_in = {
        17235,17339,17
    };
    mavlink_tk_tracking_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x_deviation_quantity = packet_in.x_deviation_quantity;
        packet1.y_deviation_quantity = packet_in.y_deviation_quantity;
        packet1.tracking_status = packet_in.tracking_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_TRACKING_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_tracking_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_tracking_status_pack(system_id, component_id, &msg , packet1.tracking_status , packet1.x_deviation_quantity , packet1.y_deviation_quantity );
    mavlink_msg_tk_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_tracking_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tracking_status , packet1.x_deviation_quantity , packet1.y_deviation_quantity );
    mavlink_msg_tk_tracking_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_tracking_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_tracking_status_send(MAVLINK_COMM_1 , packet1.tracking_status , packet1.x_deviation_quantity , packet1.y_deviation_quantity );
    mavlink_msg_tk_tracking_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_TRACKING_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_TRACKING_STATUS) != NULL);
#endif
}

static void mavlink_test_tk_infrared_temperature(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_infrared_temperature_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_tk_infrared_temperature_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.infrared_temperature_max = packet_in.infrared_temperature_max;
        packet1.infrared_temperature_min = packet_in.infrared_temperature_min;
        packet1.infrared_temperature_avg = packet_in.infrared_temperature_avg;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_infrared_temperature_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_infrared_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_infrared_temperature_pack(system_id, component_id, &msg , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg );
    mavlink_msg_tk_infrared_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_infrared_temperature_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg );
    mavlink_msg_tk_infrared_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_infrared_temperature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_infrared_temperature_send(MAVLINK_COMM_1 , packet1.infrared_temperature_max , packet1.infrared_temperature_min , packet1.infrared_temperature_avg );
    mavlink_msg_tk_infrared_temperature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_INFRARED_TEMPERATURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_INFRARED_TEMPERATURE) != NULL);
#endif
}

static void mavlink_test_tk_peripheral_device_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_peripheral_device_status_t packet_in = {
        963497464,963497672,17651,17755,17859
    };
    mavlink_tk_peripheral_device_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status_flag = packet_in.status_flag;
        packet1.reserved = packet_in.reserved;
        packet1.camera_zoom = packet_in.camera_zoom;
        packet1.laser_distance = packet_in.laser_distance;
        packet1.photos_cnt = packet_in.photos_cnt;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_peripheral_device_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_peripheral_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_peripheral_device_status_pack(system_id, component_id, &msg , packet1.camera_zoom , packet1.laser_distance , packet1.photos_cnt , packet1.status_flag , packet1.reserved );
    mavlink_msg_tk_peripheral_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_peripheral_device_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.camera_zoom , packet1.laser_distance , packet1.photos_cnt , packet1.status_flag , packet1.reserved );
    mavlink_msg_tk_peripheral_device_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_peripheral_device_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_peripheral_device_status_send(MAVLINK_COMM_1 , packet1.camera_zoom , packet1.laser_distance , packet1.photos_cnt , packet1.status_flag , packet1.reserved );
    mavlink_msg_tk_peripheral_device_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_PERIPHERAL_DEVICE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_PERIPHERAL_DEVICE_STATUS) != NULL);
#endif
}

static void mavlink_test_motor_status_info_1(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_status_info_1_t packet_in = {
        17.0,45.0,73.0,41,108
    };
    mavlink_motor_status_info_1_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.bus_voltage = packet_in.bus_voltage;
        packet1.bus_current = packet_in.bus_current;
        packet1.motor_temperature = packet_in.motor_temperature;
        packet1.motor_status = packet_in.motor_status;
        packet1.error_status = packet_in.error_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_1_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_status_info_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_1_pack(system_id, component_id, &msg , packet1.bus_voltage , packet1.bus_current , packet1.motor_temperature , packet1.motor_status , packet1.error_status );
    mavlink_msg_motor_status_info_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_1_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.bus_voltage , packet1.bus_current , packet1.motor_temperature , packet1.motor_status , packet1.error_status );
    mavlink_msg_motor_status_info_1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_status_info_1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_1_send(MAVLINK_COMM_1 , packet1.bus_voltage , packet1.bus_current , packet1.motor_temperature , packet1.motor_status , packet1.error_status );
    mavlink_msg_motor_status_info_1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_STATUS_INFO_1") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_1) != NULL);
#endif
}

static void mavlink_test_motor_status_info_2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_status_info_2_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,18275
    };
    mavlink_motor_status_info_2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.motor_temperature = packet_in.motor_temperature;
        packet1.torque_current = packet_in.torque_current;
        packet1.output_power = packet_in.output_power;
        packet1.motor_speed = packet_in.motor_speed;
        packet1.motor_angle = packet_in.motor_angle;
        packet1.encoder_position = packet_in.encoder_position;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_status_info_2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_2_pack(system_id, component_id, &msg , packet1.motor_temperature , packet1.torque_current , packet1.output_power , packet1.motor_speed , packet1.motor_angle , packet1.encoder_position );
    mavlink_msg_motor_status_info_2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_temperature , packet1.torque_current , packet1.output_power , packet1.motor_speed , packet1.motor_angle , packet1.encoder_position );
    mavlink_msg_motor_status_info_2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_status_info_2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_2_send(MAVLINK_COMM_1 , packet1.motor_temperature , packet1.torque_current , packet1.output_power , packet1.motor_speed , packet1.motor_angle , packet1.encoder_position );
    mavlink_msg_motor_status_info_2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_STATUS_INFO_2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_2) != NULL);
#endif
}

static void mavlink_test_motor_status_info_3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_status_info_3_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_motor_status_info_3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.motor_temperature = packet_in.motor_temperature;
        packet1.phase_current_a = packet_in.phase_current_a;
        packet1.phase_current_b = packet_in.phase_current_b;
        packet1.phase_current_c = packet_in.phase_current_c;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_status_info_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_3_pack(system_id, component_id, &msg , packet1.motor_temperature , packet1.phase_current_a , packet1.phase_current_b , packet1.phase_current_c );
    mavlink_msg_motor_status_info_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_temperature , packet1.phase_current_a , packet1.phase_current_b , packet1.phase_current_c );
    mavlink_msg_motor_status_info_3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_status_info_3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_status_info_3_send(MAVLINK_COMM_1 , packet1.motor_temperature , packet1.phase_current_a , packet1.phase_current_b , packet1.phase_current_c );
    mavlink_msg_motor_status_info_3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_STATUS_INFO_3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_STATUS_INFO_3) != NULL);
#endif
}

static void mavlink_test_motor_parameter_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_parameter_info_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77
    };
    mavlink_motor_parameter_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.param5 = packet_in.param5;
        packet1.param6 = packet_in.param6;
        packet1.controlParamID = packet_in.controlParamID;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_parameter_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_parameter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_parameter_info_pack(system_id, component_id, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.controlParamID );
    mavlink_msg_motor_parameter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_parameter_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.controlParamID );
    mavlink_msg_motor_parameter_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_parameter_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_parameter_info_send(MAVLINK_COMM_1 , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.controlParamID );
    mavlink_msg_motor_parameter_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_PARAMETER_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_PARAMETER_INFO) != NULL);
#endif
}

static void mavlink_test_motor_encoder_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_ENCODER_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_encoder_info_t packet_in = {
        17235,17339,17443
    };
    mavlink_motor_encoder_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.encoder = packet_in.encoder;
        packet1.encoderRaw = packet_in.encoderRaw;
        packet1.encoderOffset = packet_in.encoderOffset;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_ENCODER_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_encoder_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_info_pack(system_id, component_id, &msg , packet1.encoder , packet1.encoderRaw , packet1.encoderOffset );
    mavlink_msg_motor_encoder_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.encoder , packet1.encoderRaw , packet1.encoderOffset );
    mavlink_msg_motor_encoder_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_encoder_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_info_send(MAVLINK_COMM_1 , packet1.encoder , packet1.encoderRaw , packet1.encoderOffset );
    mavlink_msg_motor_encoder_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_ENCODER_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_ENCODER_INFO) != NULL);
#endif
}

static void mavlink_test_motor_encoder_zero_position_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_ENCODER_ZERO_POSITION_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_encoder_zero_position_info_t packet_in = {
        17235
    };
    mavlink_motor_encoder_zero_position_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.encoderZero = packet_in.encoderZero;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_ENCODER_ZERO_POSITION_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_ENCODER_ZERO_POSITION_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_zero_position_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_encoder_zero_position_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_zero_position_info_pack(system_id, component_id, &msg , packet1.encoderZero );
    mavlink_msg_motor_encoder_zero_position_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_zero_position_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.encoderZero );
    mavlink_msg_motor_encoder_zero_position_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_encoder_zero_position_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_encoder_zero_position_info_send(MAVLINK_COMM_1 , packet1.encoderZero );
    mavlink_msg_motor_encoder_zero_position_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_ENCODER_ZERO_POSITION_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_ENCODER_ZERO_POSITION_INFO) != NULL);
#endif
}

static void mavlink_test_motor_multiturn_angle_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_multiturn_angle_info_t packet_in = {
        17.0
    };
    mavlink_motor_multiturn_angle_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.motorAngle = packet_in.motorAngle;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_multiturn_angle_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_multiturn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_multiturn_angle_info_pack(system_id, component_id, &msg , packet1.motorAngle );
    mavlink_msg_motor_multiturn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_multiturn_angle_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motorAngle );
    mavlink_msg_motor_multiturn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_multiturn_angle_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_multiturn_angle_info_send(MAVLINK_COMM_1 , packet1.motorAngle );
    mavlink_msg_motor_multiturn_angle_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_MULTITURN_ANGLE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_MULTITURN_ANGLE_INFO) != NULL);
#endif
}

static void mavlink_test_motor_single_turn_angle_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_SINGLE_TURN_ANGLE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_single_turn_angle_info_t packet_in = {
        17.0
    };
    mavlink_motor_single_turn_angle_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.circleAngle = packet_in.circleAngle;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_SINGLE_TURN_ANGLE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_SINGLE_TURN_ANGLE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_single_turn_angle_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_single_turn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_single_turn_angle_info_pack(system_id, component_id, &msg , packet1.circleAngle );
    mavlink_msg_motor_single_turn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_single_turn_angle_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.circleAngle );
    mavlink_msg_motor_single_turn_angle_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_single_turn_angle_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_single_turn_angle_info_send(MAVLINK_COMM_1 , packet1.circleAngle );
    mavlink_msg_motor_single_turn_angle_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_SINGLE_TURN_ANGLE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_SINGLE_TURN_ANGLE_INFO) != NULL);
#endif
}

static void mavlink_test_motor_sys_comp_id_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_sys_comp_id_info_t packet_in = {
        5,72
    };
    mavlink_motor_sys_comp_id_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sysId = packet_in.sysId;
        packet1.compId = packet_in.compId;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_sys_comp_id_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_sys_comp_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_sys_comp_id_info_pack(system_id, component_id, &msg , packet1.sysId , packet1.compId );
    mavlink_msg_motor_sys_comp_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_sys_comp_id_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sysId , packet1.compId );
    mavlink_msg_motor_sys_comp_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_sys_comp_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_sys_comp_id_info_send(MAVLINK_COMM_1 , packet1.sysId , packet1.compId );
    mavlink_msg_motor_sys_comp_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_SYS_COMP_ID_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_SYS_COMP_ID_INFO) != NULL);
#endif
}

static void mavlink_test_motor_can_id_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_CAN_ID_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_can_id_info_t packet_in = {
        5
    };
    mavlink_motor_can_id_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.canId = packet_in.canId;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_CAN_ID_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_CAN_ID_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_can_id_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_can_id_info_pack(system_id, component_id, &msg , packet1.canId );
    mavlink_msg_motor_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_can_id_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.canId );
    mavlink_msg_motor_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_can_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_can_id_info_send(MAVLINK_COMM_1 , packet1.canId );
    mavlink_msg_motor_can_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_CAN_ID_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_CAN_ID_INFO) != NULL);
#endif
}

static void mavlink_test_boot_station_information(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BOOT_STATION_INFORMATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_boot_station_information_t packet_in = {
        5
    };
    mavlink_boot_station_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.station_type = packet_in.station_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BOOT_STATION_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_station_information_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_boot_station_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_station_information_pack(system_id, component_id, &msg , packet1.station_type );
    mavlink_msg_boot_station_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_station_information_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.station_type );
    mavlink_msg_boot_station_information_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_boot_station_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_station_information_send(MAVLINK_COMM_1 , packet1.station_type );
    mavlink_msg_boot_station_information_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BOOT_STATION_INFORMATION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BOOT_STATION_INFORMATION) != NULL);
#endif
}

static void mavlink_test_can_id_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAN_ID_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_can_id_info_t packet_in = {
        5
    };
    mavlink_can_id_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.canId = packet_in.canId;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAN_ID_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAN_ID_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_id_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_id_info_pack(system_id, component_id, &msg , packet1.canId );
    mavlink_msg_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_id_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.canId );
    mavlink_msg_can_id_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_can_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_id_info_send(MAVLINK_COMM_1 , packet1.canId );
    mavlink_msg_can_id_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAN_ID_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAN_ID_INFO) != NULL);
#endif
}

static void mavlink_test_imu_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_info_t packet_in = {
        17.0,{ 45.0, 46.0, 47.0 },{ 129.0, 130.0, 131.0 },{ 213.0, 214.0, 215.0, 216.0, 217.0, 218.0, 219.0, 220.0, 221.0 },197
    };
    mavlink_imu_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.flag = packet_in.flag;
        
        mav_array_memcpy(packet1.gyro_offset, packet_in.gyro_offset, sizeof(float)*3);
        mav_array_memcpy(packet1.acc_norm, packet_in.acc_norm, sizeof(float)*3);
        mav_array_memcpy(packet1.acc_matrix, packet_in.acc_matrix, sizeof(float)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_info_pack(system_id, component_id, &msg , packet1.temperature , packet1.gyro_offset , packet1.acc_norm , packet1.acc_matrix , packet1.flag );
    mavlink_msg_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.gyro_offset , packet1.acc_norm , packet1.acc_matrix , packet1.flag );
    mavlink_msg_imu_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_info_send(MAVLINK_COMM_1 , packet1.temperature , packet1.gyro_offset , packet1.acc_norm , packet1.acc_matrix , packet1.flag );
    mavlink_msg_imu_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("IMU_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_IMU_INFO) != NULL);
#endif
}

static void mavlink_test_ip_address_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IP_ADDRESS_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ip_address_info_t packet_in = {
        963497464,17443
    };
    mavlink_ip_address_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ipAddress = packet_in.ipAddress;
        packet1.ipPort = packet_in.ipPort;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IP_ADDRESS_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ip_address_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ip_address_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ip_address_info_pack(system_id, component_id, &msg , packet1.ipAddress , packet1.ipPort );
    mavlink_msg_ip_address_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ip_address_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ipAddress , packet1.ipPort );
    mavlink_msg_ip_address_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ip_address_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ip_address_info_send(MAVLINK_COMM_1 , packet1.ipAddress , packet1.ipPort );
    mavlink_msg_ip_address_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("IP_ADDRESS_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_IP_ADDRESS_INFO) != NULL);
#endif
}

static void mavlink_test_tk_log_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_LOG_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_log_info_t packet_in = {
        5,72,"CDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXY"
    };
    mavlink_tk_log_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.level = packet_in.level;
        
        mav_array_memcpy(packet1.message, packet_in.message, sizeof(char)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_LOG_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_log_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_log_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_log_info_pack(system_id, component_id, &msg , packet1.type , packet1.level , packet1.message );
    mavlink_msg_tk_log_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_log_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.level , packet1.message );
    mavlink_msg_tk_log_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_log_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_log_info_send(MAVLINK_COMM_1 , packet1.type , packet1.level , packet1.message );
    mavlink_msg_tk_log_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_LOG_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_LOG_INFO) != NULL);
#endif
}

static void mavlink_test_tk_strain_gage_data_collection(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_tk_strain_gage_data_collection_t packet_in = {
        { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234 }
    };
    mavlink_tk_strain_gage_data_collection_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*230);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_strain_gage_data_collection_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_tk_strain_gage_data_collection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_strain_gage_data_collection_pack(system_id, component_id, &msg , packet1.data );
    mavlink_msg_tk_strain_gage_data_collection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_strain_gage_data_collection_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.data );
    mavlink_msg_tk_strain_gage_data_collection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_tk_strain_gage_data_collection_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_tk_strain_gage_data_collection_send(MAVLINK_COMM_1 , packet1.data );
    mavlink_msg_tk_strain_gage_data_collection_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TK_STRAIN_GAGE_DATA_COLLECTION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TK_STRAIN_GAGE_DATA_COLLECTION) != NULL);
#endif
}

static void mavlink_test_unitree_dog_joystick(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_unitree_dog_joystick_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_unitree_dog_joystick_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Lx = packet_in.Lx;
        packet1.Ly = packet_in.Ly;
        packet1.Rx = packet_in.Rx;
        packet1.Ry = packet_in.Ry;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unitree_dog_joystick_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_unitree_dog_joystick_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unitree_dog_joystick_pack(system_id, component_id, &msg , packet1.Lx , packet1.Ly , packet1.Rx , packet1.Ry );
    mavlink_msg_unitree_dog_joystick_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unitree_dog_joystick_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Lx , packet1.Ly , packet1.Rx , packet1.Ry );
    mavlink_msg_unitree_dog_joystick_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_unitree_dog_joystick_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_unitree_dog_joystick_send(MAVLINK_COMM_1 , packet1.Lx , packet1.Ly , packet1.Rx , packet1.Ry );
    mavlink_msg_unitree_dog_joystick_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("UNITREE_DOG_JOYSTICK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_UNITREE_DOG_JOYSTICK) != NULL);
#endif
}

static void mavlink_test_custom(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_file_information_request(system_id, component_id, last_msg);
    mavlink_test_log_information(system_id, component_id, last_msg);
    mavlink_test_speaker_information(system_id, component_id, last_msg);
    mavlink_test_speaker_status(system_id, component_id, last_msg);
    mavlink_test_speaker_audio_info(system_id, component_id, last_msg);
    mavlink_test_speaker_connection_status(system_id, component_id, last_msg);
    mavlink_test_speaker_camera_network_params(system_id, component_id, last_msg);
    mavlink_test_skylink_information(system_id, component_id, last_msg);
    mavlink_test_skylink_file_information(system_id, component_id, last_msg);
    mavlink_test_light_information(system_id, component_id, last_msg);
    mavlink_test_light_file_information(system_id, component_id, last_msg);
    mavlink_test_light_status(system_id, component_id, last_msg);
    mavlink_test_tk_custom_data(system_id, component_id, last_msg);
    mavlink_test_tk_gimbal_information(system_id, component_id, last_msg);
    mavlink_test_tk_rc_channels(system_id, component_id, last_msg);
    mavlink_test_tk_gimbal_std_feedback_information(system_id, component_id, last_msg);
    mavlink_test_tk_imu_info(system_id, component_id, last_msg);
    mavlink_test_tk_global_position(system_id, component_id, last_msg);
    mavlink_test_tk_attitude(system_id, component_id, last_msg);
    mavlink_test_tk_tracking_status(system_id, component_id, last_msg);
    mavlink_test_tk_infrared_temperature(system_id, component_id, last_msg);
    mavlink_test_tk_peripheral_device_status(system_id, component_id, last_msg);
    mavlink_test_motor_status_info_1(system_id, component_id, last_msg);
    mavlink_test_motor_status_info_2(system_id, component_id, last_msg);
    mavlink_test_motor_status_info_3(system_id, component_id, last_msg);
    mavlink_test_motor_parameter_info(system_id, component_id, last_msg);
    mavlink_test_motor_encoder_info(system_id, component_id, last_msg);
    mavlink_test_motor_encoder_zero_position_info(system_id, component_id, last_msg);
    mavlink_test_motor_multiturn_angle_info(system_id, component_id, last_msg);
    mavlink_test_motor_single_turn_angle_info(system_id, component_id, last_msg);
    mavlink_test_motor_sys_comp_id_info(system_id, component_id, last_msg);
    mavlink_test_motor_can_id_info(system_id, component_id, last_msg);
    mavlink_test_boot_station_information(system_id, component_id, last_msg);
    mavlink_test_can_id_info(system_id, component_id, last_msg);
    mavlink_test_imu_info(system_id, component_id, last_msg);
    mavlink_test_ip_address_info(system_id, component_id, last_msg);
    mavlink_test_tk_log_info(system_id, component_id, last_msg);
    mavlink_test_tk_strain_gage_data_collection(system_id, component_id, last_msg);
    mavlink_test_unitree_dog_joystick(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CUSTOM_TESTSUITE_H
