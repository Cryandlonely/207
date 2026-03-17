#include "std_protocol_sdk.h"
#include <stdint.h>
#include <stdio.h>
#include "stdlib.h"
#include "string.h"


//指向当前正在使用的 SDK 实例
gimbal_sdk_t* g_active_sdk = NULL;

// 客户调用此函数，告诉 SDK：接下来的控制指令都通过这个 sdk 实例发出
void std_sdk_set_active_instance(gimbal_sdk_t* sdk) { g_active_sdk = sdk; }

// 设置特定实例的发送回调
void gimbal_sdk_set_write_func(gimbal_sdk_t* sdk, sdk_write_func_t func, void* user_ptr) {
  if (sdk) {
    sdk->write_func = func;
    sdk->write_user_ptr = user_ptr;
  }
}

// 内部私有发送辅助函数
void internal_auto_send(std_packet_t* pkt) {
  // 使用局部指针拷贝，防止在执行过程中全局变量被其他线程/中断修改
  gimbal_sdk_t* sdk = g_active_sdk;

  if (sdk && sdk->write_func && pkt->length > 0) {
    sdk->write_func(pkt->data, pkt->length, sdk->write_user_ptr);
  }
}

/**
 * @brief 计算截断后的长度（去除末尾的连续0x00）
 * @param data 数据指针
 * @param max_len 最大长度
 * @param min_len 最小保留长度（通常至少保留 Command ID 字段）
 */
static uint8_t std_calc_truncated_len(const uint8_t* data, uint8_t max_len, uint8_t min_len) {
  uint8_t len = max_len;
  while (len > min_len && data[len - 1] == 0) {
    len--;
  }
  return len;
}

/**
 * @brief 内部私有函数：根据流 ID 清理本地回调指针
 */
static void internal_clear_local_callback(gimbal_sdk_t* sdk, std_msg_stream_t stream_id) {
  switch (stream_id) {
    case STD_MSG_STREAM_ATTITUDE:
      // 如果你有专门的姿态请求结构体
      break;
    case STD_MSG_STREAM_IR_TEMPERATURE:
      sdk->ir_temperature_req.cb = NULL;
      sdk->ir_temperature_req.user_data = NULL;
      break;
    case STD_MSG_STREAM_GPS:
      // sdk->gps_req.cb = NULL;
      break;
    // robot dog clear callback
    case STD_MSG_STREAM_ROBOT_DOG_CONTROL:
      break;
    // 在此处添加其他流 ID 的处理...
    default:
      break;
  }
}

/**
 * @brief 内部辅助函数：封装同步命令的创建与发送
 */
static std_packet_t _create_and_send_sync_cmd(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));
  if (!g_active_sdk) return packet;

  packet.length = std_create_command_long_packet(command, p1, p2, p3, p4, p5, p6, p7, packet.data, sizeof(packet.data));

  internal_auto_send(&packet);
  return packet;
}

/**
 * @brief 内部辅助函数：封装异步命令的创建与发送
 */
static std_packet_t _create_and_send_async_cmd(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7, ResultCallback cb,
                                               void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));
  if (!g_active_sdk || !cb) return packet;

  std_cmd_long_t cmd_payload;
  memset(&cmd_payload, 0, sizeof(std_cmd_long_t));
  cmd_payload.command = command;
  cmd_payload.param1 = p1;
  cmd_payload.param2 = p2;
  cmd_payload.param3 = p3;
  cmd_payload.param4 = p4;
  cmd_payload.param5 = p5;
  cmd_payload.param6 = p6;
  cmd_payload.param7 = p7;

  // 计算截断长度 (保留 command 字段的至少2个字节)
  uint8_t truncated_payload_len = std_calc_truncated_len((uint8_t*)&cmd_payload, sizeof(cmd_payload), 2);

  uint16_t pkt_len = std_cmd_async_create(g_active_sdk, MSG_COMMAND_LONG, (uint8_t*)&cmd_payload, truncated_payload_len, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;
  internal_auto_send(&packet);
  return packet;
}

/**
 * @brief 抽象的 Command Long 创建函数（带截断功能）
 * @param command : 命令ID;
 */
size_t std_create_command_long_packet(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7, uint8_t* out_buf, size_t buf_size) {
  std_cmd_long_t payload;
  memset(&payload, 0, sizeof(payload));

  payload.command = command;
  payload.param1 = p1;
  payload.param2 = p2;
  payload.param3 = p3;
  payload.param4 = p4;
  payload.param5 = p5;
  payload.param6 = p6;
  payload.param7 = p7;

  // 1. 计算截断长度
  // 假设 std_cmd_long_t 结构体中 command 在末尾，那么 min_len 应该是 sizeof(std_cmd_long_t)
  // 如果效仿 MAVLink 将 command 放在前面，截断效果更好。
  // 这里建议为了截断 float，min_len 设为 2 (即只保留 command 字段)
  uint8_t truncated_len = std_calc_truncated_len((uint8_t*)&payload, sizeof(payload), 2);

  // 2. 调用通用组包函数
  return std_create_packet(MSG_COMMAND_LONG, (uint8_t*)&payload, truncated_len, out_buf, buf_size);
}

/**
 * @brief 通用的流取消订阅函数 (底层)
 * @return 返回生成的协议包，以便用户进行日志记录或手动重发
 */
std_packet_t std_control_unsubscribe_stream(std_msg_stream_t stream_id) {
  if (!g_active_sdk) {
    std_packet_t packet;
    memset(&packet, 0, sizeof(std_packet_t));
    return packet;
  }

  // 1. 本地逻辑清理 (回调置空)
  internal_clear_local_callback(g_active_sdk, stream_id);

  // 2. 构造通用取消指令包并发送
  return _create_and_send_sync_cmd(STD_CMD_ID_SET_MESSAGE_STREAM_INTERVAL, (float)stream_id, -1.0f, 0, 0, 0, 0, 0);
}

/**
 * @brief 通用的流订阅函数 (底层包装)
 */
std_packet_t std_control_subscribe_stream(std_msg_stream_t stream_id, float hz) {
  return _create_and_send_sync_cmd(STD_CMD_ID_SET_MESSAGE_STREAM_INTERVAL, (float)stream_id, hz, 0, 0, 0, 0, 0);
}

/**
 * @brief 一次性请求特定消息 (单次获取)
 * @param msg_id 想要获取的消息ID
 */
std_packet_t std_control_request_message_once(std_msg_stream_t msg_id) { return _create_and_send_sync_cmd(STD_CMD_REQUEST_MESSAGE, (float)msg_id, 0, 0, 0, 0, 0, 0); }

/*
 *@brief 将uint16_t转换为字节数组 (小端序)
 *@param val : 输入的uint16_t值;
 *data : 输出的字节数组, 长度至少为2;
 *@return 返回写入的字节数, 总是2
 */
int std_uint16_to_data(uint16_t val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  return 2;
}

/*
 *@brief 将int16_t转换为字节数组 (小端序)
 *@param val : 输入的int16_t值;
 *data : 输出的字节数组, 长度至少为2;
 *@return 返回写入的字节数, 总是2
 */
int std_int16_to_data(int16_t val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  return 2;
}

/*
 *@brief 将int32_t转换为字节数组 (小端序)
 *@param val : 输入的int32_t值;
 *data : 输出的字节数组, 长度至少为4;
 *@return 返回写入的字节数, 总是4
 */
int std_int32_to_data(int32_t val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  data[2] = *(((uint8_t*)(&val)) + 2);
  data[3] = *(((uint8_t*)(&val)) + 3);
  return 4;
}

/*
 *@brief 将float转换为字节数组 (小端序)
 *@param val : 输入的float值;
 *data : 输出的字节数组, 长度至少为4;
 *@return 返回写入的字节数, 总是4
 */
int std_float_to_data(float val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  data[2] = *(((uint8_t*)(&val)) + 2);
  data[3] = *(((uint8_t*)(&val)) + 3);
  return 4;
}

/*
 *@brief 将uint32_t转换为字节数组 (小端序)
 *@param val : 输入的uint32_t值;
 *@param data : 输出的字节数组, 长度至少为4;
 *@return 返回写入的字节数, 总是4
 */
int std_uint32_to_data(uint32_t val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  data[2] = *(((uint8_t*)(&val)) + 2);
  data[3] = *(((uint8_t*)(&val)) + 3);
  return 4;
}

/*
 * @brief 将uint64_t转换为字节数组 (小端序)
 *@param val : 输入的uint64_t值;
 *@param data : 输出的字节数组, 长度至少为8;
 *@return 返回写入的字节数, 总是8
 */
int std_uint64_to_data(uint64_t val, uint8_t* data) {
  data[0] = *(((uint8_t*)(&val)) + 0);
  data[1] = *(((uint8_t*)(&val)) + 1);
  data[2] = *(((uint8_t*)(&val)) + 2);
  data[3] = *(((uint8_t*)(&val)) + 3);
  data[4] = *(((uint8_t*)(&val)) + 4);
  data[5] = *(((uint8_t*)(&val)) + 5);
  data[6] = *(((uint8_t*)(&val)) + 6);
  data[7] = *(((uint8_t*)(&val)) + 7);
  return 8;
}

/*
 * @brief 将字节数组转换为uint16_t (小端序)
 * @param data : 输入的字节数组, 长度至少为2;
 */
uint16_t std_data_to_uint16(const uint8_t* data) {
  uint16_t val;
  *(((uint8_t*)(&val)) + 0) = data[0];
  *(((uint8_t*)(&val)) + 1) = data[1];
  return val;
}

/*
 * @brief 在命令事务队列中分配一个空闲的cmd_work_t
 * @param sdk : 指向 gimbal_sdk_t 实例的指针
 */
static cmd_work_t* cmd_work_alloc(gimbal_sdk_t* sdk) {
  if (!sdk) {
    return NULL;
  }

  for (int i = 0; i < CMD_WORK_QUEUE_SIZE; i++) {
    cmd_work_t* work = &sdk->cmd_queue.works[i];
    if (!work->used) {
      memset(work, 0, sizeof(*work));  // 确保干净
      work->used = 1;
      return work;
    }
  }

  return NULL;  // 队列满
}

/*
 * @brief 释放一个cmd_work_t，标记为未使用
 * @param work : 指向要释放的 cmd_work_t 实例的指针
 */
static void cmd_work_free(cmd_work_t* work) {
  if (!work) {
    return;
  }
  memset(work, 0, sizeof(*work));
}

/*
 *@brief 计算校验和 与 附加校验函数
 *@param  data : 发送的字节数组;
 *sumcheck :校验和;
 *addsum :附加校验;
 *len: 长度等于 帧头1 ~ data[length - 1]
 */
static void calculate_checksums(const uint8_t* data, size_t len, uint8_t* sumcheck, uint8_t* addsum) {
  *sumcheck = 0;
  *addsum = 0;

  for (size_t i = 0; i < len; i++) {
    *sumcheck += data[i];
    *addsum += *sumcheck;
  }
}

/*
 *@brief 创建协议包
 *@param msg_id : 消息ID;
 *payload : 数据载荷;
 *payload_len : 数据载荷长度;
 *output_buffer : 输出缓冲区;
 *buffer_size : 输出缓冲区大小;
 *@return 返回创建的包长度, 0表示失败
 */
size_t std_create_packet(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, uint8_t* output_buffer, size_t buffer_size) {
  if (payload_len > 255 || (uint16_t)buffer_size < payload_len + 8) {
    return 0;
  }

  size_t index = 0;

  // 帧头
  output_buffer[index++] = STD_HEADER_1;
  output_buffer[index++] = STD_HEADER_2;
  output_buffer[index++] = STD_SENDER_ID;
  output_buffer[index++] = STD_RECEIVER_ID;
  output_buffer[index++] = msg_id;
  output_buffer[index++] = payload_len;

  // 数据载荷
  if (payload_len > 0 && payload != NULL) {
    memcpy(&output_buffer[index], payload, payload_len);
    index += payload_len;
  }

  // 计算校验和
  uint8_t sumcheck, addsum;
  calculate_checksums(output_buffer, index, &sumcheck, &addsum);

  output_buffer[index++] = sumcheck;
  output_buffer[index++] = addsum;

  return index;
}

/*
 *@brief 创建TGA协议包
 *@param msg_id : 消息ID;
 *payload : 数据载荷;
 *payload_len : 数据载荷长度;
 *output_buffer : 输出缓冲区;
 *buffer_size : 输出缓冲区大小;
 *@return 返回创建的包长度, 0表示失败
 */
size_t tga_create_packet(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, uint8_t* output_buffer, size_t buffer_size) {
  if (payload_len > 255 || (uint16_t)buffer_size < payload_len + 8) {
    return 0;
  }

  size_t index = 0;

  // 帧头
  output_buffer[index++] = STD_HEADER_1;
  output_buffer[index++] = STD_HEADER_2;
  output_buffer[index++] = TGA_SENDER_ID;
  output_buffer[index++] = STD_RECEIVER_ID;
  output_buffer[index++] = msg_id;
  output_buffer[index++] = payload_len;

  // 数据载荷
  if (payload_len > 0 && payload != NULL) {
    memcpy(&output_buffer[index], payload, payload_len);
    index += payload_len;
  }

  // 计算校验和
  uint8_t sumcheck, addsum;
  calculate_checksums(output_buffer, index, &sumcheck, &addsum);

  output_buffer[index++] = sumcheck;
  output_buffer[index++] = addsum;

  return index;
}

/*
 *@brief 创建速度控制协议包
 *@param pitch_speed : 俯仰速度;
    *yaw_speed : 偏航速度;
 *@note:
    数据范围:[-30000,30000],上正下负,左正右负,1倍下: 60°/s
    *@return 返回创建的协议包
*/
std_packet_t std_control_velocity(int16_t pitch_speed, int16_t yaw_speed) {
  std_packet_t packet;
  uint8_t payload[4];

  std_int16_to_data(pitch_speed, payload);
  std_int16_to_data(yaw_speed, payload + 2);

  packet.length = std_create_packet(MSG_ID_VEL_CTRL, payload, 4, packet.data, sizeof(packet.data));

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建停止运动协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_stop_movement(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_STOP_MOVE, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
* @brief 创建云台方向控制协议包
* @param direction : 方向控制命令;
    STD_DIRECTION_UP = 0x00,      // 上
    STD_DIRECTION_DOWN = 0x01,    // 下
    STD_DIRECTION_LEFT = 0x02,    // 左
    STD_DIRECTION_RIGHT = 0x03,   // 右
    STD_DIRECTION_STOP = 0x04,    // 停止
*/
std_packet_t std_control_direction_control(std_direction_control_t direction, float step) {
  if (direction < STD_DIRECTION_STOP || direction >= STD_DIRECTION_END) {
    // 无效的方向控制命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));

    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[3] = {0};
  payload[0] = (uint8_t)direction;
  uint16_t step_value = (uint16_t)(step * 100);  // 转换为整数表示，保留两位小数
  std_uint16_to_data(step_value, &payload[1]);   // 步进值转换为字节数组

  packet.length = std_create_packet(MSG_ID_DIRECTION_CONTROL, payload, 3, packet.data, sizeof(packet.data));

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

std_packet_t std_control_direction_control_async(std_direction_control_t direction, float step, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (direction < STD_DIRECTION_STOP || direction >= STD_DIRECTION_END) {
    return packet;
  }

  uint8_t payload[3] = {0};
  payload[0] = (uint8_t)direction;
  uint16_t step_value = (uint16_t)(step * 100);  // 转换为整数表示，保留两位小数
  std_uint16_to_data(step_value, &payload[1]);   // 步进值转换为字节数组

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_DIRECTION_CONTROL, payload, 3, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建模式切换协议包
*@param mode : 目标模式;
    MODE_LOCK = 0x00,    // 锁定模式
    MODE_FOLLOW = 0x01   // 跟随模式
    *@return 返回创建的协议包
*/
std_packet_t std_control_mode_switch(std_mode_t mode) {
  if (mode != MODE_LOCK && mode != MODE_FOLLOW) {
    // 无效模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {mode};
  packet.length = std_create_packet(MSG_ID_MODE_SWITCH, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
*@brief 创建异步模式切换协议包
*@param sdk : 指向 gimbal_sdk_t 实例的指针
*@param mode : 目标模式;
    MODE_LOCK = 0x00,    // 锁定模式
    MODE_FOLLOW = 0x01   // 跟随模式
    *@return 返回创建的协议包
*/
std_packet_t std_control_mode_switch_async(const std_mode_t mode, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (mode != MODE_LOCK && mode != MODE_FOLLOW) {
    return packet;
  }

  uint8_t payload[1] = {mode};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_MODE_SWITCH, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建指点跟踪协议包
 *@param enable : 是否启用指点跟踪;
 *@note:   画面左上角为原点使用归一化处理,X,Y类型float,范围均为[0,1]
 *@return 返回创建的协议包
 */
std_packet_t std_control_point_tracking(float x, float y) {
  std_packet_t packet;
  uint8_t payload[8];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x, payload);
  std_float_to_data(y, payload + 4);
  packet.length = std_create_packet(MSG_ID_POINT_TRACKING, payload, 8, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 *@brief 创建异步指点跟踪协议包
 *@param sdk : 指向 gimbal_sdk_t 实例的指针
 *@param x : 目标点X坐标;
 *@param y : 目标点Y坐标;
 *@note:   画面左上角为原点使用归一化处理,X,Y类型float,范围均为[0,1]
 *@return 返回创建的协议包
 */
std_packet_t std_control_point_tracking_async(float x, float y, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[8];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x, payload);
  std_float_to_data(y, payload + 4);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_POINT_TRACKING, payload, 8, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建取消跟踪协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_tracking_off(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_TRACKING_OFF, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建异步取消跟踪协议包
 *@param sdk : 指向 gimbal_sdk_t 实例的指针
 *@return 返回创建的协议包
 */
std_packet_t std_control_tracking_off_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TRACKING_OFF, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建一键向下协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_onekey_down(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_ONEKEY_DOWN, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 *@brief 创建异步一键向下协议包
 *@param sdk : 指向 gimbal_sdk_t 实例的指针
 *@return 返回创建的协议包
 */
std_packet_t std_control_onekey_down_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_ONEKEY_DOWN, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建一键回中协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_onekey_center(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_ONEKEY_CENTER, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/**
 * @brief 创建异步一键回中协议包
 * @param sdk : 指向 gimbal_sdk_t 实例的指针
 * @return 返回创建的协议包
 */
std_packet_t std_control_onekey_center_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_ONEKEY_CENTER, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建指定航向角度协议包
 *@param yaw_angle : 目标航向角度;
 *@note: 范围[-180,180],正顺时针为正,受特定云台限位影响,实际可达范围可能更小
 *@return 返回创建的协议包
 */
std_packet_t std_control_specify_yaw(float yaw_angle) {
  std_packet_t packet;
  int16_t yaw_angle_int = (int16_t)(yaw_angle * 10);  // 转换为整数表示
  uint8_t payload[2];
  std_int16_to_data(yaw_angle_int, payload);
  packet.length = std_create_packet(MSG_ID_SPECIFY_YAW, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 *@brief 创建异步指定航向角度协议包
 *@param sdk : 指向 gimbal_sdk_t 实例的指针
 *@param yaw_angle : 目标航向角度;
 *@note: 范围[-180,180],正顺时针为正,受特定云台限位影响,实际可达范围可能更小
 *@return 返回创建的协议包
 */
std_packet_t std_control_specify_yaw_async(float yaw_angle, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  int16_t yaw_angle_int = (int16_t)(yaw_angle * 10);  // 转换为整数表示
  uint8_t payload[2];
  std_int16_to_data(yaw_angle_int, payload);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_SPECIFY_YAW, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建指定俯仰角度协议包
 *@param pitch_angle : 目标俯仰角度;
 *@note: 范围[-90,90],正上仰为正,受特定云台限位影响,实际可达范围可能更小
 *@return 返回创建的协议包
 */
std_packet_t std_control_specify_pitch(float pitch_angle) {
  std_packet_t packet;
  int16_t pitch_angle_int = (int16_t)(pitch_angle * 10);  // 转换为整数表示
  uint8_t payload[2];
  std_int16_to_data(pitch_angle_int, payload);
  packet.length = std_create_packet(MSG_ID_SPECIFY_PITCH, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 *@brief 创建异步指定俯仰角度协议包
 *@param sdk : 指向 gimbal_sdk_t 实例的指针
 *@param pitch_angle : 目标俯仰角度;
 *@note: 范围[-90,90],正上仰为正,受特定云台限位影响,实际可达范围可能更小
 *@return 返回创建的协议包
 */
std_packet_t std_control_specify_pitch_async(float pitch_angle, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  int16_t pitch_angle_int = (int16_t)(pitch_angle * 10);  // 转换为整数表示
  uint8_t payload[2];
  std_int16_to_data(pitch_angle_int, payload);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_SPECIFY_PITCH, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建指定俯仰角度和航向角度协议包
 *@param pitch_angle : 目标俯仰角度;
 *@param yaw_angle : 目标航向角度;
 *@note: 俯仰范围[-90,90],正上仰为正,受特定云台限位影响,实际可达范围可能更小
        航向范围[-180,180],正顺时针为正,受特定云台限位影响,实际可达范围可能更小
 *@return 返回创建的协议包
*/
std_packet_t std_control_specify_angle(float pitch_angle, float yaw_angle) {
  std_packet_t packet;
  int16_t pitch_angle_int = (int16_t)(pitch_angle * 10);  // 转换为整数表示
  int16_t yaw_angle_int = (int16_t)(yaw_angle * 10);      // 转换为整数表示
  uint8_t payload[4];
  std_int16_to_data(pitch_angle_int, payload);
  std_int16_to_data(yaw_angle_int, payload + 2);
  packet.length = std_create_packet(MSG_ID_SPECIFY_ANGLE, payload, 4, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_specify_angle_async(float pitch_angle, float yaw_angle, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  int16_t pitch_angle_int = (int16_t)(pitch_angle * 10);  // 转换为整数表示
  int16_t yaw_angle_int = (int16_t)(yaw_angle * 10);      // 转换为整数表示
  uint8_t payload[4];
  std_int16_to_data(pitch_angle_int, payload);
  std_int16_to_data(yaw_angle_int, payload + 2);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_SPECIFY_ANGLE, payload, 4, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建云台增稳协议包
 *@param enable : 是否启用增稳;
        STABILIZATION_ON = 0x01,  // 增稳开
        STABILIZATION_OFF = 0x00  // 增稳关
 *@return 返回创建的协议包
*/
std_packet_t std_control_stabilization(std_stabilization_t enable) {
  std_packet_t packet;
  uint8_t payload[1] = {enable};
  packet.length = std_create_packet(MSG_ID_STABILIZATION, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_stabilization_async(std_stabilization_t enable, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {enable};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_STABILIZATION, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建变倍+协议包
 *@param speed : 变倍速度,范围[1,7],数值越大变倍速度越快;
 *@return 返回创建的协议包
 */
std_packet_t std_control_zoom_in(uint8_t speed) {
  std_packet_t packet;
  uint8_t payload[1] = {speed};
  packet.length = std_create_packet(MSG_ID_ZOOM_IN, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_zoom_in_async(uint8_t speed, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {speed};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_ZOOM_IN, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建变倍-协议包
 *@param speed : 变倍速度,范围[1,7],数值越大变倍速度越快;
 *@return 返回创建的协议包
 */
std_packet_t std_control_zoom_out(uint8_t speed) {
  std_packet_t packet;
  uint8_t payload[1] = {speed};
  packet.length = std_create_packet(MSG_ID_ZOOM_OUT, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_zoom_out_async(uint8_t speed, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {speed};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_ZOOM_OUT, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建停止变倍协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_zoom_stop(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_ZOOM_STOP, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_zoom_stop_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_ZOOM_STOP, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建指定倍率协议包
 *@param zoom_level : 目标倍率;
 *@note: 根据吊舱实际支持的倍率范围设置
 *@return 返回创建的协议包
 */
std_packet_t std_control_specify_zoom(float zoom_level) {
  std_packet_t packet;
  uint8_t payload[2];
  uint16_t zoom_level_int = (uint16_t)(zoom_level * 10);  // 转换为整数表示
  std_uint16_to_data(zoom_level_int, payload);
  packet.length = std_create_packet(MSG_ID_SPECIFY_ZOOM, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_specify_zoom_async(float zoom_level, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[2];
  uint16_t zoom_level_int = (uint16_t)(zoom_level * 10);  // 转换为整数表示
  std_uint16_to_data(zoom_level_int, payload);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_SPECIFY_ZOOM, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建对焦+协议包
 *@param speed : 对焦速度,范围[1,7],数值越大对焦速度越快;
 *@return 返回创建的协议包
 */
std_packet_t std_control_focus_in(uint8_t speed) {
  std_packet_t packet;
  uint8_t payload[1] = {speed};
  packet.length = std_create_packet(MSG_ID_FOCUS_IN, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_focus_in_async(uint8_t speed, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {speed};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_FOCUS_IN, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建对焦-协议包
 *@param speed : 对焦速度,范围[1,7],数值越大对焦速度越快;
 *@return 返回创建的协议包
 */
std_packet_t std_control_focus_out(uint8_t speed) {
  std_packet_t packet;
  uint8_t payload[1] = {speed};
  packet.length = std_create_packet(MSG_ID_FOCUS_OUT, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_focus_out_async(uint8_t speed, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {speed};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_FOCUS_OUT, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建停止对焦协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_focus_stop(void) {
  std_packet_t packet;
  uint8_t payload[2] = {0, 0};  // 停止对焦时不需要参数
  packet.length = std_create_packet(MSG_ID_FOCUS_STOP, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_focus_stop_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[2] = {0, 0};  // 停止对焦时不需要参数

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_FOCUS_STOP, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建拍照协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_take_photo(void) {
  std_packet_t packet;
  uint8_t payload[1] = {0x01};
  packet.length = std_create_packet(MSG_ID_TAKE_PHOTO, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_take_photo_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[1] = {0x01};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TAKE_PHOTO, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建开始录像协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_start_recording(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_START_RECORDING, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_start_recording_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_START_RECORDING, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建停止录像协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_stop_recording(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_STOP_RECORDING, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_stop_recording_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_STOP_RECORDING, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建画中画切换协议包
*@param mode : 画中画模式;
    PIP_VI_ONLY = 0x00,    // 可见光
    PIP_IR_ONLY = 0x01,    // 热成像
    PIP_VI_IN_IR = 0x02,   // 可见光嵌入热成像
    PIP_IR_IN_VI = 0x03,   // 热成像嵌入可见光
    PIP_WIDE_ONLY = 0x04,  // 广角
    PIP_VI_IN_WIDE = 0x05, // 可见光嵌入广角
    PIP_WIDE_IN_VI = 0x06, // 广角嵌入可见光
*@return 返回创建的协议包
*/
std_packet_t std_control_pip_switch(std_pip_t mode) {
  if (mode < PIP_VI_ONLY || mode >= PIP_END) {
    // 无效的画中画模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {mode};
  packet.length = std_create_packet(MSG_ID_PIP_SWITCH, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_pip_switch_async(std_pip_t mode, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (mode < PIP_VI_ONLY || mode >= PIP_END) {
    return packet;
  }

  uint8_t payload[1] = {mode};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_PIP_SWITCH, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建伪彩切换协议包
*@param mode : 伪彩模式;
    PSEUDO_COLOR_1 = 0x00,
    PSEUDO_COLOR_2 = 0x01,
    PSEUDO_COLOR_3 = 0x02,
    PSEUDO_COLOR_4 = 0x03,
    PSEUDO_COLOR_5 = 0x04,
    PSEUDO_COLOR_6 = 0x05,
    PSEUDO_COLOR_7 = 0x06,
    PSEUDO_COLOR_8 = 0x07,
    PSEUDO_COLOR_9 = 0x08,
    PSEUDO_COLOR_10 = 0x09,
    PSEUDO_COLOR_11 = 0x0A,
    PSEUDO_COLOR_12 = 0x0B,
    PSEUDO_COLOR_13 = 0x0C,
    PSEUDO_COLOR_14 = 0x0D,
    PSEUDO_COLOR_15 = 0x0E,
    PSEUDO_COLOR_16 = 0x0F,
    PSEUDO_COLOR_END = 0x10
*note: 具体支持的伪彩模式数量和编号取决于吊舱型号
*@return 返回创建的协议包
*/
std_packet_t std_control_pseudo_color(std_pseudo_color_t mode) {
  if (mode < PSEUDO_COLOR_1 || mode > PSEUDO_COLOR_END) {
    // 无效的伪彩模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {mode};
  packet.length = std_create_packet(MSG_ID_PSEUDO_COLOR, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}
std_packet_t std_control_pseudo_color_async(std_pseudo_color_t mode, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (mode < PSEUDO_COLOR_1 || mode > PSEUDO_COLOR_END) {
    return packet;
  }

  uint8_t payload[1] = {mode};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_PSEUDO_COLOR, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建OSD开协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_osd_on(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_OSD_ON, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_osd_on_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_OSD_ON, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建OSD关协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_osd_off(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_OSD_OFF, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_osd_off_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_OSD_OFF, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建电子变倍协议包
*@param enable : 电子变倍使能;
    DZOOM_OFF = 0x00,   // 电子变倍关
    DZOOM_ON = 0x01,    // 电子变倍开
*/
std_packet_t std_control_digital_zoom(std_dzoom_t enable) {
  if (enable < DZOOM_OFF || enable >= DZOOM_END) {
    // 无效的电子变倍命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {enable};
  packet.length = std_create_packet(MSG_ID_DZOOM, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_digital_zoom_async(std_dzoom_t enable, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (enable < DZOOM_OFF || enable >= DZOOM_END) {
    return packet;
  }

  uint8_t payload[1] = {enable};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_DZOOM, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 * @brief 创建相机电子增稳开协议包
 * @return 返回创建的协议包
 */
std_packet_t std_control_camera_stabilization(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_CAMERA_STABILIZATION_ON, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_camera_stabilization_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_CAMERA_STABILIZATION_ON, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建相机电子增稳关协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_camera_stabilization_off(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_CAMERA_STABILIZATION_OFF, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_camera_stabilization_off_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_CAMERA_STABILIZATION_OFF, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 * @brief 创建激光控制协议包
 * @param command : 激光控制命令;
    LASER_ON = 0x00,   // 激光开
    LASER_OFF = 0x01,    // 激光关
    * @return 返回创建的协议包
 */
std_packet_t std_control_laser_control(std_laser_control_t command) {
  if (command < LASER_ON || command >= LASER_END) {
    // 无效的激光控制命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {command};
  packet.length = std_create_packet(MSG_ID_LASER, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_laser_control_async(std_laser_control_t command, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (command < LASER_ON || command >= LASER_END) {
    return packet;
  }

  uint8_t payload[1] = {command};

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_LASER, payload, 1, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建激光测距控制协议包
*@param command : 激光测距控制命令;
    LASER_DISTANCE_SINGLE = 0x00,   // 单次测距
    LASER_DISTANCE_CONTINUOUS = 0x01, // 连续测距
    LASER_DISTANCE_STOP = 0x02,      // 停止测距
* @return 返回创建的协议包
*/
std_packet_t std_control_laser_ranging(std_laser_distance_control_t command) {
  if (command < LASER_DISTANCE_SINGLE || command >= LASER_DISTANCE_END) {
    // 无效的激光测距控制命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));  // 初始化空包

  if (command == LASER_DISTANCE_SINGLE) {
    // 单次测距
    packet.length = std_create_packet(MSG_ID_RANGE_SINGLE, NULL, 0, packet.data, sizeof(packet.data));
  } else if (command == LASER_DISTANCE_CONTINUOUS) {
    // 连续测距
    packet.length = std_create_packet(MSG_ID_RANGE_CONTINUOUS, NULL, 0, packet.data, sizeof(packet.data));
  } else if (command == LASER_DISTANCE_STOP) {
    // 停止测距
    packet.length = std_create_packet(MSG_ID_RANGE_STOP, NULL, 0, packet.data, sizeof(packet.data));
  }
  internal_auto_send(&packet);  // 自动发送
  return packet;                // 确保所有路径都有返回值
}

std_packet_t std_control_laser_ranging_async(std_laser_distance_control_t command, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (command < LASER_DISTANCE_SINGLE || command >= LASER_DISTANCE_END) {
    return packet;
  }

  uint16_t pkt_len = 0;

  if (command == LASER_DISTANCE_SINGLE) {
    // 单次测距
    pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                   MSG_ID_RANGE_SINGLE, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  } else if (command == LASER_DISTANCE_CONTINUOUS) {
    // 连续测距
    pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                   MSG_ID_RANGE_CONTINUOUS, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  } else if (command == LASER_DISTANCE_STOP) {
    // 停止测距
    pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                   MSG_ID_RANGE_STOP, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  }

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建人车识别控制协议包
*@param command : 人车识别控制命令;
    AI_DETECTION_ON = 0x01,   // 人车识别开
    AI_DETECTION_OFF = 0x00,  // 人车识别关
* @return 返回创建的协议包
*/
std_packet_t std_control_object_recognition(std_ai_detection_t command) {
  if (command < AI_DETECTION_OFF || command >= AI_DETECTION_END) {
    // 无效的人车识别命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));  // 初始化空包

  if (command == AI_DETECTION_ON) {
    // 人车识别开
    packet.length = std_create_packet(MSG_ID_AI_DETECTION_ON, NULL, 0, packet.data, sizeof(packet.data));
  } else if (command == AI_DETECTION_OFF) {
    // 人车识别关
    packet.length = std_create_packet(MSG_ID_AI_DETECTION_OFF, NULL, 0, packet.data, sizeof(packet.data));
  }
  internal_auto_send(&packet);  // 自动发送
  return packet;                // 确保所有路径都有返回值
}

std_packet_t std_control_object_recognition_async(std_ai_detection_t command, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));  // 初始化空包

  // 状态校验
  if (!g_active_sdk) return packet;

  if (command < AI_DETECTION_OFF || command >= AI_DETECTION_END) {
    return packet;
  }

  uint16_t pkt_len = 0;

  if (command == AI_DETECTION_ON) {
    // 人车识别开
    pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                   MSG_ID_AI_DETECTION_ON, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  } else if (command == AI_DETECTION_OFF) {
    // 人车识别关
    pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                   MSG_ID_AI_DETECTION_OFF, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  }

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 * @brief 创建GPS导航控制协议包
 * @param longitude  经度   deg * 1e7
 * @param latitude   纬度   deg * 1e7
 * @param altitude   海拔   mm
 * @param relative_height  相对高度   mm
 * @param heading    航向   degree * 100 (0.01°)
 * @return std_packet_t
 */
std_packet_t std_control_gps_navigation(int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_height, uint16_t heading) {
  std_packet_t packet;
  uint8_t payload[18];

  std_int32_to_data(longitude, payload);
  std_int32_to_data(latitude, payload + 4);
  std_int32_to_data(altitude, payload + 8);
  std_int32_to_data(relative_height, payload + 12);
  std_uint16_to_data(heading, payload + 16);

  packet.length = std_create_packet(MSG_ID_GPS_SET, payload, 18, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送

  return packet;
}

std_packet_t std_control_gps_navigation_async(int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_height, uint16_t heading, ResultCallback cb,
                                              void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[18];

  std_int32_to_data(longitude, payload);
  std_int32_to_data(latitude, payload + 4);
  std_int32_to_data(altitude, payload + 8);
  std_int32_to_data(relative_height, payload + 12);
  std_uint16_to_data(heading, payload + 16);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_GPS_SET, payload, 18, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建相机参数保存协议包
 *@return 返回创建的协议包
 */
std_packet_t std_control_camera_param_save(void) {
  std_packet_t packet;
  packet.length = std_create_packet(MSG_ID_CAMERA_PARAM_SAVE, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_camera_param_save_async(ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_CAMERA_PARAM_SAVE, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建补光灯控制协议包
*@param command : 补光灯控制命令;
    FILL_LIGHT_OFF = 0x00,   // 补光灯关
    FILL_LIGHT_ON = 0x01,    // 补光灯开
* @return 返回创建的协议包
*/
std_packet_t std_control_fill_light_enable(std_fill_light_control_t command) {
  if (command < FILL_LIGHT_OFF || command >= FILL_LIGHT_END) {
    // 无效的补光灯命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[3] = {FILL_LIGHT_CONTROL_MODE, command, 0};  // mode=0x01(补光灯控制),开关状态，保留位0
  packet.length = std_create_packet(MSG_ID_LIGHT_CONTROL, payload, 3, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_fill_light_enable_async(std_fill_light_control_t command, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (command < FILL_LIGHT_OFF || command >= FILL_LIGHT_END) {
    return packet;
  }

  uint8_t payload[3] = {FILL_LIGHT_CONTROL_MODE, command, 0};  // mode=0x01(补光灯控制),开关状态，保留位0

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_LIGHT_CONTROL, payload, 3, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建补光灯亮度调节协议包
 *@param brightness : 亮度值;
 *@note: 范围[0,255],0最暗,255最亮
 *@return 返回创建的协议包
 */
std_packet_t std_control_fill_light_brightness(uint16_t brightness) {
  if (brightness > 255) {
    brightness = 255;  // 限制亮度在0-255范围内
  }
  std_packet_t packet;
  uint8_t payload[3] = {0};
  payload[0] = FILL_LIGHT_CONTROL_BRIGHTNESS;  // mode=0x02(补光灯亮度调节)

  std_uint16_to_data(brightness, &payload[1]);  // 亮度值, 0-255

  packet.length = std_create_packet(MSG_ID_LIGHT_CONTROL, payload, 3, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_fill_light_brightness_async(uint16_t brightness, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (brightness > 255) {
    brightness = 255;  // 限制亮度在0-255范围内
  }

  uint8_t payload[3] = {0};
  payload[0] = FILL_LIGHT_CONTROL_BRIGHTNESS;  // mode=0x02(补光灯亮度调节)

  std_uint16_to_data(brightness, &payload[1]);  // 亮度值, 0-255

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_LIGHT_CONTROL, payload, 3, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

//补光灯关斑调节
//范围[0,180]
/*
 *@brief 创建补光灯关斑大小调节协议包
 *@param size : 关斑大小;
 *@note: 范围[0,180],0最小,180最大
 *@return 返回创建的协议包
 */
std_packet_t std_control_fill_light_size(uint16_t size) {
  if (size > 180) {
    size = 180;  // 限制关斑大小在0-180范围内
  }
  std_packet_t packet;
  uint8_t payload[3] = {0};
  payload[0] = FILL_LIGHT_CONTROL_SIZE;  // mode=0x03(补光灯关斑大小)

  std_uint16_to_data(size, &payload[1]);  // 频率值, 0-180

  packet.length = std_create_packet(MSG_ID_LIGHT_CONTROL, payload, 3, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_fill_light_size_async(uint16_t size, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (size > 180) {
    size = 180;  // 限制关斑大小在0-180范围内
  }

  uint8_t payload[3] = {0};
  payload[0] = FILL_LIGHT_CONTROL_SIZE;  // mode=0x03(补光灯关斑大小)

  std_uint16_to_data(size, &payload[1]);  // 频率值, 0-180

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_LIGHT_CONTROL, payload, 3, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建云台瞄准校准协议包
 *@param x : 画面X坐标;
 *@param y : 画面Y坐标;
 *@note: 坐标以左上角为原点，分辨率 1920×1080（宽×高） 为参考基准进行归一化,X,Y类型float,范围均为[0,1]
 *@return 返回创建的协议包
 */
std_packet_t std_control_sight_calibration(float x, float y) {
  if (x < 0.0f || x > 1.0f || y < 0.0f || y > 1.0f) {
    // 无效的坐标, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[8];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x, payload);
  std_float_to_data(y, payload + 4);
  packet.length = std_create_packet(MSG_ID_SIGHT_CALIBRATION, payload, 8, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_sight_calibration_async(float x, float y, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (x < 0.0f || x > 1.0f || y < 0.0f || y > 1.0f) {
    return packet;
  }

  uint8_t payload[8];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x, payload);
  std_float_to_data(y, payload + 4);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_SIGHT_CALIBRATION, payload, 8, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
*@brief 创建触发器控制协议包
*@param num : 触发器编号,范围[1,8];
*@param state : 触发器状态;
    TRIGGER_OFF = 0x00,   // 触发器关
    TRIGGER_ON = 0x01,    // 触发器开
*@return 返回创建的协议包
*/
std_packet_t std_trigger_control(uint8_t num, uint8_t state) {
  if (num < 1 || num > 8) {
    // 无效的触发器编号, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[2] = {num, state};  // 触发器编号和状态
  packet.length = std_create_packet(MSG_ID_TRIGGER_CONTROL, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_trigger_control_async(uint8_t num, uint8_t state, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (num < 1 || num > 8) {
    return packet;
  }

  uint8_t payload[2] = {num, state};  // 触发器编号和状态

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TRIGGER_CONTROL, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 *@brief 创建触发器角度控制协议包
 *@param num : 触发器编号,范围[1,8];
 *@param angle : 目标角度;
 *@note: 范围[-90,90],正顺时针为正
 *@return 返回创建的协议包
 */
std_packet_t std_trigger_angle_control(uint8_t num, float angle) {
  if (num < 1 || num > 8) {
    // 无效的触发器编号, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  //角度限幅
  if (angle < -90.0f || angle > 90.0f) {
    angle = (angle < -90.0f) ? -90.0f : 90.0f;  // 限制角度在-90到90范围内
  }

  std_packet_t packet;
  uint8_t payload[5];
  payload[0] = num;                       // 触发器编号
  std_float_to_data(angle, &payload[1]);  // 角度值转换为字节数组
  packet.length = std_create_packet(MSG_ID_TRIGGER_ANGLE_CONTROL, payload, 5, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_trigger_angle_control_async(uint8_t num, float angle, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (num < 1 || num > 8) {
    return packet;
  }

  //角度限幅
  if (angle < -90.0f || angle > 90.0f) {
    angle = (angle < -90.0f) ? -90.0f : 90.0f;  // 限制角度在-90到90范围内
  }

  uint8_t payload[5];
  payload[0] = num;                       // 触发器编号
  std_float_to_data(angle, &payload[1]);  // 角度值转换为字节数组

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TRIGGER_ANGLE_CONTROL, payload, 5, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 * @brief 创建触发器模式切换协议包
 * @param num : 触发器编号, 范围[1,8];
 * @param mode : 触发器模式;
 *    TRIGGER_MODE_AUTO = 0x00,   // 全自动
 *    TRIGGER_MODE_CONTINUOUS = 0x01 //连发
 */
std_packet_t std_trigger_mode_switch(uint8_t num, uint8_t mode) {
  if (num < 1 || num > 8) {
    // 无效的触发器编号, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  if (mode != TRIGGER_MODE_AUTO && mode != TRIGGER_MODE_CONTINUOUS) {
    // 无效的触发器模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  uint8_t payload[2] = {num, mode};  // 触发器编号和模式
  packet.length = std_create_packet(MSG_ID_TRIGGER_MODE_SWITCH, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_trigger_mode_switch_async(uint8_t num, uint8_t mode, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (num < 1 || num > 8) {
    return packet;
  }
  if (mode != TRIGGER_MODE_AUTO && mode != TRIGGER_MODE_CONTINUOUS) {
    return packet;
  }

  uint8_t payload[2] = {num, mode};  // 触发器编号和模式

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TRIGGER_MODE_SWITCH, payload, 2, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 *
 * 触发器时间间隔控制协议包
 * @param num : 触发器编号, 范围[1,8];
 * @param interval_ms : 触发器时间间隔, 单位毫秒;
 * @return 返回创建的协议包
 */
std_packet_t std_trigger_interval_control(uint8_t num, uint16_t interval_ms) {
  if (num < 1 || num > 8) {
    // 无效的触发器编号, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  uint8_t payload[3];
  payload[0] = num;                              // 触发器编号
  std_uint16_to_data(interval_ms, &payload[1]);  // 时间间隔转换为字节数组
  packet.length = std_create_packet(MSG_ID_TRIGGER_INTERVAL, payload, 3, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_trigger_interval_control_async(uint8_t num, uint16_t interval_ms, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (num < 1 || num > 8) {
    return packet;
  }

  uint8_t payload[3];
  payload[0] = num;                              // 触发器编号
  std_uint16_to_data(interval_ms, &payload[1]);  // 时间间隔转换为字节数组

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_TRIGGER_INTERVAL, payload, 3, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/**
 *@brief 创建时间同步协议包
 *@param utc : UTC时间戳，单位秒
 *@return 返回创建的协议包
 */
std_packet_t std_control_time_sync(uint64_t utc) {
  std_packet_t packet;
  uint8_t payload[8];
  std_uint64_to_data(utc, payload);  // 时间戳转换为字节数组
  packet.length = std_create_packet(MSG_ID_UTC_TIME_SYNC, payload, 8, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_time_sync_async(uint64_t utc, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  uint8_t payload[8];
  std_uint64_to_data(utc, payload);  // 时间戳转换为字节数组

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_UTC_TIME_SYNC, payload, 8, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}
/**
 * @brief 创建相机透雾协议包
 * @param mode:
 */
std_packet_t std_control_fogging(std_fogging_control_t mode, std_fogging_strength_t strength) {
  if (mode < FOGGING_MODE_OFF || mode >= FOGGING_MODE_END) {
    // 无效的透雾模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  if (strength < FOGGING_STRENGTH_LOW || strength >= FOGGING_STRENGTH_END) {
    // 无效的透雾强度, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  uint8_t payload[2] = {mode, strength};
  packet.length = std_create_packet(MSG_ID_FOGGING_CONTROL, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/**
 * @brief 创建相机透雾异步协议包
 * @param mode:
 */
std_packet_t std_control_fogging_async(std_fogging_control_t mode, std_fogging_strength_t strength, ResultCallback cb, void* user_data) {
  if (mode < FOGGING_MODE_OFF || mode >= FOGGING_MODE_END) {
    // 无效的透雾模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  if (strength < FOGGING_STRENGTH_LOW || strength >= FOGGING_STRENGTH_END) {
    // 无效的透雾强度, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }

  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));
  uint8_t payload[2] = {mode, strength};

  uint16_t pkt_len = std_cmd_async_create(g_active_sdk, MSG_ID_FOGGING_CONTROL, payload, sizeof(payload), cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * #brief 创建框选跟踪协议包
 * @param x_min : 画面左上角X坐标;
 * @param y_min : 画面左上角Y坐标;
 * @param x_max : 画面右下角X坐标;
 * @param y_max : 画面右下角Y坐标;
 * @note: 坐标以左上角为原点，X,Y类型float,范围均为[0,1]
 * @return 返回创建的协议包
 */
std_packet_t std_control_box_tracking(float x_min, float y_min, float x_max, float y_max) {
  if (x_min < 0.0f || x_min > 1.0f || y_min < 0.0f || y_min > 1.0f || x_max < 0.0f || x_max > 1.0f || y_max < 0.0f || y_max > 1.0f || x_min >= x_max || y_min >= y_max) {
    // 无效的坐标, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[16];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x_min, payload);
  std_float_to_data(y_min, payload + 4);
  std_float_to_data(x_max, payload + 8);
  std_float_to_data(y_max, payload + 12);
  packet.length = std_create_packet(MSG_ID_BOX_TRACKING, payload, 16, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_box_tracking_async(float x_min, float y_min, float x_max, float y_max, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 状态校验
  if (!g_active_sdk) return packet;

  if (x_min < 0.0f || x_min > 1.0f || y_min < 0.0f || y_min > 1.0f || x_max < 0.0f || x_max > 1.0f || y_max < 0.0f || y_max > 1.0f || x_min >= x_max || y_min >= y_max) {
    return packet;
  }

  uint8_t payload[16];
  // 将float转换为字节数组 (小端序)
  std_float_to_data(x_min, payload);
  std_float_to_data(y_min, payload + 4);
  std_float_to_data(x_max, payload + 8);
  std_float_to_data(y_max, payload + 12);

  // 内部创建异步任务时，直接传入全局静态指针
  uint16_t pkt_len = std_cmd_async_create(g_active_sdk,  // 使用静态指针
                                          MSG_ID_BOX_TRACKING, payload, 16, cb, user_data, packet.data, sizeof(packet.data));

  packet.length = pkt_len;

  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 * @brief 创建触发器参数设置协议包
 * @param num : 触发器编号,范围[1,8];
 * @param center_pos : 中位(启动位置),范围-90,90;
 * @param left_limit : 左限位 1~100 ，默认50;
 * @param right_limit : 右限位 1~100，默认50;
 * @param invert : 左右是否翻转,0不翻转，1翻转;
 * @return 返回创建的协议包
 */
std_packet_t std_trigger_parameter_setting(uint8_t num, int8_t center_pos, uint8_t left_limit, uint8_t right_limit, uint8_t invert) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // 参数验证
  if (num < 1 || num > 8) {
    return packet;  // 返回空包
  }

  if (center_pos < -90 || center_pos > 90) {
    return packet;
  }

  if (left_limit < 1 || left_limit > 100 || right_limit < 1 || right_limit > 100) {
    return packet;
  }

  if (invert > 1) {
    return packet;
  }
  uint8_t payload[5];
  // 初始化结构体
  std_trigger_param_t param = {.servo_id = num, .center_pos = center_pos, .left_limit = left_limit, .right_limit = right_limit, .invert = invert};

  // 安全拷贝：确保不会溢出
  size_t copy_size = sizeof(param);
  if (copy_size > sizeof(payload)) {
    copy_size = sizeof(payload);
  }
  memcpy(payload, &param, copy_size);

  packet.length = std_create_packet(MSG_ID_TRIGGER_PARAMER_SETTING, payload, sizeof(payload), packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 * @brief 创建触发器参数设置协议包,异步接口
 * @param num : 触发器编号,范围[1,8];
 * @param center_pos : 中位(启动位置),范围-90,90;
 * @param left_limit : 左限位 1~100 ，默认50;
 * @param right_limit : 右限位 1~100，默认50;
 * @param invert : 左右是否翻转,0不翻转，1翻转;
 * @return 返回创建的协议包
 */
std_packet_t std_trigger_parameter_setting_async(gimbal_sdk_t* sdk, uint8_t num, int8_t center_pos, uint8_t left_limit, uint8_t right_limit, uint8_t invert,
                                                 ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  // ---------- 参数校验 ----------
  if (!sdk || !cb) return packet;
  if (num < 1 || num > 8) return packet;
  if (center_pos < -90 || center_pos > 90) return packet;
  if (left_limit < 1 || left_limit > 100 || right_limit < 1 || right_limit > 100) return packet;
  if (invert > 1) return packet;

  // ---------- 构造 payload ----------
  std_trigger_param_t param = {.servo_id = num, .center_pos = center_pos, .left_limit = left_limit, .right_limit = right_limit, .invert = invert};
  // ---------- 一行完成 async 命令 ----------
  packet.length = std_cmd_async_create(sdk, MSG_ID_TRIGGER_PARAMER_SETTING, (uint8_t*)&param, sizeof(param), cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送

  return packet;
}

/*
 * @brief 创建触发器参数保存协议包
 * @return 返回创建的协议包
 */
std_packet_t std_trigger_parameter_save() {
  std_packet_t packet;

  packet.length = std_create_packet(MSG_ID_TRIGGER_PARAMER_SAVE, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建触发器参数保存异步协议包
 * @param cb : 触发器参数保存回调函数;
 * @param user_data : 用户数据指针;
 * @return 返回创建的协议包
 */
std_packet_t std_trigger_parameter_save_async(gimbal_sdk_t* sdk, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  // ---------- 一行完成 async 命令 ----------
  packet.length = std_cmd_async_create(sdk, MSG_ID_TRIGGER_PARAMER_SAVE, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建订阅触发器参数协议包
 * @param num : 触发器编号,范围[1,8];
 * @param cb : 触发器参数读取回调函数;
 * @param user_data : 用户数据指针;
 * @return 返回创建的协议包
 */
std_packet_t std_subscribe_trigger_parameter_async(gimbal_sdk_t* sdk, uint8_t num, trigger_param_callback_t cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  if (num < 1 || num > 8) {
    return packet;
  }

  // 保存“本次请求”的回调上下文
  sdk->trigger_param_req.cb = cb;
  sdk->trigger_param_req.user_data = user_data;

  // 生成协议包
  uint8_t payload[1] = {num};
  packet.length = std_create_packet(MSG_ID_TRIGGER_PARAMER_READ, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 取消订阅触发器参数异步
 * @return 无
 */
void unsubscribe_trigger_parameter_async(gimbal_sdk_t* sdk) {
  if (sdk) {
    sdk->trigger_param_req.cb = NULL;
    sdk->trigger_param_req.user_data = NULL;
  }
}

/**
 * @brief 创建ACK协议包
 * @param msg_id : 需要确认的消息ID
 * @param result : 结果状态
 * @return 返回创建的协议包
 */
std_packet_t std_ack(uint8_t msg_id, StdResult result) {
  std_packet_t packet;
  uint8_t payload[2];
  payload[0] = msg_id;
  payload[1] = (uint8_t)result;
  packet.length = std_create_packet(MSG_ID_ACK, payload, 2, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
* @brief 创建ICR模式控制协议包
* @param mode : ICR模式;
    ICR_AUTO = 0x00,          // 自动
    ICR_VISIBLE_LIGHT = 0x01, // 可见光
    ICR_NIGHT_LIGHT = 0x02,   // 夜视
* @return 返回创建的协议包
*/
std_packet_t std_control_icr_mode(std_icr_mode_t mode) {
  if (mode < ICR_AUTO || mode > ICR_NIGHT_LIGHT) {
    // 无效的ICR模式, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;

  switch (mode) {
    case ICR_AUTO:
      packet.length = std_create_packet(MSG_ID_ICR_AUTO, NULL, 0, packet.data, sizeof(packet.data));
      break;

    case ICR_VISIBLE_LIGHT:
      packet.length = std_create_packet(MSG_ID_ICR_VISIBLE_LIGHT, NULL, 0, packet.data, sizeof(packet.data));
      break;

    case ICR_NIGHT_LIGHT:
      packet.length = std_create_packet(MSG_ID_ICR_NIGHT_LIGHT, NULL, 0, packet.data, sizeof(packet.data));
      break;

    default:
      break;
  }
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

std_packet_t std_control_icr_mode_async(gimbal_sdk_t* sdk, std_icr_mode_t mode, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  if (mode < ICR_AUTO || mode > ICR_NIGHT_LIGHT) {
    return packet;
  }

  uint8_t msg_id = 0;
  switch (mode) {
    case ICR_AUTO:
      msg_id = MSG_ID_ICR_AUTO;
      break;

    case ICR_VISIBLE_LIGHT:
      msg_id = MSG_ID_ICR_VISIBLE_LIGHT;
      break;

    case ICR_NIGHT_LIGHT:
      msg_id = MSG_ID_ICR_NIGHT_LIGHT;
      break;

    default:
      return packet;
  }

  packet.length = std_cmd_async_create(sdk, msg_id, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}
/*
*@brief 创建红外控制协议包
*@param command : 红外控制命令;
    IR_ON = 0x01,    // 测温开
    IR_OFF = 0x02,   // 测温关
    IR_SWITCH_1 = 0x03, //切换温度测量档位1
    IR_SWITCH_2 = 0x04, //切换温度测量档位2
*/
std_packet_t std_control_ir_control(std_ir_control_t command) {
  if (command < IR_ON || command >= IR_END) {
    // 无效的红外控制命令, 返回空包
    std_packet_t empty_packet;
    memset(&empty_packet, 0, sizeof(std_packet_t));
    return empty_packet;
  }
  std_packet_t packet;
  uint8_t payload[1] = {command};
  packet.length = std_create_packet(MSG_ID_IR_CONTROL, payload, 1, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
* @brief 创建红外控制异步协议包
* @param command : 红外控制命令;
    IR_ON = 0x01,    // 测温开
    IR_OFF = 0x02,   // 测温关
    IR_SWITCH_1 = 0x03, //切换温度测量档位1
    IR_SWITCH_2 = 0x04, //切换温度测量档位2
*/
std_packet_t std_control_ir_control_async(gimbal_sdk_t* sdk, std_ir_control_t command, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  if (command < IR_ON || command >= IR_END) {
    return packet;
  }

  uint8_t cmd_val = (uint8_t)command;

  packet.length = std_cmd_async_create(sdk, MSG_ID_IR_CONTROL,
                                       &cmd_val,  // 传入指针
                                       1,         // 长度为 1
                                       cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建订阅红外温度数据协议包
 * @param hz : 订阅频率,单位Hz;
 */
std_packet_t std_subscribe_ir_temperature_data(uint8_t hz, ir_temperature_callback_t cb, void* user_data) {
  if (g_active_sdk && cb) {
    g_active_sdk->ir_temperature_req.cb = cb;
    g_active_sdk->ir_temperature_req.user_data = user_data;
    return std_control_subscribe_stream(STD_MSG_STREAM_IR_TEMPERATURE, (float)hz);
  }
  std_packet_t empty;
  memset(&empty, 0, sizeof(std_packet_t));
  return empty;
}

/**
 * @brief 取消订阅红外温度数据
 * @return std_packet_t : 返回生成的取消订阅协议包
 */
std_packet_t std_unsubscribe_ir_temperature_data(void) {
  // 直接透传底层生成的包
  return std_control_unsubscribe_stream(STD_MSG_STREAM_IR_TEMPERATURE);
}

/**
 * @brief 创建订阅云台姿态数据协议包
 * @param hz : 订阅频率,单位Hz;
 */
std_packet_t std_subscribe_attitude_data(uint8_t hz, attitude_callback_t cb, void* user_data) {
  if (g_active_sdk && cb) {
    g_active_sdk->attitude_req.cb = cb;
    g_active_sdk->attitude_req.user_data = user_data;
    return std_control_subscribe_stream(STD_MSG_STREAM_ATTITUDE, (float)hz);
  }
  std_packet_t empty;
  memset(&empty, 0, sizeof(std_packet_t));
  return empty;
}

/**
 * @brief 取消订阅云台姿态数据
 * @return std_packet_t : 返回生成的取消订阅协议包
 */
std_packet_t std_unsubscribe_attitude_data(void) { return std_control_unsubscribe_stream(STD_MSG_STREAM_ATTITUDE); }

/**
 * @brief 日志功能控制接口 (普通版)
 * @param enable : 1 开启日志, 0 关闭日志
 * @return std_packet_t : 生成的协议包
 */
std_packet_t std_control_logging(std_log_control_t enable) { return _create_and_send_sync_cmd(STD_CMD_LOGGING_START, (float)enable, 0, 0, 0, 0, 0, 0); }

/**
 * @brief 日志功能控制接口 (异步版，带回调)
 * @param enable : 1 开启日志, 0 关闭日志
 * @param cb : 结果回调函数
 * @param user_data : 用户数据
 */
std_packet_t std_control_logging_async(std_log_control_t enable, ResultCallback cb, void* user_data) {
  return _create_and_send_async_cmd(STD_CMD_LOGGING_START, (float)enable, 0, 0, 0, 0, 0, 0, cb, user_data);
}

/**
 * @brief 烟雾/喷火器开关
 * @param enable : 1 开启烟雾/喷火器, 0 关闭烟雾/喷火器
 * @return 返回创建的协议包
 */
std_packet_t std_control_smoke_control(std_smoke_control_t enable) { return _create_and_send_sync_cmd(STD_CMD_SMOKE_CONTROL, (float)enable, 0, 0, 0, 0, 0, 0); }

/**
 * @brief 烟雾/喷火器开关 异步版，带回调
 * @param enable : 1 开启烟雾/喷火器, 0 关闭烟雾/喷火器
 * @param cb : 结果回调函数
 * @param user_data : 用户数据
 * @return 返回创建的协议包
 */
std_packet_t std_control_smoke_control_async(std_smoke_control_t enable, ResultCallback cb, void* user_data) {
  return _create_and_send_async_cmd(STD_CMD_SMOKE_CONTROL, (float)enable, 0, 0, 0, 0, 0, 0, cb, user_data);
}

//==================== TGA 相关协议包创建 BEGIN ===================//

/*
 * @brief 创建订阅TGA Scouter数据协议包
 * @param cb : 触发器参数读取回调函数;
 * @param user_data : 用户数据指针;
 * @return 返回创建的协议包
 */
std_packet_t std_subscribe_tga_scouter_data(gimbal_sdk_t* sdk, scouter_param_callback_t cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  // 将回调和用户数据封装到结构体中
  sdk->scouter_param_req.cb = cb;
  sdk->scouter_param_req.user_data = user_data;

  // 构造订阅 / 读取命令
  packet.length = tga_create_packet(MSG_TGA_ID_SCOUTER_PARA_READ,  // 这里使用 TGA 的读取消息ID
                                    NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 取消订阅TGA Scouter数据
 * @return 无
 */
void std_unsubscribe_tga_scouter_data(gimbal_sdk_t* sdk) {
  // 清除回调和用户数据
  if (sdk) {
    sdk->scouter_param_req.cb = NULL;
    sdk->scouter_param_req.user_data = NULL;
  }
}

/*
 * @brief 创建保存TGA Scouter参数协议包
 * @return 返回创建的协议包
 */
std_packet_t std_scouter_param_save() {
  std_packet_t packet;

  packet.length = tga_create_packet(MSG_TGA_ID_SCOUTER_PARA_SAVE, NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建保存TGA Scouter参数异步协议包
 * @param cb : 保存回调函数;
 * @param user_data : 用户数据指针;
 * @return 返回创建的协议包
 */
std_packet_t std_scouter_param_save_async(gimbal_sdk_t* sdk,  // SDK上下文指针
                                          ResultCallback cb,  // 结果回调
                                          void* user_data     // 用户this指针
) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  // ---------- 一行完成 async 命令 ----------
  packet.length = tga_cmd_async_create(sdk, MSG_TGA_ID_SCOUTER_PARA_SAVE, NULL, 0, cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建设置TGA Scouter参数协议包
 * @param param : 参数结构体指针;
 * @return 返回创建的协议包
 */
std_packet_t std_scouter_param_set(const std_scouter_paramterst_t* param) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!param) {
    return packet;
  }

  uint8_t payload[sizeof(std_scouter_paramterst_t)];
  memcpy(payload, param, sizeof(std_scouter_paramterst_t));

  packet.length = tga_create_packet(MSG_TGA_ID_SCOUTER_PARA_WRITE, payload, sizeof(std_scouter_paramterst_t), packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建设置TGA Scouter参数协议包
 * @param param : 参数结构体指针;
 * @return 返回创建的协议包
 */
std_packet_t std_scouter_param_set_async(gimbal_sdk_t* sdk, const std_scouter_paramterst_t* param, ResultCallback cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !param || !cb) {
    return packet;
  }

  uint8_t payload[sizeof(std_scouter_paramterst_t)];
  memcpy(payload, param, sizeof(std_scouter_paramterst_t));

  // ---------- 一行完成 async 命令 ----------
  packet.length = tga_cmd_async_create(sdk, MSG_TGA_ID_SCOUTER_PARA_WRITE, payload, sizeof(std_scouter_paramterst_t), cb, user_data, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/*
 * @brief 创建订阅TGA sbus通道值数据协议包
 * @param cb : sbus通道值数据回调函数;
 * @param user_data : 用户数据指针;
 * @return 返回创建的协议包
 */
std_packet_t std_subscribe_tga_sbus_channel_data(gimbal_sdk_t* sdk, sbus_channel_callback_t cb, void* user_data) {
  std_packet_t packet;
  memset(&packet, 0, sizeof(std_packet_t));

  if (!sdk || !cb) {
    return packet;
  }

  // 将回调和用户数据封装到结构体中
  sdk->sbus_channel_req.cb = cb;
  sdk->sbus_channel_req.user_data = user_data;

  // 构造订阅 / 读取命令
  packet.length = tga_create_packet(MSG_TGA_ID_SCOUTER_SBUS_UPLOAD,  // 这里使用 TGA 的读取消息ID
                                    NULL, 0, packet.data, sizeof(packet.data));
  internal_auto_send(&packet);  // 自动发送
  return packet;
}

/**
 * * @brief 取消订阅TGA sbus通道值数据
 * * @return 无
 */
void std_unsubscribe_tga_sbus_channel_data(gimbal_sdk_t* sdk) {
  // 清除回调和用户数据
  if (sdk) {
    sdk->sbus_channel_req.cb = NULL;
    sdk->sbus_channel_req.user_data = NULL;
  }
}

//==================== TGA 相关协议包创建 BEGIN ===================//

/**
 * @brief 更新校验值
 * @param ctx 上下文指针
 * @param ch  新接收的字节
 */
static void update_check(unpack_ctx_t* ctx, uint8_t ch) {
  ctx->sumCheck += ch;
  ctx->addCheck += ctx->sumCheck;
}

/**
 * @brief 状态机复位（回到初始状态，清除缓存）
 * @param ctx 上下文指针
 */
static void reset_unpack(unpack_ctx_t* ctx) {
  ctx->state = STATE_WAIT_HEADER1;
  ctx->dataIndex = 0;
  ctx->sumCheck = 0;
  ctx->addCheck = 0;
  ctx->frameReady = 0;
}

/**
 * @brief 转换原始反馈数据到用户友好的格式
 * @param raw 原始数据指针
 * @param converted 转换后数据指针
 */
void convert_feedback_data(const std_feedback_t* raw, std_feedback_converted_t* converted) {
  if (!raw || !converted) return;

  // 直接复制的字段
  converted->pod_status_1 = raw->pod_status_t1;
  converted->pod_status_2 = raw->pod_status_t2;
  converted->off_target_x = raw->off_target_x;
  converted->off_target_y = raw->off_target_y;

  // 单位转换的字段
  converted->roll_angle = raw->roll_angle * 0.1f;          // 0.1度 -> 度
  converted->pitch_angle = raw->pitch_angle * 0.1f;        // 0.1度 -> 度
  converted->yaw_angle = raw->yaw_angle * 0.1f;            // 0.1度 -> 度
  converted->laser_distance = raw->laser_distance * 0.1f;  // 分米 -> 米
  converted->zoom_ratio = raw->zoom_ratio * 0.1f;          // 0.1倍 -> 倍

  // GPS坐标转换 (度 * 1e7 -> 度)
  converted->target_longitude = raw->target_longitude * 1e-7;
  converted->target_latitude = raw->target_latitude * 1e-7;
  converted->target_altitude = raw->target_altitude * 0.1f;  // 分米 -> 米
  converted->target_distance = raw->target_distance * 0.1f;  // 分米 -> 米
}

/**
 * @brief 转换触发器反馈数据
 * @param raw 原始数据指针
 * @param converted 转换后数据指针
 */
void convert_trigger_feedback_data(const std_trigger_feedback_t* raw, std_trigger_feedback_converted_t* converted) {
  if (!raw || !converted) return;

  // 触发器数据单位已经是正确的，直接复制
  converted->x = raw->x;
  converted->y = raw->y;
  converted->state = raw->state;
  converted->mode = raw->mode;
  converted->interval_ms = raw->interval_ms;
}

/**
 * @brief 处理完整的数据包
 * @param sdk 云台SDK实例指针
 * @param ctx 数据包指针
 */
void process_packet(gimbal_sdk_t* sdk, std_pk_t* ctx) {
  // recvice id is 0x03(gimbal) 0x02(user)
  if (ctx->Slave_id == STD_SENDER_ID) {
    uint8_t message_id = ctx->MSG_id;

    switch (message_id) {
      case MSG_ID_FEEDBACK: {
        if (sdk->on_feedback) {
          std_feedback_t raw_feedback;
          std_feedback_converted_t converted_feedback;

          // 解析原始数据
          memcpy(&raw_feedback, ctx->Data, sizeof(raw_feedback));

          // 转换数据单位
          convert_feedback_data(&raw_feedback, &converted_feedback);

          // 调用用户回调，传递转换后的数据
          sdk->on_feedback(&converted_feedback, sdk->feedback_user_data);
        }
        break;
      }

      case MSG_ID_TURNTABLE_REPORTTED: {
        if (sdk->on_trigger_feedback) {
          std_trigger_feedback_t raw_trigger;
          std_trigger_feedback_converted_t converted_trigger;

          // 解析原始数据
          memcpy(&raw_trigger, ctx->Data, sizeof(raw_trigger));

          // 转换数据（这里主要是为了结构一致性）
          convert_trigger_feedback_data(&raw_trigger, &converted_trigger);

          // 调用用户回调
          sdk->on_trigger_feedback(&converted_trigger, sdk->trigger_user_data);
        }
        break;
      }

      case MSG_ID_TRIGGER_PARAMER_READ: {
        trigger_param_request_t* req = &sdk->trigger_param_req;
        if (ctx->Data_length != sizeof(std_trigger_param_t)) {
          break;
        }
        std_trigger_param_t param;
        memcpy(&param, ctx->Data, sizeof(param));

        // 调用一次性回调
        req->cb(&param, req->user_data);

        // 自动清空（一次性）
        memset(req, 0, sizeof(*req));
        break;
      }

      case MSG_ID_ACK: {
        /* ACK 数据至少 2 字节：
        Data[0] = acked_msg_id 低位
        Data[1] = acked_msg_id 高位
        Data[2] = StdResult
        */
        if (ctx->Data_length < 2) {
          break;
        }

        uint8_t acked_msg_id = std_data_to_uint16(&ctx->Data[0]);
        StdResult result = (StdResult)ctx->Data[2];

        for (int i = 0; i < CMD_WORK_QUEUE_SIZE; i++) {
          cmd_work_t* work = &sdk->cmd_queue.works[i];

          if (!work->used) {
            continue;
          }

          if (work->msg_id != acked_msg_id) {
            continue;
          }

          /* ===== 命中事务，回调 ===== */
          if (work->cb) {
            work->cb(acked_msg_id, result, work->user_data);
          }

          /* ===== 一次性事务完成 ===== */
          cmd_work_free(work);

          break;
        }
        break;
      }
      case MSG_ID_MESSAGES_STREAM: {
        if (ctx->Data_length < 2) break;
        uint16_t stream_id = std_data_to_uint16(&ctx->Data[0]);
        // 指向流数据载荷起始位置 (跳过 2 字节的 stream_id)
        uint8_t* stream_payload = &ctx->Data[2];
        uint8_t stream_payload_len = ctx->Data_length - 2;
        // 3. 根据 stream_id 分发处理
        switch (stream_id) {
          case STD_MSG_STREAM_IR_TEMPERATURE: {
            if (sdk->ir_temperature_req.cb) {
              // 校验载荷长度是否匹配 std_ir_temperature_t (3个float = 12字节)
              if (stream_payload_len >= sizeof(std_ir_temperature_t)) {
                std_ir_temperature_t ir_temp_data;
                // 直接拷贝数据到结构体
                memcpy(&ir_temp_data, stream_payload, sizeof(std_ir_temperature_t));
                // 执行回调
                sdk->ir_temperature_req.cb(&ir_temp_data, sdk->ir_temperature_req.user_data);
              }
            }
            break;
          }
          case STD_MSG_STREAM_ATTITUDE: {
            // 如果以后有姿态流，代码逻辑如下：
            if (sdk->attitude_req.cb) {
              std_attitude_t attitude_data;
              memcpy(&attitude_data, stream_payload, sizeof(std_attitude_t));
              sdk->attitude_req.cb(&attitude_data, sdk->attitude_req.user_data);
            }
            break;
          }
          case STD_MSG_STREAM_GPS: {
            // 如果以后有解析 GPS 流的逻辑...
            break;
          }
          default:
            // 未定义的流 ID
            break;
        }
        break;
      }
      default:
        // 未知的消息ID，忽略
        break;
    }
  } else if (ctx->Slave_id == 0x00) {
    uint8_t message_id = ctx->MSG_id;
    switch (message_id) {
      case MSG_TGA_ID_SCOUTER_PARA_UPLOAD: {  // scouter数据

        if (sdk->scouter_param_req.cb) {  //回调存在
          // 调用用户回调
          std_scouter_paramterst_t scouter_params;
          memcpy(&scouter_params, ctx->Data, sizeof(scouter_params));
          // 调用回调函数
          sdk->scouter_param_req.cb(&scouter_params, sdk->scouter_param_req.user_data);
        }
      } break;

      case MSG_TGA_ID_SCOUTER_SBUS_UPLOAD:  // sbus通道数据
      {
        if (sdk->sbus_channel_req.cb) {
          // 调用用户回调
          std_scouter_sbus_t sbus_channels;
          memcpy(&sbus_channels, ctx->Data, sizeof(sbus_channels));
          // 调用回调函数
          sdk->sbus_channel_req.cb(&sbus_channels, sdk->sbus_channel_req.user_data);
        }
      } break;

      case MSG_TGA_REPLY_PALINDROME: {
        uint8_t acked_msg_id = 0;
        StdResult result = Failed;
        if (ctx->Data[0] == MSG_TGA_ACK_SET_CONFIG) {
          acked_msg_id = MSG_TGA_ID_SCOUTER_PARA_WRITE;
          result = Success;

        } else if (ctx->Data[0] == MSG_TGA_ACK_SAVE) {
          acked_msg_id = MSG_TGA_ID_SCOUTER_PARA_SAVE;
          result = Success;
        }

        for (int i = 0; i < CMD_WORK_QUEUE_SIZE; i++) {
          cmd_work_t* work = &sdk->cmd_queue.works[i];

          if (!work->used) {
            continue;
          }

          if (work->msg_id != acked_msg_id) {
            continue;
          }

          /* ===== 命中事务，回调 ===== */
          if (work->cb) {
            work->cb(acked_msg_id, result, work->user_data);
          }

          /* ===== 一次性事务完成 ===== */
          cmd_work_free(work);
        }
      } break;

      case MSG_TGA_ID_ACK: {
        /* ACK 数据至少 2 字节：
        Data[0] = acked_msg_id
        Data[1] = StdResult
        */
        if (ctx->Data_length < 2) {
          break;
        }

        uint8_t acked_msg_id = ctx->Data[0];
        StdResult result = (StdResult)ctx->Data[1];

        for (int i = 0; i < CMD_WORK_QUEUE_SIZE; i++) {
          cmd_work_t* work = &sdk->cmd_queue.works[i];

          if (!work->used) {
            continue;
          }

          if (work->msg_id != acked_msg_id) {
            continue;
          }

          /* ===== 命中事务，回调 ===== */
          if (work->cb) {
            work->cb(acked_msg_id, result, work->user_data);
          }

          /* ===== 一次性事务完成 ===== */
          cmd_work_free(work);
        }
      } break;

      default: {
        // 未知的消息ID，忽略
      } break;
    }
  } else {
    // 接收方ID不匹配，忽略
  }
}

/**
 * @brief 逐字节解析函数
 * @param ctx 上下文指针（每个通道一个）
 * @param ch  新接收的字节
 * @return 1 = 解包成功，0 = 未完成
 */
uint8_t Unpack_Function(unpack_ctx_t* ctx, uint8_t ch) {
  switch (ctx->state) {
    case STATE_WAIT_HEADER1: /* 等待帧头1 */
      if (ch == STD_HEADER_1) {
        reset_unpack(ctx);               /* 清理状态机 */
        ctx->state = STATE_WAIT_HEADER2; /* 转到等待帧头2 */
        ctx->rxPacket.Frame_Header = ch;
        update_check(ctx, ch);
      }
      break;

    case STATE_WAIT_HEADER2: /* 等待帧头2 */
      if (ch == STD_HEADER_1 || ch == STD_HEADER_2) {
        ctx->state = STATE_WAIT_MASTER_ID;
        ctx->rxPacket.Function_Mark = ch;
        update_check(ctx, ch);
      } else {
        reset_unpack(ctx); /* 非法，重新等待帧头 */
      }
      break;

    case STATE_WAIT_MASTER_ID: /* 发送方ID */
      ctx->rxPacket.Master_id = ch;
      ctx->state = STATE_WAIT_SLAVE_ID;
      update_check(ctx, ch);
      break;

    case STATE_WAIT_SLAVE_ID: /* 接收方ID */
      ctx->rxPacket.Slave_id = ch;
      ctx->state = STATE_WAIT_MSG_ID;
      update_check(ctx, ch);
      break;

    case STATE_WAIT_MSG_ID: /* 消息ID */
      ctx->rxPacket.MSG_id = ch;
      ctx->state = STATE_WAIT_LENGTH;
      update_check(ctx, ch);
      break;

    case STATE_WAIT_LENGTH: /* 数据长度 */
      if (ch <= TCA_MAX_DATA_SIZE) {
        ctx->rxPacket.Data_length = ch;
        ctx->dataIndex = 0;
        ctx->state = (ch > 0) ? STATE_WAIT_DATA : STATE_WAIT_SUM;
        update_check(ctx, ch);
      } else {
        reset_unpack(ctx); /* 数据长度非法，丢弃 */
      }
      break;

    case STATE_WAIT_DATA: /* 数据区 */
      ctx->rxPacket.Data[ctx->dataIndex++] = ch;
      update_check(ctx, ch);
      if (ctx->dataIndex >= ctx->rxPacket.Data_length) ctx->state = STATE_WAIT_SUM;
      break;

    case STATE_WAIT_SUM: /* 校验和 */
      if (ctx->sumCheck == ch) {
        ctx->rxPacket.Sum_check = ch;
        ctx->state = STATE_WAIT_ADDITIONAL;
      } else {
        reset_unpack(ctx); /* 校验错误，丢弃 */
      }
      break;

    case STATE_WAIT_ADDITIONAL: /* 附加校验 */

      if (ctx->addCheck == ch) {
        ctx->rxPacket.Additional_check = ch;
        ctx->frameReady = 1; /* 标记帧完成 */
        reset_unpack(ctx);   /* 准备下一帧 */
        return 1;            /* 解包成功 */
      }
      reset_unpack(ctx); /* 校验失败，丢弃 */
      break;

    default:
      reset_unpack(ctx);
      break;
  }
  return 0;
}

/*
 * @brief 创建异步命令协议包
 * @param sdk : 云台SDK实例指针;
 * @param msg_id : 消息ID;
 * @param payload : 负载数据指针;
 * @param payload_len : 负载数据长度;
 * @param cb : 结果回调函数指针;
 * @param user_data : 用户数据指针;
 * @param out_buf : 输出缓冲区指针;
 * @param out_buf_size : 输出缓冲区大小;
 */
uint16_t std_cmd_async_create(gimbal_sdk_t* sdk, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, ResultCallback cb, void* user_data, uint8_t* out_buf,
                              uint16_t out_buf_size) {
  if (!sdk || !cb || !out_buf) {
    return 0;
  }

  // 分配命令事务
  cmd_work_t* work = cmd_work_alloc(sdk);
  if (!work) {
    return 0;  // 队列满
  }

  // --- 使用 SDK 实例中定义的参数 ---
  work->timeout_ms = sdk->default_timeout_ms;
  work->retries_left = sdk->default_retries;
  work->time_started_ms = 0;  // 内部计时逻辑，用户不可改

  // 填事务信息
  if (msg_id == MSG_COMMAND_LONG && payload != NULL && payload_len >= 2) {
    // 如果是 COMMAND_LONG，将其内部的子命令 ID 作为事务匹配 ID
    work->msg_id = (uint8_t)std_data_to_uint16(payload);
  } else {
    work->msg_id = msg_id;
  }
  work->cb = cb;
  work->user_data = user_data;

  // 创建协议包
  uint16_t len = std_create_packet(msg_id, payload, payload_len, out_buf, out_buf_size);

  if (len > 0) {
    memcpy(work->packet.data, out_buf, len);
    work->packet.length = len;
  }

  return len;
}

/*
 * @brief 创建异步命令协议包
 * @param sdk : 云台SDK实例指针;
 * @param msg_id : 消息ID;
 * @param payload : 负载数据指针;
 * @param payload_len : 负载数据长度;
 * @param cb : 结果回调函数指针;
 * @param user_data : 用户数据指针;
 * @param out_buf : 输出缓冲区指针;
 * @param out_buf_size : 输出缓冲区大小;
 */
uint16_t tga_cmd_async_create(gimbal_sdk_t* sdk, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, ResultCallback cb, void* user_data, uint8_t* out_buf,
                              uint16_t out_buf_size) {
  if (!sdk || !cb || !out_buf) {
    return 0;
  }

  // 分配命令事务
  cmd_work_t* work = cmd_work_alloc(sdk);
  if (!work) {
    return 0;  // 队列满
  }

  // 填事务信息
  work->msg_id = msg_id;
  work->cb = cb;
  work->user_data = user_data;

  // 创建协议包
  return tga_create_packet(msg_id, payload, payload_len, out_buf, out_buf_size);
}

/**
 * @brief 初始化云台SDK
 * @param sdk 云台SDK实例指针
 * @param write_func 底层发送函数（如串口发送），如果传 NULL 则不自动发送
 * @param write_user_ptr 发送函数用到的句柄（如 UART 结构体指针）
 */
void gimbal_sdk_init(gimbal_sdk_t* sdk, sdk_write_func_t write_func, void* write_user_ptr) {
  if (!sdk) return;

  memset(sdk, 0, sizeof(*sdk));

  // 设置出厂默认重发参数
  sdk->default_timeout_ms = 200;
  sdk->default_retries = 3;

  // 绑定发送接口
  sdk->write_func = write_func;
  sdk->write_user_ptr = write_user_ptr;

  // 初始化时自动将当前 sdk 设为激活实例
  g_active_sdk = sdk;
}

/**
 * @brief 注册反馈回调函数
 * @param sdk 云台SDK实例指针
 * @param cb 反馈回调函数
 * @param user_data 用户数据指针
 */
void gimbal_sdk_register_feedback_callback(gimbal_sdk_t* sdk, feedback_callback_t cb, void* user_data) {
  sdk->on_feedback = cb;
  sdk->feedback_user_data = user_data;
}

/**
 * @brief 注册触发器反馈回调函数
 * @param sdk 云台SDK实例指针
 * @param cb 触发器反馈回调函数
 * @param user_data 用户数据指针
 */
void gimbal_sdk_register_trigger_feedback_callback(gimbal_sdk_t* sdk, trigger_feedback_callback_t cb, void* user_data) {
  sdk->on_trigger_feedback = cb;
  sdk->trigger_user_data = user_data;
}

/**
 * @brief 处理接收到的字节流
 * @param sdk 云台SDK实例指针
 * @param ch  接收到的字节
 */
void gimbal_sdk_input_byte(gimbal_sdk_t* sdk, uint8_t ch) {
  if (Unpack_Function(&sdk->ctx, ch)) {
    // 帧完成，解析
    process_packet(sdk, &sdk->ctx.rxPacket);
  }
}

/**
 * @brief 接收数据接口（面向当前激活的 SDK 实例）
 * @note 用户只需在串口中断或接收循环中调用此函数，无需关心 sdk 指针
 */
void std_sdk_input_byte(uint8_t ch) {
  // 使用全局静态指针 g_active_sdk
  if (g_active_sdk != NULL) {
    // 调用原有的解析逻辑
    if (Unpack_Function(&g_active_sdk->ctx, ch)) {
      // 帧完成，解析数据包
      process_packet(g_active_sdk, &g_active_sdk->ctx.rxPacket);
    }
  }
}

/**
 * @brief 配置异步指令的重发参数
 * @param sdk 实例指针
 * @param timeout_ms 超时时间 (建议 100-500ms)
 * @param retries 重试次数 (建议 1-5 次)
 */
void gimbal_sdk_set_retry_config(gimbal_sdk_t* sdk, uint32_t timeout_ms, uint8_t retries) {
  if (sdk) {
    sdk->default_timeout_ms = timeout_ms;
    sdk->default_retries = retries;
  }
}

/**
 * @brief 异步任务后台处理（重发管理）
 * @param current_time_ms 当前系统运行时间戳（毫秒）
 */
void gimbal_sdk_process_ms(uint32_t current_time_ms) {
  // 如果没有激活的 SDK 实例，直接返回
  if (g_active_sdk == NULL) return;

  for (int i = 0; i < CMD_WORK_QUEUE_SIZE; i++) {
    cmd_work_t* work = &g_active_sdk->cmd_queue.works[i];

    if (!work->used) continue;

    // 第一次执行时，记录起始时间
    if (work->time_started_ms == 0) {
      work->time_started_ms = current_time_ms;
      continue;
    }

    // 判断是否超时
    if (current_time_ms - work->time_started_ms >= work->timeout_ms) {
      if (work->retries_left > 0) {
        // 执行重发：直接使用当前激活实例的发送函数
        if (g_active_sdk->write_func) {
          g_active_sdk->write_func(work->packet.data, work->packet.length, g_active_sdk->write_user_ptr);
        }

        work->retries_left--;
        work->time_started_ms = current_time_ms;  // 重置计时器
      } else {
        // 重试次数用尽（retries_left == 0）
        if (work->cb) {
          // 调用回调函数，并将结果设置为 Timeout
          // 这样用户在回调里判断 if(result == Timeout) 就能处理重连或报警逻辑
          work->cb(work->msg_id, Timeout, work->user_data);
        }
        // 任务彻底结束，释放槽位，防止阻塞后续指令
        cmd_work_free(work);
      }
    }
  }
}
