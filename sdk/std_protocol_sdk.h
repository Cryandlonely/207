#ifndef STD_PROTOCOL_SDK_H
#define STD_PROTOCOL_SDK_H

#include <stddef.h>
#include <stdint.h>
#include "std_common.h"  // 底层

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 命令事务 ================= */

typedef struct {
  uint8_t used;    // 是否占用
  uint8_t msg_id;  // 命令ID（用于 ACK 匹配）

  // --- 发送相关
  std_packet_t packet;  // 要发送的完整数据包

  uint8_t already_sent;  // 是否已发送过（预留,以后扩展）

  // --- 回调 ---
  ResultCallback cb;  // 结果回调
  void* user_data;    // 用户数据

  // --- 时间 & 重试
  uint32_t time_started_ms;  // 发送时间（预留,以后扩展）
  uint32_t timeout_ms;       // 超时时间（预留,以后扩展）
  uint8_t retries_left;      // 剩余重试次数（预留,以后扩展）

} cmd_work_t;

// 声明 1ms 处理函数
void gimbal_sdk_process_ms(uint32_t current_time_ms);

/* ================= 命令事务队列 ================= */

#define CMD_WORK_QUEUE_SIZE 10
typedef struct {
  cmd_work_t works[CMD_WORK_QUEUE_SIZE];
} cmd_work_queue_t;

/* ================= 需要字节对齐 结构体定义 开始 ================= */
/* ================= 处理编译器不同的字节对齐 ================= */

#if defined(_MSC_VER)
#define PACKED
#pragma pack(push, 1)
#elif defined(__GNUC__) || defined(__clang__)
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif

typedef struct {
  uint16_t command; /*<  Command ID (of command to send).*/
  float param1;     /*<  Parameter 1 (for the specific command).*/
  float param2;     /*<  Parameter 2 (for the specific command).*/
  float param3;     /*<  Parameter 3 (for the specific command).*/
  float param4;     /*<  Parameter 4 (for the specific command).*/
  float param5;     /*<  Parameter 5 (for the specific command).*/
  float param6;     /*<  Parameter 6 (for the specific command).*/
  float param7;     /*<  Parameter 7 (for the specific command).*/
} PACKED std_cmd_long_t;

typedef struct {
  uint8_t servo_id;     //舵机ID
  int8_t center_pos;    //中位(启动位置),范围-90,90
  uint8_t left_limit;   //左限位 1~100 ，默认50
  uint8_t right_limit;  //右限位 1~100，默认50
  uint8_t invert;       //左右是否翻转,0不翻转，1翻转
} PACKED std_trigger_param_t;

typedef struct {
  uint8_t SetCh_Shot;
  uint8_t SetCh_Video;
  uint8_t SetCh_Magnificant;
  uint8_t SetCh_Focus;
  uint8_t SetCh_DayNight;
  uint8_t SetCh_PIP;
  uint8_t SetCh_Servo4;
  uint8_t SetCh_Color_SW;
  uint8_t SetCh_Laser_trig;
  uint8_t SetCh_Dis_trig;
  uint8_t SetCh_Pitch;
  uint8_t SetCh_Yaw;
  uint8_t SetCh_Servo3;
  uint8_t SetCh_Servo5;

  uint8_t S1_PARA_L_LIM;
  uint8_t S1_PARA_R_LIM;
  uint8_t S1_PARA_REV;
  uint8_t S1_PARA_RANK;

  uint8_t S2_PARA_L_LIM;
  uint8_t S2_PARA_R_LIM;
  uint8_t S2_PARA_REV;
  uint8_t S2_PARA_RANK;

  uint8_t S3_PARA_L_LIM;
  uint8_t S3_PARA_R_LIM;
  uint8_t S3_PARA_REV;
  uint8_t S3_PARA_RANK;

  uint8_t S4_PARA_L_LIM;
  uint8_t S4_PARA_R_LIM;
  uint8_t S4_PARA_REV;
  uint8_t S4_PARA_RANK;

  uint8_t S5_PARA_L_LIM;
  uint8_t S5_PARA_R_LIM;
  uint8_t S5_PARA_REV;
  uint8_t S5_PARA_RANK;

  uint8_t S6_PARA_L_LIM;
  uint8_t S6_PARA_R_LIM;
  uint8_t S6_PARA_REV;
  uint8_t S6_PARA_RANK;

  uint8_t S7_PARA_L_LIM;
  uint8_t S7_PARA_R_LIM;
  uint8_t S7_PARA_REV;
  uint8_t S7_PARA_RANK;

  uint8_t S8_PARA_L_LIM;
  uint8_t S8_PARA_R_LIM;
  uint8_t S8_PARA_REV;
  uint8_t S8_PARA_RANK;

  uint8_t S9_PARA_L_LIM;
  uint8_t S9_PARA_R_LIM;
  uint8_t S9_PARA_REV;
  uint8_t S9_PARA_RANK;

  uint8_t S10_PARA_L_LIM;
  uint8_t S10_PARA_R_LIM;
  uint8_t S10_PARA_REV;
  uint8_t S10_PARA_RANK;
} PACKED std_scouter_paramterst_t;

typedef struct {
  uint16_t channels[16];
} PACKED std_scouter_sbus_t;

//红外温度
typedef struct {
  //最高温
  float max_temperature;
  //最低温
  float min_temperature;
  //平均温
  float avg_temperature;
} PACKED std_ir_temperature_t;

/* 原始反馈数据结构 - 用户不可见 */
typedef struct {
  uint8_t pod_status_t1; /* 吊舱状态1 */
  uint8_t pod_status_t2; /* 吊舱状态2 */

  int16_t roll_angle;  /* 横滚框架角 ,单位0.1度 */
  int16_t pitch_angle; /* 俯仰框架角 ,单位0.1度 */
  int16_t yaw_angle;   /* 航向框架角 ,单位0.1度 */

  int16_t off_target_x; /* 脱靶量坐标X ,单位像素 */
  int16_t off_target_y; /* 脱靶量坐标Y ,单位像素 */

  uint16_t laser_distance; /* 激光测距距离 ,单位dm */
  uint8_t zoom_ratio;      /* 倍率 ,单位0.1x */

  int32_t target_longitude; /* 目标经度 ,单位 deg * 1e7 */
  int32_t target_latitude;  /* 目标纬度 ,单位 deg * 1e7 */
  int16_t target_altitude;  /* 目标海拔高度,单位 dm */
  uint16_t target_distance; /* 目标水平距离，单位 dm */

} PACKED std_feedback_t;

/**
 * @brief 云台 IMU 用户数据结构（与姿态角同一机体轴系）
 *        姿态角相对地平面；加速度/角速度与姿态解算同轴系。
 */
typedef struct {
  float frame_angle_x; /**< 框架角-俯仰轴机械角 [°]  */
  float frame_angle_y; /**< 框架角-横滚轴机械角 [°]  */
  float frame_angle_z; /**< 框架角-航向轴机械角 [°]  */

  float pitch_deg; /**< 俯仰角 [°] */
  float roll_deg;  /**< 横滚角 [°] */
  float yaw_deg;   /**< 航向角 [°] */

  float accel[3]; /**< 加速度 [m/s²]，机体轴 X/Y/Z */
  float gyro[3];  /**< 角速度 [rad/s]，机体轴 X/Y/Z */

} PACKED std_attitude_t;

typedef struct {
  float x;               // x轴校准坐标
  float y;               // y轴校准坐标
  uint8_t state;         // 触发器状态
  uint8_t mode;          // 触发器模式
  uint16_t interval_ms;  // 触发时间间隔
} PACKED std_trigger_feedback_t;

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

/* ================= 需要字节对齐 结构体定义 结束 ================= */

typedef struct {
  uint8_t Frame_Header;            /* 帧头1 */
  uint8_t Function_Mark;           /* 帧头2 (帧类型标志：命令/ACK) */
  uint8_t Master_id;               /* 发送方ID */
  uint8_t Slave_id;                /* 接收方ID */
  uint8_t MSG_id;                  /* 消息ID */
  uint8_t Data_length;             /* 数据段长度 */
  uint8_t Data[TCA_MAX_DATA_SIZE]; /* 数据段内容 */
  uint8_t Sum_check;               /* 和校验 (逐字节累加) */
  uint8_t Additional_check;        /* 附加校验 (逐步累加和校验) */
} std_pk_t;

/* 每个状态对应接收一个字节的处理 */
typedef enum {
  STATE_WAIT_HEADER1 = 0, /* 等待帧头1 */
  STATE_WAIT_HEADER2,     /* 等待帧头2 */
  STATE_WAIT_MASTER_ID,   /* 等待发送方ID */
  STATE_WAIT_SLAVE_ID,    /* 等待接收方ID */
  STATE_WAIT_MSG_ID,      /* 等待消息ID */
  STATE_WAIT_LENGTH,      /* 等待数据长度 */
  STATE_WAIT_DATA,        /* 等待数据区 */
  STATE_WAIT_SUM,         /* 等待和校验 */
  STATE_WAIT_ADDITIONAL   /* 等待附加校验 */
} unpack_state_t;

/* 每个通道一个上下文，互不干扰 */
typedef struct {
  unpack_state_t state; /* 当前解析状态 */
  std_pk_t rxPacket;    /* 正在组装的包 */
  uint8_t dataIndex;    /* 当前写入数据区的下标 */
  uint8_t sumCheck;     /* 当前和校验值 */
  uint8_t addCheck;     /* 当前附加校验值 */
  uint8_t frameReady;   /* 帧完成标志：1=完成，0=未完成 */
} unpack_ctx_t;

/* 转换后的反馈数据结构 - 用户直接使用的最终数据 */
typedef struct {
  /* 状态信息 */
  uint8_t pod_status_1;
  uint8_t pod_status_2;

  /* 角度信息 (单位: 度) */
  float roll_angle;
  float pitch_angle;
  float yaw_angle;

  /* 脱靶量信息 (单位: 像素) */
  int16_t off_target_x;
  int16_t off_target_y;

  /* 测距信息 (单位: 米) */
  float laser_distance;

  /* 光学信息 */
  float zoom_ratio;

  /* 目标位置信息 */
  double target_longitude;
  double target_latitude;
  float target_altitude;
  float target_distance;

} std_feedback_converted_t;

/* 转换后的触发器反馈数据结构 */
typedef struct {
  float x;
  float y;
  uint8_t state;
  uint8_t mode;
  uint16_t interval_ms;
} std_trigger_feedback_converted_t;

// 修改回调函数定义，使用转换后的数据结构
typedef void (*feedback_callback_t)(const std_feedback_converted_t* feedback, void* user_data);
typedef void (*trigger_feedback_callback_t)(const std_trigger_feedback_converted_t* feedback, void* user_data);
typedef void (*trigger_param_callback_t)(const std_trigger_param_t* param, void* user_data);

typedef struct {
  trigger_param_callback_t cb;  // 读取完成回调
  void* user_data;              // 回调用户数据
} trigger_param_request_t;

typedef void (*scouter_param_callback_t)(const std_scouter_paramterst_t* feedback, void* user_data);

typedef void (*sbus_channel_callback_t)(const std_scouter_sbus_t* feedback, void* user_data);

typedef struct {
  scouter_param_callback_t cb;  // 读取完成回调
  void* user_data;              // 回调用户数据
} scouter_param_request_t;

typedef struct {
  sbus_channel_callback_t cb;  // 读取完成回调
  void* user_data;             // 回调用户数据
} sbus_channel_request_t;

//订阅红外温度
typedef void (*ir_temperature_callback_t)(const std_ir_temperature_t* feedback, void* user_data);

typedef struct {
  ir_temperature_callback_t cb;  // 读取完成回调
  void* user_data;               // 回调用户数据
} ir_temperature_request_t;

typedef void (*attitude_callback_t)(const std_attitude_t* feedback, void* user_data);

typedef struct {
  attitude_callback_t cb;  // 读取完成回调
  void* user_data;         // 回调用户数据
} attitude_request_t;

/**
 * @brief 用户定义的底层发送函数原型
 * @param data 需要发送的字节数组
 * @param len  数据长度
 * @param user_ptr 用户自定义指针（例如指向某个串口对象或网络句柄）
 * @return 实际发送的字节数，或者错误码
 */
typedef int (*sdk_write_func_t)(const uint8_t* data, uint16_t len, void* user_ptr);

typedef struct {
  unpack_ctx_t ctx;                                 // 解包上下文
  feedback_callback_t on_feedback;                  //标准信息反馈数据回调
  trigger_feedback_callback_t on_trigger_feedback;  // 触发器反馈数据回调
  void* feedback_user_data;                         // 反馈回调用户数据
  void* trigger_user_data;                          // 触发器回调用户数据
  cmd_work_queue_t cmd_queue;                       //命令事务队列

  trigger_param_request_t trigger_param_req;    // 触发器参数读取请求管理
  scouter_param_request_t scouter_param_req;    // 舵机参数读取请求管理
  sbus_channel_request_t sbus_channel_req;      // sbus通道数据请求管理
  ir_temperature_request_t ir_temperature_req;  // 红外温度数据请求管理
  attitude_request_t attitude_req;              // 姿态数据请求管理

  // 重发配置属性
  uint32_t default_timeout_ms;  // 默认超时时间
  uint8_t default_retries;      // 默认重试次数

  // 发送接口
  sdk_write_func_t write_func;  // 底层写函数回调
  void* write_user_ptr;         // 底层写函数用到的句柄（如 UART_Handle,TCP_Handle,CAN_Handle）

} gimbal_sdk_t;

// std_protocol_sdk.c 所有函数声明
int std_uint16_to_data(uint16_t val, uint8_t* data);
int std_int16_to_data(int16_t val, uint8_t* data);
int std_int32_to_data(int32_t val, uint8_t* data);
int std_float_to_data(float val, uint8_t* data);
size_t std_create_packet(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, uint8_t* output_buffer, size_t buffer_size);
size_t tga_create_packet(uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, uint8_t* output_buffer, size_t buffer_size);
uint16_t std_cmd_async_create(gimbal_sdk_t* sdk, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, ResultCallback cb, void* user_data, uint8_t* out_buf,
                              uint16_t out_buf_size);

uint16_t tga_cmd_async_create(gimbal_sdk_t* sdk, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, ResultCallback cb, void* user_data, uint8_t* out_buf,
                              uint16_t out_buf_size);

std_packet_t std_control_velocity(int16_t pitch_speed, int16_t yaw_speed);

std_packet_t std_control_stop_movement(void);

std_packet_t std_control_direction_control(std_direction_control_t direction, float step);

std_packet_t std_control_direction_control_async(std_direction_control_t direction, float step, ResultCallback cb, void* user_data);

std_packet_t std_control_mode_switch(std_mode_t mode);

std_packet_t std_control_mode_switch_async(const std_mode_t mode, ResultCallback cb, void* user_data);

std_packet_t std_control_point_tracking(float x, float y);

std_packet_t std_control_point_tracking_async(float x, float y, ResultCallback cb, void* user_data);

std_packet_t std_control_tracking_off(void);

std_packet_t std_control_tracking_off_async(ResultCallback cb, void* user_data);

std_packet_t std_control_onekey_down(void);

std_packet_t std_control_onekey_down_async(ResultCallback cb, void* user_data);

std_packet_t std_control_onekey_center(void);
std_packet_t std_control_onekey_center_async(ResultCallback cb, void* user_data);

std_packet_t std_control_specify_yaw(float yaw_angle);
std_packet_t std_control_specify_yaw_async(float yaw_angle, ResultCallback cb, void* user_data);

std_packet_t std_control_specify_pitch(float pitch_angle);
std_packet_t std_control_specify_pitch_async(float pitch_angle, ResultCallback cb, void* user_data);

std_packet_t std_control_specify_angle(float pitch_angle, float yaw_angle);
std_packet_t std_control_specify_angle_async(float pitch_angle, float yaw_angle, ResultCallback cb, void* user_data);

std_packet_t std_control_stabilization(std_stabilization_t enable);
std_packet_t std_control_stabilization_async(std_stabilization_t enable, ResultCallback cb, void* user_data);

std_packet_t std_control_zoom_in(uint8_t speed);
std_packet_t std_control_zoom_in_async(uint8_t speed, ResultCallback cb, void* user_data);

std_packet_t std_control_zoom_out(uint8_t speed);
std_packet_t std_control_zoom_out_async(uint8_t speed, ResultCallback cb, void* user_data);

std_packet_t std_control_zoom_stop(void);
std_packet_t std_control_zoom_stop_async(ResultCallback cb, void* user_data);

std_packet_t std_control_specify_zoom(float zoom_level);
std_packet_t std_control_specify_zoom_async(float zoom_level, ResultCallback cb, void* user_data);

std_packet_t std_control_focus_in(uint8_t speed);
std_packet_t std_control_focus_in_async(uint8_t speed, ResultCallback cb, void* user_data);

std_packet_t std_control_focus_out(uint8_t speed);
std_packet_t std_control_focus_out_async(uint8_t speed, ResultCallback cb, void* user_data);

std_packet_t std_control_focus_stop(void);
std_packet_t std_control_focus_stop_async(ResultCallback cb, void* user_data);

std_packet_t std_control_take_photo(void);
std_packet_t std_control_take_photo_async(ResultCallback cb, void* user_data);

std_packet_t std_control_start_recording(void);
std_packet_t std_control_start_recording_async(ResultCallback cb, void* user_data);

std_packet_t std_control_stop_recording(void);
std_packet_t std_control_stop_recording_async(ResultCallback cb, void* user_data);

std_packet_t std_control_pip_switch(std_pip_t mode);
std_packet_t std_control_pip_switch_async(std_pip_t mode, ResultCallback cb, void* user_data);

std_packet_t std_control_pseudo_color(std_pseudo_color_t mode);
std_packet_t std_control_pseudo_color_async(std_pseudo_color_t mode, ResultCallback cb, void* user_data);

std_packet_t std_control_osd_on(void);
std_packet_t std_control_osd_on_async(ResultCallback cb, void* user_data);

std_packet_t std_control_osd_off(void);
std_packet_t std_control_osd_off_async(ResultCallback cb, void* user_data);

std_packet_t std_control_digital_zoom(std_dzoom_t enable);
std_packet_t std_control_digital_zoom_async(std_dzoom_t enable, ResultCallback cb, void* user_data);

std_packet_t std_control_camera_stabilization(void);
std_packet_t std_control_camera_stabilization_async(ResultCallback cb, void* user_data);

std_packet_t std_control_camera_stabilization_off(void);
std_packet_t std_control_camera_stabilization_off_async(ResultCallback cb, void* user_data);

std_packet_t std_control_laser_control(std_laser_control_t command);
std_packet_t std_control_laser_control_async(std_laser_control_t command, ResultCallback cb, void* user_data);

std_packet_t std_control_laser_ranging(std_laser_distance_control_t command);
std_packet_t std_control_laser_ranging_async(std_laser_distance_control_t command, ResultCallback cb, void* user_data);

std_packet_t std_control_object_recognition(std_ai_detection_t command);
std_packet_t std_control_object_recognition_async(std_ai_detection_t command, ResultCallback cb, void* user_data);

std_packet_t std_control_gps_navigation(int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_height, uint16_t heading);
std_packet_t std_control_gps_navigation_async(int32_t longitude, int32_t latitude, int32_t altitude, int32_t relative_height, uint16_t heading, ResultCallback cb,
                                              void* user_data);

std_packet_t std_control_camera_param_save(void);
std_packet_t std_control_camera_param_save_async(ResultCallback cb, void* user_data);

std_packet_t std_control_fill_light_enable(std_fill_light_control_t command);
std_packet_t std_control_fill_light_enable_async(std_fill_light_control_t command, ResultCallback cb, void* user_data);

std_packet_t std_control_fill_light_brightness(uint16_t brightness);
std_packet_t std_control_fill_light_brightness_async(uint16_t brightness, ResultCallback cb, void* user_data);

std_packet_t std_control_fill_light_size(uint16_t size);
std_packet_t std_control_fill_light_size_async(uint16_t size, ResultCallback cb, void* user_data);

std_packet_t std_trigger_control(uint8_t num, uint8_t state);
std_packet_t std_trigger_control_async(uint8_t num, uint8_t state, ResultCallback cb, void* user_data);

std_packet_t std_control_sight_calibration(float x, float y);
std_packet_t std_control_sight_calibration_async(float x, float y, ResultCallback cb, void* user_data);

std_packet_t std_trigger_angle_control(uint8_t num, float angle);
std_packet_t std_trigger_angle_control_async(uint8_t num, float angle, ResultCallback cb, void* user_data);

std_packet_t std_trigger_mode_switch(uint8_t num, uint8_t mode);
std_packet_t std_trigger_mode_switch_async(uint8_t num, uint8_t mode, ResultCallback cb, void* user_data);

std_packet_t std_trigger_interval_control(uint8_t num, uint16_t interval_ms);
std_packet_t std_trigger_interval_control_async(uint8_t num, uint16_t interval_ms, ResultCallback cb, void* user_data);

std_packet_t std_control_time_sync(uint64_t utc);
std_packet_t std_control_time_sync_async(uint64_t utc, ResultCallback cb, void* user_data);

std_packet_t std_control_fogging(std_fogging_control_t mode, std_fogging_strength_t strength);
std_packet_t std_control_fogging_async(std_fogging_control_t mode, std_fogging_strength_t strength, ResultCallback cb, void* user_data);

std_packet_t std_control_box_tracking(float x_min, float y_min, float x_max, float y_max);
std_packet_t std_control_box_tracking_async(float x_min, float y_min, float x_max, float y_max, ResultCallback cb, void* user_data);

std_packet_t std_control_icr_mode(std_icr_mode_t mode);
std_packet_t std_control_icr_mode_async(gimbal_sdk_t* sdk, std_icr_mode_t mode, ResultCallback cb, void* user_data);

std_packet_t std_control_ir_control(std_ir_control_t command);
std_packet_t std_control_ir_control_async(gimbal_sdk_t* sdk, std_ir_control_t command, ResultCallback cb, void* user_data);

std_packet_t std_ack(uint8_t msg_id, StdResult result);

std_packet_t std_trigger_parameter_setting(uint8_t num, int8_t center_pos, uint8_t left_limit, uint8_t right_limit, uint8_t invert);
std_packet_t std_trigger_parameter_setting_async(gimbal_sdk_t* sdk, uint8_t num, int8_t center_pos, uint8_t left_limit, uint8_t right_limit, uint8_t invert,
                                                 ResultCallback cb, void* user_data);

std_packet_t std_subscribe_trigger_parameter_async(gimbal_sdk_t* sdk, uint8_t num, trigger_param_callback_t cb, void* user_data);
void unsubscribe_trigger_parameter_async(gimbal_sdk_t* sdk);

std_packet_t std_trigger_parameter_save();
std_packet_t std_trigger_parameter_save_async(gimbal_sdk_t* sdk, ResultCallback cb, void* user_data);

std_packet_t std_subscribe_tga_scouter_data(gimbal_sdk_t* sdk, scouter_param_callback_t cb, void* user_data);
void std_unsubscribe_tga_scouter_data(gimbal_sdk_t* sdk);

std_packet_t std_scouter_param_set(const std_scouter_paramterst_t* param);
std_packet_t std_scouter_param_set_async(gimbal_sdk_t* sdk, const std_scouter_paramterst_t* param, ResultCallback cb, void* user_data);

std_packet_t std_scouter_param_save();
std_packet_t std_scouter_param_save_async(gimbal_sdk_t* sdk, ResultCallback cb, void* user_data);

std_packet_t std_subscribe_tga_sbus_channel_data(gimbal_sdk_t* sdk, sbus_channel_callback_t cb, void* user_data);
void std_unsubscribe_tga_sbus_channel_data(gimbal_sdk_t* sdk);

std_packet_t std_subscribe_ir_temperature_data(uint8_t hz, ir_temperature_callback_t cb, void* user_data);
std_packet_t std_unsubscribe_ir_temperature_data(void);

std_packet_t std_subscribe_attitude_data(uint8_t hz, attitude_callback_t cb, void* user_data);
std_packet_t std_unsubscribe_attitude_data(void);

std_packet_t std_control_logging_async(std_log_control_t enable, ResultCallback cb, void* user_data);
std_packet_t std_control_logging(std_log_control_t enable);

std_packet_t std_control_smoke_control(std_smoke_control_t enable);
std_packet_t std_control_smoke_control_async(std_smoke_control_t enable, ResultCallback cb, void* user_data);

void process_packet(gimbal_sdk_t* sdk, std_pk_t* ctx);
uint8_t Unpack_Function(unpack_ctx_t* ctx, uint8_t ch);

void gimbal_sdk_init(gimbal_sdk_t* sdk, sdk_write_func_t write_func, void* write_user_ptr);

void gimbal_sdk_input_byte(gimbal_sdk_t* sdk, uint8_t ch);
void std_sdk_input_byte(uint8_t ch);

void convert_trigger_feedback_data(const std_trigger_feedback_t* raw, std_trigger_feedback_converted_t* converted);

void convert_feedback_data(const std_feedback_t* raw, std_feedback_converted_t* converted);

// 修改注册回调函数声明，添加用户数据参数
void gimbal_sdk_register_feedback_callback(gimbal_sdk_t* sdk, feedback_callback_t cb, void* user_data);
void gimbal_sdk_register_trigger_feedback_callback(gimbal_sdk_t* sdk, trigger_feedback_callback_t cb, void* user_data);

// 声明设置激活实例的接口
void std_sdk_set_active_instance(gimbal_sdk_t* sdk);

// 设置发送接口的函数声明
void gimbal_sdk_set_write_func(gimbal_sdk_t* sdk, sdk_write_func_t func, void* user_ptr);

void gimbal_sdk_set_retry_config(gimbal_sdk_t* sdk, uint32_t timeout_ms, uint8_t retries);

void gimbal_sdk_process_ms(uint32_t current_time_ms);

size_t std_create_command_long_packet(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7, uint8_t* out_buf, size_t buf_size);

std_packet_t std_control_unsubscribe_stream(std_msg_stream_t stream_id);
std_packet_t std_control_subscribe_stream(std_msg_stream_t stream_id, float hz);
std_packet_t std_control_request_message_once(std_msg_stream_t msg_id);

void internal_auto_send(std_packet_t* pkt);  //底层发送辅助函数声明

extern gimbal_sdk_t* g_active_sdk;

#ifdef __cplusplus
}
#endif

#endif  // STD_PROTOCOL_SDK_H
