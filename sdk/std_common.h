#ifndef STD_COMMON_H
#define STD_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 协议常量
#define STD_HEADER_1 0x33        // 协议头1
#define STD_HEADER_2 0x33        // 协议头2
#define STD_SENDER_ID 0x02       // 发送者ID
#define STD_RECEIVER_ID 0x03     // 接收者ID
#define STD_MAX_PACKET_SIZE 255  // 最大包长度
#define TCA_MAX_DATA_SIZE 254

#define TGA_SENDER_ID 0x00  // TGA发送者ID

// 协议包结构
typedef struct {
  uint8_t data[STD_MAX_PACKET_SIZE];
  uint16_t length;
} std_packet_t;

// 消息ID定义 (根据Usr_Protocol_Define.h)
typedef enum {

  MSG_ID_VEL_CTRL = 0x04,         // 云台速度直接控制
  MSG_ID_STOP_MOVE = 0x05,        // 云台运动停止
  MSG_ID_MODE_SWITCH = 0x0C,      // 云台模式切换
  MSG_ID_POINT_TRACKING = 0x26,   // 指点跟踪
  MSG_ID_TRACKING_OFF = 0x29,     // 取消跟踪
  MSG_ID_ONEKEY_DOWN = 0x0F,      // 一键向下
  MSG_ID_ONEKEY_CENTER = 0x10,    // 一键回中
  MSG_ID_SPECIFY_YAW = 0x08,      // 指定航向角度
  MSG_ID_SPECIFY_PITCH = 0x07,    // 指定俯仰角度
  MSG_ID_SPECIFY_ANGLE = 0x57,    // 指定位置（角度）
  MSG_ID_STABILIZATION = 0x62,    // 云台增稳
  MSG_ID_ZOOM_IN = 0x11,          // 变倍+
  MSG_ID_ZOOM_OUT = 0x12,         // 变倍-
  MSG_ID_ZOOM_STOP = 0x13,        // 停止变倍
  MSG_ID_SPECIFY_ZOOM = 0x5E,     // 指定倍率
  MSG_ID_FOCUS_IN = 0x15,         // 对焦+
  MSG_ID_FOCUS_OUT = 0x16,        // 对焦-
  MSG_ID_FOCUS_STOP = 0x17,       // 停止对焦
  MSG_ID_TAKE_PHOTO = 0x1B,       // 拍照
  MSG_ID_START_RECORDING = 0x1C,  // 开始录像
  MSG_ID_STOP_RECORDING = 0x1D,   // 停止录像

  MSG_ID_ICR_VISIBLE_LIGHT = 0x1E,  ///< ICR可见光模式
  MSG_ID_ICR_NIGHT_LIGHT = 0x1F,    ///< ICR夜间模式
  MSG_ID_ICR_AUTO = 0x73,           ///< ICR自动模式

  MSG_ID_PIP_SWITCH = 0x20,                // 画中画切换
  MSG_ID_PSEUDO_COLOR = 0x21,              // 伪彩切换
  MSG_ID_CAMERA_STABILIZATION_ON = 0x22,   // 相机电子增稳开
  MSG_ID_CAMERA_STABILIZATION_OFF = 0x23,  // 相机电子增稳关
  MSG_ID_OSD_ON = 0x24,                    // OSD开
  MSG_ID_OSD_OFF = 0x25,                   // OSD关
  MSG_ID_DZOOM = 0x5A,                     // 电子变倍
  MSG_ID_IR_CONTROL = 0x70,                // 红外相机控制
  MSG_ID_LASER = 0x31,                     // 激光控制
  MSG_ID_RANGE_SINGLE = 0x33,              // 单次测距
  MSG_ID_RANGE_CONTINUOUS = 0x34,          // 连续测距
  MSG_ID_RANGE_STOP = 0x35,                // 停止测距
  MSG_ID_AI_DETECTION_OFF = 0x36,          // 关闭人车识别
  MSG_ID_AI_DETECTION_ON = 0x37,           // 开启人车识别
  MSG_ID_GPS_SET = 0x58,                   // GPS设置
  MSG_ID_CAMERA_PARAM_SAVE = 0x59,         // 相机参数保存
  MSG_ID_LIGHT_CONTROL = 0x65,             // 补光灯控制
  MSG_ID_FEEDBACK = 0x63,                  // 反馈数据报文
  MSG_ID_FOGGING_CONTROL = 0x72,           //相机透雾
  MSG_ID_SIGHT_CALIBRATION = 0x74,         // 标准心校准
  MSG_ID_TRIGGER_CONTROL = 0x75,           // 触发器控制

  MSG_ID_TRIGGER_ANGLE_CONTROL = 0x77,    // 触发器角度控制
  MSG_ID_TRIGGER_MODE_SWITCH = 0x78,      // 触发器模式切换
  MSG_ID_TRIGGER_INTERVAL = 0x79,         // 触发器时间间隔
  MSG_ID_TURNTABLE_REPORTTED = 0x7A,      // 转台状态上报
  MSG_ID_BOX_TRACKING = 0x7C,             //框选跟踪
  MSG_ID_TRIGGER_PARAMER_SETTING = 0x7D,  //触发器参设置
  MSG_ID_TRIGGER_PARAMER_SAVE = 0x7E,     //触发器参保存
  MSG_ID_TRIGGER_PARAMER_READ = 0x7F,     //触发器参读取
  MSG_ID_UTC_TIME_SYNC = 0x80,            // utc时间同步

  MSG_ID_DIRECTION_CONTROL = 0x84,  // 方向控制

  MSG_COMMAND_LONG = 0xFD,        // 用户命令指令，设计参考Mavlink2命令，共 CMD_ID + 7个float字段
  MSG_ID_MESSAGES_STREAM = 0xFE,  // 消息数据流 ,格式：msg_id（2字节）  + data(最大253字节)

  MSG_ID_ACK = 0xFF,  // ACK应答 msg_id + result

} std_msg_id_t;

// cmd long
typedef enum {

  //设置特定的消息流频率
  STD_CMD_ID_SET_MESSAGE_STREAM_INTERVAL =
      1, /*param1: msg_stream ID param2: The interval between two messages. -1: disable. 0: request default rate (which may be zero). */

  //一次新版本的 STD_CMD_ID_SET_MESSAGE_STREAM_INTERVAL，请求一次实例
  STD_CMD_REQUEST_MESSAGE = 2, /*param1: msg_stream ID*/

  //日志功能
  STD_CMD_LOGGING_START = 3, /*param1: 1: enable 0: disable*/

  //烟雾喷火器开关
  STD_CMD_SMOKE_CONTROL = 4, /*param1: 1: enable 0: disable*/

} std_cmd_id_t;

//消息数据流
typedef enum {
  STD_MSG_STREAM_ATTITUDE = 0x01,                //云台姿态
  STD_MSG_STREAM_GPS = 0x02,                     // GPS信息
  STD_MSG_STREAM_IR_TEMPERATURE = 0x03,          //红外信息
  STD_MSG_STREAM_CAMERA_INFO = 0x04,             //相机信息
  STD_MSG_STREAM_TRACKING_INFO = 0x05,           //跟踪信息
  STD_MSG_STREAM_ROBOT_DOG_CONTROL = 0x10,       // robot dog control msg
  STD_MSG_STREAM_ROBOT_DOG_SBUS_CHANNEL = 0x11,  // robot dog sbus channel msg
  STD_MSG_STREAM_ROBOT_DOG_PARAMETERS = 0x12,    // robot dog parameters msg
  STD_MSG_STREAM_END,                            // end of stream
} std_msg_stream_t;

// 单一方向控制枚举
typedef enum {
  STD_DIRECTION_STOP = 0x00,  /* 停止*/
  STD_DIRECTION_UP = 0x01,    /* 向上*/
  STD_DIRECTION_DOWN = 0x02,  /* 向下*/
  STD_DIRECTION_LEFT = 0x03,  /* 向左*/
  STD_DIRECTION_RIGHT = 0x04, /* 向右*/
  STD_DIRECTION_END
} std_direction_control_t;

typedef enum {
  STD_LOG_CONTROL_START = 0x01, /*开启*/
  STD_LOG_CONTROL_STOP = 0x00,  /*关闭*/
} std_log_control_t;

// 烟雾/喷火器开关
typedef enum {
  STD_SMOKE_CONTROL_ON = 0x01,  /*开启*/
  STD_SMOKE_CONTROL_OFF = 0x00, /*关闭*/
} std_smoke_control_t;

// TGA专用协议
typedef enum {

  MSG_TGA_ID_SCOUTER_PARA_READ = 0x4F,   /* 参数读取 */
  MSG_TGA_ID_SCOUTER_PARA_WRITE = 0x50,  /* 参数写入 */
  MSG_TGA_ID_SCOUTER_PARA_SAVE = 0x51,   /* 参数保存 */
  MSG_TGA_ID_SCOUTER_PARA_UPLOAD = 0x52, /* 参数上传 */
  MSG_TGA_ID_SCOUTER_SBUS_UPLOAD = 0x53, /* SBUS上传 */

  MSG_TGA_REPLY_PALINDROME = 0x38, /*回文*/
  MSG_TGA_ID_ACK = 0xFF            /* ACK应答 */

} std_tga_msg_id_t;

// 具体应答类型
typedef enum {
  MSG_TGA_ACK_SAVE = 0x02,
  MSG_TGA_ACK_SET_CONFIG = 0x09,
} tga_msg_reply;

#define TRUE 1
#define FALSE 0

/* ================= ACK 结果 ================= */
typedef enum {
  Success = 0,              //成功
  NoSystem = 1,             // 未连接系统
  ConnectionError = 2,      // 连接错误
  Busy = 3,                 // 繁忙
  Denied = 4,               // 拒绝
  Unsupported = 5,          // 不支持
  Timeout = 6,              // 超时
  InProgress = 7,           // 进行中
  TemporarilyRejected = 8,  // 临时拒绝
  Failed = 9,               // 失败
  Cancelled = 10,           // 取消
  UnknownError = 11,        // 未知错误
  ParameterError = 12,      //参数错误
} StdResult;

// 模式枚举
typedef enum {
  MODE_LOCK = 0x00,    // 锁定模式
  MODE_FOLLOW = 0x01,  // 跟随模式
  MODE_END
} std_mode_t;

// ICR模式
typedef enum {
  ICR_AUTO = 0x00,           ///< ICR自动模式
  ICR_VISIBLE_LIGHT = 0x01,  ///< ICR可见光模式
  ICR_NIGHT_LIGHT = 0x02,    ///< ICR夜间模式
  ICR_END
} std_icr_mode_t;

//云台增稳
typedef enum {
  STABILIZATION_OFF = 0x00,  // 增稳关
  STABILIZATION_ON = 0x01,   // 增稳开
  STABILIZATION_END
} std_stabilization_t;

//画中画切换
typedef enum {
  PIP_VI_ONLY = 0x00,     // 可见光
  PIP_IR_ONLY = 0x01,     // 热成像
  PIP_VI_IN_IR = 0x02,    // 可见光嵌入热成像
  PIP_IR_IN_VI = 0x03,    // 热成像嵌入可见光
  PIP_WIDE_ONLY = 0x04,   // 广角
  PIP_VI_IN_WIDE = 0x05,  // 可见光嵌入广角
  PIP_WIDE_IN_VI = 0x06,  // 广角嵌入可见光
  PIP_END,
} std_pip_t;

//伪彩切换
typedef enum {
  PSEUDO_COLOR_1 = 0x00,
  PSEUDO_COLOR_3 = 0x02,
  PSEUDO_COLOR_2 = 0x01,
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
  PSEUDO_COLOR_END
} std_pseudo_color_t;

// 激光控制枚举
typedef enum {
  LASER_ON = 0x00,   // 激光开
  LASER_OFF = 0x01,  // 激光关
  LASER_END
} std_laser_control_t;

//电子变倍设置
typedef enum {
  DZOOM_OFF = 0x00,  // 电子变倍关
  DZOOM_ON = 0x01,   // 电子变倍开
  DZOOM_END
} std_dzoom_t;

//红外控制
typedef enum {
  IR_ON = 0x01,        // 测温开
  IR_OFF = 0x02,       // 测温关
  IR_SWITCH_1 = 0x03,  //切换温度测量档位1
  IR_SWITCH_2 = 0x04,  //切换温度测量档位2
  IR_END,
} std_ir_control_t;

//激光测距控制枚举
typedef enum {
  LASER_DISTANCE_SINGLE = 0x00,      // 单次测距
  LASER_DISTANCE_CONTINUOUS = 0x01,  // 连续测距
  LASER_DISTANCE_STOP = 0x02,        // 停止测距
  LASER_DISTANCE_END
} std_laser_distance_control_t;

//人车识别控制枚举
typedef enum {
  AI_DETECTION_OFF = 0x00,  // 关闭人车识别
  AI_DETECTION_ON = 0x01,   // 开启人车识别
  AI_DETECTION_END
} std_ai_detection_t;

//补光灯控制枚举
typedef enum {
  FILL_LIGHT_OFF = 0x00,  //关
  FILL_LIGHT_ON = 0x01,   //开
  FILL_LIGHT_END
} std_fill_light_control_t;

//补光灯控制枚举
typedef enum {
  FILL_LIGHT_CONTROL_MODE = 0x00,        //设置补光灯状态
  FILL_LIGHT_CONTROL_BRIGHTNESS = 0x02,  //设置补光灯亮度
  FILL_LIGHT_CONTROL_SIZE = 0x04,        //设置补光灯大小
  FILL_LIGHT_CONTROL_END
} std_fill_light_control_mode_t;

//透雾模式
typedef enum {
  //关闭 自动 手动
  FOGGING_MODE_OFF = 0x00,     // 透雾关
  FOGGING_MODE_AUTO = 0x01,    // 自动透雾
  FOGGING_MODE_MANUAL = 0x02,  // 手动透雾
  FOGGING_MODE_END
} std_fogging_control_t;

//透雾强度
typedef enum {
  FOGGING_STRENGTH_LOW = 0x00,     // 低强度透雾
  FOGGING_STRENGTH_MEDIUM = 0x01,  // 中强度透雾
  FOGGING_STRENGTH_HIGH = 0x02,    // 高强度透雾
  FOGGING_STRENGTH_END
} std_fogging_strength_t;

typedef enum {
  TRIGGER_OFF = 0x00,  // 触发器关
  TRIGGER_ON = 0x01,   // 触发器开
} std_trigger_control_t;

//触发器模式
typedef enum {
  TRIGGER_MODE_AUTO = 0x00,        // 全自动
  TRIGGER_MODE_CONTINUOUS = 0x01,  // 连发
  TRIGGER_MODE_END
} std_trigger_mode_t;

typedef void (*ResultCallback)(uint8_t msg_id, StdResult result,
                               void* user_data);  // ACK回调

#ifdef __cplusplus
}
#endif

#endif  // STD_COMMON_H
