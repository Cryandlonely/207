#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// SDK 是 C 代码，需用 extern "C" 链接
extern "C" {
#include "std_protocol_sdk.h"
}

/**
 * @brief 云台 TCP 控制器
 *
 * 封装了云台 SDK 的 TCP 通信、接收线程和所有控制指令，
 * 可直接在 main.cpp 中使用。
 *
 * 用法示例:
 * @code
 *   GimbalController gimbal("192.168.100.12", 2000);
 *
 *   gimbal.setFeedbackCallback([](const std_feedback_converted_t& fb) {
 *       printf("俯仰: %.2f°  航向: %.2f°\n", fb.pitch_angle, fb.yaw_angle);
 *   });
 *
 *   gimbal.start();                   // 连接并启动接收
 *   gimbal.specifyAngle(-30.0f, 0);   // 俯仰-30°, 航向0°
 *   gimbal.onekeyCenter();            // 回中
 *   gimbal.stop();                    // 断开
 * @endcode
 */
class GimbalController {
 public:
  using FeedbackCallback = std::function<void(const std_feedback_converted_t&)>;
  using TriggerFeedbackCallback = std::function<void(const std_trigger_feedback_converted_t&)>;

  /**
   * @brief 构造函数
   * @param ip   云台 IP 地址 (默认 "192.168.100.12")
   * @param port 云台端口     (默认 2000)
   */
  GimbalController(const std::string& ip = "192.168.100.12", uint16_t port = 2000);
  ~GimbalController();

  // 禁止拷贝
  GimbalController(const GimbalController&) = delete;
  GimbalController& operator=(const GimbalController&) = delete;

  // ======================== 生命周期 ========================

  /**
   * @brief 连接云台并启动接收线程
   * @return true 连接成功
   */
  bool start();

  /**
   * @brief 断开连接并停止接收线程
   */
  void stop();

  /**
   * @brief 是否正在运行
   */
  bool isRunning() const { return running_.load(); }

  /**
   * @brief 是否已连接
   */
  bool isConnected() const { return sock_fd_ >= 0 && running_.load(); }

  // ======================== 角度控制 ========================

  /** @brief 指定俯仰角和航向角 (度) */
  bool specifyAngle(float pitch, float yaw);

  /** @brief 仅指定俯仰角 (度) */
  bool specifyPitch(float pitch);

  /** @brief 仅指定航向角 (度) */
  bool specifyYaw(float yaw);

  // ======================== 速度控制 ========================

  /** @brief 速度控制 (-100 ~ 100) */
  bool velocityControl(int16_t pitch_speed, int16_t yaw_speed);

  /** @brief 停止运动 */
  bool stopMovement();

  // ======================== 一键操作 ========================

  /** @brief 一键回中 */
  bool onekeyCenter();

  /** @brief 一键向下 */
  bool onekeyDown();

  // ======================== 模式切换 ========================

  /** @brief 切换锁定模式 */
  bool modeLock();

  /** @brief 切换跟随模式 */
  bool modeFollow();

  /** @brief 模式切换 (MODE_LOCK / MODE_FOLLOW) */
  bool modeSwitch(std_mode_t mode);

  // ======================== 相机控制 ========================

  /** @brief 拍照 */
  bool takePhoto();

  /** @brief 开始录像 */
  bool startRecording();

  /** @brief 停止录像 */
  bool stopRecording();

  // ======================== 变倍控制 ========================

  /** @brief 变倍+ (speed: 1~N) */
  bool zoomIn(uint8_t speed = 1);

  /** @brief 变倍- (speed: 1~N) */
  bool zoomOut(uint8_t speed = 1);

  /** @brief 停止变倍 */
  bool zoomStop();

  /** @brief 指定倍率 */
  bool specifyZoom(float level);

  // ======================== 对焦控制 ========================

  /** @brief 对焦+ */
  bool focusIn(uint8_t speed = 1);

  /** @brief 对焦- */
  bool focusOut(uint8_t speed = 1);

  /** @brief 停止对焦 */
  bool focusStop();

  // ======================== 跟踪控制 ========================

  /** @brief 指点跟踪 (归一化坐标 0~1) */
  bool pointTracking(float x, float y);

  /** @brief 框选跟踪 (归一化坐标 0~1) */
  bool boxTracking(float x_min, float y_min, float x_max, float y_max);

  /** @brief 取消跟踪 */
  bool trackingOff();

  // ======================== 激光控制 ========================

  /** @brief 激光开 */
  bool laserOn();

  /** @brief 激光关 */
  bool laserOff();

  /** @brief 单次测距 */
  bool laserRangeSingle();

  /** @brief 连续测距 */
  bool laserRangeContinuous();

  /** @brief 停止测距 */
  bool laserRangeStop();

  // ======================== 其他控制 ========================

  /** @brief 增稳开 */
  bool stabilizationOn();

  /** @brief 增稳关 */
  bool stabilizationOff();

  /** @brief 画中画切换 */
  bool pipSwitch(std_pip_t mode);

  /** @brief 伪彩切换 */
  bool pseudoColor(std_pseudo_color_t mode);

  /** @brief OSD 开 */
  bool osdOn();

  /** @brief OSD 关 */
  bool osdOff();

  // ======================== 回调注册 ========================

  /**
   * @brief 注册反馈数据回调
   *
   * 云台回传反馈数据时触发，包含俯仰/航向/横滚角度、测距、倍率等信息。
   * 回调在接收线程中执行。
   */
  void setFeedbackCallback(FeedbackCallback cb);

  /**
   * @brief 注册触发器反馈回调
   */
  void setTriggerFeedbackCallback(TriggerFeedbackCallback cb);

  // ======================== 状态查询 ========================

  /**
   * @brief 获取最新反馈数据 (线程安全)
   */
  std_feedback_converted_t getLatestFeedback() const;

  /**
   * @brief 获取底层 SDK 实例引用 (高级用法)
   */
  gimbal_sdk_t& getSdk() { return sdk_; }

 private:
  /// TCP 连接
  bool tcpConnect();

  /// 发送回调 (供 SDK 内部调用)
  static int tcpWriteFunc(const uint8_t* data, uint16_t len, void* user_ptr);

  /// SDK 反馈回调桥接
  static void feedbackBridge(const std_feedback_converted_t* fb, void* user_data);

  /// SDK 触发器反馈回调桥接
  static void triggerFeedbackBridge(const std_trigger_feedback_converted_t* fb, void* user_data);

  /// 接收线程
  void recvLoop();

  /// 发送 packet 的辅助函数
  bool sendPacket(const std_packet_t& pkt);

  // 配置
  std::string ip_;
  uint16_t port_;

  // Socket
  int sock_fd_ = -1;

  // SDK 实例
  gimbal_sdk_t sdk_{};

  // 接收线程
  std::atomic<bool> running_{false};
  std::thread recv_thread_;

  // 状态缓存
  mutable std::mutex feedback_mutex_;
  std_feedback_converted_t latest_feedback_{};

  // 用户回调
  std::mutex cb_mutex_;
  FeedbackCallback feedback_cb_;
  TriggerFeedbackCallback trigger_feedback_cb_;
};

#endif  // GIMBAL_CONTROLLER_H
