#include "include/GimbalController.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

// ======================== 构造 / 析构 ========================

GimbalController::GimbalController(const std::string& ip, uint16_t port) : ip_(ip), port_(port) { memset(&latest_feedback_, 0, sizeof(latest_feedback_)); }

GimbalController::~GimbalController() { stop(); }

// ======================== 生命周期 ========================

bool GimbalController::start() {
  if (running_.load()) {
    std::cerr << "[GimbalController] 已在运行中" << std::endl;
    return false;
  }

  if (!tcpConnect()) {
    return false;
  }

  // 初始化 SDK
  gimbal_sdk_init(&sdk_, tcpWriteFunc, &sock_fd_);
  gimbal_sdk_register_feedback_callback(&sdk_, feedbackBridge, this);
  gimbal_sdk_register_trigger_feedback_callback(&sdk_, triggerFeedbackBridge, this);

  // 设为活跃实例（SDK 全局函数需要）
  std_sdk_set_active_instance(&sdk_);

  running_.store(true);
  recv_thread_ = std::thread(&GimbalController::recvLoop, this);

  std::cout << "[GimbalController] 已启动，连接 " << ip_ << ":" << port_ << std::endl;
  return true;
}

void GimbalController::stop() {
  running_.store(false);

  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }

  if (sock_fd_ >= 0) {
    close(sock_fd_);
    sock_fd_ = -1;
  }

  std::cout << "[GimbalController] 已停止" << std::endl;
}

// ======================== 角度控制 ========================

bool GimbalController::specifyAngle(float pitch, float yaw) {
  auto pkt = std_control_specify_angle(pitch, yaw);
  return sendPacket(pkt);
}

bool GimbalController::specifyPitch(float pitch) {
  auto pkt = std_control_specify_pitch(pitch);
  return sendPacket(pkt);
}

bool GimbalController::specifyYaw(float yaw) {
  auto pkt = std_control_specify_yaw(yaw);
  return sendPacket(pkt);
}

// ======================== 速度控制 ========================

bool GimbalController::velocityControl(int16_t pitch_speed, int16_t yaw_speed) {
  auto pkt = std_control_velocity(pitch_speed, yaw_speed);
  return sendPacket(pkt);
}

bool GimbalController::stopMovement() {
  auto pkt = std_control_stop_movement();
  return sendPacket(pkt);
}

// ======================== 一键操作 ========================

bool GimbalController::onekeyCenter() {
  auto pkt = std_control_onekey_center();
  return sendPacket(pkt);
}

bool GimbalController::onekeyDown() {
  auto pkt = std_control_onekey_down();
  return sendPacket(pkt);
}

// ======================== 模式切换 ========================

bool GimbalController::modeLock() { return modeSwitch(MODE_LOCK); }

bool GimbalController::modeFollow() { return modeSwitch(MODE_FOLLOW); }

bool GimbalController::modeSwitch(std_mode_t mode) {
  auto pkt = std_control_mode_switch(mode);
  return sendPacket(pkt);
}

// ======================== 相机控制 ========================

bool GimbalController::takePhoto() {
  auto pkt = std_control_take_photo();
  return sendPacket(pkt);
}

bool GimbalController::startRecording() {
  auto pkt = std_control_start_recording();
  return sendPacket(pkt);
}

bool GimbalController::stopRecording() {
  auto pkt = std_control_stop_recording();
  return sendPacket(pkt);
}

// ======================== 变倍控制 ========================

bool GimbalController::zoomIn(uint8_t speed) {
  auto pkt = std_control_zoom_in(speed);
  return sendPacket(pkt);
}

bool GimbalController::zoomOut(uint8_t speed) {
  auto pkt = std_control_zoom_out(speed);
  return sendPacket(pkt);
}

bool GimbalController::zoomStop() {
  auto pkt = std_control_zoom_stop();
  return sendPacket(pkt);
}

bool GimbalController::specifyZoom(float level) {
  auto pkt = std_control_specify_zoom(level);
  return sendPacket(pkt);
}

// ======================== 对焦控制 ========================

bool GimbalController::focusIn(uint8_t speed) {
  auto pkt = std_control_focus_in(speed);
  return sendPacket(pkt);
}

bool GimbalController::focusOut(uint8_t speed) {
  auto pkt = std_control_focus_out(speed);
  return sendPacket(pkt);
}

bool GimbalController::focusStop() {
  auto pkt = std_control_focus_stop();
  return sendPacket(pkt);
}

// ======================== 跟踪控制 ========================

bool GimbalController::pointTracking(float x, float y) {
  auto pkt = std_control_point_tracking(x, y);
  return sendPacket(pkt);
}

bool GimbalController::boxTracking(float x_min, float y_min, float x_max, float y_max) {
  auto pkt = std_control_box_tracking(x_min, y_min, x_max, y_max);
  return sendPacket(pkt);
}

bool GimbalController::trackingOff() {
  auto pkt = std_control_tracking_off();
  return sendPacket(pkt);
}

// ======================== 激光控制 ========================

bool GimbalController::laserOn() {
  auto pkt = std_control_laser_control(LASER_ON);
  return sendPacket(pkt);
}

bool GimbalController::laserOff() {
  auto pkt = std_control_laser_control(LASER_OFF);
  return sendPacket(pkt);
}

bool GimbalController::laserRangeSingle() {
  auto pkt = std_control_laser_ranging(LASER_DISTANCE_SINGLE);
  return sendPacket(pkt);
}

bool GimbalController::laserRangeContinuous() {
  auto pkt = std_control_laser_ranging(LASER_DISTANCE_CONTINUOUS);
  return sendPacket(pkt);
}

bool GimbalController::laserRangeStop() {
  auto pkt = std_control_laser_ranging(LASER_DISTANCE_STOP);
  return sendPacket(pkt);
}

// ======================== 其他控制 ========================

bool GimbalController::stabilizationOn() {
  auto pkt = std_control_stabilization(STABILIZATION_ON);
  return sendPacket(pkt);
}

bool GimbalController::stabilizationOff() {
  auto pkt = std_control_stabilization(STABILIZATION_OFF);
  return sendPacket(pkt);
}

bool GimbalController::pipSwitch(std_pip_t mode) {
  auto pkt = std_control_pip_switch(mode);
  return sendPacket(pkt);
}

bool GimbalController::pseudoColor(std_pseudo_color_t mode) {
  auto pkt = std_control_pseudo_color(mode);
  return sendPacket(pkt);
}

bool GimbalController::osdOn() {
  auto pkt = std_control_osd_on();
  return sendPacket(pkt);
}

bool GimbalController::osdOff() {
  auto pkt = std_control_osd_off();
  return sendPacket(pkt);
}

// ======================== 回调注册 ========================

void GimbalController::setFeedbackCallback(FeedbackCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  feedback_cb_ = std::move(cb);
}

void GimbalController::setTriggerFeedbackCallback(TriggerFeedbackCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  trigger_feedback_cb_ = std::move(cb);
}

// ======================== 状态查询 ========================

std_feedback_converted_t GimbalController::getLatestFeedback() const {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return latest_feedback_;
}

// ======================== 内部实现 ========================

bool GimbalController::tcpConnect() {
  sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    std::cerr << "[GimbalController] socket 创建失败: " << strerror(errno) << std::endl;
    return false;
  }

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  if (inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) <= 0) {
    std::cerr << "[GimbalController] 无效 IP: " << ip_ << std::endl;
    close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  if (connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "[GimbalController] 连接云台失败 (" << ip_ << ":" << port_ << "): " << strerror(errno) << std::endl;
    close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  // 设置接收超时 2 秒
  struct timeval tv {};
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  std::cout << "[GimbalController] TCP 已连接 " << ip_ << ":" << port_ << std::endl;
  return true;
}

int GimbalController::tcpWriteFunc(const uint8_t* data, uint16_t len, void* user_ptr) {
  int fd = *static_cast<int*>(user_ptr);
  if (fd < 0 || !data) return -1;

  ssize_t n = send(fd, data, len, MSG_NOSIGNAL);
  if (n < 0) {
    perror("[GimbalController] tcp send");
    return -1;
  }
  return static_cast<int>(n);
}

void GimbalController::feedbackBridge(const std_feedback_converted_t* fb, void* user_data) {
  if (!fb || !user_data) return;
  auto* self = static_cast<GimbalController*>(user_data);

  // 缓存最新数据
  {
    std::lock_guard<std::mutex> lock(self->feedback_mutex_);
    self->latest_feedback_ = *fb;
  }

  // 触发用户回调
  {
    std::lock_guard<std::mutex> lock(self->cb_mutex_);
    if (self->feedback_cb_) {
      self->feedback_cb_(*fb);
    }
  }
}

void GimbalController::triggerFeedbackBridge(const std_trigger_feedback_converted_t* fb, void* user_data) {
  if (!fb || !user_data) return;
  auto* self = static_cast<GimbalController*>(user_data);

  std::lock_guard<std::mutex> lock(self->cb_mutex_);
  if (self->trigger_feedback_cb_) {
    self->trigger_feedback_cb_(*fb);
  }
}

void GimbalController::recvLoop() {
  constexpr int BUF_SIZE = 512;
  uint8_t buf[BUF_SIZE];

  while (running_.load() && sock_fd_ >= 0) {
    ssize_t n = recv(sock_fd_, buf, sizeof(buf), 0);
    if (n > 0) {
      for (ssize_t i = 0; i < n; i++) {
        gimbal_sdk_input_byte(&sdk_, buf[i]);
      }
    } else if (n == 0) {
      std::cerr << "[GimbalController] 云台断开连接" << std::endl;
      running_.store(false);
      break;
    } else {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;  // 接收超时，继续
      }
      if (running_.load()) {
        std::cerr << "[GimbalController] recv 错误: " << strerror(errno) << std::endl;
      }
      break;
    }
  }
}

bool GimbalController::sendPacket(const std_packet_t& pkt) {
  if (sock_fd_ < 0) {
    std::cerr << "[GimbalController] 未连接，无法发送" << std::endl;
    return false;
  }

  if (pkt.length == 0) {
    std::cerr << "[GimbalController] 数据包为空" << std::endl;
    return false;
  }

  ssize_t n = send(sock_fd_, pkt.data, pkt.length, MSG_NOSIGNAL);
  if (n < 0) {
    std::cerr << "[GimbalController] 发送失败: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}
