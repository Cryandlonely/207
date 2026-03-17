#include "include/DogController.h"

#include <sys/select.h>
#include <cstring>

// ======================== 构造 / 析构 ========================

DogController::DogController(const std::string& dog_ip, uint16_t dog_port, const std::string& listen_ip, uint16_t listen_port)
    : dog_ip_(dog_ip), dog_port_(dog_port), listen_ip_(listen_ip), listen_port_(listen_port) {}

DogController::~DogController() { stop(); }

// ======================== 生命周期 ========================

bool DogController::start() {
  if (running_.load()) {
    std::cerr << "[DogController] 已在运行中，请勿重复启动" << std::endl;
    return false;
  }

  // 创建接收 socket
  recv_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (recv_sockfd_ < 0) {
    std::cerr << "[DogController] 创建接收 socket 失败: " << strerror(errno) << std::endl;
    return false;
  }

  // 地址重用
  int opt = 1;
  setsockopt(recv_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // 绑定监听地址
  sockaddr_in listen_addr{};
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_port = htons(listen_port_);
  if (inet_pton(AF_INET, listen_ip_.c_str(), &listen_addr.sin_addr) <= 0) {
    std::cerr << "[DogController] 监听 IP 地址无效: " << listen_ip_ << std::endl;
    close(recv_sockfd_);
    recv_sockfd_ = -1;
    return false;
  }

  if (bind(recv_sockfd_, (sockaddr*)&listen_addr, sizeof(listen_addr)) < 0) {
    std::cerr << "[DogController] 绑定监听端口失败 (" << listen_ip_ << ":" << listen_port_ << "): " << strerror(errno) << std::endl;
    close(recv_sockfd_);
    recv_sockfd_ = -1;
    return false;
  }

  std::cout << "[DogController] 开始监听机器狗回传 [" << listen_ip_ << ":" << listen_port_ << "]" << std::endl;

  // 启动接收线程
  running_.store(true);
  recv_thread_ = std::thread(&DogController::recvLoop, this);

  return true;
}

void DogController::stop() {
  running_.store(false);

  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }

  if (recv_sockfd_ >= 0) {
    close(recv_sockfd_);
    recv_sockfd_ = -1;
  }

  if (send_sockfd_ >= 0) {
    close(send_sockfd_);
    send_sockfd_ = -1;
  }

  std::cout << "[DogController] 已停止" << std::endl;
}

// ======================== 发送指令 ========================

bool DogController::sendNavPath(const std::vector<std::array<double, 4>>& waypoints, float target_vel) {
  if (waypoints.empty()) {
    std::cerr << "[DogController] sendNavPath: 航点列表不能为空" << std::endl;
    return false;
  }

  // 限速范围 (0, 5.0]
  if (target_vel <= 0.0f || target_vel > 5.0f) {
    std::cerr << "[DogController] sendNavPath: target_vel 超出范围 (0, 5.0]，已钳制" << std::endl;
    target_vel = std::min(std::max(target_vel, 0.01f), 5.0f);
  }

  json j;
  j["type"] = "nav_path";
  j["target_vel"] = target_vel;

  json path_pt = json::array();
  for (const auto& wp : waypoints) {
    path_pt.push_back({wp[0], wp[1], wp[2], wp[3]});
  }
  j["path_pt"] = path_pt;

  return sendUDP(j.dump());
}

bool DogController::sendAction(const std::string& action) {
  // 校验合法值
  if (action != "stand_up" && action != "sit_down" && action != "emergency_stop") {
    std::cerr << "[DogController] sendAction: 不支持的动作 \"" << action << "\"，合法值: stand_up / sit_down / emergency_stop" << std::endl;
    return false;
  }

  json j;
  j["type"] = "action";
  j["data"] = action;

  return sendUDP(j.dump());
}

bool DogController::sendCancelNav() {
  json j;
  j["type"] = "cancel_nav";

  return sendUDP(j.dump());
}

bool DogController::sendMode(const std::string& mode) {
  if (mode != "sdk" && mode != "remote") {
    std::cerr << "[DogController] sendMode: 不支持的模式 \"" << mode << "\"，合法值: sdk / remote" << std::endl;
    return false;
  }

  json j;
  j["type"] = "mode";
  j["data"] = mode;

  return sendUDP(j.dump());
}

bool DogController::sendPosture(bool stand) {
  json j;
  j["type"] = "posture";
  j["data"] = stand;

  return sendUDP(j.dump());
}

// ======================== 回调注册 ========================

void DogController::setDogStateCallback(DogStateCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  dog_state_cb_ = std::move(cb);
}

void DogController::setNavStateCallback(NavStateCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  nav_state_cb_ = std::move(cb);
}

// ======================== 状态查询 ========================

DogState DogController::getLatestDogState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_dog_state_;
}

NavState DogController::getLatestNavState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_nav_state_;
}

std::chrono::steady_clock::time_point DogController::getLastHeartbeatTime() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_heartbeat_time_;
}

// ======================== 内部实现 ========================

bool DogController::sendUDP(const std::string& json_msg) {
  // 每次发送创建独立 socket，避免多线程竞争（与原 pad_to_device 风格一致）
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "[DogController] 创建发送 socket 失败: " << strerror(errno) << std::endl;
    return false;
  }

  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in dest_addr{};
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(dog_port_);

  if (inet_pton(AF_INET, dog_ip_.c_str(), &dest_addr.sin_addr) <= 0) {
    std::cerr << "[DogController] 目标 IP 地址无效: " << dog_ip_ << std::endl;
    close(sockfd);
    return false;
  }

  ssize_t sent = sendto(sockfd, json_msg.c_str(), json_msg.size(), 0, (sockaddr*)&dest_addr, sizeof(dest_addr));
  close(sockfd);

  if (sent < 0) {
    std::cerr << "[DogController] UDP 发送失败: " << strerror(errno) << std::endl;
    return false;
  }

  std::cout << "[DogController] --> dog: " << json_msg << std::endl;
  return true;
}

void DogController::recvLoop() {
  constexpr int RECV_BUF_SIZE = 4096;
  char buffer[RECV_BUF_SIZE];
  sockaddr_in src_addr{};
  socklen_t addr_len = sizeof(src_addr);

  while (running_.load()) {
    // 使用 select 设置超时，避免阻塞导致无法退出
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(recv_sockfd_, &readfds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int sel = select(recv_sockfd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;  // 信号中断，重试
      std::cerr << "[DogController] select 错误: " << strerror(errno) << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    } else if (sel == 0) {
      // 超时，继续检查 running_ 标志
      continue;
    }

    if (FD_ISSET(recv_sockfd_, &readfds)) {
      ssize_t n = recvfrom(recv_sockfd_, buffer, RECV_BUF_SIZE - 1, 0, (sockaddr*)&src_addr, &addr_len);
      if (n < 0) {
        std::cerr << "[DogController] recvfrom 错误: " << strerror(errno) << std::endl;
        continue;
      }

      buffer[n] = '\0';
      try {
        std::string json_str(buffer, n);
        parseReceivedData(json_str);
      } catch (const std::exception& e) {
        std::cerr << "[DogController] 数据解析异常: " << e.what() << std::endl;
      }
    }
  }
}

void DogController::parseReceivedData(const std::string& json_str) {
  json j = json::parse(json_str);

  if (!j.contains("type")) {
    std::cerr << "[DogController] 收到的 JSON 缺少 type 字段" << std::endl;
    return;
  }

  std::string type = j["type"].get<std::string>();

  if (type == "dog_state") {
    DogState state;
    j.get_to(state);

    // 更新缓存状态
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_dog_state_ = state;
      last_heartbeat_time_ = std::chrono::steady_clock::now();
    }
    heartbeat_received_.store(true);

    // 触发回调
    {
      std::lock_guard<std::mutex> lock(cb_mutex_);
      if (dog_state_cb_) {
        dog_state_cb_(state);
      }
    }
  } else if (type == "nav_state") {
    NavState state;
    j.get_to(state);

    // 更新缓存状态
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_nav_state_ = state;
    }

    // 触发回调
    {
      std::lock_guard<std::mutex> lock(cb_mutex_);
      if (nav_state_cb_) {
        nav_state_cb_(state);
      }
    }

    std::cout << "[DogController] <-- dog: nav_state = " << (int)state.nav_state << (state.nav_state == 1 ? " (导航成功)" : " (导航失败)") << std::endl;
  } else {
    std::cout << "[DogController] <-- dog: 未知消息类型 \"" << type << "\"" << std::endl;
  }
}
