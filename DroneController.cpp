#include "include/DroneController.h"

#include <sys/select.h>
#include <cstring>

// ======================== 构造 / 析构 ========================

DroneController::DroneController(const std::string& drone_ip, uint16_t drone_port, const std::string& listen_ip, uint16_t hb_port, uint16_t reply_port)
    : drone_ip_(drone_ip), drone_port_(drone_port), listen_ip_(listen_ip), hb_port_(hb_port), reply_port_(reply_port) {}

DroneController::~DroneController() { stop(); }

// ======================== 生命周期 ========================

bool DroneController::start() {
  if (running_.load()) {
    std::cerr << "[DroneController] 已在运行中，请勿重复启动" << std::endl;
    return false;
  }

  // ---- 创建心跳接收 socket (hb_port_) ----
  hb_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (hb_sockfd_ < 0) {
    std::cerr << "[DroneController] 创建心跳接收 socket 失败: " << strerror(errno) << std::endl;
    return false;
  }

  int opt = 1;
  setsockopt(hb_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in hb_addr{};
  hb_addr.sin_family = AF_INET;
  hb_addr.sin_port = htons(hb_port_);
  if (inet_pton(AF_INET, listen_ip_.c_str(), &hb_addr.sin_addr) <= 0) {
    std::cerr << "[DroneController] 监听 IP 地址无效: " << listen_ip_ << std::endl;
    close(hb_sockfd_);
    hb_sockfd_ = -1;
    return false;
  }

  if (bind(hb_sockfd_, (sockaddr*)&hb_addr, sizeof(hb_addr)) < 0) {
    std::cerr << "[DroneController] 绑定心跳端口失败 (" << listen_ip_ << ":" << hb_port_ << "): " << strerror(errno) << std::endl;
    close(hb_sockfd_);
    hb_sockfd_ = -1;
    return false;
  }

  // ---- 创建回执接收 socket (reply_port_) ----
  reply_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (reply_sockfd_ < 0) {
    std::cerr << "[DroneController] 创建回执接收 socket 失败: " << strerror(errno) << std::endl;
    close(hb_sockfd_);
    hb_sockfd_ = -1;
    return false;
  }

  setsockopt(reply_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in reply_addr{};
  reply_addr.sin_family = AF_INET;
  reply_addr.sin_port = htons(reply_port_);
  if (inet_pton(AF_INET, listen_ip_.c_str(), &reply_addr.sin_addr) <= 0) {
    std::cerr << "[DroneController] 监听 IP 地址无效: " << listen_ip_ << std::endl;
    close(hb_sockfd_);
    hb_sockfd_ = -1;
    close(reply_sockfd_);
    reply_sockfd_ = -1;
    return false;
  }

  if (bind(reply_sockfd_, (sockaddr*)&reply_addr, sizeof(reply_addr)) < 0) {
    std::cerr << "[DroneController] 绑定回执端口失败 (" << listen_ip_ << ":" << reply_port_ << "): " << strerror(errno) << std::endl;
    close(hb_sockfd_);
    hb_sockfd_ = -1;
    close(reply_sockfd_);
    reply_sockfd_ = -1;
    return false;
  }

  std::cout << "[DroneController] 开始监听无人机心跳 [" << listen_ip_ << ":" << hb_port_ << "]" << std::endl;
  std::cout << "[DroneController] 开始监听无人机回执 [" << listen_ip_ << ":" << reply_port_ << "]" << std::endl;

  // 启动接收线程
  running_.store(true);
  hb_thread_ = std::thread(&DroneController::heartbeatRecvLoop, this);
  reply_thread_ = std::thread(&DroneController::replyRecvLoop, this);

  return true;
}

void DroneController::stop() {
  running_.store(false);

  if (hb_thread_.joinable()) {
    hb_thread_.join();
  }

  if (reply_thread_.joinable()) {
    reply_thread_.join();
  }

  if (hb_sockfd_ >= 0) {
    close(hb_sockfd_);
    hb_sockfd_ = -1;
  }

  if (reply_sockfd_ >= 0) {
    close(reply_sockfd_);
    reply_sockfd_ = -1;
  }

  std::cout << "[DroneController] 已停止" << std::endl;
}

// ======================== 发送指令 ========================

bool DroneController::sendMission(const std::vector<TargetLocation>& targets, double speed, uint8_t uav_id) {
  if (targets.empty()) {
    std::cerr << "[DroneController] sendMission: 航点列表不能为空" << std::endl;
    return false;
  }

  json j;
  j["uav_id"] = uav_id;
  j["method"] = "mission";
  j["target_num"] = static_cast<uint8_t>(targets.size());
  j["speed"] = speed;

  json target_arr = json::array();
  for (const auto& t : targets) {
    json loc;
    loc["lon"] = t.lon;
    loc["lat"] = t.lat;
    loc["relative_alt"] = t.relative_alt;
    target_arr.push_back(loc);
  }
  j["target_location"] = target_arr;

  return sendUDP(j.dump());
}

bool DroneController::sendFlyto(double lon, double lat, double relative_alt, double speed, uint8_t uav_id, uint8_t drop) {
  json j;
  j["uav_id"] = uav_id;
  j["method"] = "flyto";
  j["lon"] = lon;
  j["lat"] = lat;
  j["relative_alt"] = relative_alt;
  j["speed"] = speed;
  j["drop"] = drop;

  return sendUDP(j.dump());
}

bool DroneController::sendRTL(uint8_t uav_id) {
  json j;
  j["uav_id"] = uav_id;
  j["method"] = "rtl";

  return sendUDP(j.dump());
}

// ======================== 回调注册 ========================

void DroneController::setHeartbeatCallback(HeartbeatCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  heartbeat_cb_ = std::move(cb);
}

void DroneController::setReplyCallback(ReplyCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  reply_cb_ = std::move(cb);
}

// ======================== 状态查询 ========================

FlightUpload DroneController::getLatestHeartbeat() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_heartbeat_;
}

CommonReply DroneController::getLatestReply() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_reply_;
}

std::chrono::steady_clock::time_point DroneController::getLastHeartbeatTime() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_heartbeat_time_;
}

// ======================== 内部实现 ========================

bool DroneController::sendUDP(const std::string& json_msg) {
  // 每次发送创建独立 socket，避免多线程竞争
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "[DroneController] 创建发送 socket 失败: " << strerror(errno) << std::endl;
    return false;
  }

  // 绑定回执接收端口，使无人机回复能发回此端口
  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(reply_port_);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  bind(sockfd, (sockaddr*)&local_addr, sizeof(local_addr));

  sockaddr_in dest_addr{};
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(drone_port_);

  if (inet_pton(AF_INET, drone_ip_.c_str(), &dest_addr.sin_addr) <= 0) {
    std::cerr << "[DroneController] 目标 IP 地址无效: " << drone_ip_ << std::endl;
    close(sockfd);
    return false;
  }

  ssize_t sent = sendto(sockfd, json_msg.c_str(), json_msg.size(), 0, (sockaddr*)&dest_addr, sizeof(dest_addr));
  close(sockfd);

  if (sent < 0) {
    std::cerr << "[DroneController] UDP 发送失败: " << strerror(errno) << std::endl;
    return false;
  }

  std::cout << "[DroneController] --> drone: " << json_msg << std::endl;
  return true;
}

void DroneController::heartbeatRecvLoop() {
  constexpr int RECV_BUF_SIZE = 4096;
  char buffer[RECV_BUF_SIZE];
  sockaddr_in src_addr{};
  socklen_t addr_len = sizeof(src_addr);

  while (running_.load()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(hb_sockfd_, &readfds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int sel = select(hb_sockfd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;
      std::cerr << "[DroneController] 心跳 select 错误: " << strerror(errno) << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    } else if (sel == 0) {
      continue;  // 超时，重新检查 running_
    }

    if (FD_ISSET(hb_sockfd_, &readfds)) {
      ssize_t n = recvfrom(hb_sockfd_, buffer, RECV_BUF_SIZE - 1, 0, (sockaddr*)&src_addr, &addr_len);
      if (n < 0) {
        std::cerr << "[DroneController] 心跳 recvfrom 错误: " << strerror(errno) << std::endl;
        continue;
      }

      buffer[n] = '\0';
      try {
        parseHeartbeat(std::string(buffer, n));
      } catch (const std::exception& e) {
        std::cerr << "[DroneController] 心跳解析异常: " << e.what() << std::endl;
      }
    }
  }
}

void DroneController::replyRecvLoop() {
  constexpr int RECV_BUF_SIZE = 4096;
  char buffer[RECV_BUF_SIZE];
  sockaddr_in src_addr{};
  socklen_t addr_len = sizeof(src_addr);

  while (running_.load()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(reply_sockfd_, &readfds);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int sel = select(reply_sockfd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) continue;
      std::cerr << "[DroneController] 回执 select 错误: " << strerror(errno) << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    } else if (sel == 0) {
      continue;
    }

    if (FD_ISSET(reply_sockfd_, &readfds)) {
      ssize_t n = recvfrom(reply_sockfd_, buffer, RECV_BUF_SIZE - 1, 0, (sockaddr*)&src_addr, &addr_len);
      if (n < 0) {
        std::cerr << "[DroneController] 回执 recvfrom 错误: " << strerror(errno) << std::endl;
        continue;
      }

      buffer[n] = '\0';
      try {
        parseReply(std::string(buffer, n));
      } catch (const std::exception& e) {
        std::cerr << "[DroneController] 回执解析异常: " << e.what() << std::endl;
      }
    }
  }
}

void DroneController::parseHeartbeat(const std::string& json_str) {
  json j = json::parse(json_str);

  FlightUpload hb;
  j.get_to(hb);

  // 更新缓存
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_heartbeat_ = hb;
    last_heartbeat_time_ = std::chrono::steady_clock::now();
  }
  heartbeat_received_.store(true);

  // 触发回调
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (heartbeat_cb_) {
      heartbeat_cb_(hb);
    }
  }
}

void DroneController::parseReply(const std::string& json_str) {
  json j = json::parse(json_str);

  // 判断是否为 CommonReply：含 method + status 字段
  if (!j.contains("method") || !j.contains("status")) {
    std::cerr << "[DroneController] 收到的回执 JSON 缺少 method/status 字段" << std::endl;
    return;
  }

  CommonReply reply;
  j.get_to(reply);

  // 更新缓存
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_reply_ = reply;
  }

  // 触发回调
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (reply_cb_) {
      reply_cb_(reply);
    }
  }

  std::string status_str = (reply.status == 1) ? "成功" : "失败";
  std::cout << "[DroneController] <-- drone: " << reply.method << " 回执 (" << status_str << ")" << std::endl;
}
