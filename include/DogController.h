#ifndef DOG_CONTROLLER_H
#define DOG_CONTROLLER_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "global_common.h"

/**
 * @brief 机器狗 UDP 通信控制器
 *
 * 基于 comm_protocol.md (v1.0) 实现，封装了与机器狗之间的全部 UDP 通信：
 *   - 外界 → 机器狗 (发送指令): nav_path / action / cancel_nav / mode / posture
 *   - 机器狗 → 外界 (接收回传): dog_state (心跳) / nav_state (导航事件)
 *
 * 用法示例:
 * @code
 *   DogController dog("192.168.168.100", 8082, "0.0.0.0", 8081);
 *
 *   dog.setDogStateCallback([](const DogState& s) {
 *       std::cout << "电量: " << (int)s.power << "%" << std::endl;
 *   });
 *
 *   dog.setNavStateCallback([](const NavState& s) {
 *       std::cout << "导航结果: " << (s.nav_state == 1 ? "成功" : "失败") << std::endl;
 *   });
 *
 *   dog.start();                          // 启动接收线程
 *   dog.sendAction("stand_up");           // 站立
 *   dog.sendNavPath({{117.017, 36.661, 0.0, 1.57}}, 0.5);  // 导航
 *   // ...
 *   dog.stop();                           // 停止
 * @endcode
 */
class DogController {
 public:
  /// 心跳状态回调类型
  using DogStateCallback = std::function<void(const DogState&)>;
  /// 导航事件回调类型
  using NavStateCallback = std::function<void(const NavState&)>;

  /**
   * @brief 构造函数
   * @param dog_ip      机器狗 IP 地址
   * @param dog_port    机器狗接收指令端口 (默认 8082)
   * @param listen_ip   本地监听 IP (通常 "0.0.0.0")
   * @param listen_port 本地监听端口，用于接收机器狗的心跳与事件 (默认 8081)
   */
  DogController(const std::string& dog_ip, uint16_t dog_port, const std::string& listen_ip = "0.0.0.0", uint16_t listen_port = 8081);

  ~DogController();

  // 禁止拷贝
  DogController(const DogController&) = delete;
  DogController& operator=(const DogController&) = delete;

  // ======================== 生命周期 ========================

  /**
   * @brief 启动接收线程，开始监听机器狗回传的心跳和事件
   * @return true 启动成功
   */
  bool start();

  /**
   * @brief 停止接收线程并关闭所有 socket
   */
  void stop();

  /**
   * @brief 接收线程是否正在运行
   */
  bool isRunning() const { return running_.load(); }

  // ======================== 发送指令 ========================

  /**
   * @brief 发送导航路径 (nav_path)
   *
   * 发送一组 GPS 航点，机器狗将依次驶向各航点。
   * 收到新路径将覆盖当前任务。
   *
   * @param waypoints  航点列表，每个航点为 {经度, 纬度, 高度, 航向角(rad)}
   * @param target_vel 目标行驶速度 (m/s)，范围 (0, 0.5]，默认 0.5
   * @return true 发送成功
   */
  bool sendNavPath(const std::vector<std::array<double, 4>>& waypoints, float target_vel = 0.5f);

  /**
   * @brief 发送动作指令 (action)
   * @param action 动作名称: "stand_up" / "sit_down" / "emergency_stop"
   * @return true 发送成功
   */
  bool sendAction(const std::string& action);

  /**
   * @brief 取消导航 (cancel_nav)
   *
   * 立即取消当前导航任务，同时向机器狗发出急停指令。
   * @return true 发送成功
   */
  bool sendCancelNav();

  /**
   * @brief 切换模式 (mode)
   * @param mode "sdk" = SDK 自动模式, "remote" = 遥控器手动模式
   * @return true 发送成功
   */
  bool sendMode(const std::string& mode);

  /**
   * @brief 姿态控制 (posture) — 兼容旧协议
   * @param stand true = 站立, false = 趴下
   * @return true 发送成功
   */
  bool sendPosture(bool stand);

  // ======================== 回调注册 ========================

  /**
   * @brief 注册心跳状态回调
   *
   * 机器狗默认以 20Hz 频率回传 dog_state，每次收到都会触发此回调。
   * 回调在接收线程中执行，请勿在回调中做耗时操作。
   *
   * @param cb 回调函数
   */
  void setDogStateCallback(DogStateCallback cb);

  /**
   * @brief 注册导航事件回调
   *
   * 当导航状态变为 REACHED(1) 或 ERROR(0) 时触发，非持续发送。
   *
   * @param cb 回调函数
   */
  void setNavStateCallback(NavStateCallback cb);

  // ======================== 状态查询 ========================

  /**
   * @brief 获取最新的机器狗状态 (线程安全)
   */
  DogState getLatestDogState() const;

  /**
   * @brief 获取最新的导航状态 (线程安全)
   */
  NavState getLatestNavState() const;

  /**
   * @brief 是否至少收到过一次心跳 (可作为连接状态参考)
   */
  bool hasReceivedHeartbeat() const { return heartbeat_received_.load(); }

  /**
   * @brief 获取最后一次收到心跳的时间点
   */
  std::chrono::steady_clock::time_point getLastHeartbeatTime() const;

 private:
  /// 通过 UDP 发送 JSON 字符串到机器狗
  bool sendUDP(const std::string& json_msg);

  /// 接收线程主循环
  void recvLoop();

  /// 解析收到的 JSON 数据
  void parseReceivedData(const std::string& json_str);

  // 配置
  std::string dog_ip_;
  uint16_t dog_port_;
  std::string listen_ip_;
  uint16_t listen_port_;

  // Socket
  int send_sockfd_ = -1;
  int recv_sockfd_ = -1;

  // 接收线程
  std::atomic<bool> running_{false};
  std::thread recv_thread_;

  // 状态缓存 (带锁保护)
  mutable std::mutex state_mutex_;
  DogState latest_dog_state_{};
  NavState latest_nav_state_{};
  std::atomic<bool> heartbeat_received_{false};
  std::chrono::steady_clock::time_point last_heartbeat_time_;

  // 回调
  std::mutex cb_mutex_;
  DogStateCallback dog_state_cb_;
  NavStateCallback nav_state_cb_;
};

#endif  // DOG_CONTROLLER_H
