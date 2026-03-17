#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

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
 * @brief 无人机 UDP 通信控制器
 *
 * 基于 说明.md 协议文档实现，封装了与无人机之间的全部 UDP 通信：
 *   - 外界 → 无人机 (发送指令): mission / flyto / rtl
 *   - 无人机 → 外界 (接收回传): FlightUpload (20Hz 心跳) / CommonReply (任务完成回执)
 *
 * 通信端口：
 *   - 命令发送：  发往无人机 droneIP:8083
 *   - 心跳接收：  监听本地 0.0.0.0:8081（无人机主动推送 FlightUpload，20Hz）
 *   - 回执接收：  监听本地 0.0.0.0:cmd_port（无人机任务完成后回复 CommonReply）
 *
 * 用法示例:
 * @code
 *   DroneController drone("192.168.168.33", 8083, "0.0.0.0", 8081, 8088);
 *
 *   drone.setHeartbeatCallback([](const FlightUpload& hb) {
 *       printf("位置: %.7f, %.7f  高度: %.2fm\n", hb.lon, hb.lat, hb.relative_alt);
 *   });
 *
 *   drone.setReplyCallback([](const CommonReply& reply) {
 *       printf("任务 %s 完成, status=%d\n", reply.method.c_str(), reply.status);
 *   });
 *
 *   drone.start();
 *
 *   // 发送多航点任务
 *   std::vector<TargetLocation> targets = {
 *       {113.4567890, 22.1234567, 30.0},
 *       {113.4580000, 22.1245000, 25.0},
 *   };
 *   drone.sendMission(targets, 10.0);
 *
 *   // 飞向指定点
 *   drone.sendFlyto(113.456, 22.123, 30.0, 8.0);
 *
 *   // 返航
 *   drone.sendRTL();
 *
 *   drone.stop();
 * @endcode
 */
class DroneController {
 public:
  /// 心跳回调类型 (FlightUpload, 20Hz)
  using HeartbeatCallback = std::function<void(const FlightUpload&)>;
  /// 命令回执回调类型 (CommonReply, 任务完成时触发)
  using ReplyCallback = std::function<void(const CommonReply&)>;

  /**
   * @brief 构造函数
   * @param drone_ip     无人机 IP 地址
   * @param drone_port   无人机命令接收端口 (默认 8083)
   * @param listen_ip    本地监听 IP (通常 "0.0.0.0")
   * @param hb_port      本地心跳监听端口 (默认 8081，接收 FlightUpload)
   * @param reply_port   本地回执监听端口 (默认 8088，接收 CommonReply)
   */
  DroneController(const std::string& drone_ip, uint16_t drone_port = 8083, const std::string& listen_ip = "0.0.0.0", uint16_t hb_port = 8081, uint16_t reply_port = 8088);

  ~DroneController();

  // 禁止拷贝
  DroneController(const DroneController&) = delete;
  DroneController& operator=(const DroneController&) = delete;

  // ======================== 生命周期 ========================

  /**
   * @brief 启动接收线程，开始监听心跳和回执
   * @return true 启动成功
   */
  bool start();

  /**
   * @brief 停止所有接收线程并关闭 socket
   */
  void stop();

  /**
   * @brief 接收线程是否正在运行
   */
  bool isRunning() const { return running_.load(); }

  // ======================== 发送指令 ========================

  /**
   * @brief 发送多航点任务 (mission)
   *
   * 发送一组航点到无人机，每个航点可设置独立飞行高度。
   * 无人机会依次飞过每个航点，全部到达后回复 CommonReply。
   *
   * @param targets    航点列表 (TargetLocation: lon, lat, relative_alt)
   * @param speed      飞行速度 (m/s)，0 则使用无人机默认值 5.0
   * @param uav_id     无人机编号，默认 1
   * @return true 发送成功
   */
  bool sendMission(const std::vector<TargetLocation>& targets, double speed = 5.0, uint8_t uav_id = 1);

  /**
   * @brief 飞向指定点 (flyto)
   *
   * 飞至一个指定经纬度坐标点，到达后回复 CommonReply。
   *
   * @param lon          目标经度 (WGS84)
   * @param lat          目标纬度 (WGS84)
   * @param relative_alt 相对起飞点高度 (m)
   * @param speed        飞行速度 (m/s)，0 则使用无人机默认值 5.0
   * @param uav_id       无人机编号，默认 1
   * @param drop         保留字段，默认 0
   * @return true 发送成功
   */
  bool sendFlyto(double lon, double lat, double relative_alt, double speed = 5.0, uint8_t uav_id = 1, uint8_t drop = 0);

  /**
   * @brief 返航 (rtl)
   *
   * 命令无人机切换至 RTL 模式，自动返回起飞点并降落。
   * 落地上锁后回复 CommonReply。
   *
   * @param uav_id 无人机编号，默认 1
   * @return true 发送成功
   */
  bool sendRTL(uint8_t uav_id = 1);

  // ======================== 回调注册 ========================

  /**
   * @brief 注册心跳回调
   *
   * 无人机以 20Hz 频率回传 FlightUpload，每次收到都会触发此回调。
   * 回调在接收线程中执行，请勿做耗时操作。
   *
   * @param cb 回调函数
   */
  void setHeartbeatCallback(HeartbeatCallback cb);

  /**
   * @brief 注册命令回执回调
   *
   * 无人机任务完成（mission/flyto/rtl）后回复 CommonReply 时触发。
   * 回调在接收线程中执行。
   *
   * @param cb 回调函数
   */
  void setReplyCallback(ReplyCallback cb);

  // ======================== 状态查询 ========================

  /**
   * @brief 获取最新的心跳数据 (线程安全)
   */
  FlightUpload getLatestHeartbeat() const;

  /**
   * @brief 获取最新的命令回执 (线程安全)
   */
  CommonReply getLatestReply() const;

  /**
   * @brief 是否至少收到过一次心跳 (可作为连接状态参考)
   */
  bool hasReceivedHeartbeat() const { return heartbeat_received_.load(); }

  /**
   * @brief 获取最后一次收到心跳的时间点
   */
  std::chrono::steady_clock::time_point getLastHeartbeatTime() const;

 private:
  /// 通过 UDP 发送 JSON 字符串到无人机
  bool sendUDP(const std::string& json_msg);

  /// 心跳接收线程主循环 (监听 hb_port_)
  void heartbeatRecvLoop();

  /// 回执接收线程主循环 (监听 reply_port_)
  void replyRecvLoop();

  /// 解析心跳数据 (FlightUpload)
  void parseHeartbeat(const std::string& json_str);

  /// 解析回执数据 (CommonReply)
  void parseReply(const std::string& json_str);

  // 配置
  std::string drone_ip_;
  uint16_t drone_port_;
  std::string listen_ip_;
  uint16_t hb_port_;
  uint16_t reply_port_;

  // Socket
  int hb_sockfd_ = -1;     // 心跳接收 socket
  int reply_sockfd_ = -1;  // 回执接收 socket

  // 接收线程
  std::atomic<bool> running_{false};
  std::thread hb_thread_;
  std::thread reply_thread_;

  // 状态缓存 (带锁保护)
  mutable std::mutex state_mutex_;
  FlightUpload latest_heartbeat_{};
  CommonReply latest_reply_{};
  std::atomic<bool> heartbeat_received_{false};
  std::chrono::steady_clock::time_point last_heartbeat_time_;

  // 回调
  std::mutex cb_mutex_;
  HeartbeatCallback heartbeat_cb_;
  ReplyCallback reply_cb_;
};

#endif  // DRONE_CONTROLLER_H
