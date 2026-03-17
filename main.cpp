#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <zmq.h>
#include <string>
#include <thread>
#include <zmq.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

#include "include/DogController.h"
#include "include/DroneController.h"
#include "include/GimbalController.h"
#include "include/SerialManager.h"
#include "include/geo_utils.h"
#include "include/global_common.h"
#include "speaker_interface.h"
#include "speaker_utils.h"

#include <log4cpp/Appender.hh>
#include <log4cpp/Category.hh>
#include <log4cpp/Layout.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/Priority.hh>
#include <log4cpp/RollingFileAppender.hh>

log4cpp::Category& logger = log4cpp::Category::getRoot();

static std::string Pad_ip;
static std::string Pad_port_pub;
static std::string Pad_port_sub;
static std::string device_id;
static std::string device_ip;

// 机器狗配置
static std::string dog_ip;
static uint16_t dog_send_port;
static uint16_t dog_listen_port;

// 无人机配置
static std::string plane_ip;
static uint16_t drone_cmd_port;
static uint16_t drone_hb_port;
static uint16_t drone_reply_port;

// 云台配置
static std::string gimbal_ip;
static uint16_t gimbal_port;

std::string dog_or_plane = "dog";  // 默认为无人狗
bool is_live = true;

// 控制器实例（配置加载后构造）
std::unique_ptr<DogController> dog;
std::unique_ptr<DroneController> drone;
std::unique_ptr<GimbalController> gimbalController;

DogState dog_state;
NavState nav_state;

// 云台反馈数据缓存（由 gimbalController feedback 回调更新）
std_feedback_converted_t gimbal_feedback{};

FlightUpload flight_upload;

Flyto flyto;
Mission mission;
Flight flight;
Gimbal gimbal;
Follow follow;
Upload upload;

std::atomic<bool> hitFlag{false};
std::atomic<bool> flytoFlag{false};
std::atomic<bool> missionFlag{false};
std::atomic<bool> flightFLag{false};
std::atomic<bool> flight_upload_flag{false};
std::atomic<bool> dog_upload_flag{false};

std::atomic<bool> followReply{false};

uint8_t taskType = 0;

// 当前正在执行的任务类型：0=无任务, 1=flyto(单点巡航), 2=mission(多点航线)
std::atomic<uint8_t> currentTaskType{0};

// 保存本次 mission 的 type 字段，用于回执时原样带回
std::atomic<int8_t> mission_type{0};

double takeoff_target_distance = 0;

int8_t flyto_nodeid = -1;
int8_t mission_nodeid = -1;
int8_t flight_nodeid = -1;

zmq::context_t context(1);
zmq::socket_t socket_pad(context, ZMQ_SUB);
zmq::socket_t socket_heart_over(context, ZMQ_PUB);
std::mutex socket_pub_mutex;  // 保护 socket_heart_over 多线程并发发送

PodReturnPad pod_return_pad;

void getIpByYaml(const std::string& path) {
  YAML::Node config = YAML::LoadFile(path);

  // Pad 配置
  Pad_ip = config["Pad"]["ip"].as<std::string>();
  Pad_port_sub = config["Pad"]["port_sub"].as<std::string>();
  Pad_port_pub = config["Pad"]["port_pub"].as<std::string>();

  // 本机配置
  device_ip = config["deviceIP"].as<std::string>();
  device_id = config["deviceId"].as<std::string>();

  // 机器狗
  dog_ip = config["dogIP"].as<std::string>();
  dog_send_port = config["dogSendPort"].as<uint16_t>();
  dog_listen_port = config["dogListenPort"].as<uint16_t>();

  // 无人机
  plane_ip = config["planIP"].as<std::string>();
  drone_cmd_port = config["droneCmdPort"].as<uint16_t>();
  drone_hb_port = config["droneHbPort"].as<uint16_t>();
  drone_reply_port = config["droneReplyPort"].as<uint16_t>();

  // 云台
  gimbal_ip = config["gimbalIP"].as<std::string>();
  gimbal_port = config["gimbalPort"].as<uint16_t>();

  // 设备类型
  if ("1" == config["identy"].as<std::string>()) {
    dog_or_plane = "plane";
  }
}

int pad_to_device(std::string ip, uint16_t port, std::string msg) {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  // 设置地址重用
  int opt = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in dest_addr{};
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(port);

  if (inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr) <= 0) {
    logger.error("目标IP地址无效: " + ip);
    close(sockfd);
    return 0;
  }
  ssize_t sent = sendto(sockfd, msg.c_str(), strlen(msg.c_str()), 0, (sockaddr*)&dest_addr, sizeof(dest_addr));
  if (sent < 0) {
    logger.error("UDP发送失败");
    return 0;
  }
  logger.info("3588 --> device:   " + msg);
  close(sockfd);
  return 1;
}

// 向Pad端发送实时信息
void heartbeat_thread() {
  logger.info("[heartbeat] bind: tcp://" + device_ip + ":" + Pad_port_pub);
  socket_heart_over.bind("tcp://" + device_ip + ":" + Pad_port_pub);

  // 设置ZMQ高水位标记
  int hwm = 100;
  socket_heart_over.set(zmq::sockopt::sndhwm, hwm);

  // 等待订阅者连接
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  while (true) {
    if ("dog" == dog_or_plane) {
      upload.sn = dog_state.sn;
      upload.battery.capacity_percent = dog_state.power;
      upload.signal = "强";
      upload.location.longitude = dog_state.lon;
      upload.location.latitude = dog_state.lat;
      upload.location.altitude = dog_state.alt;

      upload.angel.head = dog_state.yaw;
      upload.angel.pitch = dog_state.pitch;
      upload.angel.roll = dog_state.roll;

      // 云台反馈角度
      upload.angel.gimbal_pitch = gimbal_feedback.pitch_angle;
      upload.angel.gimbal_roll = gimbal_feedback.roll_angle;
      upload.angel.gimbal_head = gimbal_feedback.yaw_angle;
    }
    if ("plane" == dog_or_plane) {
      upload.battery.capacity_percent = flight_upload.battery_voltage;
      upload.location.longitude = flight_upload.lon;
      upload.location.latitude = flight_upload.lat;
      upload.location.altitude = flight_upload.alt;
      upload.angel.head = flight_upload.yaw;
      upload.angel.roll = flight_upload.roll;
      upload.angel.pitch = flight_upload.pitch;
      upload.speed = flight_upload.speed;
      upload.angel.gimbal_head = gimbal.yaw;
      upload.angel.gimbal_pitch = gimbal.pitch;
      upload.rtkMode = flight_upload.gnss_fix_type;
    }

    if (dog_upload_flag.load()) {
      json j_upload = upload;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/state" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      std::lock_guard<std::mutex> lock(socket_pub_mutex);
      socket_heart_over.send(topic_msg, zmq::send_flags::none);
    }
    // 发送实时信息
    if (flight_upload_flag.load()) {
      json j_upload = upload;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/state" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      std::lock_guard<std::mutex> lock(socket_pub_mutex);
      socket_heart_over.send(topic_msg, zmq::send_flags::none);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void reply_pad_thread() {
  // 复用 socket_heart_over，不再重复 bind，通过 socket_pub_mutex 保证线程安全

  // 等待 heartbeat_thread 完成 bind
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  while (true) {
    try {
    if (followReply.load()) {
      FollowReply follow_reply;
      follow_reply.nodeId = follow.nodeId;
      follow_reply.method = "follow_reply";
      follow_reply.latitude = pod_return_pad.target_lat;
      follow_reply.longitude = pod_return_pad.target_lon;
      follow_reply.altitude = pod_return_pad.target_high;
      follow_reply.status = 1;
      json j_upload = follow_reply;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/follow_reply" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      std::lock_guard<std::mutex> lock(socket_pub_mutex);
      socket_heart_over.send(topic_msg, zmq::send_flags::none);
      logger.info("3588 --> pad:   followReply");
    }

    if (hitFlag.load()) {
      MissionReply mission_reply;
      mission_reply.nodeId = mission_nodeid;
      mission_reply.method = "mission_reply";
      mission_reply.type = taskType;
      mission_reply.status = 1;
      json j_upload = mission_reply;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/mission_reply" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      {
        std::lock_guard<std::mutex> lock(socket_pub_mutex);
        socket_heart_over.send(topic_msg, zmq::send_flags::none);
      }
      logger.info("3588 --> pad:   missionReply");
      hitFlag.store(false);
    }

    if (flytoFlag.load()) {
      FlytoReply flyto_reply;
      flyto_reply.nodeId = flyto_nodeid;
      flyto_reply.method = "flyto_reply";
      flyto_reply.type = 0;
      flyto_reply.status = 1;
      json j_upload = flyto_reply;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/flyto_reply" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      {
        std::lock_guard<std::mutex> lock(socket_pub_mutex);
        socket_heart_over.send(topic_msg, zmq::send_flags::none);
      }
      logger.info("3588 --> pad: flyto_reply, nodeId=" + std::to_string(flyto_nodeid));
      flytoFlag.store(false);
    }

    if (missionFlag.load()) {
      MissionReply mission_reply;
      mission_reply.nodeId = mission_nodeid;
      mission_reply.method = "mission_reply";
      mission_reply.type = mission_type.load();  // 使用实际保存的 type
      mission_reply.status = 1;
      json j_upload = mission_reply;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/mission_reply" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      {
        std::lock_guard<std::mutex> lock(socket_pub_mutex);
        socket_heart_over.send(topic_msg, zmq::send_flags::none);
      }
      logger.info("3588 --> pad: mission_reply, nodeId=" + std::to_string(mission_nodeid) + " type=" + std::to_string(mission_type.load()));
      missionFlag.store(false);
    }

    if (flightFLag.load()) {
      FlightReply flight_reply;
      flight_reply.nodeId = flight_nodeid;
      flight_reply.method = "gohome_reply";
      flight_reply.status = 1;
      json j_upload = flight_reply;
      std::string upload_str = j_upload.dump();
      std::string topic = "zmq/" + device_id + "/gohome_reply" + upload_str;
      zmq::message_t topic_msg(topic.data(), topic.size());
      {
        std::lock_guard<std::mutex> lock(socket_pub_mutex);
        socket_heart_over.send(topic_msg, zmq::send_flags::none);
      }
      logger.info("3588 --> pad: gohome_reply, nodeId=" + std::to_string(flight_nodeid));
      flightFLag.store(false);
    }

      // 避免CPU空转，添加短暂休眠
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } catch (const zmq::error_t& e) {
      logger.error(std::string("[reply_pad] ZMQ异常，线程继续运行: ") + e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception& e) {
      logger.error(std::string("[reply_pad] 异常，线程继续运行: ") + e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

// 监听 Pad端 / 主站 指令
void listen_pad_thread() {
  zmq::socket_t socket_pubPod(context, ZMQ_PUB);
  socket_pubPod.bind("tcp://" + device_ip + ":18888");

  // 构造订阅前缀(zmq)
  std::string topic_prefix = "zmq/" + device_id;

  logger.info("[listen_pad] topic_prefix: " + topic_prefix);

  socket_pad.set(zmq::sockopt::subscribe, topic_prefix);
  // 连接主站（原逻辑保留）
  socket_pad.connect("tcp://" + Pad_ip + ":" + Pad_port_sub);

  logger.info("[listen_pad] connected: tcp://" + Pad_ip + ":" + Pad_port_sub);

  while (true) {
    zmq::message_t topic_frame;
    if (socket_pad.recv(topic_frame)) {
      std::string full_topic(static_cast<char*>(topic_frame.data()), topic_frame.size());

      // 分割主题层级
      size_t first_slash = full_topic.find('/');                    // zmq后的/位置
      size_t second_slash = full_topic.find('/', first_slash + 1);  // 序列号后的/位置
      std::string data = full_topic.substr(second_slash + 1);

      logger.info("pad --> 3588:  " + data);

      size_t pos = data.find("{");

      std::string action_type = data.substr(0, pos);

      json j = json::parse(data.substr(pos));

      // 接收到pad信息后发给无人装备
      if (action_type == "flyto") {
        j.get_to(flyto);
        flyto_nodeid = flyto.nodeId;
        Location target_location = flyto.location;
        if ("dog" == dog_or_plane) {
          currentTaskType.store(1);  // 标记为 flyto 任务
          std::vector<std::array<double, 4>> waypoints;
          std::array<double, 4> waypoint;
          waypoint[0] = target_location.longitude;
          waypoint[1] = target_location.latitude;
          waypoint[2] = target_location.altitude;
          waypoint[3] = -calculateRotationFromGPS(dog_state.lat, dog_state.lon, target_location.latitude, target_location.longitude);
          waypoints.push_back(waypoint);
          logger.info("[flyto->dog] 发送航点: lon=" + std::to_string(waypoint[0])
            + " lat=" + std::to_string(waypoint[1])
            + " alt=" + std::to_string(waypoint[2])
            + " yaw=" + std::to_string(waypoint[3])
            + " speed=" + std::to_string(flyto.speed));
          dog->sendNavPath(waypoints, flyto.speed);
        }

        if ("plane" == dog_or_plane) {
          currentTaskType.store(1);  // 标记为 flyto 任务
          logger.info("[flyto->plane] 发送目标点: lon=" + std::to_string(target_location.longitude)
            + " lat=" + std::to_string(target_location.latitude)
            + " alt=" + std::to_string(target_location.altitude)
            + " speed=" + std::to_string(flyto.speed));
          drone->sendFlyto(target_location.longitude, target_location.latitude, target_location.altitude, flyto.speed);
        }

        logger.info("pad --> 3588: flyto, nodeId=" + std::to_string(flyto_nodeid));
      } else if (action_type == "mission") {
        j.get_to(mission);
        if (mission.location.size() > 0) {
          std::vector<std::array<double, 4>> waypoints;
          if ("dog" == dog_or_plane) {
            currentTaskType.store(2);  // 标记为 mission 任务
            mission_nodeid = mission.nodeId;
            mission_type.store(mission.type);  // 保存 type，回执时原样带回

            // 先填充经纬度高度，yaw 暂置 0
            for (const auto& loc : mission.location) {
              std::array<double, 4> waypoint;
              waypoint[0] = loc.longitude;
              waypoint[1] = loc.latitude;
              waypoint[2] = loc.altitude;
              waypoint[3] = 0.0;
              waypoints.push_back(waypoint);
            }

            // 计算每个航点的 yaw：指向下一个航点方向，最后一个与前一个保持一致
            for (size_t i = 0; i < waypoints.size(); i++) {
              if (i + 1 < waypoints.size()) {
                // 当前点 → 下一个点的方向
                waypoints[i][3] = -calculateRotationFromGPS(waypoints[i][1], waypoints[i][0],         // 当前点 lat, lon
                                                            waypoints[i + 1][1], waypoints[i + 1][0]  // 下一点 lat, lon
                );
              } else {
                // 最后一个航点，沿用前一个的 yaw
                waypoints[i][3] = (i > 0) ? waypoints[i - 1][3] : 0.0;
              }
            }

            dog->sendNavPath(waypoints, mission.speed);
            // 记录最终发送的所有航点详情
            for (size_t i = 0; i < waypoints.size(); i++) {
              logger.info("[mission->dog] 航点[" + std::to_string(i) + "]: lon=" + std::to_string(waypoints[i][0])
                + " lat=" + std::to_string(waypoints[i][1])
                + " alt=" + std::to_string(waypoints[i][2])
                + " yaw=" + std::to_string(waypoints[i][3])
                + " speed=" + std::to_string(mission.speed));
            }
          }
          if ("plane" == dog_or_plane) {
            currentTaskType.store(2);  // 标记为 mission 任务
            mission_nodeid = mission.nodeId;
            mission_type.store(mission.type);  // 保存 type，回执时原样带回
            std::vector<TargetLocation> targets;
            for (const auto& loc : mission.location) {
              TargetLocation t;
              t.lon = loc.longitude;
              t.lat = loc.latitude;
              t.relative_alt = loc.altitude;
              targets.push_back(t);
            }
            drone->sendMission(targets, mission.speed);
            // 记录最终发送的所有目标点详情
            for (size_t i = 0; i < targets.size(); i++) {
              logger.info("[mission->plane] 目标[" + std::to_string(i) + "]: lon=" + std::to_string(targets[i].lon)
                + " lat=" + std::to_string(targets[i].lat)
                + " alt=" + std::to_string(targets[i].relative_alt)
                + " speed=" + std::to_string(mission.speed));
            }
          }
          logger.info("pad --> 3588: mission, nodeId=" + std::to_string(mission.nodeId) + ", waypoints=" + std::to_string(mission.location.size()));
        }
      } else if (action_type == "flight") {
        j.get_to(flight);
        if ("plane" == dog_or_plane) {
          if (flight.method == "land") {
            LandControl land_control;
            land_control.uav_id = std::stoi(device_id);
            land_control.method = "land";
            json j_land_control = land_control;
            pad_to_device(plane_ip, drone_cmd_port, j_land_control.dump());
            logger.info("pad --> 3588: land");
          }
          if ("takeoff" == flight.method) {
            HoverControl hover_control;
            hover_control.uav_id = std::stoi(device_id);
            hover_control.method = "takeoff";
            hover_control.height = flight.height;
            json j_hover_control = hover_control;
            if (1 == pad_to_device(plane_ip, drone_cmd_port, j_hover_control.dump())) {
              takeoff_target_distance = flight.height;
            }
            logger.info("pad --> 3588: takeoff, height=" + std::to_string(flight.height));
          }
          if ("gohome" == flight.method) {
            flight_nodeid = flight.nodeId;
            drone->sendRTL();
            logger.info("pad --> 3588: gohome, nodeId=" + std::to_string(flight.nodeId));
          }
        }
      } else if (action_type == "cmd") {
        if ("dog" == dog_or_plane) {
          CmdTmp cmd_tmp;
          j.get_to(cmd_tmp);

          if (cmd_tmp.method == "cmd") {
            if (cmd_tmp.cmd == 122) {
              dog->sendAction("stand_up");
            }
            if (cmd_tmp.cmd == 90) {
              dog->sendAction("emergency_stop");
            }
            if(cmd_tmp.cmd == 100){
              dog->sendCancelNav();
            }
            logger.info("pad --> 3588: cmd=" + std::to_string(cmd_tmp.cmd));
          }
        }
        if ("plane" == dog_or_plane) {
        }
      } else if (action_type == "gimbal") {
        Gimbal gimbal;
        j.get_to(gimbal);
        gimbalController->specifyAngle(gimbal.pitch, gimbal.yaw);
        logger.info("pad --> 3588: gimbal, pitch=" + std::to_string(gimbal.pitch) + " yaw=" + std::to_string(gimbal.yaw));
      } else if (action_type == "follow") {
        if ("plane" == dog_or_plane) {
          // StartDistance(socket_pod);
          j.get_to(follow);
          uint16_t x = follow.x;
          uint16_t y = follow.y;
          // PointTrace(x, y, socket_pod);
          logger.info("pad --> 3588: follow, x=" + std::to_string(x) + " y=" + std::to_string(y));
          followReply.store(true);
        }
      } else if (action_type == "stopfollow") {
        // StopTrace(socket_pod);
        followReply.store(false);
      } else if (action_type == "dot") {
        CmdTmp cmd_tmp;
        j.get_to(cmd_tmp);
        if (cmd_tmp.method == "dot") {
          if (cmd_tmp.cmd == 4) {
            mesh();
            logger.info("pad --> 3588: 自组网切换");
          }
          if (cmd_tmp.cmd == 5) {
            fiveG();
            logger.info("pad --> 3588: 5G切换");
          }
          if (cmd_tmp.cmd == 6) {
            dog->sendMode("sdk");
            logger.info("pad --> 3588: 自动模式(sdk)");
          }
          if (cmd_tmp.cmd == 7) {
            dog->sendMode("remote");
            logger.info("pad --> 3588: 手动模式(remote)");
          }
        }
      }
    }
  }
}

// 监听机器狗心跳与任务状态（基于 DogController 回调）
void listen_device_thread() {
  if (dog_or_plane == "dog") {
    // 注册心跳回调 — 更新全局 dog_state，标记可上报给 Pad
    dog->setDogStateCallback([](const DogState& s) {
      dog_state = s;
      dog_upload_flag.store(true);
    });

    // 注册导航状态回调 — 根据当前任务类型设置对应回执标志
    dog->setNavStateCallback([](const NavState& s) {
      nav_state = s;
      json j_nav = s;
      logger.info("device --> 3588:   " + j_nav.dump());

      // nav_state == 1 表示导航完成，根据当前任务类型设置回执标志
      if (s.nav_state == 1) {
        uint8_t task = currentTaskType.load();
        if (task == 1) {
          flytoFlag.store(true);
          logger.info("[listen_device] dog flyto 任务完成");
        } else if (task == 2) {
          missionFlag.store(true);
          logger.info("[listen_device] dog mission 任务完成");
        }
        currentTaskType.store(0);  // 任务已完成，清除标记
      }
    });

    // 启动 DogController 接收线程（监听 0.0.0.0:8084）
    if (!dog->start()) {
      logger.error("[listen_device] DogController 启动失败");
      return;
    }

    logger.info("[listen_device] 已开始监听机器狗心跳与任务状态");

    // 保持线程存活，直到程序退出
    while (is_live) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    dog->stop();
  }
  if (dog_or_plane == "plane") {
    // 注册无人机心跳回调 — 更新全局 flight_upload，标记可上报给 Pad
    drone->setHeartbeatCallback([](const FlightUpload& hb) {
      flight_upload = hb;
      flight_upload_flag.store(true);
    });

    // 注册无人机回执回调 — 根据 method 设置对应回执标志通知 reply_pad_thread
    drone->setReplyCallback([](const CommonReply& reply) {
      logger.info("drone reply: method=" + reply.method + " status=" + std::to_string(reply.status));
      if (reply.status == 1) {
        if (reply.method == "flyto") {
          flytoFlag.store(true);
          logger.info("[listen_device] drone flyto 任务完成");
        } else if (reply.method == "mission") {
          missionFlag.store(true);
          logger.info("[listen_device] drone mission 任务完成");
        } else if (reply.method == "rtl") {
          flightFLag.store(true);
          logger.info("[listen_device] drone rtl 返航完成");
        }
        currentTaskType.store(0);
      }
    });

    // 启动无人机控制器（UDP 心跳监听 + 回执监听）
    if (!drone->start()) {
      logger.error("[listen_device] DroneController 启动失败");
      return;
    }

    logger.info("[listen_device] 已开始监听无人机心跳与任务回执");

    // 保持线程存活，直到程序退出
    while (is_live) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    drone->stop();
  }
}

void initLogger() {
  try {
    // 创建一个带文件大小限制的 appender
    log4cpp::PatternLayout* fileLayout = new log4cpp::PatternLayout();
    fileLayout->setConversionPattern("%d{%Y-%m-%d-%H-%M-%S.%l} [%p] %m%n");

    // 使用 RollingFileAppender 替代 FileAppender
    // 参数依次为：名称、日志文件路径、最大文件大小(bytes)、备份文件数量
    log4cpp::RollingFileAppender* fileAppender = new log4cpp::RollingFileAppender("fileAppender", "../logs/client.log",
                                                                                  10 * 1024 * 1024,  // 10MB
                                                                                  5);                // 保留5个备份文件
    fileAppender->setLayout(fileLayout);

    // 控制台输出 appender
    log4cpp::PatternLayout* consoleLayout = new log4cpp::PatternLayout();
    consoleLayout->setConversionPattern("%d{%H:%M:%S.%l} [%p] %m%n");
    log4cpp::OstreamAppender* consoleAppender = new log4cpp::OstreamAppender("console", &std::cout);
    consoleAppender->setLayout(consoleLayout);

    // 配置 root category
    logger.setPriority(log4cpp::Priority::DEBUG);
    logger.addAppender(fileAppender);
    logger.addAppender(consoleAppender);

    logger.info("日志系统初始化成功");
  } catch (const std::exception& e) {
    std::cerr << "日志初始化失败: " << e.what() << std::endl;
    logger.error("日志初始化失败");
  }
}

int main() {
  // ======================== 原始 main 代码（保留） ========================
  sleep(2);
  //串口初始化
  //serialInit("g5", "g4");

  initLogger();

  sleep(1);
  //   读取配置文件ip信息
  getIpByYaml("../config/config.yaml");

  // 根据配置文件构造控制器实例
  dog = std::make_unique<DogController>(dog_ip, dog_send_port, "0.0.0.0", dog_listen_port);
  drone = std::make_unique<DroneController>(plane_ip, drone_cmd_port, "0.0.0.0", drone_hb_port, drone_reply_port);
  gimbalController = std::make_unique<GimbalController>(gimbal_ip, gimbal_port);

  // 注册云台反馈回调 — 更新全局 gimbal_feedback，供 heartbeat_thread 上报
  gimbalController->setFeedbackCallback([](const std_feedback_converted_t& fb) {
    gimbal_feedback = fb;
    //logger.debug("gimbal feedback: pitch=%.2f yaw=%.2f roll=%.2f zoom=%.1f dist=%.2f", fb.pitch_angle, fb.yaw_angle, fb.roll_angle, fb.zoom_ratio, fb.laser_distance);
  });

  // 启动云台控制器（TCP 连接 + 接收线程）
  if (!gimbalController->start()) {
    logger.error("[main] GimbalController 启动失败");
  } else {
    logger.info("[main] GimbalController 已启动");
    gimbalController->onekeyCenter();
  }

  std::thread heartbeatThread(heartbeat_thread);
  std::thread listenPadThread(listen_pad_thread);
  std::thread listenDeviceThread(listen_device_thread);
  std::thread replyThread(reply_pad_thread);

  listenPadThread.join();
  heartbeatThread.join();
  listenDeviceThread.join();
  replyThread.join();

  log4cpp::Category::shutdown();
  return 0;
}
