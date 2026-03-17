#ifndef GLOBAL_COMMON_H
#define GLOBAL_COMMON_H
#include <ctime>
#include <string>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

struct DogState {
  std::string type;
  uint8_t power;
  std::string sn;
  float speed;
  float angle_speed;
  double lon;
  double lat;
  double alt;
  double yaw;
  double pitch;
  double roll;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(DogState, type, power, sn, speed, angle_speed, lon, lat, alt, yaw, pitch, roll);
};

struct NavState {
  std::string type;
  uint8_t nav_state;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(NavState, type, nav_state);
};

struct Location {  // 附表1：位置结构体
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Location, longitude, latitude, altitude);
};

struct Battery {
  float capacity_percent = 0.0;  // 电量百分比
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Battery, capacity_percent);
};

struct Angle {
  double pitch = 0.0;         // 无人装备 俯仰角度
  double roll = 0.0;          // 无人装备 横滚角度
  double head = 0.0;          // 无人装备 偏航角度
  double gimbal_pitch = 0.0;  // 无人装备云台 俯仰角度
  double gimbal_roll = 0.0;   // 无人装备云台 横滚角度
  double gimbal_head = 0.0;   // 无人装备云台 偏航角度
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Angle, pitch, roll, head, gimbal_pitch, gimbal_roll, gimbal_head);
};

struct Upload {
  std::string sn = "";  // 无人机装备序列号
  Battery battery;
  std::string signal = "弱";  // 信号质量 强 中 弱
  Location location;
  Angle angel;
  double speed;
  uint8_t rtkMode;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Upload, sn, battery, signal, location, angel, rtkMode);
};

//-------------------------------------------------Pad下发及回执------------------------------------------------------------------------------

struct Flyto {
  int8_t nodeId;
  std::string method;
  int8_t type;
  Location location;
  double speed;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Flyto, nodeId, method, type, location, speed, timestamp);
};

struct FlytoReply {
  int8_t nodeId;
  std::string method;
  int8_t type;
  int8_t status;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(FlytoReply, nodeId, method, type, status, timestamp);
};

struct Mission {
  int8_t nodeId;
  std::string method;
  int8_t type;
  int8_t target_type;
  std::vector<Location> location;
  double speed;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Mission, nodeId, method, type, target_type, location, speed, timestamp);
};

struct MissionReply {
  int8_t nodeId;
  std::string method;
  int8_t type;
  int8_t status;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(MissionReply, nodeId, method, type, status, timestamp);
};

struct Flight {
  int8_t nodeId;
  std::string method;
  double speed;
  double height;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Flight, nodeId, method, speed, height, timestamp);
};

struct FlightReply {
  int8_t nodeId;
  std::string method;
  int8_t status;
  int64_t timestamp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(FlightReply, nodeId, method, status, timestamp);
};

struct Follow {
  int8_t nodeId;
  std::string method;
  uint8_t x;
  uint8_t y;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Follow, nodeId, method, x, y);
};

struct FollowReply {
  int8_t nodeId;
  std::string method;
  double longitude;
  double latitude;
  double altitude;
  int8_t status;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(FollowReply, nodeId, method, longitude, latitude, altitude, status);
};

struct CmdTmp {
  std::string method;
  uint8_t cmd;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(CmdTmp, method, cmd);
};

struct Gimbal {
  std::string method;
  int64_t flag;
  double yaw;
  double yspeed;
  double pitch;
  double pspeed;
  double roll;
  double rspeed;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Gimbal, method, flag, yaw, pitch, roll);
};

//-----------------------------------------------------------------无人机--------------------------------------------------------------

struct TargetLocation {
  double lon;
  double lat;
  double relative_alt;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(TargetLocation, lon, lat, relative_alt);
};

// 起飞悬停
struct HoverControl {
  uint8_t uav_id;
  std::string method;
  double height;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(HoverControl, uav_id, method, height);
};

// 降落
struct LandControl {
  uint8_t uav_id;
  std::string method;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(LandControl, uav_id, method);
};

// 无人机上报状态
struct FlightUpload {
  uint8_t uav_id;
  uint8_t type;
  double lon;
  double lat;
  double alt;
  double relative_alt;
  double pitch;
  double roll;
  double yaw;
  double speed;
  uint8_t state;
  double battery_remaining;
  double battery_voltage;
  std::string method;
  uint8_t gnss_fix_type;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(FlightUpload, uav_id, type, lon, lat, alt, relative_alt, pitch, roll, yaw, speed, state, battery_remaining, battery_voltage, method,
                                 gnss_fix_type);
};

struct CommonReply {
  uint8_t uav_id;
  std::string method;
  uint8_t status;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(CommonReply, uav_id, method, status);
};

//--------------------------------------------------------------吊舱-----------------------------------------------------------------------------

struct PodReturnPad {
  double target_lat;
  double target_lon;
  double target_high;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PodReturnPad, target_lat, target_lon, target_high);
};

#endif  // GLOBAL_COMMON_H
