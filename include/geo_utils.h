#pragma once

#include <cmath>
#include <limits>

constexpr double EARTH_RADIUS_KM = 6371.004;  // 地球平均半径（千米）

// 角度转弧度
inline double deg2rad(double deg) {
  constexpr double PI = 3.14159265358979323846;
  return deg * (PI / 180.0);
}

// 计算两点球面距离（千米），Haversine 公式
inline double distance_haversine_km(double lat1, double lon1, double lat2, double lon2) {
  double dlat = deg2rad(lat2 - lat1);
  double dlon = deg2rad(lon2 - lon1);
  double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) + std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  if (a > 1.0) a = 1.0;  // 数值稳定性保护
  double c = 2.0 * std::asin(std::sqrt(a));
  return EARTH_RADIUS_KM * c;
}

// 计算两点球面距离（米），基于 Haversine 公式
inline double distance_haversine_m(double lat1, double lon1, double lat2, double lon2) { return distance_haversine_km(lat1, lon1, lat2, lon2) * 1000.0; }

// 根据 GPS 坐标计算两点方向角（真东基准，弧度），用于航点 yaw 计算
inline double calculateRotationFromGPS(double currentLat, double currentLon, double targetLat, double targetLon) {
  // 重合检测
  if (currentLat == targetLat && currentLon == targetLon) {
    return std::nan("");
  }

  // 经纬度转弧度
  double lat1 = currentLat * M_PI / 180.0;
  double lon1 = currentLon * M_PI / 180.0;
  double lat2 = targetLat * M_PI / 180.0;
  double lon2 = targetLon * M_PI / 180.0;

  // 计算目标方向角（真北基准）
  double dLon = lon2 - lon1;
  double y = std::sin(dLon) * std::cos(lat2);
  double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
  double targetYaw = std::atan2(y, x);  // [-π, π]

  // 转换为真东基准：真北基准顺时针旋转90度
  targetYaw = targetYaw - M_PI / 2.0;

  // 规范化到[-π, π]范围
  if (targetYaw > M_PI)
    targetYaw -= 2 * M_PI;
  else if (targetYaw < -M_PI)
    targetYaw += 2 * M_PI;

  // 返回绝对角度（真东基准），表示转到多少度而不是旋转多少度
  return targetYaw;
}
