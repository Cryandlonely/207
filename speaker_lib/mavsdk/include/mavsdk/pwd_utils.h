
#pragma once
#include <cstdint>
#include <string>
#include <cstring>
namespace mavsdk {

inline uint32_t crc32(const std::string& data) {
    uint32_t crc = 0xFFFFFFFF; // 每次都从标准的初始值开始
    for (char c : data) {
        uint8_t byte = static_cast<uint8_t>(c);
        crc = crc ^ byte;
        for (int j = 0; j < 8; ++j) {
            uint32_t mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }
    return ~crc; // 返回最终值
}

union PwdWrapper {
    float f;
    uint32_t u;
};
}