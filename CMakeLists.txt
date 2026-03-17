cmake_minimum_required(VERSION 3.31)
project(client)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LOG4CPP REQUIRED log4cpp)

# 云台 SDK 源文件 (C 语言)
set(GIMBAL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk)

# 喊话器 SDK 目录
set(SPEAKER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/speaker_lib)
set(SPEAKER_THIRD_PARTY_LIB_DIR ${SPEAKER_LIB_DIR}/third_party/lib)
set(SPEAKER_THIRD_PARTY_INC_DIR ${SPEAKER_LIB_DIR}/third_party/include)

add_executable(client main.cpp
        include/global_common.h
        SerialManager.cpp
        include/SerialManager.h
        DogController.cpp
        include/DogController.h
        GimbalController.cpp
        include/GimbalController.h
        DroneController.cpp
        include/DroneController.h
        speaker_utils.cpp
        include/speaker_utils.h
        ${GIMBAL_SDK_DIR}/std_protocol_sdk.c
)

# 添加包含目录
target_include_directories(client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LOG4CPP_INCLUDE_DIRS}
    ${GIMBAL_SDK_DIR}
    ${SPEAKER_LIB_DIR}/speaker_interface/include
    ${SPEAKER_LIB_DIR}/mavsdk/include
    ${SPEAKER_LIB_DIR}/mavsdk/include/mavsdk
    ${SPEAKER_LIB_DIR}/mavftp_pro/include
    ${SPEAKER_THIRD_PARTY_INC_DIR}
)

# 查找喊话器相关库
find_library(SPEAKER_INTERFACE_LIB speaker_interface PATHS ${SPEAKER_LIB_DIR}/speaker_interface/lib REQUIRED)
find_library(MAVSDK_LIB mavsdk PATHS ${SPEAKER_LIB_DIR}/mavsdk/lib REQUIRED)
find_library(MAVFTP_LIB mavftp_pro PATHS ${SPEAKER_LIB_DIR}/mavftp_pro/lib ${SPEAKER_LIB_DIR}/mav_ftp_pro/lib REQUIRED)
find_library(JSONCPP_LIB jsoncpp PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(CURL_LIB curl PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(PAHO_MQTT_LIB paho-mqttpp3 PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(PAHO_MQTTC_LIB paho-mqtt3a PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(TINYXML_LIB tinyxml2 PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(LZMA_LIB lzma PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})
find_library(Z_LIB z PATHS ${SPEAKER_THIRD_PARTY_LIB_DIR})

target_link_libraries(client
        PRIVATE
        yaml-cpp::yaml-cpp
        nlohmann_json::nlohmann_json
        Threads::Threads
        zmq
        ${LOG4CPP_LIBRARIES}
        ${SPEAKER_INTERFACE_LIB}
        ${MAVFTP_LIB}
        ${MAVSDK_LIB}
        ${JSONCPP_LIB}
        ${CURL_LIB}
        ${PAHO_MQTT_LIB}
        ${PAHO_MQTTC_LIB}
        ${TINYXML_LIB}
        ${LZMA_LIB}
        ${Z_LIB}
)

# 添加编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(client PRIVATE DEBUG)
endif()

# Ubuntu: 解析符号到函数/文件/行号
if(NOT WIN32)
    target_compile_options(client PRIVATE -g -O0 -fno-omit-frame-pointer)
    target_link_options(client PRIVATE -rdynamic)
endif()
