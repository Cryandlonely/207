include(CMakeFindDependencyMacro)

if(NOT OFF)
    find_dependency(Threads)
    find_dependency(PahoMqttCpp)
endif()

get_filename_component(MAVSDK_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include("${MAVSDK_CMAKE_DIR}/MAVFTP_ProTargets.cmake")
