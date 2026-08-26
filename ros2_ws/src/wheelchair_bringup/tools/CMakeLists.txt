cmake_minimum_required(VERSION 3.10)
project(wheelchair_l2_hardware_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(
  UNITREE_LIDAR_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../unilidar_sdk2/unitree_lidar_sdk"
)
set(
  UNITREE_LIDAR_SDK_LIBRARY
  "${UNITREE_LIDAR_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a"
)

if(NOT EXISTS "${UNITREE_LIDAR_SDK_LIBRARY}")
  message(FATAL_ERROR "Unitree SDK library not found: ${UNITREE_LIDAR_SDK_LIBRARY}")
endif()

add_executable(configure_l2_left_mac configure_l2_left_mac.cpp)
target_include_directories(
  configure_l2_left_mac
  PRIVATE "${UNITREE_LIDAR_SDK_ROOT}/include"
)
target_link_libraries(
  configure_l2_left_mac
  PRIVATE "${UNITREE_LIDAR_SDK_LIBRARY}"
)
