cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_library(io STATIC
    camera.cpp
    camera/cameraConnect.cpp
    other_input/VideoInput.cpp
    other_input/ImagesInput.cpp
    communication/Com.cpp
    communication/CRC.cpp
    communication/HeadIMU.cpp
    communication/WatchdogClient.cpp
    shm/SharedMemoryClassifier.cpp
    communication.cpp
    watchdog.cpp
)

# 保留 "communication/xxx.h" 这类原项目 include 写法
target_include_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# 相机（transistor GigE/USB，海康 SDK）
target_include_directories(io PUBLIC camera)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC camera/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC camera/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# 海康 SDK 用完整路径链接，避免 -L 传播问题
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/camera/lib/amd64/libMvCameraControl.so yaml-cpp udev pthread)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/camera/lib/arm64/libMvCameraControl.so yaml-cpp udev pthread)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

################## 检查 ROS 环境（可选 ROS 桥） ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 bridge.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2 bridge.")
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
