cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_try)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks)

####################users###################
add_executable(infantry users/infantry.cpp)
target_link_libraries(infantry ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io tasks)

add_executable(infantry_debug users/infantry_debug.cpp)
target_link_libraries(infantry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io tasks)

################## TorqueController 手动测试（command_channel=torque 联调用）##################
add_executable(torque_manual_test users/torque_manual_test.cpp)
target_include_directories(torque_manual_test PRIVATE
  $ENV{HOME}/TorqueController/include
  $ENV{HOME}/TorqueController/include/communication
  $ENV{HOME}/TorqueController/include/mpc
)
target_link_libraries(torque_manual_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

################## 独立可视化进程（共享内存桥，无 ROS）##################
add_executable(visualizer users/visualizer.cpp io/shm/VisualizerShm.cpp)
target_include_directories(visualizer PRIVATE io)
target_link_libraries(visualizer ${OpenCV_LIBS} yaml-cpp pthread rt)
