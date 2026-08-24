cmake_minimum_required(VERSION 3.10)
project(TorqueController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 查找udev库
find_library(UDEV_LIBRARY udev)
if(NOT UDEV_LIBRARY)
    message(FATAL_ERROR "libudev not found")
endif()

# ============================================================================
# 通信库（src/communication + include/communication）
# ============================================================================
set(COMMUNICATION_SOURCES
    src/communication/CRC.cpp
    src/communication/McuDataPreprocessor.cpp
    src/communication/FusionFilter.cpp
)

# 构建通信库
add_library(communication STATIC ${COMMUNICATION_SOURCES})
target_include_directories(communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication
)
target_link_libraries(communication PUBLIC ${UDEV_LIBRARY})

# 构建测试可执行文件
add_executable(test_serial src/test_serial.cpp)
target_link_libraries(test_serial PRIVATE communication)

# 构建 Pitch 轴标定程序
add_executable(pitch_calibration src/pitch_calibration.cpp)
target_link_libraries(pitch_calibration PRIVATE communication)

# ============================================================================
# MPC（src/mpc + include/mpc）
# ============================================================================
set(MPC_SOURCES
    src/mpc/mpc_controller.cpp
    src/mpc/yaw_mpc_controller.cpp
    src/mpc/mcu_mpc_controller.cpp
)

# ============================================================================
# C API + 一体化封装动态库（c_api/ + src/RobotController.cpp；供 Python 与外部 cpp）
# ============================================================================
find_package(Ceres REQUIRED)

add_library(robot_comm_c SHARED
    src/c_api/RobotCommunicationC.cpp
    ${COMMUNICATION_SOURCES}
    ${MPC_SOURCES}
    src/RobotController.cpp
)
target_include_directories(robot_comm_c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mpc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/c_api
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(robot_comm_c PUBLIC ${UDEV_LIBRARY} ${CERES_LIBRARIES})

# 构建控制演示程序（使用 RobotController 一体化封装）
add_executable(control_demo src/control_demo.cpp)
target_link_libraries(control_demo PRIVATE robot_comm_c)
