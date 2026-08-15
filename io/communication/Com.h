// Com.h
#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <queue>
#include <array>
#include <mutex>
#include <atomic>
#include <chrono>
#include "communication/CRC.h"
#include <dirent.h>  // 用于遍历/dev目录
#include <sys/types.h>
#include <sys/stat.h>
#define _USE_MATH_DEFINES // 启用数学常量
#include <cmath>
#include <functional>
#include <iostream>
#include <libudev.h>
#include <thread>

struct MCUDataFrame {
    float bullet_velocity;
    float bullet_angle;
    int16_t gimbal_yaw;
    uint16_t mark;
    uint8_t color;
    float z_rotation_velocity;
};

struct SerialData {
    MCUDataFrame origin_data_frame;
    float bullet_velocity;  // 子弹速度
    float bullet_angle;    // 子弹角度
    int16_t gimbal_yaw;       // 云台当前偏航角
    uint8_t color;            // 敌方颜色(0:红色, 1:蓝色)
};

class SerialCommunicationClass {
public:
    // 去掉 ROS2：原来 node 只用于打日志，改用 tools::logger()
    SerialCommunicationClass(std::function<void(const SerialData&)> serialDataCallback);
    ~SerialCommunicationClass();
    void stop();   // 停止通信线程（running=false），避免析构时手动调 ~ 造成双重析构
    void timerCallback();
    bool sendData(float pitch_target, float yaw_target, bool fire = true);
    void timerThread();
    
private:
    static constexpr size_t BUFFER_SIZE = 1024;
    static constexpr uint8_t FRAME_HEADER1 = 0x42;
    static constexpr uint8_t FRAME_HEADER2 = 0x52;
    static constexpr uint8_t COMMAND_CODE = 0xCD;
    static constexpr size_t FRAME_MIN_SIZE = 5;
    // std::mutex queue_mutex_;
    // static constexpr size_t MAX_QUEUE_SIZE = 1;

    int fd_;
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    size_t buffer_index_ = 0;
    // std::atomic<int> received_commands_count_{0};
    // std::atomic<int> sent_commands_count_{0};

    std::function<void(const SerialData&)> serialDataCallback;
    bool running = true;

    std::chrono::steady_clock::time_point last_reconnect_time;
    std::chrono::steady_clock::time_point last_received_time;
    
    void initializeSerial();
    std::vector<std::string> findAvailableSerialPorts();
    void processFrame(const uint8_t* data);
    void processBuffer();
    void tryReconnect();
    std::string getSerialProductInfo(const std::string& port);
};

#endif // COM_H
