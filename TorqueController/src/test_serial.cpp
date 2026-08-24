// test_serial.cpp
#include "communication/Communications.hpp"
#include <iostream>
#include <thread>
#include <csignal>
#include <iomanip>

std::atomic<bool> keep_running{true};

void signalHandler(int signum) {
    keep_running = false;
}

void onReceive(const mcu::ReceivePacket& packet) {
    std::cout << "\n[Received Packet]" << std::endl;
    std::cout << "  frame_header1:      0x" << std::hex << std::uppercase 
              << static_cast<int>(packet.frame_header1) << std::dec << std::endl;
    std::cout << "  frame_header2:      0x" << std::hex << std::uppercase 
              << static_cast<int>(packet.frame_header2) << std::dec << std::endl;
    std::cout << "  protocol_version:   " << static_cast<int>(packet.protocol_version) << std::endl;
    std::cout << "  data_size:          " << static_cast<int>(packet.data_size) << std::endl;
    std::cout << "  bullet_velocity:    " << std::fixed << std::setprecision(2) 
              << packet.bullet_velocity << " m/s" << std::endl;
    std::cout << "  pitch_angle:        " << packet.pitch_angle << std::endl;
    std::cout << "  yaw_angle:          " << packet.yaw_angle << std::endl;
    std::cout << "  yaw_omega:          " << packet.yaw_omega << " rad/s" << std::endl;
    std::cout << "  chassis_imu_yaw:    " << packet.chassis_imu_yaw << std::endl;
    std::cout << "  chassis_imu_omega:  " << packet.chassis_imu_omega << " rad/s" << std::endl;
    std::cout << "  mark:               " << static_cast<int>(packet.mark) << std::endl;
    std::cout << "  color:              " << static_cast<int>(packet.color) << std::endl;
    std::cout << "  auto_aim_switch:    " << static_cast<int>(packet.auto_aim_switch) << std::endl;
    std::cout << "  yaw_temperature:    " << static_cast<int>(packet.yaw_temperature) << " C" << std::endl;
    std::cout << "  crc8:               0x" << std::hex << std::uppercase 
              << static_cast<int>(packet.crc8) << std::dec << std::endl;
    std::cout << std::endl;
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Starting Serial Communication Test..." << std::endl;
    std::cout << "SendPacket size: " << sizeof(mcu::SendPacket) << " bytes" << std::endl;
    std::cout << "ReceivePacket size: " << sizeof(mcu::ReceivePacket) << " bytes" << std::endl;

    McuCommunication serial(onReceive);

    std::cout << "Sending packets... (false, 0, 0, false)" << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;

    int send_count = 0;
    while (keep_running) {
        mcu::SendPacket packet;
        // 默认初始化的frame_header1, frame_header2, protocol_version, data_size 已自动设置
        packet.auto_aim_enable = 1;       // false
        packet.fire = 0;                  // false
        packet.pitch_target_angle = 10.0f; // 0
        packet.yaw_torque_only_mode = 1;
        packet.yaw_target_angle = 0.0;
        packet.yaw_target_velocity = 0.0f;
        packet.yaw_torque = 0.0f;         // 0

        if (serial.sendData(packet)) {
            send_count++;
            if (send_count % 100 == 0) {
                std::cout << "Sent " << send_count << " packets..." << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nExiting... Total packets sent: " << send_count << std::endl;

    return 0;
}
