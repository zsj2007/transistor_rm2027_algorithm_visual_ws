// SerialProtocol.hpp — 模板化串口协议通信类
// 模板参数:
//   SendPacketT     : 发送包类型（完整 wire-format 结构体，含前导/长度/CRC）
//   ReceivePacketT  : 接收包类型（完整 wire-format 结构体，含前导/长度/CRC）
//   CRC_Func        : CRC 计算函数，签名 CRCType(const uint8_t*, size_t)
//   PortSelector    : 端口筛选函数，签名 bool(const std::string& product_info)
//   PreambleLen     : 帧同步前导字节数（前导字节值取自 ReceivePacketT 前 N 字节）
//
// CRC 返回值类型和字节数从 CRC_Func 自动推导。
// 帧格式: [前导(PreambleLen 字节)] [data_size(1B)] [payload] [CRC(CRC_SIZE)]
//
#ifndef SERIALPROTOCOL_HPP
#define SERIALPROTOCOL_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <array>
#include <mutex>
#include <atomic>
#include <chrono>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <functional>
#include <iostream>
#include <string>
#include <libudev.h>
#include <thread>
#include <algorithm>
#include <type_traits>

// 从函数指针类型提取返回值类型
template <typename T>
struct FunctionTraits;

template <typename R, typename... Args>
struct FunctionTraits<R(*)(Args...)> {
    using return_type = R;
};

template <
    typename SendPacketT,
    typename ReceivePacketT,
    auto CRC_Func,
    auto PortSelector,
    size_t PreambleLen
>
class SerialProtocol {
public:
    // ── 从 CRC 函数自动推导 ──
    using CRCType = typename FunctionTraits<decltype(CRC_Func)>::return_type;
    static constexpr size_t CRC_SIZE       = sizeof(CRCType);
    static constexpr size_t PREAMBLE_SIZE  = PreambleLen;

    // data_size 字段在前导字节之后
    static constexpr size_t DATA_SIZE_OFFSET = PREAMBLE_SIZE;

    // 帧最小长度 = 前导 + data_size 字段 + CRC
    static constexpr size_t FRAME_MIN_SIZE  = PREAMBLE_SIZE + 1 + CRC_SIZE;

    SerialProtocol(std::function<void(const ReceivePacketT&)> callback, bool auto_start = true);
    ~SerialProtocol();
    void startWorker();
    void stopWorker();
    bool sendData(SendPacketT& packet);

private:
    static constexpr size_t BUFFER_SIZE     = 1024;
    static constexpr size_t MAX_FRAME_LENGTH = 256;

    // 前导序列：构造时从默认 ReceivePacketT 的前 PREAMBLE_SIZE 字节提取并保存
    std::array<uint8_t, PREAMBLE_SIZE> preamble_bytes_;

    int fd_;
    std::mutex fd_mutex_;
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    size_t buffer_index_ = 0;

    std::function<void(const ReceivePacketT&)> serialDataCallback;
    std::atomic<bool> running{true};
    std::thread recv_thread_;

    std::chrono::steady_clock::time_point last_reconnect_time;
    std::chrono::steady_clock::time_point last_received_time;

    std::string getSerialProductInfo(const std::string& port);
    std::vector<std::string> findAvailableSerialPorts();
    void initializeSerial();
    void processFrame(const uint8_t* data, size_t frame_length);
    void processBuffer();
    void tryReconnect();
    void timerCallback();
    void timerThread();
};

// ============================================================================
// 模板成员函数实现
// ============================================================================

template <typename S, typename R, auto C, auto P, size_t L>
SerialProtocol<S, R, C, P, L>::SerialProtocol(std::function<void(const R&)> callback, bool auto_start)
    : serialDataCallback(callback), fd_(-1)
{
    // 从默认 ReceivePacketT 提取前导序列保存到成员变量
    const R dflt{};
    const uint8_t* src = reinterpret_cast<const uint8_t*>(&dflt);
    std::copy_n(src, PREAMBLE_SIZE, preamble_bytes_.begin());

    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
    if (auto_start) {
        startWorker();
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
SerialProtocol<S, R, C, P, L>::~SerialProtocol() {
    stopWorker();
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::startWorker() {
    if (recv_thread_.joinable()) {
        running = false;
        recv_thread_.join();
    }
    running = true;
    recv_thread_ = std::thread(&SerialProtocol::timerThread, this);
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::stopWorker() {
    running = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
bool SerialProtocol<S, R, C, P, L>::sendData(S& packet) {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0) {
        return false;
    }

    CRCType crc = C(reinterpret_cast<const uint8_t*>(&packet), sizeof(S) - CRC_SIZE);
    memcpy(reinterpret_cast<uint8_t*>(&packet) + sizeof(S) - CRC_SIZE, &crc, CRC_SIZE);

    ssize_t written = write(fd_, &packet, sizeof(S));
    if (written == static_cast<ssize_t>(sizeof(S))) {
        return true;
    } else {
        printf("TX write failed: written %ld bytes, expected %zu\n", written, sizeof(S));
        return false;
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
std::string SerialProtocol<S, R, C, P, L>::getSerialProductInfo(const std::string& port) {
    struct udev *udev;
    struct udev_device *dev;
    std::string result = "";
    udev = udev_new();
    if (!udev) return "Failed to create udev";
    dev = udev_device_new_from_subsystem_sysname(udev, "tty", port.c_str());
    if (!dev) { udev_unref(udev); return "Device not found"; }
    struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (parent) {
        const char *product = udev_device_get_sysattr_value(parent, "product");
        if (product) result += std::string(product);
    }
    udev_device_unref(dev);
    udev_unref(udev);
    return result;
}

template <typename S, typename R, auto C, auto P, size_t L>
std::vector<std::string> SerialProtocol<S, R, C, P, L>::findAvailableSerialPorts() {
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (dp == nullptr) { printf("Failed to open /dev/ directory\n"); return {}; }
    std::vector<std::string> ports;
    while ((entry = readdir(dp)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
            std::string candidate_port = "/dev/" + std::string(entry->d_name);
            int fd = open(candidate_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd >= 0) { close(fd); ports.push_back(candidate_port); }
        }
    }
    closedir(dp);
    return ports;
}
template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::initializeSerial() {
    auto ports = findAvailableSerialPorts();
    if (ports.empty()) { printf("No available serial port found!\n"); return; }
    std::string port;
    for (auto test_port : ports) {
        try {
            if (P(getSerialProductInfo(test_port.substr(5)))) { port = test_port; break; }
        } catch (...) {}
    }
    if (port.empty()) { printf("Target serial port not found!\n"); return; }
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) { printf("Failed to open port %s: %s\n", port.c_str(), strerror(errno)); return; }
    struct termios tty; memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) { printf("tcgetattr failed\n"); close(fd_); fd_ = -1; return; }
    cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD); tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON; tty.c_lflag &= ~ECHO; tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { printf("tcsetattr failed\n"); close(fd_); fd_ = -1; return; }
    tcflush(fd_, TCIOFLUSH);
    printf("Serial initialized: %s\n", port.c_str());
}
template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::processFrame(const uint8_t* data, size_t frame_length) {
    (void)frame_length;
    R packet{};
    memcpy(&packet, data, std::min(frame_length, sizeof(R)));
    serialDataCallback(packet);
    last_received_time = std::chrono::steady_clock::now();
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::processBuffer() {
    static const size_t MAX_FRAMES_PER_LOOP = 10;
    size_t frames_processed = 0;
    while (buffer_index_ >= FRAME_MIN_SIZE && frames_processed < MAX_FRAMES_PER_LOOP) {
        if (buffer_index_ >= BUFFER_SIZE - 128) { printf("Buffer near full, clearing\n"); buffer_index_ = 0; return; }
        size_t header_pos = 0; bool found_header = false;
        while (header_pos + PREAMBLE_SIZE <= buffer_index_ && header_pos < 128) {
            bool match = true;
            for (size_t k = 0; k < PREAMBLE_SIZE; ++k)
                if (buffer_[header_pos + k] != preamble_bytes_[k]) { match = false; break; }
            if (match) { found_header = true; break; }
            header_pos++;
        }
        if (!found_header) {
            if (buffer_index_ > PREAMBLE_SIZE - 1) {
                size_t keep = PREAMBLE_SIZE - 1;
                for (size_t i = 0; i < keep; ++i) buffer_[i] = buffer_[buffer_index_ - keep + i];
                buffer_index_ = keep;
            }
            return;
        }
        if (header_pos > 0) {
            memmove(buffer_.data(), buffer_.data() + header_pos, buffer_index_ - header_pos);
            buffer_index_ -= header_pos;
        }
        if (buffer_index_ < PREAMBLE_SIZE + 1) return;
        uint8_t data_length = buffer_[DATA_SIZE_OFFSET];
        size_t frame_length = PREAMBLE_SIZE + 1 + data_length + CRC_SIZE;
        if (data_length > MAX_FRAME_LENGTH || frame_length > BUFFER_SIZE) { buffer_index_ = 0; return; }
        if (buffer_index_ < frame_length) return;
        CRCType computed_crc = C(buffer_.data(), frame_length - CRC_SIZE);
        CRCType received_crc;
        memcpy(&received_crc, buffer_.data() + frame_length - CRC_SIZE, CRC_SIZE);
        if (computed_crc == received_crc) {
            processFrame(buffer_.data(), frame_length); frames_processed++;
        } else {
            printf("CRC check failed, discarding frame\n");
            memmove(buffer_.data(), buffer_.data() + PREAMBLE_SIZE, buffer_index_ - PREAMBLE_SIZE);
            buffer_index_ -= PREAMBLE_SIZE; continue;
        }
        if (frame_length < buffer_index_) {
            memmove(buffer_.data(), buffer_.data() + frame_length, buffer_index_ - frame_length);
            buffer_index_ -= frame_length;
        } else { buffer_index_ = 0; }
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::tryReconnect() {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) close(fd_);
    buffer_index_ = 0;
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::timerCallback() {
    auto now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
        if (now - last_reconnect_time > std::chrono::seconds(3)) {
            printf("Serial port not available, trying reconnect\n"); tryReconnect();
        }
        return;
    }
    if (now - last_received_time > std::chrono::seconds(3)) {
        if (now - last_reconnect_time > std::chrono::seconds(3)) {
            printf("No data received, trying reconnect\n"); tryReconnect();
        }
        return;
    }
    if (buffer_index_ < BUFFER_SIZE - 128) {
        uint8_t temp_buffer[128];
        ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));
        if (bytes_read > 0) {
            if (buffer_index_ + bytes_read < BUFFER_SIZE) {
                memcpy(buffer_.data() + buffer_index_, temp_buffer, bytes_read);
                buffer_index_ += bytes_read;
                processBuffer();
            } else { printf("Buffer near full, discarding data\n"); buffer_index_ = 0; }
        }
    }
}

template <typename S, typename R, auto C, auto P, size_t L>
void SerialProtocol<S, R, C, P, L>::timerThread() {
    while (running) {
        auto start = std::chrono::steady_clock::now();
        timerCallback();
        std::this_thread::sleep_until(start + std::chrono::microseconds(100));
    }
}

#endif // SERIALPROTOCOL_HPP