#include "communication/HeadIMU.h"

std::string HeadIMUSerialCommunicationClass::getSerialProductInfo(const std::string& port) {
    struct udev *udev;
    struct udev_device *dev;
    std::string result = "";
    
    // 创建udev对象
    udev = udev_new();
    if (!udev) {
        return "Failed to create udev";
    }
    
    // 根据设备路径获取设备信息
    dev = udev_device_new_from_subsystem_sysname(udev, "tty", port.c_str());
    if (!dev) {
        udev_unref(udev);
        return "Device not found";
    }
    
    // 获取父设备（USB设备）
    struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
        dev, "usb", "usb_device");
    
    if (parent) {
        // 获取产品信息
        const char *product = udev_device_get_sysattr_value(parent, "product");
        
        if (product) {
            result += std::string(product);
        }
    }
    
    udev_device_unref(dev);
    udev_unref(udev);
    return result;
}

HeadIMUSerialCommunicationClass::HeadIMUSerialCommunicationClass(std::function<void(const HeadIMUSerialData&)> serialDataCallback) 
: serialDataCallback(serialDataCallback), fd_(-1) {
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}

HeadIMUSerialCommunicationClass::~HeadIMUSerialCommunicationClass() {
    running = false;
    if (fd_ >= 0) {
        close(fd_);
    }
}

void HeadIMUSerialCommunicationClass::stop() {
    running = false;
    if (fd_ >= 0) {
        close(fd_);
    }
}

void HeadIMUSerialCommunicationClass::tryReconnect() {
    if (fd_ >= 0) {
        close(fd_);
    }
    buffer_index_ = 0;
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}
    
void HeadIMUSerialCommunicationClass::initializeSerial() {
    std::vector<std::string> ports = findAvailableSerialPorts();
    if (ports.empty()) {
        printf("No available serial port found!\n");
        return;
    }
    std::string port;
    for (auto test_port : ports) {
        try {
            if(getSerialProductInfo(test_port.substr(5)) == std::string("AutoAim_IMU_Com")) {
                port = test_port;
                break;
            };
        } catch (...) {

        }
    }
    if (port.empty()) {
        printf("Target serial port Not found!\n");
        return;
    }

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        printf("Failed to open port %s: %s\n", port.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        printf("Failed to get serial attributes\n");
        close(fd_);
        fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        printf("Failed to set serial attributes\n");
        close(fd_);
        fd_ = -1;
        return;
    }

    tcflush(fd_, TCIOFLUSH);
    printf("Serial initialized: %s\n", port.c_str());
}

// 查找可用的串口
std::vector<std::string> HeadIMUSerialCommunicationClass::findAvailableSerialPorts() {
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (dp == nullptr) {
        printf("Failed to open /dev/ directory\n");
        return std::vector<std::string>(0);
    }

    std::vector<std::string> ports;
    while ((entry = readdir(dp)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {  // 匹配ttyACM串口
            std::string candidate_port = "/dev/" + std::string(entry->d_name);
            int fd = open(candidate_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd >= 0) {
                close(fd);  // 串口可用，返回串口名称
                ports.push_back(candidate_port);
                // break;
            }
        }
    }

    closedir(dp);
    return ports;
}

void HeadIMUSerialCommunicationClass::processFrame(const uint8_t* data) {
    DataFrame frame{};

    memcpy(&frame.header1, &data[0], sizeof(uint8_t));
    memcpy(&frame.header2, &data[1], sizeof(uint8_t));
    memcpy(&frame.header3, &data[2], sizeof(uint8_t));
    memcpy(&frame.data_len, &data[3], sizeof(uint8_t));
    memcpy(&frame.gx, &data[4], sizeof(float));
    memcpy(&frame.gy, &data[8], sizeof(float));
    memcpy(&frame.gz, &data[12], sizeof(float));
    memcpy(&frame.ax, &data[16], sizeof(float));
    memcpy(&frame.ay, &data[20], sizeof(float));
    memcpy(&frame.az, &data[24], sizeof(float));
    memcpy(&frame.euler_yaw, &data[28], sizeof(double));
    memcpy(&frame.euler_pitch, &data[36], sizeof(double));
    memcpy(&frame.euler_roll, &data[44], sizeof(double));
    memcpy(&frame.dt_one_tenth_ms, &data[52], sizeof(uint32_t));
    memcpy(&frame.crc32, &data[56], sizeof(uint32_t));

    HeadIMUSerialData msg;
    msg.gx = frame.gx;
    msg.gy = frame.gy;
    msg.gz = frame.gz;
    msg.ax = frame.ax;
    msg.ay = frame.ay;
    msg.az = frame.az;
    msg.euler_yaw = frame.euler_yaw;
    msg.euler_pitch = frame.euler_pitch;
    msg.euler_roll = frame.euler_roll;
    msg.dt_one_tenth_ms = frame.dt_one_tenth_ms;
    
    serialDataCallback(msg);

    last_received_time = std::chrono::steady_clock::now();
}

void HeadIMUSerialCommunicationClass::processBuffer() {
    
    // 每次处理最多处理10个帧，防止处理过多数据导致阻塞
    static const size_t MAX_FRAMES_PER_LOOP = 10;
    size_t frames_processed = 0;

    while (buffer_index_ >= FRAME_MIN_SIZE && frames_processed < MAX_FRAMES_PER_LOOP) {
        // 安全检查：如果缓冲区接近满，立即清空
        if (buffer_index_ >= BUFFER_SIZE - 128) {
            printf("Buffer approaching capacity (%zu bytes), clearing\n", buffer_index_);
            buffer_index_ = 0;
            return;
        }

        // 查找帧头
        size_t header_pos = 0;
        bool found_header = false;
        
        // 只在合理范围内查找帧头
        while (header_pos <= buffer_index_ - 3 && header_pos < 128) {
            if (buffer_[header_pos] == FRAME_HEADER1 && 
                buffer_[header_pos + 1] == FRAME_HEADER2 && 
                buffer_[header_pos + 2] == FRAME_HEADER3) {
                found_header = true;
                break;
            }
            ++header_pos;
        }

        if (!found_header) {
            // 如果找不到帧头，清空缓冲区
            buffer_index_ = 0;
            return;
        }

        // 如果帧头前有无效数据，移除它们
        if (header_pos > 0) {
            if (header_pos < buffer_index_) {
                memmove(buffer_.data(), buffer_.data() + header_pos, buffer_index_ - header_pos);
                buffer_index_ -= header_pos;
            } else {
                buffer_index_ = 0;
                return;
            }
        }

        // 检查是否有完整的帧
        if (buffer_index_ < 4) {
            return;  // 等待更多数据
        }

        uint8_t data_length = buffer_[3];
        size_t frame_length = data_length + 4 + 4;

        // 验证帧长度的合理性
        if (data_length > 64 || frame_length > BUFFER_SIZE) {  // 假设最大帧长度为64字节
            printf("Invalid frame length detected: %zu\n", frame_length);
            buffer_index_ = 0;
            return;
        }

        if (buffer_index_ < frame_length) {
            return;  // 等待完整帧
        }

        // CRC校验
        uint32_t computed_crc32 = HAL_CRC_Calculate(buffer_.data(), frame_length-4);
        uint32_t received_crc32;
        memcpy(&received_crc32, buffer_.data()+(frame_length-4), sizeof(uint32_t));
        if (computed_crc32 == received_crc32) {
            processFrame(buffer_.data());
            frames_processed++;
        } else {
            // CRC错误，移除这一帧
            printf("CRC check failed, discarding frame\n");
            memmove(buffer_.data(), buffer_.data() + 3, buffer_index_ - 3);
            buffer_index_ -= 3;
            continue;
        }

        // 移除已处理的帧
        if (frame_length < buffer_index_) {
            memmove(buffer_.data(), buffer_.data() + frame_length, buffer_index_ - frame_length);
            buffer_index_ -= frame_length;
        } else {
            buffer_index_ = 0;
        }
    }

    // 如果还有数据未处理，在下一个循环继续处理
    if (buffer_index_ >= FRAME_MIN_SIZE) {
        printf("Remaining data in buffer: %zu bytes\n", buffer_index_);
    }
}

void HeadIMUSerialCommunicationClass::timerCallback() {
    // 检查串口状态
    if (fd_ < 0) {
        if (std::chrono::steady_clock::now() - last_reconnect_time > std::chrono::seconds(3)) {
            printf("Serial port not available, trying reconnect\n");
            tryReconnect();
        }
        return;
    }
    if (std::chrono::steady_clock::now() - last_received_time > std::chrono::seconds(3)) {
        if (std::chrono::steady_clock::now() - last_reconnect_time > std::chrono::seconds(3)) {
            printf("No data received, trying reconnect\n");
            tryReconnect();
        }
        return;
    }

    // 读取串口数据
    if (buffer_index_ < BUFFER_SIZE - 128) {
        uint8_t temp_buffer[128];
        ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));
        
        if (bytes_read > 0) {
            if (buffer_index_ + bytes_read < BUFFER_SIZE) {
                memcpy(buffer_.data() + buffer_index_, temp_buffer, bytes_read);
                buffer_index_ += bytes_read;
                processBuffer();
            } else {
                printf("Buffer near full, discarding data\n");
                buffer_index_ = 0;
            }
        }
    }
}

void HeadIMUSerialCommunicationClass::timerThread() {
    while (running) {
        auto start = std::chrono::steady_clock::now();

        timerCallback();

        // 休眠至下一次调用
        std::this_thread::sleep_until(start + std::chrono::microseconds(1000));  // 大约1ms周期
    }
}



// ================= 工具函数 =================
uint32_t HAL_CRC_Calculate(const uint8_t* data, size_t length) {
    const uint32_t POLYNOMIAL = 0x04C11DB7;
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i += 4) {
        uint32_t word = 0;
        if (i + 3 < length) {
            word = *reinterpret_cast<const uint32_t*>(&data[i]);
        } else {
            // 处理不足4字节的情况
            for (size_t j = 0; j < 4 && i + j < length; ++j) {
                word |= static_cast<uint32_t>(data[i + j]) << (8 * j);
            }
        }
        
        crc ^= word;
        
        for (int j = 0; j < 32; ++j) {
            if (crc & 0x80000000) {
                crc = (crc << 1) ^ POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
