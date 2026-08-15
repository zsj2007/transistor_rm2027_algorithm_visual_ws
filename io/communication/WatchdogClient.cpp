#include "communication/WatchdogClient.h"

// 只在实现文件中包含系统头文件
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <cerrno>
#include <system_error>

WatchdogClient::WatchdogClient(const std::string& socket_path) {
    // 确定套接字路径
    if (!socket_path.empty()) {
        socket_path_ = socket_path;
    } else if (const char* env_path = getenv("WATCHDOG_SOCKET_PATH")) {
        socket_path_ = env_path;
    } else {
        socket_path_ = "/tmp/rm2026_vision_watchdog.sock";
    }
    
    // 分配缓冲区存储sockaddr_un
    sockaddr_buffer_ = new char[sizeof(sockaddr_un)];
}

WatchdogClient::~WatchdogClient() {
    stop();
    
    // 清理缓冲区
    if (sockaddr_buffer_) {
        delete[] static_cast<char*>(sockaddr_buffer_);
        sockaddr_buffer_ = nullptr;
    }
}

bool WatchdogClient::init() {
    if (initialized_ || stopped_) {
        return connected_;
    }
    
    // 尝试连接看门狗
    connected_ = connectToWatchdog();
    initialized_ = true;
    
    if (connected_) {
        std::cout << "[Watchdog] 已连接到看门狗" << std::endl;
        
        // 启动发送线程
        send_thread_ = std::thread(&WatchdogClient::sendThread, this);
    } else {
        std::cout << "[Watchdog] 未找到看门狗，程序将继续独立运行" << std::endl;
    }
    
    return connected_;
}

bool WatchdogClient::connectToWatchdog() {
    // 检查套接字文件是否存在（看门狗服务器是否运行）
    if (access(socket_path_.c_str(), F_OK) != 0) {
        std::cout << "[Watchdog] 看门狗未运行，套接字文件不存在: " << socket_path_ << std::endl;
        return false;  // 看门狗未运行
    }
    
    // 创建Unix域套接字
    sockfd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[Watchdog] 创建套接字失败: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 准备地址结构
    sockaddr_un* addr = reinterpret_cast<sockaddr_un*>(sockaddr_buffer_);
    memset(addr, 0, sizeof(sockaddr_un));
    addr->sun_family = AF_UNIX;
    
    // 确保路径长度不超过限制
    size_t path_len = socket_path_.length();
    if (path_len >= sizeof(addr->sun_path)) {
        std::cerr << "[Watchdog] 套接字路径过长" << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    strncpy(addr->sun_path, socket_path_.c_str(), sizeof(addr->sun_path) - 1);
    
    std::cout << "[Watchdog] 正在连接到看门狗: " << socket_path_ << std::endl;
    
    // 尝试连接（设置非阻塞以避免长时间阻塞）
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);
    
    int result = connect(sockfd_, reinterpret_cast<sockaddr*>(addr), sizeof(sockaddr_un));
    
    // 使用select等待连接完成（设置1秒超时）
    if (result < 0 && errno == EINPROGRESS) {
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sockfd_, &writefds);
        
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        result = select(sockfd_ + 1, NULL, &writefds, NULL, &timeout);
        
        if (result <= 0) {
            // 连接超时或失败
            std::cerr << "[Watchdog] 连接超时" << std::endl;
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }
        
        // 检查套接字错误
        int so_error;
        socklen_t len = sizeof(so_error);
        getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &so_error, &len);
        
        if (so_error != 0) {
            std::cerr << "[Watchdog] 连接失败: " << strerror(so_error) << std::endl;
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }
    } else if (result < 0) {
        // 连接失败
        std::cerr << "[Watchdog] 连接失败: " << strerror(errno) << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    
    // 恢复阻塞模式
    fcntl(sockfd_, F_SETFL, flags & ~O_NONBLOCK);
    
    return true;
}

void WatchdogClient::disconnect() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
    connected_ = false;
}

void WatchdogClient::feed() {
    // 如果未连接或已停止，直接返回（不会影响程序运行）
    if (!connected_ || stopped_) {
        return;
    }
    
    // 异步发送心跳（不会阻塞调用线程）
    sendHeartbeatAsync();
}

void WatchdogClient::sendHeartbeatAsync() {
    // 将发送任务加入队列
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        send_queue_.push([this]() {
            sendHeartbeat();
        });
    }
    
    // 通知发送线程
    queue_cv_.notify_one();
}

bool WatchdogClient::sendHeartbeat() {
    if (sockfd_ < 0) {
        return false;
    }
    
    const char* heartbeat_msg = "ALIVE";
    ssize_t result = send(sockfd_, heartbeat_msg, strlen(heartbeat_msg), 0);
    
    if (result < 0) {
        // 发送失败，断开连接
        disconnect();
        return false;
    }
    
    return true;
}

void WatchdogClient::sendThread() {
    while (!stopped_) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            // 等待任务或停止信号
            queue_cv_.wait(lock, [this]() {
                return !send_queue_.empty() || stopped_;
            });
            
            if (stopped_ && send_queue_.empty()) {
                break;
            }
            
            if (!send_queue_.empty()) {
                task = std::move(send_queue_.front());
                send_queue_.pop();
            }
        }
        
        if (task) {
            task();
        }
    }
}

void WatchdogClient::stop() {
    if (stopped_) {
        return;
    }
    
    stopped_ = true;
    
    // 通知发送线程退出
    queue_cv_.notify_one();
    
    // 等待发送线程结束
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    
    // 断开连接
    disconnect();
    
    std::cout << "[Watchdog] 看门狗客户端已停止" << std::endl;
}
