#ifndef WATCHDOG_CLIENT_H
#define WATCHDOG_CLIENT_H

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

// 前置声明，避免包含系统头文件
struct sockaddr_un;

class WatchdogClient {
public:
    /**
     * @brief 构造函数
     * @param socket_path Unix域套接字路径，为空则从环境变量获取
     */
    explicit WatchdogClient(const std::string& socket_path = "");
    
    /**
     * @brief 析构函数
     */
    ~WatchdogClient();
    
    // 禁用拷贝构造和赋值
    WatchdogClient(const WatchdogClient&) = delete;
    WatchdogClient& operator=(const WatchdogClient&) = delete;
    
    /**
     * @brief 初始化看门狗客户端（只尝试连接一次）
     * @return 成功连接返回true，否则返回false
     */
    bool init();
    
    /**
     * @brief 手动喂狗（非阻塞）
     * @note 即使未连接或初始化失败，调用此函数也不会影响程序运行
     */
    void feed();
    
    /**
     * @brief 停止看门狗客户端
     */
    void stop();
    
    /**
     * @brief 检查是否已连接
     * @return 已连接返回true
     */
    bool isConnected() const { return connected_; }
    
    /**
     * @brief 检查看门狗是否可用
     * @return 如果未初始化或已停止返回false
     */
    bool isAvailable() const { return initialized_ && !stopped_; }

private:
    /**
     * @brief 尝试连接看门狗
     * @return 成功连接返回true
     */
    bool connectToWatchdog();
    
    /**
     * @brief 断开连接
     */
    void disconnect();
    
    /**
     * @brief 发送线程函数
     */
    void sendThread();
    
    /**
     * @brief 异步发送心跳消息
     */
    void sendHeartbeatAsync();
    
    /**
     * @brief 实际发送心跳消息（在线程中调用）
     * @return 成功发送返回true
     */
    bool sendHeartbeat();

private:
    std::string socket_path_;          // 套接字路径
    std::atomic<bool> connected_{false};  // 连接状态
    std::atomic<bool> initialized_{false}; // 初始化状态
    std::atomic<bool> stopped_{false};    // 停止标志
    
    // 发送线程相关
    std::thread send_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<std::function<void()>> send_queue_;
    
    // 套接字相关（使用void*避免直接暴露系统头文件）
    void* sockaddr_buffer_{nullptr};  // 用于存储sockaddr_un
    int sockfd_{-1};                  // 套接字描述符
};

#endif // WATCHDOG_CLIENT_H
