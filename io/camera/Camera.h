#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "other_input/FramePacket.h"

extern FramePacket g_frame_packet;
extern pthread_mutex_t g_mutex;
extern bool g_bExit;
extern bool image_used;

enum CameraType {
    GIGE_CAMERA,
    USB_CAMERA
};

enum CameraStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    GRABBING,
    ERROR
};

class Camera {
public:
    // GigE相机构造函数
    Camera(const std::string& deviceIp, const std::string& netIp);
    
    // USB相机构造函数
    Camera(int deviceIndex = 0);
    
    // 析构函数：释放资源
    ~Camera();
    
    // 开始连接和取流
    bool start();
    
    // 停止连接和取流
    void stop();
    
    // IP地址解析函数
    static void parseIp(const std::string& ip, unsigned int& parsedIp);

    // 新增：设置曝光时间（单位：微秒）
    bool setExposureTime(float exposureTime_);
    
    // 新增：设置增益值（范围通常在0-15之间）
    bool setGain(float gain_);
    
    // 枚举USB设备
    static std::vector<std::string> enumUSBDevices();
    
    // 获取相机状态
    CameraStatus getStatus() const { return status.load(); }

private:
    // 句柄和状态
    void* handle;
    std::atomic<CameraStatus> status;
    CameraType cameraType;
    std::atomic<bool> running;
    
    // GigE参数
    std::string deviceIp;
    std::string netIp;
    
    // USB参数
    int deviceIndex;
    
    // 图像相关
    std::atomic<std::chrono::steady_clock::time_point> lastFrameTime;
    
    // 重连相关
    std::thread reconnectThread;
    std::atomic<std::chrono::steady_clock::time_point> lastReconnectTime;
    std::mutex reconnectMutex;
    std::condition_variable reconnectCV;
    
    // 取流线程相关
    std::thread grabThread;
    std::atomic<bool> grabbing;
    std::atomic<bool> needReconnect;
    
    // 内部方法
    void reconnectLoop();
    void grabLoop();
    bool connectDevice();
    void disconnectDevice();
    bool tryConnectGigE();
    bool tryConnectUSB();
    
    // 图像处理
    bool processImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX& stImageInfo, cv::Mat& outputImage);
    
    // 初始化相机参数
    bool initCameraParams();
    bool initCameraCommonParams();
    
    // 设置连接时间为当前时间
    void updateReconnectTime() {
        lastReconnectTime.store(std::chrono::steady_clock::now());
    }

    float exposureTime;
    float gain;
};

#endif // CAMERA_H
