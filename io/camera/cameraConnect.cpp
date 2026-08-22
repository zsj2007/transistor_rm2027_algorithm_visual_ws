#include "camera/Camera.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <utility>  // std::swap

using namespace std::chrono;

// GigE相机构造函数
Camera::Camera(const std::string& deviceIp, const std::string& netIp) 
    : handle(nullptr)
    , status(DISCONNECTED)
    , cameraType(GIGE_CAMERA)
    , running(false)
    , deviceIp(deviceIp)
    , netIp(netIp)
    , deviceIndex(0)
    , grabbing(false)
    , needReconnect(false)
    , exposureTime(5000)
    , gain(16.0) {
    
    std::cout << "GigE Camera created with IP: " << deviceIp << std::endl;
}

// USB相机构造函数
Camera::Camera(int deviceIndex) 
    : handle(nullptr)
    , status(DISCONNECTED)
    , cameraType(USB_CAMERA)
    , running(false)
    , deviceIp("")
    , netIp("")
    , deviceIndex(deviceIndex)
    , grabbing(false)
    , needReconnect(false)
    , exposureTime(5000)
    , gain(16.0) {
    
    std::cout << "USB Camera created with device index: " << deviceIndex << std::endl;
}

bool Camera::start() {
    if (running.load()) {
        std::cout << "Camera is already running." << std::endl;
        return true;
    }

    // 初始化时间点
    auto now = steady_clock::now();
    lastReconnectTime.store(now);
    
    running.store(true);
    status.store(CONNECTING);
    
    // 启动重连线程
    reconnectThread = std::thread(&Camera::reconnectLoop, this);
    
    // 启动取流线程
    grabThread = std::thread(&Camera::grabLoop, this);
    
    std::cout << "Camera started." << std::endl;
    return true;
}

void Camera::stop() {
    if (!running.load()) {
        return;
    }
    
    std::cout << "Stopping camera..." << std::endl;
    running.store(false);
    grabbing.store(false);
    needReconnect.store(true);
    
    // 通知重连线程
    {
        std::lock_guard<std::mutex> lock(reconnectMutex);
        reconnectCV.notify_all();
    }
    
    // 等待线程结束
    if (reconnectThread.joinable()) {
        reconnectThread.join();
    }
    
    if (grabThread.joinable()) {
        grabThread.join();
    }
    
    // 断开设备连接
    disconnectDevice();
    
    std::cout << "Camera stopped." << std::endl;
}

Camera::~Camera() {
    stop();
}

void Camera::reconnectLoop() {
    std::cout << "Reconnect loop started." << std::endl;
    bool first_try = true;
    
    while (running.load()) {
        CameraStatus currentStatus = status.load();
        
        // 如果已经连接并且不需要重连，则等待
        if (currentStatus == GRABBING && !needReconnect.load()) {
            std::unique_lock<std::mutex> lock(reconnectMutex);
            reconnectCV.wait_for(lock, std::chrono::seconds(1));
            continue;
        }
            
        if (!first_try) { // 第一次连接不需要等待
            // 检查是否需要等待3秒
            auto now = steady_clock::now();
            auto lastReconnect = lastReconnectTime.load();
            auto timeSinceLastReconnect = duration_cast<seconds>(now - lastReconnect);
            
            if (timeSinceLastReconnect.count() < 3 && !needReconnect.load()) {
                // 等待到3秒间隔
                std::unique_lock<std::mutex> lock(reconnectMutex);
                auto waitTime = seconds(3) - timeSinceLastReconnect;
                reconnectCV.wait_for(lock, waitTime);
                continue;
            }
        }
        first_try = false;
        
        // 设置状态为正在连接
        status.store(CONNECTING);
        std::cout << "Attempting to connect camera..." << std::endl;
        
        // 尝试连接设备
        if (connectDevice()) {
            status.store(CONNECTED);
            needReconnect.store(false);
            std::cout << "Camera connected successfully." << std::endl;
            
            // 更新重连时间
            updateReconnectTime();
            
            // 短暂延迟后开始取流
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            grabbing.store(true);
        } else {
            status.store(DISCONNECTED);
            std::cout << "Failed to connect camera. Will retry in 3 seconds." << std::endl;
            
            // 更新重连时间
            updateReconnectTime();
            
            // 等待3秒
            std::unique_lock<std::mutex> lock(reconnectMutex);
            reconnectCV.wait_for(lock, std::chrono::seconds(3));
        }
    }
    
    std::cout << "Reconnect loop stopped." << std::endl;
}

void Camera::grabLoop() {
    std::cout << "Grab loop started." << std::endl;
    
    while (running.load()) {
        // 等待开始取流信号
        while (running.load() && !grabbing.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!running.load()) {
            break;
        }
        
        // 开始取流
        if (handle == nullptr) {
            needReconnect.store(true);
            grabbing.store(false);
            continue;
        }
        
        int nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) {
            std::cerr << "Start grabbing fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
            needReconnect.store(true);
            grabbing.store(false);
            continue;
        }
        
        status.store(GRABBING);
        std::cout << "Grabbing started." << std::endl;
        
        // 获取数据包大小
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet) {
            std::cerr << "Get PayloadSize fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
            needReconnect.store(true);
            grabbing.store(false);
            continue;
        }
        
        unsigned int nPayloadSize = stParam.nCurValue;
        unsigned char* pData = new unsigned char[nPayloadSize];
        if (pData == nullptr) {
            std::cerr << "Allocate memory fail!" << std::endl;
            needReconnect.store(true);
            grabbing.store(false);
            continue;
        }
        
        MV_FRAME_OUT_INFO_EX stImageInfo;
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        
        // 记录上次成功获取图像的时间
        auto lastSuccessTime = steady_clock::now();
        
        // 取流循环
        while (running.load() && grabbing.load()) {
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, nPayloadSize, &stImageInfo, 1000);
            
            if (nRet == MV_OK) {
                // 检查帧数据完整性
                if (stImageInfo.nFrameLen > 0) {
                    cv::Mat processedImage;
                    if (processImage(pData, stImageInfo, processedImage)) {
                        // 收到有效图像即更新成功时间（用于断流重连判定）
                        lastSuccessTime = steady_clock::now();
                        // 零拷贝交接：processedImage 始终持有自有内存（见 processImage），
                        // 锁内 swap 换出，避免每帧一次全图 clone
                        pthread_mutex_lock(&g_mutex);
                        std::swap(g_image, processedImage);
                        image_used = false;
                        pthread_mutex_unlock(&g_mutex);
                    }
                }
            } else {
                // 检查是否超时
                auto now = steady_clock::now();
                auto timeSinceLastSuccess = duration_cast<seconds>(now - lastSuccessTime);
                
                if (timeSinceLastSuccess.count() >= 3) {
                    std::cout << "No valid image for 3 seconds, triggering reconnect." << std::endl;
                    needReconnect.store(true);
                    grabbing.store(false);
                    break;
                }
            }
            
            // 检查是否需要重连
            if (needReconnect.load()) {
                grabbing.store(false);
                break;
            }
        }
        
        // 停止取流
        if (handle != nullptr) {
            MV_CC_StopGrabbing(handle);
            std::cout << "Grabbing stopped." << std::endl;
        }
        
        // 清理内存
        delete[] pData;
        
        // 短暂延迟，避免快速重连
        if (needReconnect.load()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    std::cout << "Grab loop stopped." << std::endl;
}

bool Camera::connectDevice() {
    int nRet = MV_OK;
    
    // 初始化SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        std::cerr << "Initialize SDK fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    
    if (cameraType == GIGE_CAMERA) {
        return tryConnectGigE();
    } else {
        return tryConnectUSB();
    }
}

bool Camera::tryConnectGigE() {
    int nRet = MV_OK;
    
    MV_CC_DEVICE_INFO stDevInfo;
    MV_GIGE_DEVICE_INFO stGigEDev;
    memset(&stDevInfo, 0, sizeof(MV_CC_DEVICE_INFO));
    memset(&stGigEDev, 0, sizeof(MV_GIGE_DEVICE_INFO));
    
    // 解析IP地址
    parseIp(deviceIp, stGigEDev.nCurrentIp);
    parseIp(netIp, stGigEDev.nNetExport);
    
    stDevInfo.nTLayerType = MV_GIGE_DEVICE;
    stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;
    
    // 创建句柄
    nRet = MV_CC_CreateHandle(&handle, &stDevInfo);
    if (MV_OK != nRet) {
        std::cerr << "Create Handle fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    
    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "Open device fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
        return false;
    }
    
    // 获取 GigE 相机的最佳数据包大小
    int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
    if (nPacketSize > 0) {
        nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
        if (MV_OK != nRet) {
            std::cerr << "Set Packet Size fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        }
    }
    
    // 初始化相机参数
    if (!initCameraCommonParams()) {
        std::cerr << "Failed to initialize camera parameters!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
        return false;
    }
    
    return true;
}

bool Camera::tryConnectUSB() {
    int nRet = MV_OK;
    
    // 枚举USB设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "Enum USB devices fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    
    if (stDeviceList.nDeviceNum == 0) {
        std::cerr << "No USB camera found!" << std::endl;
        return false;
    }
    
    if (deviceIndex >= static_cast<int>(stDeviceList.nDeviceNum)) {
        std::cerr << "Device index out of range! Found " << stDeviceList.nDeviceNum << " devices." << std::endl;
        return false;
    }
    
    // 创建句柄
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[deviceIndex]);
    if (MV_OK != nRet) {
        std::cerr << "Create Handle fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    
    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "Open device fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
        return false;
    }
    
    // 初始化相机参数
    if (!initCameraCommonParams()) {
        std::cerr << "Failed to initialize camera parameters!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
        return false;
    }
    
    return true;
}

void Camera::disconnectDevice() {
    if (handle == nullptr) {
        return;
    }
    
    // 停止取流
    if (grabbing.load()) {
        MV_CC_StopGrabbing(handle);
        grabbing.store(false);
    }
    
    // 关闭设备
    MV_CC_CloseDevice(handle);
    
    // 销毁句柄
    MV_CC_DestroyHandle(handle);
    handle = nullptr;
    
    // 释放SDK资源
    MV_CC_Finalize();
    
    std::cout << "Device disconnected." << std::endl;
}

// 其他方法保持不变，只需要进行小的调整
void Camera::parseIp(const std::string& ip, unsigned int& parsedIp) {
    int parts[4];
    sscanf(ip.c_str(), "%d.%d.%d.%d", &parts[0], &parts[1], &parts[2], &parts[3]);
    parsedIp = (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
}

bool Camera::setExposureTime(float exposureTime_) {
    exposureTime = exposureTime_;
    if (handle == nullptr) {
        return false;
    }
    
    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
    if (MV_OK != nRet) {
        std::cerr << "Set ExposureTime fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    std::cout << "Exposure time set to " << exposureTime << "us" << std::endl;
    return true;
}

bool Camera::setGain(float gain_) {
    gain = gain_;
    if (handle == nullptr) {
        return false;
    }
    
    int nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (MV_OK != nRet) {
        std::cerr << "Set Gain fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }
    std::cout << "Gain set to " << gain << std::endl;
    return true;
}

bool Camera::initCameraParams() {
    if (handle == nullptr) {
        return false;
    }
    
    int nRet;
    
    // 设置默认曝光时间 (5100微秒)
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
    if (MV_OK != nRet) {
        std::cerr << "Set ExposureTime fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        // 参数设置失败不阻断连接：相机继续用当前值，避免单个配置值超范围导致相机永远连不上
    }
    
    // 设置默认增益值 (16.0)
    nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (MV_OK != nRet) {
        std::cerr << "Set Gain fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        // 同上：警告后继续连接
    }
    
    return true;
}

bool Camera::initCameraCommonParams() {
    if (handle == nullptr) {
        return false;
    }
    
    int nRet;
    
    // 禁用自动曝光
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0);
    if (MV_OK != nRet) {
        std::cerr << "Disable auto exposure fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        // 失败不阻断连接
    }
    
    // 禁用自动增益
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 0);
    if (MV_OK != nRet) {
        std::cerr << "Disable auto gain fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        // 失败不阻断连接
    }
    
    // 禁用自动白平衡
    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
    if (MV_OK != nRet) {
        std::cerr << "Disable auto white balance fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        // 失败不阻断连接
    }
    
    // 设置相机参数
    if (!initCameraParams()) {
        std::cerr << "Failed to initialize camera exposure and gain parameters!" << std::endl;
        return false;
    }
    
    return true;
}

bool Camera::processImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX& stImageInfo, cv::Mat& outputImage) {
    switch (stImageInfo.enPixelType) {
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_BayerBG8: {
            cv::Mat img(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
            cv::Mat bgrImg;
            
            int conversionCode = -1;
            switch (stImageInfo.enPixelType) {
                case PixelType_Gvsp_BayerGB8:
                    conversionCode = cv::COLOR_BayerGB2BGR;
                    break;
                case PixelType_Gvsp_BayerRG8:
                    conversionCode = cv::COLOR_BayerRG2BGR;
                    break;
                case PixelType_Gvsp_BayerGR8:
                    conversionCode = cv::COLOR_BayerGR2BGR;
                    break;
                case PixelType_Gvsp_BayerBG8:
                    conversionCode = cv::COLOR_BayerBG2BGR;
                    break;
                default:
                    conversionCode = cv::COLOR_BayerGB2BGR;
            }
            
            cv::cvtColor(img, bgrImg, conversionCode);
            
            std::vector<cv::Mat> channels(3);
            cv::split(bgrImg, channels);
            cv::Mat temp = channels[0];
            channels[0] = channels[2];
            channels[2] = temp;
            cv::merge(channels, bgrImg);
            
            outputImage = bgrImg;
            return true;
        }
        
        case PixelType_Gvsp_RGB8_Packed:
        case PixelType_Gvsp_BGR8_Packed: {
            cv::Mat img(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
            if (stImageInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
                cv::cvtColor(img, outputImage, cv::COLOR_RGB2BGR);
            } else {
                // 必须持有自有内存：outputImage 会经 swap 交给 g_image，
                // 而 img 只是包着 SDK 缓冲 pData，下一帧取流会被覆盖
                img.copyTo(outputImage);
            }
            return true;
        }
        
        case PixelType_Gvsp_Mono8: {
            // 同上，必须持有自有内存
            cv::Mat img(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
            img.copyTo(outputImage);
            return true;
        }
            
        default:
            std::cerr << "Unsupported pixel format: " << stImageInfo.enPixelType << std::endl;
            return false;
    }
}

std::vector<std::string> Camera::enumUSBDevices() {
    std::vector<std::string> deviceList;
    int nRet = MV_OK;
    
    // 初始化SDK（如果尚未初始化）
    static bool sdkInitialized = false;
    if (!sdkInitialized) {
        nRet = MV_CC_Initialize();
        if (MV_OK != nRet) {
            std::cerr << "Initialize SDK fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
            return deviceList;
        }
        sdkInitialized = true;
    }
    
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "Enum USB devices fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return deviceList;
    }
    
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            std::string deviceName = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            std::string serialNumber = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            std::string deviceInfo = "Device " + std::to_string(i) + ": " + deviceName + " (SN: " + serialNumber + ")";
            deviceList.push_back(deviceInfo);
        }
    }
    
    return deviceList;
}
