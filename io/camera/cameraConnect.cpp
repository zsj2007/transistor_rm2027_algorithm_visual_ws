#include "camera/Camera.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>  // std::swap

using namespace std::chrono;

namespace {
double steadySeconds(const steady_clock::time_point& t) {
    return duration<double>(t.time_since_epoch()).count();
}
}  // namespace

// GigE相机构造函数
Camera::Camera(const std::string& deviceIp, const std::string& netIp,
               const CameraTimestampConfig& timestampConfig)
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
    , gain(16.0)
    , timestampConfig_(timestampConfig) {
    
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
    , gain(16.0)
    , timestampConfig_() {
    
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
    std::uint64_t next_frame_id = 0;
    
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
            //检查是否需要重设摄像机内部时间与主机时间之间的差值
            maybeResyncTimestampClock();
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, nPayloadSize, &stImageInfo, 1000);
            
            if (nRet == MV_OK) {
                const auto arrival_time = steady_clock::now();
                const double arrival_timestamp_s = steadySeconds(arrival_time);
                bool timestamp_from_device = false;
                const double frame_timestamp_s = resolveFrameTimestamp(
                    stImageInfo, arrival_time, timestamp_from_device);
                const std::uint64_t frame_id = next_frame_id++;
                // 检查帧数据完整性
                if (stImageInfo.nFrameLen > 0) {
                    cv::Mat processedImage;
                    if (processImage(pData, stImageInfo, processedImage)) {
                        // 收到有效图像即更新成功时间（用于断流重连判定）
                        lastSuccessTime = steady_clock::now();
                        // Publish pixels and source metadata atomically.
                        pthread_mutex_lock(&g_mutex);
                        g_frame_packet.image = processedImage.clone();
                        g_frame_packet.timestamp_s = frame_timestamp_s;
                        g_frame_packet.frame_id = frame_id;
                        g_frame_packet.arrival_timestamp_s = arrival_timestamp_s;
                        g_frame_packet.device_timestamp_ticks =
                            (static_cast<std::uint64_t>(stImageInfo.nDevTimeStampHigh) << 32) |
                            static_cast<std::uint64_t>(stImageInfo.nDevTimeStampLow);
                        g_frame_packet.sdk_host_timestamp = stImageInfo.nHostTimeStamp;
                        g_frame_packet.exposure_time_us =
                            stImageInfo.fExposureTime > 0.0f
                                ? static_cast<double>(stImageInfo.fExposureTime)
                                : static_cast<double>(exposureTime);
                        g_frame_packet.clock_sync_uncertainty_us = timestampSyncUncertaintyUs_;
                        g_frame_packet.timestamp_from_device = timestamp_from_device;
                        image_used = false;
                        pthread_mutex_unlock(&g_mutex);

                        static auto last_timestamp_log = steady_clock::time_point{};
                        if (last_timestamp_log.time_since_epoch().count() == 0 ||
                            arrival_time - last_timestamp_log >= seconds(1)) {
                            last_timestamp_log = arrival_time;
                            std::cout << std::fixed << std::setprecision(3)
                                      << "[camera-ts] source="
                                      << (timestamp_from_device ? "hik_device" : "arrival_fallback")
                                      << " frame=" << frame_id
                                      << " sdk_frame=" << stImageInfo.nFrameNum
                                      << " exposure_us=" << g_frame_packet.exposure_time_us
                                      << " arrival_minus_frame_ms="
                                      << (arrival_timestamp_s - frame_timestamp_s) * 1000.0
                                      << " sync_uncertainty_us=" << timestampSyncUncertaintyUs_
                                      << " dev_ticks=" << g_frame_packet.device_timestamp_ticks
                                      << " sdk_host_ts=" << stImageInfo.nHostTimeStamp
                                      << std::endl;
                        }
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

    // 初始化相机
    resetTimestampSync();
    if (timestampConfig_.use_device_timestamp && !initializeTimestampSync()) {
        std::cerr << "Hikrobot device timestamp unavailable; using arrival-time fallback ("
                  << timestampConfig_.fallback_delay_ms << " ms)." << std::endl;
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

// 清空上一次相机连接留下的时间同步状态
void Camera::resetTimestampSync() {
    timestampSyncValid_ = false;
    deviceTimestampFrequencyHz_ = 0.0;
    deviceToSteadyOffsetS_ = 0.0;
    timestampSyncUncertaintyUs_ = 0.0;
    lastTimestampSync_ = steady_clock::time_point{};
}

// 相机连接成功后，初始化硬件时间戳系统
bool Camera::initializeTimestampSync() {
    if (handle == nullptr || cameraType != GIGE_CAMERA ||
        !timestampConfig_.use_device_timestamp) {
        return false;
    }

    MVCC_INTVALUE_EX frequency;
    memset(&frequency, 0, sizeof(frequency));
    int nRet = MV_CC_GetIntValueEx(handle, "GevTimestampTickFrequency", &frequency);
    if (nRet != MV_OK || frequency.nCurValue <= 0) {
        // Some GenICam XMLs expose the newer standard node name.
        memset(&frequency, 0, sizeof(frequency));
        nRet = MV_CC_GetIntValueEx(handle, "DeviceTimestampTickFrequency", &frequency);
    }
    if (nRet != MV_OK || frequency.nCurValue <= 0) {
        std::cerr << "Get camera timestamp frequency fail! nRet [0x"
                  << std::hex << nRet << std::dec << "]" << std::endl;
        return false;
    }

    deviceTimestampFrequencyHz_ = static_cast<double>(frequency.nCurValue);
    const bool ok = calibrateTimestampSync(
        std::max(1, timestampConfig_.sync_samples), false);
    if (ok) {
        std::cout << std::fixed << std::setprecision(3)
                  << "Hikrobot timestamp sync ready: frequency="
                  << deviceTimestampFrequencyHz_ << " Hz, uncertainty<="
                  << timestampSyncUncertaintyUs_ << " us" << std::endl;
    }
    return ok;
}

// 核心的校时函数
// 计算最佳时间偏移量offset_s
bool Camera::calibrateTimestampSync(int sampleCount, bool smoothUpdate) {
    if (handle == nullptr || deviceTimestampFrequencyHz_ <= 0.0) return false;

    double best_rtt_s = std::numeric_limits<double>::infinity();
    double best_offset_s = 0.0;
    bool got_sample = false;

    for (int i = 0; i < std::max(1, sampleCount); ++i) {
        const auto command_start = steady_clock::now();
        const int latch_ret = MV_CC_SetCommandValue(handle, "GevTimestampControlLatch");
        const auto command_end = steady_clock::now();
        if (latch_ret != MV_OK) continue;

        MVCC_INTVALUE_EX latched;
        memset(&latched, 0, sizeof(latched));
        const int read_ret = MV_CC_GetIntValueEx(handle, "GevTimestampValue", &latched);
        if (read_ret != MV_OK || latched.nCurValue < 0) continue;

        // The latch instant lies within this command round trip. The midpoint
        // is our estimate and half the RTT is a conservative uncertainty bound.
        const double rtt_s = duration<double>(command_end - command_start).count();
        if (rtt_s < best_rtt_s) {
            const double host_mid_s =
                0.5 * (steadySeconds(command_start) + steadySeconds(command_end));
            best_offset_s = host_mid_s -
                static_cast<double>(latched.nCurValue) / deviceTimestampFrequencyHz_;
            best_rtt_s = rtt_s;
            got_sample = true;
        }
    }

    if (!got_sample) return false;

    if (!timestampSyncValid_ || !smoothUpdate) {
        deviceToSteadyOffsetS_ = best_offset_s;
    } else {
        const double delta_s = best_offset_s - deviceToSteadyOffsetS_;
        if (std::abs(delta_s) > 0.050) {
            std::cerr << "Camera clock resync rejected: offset jumped by "
                      << delta_s * 1000.0 << " ms" << std::endl;
            return false;
        }
        // Suppress control-channel jitter while still tracking clock drift.
        deviceToSteadyOffsetS_ += 0.2 * delta_s;
    }

    timestampSyncUncertaintyUs_ = best_rtt_s * 0.5e6;
    timestampSyncValid_ = true;
    lastTimestampSync_ = steady_clock::now();
    return true;
}

// 定期重新同步相机和主机时钟
// 当偏移量误差重新变大时 重新设置
void Camera::maybeResyncTimestampClock() {
    if (!timestampSyncValid_ || timestampConfig_.resync_interval_s <= 0.0) return;
    const auto interval = duration<double>(timestampConfig_.resync_interval_s);
    if (steady_clock::now() - lastTimestampSync_ >= interval) {
        calibrateTimestampSync(
            std::min(3, std::max(1, timestampConfig_.sync_samples)), true);
    }
}

// 把一帧的海康设备时间戳转换成最终用于查询云台姿态的时间
// arrival为图像送至SDK的时间
double Camera::resolveFrameTimestamp(
    const MV_FRAME_OUT_INFO_EX& frameInfo,
    steady_clock::time_point arrival,
    bool& fromDevice) const {
    const double arrival_s = steadySeconds(arrival);
    fromDevice = false;

    const std::uint64_t device_ticks =
        (static_cast<std::uint64_t>(frameInfo.nDevTimeStampHigh) << 32) |
        static_cast<std::uint64_t>(frameInfo.nDevTimeStampLow);
    if (timestampSyncValid_ && device_ticks != 0) {
        const double exposure_us = frameInfo.fExposureTime > 0.0f
            ? static_cast<double>(frameInfo.fExposureTime)
            : static_cast<double>(exposureTime);
        const double frame_s =
            static_cast<double>(device_ticks) / deviceTimestampFrequencyHz_ +
            deviceToSteadyOffsetS_ +
            timestampConfig_.exposure_offset_ratio * exposure_us * 1e-6;
        const double age_s = arrival_s - frame_s;

        // Reject a mismatched node/unit instead of feeding a wildly wrong time
        // into pose history. Normal exposure + readout + GigE transport is
        // positive and comfortably below this two-second guard.
        if (age_s >= -0.010 && age_s <= 2.0) {
            fromDevice = true;
            return frame_s;
        }
    }

    return arrival_s - timestampConfig_.fallback_delay_ms * 1e-3;
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
                // 必须持有自有内存：outputImage 最终会发布到 g_frame_packet.image，
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
