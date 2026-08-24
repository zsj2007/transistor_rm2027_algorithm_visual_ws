// VideoInput.h
#ifndef VIDEOINPUT_H
#define VIDEOINPUT_H

#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstdint>

#include "other_input/FramePacket.h"

// 使用相同的全局变量和互斥锁
extern bool g_bExit;
extern FramePacket g_frame_packet;
extern pthread_mutex_t g_mutex;
extern bool image_used;

class VideoInput {
public:
    // 构造函数：初始化视频输入
    VideoInput(const std::string& filename, double configured_fps);
    
    // 析构函数：释放资源
    ~VideoInput();

    // 视频取流线程
    static void* workThread(void* pThis);

private:
    cv::VideoCapture cap;
    std::string filename;
    pthread_t thread_id;
    double source_fps_ = 30.0;
    std::uint64_t global_frame_index_ = 0;
};

#endif // VIDEOINPUT_H
