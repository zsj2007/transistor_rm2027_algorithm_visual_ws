// VideoInput.h
#ifndef VIDEOINPUT_H
#define VIDEOINPUT_H

#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <unistd.h>
#include <string>
#include <iostream>

// 使用相同的全局变量和互斥锁
extern bool g_bExit;
extern cv::Mat g_image;
extern pthread_mutex_t g_mutex;
extern bool image_used;

class VideoInput {
public:
    // 构造函数：初始化视频输入
    VideoInput(const std::string& filename);
    
    // 析构函数：释放资源
    ~VideoInput();

    // 视频取流线程
    static void* workThread(void* pThis);

private:
    cv::VideoCapture cap;
    std::string filename;
    pthread_t thread_id;
    // float frame_rate;
};

#endif // VIDEOINPUT_H