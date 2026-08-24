// VideoInput.cpp
#include "other_input/VideoInput.h"

#include <cmath>

// 使用在camera.cpp中定义的全局变量
extern bool g_bExit;
extern FramePacket g_frame_packet;
extern pthread_mutex_t g_mutex;
extern bool image_used;

VideoInput::VideoInput(const std::string& filename, double configured_fps)
    : filename(filename) {
    // 打开视频文件
    cap.open(filename);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file: " << filename << std::endl;
        exit(1);
    } else {
        std::cout << "Video file opened successfully: " << filename << std::endl;
    }

    const double video_fps = cap.get(cv::CAP_PROP_FPS);
    if (std::isfinite(video_fps) && video_fps > 0.0) {
        source_fps_ = video_fps;
    } else if (std::isfinite(configured_fps) && configured_fps > 0.0) {
        source_fps_ = configured_fps;
    }

    // 启动取流线程
    pthread_create(&thread_id, NULL, workThread, this);
    std::cout << "Video grabbing thread started." << std::endl;
}

VideoInput::~VideoInput() {
    // 等待线程结束
    g_bExit = true;
    pthread_join(thread_id, NULL);
    
    // 释放视频捕获资源
    if (cap.isOpened()) {
        cap.release();
    }
    std::cout << "VideoInput released." << std::endl;
}

void* VideoInput::workThread(void* pThis) {
    VideoInput* pVideo = (VideoInput*)pThis;
    cv::Mat frame;

    while (!g_bExit) {
        pVideo->cap >> frame;  // 读取一帧
        
        if (frame.empty()) {
            // 视频结束，重置到开头重新播放
            pVideo->cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            std::cout << "Video restarted." << std::endl;
            continue;
        }

        // 将BGR转换为RGB（与摄像头处理一致）
        // cv::Mat rgbFrame;
        // cv::cvtColor(frame, rgbFrame, cv::COLOR_BGR2RGB);

        // 截取区域
        if (frame.size[1]>1280 && frame.size[0]>1024) {
            cv::Rect roi_rect((frame.size[1]-1280)/2, 0, 1280, 1024);
            frame = frame(roi_rect);
        }
        //cv::Mat resized_frame;
        //cv::resize(frame, resized_frame, cv::Size(1280, 1024));

        // 线程锁定，更新全局图像
        while (!image_used)
        {
            usleep(1000);
        }
        pthread_mutex_lock(&g_mutex);
        g_frame_packet.image = frame.clone();
        g_frame_packet.timestamp_s =
            static_cast<double>(pVideo->global_frame_index_) / pVideo->source_fps_;
        g_frame_packet.frame_id = pVideo->global_frame_index_;
        ++pVideo->global_frame_index_;
        image_used = false;
        pthread_mutex_unlock(&g_mutex);
        
        // 按视频原始帧率播放
        // int delay = static_cast<int>(1000 / pVideo->cap.get(cv::CAP_PROP_FPS));
        // int delay = (int)(1000 / pVideo->frame_rate);
        // usleep(delay * 1000);
    }
    
    std::cout << "Video grabbing thread exiting." << std::endl;
    return NULL;
}
