// ImagesInput.h
#ifndef IMAGESINPUT_H
#define IMAGESINPUT_H

#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm> // 用于排序
#include <dirent.h>
#include <sys/types.h>

// 使用相同的全局变量和互斥锁
extern bool g_bExit;
extern cv::Mat g_image;
extern pthread_mutex_t g_mutex;
extern bool image_used;

class ImagesInput {
public:
    // 构造函数：初始化图片输入
    ImagesInput(const std::string& folderPath);
    
    // 析构函数：释放资源
    ~ImagesInput();

    // 图片取流线程
    static void* workThread(void* pThis);

private:
    std::vector<std::string> imagePaths; // 存储图片路径
    pthread_t thread_id;
    size_t currentIndex; // 当前图片索引
};

#endif // IMAGESINPUT_H