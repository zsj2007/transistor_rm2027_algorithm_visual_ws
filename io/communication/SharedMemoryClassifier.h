// SharedMemoryClassifier.h
#ifndef SHARED_MEMORY_CLASSIFIER_H
#define SHARED_MEMORY_CLASSIFIER_H

#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>           // for O_CREAT, O_EXCL
#include <semaphore.h>       // POSIX 信号量头文件
#include <yaml-cpp/yaml.h>

class SharedMemoryClassifier {
public:
    SharedMemoryClassifier(std::shared_ptr<YAML::Node> config_file_ptr);
    ~SharedMemoryClassifier();
    
    // 向共享内存写入图像并等待处理结果
    std::vector<std::vector<float>> processImages(const std::vector<cv::Mat>& images);

private:
    // 共享内存数据结构（必须与Python端完全匹配）
    #pragma pack(push, 1)
    struct SharedData {
        int num_images;         // 当前批次图像数量
        bool reserved0; // 原来的 is_processed 可以废弃不用了，但为了兼容内存结构保留
        bool show_windows;      // 显示图像
        bool reserved2;         // 备用标志2
        bool reserved3;         // 备用标志3
        
        float results[100][12]; // 结果存储区 (100x12 float)
        unsigned char image_data[100][64*48*3]; // 图像存储区 (100张64x48 RGB图像)
    };
    #pragma pack(pop)
    
    int shm_id_;
    SharedData* shared_data_;
    const size_t MAX_IMAGES = 100;
    int CLASSIFIER_SHM_KEY; // 共享内存键值
    bool show_windows_ = false;
    bool python_available_ = true;   // Python 端是否可用（首次超时后置 false）
    std::chrono::steady_clock::time_point last_retry_;  // 上次重试探测时间

    // 新增：POSIX 信号量
    sem_t* sem_data_ready_;  
    sem_t* sem_result_ready_; 
    
    void attachSharedMemory();
    void detachSharedMemory();
    void resetSemaphore(sem_t* sem);
};

#endif // SHARED_MEMORY_CLASSIFIER_H
