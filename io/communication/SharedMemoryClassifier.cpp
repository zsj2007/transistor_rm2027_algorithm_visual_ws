// SharedMemoryClassifier.cpp
#include "communication/SharedMemoryClassifier.h"
#include <errno.h>  // 必须包含这个，用于检查 EAGAIN
#include <chrono>
#include <iostream>

// 提取一个私有方法用于安全清零信号量
void SharedMemoryClassifier::resetSemaphore(sem_t* sem) {
    // 循环尝试将信号量减 1
    while (true) {
        if (sem_trywait(sem) == 0) {
            // 成功减 1，说明消耗了一个残留的信号，继续循环尝试
            continue;
        } else {
            // 返回 -1，检查错误码
            if (errno == EAGAIN) {
                // 错误码是 EAGAIN，说明信号量已经是 0 了，清零完成，退出循环
                break; 
            } else {
                // 其他错误（比如信号量被意外删除等），打印日志并退出
                perror("sem_trywait failed during reset");
                break;
            }
        }
    }
}

SharedMemoryClassifier::SharedMemoryClassifier(std::shared_ptr<YAML::Node> config_file_ptr) {
    CLASSIFIER_SHM_KEY = (*config_file_ptr)["CLASSIFIER_SHM_KEY"].as<int>();
    show_windows_ = (*config_file_ptr)["SHOW_WINDOWS"] ? (*config_file_ptr)["SHOW_WINDOWS"].as<bool>() : false;

    // 1. 初始化信号量 (使用 O_EXCL 确保只有第一个进程创建并初始化值，后续进程直接打开)
    // 初始值设为 0，表示一开始没有数据
    sem_data_ready_ = sem_open("/shm_classifier_data", O_CREAT | O_EXCL, 0666, 0);
    if (sem_data_ready_ == SEM_FAILED) {
        sem_data_ready_ = sem_open("/shm_classifier_data", 0); // 已存在，直接打开
    }
    
    sem_result_ready_ = sem_open("/shm_classifier_result", O_CREAT | O_EXCL, 0666, 0);
    if (sem_result_ready_ == SEM_FAILED) {
        sem_result_ready_ = sem_open("/shm_classifier_result", 0);
    }

    // 2. 【新增】初始化时强制清零，消除上次崩溃残留的脏状态
    resetSemaphore(sem_data_ready_);
    resetSemaphore(sem_result_ready_);

    // 创建或附加共享内存
    size_t shm_size = sizeof(SharedData);
    shm_id_ = shmget(CLASSIFIER_SHM_KEY, shm_size, IPC_CREAT | 0666);
    if (shm_id_ == -1) {
        throw std::runtime_error("Failed to create shared memory");
    }
    
    attachSharedMemory();    
    
    // 初始化共享内存
    shared_data_->reserved0 = false; // is_processed 废弃
    shared_data_->show_windows = false;
    shared_data_->reserved2 = false;
    shared_data_->reserved3 = false;
    shared_data_->num_images = 0;
}

SharedMemoryClassifier::~SharedMemoryClassifier() {
    detachSharedMemory();
    // 关闭信号量 (注意：通常不在这里 unlink，防止影响其他正在运行的实例)
    sem_close(sem_data_ready_);
    sem_close(sem_result_ready_);
}

void SharedMemoryClassifier::attachSharedMemory() {
    shared_data_ = static_cast<SharedData*>(shmat(shm_id_, nullptr, 0));
    if (shared_data_ == reinterpret_cast<void*>(-1)) {
        throw std::runtime_error("Failed to attach shared memory");
    }
}

void SharedMemoryClassifier::detachSharedMemory() {
    if (shmdt(shared_data_) == -1) {
        // 处理错误但不抛出异常，防止在析构中产生问题
    }
}

std::vector<std::vector<float>> SharedMemoryClassifier::processImages(const std::vector<cv::Mat>& images) {
    // Python 端已确认不可用：快速降级，不再每帧等 1 秒；每 10 秒重试探测一次
    if (!python_available_) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_retry_ < std::chrono::seconds(10)) {
            return {};
        }
        last_retry_ = now;
    }

    // 1. 准备数据
    int num_images = std::min(images.size(), MAX_IMAGES);
    shared_data_->num_images = num_images;
    
    // 2. 复制图像数据到共享内存
    for (int i = 0; i < num_images; ++i) {
        // 验证图像格式和尺寸
        if (images[i].cols != 64 || images[i].rows != 48 || images[i].channels() != 3) {
            throw std::invalid_argument("Invalid image format. Expected 64x48 RGB");
        }
        
        // 复制连续的内存块
        size_t data_size = 64 * 48 * 3;
        if (images[i].isContinuous()) {
            std::memcpy(shared_data_->image_data[i], images[i].data, data_size);
        } else {
            cv::Mat continuous_img = images[i].clone();
            std::memcpy(shared_data_->image_data[i], continuous_img.data, data_size);
        }
    }

    shared_data_->show_windows = show_windows_;
    

    // 2. 【关键】数据写入完毕，释放内存屏障，通知 Python 端
    sem_post(sem_data_ready_);

    // 3. 【关键】等待 Python 端处理完成并通知，带 1 秒超时。
    // Python 端未启动/崩溃时降级返回空结果，避免永久阻塞。
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;
    if (sem_timedwait(sem_result_ready_, &ts) == -1) {
        if (errno == ETIMEDOUT) {
            python_available_ = false;
            last_retry_ = std::chrono::steady_clock::now();
            static std::chrono::steady_clock::time_point last_warn;
            auto now = std::chrono::steady_clock::now();
            if (now - last_warn > std::chrono::seconds(5)) {
                std::cerr << "[Classifier] timeout waiting python node, degrade to empty result"
                          << std::endl;
                last_warn = now;
            }
        }
        // 超时或其他错误：降级返回空结果
        return {};
    }
    python_available_ = true;  // Python 端恢复可用
    
    // 4. 读取处理结果
    std::vector<std::vector<float>> results;
    for (int i = 0; i < num_images; ++i) {
        results.emplace_back(
            shared_data_->results[i],
            shared_data_->results[i] + 12
        );
    }
    
    return results;
}
