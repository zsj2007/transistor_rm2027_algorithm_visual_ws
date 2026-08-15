// ImagesInput.cpp
#include "other_input/ImagesInput.h"
#include <utility>  // std::swap

// 使用在camera.cpp中定义的全局变量
extern bool g_bExit;
extern cv::Mat g_image;
extern pthread_mutex_t g_mutex;
extern bool image_used;

ImagesInput::ImagesInput(const std::string& folderPath) : currentIndex(0) {
    // 获取文件夹中所有图片文件
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(folderPath.c_str())) != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            std::string filename = ent->d_name;
            // 检查常见图片扩展名
            if (filename.find(".jpg") != std::string::npos ||
                filename.find(".jpeg") != std::string::npos ||
                filename.find(".png") != std::string::npos ||
                filename.find(".bmp") != std::string::npos) {
                imagePaths.push_back(folderPath + "/" + filename);
            }
        }
        closedir(dir);
        
        // 按文件名排序
        std::sort(imagePaths.begin(), imagePaths.end());
        std::cout << "Found " << imagePaths.size() << " images in folder: " << folderPath << std::endl;
    } else {
        std::cerr << "Error: Could not open directory: " << folderPath << std::endl;
        exit(1);
    }

    // 启动取流线程
    pthread_create(&thread_id, NULL, workThread, this);
    std::cout << "Image grabbing thread started." << std::endl;
}

ImagesInput::~ImagesInput() {
    // 等待线程结束
    g_bExit = true;
    pthread_join(thread_id, NULL);
    std::cout << "ImagesInput released." << std::endl;
}

void* ImagesInput::workThread(void* pThis) {
    ImagesInput* pImages = (ImagesInput*)pThis;
    
    while (!g_bExit && pImages->currentIndex < pImages->imagePaths.size()) {
        // 读取当前图片
        cv::Mat frame = cv::imread(pImages->imagePaths[pImages->currentIndex]);
        
        if (frame.empty()) {
            std::cerr << "Warning: Could not read image: " 
                      << pImages->imagePaths[pImages->currentIndex] << std::endl;
            pImages->currentIndex++;
            continue;
        }

        // 等待上一帧被使用
        while (!image_used && !g_bExit) {
            usleep(1000); // 避免忙等待
        }

        // 更新全局图像
        pthread_mutex_lock(&g_mutex);
        std::swap(g_image, frame);  // 零拷贝交接：frame 持有自有内存（imread 分配）
        image_used = false;
        pthread_mutex_unlock(&g_mutex);
        
        std::cout << "Displaying image: " << pImages->imagePaths[pImages->currentIndex] << std::endl;
        pImages->currentIndex++;
        
        // 每张图片显示间隔（模拟帧率）
        // usleep(33333); // 约30FPS (1/30 ≈ 0.033秒)
    }
    
    if (pImages->currentIndex >= pImages->imagePaths.size()) {
        std::cout << "All images displayed. Exiting thread." << std::endl;
    } else {
        std::cout << "Image grabbing thread exiting by request." << std::endl;
    }
    return NULL;
}
