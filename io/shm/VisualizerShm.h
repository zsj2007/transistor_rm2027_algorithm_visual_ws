// io/shm/VisualizerShm.h
// 共享内存可视化桥：替代 ROS2 话题的调试可视化通道。
//
// 进程模型：
//   - writer：自瞄算法进程（AutoAimPipeline Stage4）。每帧把原始画面、
//     灯条/装甲板/解算结果/云台角度等调试数据写入共享内存，并用信号量通知。
//   - reader：独立可视化进程（users/visualizer.cpp）。等待通知后把整帧
//     拷贝到本进程内存，再用 OpenCV 绘制窗口（内容与 2026 的
//     auto_aim_visualizer 节点保持一致）。
//
// 设计要点：
//   - 全部固定大小数组（无指针/STL），跨进程安全；
//   - frame_id 最后写入 + 读取前后校验，避免读到半帧；
//   - 没有 reader 挂接时 writer 跳过整帧发布，避免无谓的大块 memcpy；
//   - 布局与旧 ROS 消息字段一一对应，方便对照迁移。
#ifndef VISUALIZER_SHM_H
#define VISUALIZER_SHM_H

#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <string>

#include <fcntl.h>
#include <semaphore.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace visualizer_shm {

// ---- 固定容量上限（超出部分截断，仅影响可视化，不影响算法）----
constexpr int kMaxLights = 64;
constexpr int kMaxArmors = 32;
constexpr int kMaxSolvedResults = 16;
constexpr int kMaxJointEkfPairs = 8;
constexpr int kMaxPredictionsPerArmor = 64;
constexpr int kMaxEnemyColorLen = 16;

// 原始画面缓冲上限（当前相机 1280x1024，留余量到 1920x1080）
constexpr int kMaxRawImageBytes = 1920 * 1080 * 3;
// RMM / 示波器等辅助画面（800x800 / 800x400，留余量到 1280x1024）
constexpr int kMaxAuxImageBytes = 1280 * 1024 * 3;

constexpr key_t kDefaultShmKey = 0x0251;
constexpr const char* kSemaphoreName = "/transistor_rm_visualizer_ready";

// 共享内存布局魔数（校验旧段/版本不匹配）
constexpr uint32_t kMagic = 0x56495352;  // "VISR"

struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

struct Light {
    Point2f vertices[4];
};

struct Armor {
    Point2f corners[4];
    Point2f light_bar_corners[4];
    float confidence = 0.0f;
};

struct ArmorResult {
    Point2f corners[4];
    Point2f light_bar_corners[4];
    Point2f center;
    Point2f center_predicted;
    Point2f predictions[kMaxPredictionsPerArmor];
    uint32_t prediction_count = 0;
    int32_t number = 0;
    float confidence = 0.0f;
    bool is_tracked_now = false;
};

struct JointEkfPair {
    Point2f center_a;
    Point2f center_b;
    int32_t number = -1;
    int32_t track_id_a = -1;
    int32_t track_id_b = -1;
    int32_t consecutive_frames = 0;
    bool ready = false;
};

struct RawImage {
    int32_t cols = 0;
    int32_t rows = 0;
    int32_t type = 0;
    uint32_t bytes = 0;
    uint8_t data[kMaxRawImageBytes];
};

struct AuxImage {
    int32_t cols = 0;
    int32_t rows = 0;
    int32_t type = 0;
    uint32_t bytes = 0;
    uint8_t data[kMaxAuxImageBytes];
};

// 对应旧 auto_aim/msg/VisualizerDebugData.msg 的字段
struct DebugData {
    float bullet_velocity = 0.0f;
    char enemy_color[kMaxEnemyColorLen] = {};
    float pitch = 0.0f;
    float yaw = 0.0f;
    float roll = 0.0f;
    float mcu_command_yaw = 0.0f;
    uint8_t armor_type = 0;
    uint8_t predictor_type = 0;
    Point2f ground_stable_point;

    Light lights[kMaxLights];
    uint32_t light_count = 0;

    Armor armors[kMaxArmors];
    uint32_t armor_count = 0;

    ArmorResult solved_results[kMaxSolvedResults];
    uint32_t solved_count = 0;

    JointEkfPair joint_ekf_pairs[kMaxJointEkfPairs];
    uint32_t joint_ekf_count = 0;
};

struct Snapshot {
    // 单调递增帧号，发布时最后写入；读取侧用它校验整帧一致性
    uint64_t frame_id = 0;
    // writer 稳态时钟毫秒，reader 可据此判断数据是否新鲜
    int64_t writer_timestamp_ms = 0;
    // writer 侧实测的发布帧率（滚动 1s 窗口，等价于算法流水线吞吐）
    float writer_fps = 0.0f;

    DebugData debug;
    RawImage raw_frame;
    AuxImage rmm_frame;
    AuxImage cdo_frame;
};

struct SharedLayout {
    uint32_t magic = 0;
    uint32_t layout_size = 0;
    bool reader_attached = false;
    Snapshot snapshot;
};

inline key_t shmKeyFromConfig(const std::shared_ptr<YAML::Node>& config_file_ptr)
{
    if (config_file_ptr && (*config_file_ptr)["visualizer"] &&
        (*config_file_ptr)["visualizer"]["shm_key"]) {
        return (*config_file_ptr)["visualizer"]["shm_key"].as<int>();
    }
    return kDefaultShmKey;
}

// 清掉信号量里残留的计数（进程崩溃后可能遗留，导致 reader 空转）
inline void resetSemaphore(sem_t* sem)
{
    while (sem_trywait(sem) == 0) {
        // 成功减一，说明还有残留计数，继续清
    }
}

}  // namespace visualizer_shm

// 算法侧写入器：Stage4 每帧 publish() 一次
class VisualizerShmWriter {
public:
    explicit VisualizerShmWriter(const std::shared_ptr<YAML::Node>& config_file_ptr);
    ~VisualizerShmWriter();

    bool valid() const { return valid_; }

    // 发布一帧。debug 由调用方填好；raw/rmm/cdo 可为空 Mat（跳过对应图像）。
    // 没有 reader 挂接时直接返回 true，不拷贝图像。
    bool publish(const visualizer_shm::DebugData& debug,
                 const cv::Mat& raw_frame,
                 const cv::Mat& rmm_frame,
                 const cv::Mat& cdo_frame);

private:
    bool valid_ = false;
    int shm_id_ = -1;
    visualizer_shm::SharedLayout* shared_ = nullptr;
    sem_t* sem_ = SEM_FAILED;
    uint64_t next_frame_id_ = 1;
    std::deque<int64_t> publish_times_ms_;
};

// 可视化侧读取器：等待新帧并拷贝到本进程内存
class VisualizerShmReader {
public:
    explicit VisualizerShmReader(const std::shared_ptr<YAML::Node>& config_file_ptr);
    ~VisualizerShmReader();

    bool valid() const { return valid_; }

    // 阻塞等待新一帧（最多 timeout_ms）。有则拷贝到 out 并返回 true；
    // 超时或无新帧返回 false。
    bool waitForSnapshot(visualizer_shm::Snapshot& out, int timeout_ms = 500);

private:
    bool valid_ = false;
    int shm_id_ = -1;
    visualizer_shm::SharedLayout* shared_ = nullptr;
    sem_t* sem_ = SEM_FAILED;
    uint64_t last_frame_id_ = 0;
};

#endif  // VISUALIZER_SHM_H
