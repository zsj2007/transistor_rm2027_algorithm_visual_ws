#ifndef RP24_YOLO_WRAPPER_H
#define RP24_YOLO_WRAPPER_H

#include "RP24_YOLO/OpenvinoInfer.h"
#include "2d_armor_detector/Armor.h"
#include "2d_armor_detector/ArmorTracker.h"
#include "utils/ThreadPool.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <utility>

class RP24YOLOWrapper {
public:
    // 异步流水线的一帧结果
    struct YoloResult {
        uint64_t frame_id = 0;
        void* user_data = nullptr;      // 调用方自定义数据（Stage1 用它带回数据指针）
        vector<Armor> armors;
        vector<int> rp24_classes;
    };

    RP24YOLOWrapper(std::shared_ptr<YAML::Node> config_file_ptr, string model_path, string device);
    ~RP24YOLOWrapper();

    vector<Armor> detectArmors(cv::Mat& frame, string detect_color, vector<int>* rp24_classes = nullptr);
    vector<ArmorResult> detectArmorsWithClassifyAndTrack(cv::Mat& frame, string detect_color, 
        const cv::Point2f& ground_stable_point, vector<Armor>* armors_out = nullptr);

    // 异步接口：submit 立即返回，结果按提交顺序用 takeResult 取回
    uint64_t submitFrame(cv::Mat frame, int detect_color, void* user_data = nullptr);
    YoloResult takeResult(uint64_t frame_id);
    // 停止流水线（幂等）：关闭队列/寄存器并 join 三个线程
    void stop();
    bool tryTakeResult(YoloResult* out);   // 非阻塞取结果：有则 true，无则 false
    // 结果就绪回调：有结果推入 results_ 时触发（事件唤醒，替代调用方轮询）
    void setResultNotify(std::function<void()> notify);
    vector<ArmorResult> classifyAndTrack(vector<Armor> armors, const vector<int>& rp24_classes,
                                         const cv::Point2f& ground_stable_point);

private:
    // 一帧在流水线中流动的数据
    struct YoloWork {
        uint64_t frame_id = 0;
        void* user_data = nullptr;
        cv::Mat frame;              // 原图（postprocess 缩放用）
        int detect_color = -1;
        cv::Mat infer_input;        // preprocess 输出
        vector<Object> objects;     // infer 输出
        vector<int> rp24_classes;   // postprocess 输出
        vector<Armor> armors;       // postprocess 输出
    };

    // ---------- 阶段函数（原有逻辑，保持不变） ----------
    cv::Mat preprocess(const cv::Mat& frame);
    // OpenvinoInfer 内部请求池并发推理，无全局锁；wait_ms/infer_ms 为可选计时输出
    vector<Object> infer(const cv::Mat& input, int detect_color,
                         double* wait_ms = nullptr, double* infer_ms = nullptr);
    vector<Armor> postprocess(const cv::Mat& frame, const vector<Object>& objects, vector<int>* rp24_classes);

    // 线程池任务：一帧完整处理（preprocess -> infer -> postprocess -> 结果入队）
    void processOneFrame(std::shared_ptr<YoloWork> work);
    // 结果就绪时触发 result_notify_（锁外调用回调）
    void notifyResultAvailable();

    // ---------- 线程池流水线成员 ----------
    static constexpr size_t kMaxResults = 8;   // 结果队列上限（最新优先，满则丢最旧）
    std::deque<YoloResult> results_;           // 已完成的帧结果
    std::mutex results_mtx_;
    std::condition_variable results_cv_;
    std::function<void()> result_notify_;      // 受 results_mtx_ 保护
    std::atomic<size_t> pending_tasks_{0};     // 在飞任务数（stop 时等待归零）
    std::atomic<bool> stopping_{false};
    std::atomic<uint64_t> next_frame_id_{0};

    std::shared_ptr<OpenvinoInfer> openvino_infer;
    std::shared_ptr<YAML::Node> config_file_ptr;
    // YOLO 专用线程池：绑 P 核（cpu_pinning.yolo_cores），推理任务在此池执行；
    // OpenVINO 内部线程继承调用线程亲和，从而也落在 P 核（不依赖官方 hint）。
    // 注意：必须用全局限定 ::utils（OpenvinoInfer.h 的 using namespace cv 会把
    // 裸写 utils 解析成 cv::utils 命名空间，导致编译失败）。
    std::unique_ptr<::utils::ThreadPool> yolo_pool_;
    float lightBarLengthScale = 0.82;

    // 新模型（FasterNet-P345_pose，17类）：0-8=蓝(B)，9-16=红(R)
    // 类别语义: 0=G(基地) 1=1(英雄) 2=2(工程) 3=3 4=4 5=5 6=O(前哨站) 7=Bs(小符) 8=Bb(大符)
    // class_map: 类别 → ArmorType（Base=7, Hero=0, Engineer=1, Infantry1=2, Infantry2=3,
    //            Infantry3=4, Sentry=5, Outpost=6）
    int class_map_fasternet[17] = {
        7, 0, 1, 2, 3, 4, 6, 5, 7,
        7, 0, 1, 2, 3, 4, 6, 5
    };
    // big_map: 大装甲板（G基地 / 1英雄 / Bb大符），测试后按实际调整
    bool big_map_fasternet[17] = {
        true, true, false, false, false, false, false, false, true,
        true, true, false, false, false, false, false, false
    };
    // 旧 0526 模型（9类，颜色分列）：类别 → ArmorType 编号（同济 yolov5 同为该布局）
    int class_map_legacy[9] = {5, 0, 1, 2, 3, 4, 6, 7, 7};
    bool big_map_legacy[9] = {false, true, false, false, false, false, false, false, true};
    bool is_fasternet_model_ = false;   // 模型路径含 "fasternet" 时为 true（决定用哪套映射）

    // ---------- 模型族适配 ----------
    bool letterbox_ = false;     // 使用 letterbox 预处理（同济 yolov5 需要，RP24 保持拉伸）
    bool tongji_model_ = false;  // 同济 yolov5（0=蓝 1=红，第9类=非装甲板需过滤）
    std::shared_ptr<ArmorTracker> armor_tracker;
    int fix_armor_class_ = -1;
    int input_size_ = 640;   // 模型输入边长（配置 RP24_YOLO_input_size，必须与模型一致）
};

#endif  // RP24_YOLO_WRAPPER_H
