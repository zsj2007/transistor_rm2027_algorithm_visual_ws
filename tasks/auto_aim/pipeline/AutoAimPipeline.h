#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "2d_armor_detector/Armor.h"
#include "2d_armor_detector/ArmorClassifier.h"
#include "2d_armor_detector/ArmorDetector.h"
#include "2d_armor_detector/LightBar.h"
#include "2d_armor_detector/LightBarDetector.h"
#include "3d_processing/ArmorSolver.h"
#include "3d_processing/BallisticSolver.h"
#include "3d_processing/RestFrame.h"
#include "RP24_YOLO/RP24_YOLO_Wrapper.h"
#include "logger/TwoVideoLogger.h"
#include "predictor/PredictorMain.h"
#include "shm/VisualizerShm.h"
#include "utils/FrameRateCounter.h"
#include "utils/PerformanceMonitor.h"
#include "utils/VisualizerConfig.h"

struct JointEkfPair {
    int number = -1;
    int track_id_a = -1;
    int track_id_b = -1;
    cv::Point2f center_a;
    cv::Point2f center_b;
    int consecutive_frames = 0;
    bool ready = false;
};

struct AutoAimVisualizerDebugFrame {
    cv::Mat frame;
    std::chrono::steady_clock::time_point node_start_time;

    float bullet_velocity = 0.0f;
    std::string enemy_color;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float roll = 0.0f;
    cv::Point2f ground_stable_point;

    std::vector<Light> lights;
    std::vector<Armor> armors;
    std::vector<ArmorResult> solved_results;
    std::vector<JointEkfPair> joint_ekf_pairs;
    ArmorType::ArmorType armor_type = ArmorType::Hero;
    PredictorType::PredictorType predictor_type = PredictorType::None;
    float mcu_command_yaw = 0.0f;
};

struct AutoAimPipelineData {
    struct InitialData {
        cv::Mat frame;
        cv::Mat com_data_visualize_frame;
        std::chrono::steady_clock::time_point frame_timestamp;
        // EKF 使用输入帧时间；视频模式按固定帧率递增，不受处理速度影响。
        double source_timestamp_s = 0.0;
        std::chrono::steady_clock::time_point node_start_time;
        std::chrono::steady_clock::time_point performance_start_time;
        std::shared_ptr<FrameProfile> performance_profile;

        float bullet_velocity = 0.0f;
        std::string enemy_color;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float total_yaw = 0.0f;
        float roll = 0.0f;
        cv::Point2f ground_stable_point;

        bool auto_aim_switch = true;
        bool use_head_imu = false;
        bool mcu_yaw_online = true;
        float to_mcu_delta_yaw = 0.0f;
        float to_mcu_delta_pitch = 0.0f;
    } initial;

    struct Stage1Data {
        std::vector<Light> lights;
        std::vector<Armor> armors;
        std::vector<ArmorResult> classify_results;
        std::vector<JointEkfPair> joint_ekf_pairs;
        bool used_yolo = false;
    } stage1;

    struct Stage2Data {
        std::vector<ArmorResult> solved_results;
        std::vector<cv::Point3f> rest_frame_positions;
        size_t valid_count = 0;
    } stage2;

    struct Stage3Data {
        PredictorResult predictor_result;
        float mcu_command_pitch = 0.0f;
        float mcu_command_yaw = 0.0f;
        bool should_send_reset = false;
    } stage3;

    struct Stage4Data {
        cv::Mat display;
        cv::Mat yaw_visualizer_frame;
        cv::Mat rmm_visualize_frame;
        cv::Mat common_debug_oscilloscope_frame;
        AutoAimVisualizerDebugFrame visualizer_debug_frame;
        size_t armor_count = 0;
        bool request_com_frame_refresh = false;
    } stage4;
};

class AutoAimPipeline {
public:
    struct AlwaysValidData {
        int queue_input = 0;
        int queue_inter0 = 0;
        int queue_inter1 = 0;
        int queue_inter2 = 0;
        int queue_output = 0;
    };

    struct ValidData {
        PredictorResult predictor_result;
        float mcu_command_pitch = 0.0f;
        float mcu_command_yaw = 0.0f;
        bool should_send_reset = false;
        cv::Mat display;
        cv::Mat yaw_visualizer_frame;
        cv::Mat rmm_visualize_frame;
        cv::Mat common_debug_oscilloscope_frame;
        AutoAimVisualizerDebugFrame visualizer_debug_frame;
        size_t armor_count = 0;
        bool request_com_frame_refresh = false;
    };

    struct ProcessResult {
        ValidData valid_data;
        AlwaysValidData always_valid_data;
        bool valid = false;
    };

    AutoAimPipeline(std::shared_ptr<YAML::Node> config_file_ptr,
                    const std::filesystem::path& workspace_path,
                    std::chrono::steady_clock::time_point node_start_time,
                    std::shared_ptr<PerformanceMonitor> performance_monitor = nullptr,
                    int max_queue_size = 4,
                    float max_delay_seconds = 0.0f);
    ~AutoAimPipeline();

    void addFrame(AutoAimPipelineData::InitialData initial);
    ProcessResult tryPopResult(const std::chrono::steady_clock::time_point& timestamp);
    void resetYawIntegration();

private:
    static constexpr int NUM_STAGES = 4;
    static constexpr int NUM_QUEUES = NUM_STAGES + 1;

    int max_queue_size_;
    float max_delay_seconds_;
    std::shared_ptr<PerformanceMonitor> performance_monitor_;
    uint64_t performance_frame_id_ = 0;

    std::deque<std::unique_ptr<AutoAimPipelineData>> input_queue_;
    std::mutex input_mtx_;
    std::condition_variable input_cv_;

    std::deque<std::unique_ptr<AutoAimPipelineData>> inter_queues_[NUM_STAGES - 1];
    std::deque<std::unique_ptr<AutoAimPipelineData>> output_queue_;
    std::mutex output_mtx_;

    std::unique_ptr<AutoAimPipelineData> in_flight_[NUM_STAGES];
    std::atomic<int> queue_sizes_[NUM_QUEUES];

    struct Stage1 {
        std::shared_ptr<LightBarDetector> light_detector;
        std::shared_ptr<ArmorDetector> armor_detector;
        std::shared_ptr<ArmorClassifier> classifier;
        std::shared_ptr<RP24YOLOWrapper> rp24_yolo_wrapper;
        bool use_rp24_yolo = false;
        bool joint_ekf_pair_enabled = false;
        int joint_ekf_pair_min_consecutive_frames = 2;

        struct JointEkfPairState {
            int track_id_a = -1;
            int track_id_b = -1;
            int consecutive_frames = 0;
        };
        std::map<int, JointEkfPairState> joint_ekf_pair_states;

        std::thread worker;
        std::atomic<bool> idle{true};
        mutable std::mutex mtx;   // inflightCount() 是 const，锁需要 mutable
        std::condition_variable cv;
        AutoAimPipelineData* data = nullptr;
        bool exit_flag = false;

        // ---- 异步路径（use_rp24_yolo = true）----
        std::deque<std::unique_ptr<AutoAimPipelineData>> inbox_;   // 待提交给 YOLO 的帧
        bool result_wakeup_ = false;   // 受 mtx_ 保护：YOLO 结果就绪时置位（事件唤醒用）
        struct InFlight {
            std::unique_ptr<AutoAimPipelineData> data;
            std::chrono::steady_clock::time_point submitted;       // 提交时刻（算延迟）
        };
        std::deque<InFlight> in_flight_;                           // 已提交、结果未回
        std::deque<std::unique_ptr<AutoAimPipelineData>> done_;    // 结果已回、待交给 stage2
        // 乱序完成的结果缓存：线程池任务可能乱序完成，按提交顺序消费前先暂存。
        // 关键：不能提前 drop 在飞帧——任务的 user_data 指向帧数据，提前销毁会悬垂。
        std::unordered_map<void*, RP24YOLOWrapper::YoloResult> pending_results_;

        Stage1(std::shared_ptr<YAML::Node> config_file_ptr,
               const std::filesystem::path& workspace_path);
        void start(AutoAimPipelineData& d);
        bool isIdle() const;
        void run();
        // 异步路径接口（调度线程调用）
        void pushInput(std::unique_ptr<AutoAimPipelineData> d);
        std::unique_ptr<AutoAimPipelineData> popDone();
        size_t inflightCount() const;
        // 异步路径内部
        void runAsync();
        void runLegacy();
        void drainResults();
        void finishFrame(RP24YOLOWrapper::YoloResult& res, std::chrono::steady_clock::time_point now);
        void flushAll();
        void runJointEkfPairGate(AutoAimPipelineData& d);
    } stage1_;

    struct Stage2 {
        std::shared_ptr<ArmorSolver> armor_solver;
        std::shared_ptr<RestFrame> rest_frame;
        float max_armor_position_height = 0.0f;

        std::thread worker;
        std::atomic<bool> idle{true};
        std::mutex mtx;
        std::condition_variable cv;
        AutoAimPipelineData* data = nullptr;
        bool exit_flag = false;

        Stage2(std::shared_ptr<YAML::Node> config_file_ptr);
        void start(AutoAimPipelineData& d);
        bool isIdle() const;
        void run();
    } stage2_;

    struct Stage3 {
        std::shared_ptr<ArmorSolver> armor_solver;
        std::shared_ptr<BallisticSolver> ballistic_solver;
        std::shared_ptr<RestFrame> rest_frame;
        std::shared_ptr<FrameRateCounter> fps_counter;
        std::shared_ptr<PredictorMain> predictor_main;

        std::thread worker;
        std::atomic<bool> idle{true};
        std::mutex mtx;
        std::condition_variable cv;
        AutoAimPipelineData* data = nullptr;
        bool exit_flag = false;

        Stage3(std::shared_ptr<YAML::Node> config_file_ptr,
               std::chrono::steady_clock::time_point node_start_time);
        void start(AutoAimPipelineData& d);
        bool isIdle() const;
        void run();
        void resetYawIntegration();
    } stage3_;

    struct Stage4 {
        std::shared_ptr<TwoVideoLogger> two_video_logger;
        VisualizerConfig visualizer_config;
        std::unique_ptr<VisualizerShmWriter> shm_writer_;
        bool log_result_video = false;
        bool log_origin_video = false;

        std::thread worker;
        std::atomic<bool> idle{true};
        std::mutex mtx;
        std::condition_variable cv;
        AutoAimPipelineData* data = nullptr;
        bool exit_flag = false;

        Stage4(std::shared_ptr<YAML::Node> config_file_ptr,
               const std::filesystem::path& workspace_path);
        void start(AutoAimPipelineData& d);
        bool isIdle() const;
        void run();
    } stage4_;

    std::thread scheduler_thread_;
    std::atomic<bool> scheduler_exit_{false};

    void schedulerLoop();
    void updateQueueSizes();
};
