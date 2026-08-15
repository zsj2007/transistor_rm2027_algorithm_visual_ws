#include "utils/PerformanceMonitor.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>

PerformanceMonitor::PerformanceMonitor(bool enabled, size_t report_interval)
    : enabled_(enabled)
    , report_interval_(report_interval == 0 ? 1 : report_interval)
    , start_time_(PerfClock::now())
{
}

bool PerformanceMonitor::enabled() const
{
    return enabled_.load();
}

void PerformanceMonitor::setEnabled(bool enabled)
{
    enabled_.store(enabled);
}

FrameProfile PerformanceMonitor::beginFrame(uint64_t frame_id,
                                            PerfTimePoint start_time) const
{
    FrameProfile profile;
    profile.frame_id = frame_id;
    profile.frame_start_time = start_time;
    return profile;
}

void PerformanceMonitor::recordStage(FrameProfile& profile,
                                     const std::string& stage,
                                     PerfTimePoint start_time,
                                     PerfTimePoint end_time) const
{
    if (!enabled()) return;
    profile.stages[stage] += durationMs(start_time, end_time);
}

void PerformanceMonitor::endFrame(FrameProfile& profile,
                                  PerfTimePoint end_time)
{
    if (!enabled()) return;

    profile.total_ms = durationMs(profile.frame_start_time, end_time);

    std::lock_guard<std::mutex> lock(history_mtx_);
    history_.push_back(profile);
    if (history_.size() >= report_interval_) {
        printStatisticsLocked();
        history_.clear();
    }
}

double PerformanceMonitor::durationMs(PerfTimePoint start_time,
                                      PerfTimePoint end_time)
{
    return std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

void PerformanceMonitor::printStatistics()
{
    std::lock_guard<std::mutex> lock(history_mtx_);
    printStatisticsLocked();
}

void PerformanceMonitor::reset()
{
    std::lock_guard<std::mutex> lock(history_mtx_);
    history_.clear();
}

void PerformanceMonitor::printStatisticsLocked() const
{
    if (history_.empty()) return;

    // 期望顺序：阶段1（含 YOLO 子阶段）-> 阶段2 -> 阶段3 -> 阶段4，
    // 其余新出现的阶段（如 stage1_classify_track）按出现顺序排在后面
    const std::vector<std::string> preferred_order = {
        "stage1_2d_detect_classify",
        "stage1_yolo_latency",
        "yolo_preprocess",
        "yolo_infer",
        "yolo_infer_wait",
        "yolo_postprocess",
        "stage2_3d_solve_transform",
        "stage3_predict_command",
        "stage4_visualize_log",
    };

    std::unordered_map<std::string, double> stage_total;
    std::unordered_map<std::string, double> stage_min;
    std::unordered_map<std::string, double> stage_max;
    double total_time = 0.0;
    uint64_t frame_id_first = 0, frame_id_last = 0;

    for (size_t i = 0; i < history_.size(); ++i) {
        const auto& frame = history_[i];
        if (i == 0) frame_id_first = frame.frame_id;
        frame_id_last = frame.frame_id;
        total_time += frame.total_ms;
        for (const auto& stage : frame.stages) {
            stage_total[stage.first] += stage.second;
            const auto min_it = stage_min.find(stage.first);
            if (min_it == stage_min.end()) {
                stage_min[stage.first] = stage.second;
                stage_max[stage.first] = stage.second;
            } else {
                min_it->second = std::min(min_it->second, stage.second);
                stage_max[stage.first] = std::max(stage_max[stage.first], stage.second);
            }
        }
    }

    const double frame_count = static_cast<double>(history_.size());

    // 报告时间戳：进程运行时长（单调钟）+ 当前墙钟时间
    const double uptime_s = std::chrono::duration<double>(
        PerfClock::now() - start_time_).count();
    const std::time_t now_wall = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &now_wall);
#else
    localtime_r(&now_wall, &tm_buf);
#endif

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n========== Auto Aim Performance ==========\n";
    std::cout << "Report time: " << std::put_time(&tm_buf, "%H:%M:%S")
              << " (uptime " << uptime_s << " s)\n";
    std::cout << "Frames: " << history_.size()
              << " (id " << frame_id_first << " .. " << frame_id_last << ")\n";

    // 收集要打印的阶段：优先 preferred_order，其余按出现顺序追加
    std::vector<std::string> ordered;
    ordered.reserve(stage_total.size());
    for (const auto& name : preferred_order) {
        if (stage_total.find(name) != stage_total.end()) {
            ordered.push_back(name);
        }
    }
    for (const auto& kv : stage_total) {
        if (std::find(ordered.begin(), ordered.end(), kv.first) == ordered.end()) {
            ordered.push_back(kv.first);
        }
    }

    for (const auto& name : ordered) {
        const double avg_ms = stage_total[name] / frame_count;
        std::cout << name << ": avg " << avg_ms << " ms"
                  << " [min " << stage_min[name]
                  << ", max " << stage_max[name] << "]\n";
    }

    const double avg_total = total_time / frame_count;
    std::cout << "------------------------------------------\n";
    std::cout << "Total(from data init): " << avg_total << " ms\n";
    if (avg_total > 0.0) {
        std::cout << "FPS: " << 1000.0 / avg_total << "\n";
    }
    std::cout << "==========================================\n\n" << std::flush;
}
