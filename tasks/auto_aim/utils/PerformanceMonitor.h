#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using PerfClock = std::chrono::steady_clock;
using PerfTimePoint = PerfClock::time_point;

struct FrameProfile {
    uint64_t frame_id = 0;
    PerfTimePoint frame_start_time;
    std::unordered_map<std::string, double> stages;
    double total_ms = 0.0;
};

class PerformanceMonitor {
public:
    explicit PerformanceMonitor(bool enabled = false, size_t report_interval = 90);

    bool enabled() const;
    void setEnabled(bool enabled);

    FrameProfile beginFrame(uint64_t frame_id,
                            PerfTimePoint start_time = PerfClock::now()) const;
    void recordStage(FrameProfile& profile,
                     const std::string& stage,
                     PerfTimePoint start_time,
                     PerfTimePoint end_time = PerfClock::now()) const;
    void endFrame(FrameProfile& profile,
                  PerfTimePoint end_time = PerfClock::now());

    void printStatistics();
    void reset();

    static double durationMs(PerfTimePoint start_time, PerfTimePoint end_time);

private:
    void printStatisticsLocked() const;

    std::atomic<bool> enabled_;
    size_t report_interval_;
    PerfTimePoint start_time_;   // 构造时刻，用于报告里打印进程运行时长
    mutable std::mutex history_mtx_;
    std::vector<FrameProfile> history_;
};
