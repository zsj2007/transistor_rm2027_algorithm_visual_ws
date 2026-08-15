// FrameRateCounter.h
#ifndef FRAME_RATE_COUNTER_HPP
#define FRAME_RATE_COUNTER_HPP

#include <deque>
#include <chrono>
#include <algorithm>

class FrameRateCounter {
public:
    /**
     * @brief 构造函数
     * @param window_size 滑动平均窗口大小（帧数）
     */
    explicit FrameRateCounter(size_t window_size = 60);
    
    /**
     * @brief 在每帧结束时调用此方法
     */
    void tick();
    
    /**
     * @brief 获取当前平均帧率（FPS）
     * @return 平均帧率（帧/秒）
     */
    double fps() const;
    
    /**
     * @brief 获取平均帧耗时
     * @return 平均每帧耗时（秒）
     */
    double avg_frame_time() const;
    
    /**
     * @brief 重置计数器（清除所有历史数据）
     */
    void reset();
    
    /**
     * @brief 获取当前滑动窗口大小
     * @return 当前使用的窗口大小（帧数）
     */
    size_t window_size() const;
    
    /**
     * @brief 修改滑动窗口大小
     * @param new_size 新的窗口大小（帧数）
     */
    void set_window_size(size_t new_size);

private:
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    std::deque<double> frame_times_;  // 存储帧耗时的滑动窗口
    double time_sum_ = 0.0;           // 窗口内帧耗时总和
    size_t window_size_;               // 滑动窗口大小（帧数）
};

#endif // FRAME_RATE_COUNTER_HPP