#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>

// Image data and its source-time metadata are published and consumed together
// while holding g_mutex. This prevents mixing metadata from adjacent frames.
struct FramePacket {
    cv::Mat image;
    // Timestamp used by the pose-history query. Live Hikrobot frames use the
    // camera device clock mapped into steady_clock; other sources use their
    // existing source-time semantics.
    double timestamp_s = 0.0;
    std::uint64_t frame_id = 0;

    // Live-camera timestamp diagnostics. These stay zero/false for offline
    // video and image inputs.
    double arrival_timestamp_s = 0.0;
    std::uint64_t device_timestamp_ticks = 0;
    std::int64_t sdk_host_timestamp = 0;
    double exposure_time_us = 0.0;
    double clock_sync_uncertainty_us = 0.0;
    bool timestamp_from_device = false;
};
