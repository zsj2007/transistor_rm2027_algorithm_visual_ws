#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>

// Image data and its source-time metadata are published and consumed together
// while holding g_mutex. This prevents mixing metadata from adjacent frames.
struct FramePacket {
    cv::Mat image;
    double timestamp_s = 0.0;
    std::uint64_t frame_id = 0;
};

