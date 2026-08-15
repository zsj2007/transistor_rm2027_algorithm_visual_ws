#include "logger/TwoVideoLogger.h"
#include <sys/statvfs.h>

namespace fs = std::filesystem;


// 获取指定路径所在文件系统的可用空间（字节）
long long get_available_space(const std::string& path) {
    struct statvfs stat;
    if (statvfs(path.c_str(), &stat) != 0) {
        std::cerr << "statvfs failed: " << std::strerror(errno) << std::endl;
        return -1;
    }
    // 可用块数 * 块大小 = 可用字节数
    return static_cast<long long>(stat.f_bavail) * stat.f_frsize;
}

TwoVideoLogger::TwoVideoLogger(const std::string& log_folder_str, bool log_origin_video, bool log_result_video)
    : log_origin_video_(log_origin_video), log_result_video_(log_result_video) {

    auto system_clock_now = std::chrono::system_clock::now();
    std::time_t system_clock_now_t = std::chrono::system_clock::to_time_t(system_clock_now);
    std::tm* system_clock_now_tm = std::localtime(&system_clock_now_t);
    char system_clock_now_str_buffer[80];
    memset(system_clock_now_str_buffer, 0, 80);
    std::strftime(system_clock_now_str_buffer, sizeof(system_clock_now_str_buffer), "%Y-%m-%d_%H-%M-%S", system_clock_now_tm);
    std::string system_clock_now_str(system_clock_now_str_buffer);


    fs::path log_folder_path(log_folder_str);
    fs::create_directories(log_folder_path);
    this_log_folder_path = log_folder_path / system_clock_now_str;
    fs::create_directories(this_log_folder_path);
    origin_video_path = this_log_folder_path / "origin.mkv";
    info_video_path = this_log_folder_path / "info.mkv";

    if (log_origin_video_) {
        origin_video_writer = std::make_shared<MkvAllIntraWriter>();
        origin_video_writer->open(origin_video_path.string(), 1280, 1024, 30.0, (int)50e6);
    }
    if (log_result_video_) {
        info_video_writer = std::make_shared<MkvAllIntraWriter>();
        info_video_writer->open(info_video_path.string(), 1280 + 800, 800 * 2, 30.0, (int)20e6);
    }
}

void TwoVideoLogger::updateOriginFrame(const cv::Mat& frame) {
    frames.origin_frame = frame.clone();
}

void TwoVideoLogger::updateDrewFrame(const cv::Mat& frame) {
    frames.drew_frame = frame.clone();
}

void TwoVideoLogger::updateRMMFrame(const cv::Mat& frame) {
    frames.rmm_frame = frame.clone();
}

void TwoVideoLogger::updateCDOFrame(const cv::Mat& frame) {
    frames.cdo_frame = frame.clone();
}

void TwoVideoLogger::updateYawFrame(const cv::Mat& frame) {
    frames.yaw_frame = frame.clone();
}

void TwoVideoLogger::updateComFrame(const cv::Mat& frame) {
    frames.com_frame = frame.clone();
}

void TwoVideoLogger::writeTwoFrame() {
    frame_count += 1;

    cv::Mat info_frame = cv::Mat::zeros(800*2, 1280+800, CV_8UC3);

    cv::Mat drew_roi = info_frame(cv::Rect(0, 0, 1280, 1024));
    frames.drew_frame.copyTo(drew_roi);
    if (!frames.rmm_frame.empty()) {
        cv::Mat rmm_roi = info_frame(cv::Rect(1280, 0, 800, 800));
        frames.rmm_frame.copyTo(rmm_roi);
    }
    if (!frames.cdo_frame.empty()) {
        cv::Mat cdo_roi = info_frame(cv::Rect(640, 1024, 640, 480));
        frames.cdo_frame.copyTo(cdo_roi);
    }
    if (!frames.yaw_frame.empty()) {
        cv::Mat yaw_roi = info_frame(cv::Rect(1280, 800, 800, 800));
        frames.yaw_frame.copyTo(yaw_roi);
    }
    if (!frames.com_frame.empty()) {
        cv::Mat yaw_roi = info_frame(cv::Rect(0, 1024, 640, 480));
        frames.com_frame.copyTo(yaw_roi);
    }
    cv::putText(info_frame, 
        cv::format("frame_count: %ld", frame_count), 
        cv::Point(20, 1600-50),
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        cv::Scalar(0, 255, 0), 1, 8, false);


    long long max_left_size = 16ll * 1024ll*1024ll*1024ll; // 最少留下16G，防止系统无法启动
    if (get_available_space(this_log_folder_path.string()) > max_left_size) {
        if (log_origin_video_ && origin_video_writer) {
            origin_video_writer->writeFrame(frames.origin_frame);
        }
        if (log_result_video_ && info_video_writer) {
            info_video_writer->writeFrame(info_frame);
        }
    }
}