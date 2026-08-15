#include "logger/MkvWriter.h"
#include "memory"
#include <filesystem>
#include <chrono>

class TwoVideoLogger {

private:

    std::shared_ptr<MkvAllIntraWriter> origin_video_writer;
    std::shared_ptr<MkvAllIntraWriter> info_video_writer;
    bool log_origin_video_ = false;
    bool log_result_video_ = false;

    std::filesystem::path this_log_folder_path;
    std::filesystem::path origin_video_path;
    std::filesystem::path info_video_path;

    struct {
        cv::Mat origin_frame;
        cv::Mat drew_frame;
        cv::Mat rmm_frame;
        cv::Mat cdo_frame;
        cv::Mat yaw_frame;
        cv::Mat com_frame;
    } frames;

    int64_t frame_count = 0;

public:
    TwoVideoLogger(const std::string& log_folder_str, bool log_origin_video, bool log_result_video);

    ~TwoVideoLogger() {}

    void updateOriginFrame(const cv::Mat& frame);
    void updateDrewFrame(const cv::Mat& frame);
    void updateRMMFrame(const cv::Mat& frame);
    void updateCDOFrame(const cv::Mat& frame);
    void updateYawFrame(const cv::Mat& frame);
    void updateComFrame(const cv::Mat& frame);

    void writeTwoFrame();
};