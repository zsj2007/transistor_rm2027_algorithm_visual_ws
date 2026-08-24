#include "io/camera.hpp"

#include <filesystem>
#include <unistd.h>  // usleep

#include "tools/exiter.hpp"
#include "tools/yaml.hpp"
#include "other_input/FramePacket.h"

// 三个输入源共用的全局帧交接区（原 ArmorDetect_Node.cpp 里的全局变量，
// 现在收进 io/camera 模块，由 io::Camera::read() 消费）
FramePacket g_frame_packet;
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_bExit = false;
bool image_used = true;

namespace io
{
namespace fs = std::filesystem;

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  bool use_video = yaml["USE_VIDEO"] ? yaml["USE_VIDEO"].as<bool>() : false;
  bool use_images = yaml["USE_IMAGES"] ? yaml["USE_IMAGES"].as<bool>() : false;
  double configured_fps = yaml["frame_rate"] ? yaml["frame_rate"].as<double>() : 30.0;
  // 相对路径以 config 所在目录的上一级（项目根）为基准
  fs::path base = fs::path(config_path).parent_path().parent_path();

  if (use_video) {
    auto rel = tools::read<std::string>(yaml, "video_relative_path");
    video_ = std::make_shared<VideoInput>((base / rel).string(), configured_fps);
  } else if (use_images) {
    auto rel = tools::read<std::string>(yaml, "images_relative_path");
    images_ = std::make_shared<ImagesInput>((base / rel).string(), configured_fps);
  } else {
    auto cam_ip = tools::read<std::string>(yaml, "cam_ip");
    auto pc_ip = tools::read<std::string>(yaml, "pc_ip");
    camera_ = std::make_unique<::Camera>(cam_ip, pc_ip);
    if (yaml["camera_ExposureTime"])
      camera_->setExposureTime(yaml["camera_ExposureTime"].as<float>());
    if (yaml["camera_Gain"]) camera_->setGain(yaml["camera_Gain"].as<float>());
    camera_->start();
  }
}

Camera::~Camera()
{
  g_bExit = true;  // 通知取流线程退出（原节点析构里同样置位）
  if (camera_) camera_->stop();
  pthread_mutex_destroy(&g_mutex);
}

bool Camera::read(cv::Mat & img,
                  std::chrono::steady_clock::time_point & t,
                  double & source_timestamp_s)
{
  // Wait first, then read image + source-time metadata in the same critical
  // section. The old code timestamped before waiting and could pair a frame
  // with the wrong time.
  while (image_used && !g_bExit && !tools::exitRequested()) {
    usleep(1000);
  }

  bool got_frame = false;
  pthread_mutex_lock(&g_mutex);
  if (!g_frame_packet.image.empty()) {
    std::swap(img, g_frame_packet.image);
    source_timestamp_s = g_frame_packet.timestamp_s;

    if (camera_) {
      // Live-camera timestamps are steady_clock epoch seconds recorded at SDK
      // frame arrival, so reconstruct the same time point for gimbal alignment.
      const auto d = std::chrono::duration<double>(source_timestamp_s);
      t = std::chrono::steady_clock::time_point(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(d));
    } else {
      // Offline video/image source time starts from zero and must NOT be mixed
      // with steady_clock. Use current steady time only for queue/gimbal plumbing.
      t = std::chrono::steady_clock::now();
    }

    image_used = true;
    got_frame = true;
  }
  pthread_mutex_unlock(&g_mutex);

  return got_frame;
}

}  // namespace io
