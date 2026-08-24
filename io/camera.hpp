#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

// transistor 相机类（GigE/USB，自管重连+取流线程，把帧写到全局 g_image）
#include "camera/Camera.h"
#include "other_input/ImagesInput.h"
#include "other_input/VideoInput.h"

namespace io
{
// 输入源抽象：按 config 选择 视频/图片/相机，对外统一 read(img, t) 阻塞取帧。
// 对应原 ArmorDetect_Node 的 USE_VIDEO/USE_IMAGES/相机 三选一逻辑。
class Camera
{
public:
  explicit Camera(const std::string & config_path);
  ~Camera();
  Camera(const Camera &) = delete;
  Camera & operator=(const Camera &) = delete;

  // 阻塞到新帧就绪，再零拷贝换出（原 processImage 的 image_used + swap）
  bool read(cv::Mat & img,
            std::chrono::steady_clock::time_point & t,
            double & source_timestamp_s);

private:
  std::unique_ptr<::Camera> camera_;          // GigE/USB 相机
  std::shared_ptr<VideoInput> video_;         // 视频输入
  std::shared_ptr<ImagesInput> images_;       // 图片输入
};

}  // namespace io

#endif  // IO__CAMERA_HPP
