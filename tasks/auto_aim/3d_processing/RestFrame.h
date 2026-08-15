// RestFrame.h
#ifndef REST_FRAME_H
#define REST_FRAME_H

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

class RestFrame {
public:
    RestFrame() : camera_yaw(0), camera_pitch(0), camera_roll(0), camera_x(0), camera_y(0), camera_z(0) {}
    ~RestFrame() {}
    
    void updateCamOrientation(float yaw, float pitch, float roll);
    void updateCamPosition(float x, float y, float z);

    std::vector<float> getCamOrientation();
    std::vector<float> getCamPosition();

    std::vector<float> pnpResultToNormalFrame(float x_pnp, float y_pnp, float z_pnp);
    std::vector<float> normalToPnpResultFrame(float x_cam_normal, float y_cam_normal, float z_cam_normal);

    std::vector<float> getWorldPositionFromCam(float x_cam_normal, float y_cam_normal, float z_cam_normal);
    std::vector<float> getCamPositionFromWorld(float x_global, float y_global, float z_global);

    std::vector<float> getWorldEulerAnglesFromCam(float yaw_cam, float pitch_cam, float roll_cam);
    std::vector<float> getCamEulerAnglesFromWorld(float yaw_world, float pitch_world, float roll_world);

    cv::Point3f pnpToWorldP3f(const cv::Point3f& pnp_pos);
    cv::Point3f worldToPnpP3f(const cv::Point3f& world_pos);

private:
    float camera_yaw;
    float camera_pitch;
    float camera_roll;
    float camera_x;
    float camera_y;
    float camera_z;
    
    // 添加辅助函数
    std::vector<std::vector<float>> eulerToRotationMatrix(float yaw, float pitch, float roll);
    std::vector<float> rotationMatrixToEuler(const std::vector<std::vector<float>>& R);
    std::vector<std::vector<float>> multiplyMatrixAndMatrix(
        const std::vector<std::vector<float>>& A, 
        const std::vector<std::vector<float>>& B);
};

#endif // REST_FRAME_H