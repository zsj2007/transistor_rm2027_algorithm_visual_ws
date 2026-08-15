// RestFrame.cpp
#include "3d_processing/RestFrame.h"
#include <stdexcept>

// 坐标系定义(NormalFrame)：
// x：向右，y：向前，z：向上
// yaw：绕z轴 从上方看逆时针 x轴转向y轴，pitch：绕x轴 抬头 y轴转向z轴，roll：绕y轴 从画面看顺时针 z轴转向x轴

void RestFrame::updateCamOrientation(float yaw, float pitch, float roll) {
    camera_yaw = yaw;
    camera_pitch = pitch;
    camera_roll = roll;
}

void RestFrame::updateCamPosition(float x, float y, float z) {
    camera_x = x;
    camera_y = y;
    camera_z = z;
}

std::vector<float> RestFrame::getCamOrientation() {
    return {camera_yaw, camera_pitch, camera_roll};
}

std::vector<float> RestFrame::getCamPosition() {
    return {camera_x, camera_y, camera_z};
}

std::vector<float> RestFrame::pnpResultToNormalFrame(float x_pnp, float y_pnp, float z_pnp) {
    return {x_pnp, z_pnp, -y_pnp};
}

std::vector<float> RestFrame::normalToPnpResultFrame(float x_cam_normal, float y_cam_normal, float z_cam_normal) {
    return {x_cam_normal, -z_cam_normal, y_cam_normal};
}

std::vector<float> RestFrame::getWorldPositionFromCam(float x_cam_normal, float y_cam_normal, float z_cam_normal) {
    // 使用旋转矩阵方法
    auto R = eulerToRotationMatrix(camera_yaw, camera_pitch, camera_roll);
    std::vector<float> v = {x_cam_normal, y_cam_normal, z_cam_normal};
    std::vector<float> v_rotated(3, 0.0f);
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            v_rotated[i] += R[i][j] * v[j];
        }
    }
    
    return {v_rotated[0] + camera_x, v_rotated[1] + camera_y, v_rotated[2] + camera_z};
    
    /*
    // 原始代码保留
    float cosY = std::cos(camera_yaw);
    float sinY = std::sin(camera_yaw);
    float cosP = std::cos(camera_pitch);
    float sinP = std::sin(camera_pitch);
    float cosR = std::cos(camera_roll);
    float sinR = std::sin(camera_roll);

    // Apply roll, pitch, yaw rotations
    float x_temp = x_cam_normal;
    float y_temp = y_cam_normal;
    float z_temp = z_cam_normal;

    // Roll rotation
    float x_roll = x_temp * cosR + z_temp * sinR;
    float y_roll = y_temp;
    float z_roll = z_temp * cosR - x_temp * sinR;

    // Pitch rotation
    float x_pitch = x_roll;
    float y_pitch = y_roll * cosP - z_roll * sinP;
    float z_pitch = z_roll * cosP + y_roll * sinP;

    // Yaw rotation
    float x_yaw = x_pitch * cosY - y_pitch * sinY;
    float y_yaw = y_pitch * cosY + x_pitch * sinY;
    float z_yaw = z_pitch;

    // Translation
    return {x_yaw + camera_x, y_yaw + camera_y, z_yaw + camera_z};
    */
}

std::vector<float> RestFrame::getCamPositionFromWorld(float x_global, float y_global, float z_global) {
    // 使用旋转矩阵方法
    auto R = eulerToRotationMatrix(camera_yaw, camera_pitch, camera_roll);
    std::vector<float> v = {x_global - camera_x, y_global - camera_y, z_global - camera_z};
    std::vector<float> v_rotated(3, 0.0f);
    
    // 使用旋转矩阵的转置（逆）
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            v_rotated[i] += R[j][i] * v[j]; // 转置矩阵
        }
    }
    
    return {v_rotated[0], v_rotated[1], v_rotated[2]};
    
    /*
    // 原始代码保留
    // Translation
    float x_temp = x_global - camera_x;
    float y_temp = y_global - camera_y;
    float z_temp = z_global - camera_z;

    float cosY = std::cos(-camera_yaw);
    float sinY = std::sin(-camera_yaw);
    float cosP = std::cos(-camera_pitch);
    float sinP = std::sin(-camera_pitch);
    float cosR = std::cos(-camera_roll);
    float sinR = std::sin(-camera_roll);

    // Inverse yaw rotation
    float x_yaw = x_temp * cosY - y_temp * sinY;
    float y_yaw = y_temp * cosY + x_temp * sinY;
    float z_yaw = z_temp;

    // Inverse pitch rotation
    float x_pitch = x_yaw;
    float y_pitch = y_yaw * cosP - z_yaw * sinP;
    float z_pitch = z_yaw * cosP + y_yaw * sinP;

    // Inverse roll rotation
    float x_roll = x_pitch * cosR + z_pitch * sinR;
    float y_roll = y_pitch;
    float z_roll = z_pitch * cosR - x_pitch * sinR;

    return {x_roll, y_roll, z_roll};
    */
}

std::vector<std::vector<float>> RestFrame::eulerToRotationMatrix(float yaw, float pitch, float roll) {
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);
    float cr = std::cos(roll);
    float sr = std::sin(roll);

    return {
        {cy * cr - sy * sp * sr, - sy * cp, cy * sr + sy * sp * cr},
        {sy * cr + cy * sp * sr, cy * cp, sy * sr - cy * sp * cr},
        {- cp * sr, sp, cp * cr}
        /* {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr} */
    };
}

std::vector<float> RestFrame::rotationMatrixToEuler(const std::vector<std::vector<float>>& R) {
    // 检查矩阵尺寸
    if (R.size() != 3 || R[0].size() != 3 || R[1].size() != 3 || R[2].size() != 3) {
        throw std::invalid_argument("Input matrix must be 3x3");
    }
    
    // 提取矩阵元素
    float M00 = R[0][0];
    float M01 = R[0][1];
    float M02 = R[0][2];
    float M10 = R[1][0];
    float M11 = R[1][1];
    float M12 = R[1][2];
    float M20 = R[2][0];
    float M21 = R[2][1];
    float M22 = R[2][2];
    
    float yaw, pitch, roll;
    
    // 计算pitch from M21
    pitch = std::asin(std::clamp(M21, -1.0f, 1.0f));
    
    // 检查万向节死锁（cos(pitch)是否接近零）
    const float epsilon = 1e-6;
    if (std::fabs(std::cos(pitch)) > epsilon) {
        // 正常情况
        yaw = std::atan2(-M01, M11);
        roll = std::atan2(-M20, M22);
    } else {
        // 万向节死锁处理
        roll = 0.0f;
        yaw = std::atan2(M10, M00);
    }
    
    // 返回欧拉角向量 [yaw, pitch, roll]
    return {yaw, pitch, roll};
}

std::vector<std::vector<float>> RestFrame::multiplyMatrixAndMatrix(
    const std::vector<std::vector<float>>& A, 
    const std::vector<std::vector<float>>& B) 
{
    if (A.size() != 3 || B.size() != 3 || A[0].size() != 3 || B[0].size() != 3) {
        throw std::invalid_argument("Matrices must be 3x3");
    }

    std::vector<std::vector<float>> result(3, std::vector<float>(3, 0.0f));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

// 修正后的函数
std::vector<float> RestFrame::getWorldEulerAnglesFromCam(float yaw_cam, float pitch_cam, float roll_cam) {
    auto camRotationMatrix = eulerToRotationMatrix(camera_yaw, camera_pitch, camera_roll);
    auto objectRotationMatrix = eulerToRotationMatrix(yaw_cam, pitch_cam, roll_cam);
    auto worldRotationMatrix = multiplyMatrixAndMatrix(camRotationMatrix, objectRotationMatrix);
    
    // 使用正确的欧拉角提取方法
    return rotationMatrixToEuler(worldRotationMatrix);
}

// 修正后的函数
std::vector<float> RestFrame::getCamEulerAnglesFromWorld(float yaw_world, float pitch_world, float roll_world) {
    auto camRotationMatrix = eulerToRotationMatrix(camera_yaw, camera_pitch, camera_roll);
    // 计算相机旋转矩阵的逆（转置）
    std::vector<std::vector<float>> camRotationMatrixInv(3, std::vector<float>(3));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            camRotationMatrixInv[i][j] = camRotationMatrix[j][i];
        }
    }
    
    auto objectRotationMatrix = eulerToRotationMatrix(yaw_world, pitch_world, roll_world);
    auto camRotationResult = multiplyMatrixAndMatrix(camRotationMatrixInv, objectRotationMatrix);
    
    // 使用正确的欧拉角提取方法
    return rotationMatrixToEuler(camRotationResult);
}

cv::Point3f RestFrame::pnpToWorldP3f(const cv::Point3f& pnp_pos) {    
    std::vector<float> cam_normal_pos = pnpResultToNormalFrame(pnp_pos.x, pnp_pos.y, pnp_pos.z);
    std::vector<float> rest_frame_pos = getWorldPositionFromCam(cam_normal_pos[0], cam_normal_pos[1], cam_normal_pos[2]);
    return cv::Point3f(rest_frame_pos[0], rest_frame_pos[1], rest_frame_pos[2]);
}

cv::Point3f RestFrame::worldToPnpP3f(const cv::Point3f& world_pos) {
    std::vector<float> cam_normal_pos = getCamPositionFromWorld(world_pos.x, world_pos.y, world_pos.z);
    std::vector<float> pnp_pos = normalToPnpResultFrame(cam_normal_pos[0], cam_normal_pos[1], cam_normal_pos[2]);
    return cv::Point3f(pnp_pos[0], pnp_pos[1], pnp_pos[2]);
}
