// ArmorSolver.h
#ifndef ARMOR_SOLVER_H
#define ARMOR_SOLVER_H
#include <Eigen/Dense>


#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#define _USE_MATH_DEFINES // 启用数学常量
#include <cmath>
#include <opencv2/core/eigen.hpp> // 用于Eigen转换
#include <fstream> // <-- 添加文件流头文件
#include <memory>


#include "2d_armor_detector/LightBar.h"
#include "2d_armor_detector/Armor.h"
#include "ba_solver/ba_solver.hpp"
#include "ba_solver/utils.hpp"

#include <Eigen/Geometry> // For Quaternion and rotation matrix math


double getYawFromRvec(const cv::Mat& rvec);

std::vector<double> getNormalYawPitchRollFromRvec(const cv::Mat& rvec);


class ArmorSolver {
    
public:
    ArmorSolver(std::shared_ptr<YAML::Node> config_file_ptr) {
        // 初始化相机参数
        initCameraMatrix(config_file_ptr);
        initArmorPoints();

        delta_x_ = (*config_file_ptr)["delta_x_"].as<float>();
        delta_y_ = (*config_file_ptr)["delta_y_"].as<float>();
        delta_z_ = (*config_file_ptr)["delta_z_"].as<float>();

    }
    // 新增3D到像素坐标投影函数
    cv::Point2f project3DToPixel(const cv::Point3f& world_point) const;

    AimResult solveArmor(const ArmorResult& armor_result, const double last_pitch_rad_, const double last_yaw_rad_) const; // 增加number参数
    
    
     /**
     * @brief 根据图像分辨率计算最大视场角（弧度）
     * @param width  图像宽度（像素）
     * @param height 图像高度（像素）
     * @return 最大夹角（弧度），若计算失败返回 -1.0
     */
    double getMaxFOVAngle(int width, int height) const;

private:
    // 相机参数
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    // 装甲板3D点(单位：mm)
    std::vector<cv::Point3f> armor_points_3d;
    
    void initCameraMatrix(std::shared_ptr<YAML::Node> config_file_ptr);
    void initArmorPoints();

    std::unique_ptr<fyt::auto_aim::BaSolver> ba_;
    
    float delta_x_;
    float delta_y_;
    float delta_z_;

    // 用于缓存分辨率与对应最大夹角的映射（mutable 以便在 const 成员函数中修改）
    mutable std::unordered_map<std::string, double> fov_cache_;

    // 辅助函数：生成分辨率字符串键
    static std::string makeCacheKey(int width, int height) {
        return std::to_string(width) + "x" + std::to_string(height);
    }
};

#endif // ARMOR_SOLVER_H
