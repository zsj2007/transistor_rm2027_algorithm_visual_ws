// BallisticSolver.h
#ifndef BALLISTIC_SOLVER_H
#define BALLISTIC_SOLVER_H

#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <thread>

// 结构体声明
struct BallisticInfo {
    float delta_pitch_rad;  // pitch需要转动的角度
    float delta_yaw_rad;    // yaw最终的角度（逆时针为正）
    bool valid;
};

class BallisticSolver {
public:
    BallisticSolver(std::shared_ptr<YAML::Node> config_file_ptr)
    : config_file_ptr(config_file_ptr) {}
    
    // 函数声明
    BallisticInfo calcBallisticAngle(float x, float y, float z,
                                    float v_bullet, float cur_pitch, float cur_yaw);

    cv::Point3d calcNearestPointWithAirResistance(cv::Point3d target_pos, cv::Point3d self_pos, cv::Point2d aim_yaw_pitch, float v_bullet);
                                    
private:
    std::shared_ptr<YAML::Node> config_file_ptr;

    struct CalcPitchInfo {
        float target_pitch_result_smaller;
        float target_pitch_result_larger;
        bool valid;
    };
    
    float normalizeRad(float rad);
    float shortestRadDiff(float target, float current);
    CalcPitchInfo calcTargetPitch(float horizontal_distance, float vertical_distance, float v_bullet);

    struct BallisticParams {
        float drag_coeff;      // 阻力系数
        float air_density;     // 空气密度
        float bullet_diameter; // 弹丸直径
        float bullet_mass;     // 弹丸质量
        BallisticParams() : drag_coeff(0.47f), air_density(1.225f), bullet_diameter(17.0*1e-3), bullet_mass(3.2*1e-3) {}
    };
    
    // 统一的弹道模拟结果结构
    struct TrajectorySimulationResult {
        bool valid;
        double hit_height;                    // 用于高度模拟
        cv::Point2d nearest_point_2d;         // 用于最近点计算（二维平面内）
        double nearest_distance;              // 最近距离
        cv::Point2d prev_point_2d;            // 前一个点（用于最近点插值）
    };
    
    struct TrajectoryInfo {
        float pitch;
        float hit_height;
    };
    
    struct RefineInfo {
        TrajectoryInfo lower_pitch_trajectory;
        TrajectoryInfo upper_pitch_trajectory;
        bool rising;
        bool valid = false;
    };
    
    BallisticParams ballisticParams;
    CalcPitchInfo calcTargetPitchWithAirResistance(float horizontal_distance, float vertical_height, float v_bullet);
    
    // 统一的弹道模拟函数
    TrajectorySimulationResult simulateTrajectory2D(double v_bullet, double pitch_rad, double target_distance,
                                                   double max_flight_time, double dt, double min_height,
                                                   bool calculate_height = true, cv::Point2d target_point_2d = cv::Point2d(0,0));
    
    // 辅助函数：计算加速度
    struct State {
        double x, z;      // 位置 (x: 水平距离, z: 高度)
        double vx, vz;    // 速度
    };
    
    State computeAcceleration(const State& state, double drag_coeff, double air_density, 
                             double cross_section_area, double bullet_mass, double g);

    struct alignas(64) StartCheckThreadInfo {
        int thread_index;
        float pitch;
    };
    
    struct alignas(64) RefineThreadInfo {
        int thread_index;
        RefineInfo refine_info;
    };

    float extra_delta_pitch = 0.0 * M_PI / 180.0f;
};

#endif // BALLISTIC_SOLVER_H
