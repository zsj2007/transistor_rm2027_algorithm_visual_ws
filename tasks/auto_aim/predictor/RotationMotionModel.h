// RotationMotionModel.h
#pragma once
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include "utils/DataProcessFuncs.h"
#include "3d_processing/RestFrame.h"
#include "2d_armor_detector/Armor.h"


struct ObservedData {
    double x;
    double y;
    double z;
    double yaw;
    double t;

    bool yaw_jump = false;
    
    ObservedData(double x_val, double y_val, double z_val, double yaw_val, double t_val)
        : x(x_val), y(y_val), z(z_val), yaw(yaw_val), t(t_val) {}

    double dt;
};

struct SimpleArmor {
    double x;
    double y;
    double z;
    double r;
    double yaw;
};

struct PredictResult {
    double center_x;
    double center_y;
    double center_z;
    double z_another;
    double r_now;
    double r_another;
    double yaw;
    int rotation_direction;
    std::vector<SimpleArmor> armors;
};

// 用于角度和角速度跟踪的EKF
class AngleEKF {
private:
    static constexpr int STATE_DIM = 2;  // [yaw, vyaw]
    static constexpr int OBS_DIM = 3;    // [yaw, xa, ya]
    
    Eigen::Vector2d state_;              // [yaw, vyaw]
    Eigen::Matrix2d P_;                  // 协方差矩阵
    Eigen::Matrix2d Q_;                  // 过程噪声
    Eigen::Matrix3d R_;                  // 观测噪声
    
    double last_yaw_ = 0.0;
    bool initialized_ = false;

    bool is_outpost;

    int64_t total_jump_time_with_direction = 0;
    
public:
    AngleEKF(bool is_outpost) : state_(2), P_(2, 2), Q_(2, 2), R_(3, 3), is_outpost(is_outpost) {
        // 初始化状态
        state_ << 0.0, 0.0;
        
        // 初始化协方差
        P_ = Eigen::Matrix2d::Identity() * 10.0;
        
        // 初始化过程噪声
        Q_ = Eigen::Matrix2d::Identity() * 0.1;
        
        // 初始化观测噪声
        R_ = Eigen::Matrix3d::Identity() * 1.0;
    }
    
    void initialize(double init_yaw) {
        state_(0) = init_yaw;
        state_(1) = 0.0;
        last_yaw_ = init_yaw;
        initialized_ = true;
    }
    
    bool isInitialized() const { return initialized_; }
    
    /**
     * 角度跟踪的状态转移函数
     */
    Eigen::Vector2d processModel(const Eigen::Vector2d& state, double dt) {
        Eigen::Vector2d new_state;
        new_state(0) = state(0) + state(1) * dt;  // yaw += vyaw * dt
        new_state(1) = state(1);                  // vyaw 保持不变
        return new_state;
    }
    
    /**
     * 观测模型 - 使用xc, yc, r计算装甲板位置
     */
    Eigen::Vector3d observationModel(const Eigen::Vector2d& state, double xc, double yc, double r) {
        Eigen::Vector3d z_obs;
        z_obs(0) = state(0);                                  // 观测yaw
        z_obs(1) = xc + r * std::sin(state(0));               // 观测xa
        z_obs(2) = yc - r * std::cos(state(0));               // 观测ya
        return z_obs;
    }
    
    /**
     * 状态转移雅可比矩阵
     */
    Eigen::Matrix2d jacobianF(double dt) {
        Eigen::Matrix2d F;
        F << 1.0, dt,
             0.0, 1.0;
        return F;
    }
    
    /**
     * 观测雅可比矩阵
     */
    Eigen::Matrix<double, 3, 2> jacobianH(const Eigen::Vector2d& state, double xc, double yc, double r) {
        Eigen::Matrix<double, 3, 2> H;
        double yaw = state(0);
        
        // ∂z0/∂yaw = 1, ∂z0/∂vyaw = 0
        H(0, 0) = 1.0;
        H(0, 1) = 0.0;
        
        // ∂z1/∂yaw = r * cos(yaw), ∂z1/∂vyaw = 0
        H(1, 0) = r * std::cos(yaw);
        H(1, 1) = 0.0;
        
        // ∂z2/∂yaw = r * sin(yaw), ∂z2/∂vyaw = 0
        H(2, 0) = r * std::sin(yaw);
        H(2, 1) = 0.0;
        
        return H;
    }
    
    /**
     * 更新过程噪声
     */
    void updateQ(double dt) {
        double s2q_yaw = is_outpost ? 0.01 : 0.1;    // yaw过程噪声
        double s2q_vyaw = is_outpost ? 0.001 : 0.01;   // 角速度过程噪声
        
        Q_(0, 0) = std::pow(dt, 4) / 4.0 * s2q_yaw;
        Q_(0, 1) = std::pow(dt, 3) / 2.0 * s2q_yaw;
        Q_(1, 0) = std::pow(dt, 3) / 2.0 * s2q_yaw;
        Q_(1, 1) = std::pow(dt, 2) * s2q_vyaw;
    }
    
    /**
     * 处理角度跳变
     */
    void handleYawJump(double& measured_yaw, double dt) {
        if (!initialized_) return;
        
        double yaw_diff = measured_yaw - last_yaw_;
        double yaw_change = 0.0;

        // 处理角度环绕
        while (yaw_diff > M_PI) {yaw_diff -= 2.0 * M_PI; yaw_change -= 2.0 * M_PI;}
        while (yaw_diff < -M_PI) {yaw_diff += 2.0 * M_PI; yaw_change += 2.0 * M_PI;}
        
        // 如果角度差异超过阈值，可能是装甲板跳变
        double jump_threshold = M_PI / 4.0; // 45度阈值
        
        if (std::abs(yaw_diff) > jump_threshold) {
            double target_new_yaw = measured_yaw - state_(1) * dt;
            target_new_yaw = atan2(sin(target_new_yaw), cos(target_new_yaw));
            double state_yaw = state_(0);
            state_yaw = atan2(sin(state_yaw), cos(state_yaw));
            double delta_yaw = target_new_yaw - state_yaw;
            delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
            double new_state_yaw = state_yaw;
            if (is_outpost) {
                if (delta_yaw > M_PI / 3.0) {
                    total_jump_time_with_direction += 1;
                    new_state_yaw += M_PI * 2.0 / 3.0;
                } else if(delta_yaw < -M_PI / 3.0) {
                    total_jump_time_with_direction -= 1;
                    new_state_yaw -= M_PI * 2.0 / 3.0;
                }
            } else {
                if (abs(delta_yaw) < M_PI / 4.0) {
                    // nothing
                } else if (delta_yaw > M_PI / 4.0 && delta_yaw < M_PI * 3.0 / 4.0) {
                    total_jump_time_with_direction += 1;
                    new_state_yaw += M_PI / 2.0;
                } else if (delta_yaw < -M_PI / 4.0 && delta_yaw > -M_PI * 3.0 / 4.0) {
                    total_jump_time_with_direction -= 1;
                    new_state_yaw -= M_PI / 2.0;
                } else {
                    if (state_(1) > 0.0) {
                        total_jump_time_with_direction -= 2;
                        new_state_yaw -= M_PI;
                    } else {
                        total_jump_time_with_direction += 2;
                        new_state_yaw += M_PI;
                    }
                }
            }
            new_state_yaw = atan2(sin(new_state_yaw), cos(new_state_yaw));
            // 直接更新偏航角状态，保持角速度不变
            state_(0) = new_state_yaw;
            std::cout << "Yaw jump detected! Updating yaw from " 
                      << last_yaw_ << " to " << measured_yaw << std::endl;

            measured_yaw = atan2(sin(measured_yaw), cos(measured_yaw));
            return;
        } else {
            measured_yaw += yaw_change;
        }
        
        return;
    }
    
    bool isYawJumpForRMM(double measured_yaw, double dt) {
        if (!initialized_) return false;
        
        double target_new_yaw = measured_yaw - state_(1) * dt;
        target_new_yaw = atan2(sin(target_new_yaw), cos(target_new_yaw));
        double state_yaw = state_(0);
        state_yaw = atan2(sin(state_yaw), cos(state_yaw));
        double delta_yaw = target_new_yaw - state_yaw;
        delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
        int change_jump_time_with_direction = 0;
        if (is_outpost) {
            if (delta_yaw > M_PI / 3.0) {
                return true;
            } else if(delta_yaw < -M_PI / 3.0) {
                return true;
            }
        } else {
            if (abs(delta_yaw) < M_PI / 4.0) {
                return false;
            } else if (delta_yaw > M_PI / 4.0 && delta_yaw < M_PI * 3.0 / 4.0) {
                return true;
            } else if (delta_yaw < -M_PI / 4.0 && delta_yaw > -M_PI * 3.0 / 4.0) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
    
    /**
     * 预测步骤
     */
    void predict(double dt) {
        if (!initialized_) return;
        
        // 更新过程噪声
        updateQ(dt);
        
        // 计算雅可比矩阵
        Eigen::Matrix2d F = jacobianF(dt);
        
        // 状态预测
        state_ = processModel(state_, dt);
        
        // 协方差预测
        P_ = F * P_ * F.transpose() + Q_;
        
        // 处理角度环绕
        if (state_(0) > M_PI) state_(0) -= 2.0 * M_PI;
        if (state_(0) < -M_PI) state_(0) += 2.0 * M_PI;
    }
    
    /**
     * 更新步骤
     */
    void update(double measured_yaw, double measured_xa, double measured_ya, 
                double xc, double yc, double r, double dt) {
        if (!initialized_) {
            initialize(measured_yaw);
            return;
        }
        
        // 处理角度跳变
        handleYawJump(measured_yaw, dt);
        
        // 预测步骤
        predict(dt);
        
        // 观测值
        Eigen::Vector3d z;
        z << measured_yaw, measured_xa, measured_ya;
        
        // 计算雅可比矩阵
        Eigen::Matrix<double, 3, 2> H = jacobianH(state_, xc, yc, r);
        
        // 计算卡尔曼增益
        Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R_;
        Eigen::Matrix<double, 2, 3> K = P_ * H.transpose() * S.inverse();
        
        // 观测预测
        Eigen::Vector3d z_pred = observationModel(state_, xc, yc, r);
        
        // 状态更新
        Eigen::Vector3d innovation = z - z_pred;
        state_ = state_ + K * innovation;
        
        // 协方差更新
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        P_ = (I - K * H) * P_;
        
        // 处理角度环绕
        if (state_(0) > M_PI) state_(0) -= 2.0 * M_PI;
        if (state_(0) < -M_PI) state_(0) += 2.0 * M_PI;
        
        // 限制角速度
        if (std::abs(state_(1)) > 15.0) {
            state_(1) = 0.0;
        }
        
        last_yaw_ = measured_yaw;
    }
    
    // Getter方法
    double getYaw() const { return state_(0); }
    double getVyaw() const { return state_(1); }
    const Eigen::Vector2d& getState() const { return state_; }

    double getTotalYaw() const { return state_(0) - total_jump_time_with_direction * (is_outpost ? M_PI * 2.0 / 3.0 : M_PI / 2.0); }

    int getTotalJumpTimeWithDirection () { return total_jump_time_with_direction; }
};

struct RotationMotionState {
    double center_x;
    double center_y;
    double center_z;
    double z_another;
    double center_vx;
    double center_vy;
    double center_vz;
    double r_now;
    double r_another;
    double yaw;
    double total_yaw;
    double vyaw;
    unsigned long long update_frames;
};

// 修改状态向量维度，从5维扩展到7维
class RotationMotionModel {
private:
    // 扩展的状态向量： [center_x, center_y, center_z, center_vx, center_vy, center_vz, r]
    static constexpr int STATE_DIM = 7;  // 从5增加到7
    Eigen::MatrixXd P_center_;      // 7x7 协方差矩阵
    Eigen::VectorXd x_center_;      // 7维状态向量
    double lambda_;                 // 遗忘因子
    
    double r_now_prev_;             // 上一步的半径值，用于正则化
    double r_another_prev_;
    double regularization_weight_;  // 正则化权重
    
    // 修改指数衰减最小二乘方法
    void resetExponentialLS();
    void updateExponentialLS(double armor_x, double armor_y, double armor_z, double armor_yaw, double t, double dt, double weight=1.0, double delta_r=0.0, double delta_z=0.0);
    void updateCenterResult(double current_time);

    std::vector<ObservedData> observedDataHistory;
    ObservedData last_observed_data;
    double center_vx;
    double center_vy;
    double center_vz;
    double r_now;
    double r_another;
    double center_x;
    double center_y;
    double center_z;
    double z_another;
    int max_history;
    int n_armors;
    int rotation_direction;
    double jump_rad;

    // 用于角度和角速度跟踪的EKF
    std::unique_ptr<AngleEKF> angle_ekf_;
    double last_update_time_;

    std::shared_ptr<RestFrame> rest_frame_;
    bool is_outpost;

    unsigned long long update_frames_count = 0;

public:
    RotationMotionModel(ObservedData& initObservedData, std::shared_ptr<RestFrame> rest_frame_, bool is_outpost, double init_r);
    void update(ObservedData& observedData);
    PredictResult predict(double predictTime);
    void emptyUpdate(double update_time);
    RotationMotionState getState();
    double getTheoreticYaw(double armor_x, double armor_y);
    double getTheoreticYawFacingArmor(double armor_x, double armor_y);
    double getCamToCenterYaw();

    int debug_flip_flag = 1;
};
