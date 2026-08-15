

#ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
#define ARMOR_DETECTOR_BA_SOLVER_HPP_

// std
#include <array>
#include <cstddef>
#include <tuple>
#include <vector>
#include <cmath>
// 3rd party
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/so3.hpp>
// g2o
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_optimizer.h>
// project
#include "ba_solver/graph_optimizer.hpp" 
#include "2d_armor_detector/Armor.h"
#include <yaml-cpp/yaml.h>


namespace fyt::auto_aim {

// BA algorithm based Optimizer for the armor pose estimation (Particularly for
// the Yaw angle)
// 基于BA算法的对于装甲板位姿估计（尤其是对于yaw轴角度）的优化
class BaSolver {
public:
  BaSolver(const std::array<double, 9> &camera_matrix, //3x3的内参矩阵
           const std::vector<double> &dist_coeffs); // 畸变系数

  // Solve the armor pose using the BA algorithm, return the optimized rotation
  // 用BA算法解算装甲板位姿，并返回优化后的矩阵
  Eigen::Matrix3d solveBa(const ArmorResult &armor,       // 一个struct 在type里面
                          const Eigen::Vector3d &t_camera_armor, // 相机系下的平移
                          const Eigen::Matrix3d &R_camera_armor, // 相机系下的旋转
                          const Eigen::Matrix3d &R_imu_camera) noexcept; //外参矩阵
  
   

// 将旋转矩阵转化为Roll、Yaw、Pitch。
Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
  Eigen::Vector3d rpy;

  // // 数值稳定：对 asin 的输入做夹取
  // double sp = R(2, 1);
  // if (sp >  1.0) sp =  1.0;
  // if (sp < -1.0) sp = -1.0;

  // const double eps = 1e-6;

  // double pitch = std::asin(sp);
  // double cp = std::cos(pitch);

  // double yaw, roll;
  // if (std::abs(cp) > eps) {
  //   yaw  = std::atan2( R(0, 1), R(1, 1));
  //   roll = std::atan2( R(2, 0), R(2, 2));
  // } else {
  //   roll = 0.0;
  //   yaw  = std::atan2(-R(1, 0), R(0, 0));
  // }

  // 数值稳定：对 asin 的输入做夹取
  double sp = -R(1, 2);
  if (sp >  1.0) sp =  1.0;
  if (sp < -1.0) sp = -1.0;

  const double eps = 1e-6;

  double pitch = std::asin(sp);
  double cp = std::cos(pitch);

  double yaw, roll;
  if (std::abs(cp) > eps) {
    yaw  = std::atan2(-R(0, 2), R(2, 2));
    roll = std::atan2( R(1, 0), R(1, 1));
  } else {
    roll = 0.0;
    yaw  = std::atan2( R(2, 0), R(0, 0));
  }

  // 返回顺序保持不变：rpy[0]=pitch, rpy[1]=yaw, rpy[2]=roll
  rpy[0] = yaw;
  rpy[1] = pitch;
  rpy[2] = roll;
  return rpy;
}

  

Eigen::Matrix3d RPYTorotationMatrix(const Eigen::Vector3d& PYR) {
  const double pitch = PYR[0]; // 绕 x
  const double yaw   = PYR[1]; // 绕 z
  const double roll  = PYR[2]; // 绕 y

  const double sp = std::sin(pitch), cp = std::cos(pitch);
  const double sy = std::sin(yaw),   cy = std::cos(yaw);
  const double sr = std::sin(roll),  cr = std::cos(roll);

  Eigen::Matrix3d R;
  // 与提取公式：
  // pitch = asin(-R(1,2))
  // yaw   = atan2(-R(0,2), R(2,2))
  // roll  = atan2( R(1,0), R(1,1))
  // 互为逆（非奇异处）
  R(0,0) = -sp*sr*sy + cr*cy;
  R(0,1) = -sp*sy*cr - sr*cy;
  R(0,2) = -sy*cp;

  R(1,0) =  sr*cp;
  R(1,1) =  cp*cr;
  R(1,2) = -sp;

  R(2,0) =  sp*sr*cy + sy*cr;
  R(2,1) =  sp*cr*cy - sr*sy;
  R(2,2) =  cp*cy;

  // R(0,0) =  cy*cr -sy*sp*sr;
  // R(0,1) =  sy*cp;
  // R(0,2) = -cy*sr -sy*sp*cr;

  // R(1,0) = -sy*cr -cy*sp*sr;
  // R(1,1) =  cy*cp;
  // R(1,2) =  sy*sr -cy*sp*cr;

  // R(2,0) =  cp*sr;
  // R(2,1) =  sp;
  // R(2,2) =  cp*cr;

  return R;
}


  template<typename TPoint3>
  inline std::vector<TPoint3> buildObjectPoints(double w, double h) noexcept {
    auto make = [](double x, double y, double z) {
        // 适配 Eigen::Vector3d / cv::Point3f 的 (x,y,z) 构造
        return TPoint3(static_cast<typename std::remove_reference_t<TPoint3>::value_type>(x),
                       static_cast<typename std::remove_reference_t<TPoint3>::value_type>(y),
                       static_cast<typename std::remove_reference_t<TPoint3>::value_type>(z));
    };
    // 注意顺序：左上开始逆时针
    return {
        make(-w/2, -h/2, 0),
        make(-w/2, h/2, 0),
        make(w/2, h/2, 0),
        make(w/2, -h/2, 0),
    };
}

private:
  Eigen::Matrix3d K_; //内参矩阵
  g2o::SparseOptimizer optimizer_; // 稀疏优化器，往里面塞入顶点（具体是图论的内容）
  g2o::OptimizationAlgorithmProperty solver_property_; // 求解器
  g2o::OptimizationAlgorithmLevenberg *lm_algorithm_; // LM算法对象

};
}

// namespace fyt::auto_aim
#endif // ARMOR_DETECTOR_BAS_SOLVER_HPP_

