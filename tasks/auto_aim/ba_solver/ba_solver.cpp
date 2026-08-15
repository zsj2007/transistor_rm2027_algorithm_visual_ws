
#include "ba_solver/ba_solver.hpp"

#include "tools/logger.hpp"
// std
#include <memory>
// g2o
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/types_slam3d.h>
// 3rd party
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// project
#include "ba_solver/graph_optimizer.hpp" 
#include "ba_solver/utils.hpp" 

#include "2d_armor_detector/Armor.h"

namespace fyt::auto_aim {
G2O_USE_OPTIMIZATION_LIBRARY(dense)

BaSolver::BaSolver(const std::array<double, 9> &camera_matrix,
                   const std::vector<double> &dist_coeffs) {
  K_ = Eigen::Matrix3d::Identity();
  K_(0, 0) = camera_matrix[0];
  K_(1, 1) = camera_matrix[4];
  K_(0, 2) = camera_matrix[2];
  K_(1, 2) = camera_matrix[5];

  // Optimization information
  optimizer_.setVerbose(false);
  // Optimization method
  optimizer_.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property_));
  // Initial step size
  lm_algorithm_ = dynamic_cast<g2o::OptimizationAlgorithmLevenberg *>(
      const_cast<g2o::OptimizationAlgorithm *>(optimizer_.algorithm()));
  lm_algorithm_->setUserLambdaInit(0.1);
}

Eigen::Matrix3d
BaSolver::solveBa(const ArmorResult &armor, const Eigen::Vector3d &t_camera_armor,
                  const Eigen::Matrix3d &R_camera_armor,
                  const Eigen::Matrix3d &R_imu_camera) noexcept {
  // Reset optimizer
  optimizer_.clear();

  // Essential coordinate system transformation
  Eigen::Matrix3d R_imu_armor = R_imu_camera * R_camera_armor;
  Sophus::SO3d R_camera_imu = Sophus::SO3d(R_imu_camera.transpose()); 

  // 在世界坐标系下的 假设没有roll和pitch时候的装甲板的yaw
  
  double initial_armor_yaw;
  initial_armor_yaw = std::atan2(R_imu_armor(0,2), R_imu_armor(2,2));  //这里的负号不要了

  tools::logger()->debug("Yaw beforeOptimize: {:.2f}", initial_armor_yaw);  // 调试行：世界系下的yaw对不对

  // Get the pitch angle of the armor
  double armor_pitch =
      armor.number == 6 ? 0.2617994 : -0.2617994; //
  // Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(0, armor_pitch, 0));
  Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(armor_pitch, 0, 0));            // 更改尝试pitch位置


  // Get the 3D points of the armor
  const auto armor_size =
      // armor.is_large == 0
      //     ? Eigen::Vector2d( 135.0 , 125.0 )
      //     : Eigen::Vector2d( 230.0 , 127.0 );
      armor.is_large == 0
          ? Eigen::Vector2d( ArmorConstants::SMALL_ARMOR_LIGHT_DISTANCE , ArmorConstants::PNP_LIGHT_HEIGHT )
          : Eigen::Vector2d( ArmorConstants::LARGE_ARMOR_LIGHT_DISTANCE , ArmorConstants::PNP_LIGHT_HEIGHT );
  const auto object_points =
    buildObjectPoints<Eigen::Vector3d>(armor_size(0), armor_size(1));
    tools::logger()->debug("pitch: {:.2f}, is_large: {}", armor_pitch, armor.is_large);  // 调试行
    
  // Fill the optimizer
  size_t id_counter = 0;

  VertexYaw *v_yaw = new VertexYaw();
  v_yaw->setId(id_counter++);
  v_yaw->setEstimate(initial_armor_yaw);
  optimizer_.addVertex(v_yaw);

  // const auto &landmarks = armor.corners;
  const auto &landmarks = armor.armor.light_bar_corners;

  std::array<EdgeProjection*, 4> edges{};  // ===== 保存 4 条边，便于读取角点在相机系的 3D 坐标 （角点可视化）

  for (size_t i = 0; i < 4 ; i++) {
    g2o::VertexPointXYZ *v_point = new g2o::VertexPointXYZ();
    v_point->setId(id_counter++);
    v_point->setEstimate(Eigen::Vector3d(
        object_points[i].x(), object_points[i].y(), object_points[i].z()));
    v_point->setFixed(true);
    optimizer_.addVertex(v_point);

    EdgeProjection *edge =
        new EdgeProjection(R_camera_imu, R_pitch, t_camera_armor, K_);
    edge->setId(id_counter++);
    edge->setVertex(0, v_yaw);
    edge->setVertex(1, v_point);
    edge->setMeasurement(Eigen::Vector2d(landmarks[i].x, landmarks[i].y));
    edge->setInformation(EdgeProjection::InfoMatrixType::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    optimizer_.addEdge(edge);
    edges[i] = edge;  // ===== 记下来（角点可视化）
  }

  // Start optimizing
  optimizer_.initializeOptimization();


  // —— 优化前：触发一次 computeError()，打印四个角点的预测像素 vs 观测像素
  optimizer_.computeActiveErrors();
  for (int i = 0; i < 4; ++i) {
    const auto& uv_pred = edges[i]->getLastUV();
    // const auto& uv_obs  = armor.corners[i];
    const auto& uv_obs  = armor.armor.light_bar_corners[i];
    tools::logger()->debug(
      "[BEFORE] C{}_pred=[{:.2f} {:.2f}]  C{}_obs=[{:.2f} {:.2f}]",
      i, uv_pred.x(), uv_pred.y(), i, uv_obs.x, uv_obs.y);
  }

  optimizer_.optimize(20);

  // Get yaw angle after optimization
  double yaw_optimized = v_yaw->estimate();

  if (std::isnan(yaw_optimized)) {
    tools::logger()->debug("Yaw angle is nan after optimization");
    return R_camera_armor;
  }

  // —— 优化后：再触发一次，打印最终的预测像素 vs 观测像素
  optimizer_.computeActiveErrors();
  for (int i = 0; i < 4; ++i) {
    const auto& uv_pred = edges[i]->getLastUV();
    // const auto& uv_obs  = armor.corners[i];
    const auto& uv_obs  = armor.armor.light_bar_corners[i];
    tools::logger()->debug(
      "[AFTER ] C{}_pred=[{:.2f} {:.2f}]  C{}_obs=[{:.2f} {:.2f}]",
      i, uv_pred.x(), uv_pred.y(), i, uv_obs.x, uv_obs.y);
  }

  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, yaw_optimized, 0)); // 反吗
  tools::logger()->debug("Yaw before trans: {:.2f}", yaw_optimized);

  tools::logger()->debug("Yaw angle is valid after optimization");

  // Eigen::Matrix3d R_testM= RPYTorotationMatrix(Eigen::Vector3d(1 ,1 ,0)); // 测试矩阵
  // Sophus::SO3d R_testS = Sophus::SO3d(R_testM);
  // return (R_testS * R_yaw * R_pitch).matrix();                                        // 测试返回

  // return (R_camera_imu * R_yaw * R_pitch).matrix();                                  // 相机系返回

   return ( R_yaw * R_pitch).matrix();                                                // 世界系返回
}

} // namespace fyt::auto_aim
