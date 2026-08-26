cmake_minimum_required(VERSION 3.16)

# tasks：功能层（算法核心，全部来自 transistor_rm2026_algorithm_visual_ws，已去 ROS2）
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenVINO REQUIRED openvino)
# OpenVINO 2024.6 apt pkg-config lists JAX frontend, but the Ubuntu package does not ship it.
# This project uses OpenVINO IR (.xml/.bin), so JAX frontend is not required.
list(REMOVE_ITEM OpenVINO_LIBRARIES openvino_jax_frontend)
# MkvWriter（TwoVideoLogger）依赖 ffmpeg
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)

add_library(tasks STATIC
    auto_aim/2d_armor_detector/Armor.cpp
    auto_aim/2d_armor_detector/LightBarDetector.cpp
    auto_aim/2d_armor_detector/ArmorDetector.cpp
    auto_aim/2d_armor_detector/ArmorClassifier.cpp
    auto_aim/2d_armor_detector/ArmorTracker.cpp
    auto_aim/2d_armor_detector/UnwarpUtils.cpp
    auto_aim/3d_processing/ArmorSolver.cpp
    auto_aim/3d_processing/BallisticSolver.cpp
    auto_aim/3d_processing/RestFrame.cpp
    auto_aim/predictor/PositionPredictor2D.cpp
    auto_aim/predictor/PeriodicDataPredictor.cpp
    # PredictorSwitcher.cpp remains only for PredictorType ABI/UI labels.
    auto_aim/predictor/PredictorSwitcher.cpp
    auto_aim/EKF/SuperPowerEKF.cpp
    auto_aim/EKF/SuperPowerTarget.cpp
    auto_aim/EKF/SuperPowerTracker.cpp
    auto_aim/EKF/SuperPowerPredictor.cpp
    auto_aim/predictor/AllPredictor.cpp
    auto_aim/predictor/TargetManager.cpp
    auto_aim/predictor/PredictorMain.cpp
    auto_aim/visualizer/DataVisualizer.cpp
    auto_aim/utils/FrameRateCounter.cpp
    auto_aim/utils/PerformanceMonitor.cpp
    auto_aim/utils/SimpleDataFilter.cpp
    auto_aim/utils/DataProcessFuncs.cpp
    auto_aim/utils/PeriodFunctions.cpp
    auto_aim/ba_solver/ba_solver.cpp
    auto_aim/ba_solver/graph_optimizer.cpp
    auto_aim/ba_solver/utils.cpp
    auto_aim/logger/TwoVideoLogger.cpp
    auto_aim/logger/MkvWriter.cpp
    auto_aim/RP24_YOLO/OpenvinoInfer.cpp
    auto_aim/RP24_YOLO/RP24_YOLO_Wrapper.cpp
    auto_aim/pipeline/AutoAimPipeline.cpp
)

target_include_directories(tasks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/auto_aim
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenVINO_INCLUDE_DIRS}
)

target_link_libraries(tasks
    io
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    fmt::fmt
    TBB::tbb
    Threads::Threads
    g2o::core
    g2o::stuff
    g2o::solver_csparse
    g2o::csparse_extension
    g2o::types_slam3d
    g2o::types_sba
    g2o::solver_dense
    cxsparse
    Sophus::Sophus
    ${OpenVINO_LIBRARIES}
    ${FFMPEG_LIBRARIES}
)

if(BUILD_TESTING)
    add_executable(superpower_joint_update_test
        auto_aim/EKF/SuperPowerJointUpdateCheck.cpp
    )
    target_link_libraries(superpower_joint_update_test PRIVATE tasks)
    add_test(NAME superpower_joint_update
             COMMAND superpower_joint_update_test)
endif()
