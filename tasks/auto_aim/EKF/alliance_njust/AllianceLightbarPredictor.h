#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include "2d_armor_detector/Armor.h"
#include "3d_processing/RestFrame.h"
#include "EKF/superpower_tongji/SuperPowerPredictor.h"

class AllianceLightbarPredictor {
public:
    struct ProjectedLightbar { int id=-1; cv::Point2f upper; cv::Point2f lower; bool visible=false; };
    AllianceLightbarPredictor(std::shared_ptr<YAML::Node> config_file_ptr,
                              std::shared_ptr<RestFrame> rest_frame,
                              ArmorType::ArmorType armor_class);
    bool update(const std::vector<ArmorResult>& armors, double update_time);
    void missUpdate(double update_time);
    void clear();
    EKFTargetPrediction predict(double predict_time) const;
    EKFTargetState state() const;
    EKFTargetDebugState debugState() const;
    bool ready() const;
    bool hasState() const;
    int debugFlipFlag() const { return debug_flip_flag_; }
    std::vector<ProjectedLightbar> projectedLightbars() const;
private:
    static constexpr int kStateDim=11;
    using StateVector=Eigen::Matrix<double,kStateDim,1>;
    using Covariance=Eigen::Matrix<double,kStateDim,kStateDim>;
    using ProcessNoise=Eigen::Matrix<double,kStateDim,kStateDim>;
    using ObservationNoise=Eigen::Matrix<double,4,4>;
    using ObservationJacobian=Eigen::Matrix<double,4,kStateDim>;
    using KalmanGain=Eigen::Matrix<double,kStateDim,4>;
    enum StateIndex:int { X=0,Y=1,Z=2,VX=3,VY=4,VZ=5,YAW=6,W=7,RF=8,RL=9,HL=10 };
    struct Config {
        double radius_forward_min=.10,radius_forward_max=.40,radius_lateral_min=.10,radius_lateral_max=.40;
        double height_lateral_min=-.20,height_lateral_max=.20;
        double noise_x=1e-4,noise_y=1e-4,noise_z=1e-8,noise_vx=2e-1,noise_vy=2e-1,noise_vz=1e-8;
        double noise_yaw=1e-3,noise_w=1.,noise_rf=1e-8,noise_rl=1e-8,noise_hl=1e-8;
        double observation_pixel_variance=40.;
        double yaw_full_max_rad=70.*3.14159265358979323846/180.;
        double yaw_part_max_rad=90.*3.14159265358979323846/180.;
        double armor_pitch_rad=15.*3.14159265358979323846/180.;
        double initial_radius_m=.20,max_dt_s=.10; int min_update_count=10; double converge_cov_xy=1.;
    } config_;
    struct LightbarGeometry { Eigen::Vector3d upper=Eigen::Vector3d::Zero(),lower=Eigen::Vector3d::Zero(); };
    struct Observable { std::array<LightbarGeometry,8> world; std::array<Eigen::Vector2d,8> upper2d,lower2d; std::array<bool,8> visible; } observable_;
    struct ObservedBar { Eigen::Vector2d upper=Eigen::Vector2d::Zero(),lower=Eigen::Vector2d::Zero(); };
    std::shared_ptr<RestFrame> rest_frame_; cv::Mat camera_matrix_,dist_coeffs_; double fx_=1.,fy_=1.;
    double delta_x_m_=0.,delta_y_m_=0.,delta_z_m_=0.;
    StateVector x_=StateVector::Zero(); Covariance P_=Covariance::Identity(); ProcessNoise Q_=ProcessNoise::Zero(); ObservationNoise R_=ObservationNoise::Identity();
    bool initialized_=false,large_armor_=false,has_update_time_=false; double last_update_time_=0.,last_dt_s_=0.;
    unsigned long long update_frames_=0; int update_count_=0,lost_frames_=0,last_matched_armor_id_=-1,debug_flip_flag_=1; EKFTargetDebugState debug_;
    Eigen::Matrix3d worldFromGunRotation() const; Eigen::Vector3d cameraPositionWorld() const;
    Eigen::Vector3d worldToCameraPnp(const Eigen::Vector3d&) const; std::optional<Eigen::Vector2d> projectWorldPoint(const Eigen::Vector3d&) const;
    std::array<Eigen::Vector3d,4> armorCenters(const StateVector&) const; std::array<Eigen::Vector3d,4> armorRadials(const StateVector&) const;
    LightbarGeometry lightbarGeometry(const StateVector&,int) const; void refreshObservable(); bool initializeFromPnp(const std::vector<ArmorResult>&);
    bool correct(const std::vector<ArmorResult>&); bool updateOneBar(const ObservedBar&,int); ObservationJacobian makeObservationJacobian(int) const;
    void predictStep(double); double consumeDt(double); bool diverged() const; void resetCovariance(); void configure(const YAML::Node&);
    static bool finitePoint(const cv::Point2f&); static bool validPixelArmor(const ArmorResult&); static std::array<ObservedBar,2> observedBars(const ArmorResult&);
    static double squaredDistance(const Eigen::Vector2d&,const Eigen::Vector2d&); static double wrapAngle(double);
};
