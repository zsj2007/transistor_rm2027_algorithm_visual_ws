#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "EKF/SuperPowerTracker.h"

struct EKFTargetObservation {
    double x; double y; double z; double yaw; double t;
};
struct EKFPredictedArmor {
    double x; double y; double z; double r; double yaw;
};
struct EKFTargetPrediction {
    double center_x=0.0, center_y=0.0, center_z=0.0, alternate_z=0.0;
    double r1=0.0, r2=0.0, h=0.0, yaw=0.0, w=0.0;
    int rotation_direction=1;
    std::vector<EKFPredictedArmor> armors;
};
struct EKFTargetState {
    double center_x=0.0, center_y=0.0, center_z=0.0;
    double center_vx=0.0, center_vy=0.0, center_vz=0.0;
    double r1=0.0, r2=0.0, h=0.0, yaw=0.0, w=0.0;
    unsigned long long update_frames=0;
};
struct EKFTargetDebugState {
    double dt_s=0.0; bool time_discontinuity=false;
    std::string tracker_state="LOST", tracker_state_before="LOST";
    int matched_id=-1; bool measurement_valid=false; bool updated=false;
    int lost_frames=0; double nis=-1.0; double position_error_m=-1.0; double yaw_error_deg=-1.0;
    bool phase_observer_valid=false; double phase_delta=0.0; double phase_w_instant=0.0; double phase_w_filtered=0.0;
    bool direction_reversal=false; bool armor_switched=false;
    bool joint_pair_requested=false; bool joint_pair_used=false; int joint_second_id=-1;
    double joint_nis=std::numeric_limits<double>::quiet_NaN();
    double joint_second_position_error_m=std::numeric_limits<double>::quiet_NaN();
    double joint_second_angle_error_rad=std::numeric_limits<double>::quiet_NaN();
    std::string joint_status="SINGLE";
    bool recovered=false; bool phase_w_applied=false; bool pending_sign_conflict=false;
    bool temp_lost_recovery=false; bool candidate_is_switch=false; bool topology_event=false;
    int best_id=-1;
    double measurement_yaw=std::numeric_limits<double>::quiet_NaN();
    double predicted_yaw=std::numeric_limits<double>::quiet_NaN();
    double yaw_innovation=std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix<double,4,1> measurement=Eigen::Matrix<double,4,1>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double,4,1> pre_predicted=measurement;
    Eigen::Matrix<double,4,1> post_predicted=measurement;
    Eigen::Matrix<double,3,1> pre_residual=Eigen::Matrix<double,3,1>::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double,3,1> post_residual=pre_residual;
    double pre_position_error=std::numeric_limits<double>::quiet_NaN();
    double post_position_error=std::numeric_limits<double>::quiet_NaN();
    double residual_radial=std::numeric_limits<double>::quiet_NaN();
    double residual_tangential=std::numeric_limits<double>::quiet_NaN();
    double nis_xyz=std::numeric_limits<double>::quiet_NaN();
    double nis_yaw=std::numeric_limits<double>::quiet_NaN();
    double yaw_variance_scale=1.0;
    double hypothetical_scaled_nis=std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix<double,4,1> hypothetical_scaled_nis_contribution=Eigen::Matrix<double,4,1>::Constant(std::numeric_limits<double>::quiet_NaN());
    double r1_m=std::numeric_limits<double>::quiet_NaN();
    double r2_m=std::numeric_limits<double>::quiet_NaN();
    double h_m=std::numeric_limits<double>::quiet_NaN();
    double p_r1_m2=std::numeric_limits<double>::quiet_NaN();
    double p_r2_m2=std::numeric_limits<double>::quiet_NaN();
    double p_h_m2=std::numeric_limits<double>::quiet_NaN();
    double p_x_m2=std::numeric_limits<double>::quiet_NaN();
    double p_vx_m2_s2=std::numeric_limits<double>::quiet_NaN();
    double p_y_m2=std::numeric_limits<double>::quiet_NaN();
    double p_vy_m2_s2=std::numeric_limits<double>::quiet_NaN();
    int armor_parity=-1; bool geometry_valid=false; bool geometry_update_allowed=false;
    bool geometry_preserved=false; int current_armor_id=-1;
};

class SuperPowerTongjiPredictor {
public:
    SuperPowerTongjiPredictor(const EKFTargetObservation& initial_observation,
                              double initial_radius_mm,
                              std::shared_ptr<YAML::Node> config_file_ptr);
    void update(const EKFTargetObservation& observation);
    void updatePair(const EKFTargetObservation& primary, const EKFTargetObservation& secondary);
    void missUpdate(double update_time);
    void clear();
    EKFTargetPrediction predict(double predict_time) const;
    EKFTargetState state() const;
    EKFTargetDebugState debugState() const;
    bool ready() const;
    bool hasState() const;
    int debugFlipFlag() const { return debug_flip_flag_; }
private:
    static sp_ekf::ArmorObservation toSuperPower(const EKFTargetObservation& observation);
    static double toProjectYaw(double superpower_angle);
    static double wrapAngle(double angle);
    void resetTracker();
    void initializeFromObservation(const EKFTargetObservation& observation);
    void updateImpl(const EKFTargetObservation& primary,
                    const std::optional<EKFTargetObservation>& secondary);
    void warnTimeIssue(const char* reason, double update_time, double dt);
    sp_ekf::TrackerConfig config_;
    std::unique_ptr<sp_ekf::Tracker> tracker_;
    sp_ekf::TrackerResult last_result_;
    std::optional<sp_ekf::ArmorObservation> last_observation_;
    double last_update_time_=0.0, last_dt_s_=0.0;
    unsigned long long update_frames_=0;
    int debug_flip_flag_=1;
    bool has_update_time_=false, timestamp_warning_active_=false, time_discontinuity_=false;
};
