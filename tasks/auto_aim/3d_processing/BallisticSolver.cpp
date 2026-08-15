// BallisticSolver.cpp
#include "3d_processing/BallisticSolver.h"
#include "utils/ThreadPool.h"

// 辅助函数：将角度限制在[-180, 180]范围内
float BallisticSolver::normalizeRad(float rad) {
    while (rad > M_PI) rad -= 2 * M_PI;
    while (rad < -M_PI) rad += 2 * M_PI;
    return rad;
}

// 辅助函数：计算最短角度差
float BallisticSolver::shortestRadDiff(float target, float current) {
    float diff = normalizeRad(target - current);
    return diff;
}

BallisticSolver::CalcPitchInfo BallisticSolver::calcTargetPitch(float horizontal_distance, float vertical_height, float v_bullet) {
    CalcPitchInfo result;
    result.valid = false;
    
    float g = 9.8f;
    float denominator = g * horizontal_distance;
    float v_bullet_square = v_bullet * v_bullet;
    float numerator_part1 = v_bullet_square;
    float numerator_part2_square = v_bullet_square * v_bullet_square - g * (g * horizontal_distance * horizontal_distance + 2 * vertical_height * v_bullet_square);
    if (numerator_part2_square < 0) {
        return result;
    }
    float tan_angle1 = (numerator_part1 + sqrt(numerator_part2_square)) / denominator;
    float tan_angle2 = (numerator_part1 - sqrt(numerator_part2_square)) / denominator;
    
    float pitch_rad1 = atan(tan_angle1);
    float pitch_rad2 = atan(tan_angle2);

    bool is_pitch1_smaller = pitch_rad1 <= pitch_rad2;
    result.target_pitch_result_smaller = is_pitch1_smaller ? pitch_rad1 : pitch_rad2;
    result.target_pitch_result_larger = (!is_pitch1_smaller) ? pitch_rad1 : pitch_rad2;
    result.valid = true;

    return result;
}

// 统一的弹道模拟函数
BallisticSolver::TrajectorySimulationResult BallisticSolver::simulateTrajectory2D(
    double v_bullet, double pitch_rad, double target_distance,
    double max_flight_time, double dt, double min_height,
    bool calculate_height, cv::Point2d target_point_2d) {
    
    double g = 9.8;
    TrajectorySimulationResult result;
    result.valid = false;
    result.nearest_distance = std::numeric_limits<double>::max();

    // 计算弹丸截面积
    double bullet_radius = ballisticParams.bullet_diameter / 2.0;
    double cross_section_area = M_PI * bullet_radius * bullet_radius;

    // 初始化状态
    State state;
    state.x = 0;
    state.z = 0;
    state.vx = v_bullet * std::cos(pitch_rad);
    state.vz = v_bullet * std::sin(pitch_rad);

    State prev_state = state;
    double time = 0;
    
    while (time <= max_flight_time && state.z >= min_height) {
        // 使用二阶欧拉法
        State accel1 = computeAcceleration(state, ballisticParams.drag_coeff, ballisticParams.air_density,
                                         cross_section_area, ballisticParams.bullet_mass, g);
        
        // 预测中间状态
        State mid_state;
        mid_state.x = state.x + 0.5 * dt * accel1.x;
        mid_state.z = state.z + 0.5 * dt * accel1.z;
        mid_state.vx = state.vx + 0.5 * dt * accel1.vx;
        mid_state.vz = state.vz + 0.5 * dt * accel1.vz;
        
        // 使用中间状态重新计算加速度
        State accel2 = computeAcceleration(mid_state, ballisticParams.drag_coeff, ballisticParams.air_density,
                                         cross_section_area, ballisticParams.bullet_mass, g);
        
        // 更新状态
        prev_state = state;
        state.x += dt * accel2.x;
        state.z += dt * accel2.z;
        state.vx += dt * accel2.vx;
        state.vz += dt * accel2.vz;
        
        time += dt;
        
        if (calculate_height) {
            // 计算命中高度
            if (state.x >= target_distance) {
                // 增加数值稳定性检查
                double delta_x = state.x - prev_state.x;
                if (std::abs(delta_x) < 1e-12) {
                    result.hit_height = state.z;
                } else {
                    double alpha = (target_distance - prev_state.x) / delta_x;
                    result.hit_height = prev_state.z + alpha * (state.z - prev_state.z);
                }
                result.valid = true;
                break;
            }
        } else {
            // 计算最近点
            cv::Point2d current_point(state.x, state.z);
            double current_distance = cv::norm(current_point - target_point_2d);
            
            if (current_distance < result.nearest_distance) {
                result.nearest_distance = current_distance;
                result.nearest_point_2d = current_point;
                result.prev_point_2d = cv::Point2d(prev_state.x, prev_state.z);
            }
        }
        
        // 通用终止条件 检查是否需要提前终止（速度过小）
        if (sqrt(state.vx*state.vx + state.vz*state.vz) < 0.1) {
            break;
        }
    }
    
    if (!calculate_height && result.nearest_distance < std::numeric_limits<double>::max()) {
        result.valid = true;
    }

    return result;
}

// 计算加速度（二维版本）
BallisticSolver::State BallisticSolver::computeAcceleration(const State& state, double drag_coeff, 
                                                          double air_density, double cross_section_area, 
                                                          double bullet_mass, double g) {
    State acceleration;
    double vel = sqrt(state.vx*state.vx + state.vz*state.vz);
    
    if (vel < 1e-6) {
        acceleration.vx = 0;
        acceleration.vz = -g;
        acceleration.x = state.vx;
        acceleration.z = state.vz;
        return acceleration;
    }
    
    // 计算空气阻力
    double drag_force = 0.5 * drag_coeff * air_density * cross_section_area * vel * vel;
    double drag_accel = drag_force / bullet_mass;
    
    // 阻力加速度分量
    acceleration.vx = -drag_accel * state.vx / vel;
    acceleration.vz = -g - drag_accel * state.vz / vel;
    
    // 位置导数就是速度
    acceleration.x = state.vx;
    acceleration.z = state.vz;
    
    return acceleration;
}

// 修改后的最近点计算函数
cv::Point3d BallisticSolver::calcNearestPointWithAirResistance(cv::Point3d target_pos, cv::Point3d self_pos, 
                                                              cv::Point2d aim_yaw_pitch, float v_bullet) {
    double max_flight_time = 5.0f;
    double dt = 1e-2;
    double min_height = -10.0f;

    // 将目标点投影到弹道平面
    // 弹道平面由发射方向和重力方向确定
    double horizontal_distance = sqrt(target_pos.x * target_pos.x + target_pos.y * target_pos.y);
    double distance_in_2d = horizontal_distance * std::cos(std::atan2(-target_pos.x, target_pos.y) - aim_yaw_pitch.x);
    cv::Point2d target_point_2d(distance_in_2d, target_pos.z);

    // 使用统一的弹道模拟函数
    TrajectorySimulationResult result = simulateTrajectory2D(
        v_bullet, aim_yaw_pitch.y, distance_in_2d, max_flight_time, dt, min_height,
        false, target_point_2d);

    if (!result.valid) {
        return self_pos;
    }

    // 使用最近两个点进行线性插值
    cv::Point2d p1 = result.prev_point_2d;
    cv::Point2d p2 = result.nearest_point_2d;
    cv::Point2d target_2d = target_point_2d;

    // 计算线段p1-p2上离目标点最近的点
    cv::Point2d line_vec = p2 - p1;
    cv::Point2d target_vec = target_2d - p1;
    
    double line_length_squared = line_vec.dot(line_vec);
    cv::Point2d nearest_2d;
    if (line_length_squared < 1e-12) {
        nearest_2d = p2; // 简单返回其中一个点
    } else {
        double t = target_vec.dot(line_vec) / line_length_squared;
        nearest_2d = p1 + t * line_vec;
    }

    // 将二维结果转换回三维坐标
    return cv::Point3d(
        nearest_2d.x * (-std::sin(aim_yaw_pitch.x)),
        target_pos.y * std::cos(aim_yaw_pitch.x), 
        nearest_2d.y
    );
}

// 修改弹道高度模拟函数（内部使用统一的模拟函数）
BallisticSolver::CalcPitchInfo BallisticSolver::calcTargetPitchWithAirResistance(
    float horizontal_distance, float vertical_height, float v_bullet) {
    
    CalcPitchInfo result;
    result.valid = false;

    float min_pitch = -60 * M_PI / 180;
    float mid_pitch = 80 * M_PI / 180;
    float max_pitch = 89 * M_PI / 180;
    int start_check_n_low = 48;
    int start_check_n_high = 16;
    int max_refine_times = 10;
    float tolerance = 1e-3;

    double max_flight_time = 5.0f;
    double dt = 1e-2;
    double min_height = -10.0f;

    // 初始计算n_low+n_high个点的[pitch-目标距离处y]对应关系
    float mid_aimed_h_at_1 = std::tan(mid_pitch);
    float max_aimed_h_at_1 = std::tan(max_pitch);
    std::vector<TrajectoryInfo> start_check_results(start_check_n_low + start_check_n_high);
    
    std::vector<StartCheckThreadInfo> start_check_thread_infos(start_check_n_low + start_check_n_high);
    for (int start_check_index = 0; start_check_index < start_check_n_low + start_check_n_high; start_check_index += 1) {
        float pitch_rad = 0.0;
        if (start_check_index < start_check_n_low) {
            pitch_rad = min_pitch + (mid_pitch - min_pitch) * (static_cast<float>(start_check_index) / static_cast<float>(start_check_n_low));
        } else {
            float aimed_h_at_1 = mid_aimed_h_at_1 + 
                                (max_aimed_h_at_1 - mid_aimed_h_at_1) * 
                                (static_cast<float>(start_check_index - start_check_n_low) / 
                                (static_cast<float>(start_check_n_high) - 1.0));
            pitch_rad = std::atan(aimed_h_at_1);
        }
        start_check_thread_infos[start_check_index].thread_index = start_check_index;
        start_check_thread_infos[start_check_index].pitch = pitch_rad;
    }
    
    // 并行粗搜索：交给线程池，替代 std::execution::par（隐式线程池、线程数不可控）
    ::utils::threadPool().parallel_for_each(start_check_thread_infos.begin(), start_check_thread_infos.end(),
        [&](StartCheckThreadInfo& start_check_thread_info) {
            float pitch_rad = start_check_thread_info.pitch;
            int start_check_index = start_check_thread_info.thread_index;
            
            // 使用统一的弹道模拟函数计算高度
            TrajectorySimulationResult simulate_result = simulateTrajectory2D(
                v_bullet, pitch_rad, horizontal_distance, max_flight_time, dt, min_height, true);
            
            start_check_results[start_check_index].pitch = pitch_rad;
            if (simulate_result.valid) {
                start_check_results[start_check_index].hit_height = simulate_result.hit_height;
            } else {
                if (start_check_index == 0) {
                    start_check_results[start_check_index].hit_height = min_height;
                } else {
                    start_check_results[start_check_index].hit_height = start_check_results[start_check_index - 1].hit_height;
                }
            }
        }
    );

    // 查找所有与目标高度差距符号转变的位置
    std::vector<RefineInfo> refine_infos;
    for (int start_check_index = 1; start_check_index < start_check_n_low + start_check_n_high; start_check_index += 1) {
        TrajectoryInfo trajectory_info1 = start_check_results[start_check_index-1];
        TrajectoryInfo trajectory_info2 = start_check_results[start_check_index];
        if ((trajectory_info1.hit_height < vertical_height) && (trajectory_info2.hit_height >= vertical_height)) {
            RefineInfo refine_info;
            refine_info.lower_pitch_trajectory = trajectory_info1;
            refine_info.upper_pitch_trajectory = trajectory_info2;
            refine_info.rising = true;
            refine_info.valid = true;
            refine_infos.push_back(refine_info);
        } else if ((trajectory_info1.hit_height >= vertical_height) && (trajectory_info2.hit_height < vertical_height)) {
            RefineInfo refine_info;
            refine_info.lower_pitch_trajectory = trajectory_info1;
            refine_info.upper_pitch_trajectory = trajectory_info2;
            refine_info.rising = false;
            refine_info.valid = true;
            refine_infos.push_back(refine_info);
        }
    }
    
    if (refine_infos.size() == 0) {
        return result;
    }
    
    // 二分查找符号转变点的准确pitch值
    std::vector<float> optional_pitchs(refine_infos.size());
    std::vector<RefineThreadInfo> refine_thread_infos(refine_infos.size());
    
    for (int optional_pitch_index = 0; optional_pitch_index < refine_infos.size(); optional_pitch_index += 1) {
        refine_thread_infos[optional_pitch_index].thread_index = optional_pitch_index;
        refine_thread_infos[optional_pitch_index].refine_info = refine_infos[optional_pitch_index];
    }
    
    // 并行细化：同上
    ::utils::threadPool().parallel_for_each(refine_thread_infos.begin(), refine_thread_infos.end(),
        [&](RefineThreadInfo& refine_thread_info) {
            RefineInfo refine_info = refine_thread_info.refine_info;
            int refine_step = 0;
            float pitch_lower = refine_info.lower_pitch_trajectory.pitch;
            float pitch_upper = refine_info.upper_pitch_trajectory.pitch;
            float pitch_mid;
            
            while (refine_step < max_refine_times) {
                pitch_mid = (pitch_lower + pitch_upper) / 2;
                
                // 使用统一的弹道模拟函数
                TrajectorySimulationResult simulate_result = simulateTrajectory2D(
                    v_bullet, pitch_mid, horizontal_distance, max_flight_time, dt, min_height, true);
                
                if (!simulate_result.valid) {
                    break;
                }
                
                float mid_pitch_hit_height = simulate_result.hit_height;
                if (std::abs(vertical_height - mid_pitch_hit_height) <= tolerance) {
                    break;
                }
                
                bool in_first_half = (mid_pitch_hit_height < vertical_height) ^ refine_info.rising;
                if (in_first_half) {
                    pitch_upper = pitch_mid;
                } else {
                    pitch_lower = pitch_mid;
                }
                refine_step += 1;
            }
            optional_pitchs[refine_thread_info.thread_index] = pitch_mid;
        }
    );
    
    result.target_pitch_result_smaller = *std::min_element(optional_pitchs.begin(), optional_pitchs.end());
    result.target_pitch_result_larger = *std::max_element(optional_pitchs.begin(), optional_pitchs.end());
    result.valid = true;
    return result;
}

BallisticInfo BallisticSolver::calcBallisticAngle(float x_camera, float y_camera, float z_camera, 
                                  float v_bullet, float cur_pitch, float cur_yaw) {
    BallisticInfo result;
    result.valid = false;
    
    // 转换单位：mm到m
    x_camera = x_camera / 1000.0f;
    y_camera = y_camera / 1000.0f; 
    z_camera = z_camera / 1000.0f;

    // 转换为水平坐标系
    float x_standard = x_camera;
    float y_standard = z_camera*sin(cur_pitch) - y_camera*cos(cur_pitch);
    float z_standard = z_camera*cos(cur_pitch) + y_camera*sin(cur_pitch);
    float r_standard = sqrt(x_standard*x_standard + z_standard*z_standard);

    // 计算目标yaw弧度
    float target_delta_yaw = atan2(-x_standard, z_standard) * 1.0;

    // 求解弹道方程
    CalcPitchInfo pitch_info = calcTargetPitchWithAirResistance(r_standard, y_standard, v_bullet);
    if (!pitch_info.valid) {
        return result;
    }
    float final_pitch_rad = pitch_info.target_pitch_result_smaller;

    result.delta_pitch_rad = final_pitch_rad - cur_pitch + extra_delta_pitch;
    result.delta_yaw_rad = target_delta_yaw;
    result.valid = true;
    return result;
}

/*
// 原四阶Runge-Kutta法的simulateTrajectory函数（已替换为二阶欧拉法）
BallisticSolver::SimulateTrajectoryInfo BallisticSolver::simulateTrajectory_original_RK4(
    double v_bullet, double pitch_rad, double horizontal_distance,
    double max_flight_time, double dt, double min_height) {
    
    double g = 9.8;
    SimulateTrajectoryInfo result;
    result.valid = false;

    // 计算弹丸截面积
    double bullet_radius = ballisticParams.bullet_diameter / 2.0;
    double cross_section_area = M_PI * bullet_radius * bullet_radius;

    // 初始化状态
    State state;
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.vx = v_bullet * std::cos(pitch_rad);
    state.vz = v_bullet * std::sin(pitch_rad);
    state.vy = 0;

    double time = 0;
    while (time <= max_flight_time && state.z >= min_height) {
        // 四阶Runge-Kutta法
        State k1 = computeAcceleration(state, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state2;
        state2.x = state.x + 0.5 * dt * k1.x;
        state2.y = state.y + 0.5 * dt * k1.y;
        state2.z = state.z + 0.5 * dt * k1.z;
        state2.vx = state.vx + 0.5 * dt * k1.vx;
        state2.vy = state.vy + 0.5 * dt * k1.vy;
        state2.vz = state.vz + 0.5 * dt * k1.vz;
        State k2 = computeAcceleration(state2, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state3;
        state3.x = state.x + 0.5 * dt * k2.x;
        state3.y = state.y + 0.5 * dt * k2.y;
        state3.z = state.z + 0.5 * dt * k2.z;
        state3.vx = state.vx + 0.5 * dt * k2.vx;
        state3.vy = state.vy + 0.5 * dt * k2.vy;
        state3.vz = state.vz + 0.5 * dt * k2.vz;
        State k3 = computeAcceleration(state3, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state4;
        state4.x = state.x + dt * k3.x;
        state4.y = state.y + dt * k3.y;
        state4.z = state.z + dt * k3.z;
        state4.vx = state.vx + dt * k3.vx;
        state4.vy = state.vy + dt * k3.vy;
        state4.vz = state.vz + dt * k3.vz;
        State k4 = computeAcceleration(state4, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        // 更新状态
        state.x += dt * (k1.x + 2*k2.x + 2*k3.x + k4.x) / 6.0;
        state.y += dt * (k1.y + 2*k2.y + 2*k3.y + k4.y) / 6.0;
        state.z += dt * (k1.z + 2*k2.z + 2*k3.z + k4.z) / 6.0;
        state.vx += dt * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx) / 6.0;
        state.vy += dt * (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy) / 6.0;
        state.vz += dt * (k1.vz + 2*k2.vz + 2*k3.vz + k4.vz) / 6.0;
        
        time += dt;
        
        if (state.x >= horizontal_distance) {
            // 线性插值得到精确的命中高度
            double alpha = (horizontal_distance - (state.x - dt * (k1.x + 2*k2.x + 2*k3.x + k4.x) / 6.0)) / 
                          (state.x - (state.x - dt * (k1.x + 2*k2.x + 2*k3.x + k4.x) / 6.0));
            result.hit_height = (state.z - dt * (k1.z + 2*k2.z + 2*k3.z + k4.z) / 6.0) + 
                               alpha * (state.z - (state.z - dt * (k1.z + 2*k2.z + 2*k3.z + k4.z) / 6.0));
            result.valid = true;
            break;
        }
        
        if (sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz) < 0.1) {
            break;
        }
    }

    return result;
}
*/

/*
// 原一阶欧拉法的simulateTrajectory函数（已替换为二阶欧拉法）
BallisticSolver::SimulateTrajectoryInfo BallisticSolver::simulateTrajectory_original_Euler(
    double v_bullet, double pitch_rad, double horizontal_distance,
    double max_flight_time, double dt, double min_height) {
    
    double g = 9.8;
    uint32_t time_step = 0;

    SimulateTrajectoryInfo result;
    result.valid = false;

    // 计算弹丸截面积
    double bullet_radius = ballisticParams.bullet_diameter / 2.0;
    double cross_section_area = M_PI * bullet_radius * bullet_radius;

    double pos_x = 0;
    double pos_y = 0;
    double vel_x = v_bullet * std::cos(pitch_rad);
    double vel_y = v_bullet * std::sin(pitch_rad);
    while (time_step * dt <= max_flight_time && pos_y >= min_height) {
        // 更新位置
        pos_x += vel_x * dt;
        pos_y += vel_y * dt;

        // 计算当前速度大小
        double vel = std::sqrt(vel_x * vel_x + vel_y * vel_y);
        if (vel < 0.1) {
            break;
        }
        
        if (pos_x >= horizontal_distance) {
            result.hit_height = pos_y - (pos_x - horizontal_distance) * (vel_y / vel_x);
            result.valid = true;
            break;
        }

        // 计算空气阻力（与速度平方成正比，方向与速度相反）
        double drag_force = 0.5 * ballisticParams.drag_coeff * ballisticParams.air_density * 
                           cross_section_area * vel * vel;
        // 计算阻力加速度分量
        double drag_accel_x = drag_force * vel_x / (ballisticParams.bullet_mass * vel);
        double drag_accel_y = drag_force * vel_y / (ballisticParams.bullet_mass * vel);
        // 更新速度（考虑空气阻力和重力）
        vel_x -= drag_accel_x * dt;
        vel_y -= (g + drag_accel_y) * dt;

        time_step += 1;
    }

    return result;
}
*/

/*
// 原四阶Runge-Kutta法的calcNearestPointWithAirResistance函数（已替换为二阶欧拉法）
cv::Point3d BallisticSolver::calcNearestPointWithAirResistance_original_RK4(cv::Point3d target_pos, cv::Point3d self_pos, 
                                                              cv::Point2d aim_yaw_pitch, float v_bullet) {
    double max_flight_time = 5.0f;
    double dt = 1e-4;
    double min_height = -100.0f;
    double g = 9.8;

    cv::Point3d nearest_point = self_pos;
    double min_target_dist = cv::norm(target_pos - self_pos);

    // 计算弹丸截面积
    double bullet_radius = ballisticParams.bullet_diameter / 2.0;
    double cross_section_area = M_PI * bullet_radius * bullet_radius;

    // 初始化状态
    State state;
    state.x = self_pos.x;
    state.y = self_pos.y;
    state.z = self_pos.z;
    state.vx = v_bullet * std::cos(aim_yaw_pitch.y) * (-std::sin(aim_yaw_pitch.x));
    state.vy = v_bullet * std::cos(aim_yaw_pitch.y) * std::cos(aim_yaw_pitch.x);
    state.vz = v_bullet * std::sin(aim_yaw_pitch.y);

    double time = 0;
    while (time <= max_flight_time && state.z >= min_height) {
        // 四阶Runge-Kutta法
        State k1 = computeAcceleration(state, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state2;
        state2.x = state.x + 0.5 * dt * k1.x;
        state2.y = state.y + 0.5 * dt * k1.y;
        state2.z = state.z + 0.5 * dt * k1.z;
        state2.vx = state.vx + 0.5 * dt * k1.vx;
        state2.vy = state.vy + 0.5 * dt * k1.vy;
        state2.vz = state.vz + 0.5 * dt * k1.vz;
        State k2 = computeAcceleration(state2, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state3;
        state3.x = state.x + 0.5 * dt * k2.x;
        state3.y = state.y + 0.5 * dt * k2.y;
        state3.z = state.z + 0.5 * dt * k2.z;
        state3.vx = state.vx + 0.5 * dt * k2.vx;
        state3.vy = state.vy + 0.5 * dt * k2.vy;
        state3.vz = state.vz + 0.5 * dt * k2.vz;
        State k3 = computeAcceleration(state3, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        State state4;
        state4.x = state.x + dt * k3.x;
        state4.y = state.y + dt * k3.y;
        state4.z = state.z + dt * k3.z;
        state4.vx = state.vx + dt * k3.vx;
        state4.vy = state.vy + dt * k3.vy;
        state4.vz = state.vz + dt * k3.vz;
        State k4 = computeAcceleration(state4, ballisticParams.drag_coeff, ballisticParams.air_density,
                                     cross_section_area, ballisticParams.bullet_mass, g);
        
        // 更新状态
        state.x += dt * (k1.x + 2*k2.x + 2*k3.x + k4.x) / 6.0;
        state.y += dt * (k1.y + 2*k2.y + 2*k3.y + k4.y) / 6.0;
        state.z += dt * (k1.z + 2*k2.z + 2*k3.z + k4.z) / 6.0;
        state.vx += dt * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx) / 6.0;
        state.vy += dt * (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy) / 6.0;
        state.vz += dt * (k1.vz + 2*k2.vz + 2*k3.vz + k4.vz) / 6.0;
        
        time += dt;
        
        // 计算当前点到目标的距离
        cv::Point3d current_pos(state.x, state.y, state.z);
        double target_dist = cv::norm(target_pos - current_pos);
        if (target_dist < min_target_dist) {
            nearest_point = current_pos;
            min_target_dist = target_dist;
        }
        
        if (sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz) < 0.1) {
            break;
        }
    }

    return nearest_point;
}
*/
