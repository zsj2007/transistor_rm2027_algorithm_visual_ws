// users/visualizer.cpp — 独立可视化进程（无 ROS2）
//
// 由 transistor_rm2026_algorithm_visual_ws 的 auto_aim_visualizer 节点移植：
//   - 去掉 rclcpp / cv_bridge / ROS2 话题，改为从共享内存读取算法进程
//     （AutoAimPipeline Stage4）每帧发布的调试数据；
//   - 绘制内容与旧节点一致：主检测画面（灯条/装甲板/解算结果/云台坐标系）、
//     云台 yaw 曲线、RMM 平面可视化、通用调试示波器；
//   - 用法：./build/visualizer configs/infantry_video.yaml
//     （需先运行 ./build/infantry_debug configs/infantry_video.yaml 作为数据源）
//
// 可选参数：
//   -headless  无窗口模式：不创建 OpenCV 窗口，仅打印接收统计，便于无显示环境调试
//   -frames=N  接收 N 帧后自动退出（headless 模式默认 30，图形模式默认 0=一直运行）

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits.h>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "shm/VisualizerShm.h"

namespace {

volatile std::sig_atomic_t g_exit_requested = 0;

void handleExitSignal(int)
{
    g_exit_requested = 1;
}

struct VisualizerConfig {
    struct DrawConfig {
        bool main_result = true;
        bool status_text = true;
        bool ground_stable_point = true;
        bool lights = true;
        bool armors = true;
        bool solved_armors = true;
        bool joint_ekf_pair = true;
        bool predictions = true;
        bool yaw_curve = true;
        bool rmm = true;
        bool common_debug_oscilloscope = true;
        bool gimbal_coordinate = true;
    };

    bool enable = true;
    bool show_windows = true;
    DrawConfig draw;

    static VisualizerConfig fromYaml(const YAML::Node& root)
    {
        VisualizerConfig config;
        const YAML::Node visualizer = root["visualizer"];
        if (!visualizer) {
            return config;
        }

        config.enable = readBool(visualizer, "enable", config.enable);
        config.show_windows = readBool(visualizer, "show_windows", config.show_windows);

        const YAML::Node draw = visualizer["draw"];
        if (draw) {
            config.draw.main_result = readBool(draw, "main_result", config.draw.main_result);
            config.draw.status_text = readBool(draw, "status_text", config.draw.status_text);
            config.draw.ground_stable_point =
                readBool(draw, "ground_stable_point", config.draw.ground_stable_point);
            config.draw.lights = readBool(draw, "lights", config.draw.lights);
            config.draw.armors = readBool(draw, "armors", config.draw.armors);
            config.draw.solved_armors = readBool(draw, "solved_armors", config.draw.solved_armors);
            config.draw.joint_ekf_pair = readBool(draw, "joint_ekf_pair", config.draw.joint_ekf_pair);
            config.draw.predictions = readBool(draw, "predictions", config.draw.predictions);
            config.draw.yaw_curve = readBool(draw, "yaw_curve", config.draw.yaw_curve);
            config.draw.rmm = readBool(draw, "rmm", config.draw.rmm);
            config.draw.common_debug_oscilloscope =
                readBool(draw, "common_debug_oscilloscope", config.draw.common_debug_oscilloscope);
            config.draw.gimbal_coordinate =
                readBool(draw, "gimbal_coordinate", config.draw.gimbal_coordinate);
        }

        return config;
    }

private:
    static bool readBool(const YAML::Node& node, const char* key, bool fallback)
    {
        const YAML::Node value = node[key];
        return value ? value.as<bool>() : fallback;
    }
};

constexpr double kPi = 3.14159265358979323846;

const std::array<std::string, 10> kArmorTypeStrings = {
    "Hero",
    "Engineer",
    "Infantry1",
    "Infantry2",
    "Infantry3",
    "Sentry",
    "Outpost",
    "Base",
    "Middle",
    "Nearest",
};

const std::array<std::string, 4> kPredictorTypeStrings = {
    "None",
    "RMM",
    "AutoSwitch",
    "SuperPowerEKF",
};

cv::Point2f toCvPoint(const visualizer_shm::Point2f& point)
{
    return {point.x, point.y};
}

template <typename PointArray>
std::array<cv::Point2f, 4> toQuad(const PointArray& points)
{
    std::array<cv::Point2f, 4> quad{};
    for (size_t i = 0; i < quad.size(); ++i) {
        quad[i] = toCvPoint(points[i]);
    }
    return quad;
}

void drawQuad(cv::Mat& image, const std::array<cv::Point2f, 4>& quad, const cv::Scalar& color, int thickness)
{
    for (size_t i = 0; i < quad.size(); ++i) {
        cv::line(image, quad[i], quad[(i + 1) % quad.size()], color, thickness);
    }
}

}  // namespace

class VisualizerApp {
public:
    VisualizerApp(const VisualizerConfig& config, std::shared_ptr<YAML::Node> config_file_ptr)
        : config_(config), reader_(config_file_ptr), start_time_(std::chrono::steady_clock::now())
    {
    }

    bool valid() const { return reader_.valid(); }

    void run(bool headless, int max_frames)
    {
        int received = 0;
        int64_t stale_printed_at_ms = -1;
        // Snapshot 约 14MB，必须放堆上（栈默认 8MB 会溢出）
        auto snap_holder = std::make_unique<visualizer_shm::Snapshot>();
        visualizer_shm::Snapshot& snap = *snap_holder;

        while (true) {
            if (g_exit_requested) {
                std::cout << "[visualizer] exit signal received" << std::endl;
                break;
            }
            const bool got_new = reader_.waitForSnapshot(snap, 200);
            if (got_new) {
                last_data_time_ = std::chrono::steady_clock::now();
                if (headless) {
                    printStats(snap);
                    ++received;
                    if (max_frames > 0 && received >= max_frames) {
                        std::cout << "[visualizer] received " << received << " frames, exiting" << std::endl;
                        break;
                    }
                } else {
                    renderAll(snap);
                    ++received;
                    if (max_frames > 0 && received >= max_frames) {
                        break;
                    }
                }
            } else if (!headless) {
                // 数据源未启动/已断开：显示等待画面
                const auto since_data = std::chrono::steady_clock::now() - last_data_time_;
                const int64_t stale_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(since_data).count();
                if (stale_ms > 1000 && stale_ms / 1000 != stale_printed_at_ms) {
                    stale_printed_at_ms = stale_ms / 1000;
                    std::cout << "[visualizer] waiting for algorithm data (" << stale_ms / 1000
                              << " s without frames)..." << std::endl;
                }
                if (stale_ms > 1000) {
                    showWaiting(stale_ms);
                }
            }

            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }
    }

private:
    VisualizerConfig config_;
    VisualizerShmReader reader_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_data_time_;
    std::chrono::steady_clock::time_point last_frame_time_;
    std::deque<double> frame_time_history_;
    double frame_time_sum_ = 0.0;

    float last_current_yaw_ = 0.0f;
    float last_target_yaw_ = 0.0f;
    int current_yaw_circle_ = 0;
    int target_yaw_circle_ = 0;
    std::deque<float> current_yaw_history_;
    std::deque<float> target_yaw_history_;

    void printStats(const visualizer_shm::Snapshot& snap)
    {
        const visualizer_shm::DebugData& d = snap.debug;
        std::cout << "[visualizer] frame_id=" << snap.frame_id
                  << " | lights=" << d.light_count
                  << " armors=" << d.armor_count
                  << " solved=" << d.solved_count
                  << " joint_ekf_pair=" << d.joint_ekf_count
                  << " alg_fps=" << snap.writer_fps
                  << " | yaw=" << d.yaw << " pitch=" << d.pitch
                  << " raw=" << snap.raw_frame.cols << "x" << snap.raw_frame.rows
                  << " rmm=" << snap.rmm_frame.bytes << "B"
                  << " cdo=" << snap.cdo_frame.bytes << "B"
                  << std::endl;
    }

    void showWaiting(int64_t stale_ms)
    {
        cv::Mat waiting = cv::Mat::zeros(240, 640, CV_8UC3);
        cv::putText(waiting,
            cv::format("Waiting for algorithm data... (%lld s)", static_cast<long long>(stale_ms)),
            cv::Point(30, 110),
            cv::FONT_HERSHEY_COMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            1);
        cv::putText(waiting,
            "start algorithm first, e.g. ./build/infantry_debug configs/infantry_video.yaml",
            cv::Point(30, 150),
            cv::FONT_HERSHEY_COMPLEX,
            0.5,
            cv::Scalar(255, 255, 255),
            1);
        cv::imshow("Armor Detection", waiting);
    }

    void renderAll(visualizer_shm::Snapshot& snap)
    {
        if (config_.draw.main_result && snap.raw_frame.bytes > 0) {
            cv::Mat display(snap.raw_frame.rows, snap.raw_frame.cols, snap.raw_frame.type,
                snap.raw_frame.data);
            drawMainResult(display, snap);
            cv::imshow("Armor Detection", display);
        }

        if (config_.draw.yaw_curve) {
            cv::imshow("Yaw Visualizer", renderYawCurve(snap.debug.yaw, snap.debug.mcu_command_yaw));
        }

        if (config_.draw.rmm && snap.rmm_frame.bytes > 0) {
            cv::Mat rmm(snap.rmm_frame.rows, snap.rmm_frame.cols, snap.rmm_frame.type,
                snap.rmm_frame.data);
            cv::imshow("RMM visualize", rmm);
        }

        if (config_.draw.common_debug_oscilloscope && snap.cdo_frame.bytes > 0) {
            cv::Mat cdo(snap.cdo_frame.rows, snap.cdo_frame.cols, snap.cdo_frame.type,
                snap.cdo_frame.data);
            cv::imshow("Common Debug Oscilloscope", cdo);
        }
    }

    void drawMainResult(cv::Mat& image, const visualizer_shm::Snapshot& snap)
    {
        const visualizer_shm::DebugData& msg = snap.debug;
        if (config_.draw.status_text) {
            drawStatusText(image, snap);
        }
        if (config_.draw.ground_stable_point) {
            cv::circle(image, toCvPoint(msg.ground_stable_point), 10, cv::Scalar(0, 255, 0), 2);
        }
        if (config_.draw.lights) {
            for (uint32_t i = 0; i < msg.light_count; ++i) {
                drawQuad(image, toQuad(msg.lights[i].vertices), cv::Scalar(0, 255, 0), 2);
            }
        }
        if (config_.draw.armors) {
            for (uint32_t i = 0; i < msg.armor_count; ++i) {
                const auto& armor = msg.armors[i];
                const auto corners = toQuad(armor.corners);
                drawQuad(image, corners, cv::Scalar(0, 255, 255), 2);
                drawQuad(image, toQuad(armor.light_bar_corners), cv::Scalar(255, 0, 0), 2);
                cv::putText(image,
                    cv::format("conf: %.2f", armor.confidence),
                    corners[0] + cv::Point2f(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
            }
        }
        if (config_.draw.solved_armors) {
            drawSolvedArmors(image, msg);
        }
        if (config_.draw.joint_ekf_pair) {
            drawJointEkfPairs(image, msg);
        }
        if (config_.draw.gimbal_coordinate) {
            drawGimbalCoordinate(image, msg);
        }
    }

    void drawStatusText(cv::Mat& image, const visualizer_shm::Snapshot& snap)
    {
        const visualizer_shm::DebugData& msg = snap.debug;
        const float render_fps = updateFrameRate();

        // ===== 布局参数 =====
        // 上方 10~185 基本被 TargetManager 左上状态框占用，
        // 这里把 visualizer 自己的状态信息整体下移，避免重叠。
        constexpr int panel_x = 10;
        constexpr int panel_y = 210;
        constexpr int text_x  = 20;
        constexpr int first_y = 235;
        constexpr int line_gap = 30;
        constexpr double font_scale = 0.7;
        constexpr int thickness = 1;

        const int panel_width = std::min(520, std::max(0, image.cols - 20));
        const int panel_height = 7 * line_gap + 20;

        if (panel_y < image.rows - 20) {
            const int safe_h = std::min(panel_height, image.rows - panel_y - 10);
            if (safe_h > 0) {
                cv::rectangle(
                    image,
                    cv::Rect(panel_x, panel_y, panel_width, safe_h),
                    cv::Scalar(20, 20, 20),
                    cv::FILLED
                );
            }
        }

        int y = first_y;

        auto put_line = [&](const std::string& text, const cv::Scalar& color)
        {
            if (y < image.rows - 5) {
                cv::putText(
                    image,
                    text,
                    cv::Point(text_x, y),
                    cv::FONT_HERSHEY_COMPLEX,
                    font_scale,
                    color,
                    thickness,
                    cv::LINE_AA
                );
            }
            y += line_gap;
        };

        put_line(
            cv::format("V: %.1f m/s, P: %.1f, Y: %.1f",
                    msg.bullet_velocity, msg.pitch, msg.yaw),
            cv::Scalar(0, 255, 0)
        );

        put_line(
            "enemy_color: " + std::string(msg.enemy_color),
            cv::Scalar(0, 255, 0)
        );

        const std::string armor_type = msg.armor_type < kArmorTypeStrings.size()
            ? kArmorTypeStrings[msg.armor_type]
            : "Unknown";

        const std::string predictor_type = msg.predictor_type < kPredictorTypeStrings.size()
            ? kPredictorTypeStrings[msg.predictor_type]
            : "Unknown";

        put_line(
            "aiming " + armor_type + ": " + predictor_type,
            cv::Scalar(0, 255, 0)
        );

        const auto since_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();

        put_line(
            cv::format("frame rate: %.1f fps", render_fps),
            cv::Scalar(0, 255, 0)
        );

        put_line(
            cv::format("algorithm rate: %.1f fps", snap.writer_fps),
            cv::Scalar(0, 255, 255)
        );

        put_line(
            cv::format("since visualizer start: %.4f s",
                    static_cast<float>(since_start_ms) / 1000.0f),
            cv::Scalar(0, 255, 0)
        );

        put_line(
            cv::format("frame_id: %llu",
                    static_cast<unsigned long long>(snap.frame_id)),
            cv::Scalar(0, 255, 0)
        );
    }

    float updateFrameRate()
    {
        const auto now = std::chrono::steady_clock::now();
        if (last_frame_time_.time_since_epoch().count() == 0) {
            last_frame_time_ = now;
            return 0.0f;
        }

        const double frame_time = std::chrono::duration<double>(now - last_frame_time_).count();
        last_frame_time_ = now;

        frame_time_history_.push_back(frame_time);
        frame_time_sum_ += frame_time;
        while (frame_time_history_.size() > 30) {
            frame_time_sum_ -= frame_time_history_.front();
            frame_time_history_.pop_front();
        }

        if (frame_time_sum_ <= 0.0) {
            return 0.0f;
        }
        return static_cast<float>(static_cast<double>(frame_time_history_.size()) / frame_time_sum_);
    }

    void drawJointEkfPairs(cv::Mat& image, const visualizer_shm::DebugData& msg)
    {
        constexpr double font_scale = 0.6;
        constexpr int font_thickness = 2;

        for (uint32_t i = 0; i < msg.joint_ekf_count; ++i) {
            const visualizer_shm::JointEkfPair& pair = msg.joint_ekf_pairs[i];
            const cv::Point2f center_a = toCvPoint(pair.center_a);
            const cv::Point2f center_b = toCvPoint(pair.center_b);
            const cv::Point2f midpoint = (center_a + center_b) * 0.5F;
            const cv::Scalar color =
                pair.ready ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
            const int line_thickness = pair.ready ? 4 : 2;

            cv::line(image, center_a, center_b, color, line_thickness, cv::LINE_AA);
            cv::circle(image, center_a, 7, color, 2, cv::LINE_AA);
            cv::circle(image, center_b, 7, color, 2, cv::LINE_AA);

            const std::string label = pair.ready
                ? "JointEKF Ready"
                : cv::format(
                    "JointEKF %d/2",
                    std::min(pair.consecutive_frames, 2));
            int baseline = 0;
            const cv::Size text_size = cv::getTextSize(
                label,
                cv::FONT_HERSHEY_SIMPLEX,
                font_scale,
                font_thickness,
                &baseline);
            cv::Point text_pos(
                cvRound(midpoint.x - text_size.width * 0.5F),
                cvRound(midpoint.y - 10.0F));
            text_pos.x = std::clamp(
                text_pos.x,
                0,
                std::max(0, image.cols - text_size.width));
            text_pos.y = std::clamp(
                text_pos.y,
                text_size.height,
                std::max(text_size.height, image.rows - baseline));

            cv::putText(
                image,
                label,
                text_pos,
                cv::FONT_HERSHEY_SIMPLEX,
                font_scale,
                cv::Scalar(0, 0, 0),
                font_thickness + 3,
                cv::LINE_AA);
            cv::putText(
                image,
                label,
                text_pos,
                cv::FONT_HERSHEY_SIMPLEX,
                font_scale,
                color,
                font_thickness,
                cv::LINE_AA);
        }
    }

    void drawSolvedArmors(cv::Mat& image, const visualizer_shm::DebugData& msg)
    {
        for (uint32_t i = 0; i < msg.solved_count; ++i) {
            const auto& result = msg.solved_results[i];
            const auto corners = toQuad(result.corners);
            const cv::Scalar contour_color = result.is_tracked_now ? cv::Scalar(0, 0, 255)
                                                                   : cv::Scalar(255, 0, 255);
            drawQuad(image, corners, contour_color, 2);
            drawQuad(image, toQuad(result.light_bar_corners), cv::Scalar(0, 255, 255), 2);

            if (config_.draw.predictions) {
                for (uint32_t p = 0; p < result.prediction_count; ++p) {
                    cv::circle(image, toCvPoint(result.predictions[p]), 3, cv::Scalar(255, 0, 255), -1);
                }
                cv::circle(image, toCvPoint(result.center_predicted), 3, cv::Scalar(0, 255, 255), -1);
            }
            cv::circle(image, toCvPoint(result.center), 3, cv::Scalar(0, 0, 255), -1);

            const std::string text = cv::format("N%d (%.2f)", result.number, result.confidence);
            cv::Point2f text_pos = corners[1] + cv::Point2f(0, -10);
            cv::putText(image, text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 0), 3);
            cv::putText(image, text, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 1);

            cv::Point2f track_pos = toCvPoint(result.center) + cv::Point2f(-30, 30);
            cv::putText(image, "TRACKING", track_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
        }
    }

    void drawGimbalCoordinate(cv::Mat& image, const visualizer_shm::DebugData& msg)
    {
        const cv::Point origin(image.cols - 180, image.rows - 120);
        const int axis_length = 70;
        cv::arrowedLine(image, origin, origin + cv::Point(axis_length, 0),
            cv::Scalar(0, 0, 255), 2);
        cv::arrowedLine(image, origin, origin + cv::Point(0, -axis_length),
            cv::Scalar(0, 255, 0), 2);
        cv::putText(image, "yaw +", origin + cv::Point(axis_length + 5, 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(image, "pitch +", origin + cv::Point(-15, -axis_length - 8),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        cv::putText(image, cv::format("roll: %.3f", msg.roll), origin + cv::Point(-20, 35),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, cv::format("cmd_yaw: %.3f", msg.mcu_command_yaw), origin + cv::Point(-20, 58),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    cv::Mat renderYawCurve(float current_yaw, float target_yaw)
    {
        constexpr int width = 800;
        constexpr int height = 800;
        cv::Mat display = cv::Mat::zeros(height, width, CV_8UC3);

        current_yaw = static_cast<float>(std::atan2(std::sin(current_yaw), std::cos(current_yaw)));
        target_yaw = static_cast<float>(std::atan2(std::sin(target_yaw), std::cos(target_yaw)));
        if (current_yaw < -kPi / 2 && last_current_yaw_ > kPi / 2) current_yaw_circle_ += 1;
        if (current_yaw > kPi / 2 && last_current_yaw_ < -kPi / 2) current_yaw_circle_ -= 1;
        if (target_yaw < -kPi / 2 && last_target_yaw_ > kPi / 2) target_yaw_circle_ += 1;
        if (target_yaw > kPi / 2 && last_target_yaw_ < -kPi / 2) target_yaw_circle_ -= 1;

        const float total_current_yaw = current_yaw + static_cast<float>(2 * kPi * current_yaw_circle_);
        const float total_target_yaw = target_yaw + static_cast<float>(2 * kPi * target_yaw_circle_);
        current_yaw_history_.push_back(total_current_yaw);
        target_yaw_history_.push_back(total_target_yaw);
        while (current_yaw_history_.size() > width) current_yaw_history_.pop_front();
        while (target_yaw_history_.size() > width) target_yaw_history_.pop_front();

        drawYawHistory(display, target_yaw_history_, cv::Scalar(0, 255, 0));
        drawYawHistory(display, current_yaw_history_, cv::Scalar(0, 0, 255));
        cv::putText(display, cv::format("total_target_yaw: %.3f", total_target_yaw),
            cv::Point(20, 50), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);
        cv::putText(display, cv::format("total_current_yaw: %.3f", total_current_yaw),
            cv::Point(20, 100), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 0, 255), 1);

        cv::line(display, cv::Point(400, 400),
            cv::Point(400 - static_cast<int>(std::sin(total_target_yaw) * 100),
                400 - static_cast<int>(std::cos(total_target_yaw) * 100)),
            cv::Scalar(0, 255, 0), 2);
        cv::line(display, cv::Point(400, 400),
            cv::Point(400 - static_cast<int>(std::sin(total_current_yaw) * 100),
                400 - static_cast<int>(std::cos(total_current_yaw) * 100)),
            cv::Scalar(0, 0, 255), 2);

        last_current_yaw_ = current_yaw;
        last_target_yaw_ = target_yaw;
        return display;
    }

    void drawYawHistory(cv::Mat& display, const std::deque<float>& history, const cv::Scalar& color)
    {
        if (history.empty()) return;

        const auto [min_it, max_it] = std::minmax_element(history.begin(), history.end());
        const float min_value = *min_it;
        const float max_value = *max_it;
        const float range = std::max(max_value - min_value, 0.1f);

        for (size_t i = 1; i < history.size(); ++i) {
            const int x0 = static_cast<int>(i - 1);
            const int x1 = static_cast<int>(i);
            const int y0 = 760 - static_cast<int>((history[i - 1] - min_value) / range * 300.0f);
            const int y1 = 760 - static_cast<int>((history[i] - min_value) / range * 300.0f);
            cv::line(display, cv::Point(x0, y0), cv::Point(x1, y1), color, 1);
        }
    }
};

int main(int argc, char* argv[])
{
    const std::string keys =
        "{help h usage ? | | 输出命令行参数说明}"
        "{headless      | false | 无窗口模式：不创建 OpenCV 窗口，仅打印接收统计}"
        "{frames f      | 0     | 接收多少帧后退出（headless 默认 30，图形模式 0=一直运行）}"
        "{@config-path  | configs/infantry_video.yaml | 位置参数，yaml配置文件路径}";

    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const std::string config_path = cli.get<std::string>(0);
    if (config_path.empty()) {
        cli.printMessage();
        return 1;
    }
    const bool headless = cli.get<bool>("headless");
    int max_frames = cli.get<int>("frames");
    if (max_frames <= 0) {
        max_frames = headless ? 30 : 0;
    }

    YAML::Node yaml = YAML::LoadFile(config_path);
    auto config_file_ptr = std::make_shared<YAML::Node>(yaml);

    const VisualizerConfig config = VisualizerConfig::fromYaml(yaml);
    if (!config.enable) {
        std::cout << "[visualizer] visualizer disabled in config, exit" << std::endl;
        return 0;
    }
    if (!headless && !config.show_windows) {
        std::cout << "[visualizer] show_windows=false, exit (use -headless for stats only)" << std::endl;
        return 0;
    }

    // Ctrl+C / Ctrl+Z / kill 都优雅退出（Ctrl+Z 默认会把进程挂起，这里改为退出）
    std::signal(SIGINT, handleExitSignal);
    std::signal(SIGTERM, handleExitSignal);
    std::signal(SIGTSTP, handleExitSignal);

    VisualizerApp app(config, config_file_ptr);
    if (!app.valid()) {
        std::cerr << "[visualizer] failed to attach shared memory, is the algorithm running?" << std::endl;
        return 1;
    }

    std::cout << "[visualizer] attached to shared memory, waiting for algorithm data..." << std::endl;
    app.run(headless, max_frames);
    return 0;
}
