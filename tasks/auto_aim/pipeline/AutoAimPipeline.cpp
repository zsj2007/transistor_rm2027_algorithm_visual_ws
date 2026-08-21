#include "pipeline/AutoAimPipeline.h"

#include "tools/cpu_affinity.hpp"
#include "tools/logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <stdexcept>

namespace {

Params loadDetectorParams(const std::shared_ptr<YAML::Node>& config_file_ptr, std::string* enemy_color_out)
{
    Params params;

    const int fix_enemy_color = (*config_file_ptr)["FIX_ENEMY_COLOR"].as<int>();
    std::string enemy_color;
    if (fix_enemy_color == 0) {
        enemy_color = "RED";
    } else if (fix_enemy_color == 1) {
        enemy_color = "BLUE";
    } else {
        enemy_color = (*config_file_ptr)["init_enemy_color"].as<std::string>();
    }

    if (enemy_color == "RED") {
        params.enemy_color = Params::RED;
    } else if (enemy_color == "BLUE") {
        params.enemy_color = Params::BLUE;
    } else if (enemy_color == "GREEN") {
        params.enemy_color = Params::GREEN;
    } else if (enemy_color == "BOTH") {
        params.enemy_color = Params::BOTH;
    } else {
        enemy_color = "GREEN";
        params.enemy_color = Params::GREEN;
    }

    params.min_light_height = (*config_file_ptr)["min_light_height"].as<int>();
    params.light_min_area = (*config_file_ptr)["light_min_area"].as<int>();
    params.light_max_area = (*config_file_ptr)["light_max_area"].as<int>();
    params.max_light_wh_ratio = (*config_file_ptr)["max_light_wh_ratio"].as<float>();
    params.min_light_wh_ratio = (*config_file_ptr)["min_light_wh_ratio"].as<float>();
    params.light_max_tilt_angle = (*config_file_ptr)["light_max_tilt_angle"].as<float>();

    if (enemy_color_out) {
        *enemy_color_out = enemy_color;
    }
    return params;
}

Params::EnemyColor toEnemyColor(const std::string& enemy_color)
{
    if (enemy_color == "RED") return Params::RED;
    if (enemy_color == "BLUE") return Params::BLUE;
    if (enemy_color == "GREEN") return Params::GREEN;
    if (enemy_color == "BOTH") return Params::BOTH;
    return Params::GREEN;
}

}  // namespace

// ==================== Stage1: 2D检测与分类 ====================

AutoAimPipeline::Stage1::Stage1(std::shared_ptr<YAML::Node> config_file_ptr,
                                const std::filesystem::path& workspace_path)
{
    std::string init_enemy_color;
    Params params = loadDetectorParams(config_file_ptr, &init_enemy_color);

    use_rp24_yolo = (*config_file_ptr)["use_RP24_YOLO"].as<bool>();
    light_detector = std::make_shared<LightBarDetector>(params, config_file_ptr);
    armor_detector = std::make_shared<ArmorDetector>(config_file_ptr);
    classifier = std::make_shared<ArmorClassifier>(config_file_ptr, workspace_path);
    rp24_yolo_wrapper = std::make_shared<RP24YOLOWrapper>(
        config_file_ptr,
        workspace_path / (*config_file_ptr)["RP24_YOLO_model_relative_path"].as<std::string>(),
        (*config_file_ptr)["RP24_YOLO_device"].as<std::string>());
    // 事件唤醒：YOLO 结果就绪时置位并通知 stage1 线程（替代 5ms 轮询取结果）
    rp24_yolo_wrapper->setResultNotify([this]() {
        {
            std::lock_guard<std::mutex> lock(mtx);
            result_wakeup_ = true;
        }
        cv.notify_one();
    });
}

void AutoAimPipeline::Stage1::start(AutoAimPipelineData& d)
{
    if (!idle.load()) {
        throw std::runtime_error("AutoAimPipeline::Stage1::start: stage is not idle");
    }
    {
        std::lock_guard<std::mutex> lock(mtx);
        data = &d;
        idle.store(false);
    }
    cv.notify_one();
}

bool AutoAimPipeline::Stage1::isIdle() const
{
    return idle.load();
}

void AutoAimPipeline::Stage1::run()
{
    // 流水线各阶段线程绑到 E 核（cpu_pinning.other_cores），主线程已提前绑核，这里兜底
    tools::cpu_affinity::applyOtherToCurrentThread();
    if (use_rp24_yolo) {
        runAsync();
    } else {
        runLegacy();
    }
}

void AutoAimPipeline::Stage1::runLegacy()
{
    // 原有同步路径：一次一帧，idle 交接（仅非 YOLO 模式使用）
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return !idle.load() || exit_flag; });
        if (exit_flag) return;

        AutoAimPipelineData* d = data;
        lock.unlock();

        const auto stage_start_time = PerfClock::now();
        d->stage1 = AutoAimPipelineData::Stage1Data{};
        d->stage1.used_yolo = false;
        light_detector->setEnemyColor(toEnemyColor(d->initial.enemy_color));
        light_detector->detectLights(d->initial.frame);
        light_detector->processLights();
        d->stage1.lights = light_detector->getLights();
        d->stage1.armors = armor_detector->detectArmors(d->stage1.lights);
        d->stage1.classify_results =
            classifier->classify(
                d->initial.frame,
                d->stage1.armors,
                d->initial.ground_stable_point);

        if (d->initial.performance_profile) {
            d->initial.performance_profile->stages["stage1_2d_detect_classify"] +=
                PerformanceMonitor::durationMs(stage_start_time, PerfClock::now());
        }
        idle.store(true);
    }
}

void AutoAimPipeline::Stage1::runAsync()
{
    while (true) {
        std::unique_ptr<AutoAimPipelineData> d;
        bool got_input = false;
        {
            std::unique_lock<std::mutex> lock(mtx);
            // 事件驱动等待：新输入 / YOLO 结果就绪 / 退出 任一发生即醒来。
            // 结果就绪由 wrapper 回调置位 result_wakeup_ + notify_one 通知，
            // 替代原先 5ms 超时轮询，最多可减少 5ms 帧延迟。
            cv.wait(lock, [this]() {
                return !inbox_.empty() || result_wakeup_ || exit_flag;
            });
            if (exit_flag) {
                // flushAll 内部会再锁 mtx，必须先释放锁，否则同一非递归锁
                // 二次加锁导致自死锁，退出时 worker 永远 join 不上
                lock.unlock();
                flushAll();
                return;
            }
            result_wakeup_ = false;
            if (!inbox_.empty()) {
                d = std::move(inbox_.front());
                inbox_.pop_front();
                got_input = true;
            }
        }

        if (got_input) {
            // 提交到 YOLO 异步流水线，不等待结果
            int detect_color_int = (d->initial.enemy_color == "BLUE") ? 0
                                 : ((d->initial.enemy_color == "RED") ? 1 : -1);
            InFlight pf;
            pf.submitted = std::chrono::steady_clock::now();
            void* user_data = d.get();
            pf.data = std::move(d);
            rp24_yolo_wrapper->submitFrame(pf.data->initial.frame, detect_color_int, user_data);

            {
                std::lock_guard<std::mutex> lock(mtx);
                in_flight_.push_back(std::move(pf));
            }
        }

        // 尽力取回已完成的结果（每轮都执行，即使没有新输入）
        drainResults();
    }
}

void AutoAimPipeline::Stage1::drainResults()
{
    while (true) {
        RP24YOLOWrapper::YoloResult res;
        if (!rp24_yolo_wrapper->tryTakeResult(&res)) break;
        finishFrame(res, std::chrono::steady_clock::now());
    }
}

void AutoAimPipeline::Stage1::finishFrame(RP24YOLOWrapper::YoloResult& res,
                                          std::chrono::steady_clock::time_point now)
{
    struct Completed {
        std::unique_ptr<AutoAimPipelineData> d;
        RP24YOLOWrapper::YoloResult res;
        double latency_ms = 0.0;
    };
    std::vector<Completed> completed;
    {
        std::lock_guard<std::mutex> lock(mtx);
        // 结果先入缓存（线程池任务可能乱序完成），再按提交顺序逐个消费。
        // 绝不提前 drop 在飞帧：任务的 user_data 指向帧数据，帧被销毁后任务访问会段错误。
        pending_results_[res.user_data] = std::move(res);

        while (!in_flight_.empty()) {
            auto it = pending_results_.find(in_flight_.front().data.get());
            if (it == pending_results_.end()) {
                break;  // 队首帧尚未完成，等待后续结果
            }
            Completed c;
            c.latency_ms = std::chrono::duration<double, std::milli>(
                now - in_flight_.front().submitted).count();
            c.res = std::move(it->second);
            pending_results_.erase(it);
            c.d = std::move(in_flight_.front().data);
            in_flight_.pop_front();
            completed.push_back(std::move(c));
        }
    }

    // 锁外处理：classifyAndTrack 耗时，不应持锁
    for (auto& c : completed) {
        if (c.d->initial.performance_profile) {
            // YOLO 异步路径：记录帧从提交到结果就绪的总延迟（含排队+预处理+推理+后处理）。
            // 名字用 stage1_yolo_latency，与 runLegacy 的传统视觉耗时区分开。
            c.d->initial.performance_profile->stages["stage1_yolo_latency"] += c.latency_ms;
        }
        c.d->stage1 = AutoAimPipelineData::Stage1Data{};
        c.d->stage1.used_yolo = true;
        c.d->stage1.armors = std::move(c.res.armors);
        for (const Armor& armor : c.d->stage1.armors) {
            c.d->stage1.lights.emplace_back(armor.leftLight);
            c.d->stage1.lights.emplace_back(armor.rightLight);
        }
        const auto classify_start = PerfClock::now();
        c.d->stage1.classify_results = rp24_yolo_wrapper->classifyAndTrack(
            c.d->stage1.armors, c.res.rp24_classes, c.d->initial.ground_stable_point);
        if (c.d->initial.performance_profile) {
            c.d->initial.performance_profile->stages["stage1_classify_track"] +=
                PerformanceMonitor::durationMs(classify_start, PerfClock::now());
        }
    }
    {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto& c : completed) {
            done_.push_back(std::move(c.d));
        }
    }
}

void AutoAimPipeline::Stage1::flushAll()
{
    std::lock_guard<std::mutex> lock(mtx);
    for (auto& d : inbox_) {
        done_.push_back(std::move(d));
    }
    inbox_.clear();
    for (auto& pf : in_flight_) {
        std::unique_ptr<AutoAimPipelineData> dd = std::move(pf.data);
        dd->stage1 = AutoAimPipelineData::Stage1Data{};
        dd->stage1.used_yolo = true;
        done_.push_back(std::move(dd));
    }
    in_flight_.clear();
}

void AutoAimPipeline::Stage1::pushInput(std::unique_ptr<AutoAimPipelineData> d)
{
    std::lock_guard<std::mutex> lock(mtx);
    inbox_.push_back(std::move(d));
    cv.notify_one();
}

std::unique_ptr<AutoAimPipelineData> AutoAimPipeline::Stage1::popDone()
{
    std::lock_guard<std::mutex> lock(mtx);
    if (done_.empty()) return nullptr;
    auto d = std::move(done_.front());
    done_.pop_front();
    return d;
}

size_t AutoAimPipeline::Stage1::inflightCount() const
{
    std::lock_guard<std::mutex> lock(mtx);
    return inbox_.size() + in_flight_.size();
}

// ==================== Stage2: 3D解算与坐标转换 ====================

AutoAimPipeline::Stage2::Stage2(std::shared_ptr<YAML::Node> config_file_ptr)
{
    armor_solver = std::make_shared<ArmorSolver>(config_file_ptr);
    rest_frame = std::make_shared<RestFrame>();
    rest_frame->updateCamOrientation(0, 0, 0);
    rest_frame->updateCamPosition(0, 0, 0);
    max_armor_position_height = (*config_file_ptr)["max_armor_position_height"].as<float>();
}

void AutoAimPipeline::Stage2::start(AutoAimPipelineData& d)
{
    if (!idle.load()) {
        throw std::runtime_error("AutoAimPipeline::Stage2::start: stage is not idle");
    }
    {
        std::lock_guard<std::mutex> lock(mtx);
        data = &d;
        idle.store(false);
    }
    cv.notify_one();
}

bool AutoAimPipeline::Stage2::isIdle() const
{
    return idle.load();
}

void AutoAimPipeline::Stage2::run()
{
    tools::cpu_affinity::applyOtherToCurrentThread();
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return !idle.load() || exit_flag; });
        if (exit_flag) return;

        AutoAimPipelineData* d = data;
        lock.unlock();

        const auto stage_start_time = PerfClock::now();
        d->stage2 = AutoAimPipelineData::Stage2Data{};
        rest_frame->updateCamOrientation(
            d->initial.yaw,
            d->initial.pitch,
            d->initial.roll);
        rest_frame->updateCamPosition(0, 0, 0);

        // 用引用遍历，避免每块装甲板按值拷贝一次 ArmorResult（内含多个 vector/Mat）
        for (ArmorResult& classify_result : d->stage1.classify_results) {
            AimResult solve_armor_result =
                armor_solver->solveArmor(classify_result, d->initial.pitch, d->initial.yaw);//pnp
            cv::Point3f rest_frame_pos = rest_frame->pnpToWorldP3f(solve_armor_result.position);//restframe
            if (rest_frame_pos.z < max_armor_position_height && solve_armor_result.valid) {
                classify_result.solve_armor_result = solve_armor_result;
                d->stage2.solved_results.emplace_back(std::move(classify_result));
                d->stage2.rest_frame_positions.emplace_back(rest_frame_pos);
            }
        }
        d->stage2.valid_count = d->stage2.solved_results.size();

        if (d->initial.performance_profile) {
            d->initial.performance_profile->stages["stage2_3d_solve_transform"] +=
                PerformanceMonitor::durationMs(stage_start_time, PerfClock::now());
        }
        idle.store(true);
    }
}

// ==================== Stage3: 预测与命令 ====================

AutoAimPipeline::Stage3::Stage3(std::shared_ptr<YAML::Node> config_file_ptr,
                                std::chrono::steady_clock::time_point node_start_time)
{
    armor_solver = std::make_shared<ArmorSolver>(config_file_ptr);
    ballistic_solver = std::make_shared<BallisticSolver>(config_file_ptr);
    rest_frame = std::make_shared<RestFrame>();
    rest_frame->updateCamOrientation(0, 0, 0);
    rest_frame->updateCamPosition(0, 0, 0);
    fps_counter = std::make_shared<FrameRateCounter>(30);
    predictor_main = std::make_shared<PredictorMain>(
        config_file_ptr,
        node_start_time,
        armor_solver,
        ballistic_solver,
        rest_frame,
        fps_counter);
}

void AutoAimPipeline::Stage3::start(AutoAimPipelineData& d)
{
    if (!idle.load()) {
        throw std::runtime_error("AutoAimPipeline::Stage3::start: stage is not idle");
    }
    {
        std::lock_guard<std::mutex> lock(mtx);
        data = &d;
        idle.store(false);
    }
    cv.notify_one();
}

bool AutoAimPipeline::Stage3::isIdle() const
{
    return idle.load();
}

void AutoAimPipeline::Stage3::run()
{
    tools::cpu_affinity::applyOtherToCurrentThread();
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return !idle.load() || exit_flag; });
        if (exit_flag) return;

        AutoAimPipelineData* d = data;
        lock.unlock();

        const auto stage_start_time = PerfClock::now();
        d->stage3 = AutoAimPipelineData::Stage3Data{};
        rest_frame->updateCamOrientation(
            d->initial.yaw,
            d->initial.pitch,
            d->initial.roll);
        rest_frame->updateCamPosition(0, 0, 0);
        predictor_main->update_serial_info(
            d->initial.bullet_velocity,
            d->initial.pitch,
            d->initial.yaw,
            d->initial.total_yaw);

        d->stage3.predictor_result =
            predictor_main->step(
                d->stage2.solved_results,
                d->initial.frame,
                PredictorType::AutoSwitch,
                ArmorType::Nearest,
                d->initial.auto_aim_switch,
                d->initial.mcu_yaw_online);

        d->stage3.mcu_command_pitch = d->stage3.predictor_result.command_pitch;
        d->stage3.mcu_command_yaw = d->stage3.predictor_result.command_yaw;
        if (d->initial.use_head_imu) {
            d->stage3.mcu_command_pitch = d->stage3.predictor_result.command_pitch;
            d->stage3.mcu_command_yaw =
                d->stage3.predictor_result.command_yaw + d->initial.to_mcu_delta_yaw;
        }
        d->stage3.should_send_reset = d->stage3.predictor_result.reset;

        if (d->initial.performance_profile) {
            d->initial.performance_profile->stages["stage3_predict_command"] +=
                PerformanceMonitor::durationMs(stage_start_time, PerfClock::now());
        }
        idle.store(true);
    }
}

void AutoAimPipeline::Stage3::resetYawIntegration()
{
    predictor_main->reset_yaw_integration();
}

// ==================== Stage4: 可视化输出与日志记录 ====================

AutoAimPipeline::Stage4::Stage4(std::shared_ptr<YAML::Node> config_file_ptr,
                                const std::filesystem::path& workspace_path)
{
    visualizer_config = VisualizerConfig::fromYaml(*config_file_ptr);
    log_result_video = (*config_file_ptr)["LOG_RESULT_VIDEO"].as<bool>();
    log_origin_video = (*config_file_ptr)["LOG_ORIGIN_VIDEO"].as<bool>();
    if (visualizer_config.enable && visualizer_config.log_video &&
        (log_result_video || log_origin_video)) {
        two_video_logger = std::make_shared<TwoVideoLogger>(
            (workspace_path / "VideoLog").string(),
            log_origin_video,
            log_result_video);
    }
    if (visualizer_config.enable && visualizer_config.publish_topics) {
        // 无 ROS2：publish_topics 改为写入共享内存，供独立 visualizer 进程读取
        shm_writer_ = std::make_unique<VisualizerShmWriter>(config_file_ptr);
        if (!shm_writer_->valid()) {
            tools::logger()->warn("[VisualizerShm] writer init failed, visualization publishing disabled");
            shm_writer_.reset();
        } else {
            tools::logger()->info("[VisualizerShm] writer ready (publish debug frames via shared memory)");
        }
    }
}

void AutoAimPipeline::Stage4::start(AutoAimPipelineData& d)
{
    if (!idle.load()) {
        throw std::runtime_error("AutoAimPipeline::Stage4::start: stage is not idle");
    }
    {
        std::lock_guard<std::mutex> lock(mtx);
        data = &d;
        idle.store(false);
    }
    cv.notify_one();
}

bool AutoAimPipeline::Stage4::isIdle() const
{
    return idle.load();
}

void AutoAimPipeline::Stage4::run()
{
    tools::cpu_affinity::applyOtherToCurrentThread();
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return !idle.load() || exit_flag; });
        if (exit_flag) return;

        AutoAimPipelineData* d = data;
        lock.unlock();

        const auto stage_start_time = PerfClock::now();
        d->stage4 = AutoAimPipelineData::Stage4Data{};
        d->stage4.rmm_visualize_frame =
            d->stage3.predictor_result.info_images.RMM_visualize_frame;
        d->stage4.common_debug_oscilloscope_frame =
            d->stage3.predictor_result.info_images.common_debug_oscilloscope_frame;
        d->stage4.armor_count = d->stage2.solved_results.size();

        if (visualizer_config.enable && visualizer_config.publish_topics) {
            AutoAimVisualizerDebugFrame debug_frame;
            // 浅拷贝即可：stage3 之后没有任何代码再写 initial.frame，
            // 发布/录制都只读；cv::Mat 引用计数共享，避免每帧一次全图 clone
            debug_frame.frame = d->initial.frame;
            debug_frame.node_start_time = d->initial.node_start_time;
            debug_frame.bullet_velocity = d->initial.bullet_velocity;
            debug_frame.enemy_color = d->initial.enemy_color;
            debug_frame.pitch = d->initial.pitch;
            debug_frame.yaw = d->initial.yaw;
            debug_frame.roll = d->initial.roll;
            debug_frame.ground_stable_point = d->initial.ground_stable_point;
            debug_frame.lights = d->stage1.lights;
            debug_frame.armors = d->stage1.armors;
            debug_frame.solved_results = d->stage2.solved_results;
            debug_frame.armor_type = d->stage3.predictor_result.armor_type;
            debug_frame.predictor_type = d->stage3.predictor_result.predictor_type;
            debug_frame.mcu_command_yaw = d->stage3.mcu_command_yaw;
            d->stage4.visualizer_debug_frame = std::move(debug_frame);
        }

        if (shm_writer_) {
            const AutoAimVisualizerDebugFrame& f = d->stage4.visualizer_debug_frame;
            visualizer_shm::DebugData debug_data;
            debug_data.bullet_velocity = f.bullet_velocity;
            std::strncpy(debug_data.enemy_color, f.enemy_color.c_str(),
                visualizer_shm::kMaxEnemyColorLen - 1);
            debug_data.enemy_color[visualizer_shm::kMaxEnemyColorLen - 1] = '\0';
            debug_data.pitch = f.pitch;
            debug_data.yaw = f.yaw;
            debug_data.roll = f.roll;
            debug_data.mcu_command_yaw = f.mcu_command_yaw;
            debug_data.armor_type = static_cast<uint8_t>(f.armor_type);
            debug_data.predictor_type = static_cast<uint8_t>(f.predictor_type);
            debug_data.ground_stable_point =
                visualizer_shm::Point2f{f.ground_stable_point.x, f.ground_stable_point.y};

            debug_data.light_count =
                std::min<size_t>(f.lights.size(), visualizer_shm::kMaxLights);
            for (size_t i = 0; i < debug_data.light_count; ++i) {
                cv::Point2f pts[4];
                f.lights[i].el.points(pts);
                for (int j = 0; j < 4; ++j) {
                    debug_data.lights[i].vertices[j] =
                        visualizer_shm::Point2f{pts[j].x, pts[j].y};
                }
            }

            debug_data.armor_count =
                std::min<size_t>(f.armors.size(), visualizer_shm::kMaxArmors);
            for (size_t i = 0; i < debug_data.armor_count; ++i) {
                const Armor& a = f.armors[i];
                for (int j = 0; j < 4; ++j) {
                    if (j < static_cast<int>(a.corners.size())) {
                        debug_data.armors[i].corners[j] =
                            visualizer_shm::Point2f{a.corners[j].x, a.corners[j].y};
                    }
                    if (j < static_cast<int>(a.light_bar_corners.size())) {
                        debug_data.armors[i].light_bar_corners[j] =
                            visualizer_shm::Point2f{a.light_bar_corners[j].x, a.light_bar_corners[j].y};
                    }
                }
                debug_data.armors[i].confidence = a.confidence;
            }

            debug_data.solved_count =
                std::min<size_t>(f.solved_results.size(), visualizer_shm::kMaxSolvedResults);
            for (size_t i = 0; i < debug_data.solved_count; ++i) {
                const ArmorResult& r = f.solved_results[i];
                visualizer_shm::ArmorResult& dst = debug_data.solved_results[i];
                for (int j = 0; j < 4; ++j) {
                    if (j < static_cast<int>(r.corners.size())) {
                        dst.corners[j] = visualizer_shm::Point2f{r.corners[j].x, r.corners[j].y};
                    }
                    if (j < static_cast<int>(r.armor.light_bar_corners.size())) {
                        dst.light_bar_corners[j] = visualizer_shm::Point2f{
                            r.armor.light_bar_corners[j].x, r.armor.light_bar_corners[j].y};
                    }
                }
                dst.center = visualizer_shm::Point2f{r.center.x, r.center.y};
                dst.center_predicted =
                    visualizer_shm::Point2f{r.center_predicted.x, r.center_predicted.y};
                dst.prediction_count = std::min<size_t>(
                    r.predictions.size(), visualizer_shm::kMaxPredictionsPerArmor);
                for (size_t p = 0; p < dst.prediction_count; ++p) {
                    dst.predictions[p] =
                        visualizer_shm::Point2f{r.predictions[p].x, r.predictions[p].y};
                }
                dst.number = r.number;
                dst.confidence = r.confidence;
                dst.is_tracked_now = r.is_tracked_now;
            }

            shm_writer_->publish(debug_data,
                d->initial.frame,
                d->stage4.rmm_visualize_frame,
                d->stage4.common_debug_oscilloscope_frame);
        }

        if (two_video_logger) {
            two_video_logger->updateOriginFrame(d->initial.frame);
            two_video_logger->updateDrewFrame(d->initial.frame);
            two_video_logger->updateRMMFrame(d->stage4.rmm_visualize_frame);
            two_video_logger->updateCDOFrame(d->stage4.common_debug_oscilloscope_frame);
            if (visualizer_config.draw.com_data) {
                two_video_logger->updateComFrame(d->initial.com_data_visualize_frame);
            }
            two_video_logger->writeTwoFrame();
            d->stage4.request_com_frame_refresh = true;
        }

        if (d->initial.performance_profile) {
            d->initial.performance_profile->stages["stage4_visualize_log"] +=
                PerformanceMonitor::durationMs(stage_start_time, PerfClock::now());
        }
        idle.store(true);
    }
}

// ==================== 流水线调度与公共API ====================

AutoAimPipeline::AutoAimPipeline(std::shared_ptr<YAML::Node> config_file_ptr,
                                 const std::filesystem::path& workspace_path,
                                 std::chrono::steady_clock::time_point node_start_time,
                                 std::shared_ptr<PerformanceMonitor> performance_monitor,
                                 int max_queue_size,
                                 float max_delay_seconds)
    : max_queue_size_(max_queue_size)
    , max_delay_seconds_(max_delay_seconds)
    , performance_monitor_(std::move(performance_monitor))
    , stage1_(config_file_ptr, workspace_path)
    , stage2_(config_file_ptr)
    , stage3_(config_file_ptr, node_start_time)
    , stage4_(config_file_ptr, workspace_path)
{
    for (auto& queue_size : queue_sizes_) {
        queue_size.store(0);
    }

    stage1_.worker = std::thread(&Stage1::run, &stage1_);
    stage2_.worker = std::thread(&Stage2::run, &stage2_);
    stage3_.worker = std::thread(&Stage3::run, &stage3_);
    stage4_.worker = std::thread(&Stage4::run, &stage4_);
    scheduler_thread_ = std::thread(&AutoAimPipeline::schedulerLoop, this);
}

AutoAimPipeline::~AutoAimPipeline()
{
    scheduler_exit_.store(true);
    if (scheduler_thread_.joinable()) scheduler_thread_.join();

    // 先停止 YOLO 异步流水线，否则 Stage1 阻塞在取结果上会让 join 挂死
    if (stage1_.rp24_yolo_wrapper) {
        stage1_.rp24_yolo_wrapper->stop();
    }

    {
        std::lock_guard<std::mutex> lock(stage1_.mtx);
        stage1_.exit_flag = true;
    }
    {
        std::lock_guard<std::mutex> lock(stage2_.mtx);
        stage2_.exit_flag = true;
    }
    {
        std::lock_guard<std::mutex> lock(stage3_.mtx);
        stage3_.exit_flag = true;
    }
    {
        std::lock_guard<std::mutex> lock(stage4_.mtx);
        stage4_.exit_flag = true;
    }
    stage1_.cv.notify_one();
    stage2_.cv.notify_one();
    stage3_.cv.notify_one();
    stage4_.cv.notify_one();

    if (stage1_.worker.joinable()) stage1_.worker.join();
    if (stage2_.worker.joinable()) stage2_.worker.join();
    if (stage3_.worker.joinable()) stage3_.worker.join();
    if (stage4_.worker.joinable()) stage4_.worker.join();
}

void AutoAimPipeline::addFrame(AutoAimPipelineData::InitialData initial)
{
    auto data = std::make_unique<AutoAimPipelineData>();
    data->initial = std::move(initial);
    if (data->initial.performance_start_time == std::chrono::steady_clock::time_point{}) {
        data->initial.performance_start_time = PerfClock::now();
    }
    if (performance_monitor_ && performance_monitor_->enabled()) {
        data->initial.performance_profile = std::make_shared<FrameProfile>(
            performance_monitor_->beginFrame(
                performance_frame_id_++,
                data->initial.performance_start_time));
    }

    std::lock_guard<std::mutex> lock(input_mtx_);
    if (scheduler_exit_.load()) return;
    if (input_queue_.size() >= static_cast<size_t>(max_queue_size_)) {
        input_queue_.pop_front();
    }
    input_queue_.push_back(std::move(data));
}

AutoAimPipeline::ProcessResult
AutoAimPipeline::tryPopResult(const std::chrono::steady_clock::time_point& timestamp)
{
    ProcessResult result;
    result.always_valid_data.queue_input = queue_sizes_[0].load();
    result.always_valid_data.queue_inter0 = queue_sizes_[1].load();
    result.always_valid_data.queue_inter1 = queue_sizes_[2].load();
    result.always_valid_data.queue_inter2 = queue_sizes_[3].load();
    result.always_valid_data.queue_output = queue_sizes_[4].load();

    std::lock_guard<std::mutex> lock(output_mtx_);
    if (output_queue_.empty()) return result;

    auto& front = output_queue_.front();
    float diff = std::chrono::duration<float>(
        timestamp - front->initial.frame_timestamp).count();
    if (diff >= max_delay_seconds_) {
        result.valid_data.predictor_result = front->stage3.predictor_result;
        result.valid_data.mcu_command_pitch = front->stage3.mcu_command_pitch;
        result.valid_data.mcu_command_yaw = front->stage3.mcu_command_yaw;
        result.valid_data.should_send_reset = front->stage3.should_send_reset;
        result.valid_data.display = std::move(front->stage4.display);
        result.valid_data.yaw_visualizer_frame = std::move(front->stage4.yaw_visualizer_frame);
        result.valid_data.rmm_visualize_frame = std::move(front->stage4.rmm_visualize_frame);
        result.valid_data.common_debug_oscilloscope_frame =
            std::move(front->stage4.common_debug_oscilloscope_frame);
        result.valid_data.visualizer_debug_frame = std::move(front->stage4.visualizer_debug_frame);
        result.valid_data.armor_count = front->stage4.armor_count;
        result.valid_data.request_com_frame_refresh = front->stage4.request_com_frame_refresh;
        result.valid = true;
        output_queue_.pop_front();
    }

    return result;
}

void AutoAimPipeline::resetYawIntegration()
{
    stage3_.resetYawIntegration();
}

void AutoAimPipeline::schedulerLoop()
{
    tools::cpu_affinity::applyOtherToCurrentThread();
    while (!scheduler_exit_.load()) {
        bool any_work = false;

        if (stage1_.use_rp24_yolo) {
            // 异步路径：先推进已完成 YOLO 的帧到 stage2，再喂新帧（限制在飞数量）
            while (auto done = stage1_.popDone()) {
                inter_queues_[0].push_back(std::move(done));
            }
            while (inter_queues_[0].size() < static_cast<size_t>(max_queue_size_)
                && stage1_.inflightCount() < static_cast<size_t>(max_queue_size_)) {
                std::unique_lock<std::mutex> lk(input_mtx_);
                if (input_queue_.empty()) break;
                auto d = std::move(input_queue_.front());
                input_queue_.pop_front();
                lk.unlock();
                input_cv_.notify_one();
                stage1_.pushInput(std::move(d));
                any_work = true;
            }
        } else if (stage1_.isIdle()) {
            // 原同步路径：一次一帧
            if (in_flight_[0]) {
                inter_queues_[0].push_back(std::move(in_flight_[0]));
            }
            if (inter_queues_[0].size() < static_cast<size_t>(max_queue_size_)) {
                std::unique_lock<std::mutex> lk(input_mtx_);
                if (!input_queue_.empty()) {
                    in_flight_[0] = std::move(input_queue_.front());
                    input_queue_.pop_front();
                    lk.unlock();
                    input_cv_.notify_one();
                    stage1_.start(*in_flight_[0]);
                    any_work = true;
                }
            }
        }

        if (stage2_.isIdle()) {
            if (in_flight_[1]) {
                inter_queues_[1].push_back(std::move(in_flight_[1]));
            }
            if (inter_queues_[1].size() < static_cast<size_t>(max_queue_size_)
                && !inter_queues_[0].empty()) {
                in_flight_[1] = std::move(inter_queues_[0].front());
                inter_queues_[0].pop_front();
                stage2_.start(*in_flight_[1]);
                any_work = true;
            }
        }

        if (stage3_.isIdle()) {
            if (in_flight_[2]) {
                inter_queues_[2].push_back(std::move(in_flight_[2]));
            }
            if (inter_queues_[2].size() < static_cast<size_t>(max_queue_size_)
                && !inter_queues_[1].empty()) {
                in_flight_[2] = std::move(inter_queues_[1].front());
                inter_queues_[1].pop_front();
                stage3_.start(*in_flight_[2]);
                any_work = true;
            }
        }

        if (stage4_.isIdle()) {
            if (in_flight_[3]) {
                if (performance_monitor_ && in_flight_[3]->initial.performance_profile) {
                    performance_monitor_->endFrame(*in_flight_[3]->initial.performance_profile);
                }
                std::lock_guard<std::mutex> lk(output_mtx_);
                output_queue_.push_back(std::move(in_flight_[3]));
            }
            {
                std::lock_guard<std::mutex> lk(output_mtx_);
                if (output_queue_.size() < static_cast<size_t>(max_queue_size_)
                    && !inter_queues_[2].empty()) {
                    in_flight_[3] = std::move(inter_queues_[2].front());
                    inter_queues_[2].pop_front();
                    stage4_.start(*in_flight_[3]);
                    any_work = true;
                }
            }
        }

        updateQueueSizes();
        if (!any_work) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    input_cv_.notify_all();
}

void AutoAimPipeline::updateQueueSizes()
{
    {
        std::lock_guard<std::mutex> lk(input_mtx_);
        queue_sizes_[0].store(static_cast<int>(input_queue_.size()));
    }
    queue_sizes_[1].store(static_cast<int>(inter_queues_[0].size()));
    queue_sizes_[2].store(static_cast<int>(inter_queues_[1].size()));
    queue_sizes_[3].store(static_cast<int>(inter_queues_[2].size()));
    {
        std::lock_guard<std::mutex> lk(output_mtx_);
        queue_sizes_[4].store(static_cast<int>(output_queue_.size()));
    }
}
