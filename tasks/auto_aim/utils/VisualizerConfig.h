#ifndef VISUALIZER_CONFIG_H
#define VISUALIZER_CONFIG_H

#include <yaml-cpp/yaml.h>

struct VisualizerConfig {
    struct DrawConfig {
        bool main_result = true;
        bool status_text = true;
        bool rest_frame = true;
        bool ground_stable_point = true;
        bool aim_reference_point = true;
        bool lights = true;
        bool armors = true;
        bool solved_armors = true;
        bool predictions = true;
        bool yaw_curve = true;
        bool rmm = true;
        bool common_debug_oscilloscope = true;
        bool gimbal_coordinate = true;
        bool com_data = true;
    };

    bool enable = true;
    bool show_windows = true;
    bool publish_topics = true;
    bool log_video = true;
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
        config.publish_topics = readBool(visualizer, "publish_topics", config.publish_topics);
        config.log_video = readBool(visualizer, "log_video", config.log_video);

        const YAML::Node draw = visualizer["draw"];
        if (draw) {
            config.draw.main_result = readBool(draw, "main_result", config.draw.main_result);
            config.draw.status_text = readBool(draw, "status_text", config.draw.status_text);
            config.draw.rest_frame = readBool(draw, "rest_frame", config.draw.rest_frame);
            config.draw.ground_stable_point = readBool(draw, "ground_stable_point", config.draw.ground_stable_point);
            config.draw.aim_reference_point = readBool(draw, "aim_reference_point", config.draw.aim_reference_point);
            config.draw.lights = readBool(draw, "lights", config.draw.lights);
            config.draw.armors = readBool(draw, "armors", config.draw.armors);
            config.draw.solved_armors = readBool(draw, "solved_armors", config.draw.solved_armors);
            config.draw.predictions = readBool(draw, "predictions", config.draw.predictions);
            config.draw.yaw_curve = readBool(draw, "yaw_curve", config.draw.yaw_curve);
            config.draw.rmm = readBool(draw, "rmm", config.draw.rmm);
            config.draw.common_debug_oscilloscope =
                readBool(draw, "common_debug_oscilloscope", config.draw.common_debug_oscilloscope);
            config.draw.gimbal_coordinate = readBool(draw, "gimbal_coordinate", config.draw.gimbal_coordinate);
            config.draw.com_data = readBool(draw, "com_data", config.draw.com_data);
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

#endif // VISUALIZER_CONFIG_H
