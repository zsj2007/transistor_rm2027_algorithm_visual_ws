// DataVisualizer.cpp
#include "visualizer/DataVisualizer.h"

// 构造函数
Oscilloscope::Oscilloscope(int w, int h, 
                           const std::string& name,
                           size_t init_layer_num,
                           cv::Scalar bg_color,
                           cv::Scalar wf_color,
                           bool show_windows)
    : width(w), height(h), scale(1.0f), offset(0.0f), 
      window_name(name), background_color(bg_color),
      layer_num(init_layer_num), // 修正：初始化layer_num为init_layer_num
      show_windows_(show_windows)
{
    // 初始化显示图像
    data_displays.resize(layer_num);
    datas.resize(layer_num);
    waveform_colors.resize(layer_num);  // 初始化waveform_colors的大小

    for (size_t layer_index = 0; layer_index < layer_num; layer_index += 1) {
        cv::Mat& data_display = data_displays[layer_index];
        data_display = cv::Mat::zeros(height, width, CV_8UC3);
        data_display.setTo(cv::Scalar(0, 0, 0));  // 每个图层的背景为黑色

        waveform_colors[layer_index] = wf_color;  // 初始化每个图层的颜色为传入的wf_color
    }

    // 初始化display为背景色
    display = cv::Mat::zeros(height, width, CV_8UC3);
    display.setTo(background_color);
}

// 添加数据点
void Oscilloscope::addDataPoint(float value, size_t layer_index, int point_size) {
    std::deque<data_point_t>& data = datas[layer_index];
    data.emplace_back(value, point_size);
    
    // 如果数据点超过窗口宽度，删除最旧的数据
    if (data.size() > width) {
        data.pop_front();
    }
}


// 重写 update 函数，完全基于 datas 中的原始数据点重新绘制每个图层
void Oscilloscope::update() {
    display.setTo(background_color);

    // 2. 重新绘制所有图层
    for (size_t layer_index = 0; layer_index < layer_num; ++layer_index) {
        cv::Mat& data_display = data_displays[layer_index];
        const std::deque<data_point_t>& data = datas[layer_index];
        const cv::Scalar& color = waveform_colors[layer_index];

        data_display.setTo(cv::Scalar(0, 0, 0));  // 清空当前图层

        size_t data_size = data.size();
        if (data_size == 0) continue;

        // 从右向左绘制：最新点位于 x = width-1，最旧点位于 x = width - data_size
        int start_x = width - static_cast<int>(data_size) * rolling_speed;

        for (size_t i = 0; i < data_size; ++i) {
            const data_point_t& point = data[i];
            int x = start_x + static_cast<int>(i) * rolling_speed;

            // 只绘制窗口内的点
            if (x >= 0 && x < width) {
                // 计算 y 坐标，应用缩放和偏移
                float normalized = (point.value * scale + offset + 1.0f) / 2.0f;
                normalized = std::max(0.0f, std::min(1.0f, normalized));
                int y = static_cast<int>((1.0f - normalized) * (height - 1));

                cv::circle(data_display, cv::Point(x, y), point.point_size, color, -1);

                // 与前一个点连线
                if (i > 0) {
                    int prev_x = start_x + static_cast<int>(i - 1) * rolling_speed;
                    if (prev_x >= 0 && prev_x < width) {
                        const data_point_t& prev_point = data[i - 1];
                        float prev_normalized = (prev_point.value * scale + offset + 1.0f) / 2.0f;
                        prev_normalized = std::max(0.0f, std::min(1.0f, prev_normalized));
                        int prev_y = static_cast<int>((1.0f - prev_normalized) * (height - 1));
                        cv::line(data_display, cv::Point(prev_x, prev_y), cv::Point(x, y), color, 1);
                    }
                }
            }
        }

        cv::add(display, data_display, display);
    }
}

// 显示窗口
void Oscilloscope::show() {
    if (!show_windows_) {
        return;
    }
    cv::imshow(window_name, display);
    cv::waitKey(1);
}

// 修改 setScale，更新缩放因子后立即重绘
void Oscilloscope::setScale(float s) {
    scale = s;
    update();  // 重绘所有图层，应用新的缩放因子
}

// 修改 setOffset，更新偏移量后立即重绘
void Oscilloscope::setOffset(float o) {
    offset = o;
    update();  // 重绘所有图层，应用新的偏移量
}

// 清除所有数据
void Oscilloscope::clear_all() {
    for (size_t layer_index = 0; layer_index < layer_num; layer_index += 1) {
        cv::Mat& data_display = data_displays[layer_index];
        std::deque<data_point_t>& data = datas[layer_index];

        data.clear();
        data_display.setTo(cv::Scalar(0, 0, 0));
    }
}

void Oscilloscope::putText(
    const std::string& text,
    cv::Point org,
    cv::Scalar color,
    double fontScale,
    int thickness,
    int fontFace,
    int lineType,
    bool bottomLeftOrigin
) {
    cv::putText(display, text, org, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin);
}

cv::Mat Oscilloscope::getDisplay() {
    return display;
}

void Oscilloscope::setLayerColor(size_t layer_index, cv::Scalar color) {
    waveform_colors[layer_index] = color;
}

void Oscilloscope::setRollingSpeed(uint32_t speed) {
    rolling_speed = speed;
}
