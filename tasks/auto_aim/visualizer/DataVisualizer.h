// DataVisualizer.h
#ifndef DATA_VISUALIZER_H
#define DATA_VISUALIZER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include <algorithm>

class Oscilloscope {
private:
    struct data_point_t {
        float value;
        int point_size;

        data_point_t(float value_, int point_size_) : value(value_), point_size(point_size_) {}
    };
    std::vector<cv::Mat> data_displays;          // 数据图像
    std::vector<std::deque<data_point_t>> datas;   // 存储数据点的双端队列
    std::vector<cv::Scalar> waveform_colors;   // 波形颜色
    cv::Mat display;          // 显示图像
    int width;                // 显示窗口宽度
    int height;               // 显示窗口高度
    float scale;              // 垂直缩放因子
    float offset;             // 垂直偏移
    std::string window_name;  // 窗口名称
    cv::Scalar background_color; // 背景颜色
    size_t layer_num;
    uint32_t rolling_speed = 1;
    bool show_windows_ = false;

public:
    // 构造函数
    Oscilloscope(int w = 800, int h = 400, 
                 const std::string& name = "Oscilloscope",
                 size_t init_layer_num = 1,
                 cv::Scalar bg_color = cv::Scalar(0, 0, 0),
                 cv::Scalar wf_color = cv::Scalar(0, 255, 0),
                 bool show_windows = false);

    // 添加数据点
    void addDataPoint(float value, size_t layer_index = 0, int point_size = 1);
    
    // 更新显示
    void update();
    
    // 显示窗口
    void show();
    
    // 设置垂直缩放
    void setScale(float s);
    
    // 设置垂直偏移
    void setOffset(float o);
    
    // 清除所有数据
    void clear_all();

    // 绘制文字
    void putText(
        const std::string& text,
        cv::Point org,
        cv::Scalar color,
        double fontScale = 1.0,
        int thickness = 1,
        int fontFace = cv::FONT_HERSHEY_COMPLEX,
        int lineType = 8,
        bool bottomLeftOrigin = false
    );

    cv::Mat getDisplay();

    void setLayerColor(size_t layer_index, cv::Scalar color);

    void setRollingSpeed(uint32_t speed);

};


#endif // DATA_VISUALIZER_H