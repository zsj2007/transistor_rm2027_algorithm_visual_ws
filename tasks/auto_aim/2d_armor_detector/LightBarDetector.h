// LightBarDetector.h
#ifndef LIGHTBARDETECTOR_H
#define LIGHTBARDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "Params.h"
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "LightBar.h"
#include <algorithm>
#include <thread>


/**
 * @brief 灯条检测器类，负责检测和处理图像中的所有灯条
 */
class LightBarDetector {
public:
    /**
     * @brief 构造函数
     * @param params 检测参数结构体，包含各种阈值和配置
     */
    explicit LightBarDetector(const Params& params, std::shared_ptr<YAML::Node> config_file_ptr);
    
    /**
     * @brief 设置敌方装甲板的颜色
     * @param color 颜色枚举值（红色或蓝色）
     */
    void setEnemyColor(int color);

    /**
     * @brief 在输入图像中检测灯条
     * @param images 输入图像
     */
    void detectLights(cv::Mat& images);

    /**
     * @brief 处理检测到的灯条（过滤和更新）
     */
    void processLights();

    /**
     * @brief 获取当前检测到的所有灯条
     * @return 灯条vector的常量引用
     */
    const std::vector<Light>& getLights() const { return lights; }

private:
    std::vector<Light> lights;           // 存储检测到的所有灯条
    Params params;                       // 检测参数
    Params::EnemyColor enemy_color;      // 敌方颜色
    std::shared_ptr<YAML::Node> config_file_ptr; // 配置文件
    bool show_windows_ = false;
    // 灯条检测参数
    float mean_color_diff_THRESHOLD_RED;
    float mean_color_diff_THRESHOLD_BLUE;
    float color_rect_expand_FACTOR;
    uint8_t binary_img_THRESHOLD;
    uint8_t subtract_value;
    // 颜色通道差值的阈值常量
    int THRES_MAX_COLOR_RED;   // 红色通道差值阈值
    int THRES_MAX_COLOR_BLUE;  // 蓝色通道差值阈值
    float light_bar_max_angle;

    /**
     * @brief 根据敌方颜色提取颜色通道差值图像
     * @param img 输入图像
     * @return 处理后的灰度图像
     */
    cv::Mat extractColorChannelDiff(const cv::Mat& img);

    /**
     * @brief 转换为灰度图后二值化图像
     * @param img 输入图像
     * @return 处理后的二值图像
     */
    cv::Mat binaryImg(const cv::Mat& img);

    /**
     * @brief 计算灰度图在矩形范围内的均值
     * @param grayImage 输入图像
     * @param rect 输入矩形
     * @return 均值结果
     */
    float calculateMeanInRotatedRect(const cv::Mat& grayImage, const cv::RotatedRect& rect);

    /**
     * @brief 按比例扩张矩形
     * @param rect 输入矩形
     * @param factor 比例系数
     * @return 扩张后矩形
     */
    cv::RotatedRect rectExpand(const cv::RotatedRect& rect, float factor);

    /**
     * @brief 在二值图像中检测可能的灯条
     * @param img 输入的二值图像
     * @return 检测到的旋转矩形vector
     */
    std::vector<cv::RotatedRect> detectLightRects(const cv::Mat& img);

    /**
     * @brief 过滤不符合条件的灯条
     */
    void filterLights();

    /**
     * @brief 更新灯条状态（用于追踪）
     */
    void updateLights();

    /**
     * @brief 计算两个灯条是否重叠
     * @param light1 灯条1
     * @param light2 灯条2
     * @return 如果两个灯条重叠，返回true
     */
    bool isOverlap(const Light& light1, const Light& light2);

    float calculateAccurateAngleByOLS(const cv::Mat& gray_img, const cv::RotatedRect& rotatedRect);
};

#endif // LIGHTBARDETECTOR_H
