// LightBarDetector.cpp

#include "2d_armor_detector/LightBarDetector.h"
#include "utils/ThreadPool.h"

#include "tools/logger.hpp"

/************************* Light类实现 *************************/

// 默认构造函数
Light::Light() {};

Light::Light(const cv::RotatedRect& rect) 
    : el(rect), length(0), width(0), angle(0) {
    calculateDimensions();  // 构造时计算所有几何参数
}

void Light::calculateDimensions() {
    // 1. 计算角度并标准化到[-90, 90]
    angle = el.angle;

    // 2. 计算长宽（确保length始终为较长边）
    length = el.size.height;
    width = el.size.width;
}

/* void Light::correctLength(const cv::Mat& binary_img) {
    float sum_value_target = computeRotatedRectSum(el, binary_img) * 0.99;
    // 二分查找使旋转矩形内二值图像总值下降为原来0.99倍的长度
    int binarySearchFrequency = 10; // 二分查找次数
    float upper_ratio = 1.0;
    float lower_ratio = 0.0;
    float try_ratio;
    float length_original = length;
    for (int i = 0; i < binarySearchFrequency; i++) {
        try_ratio = (upper_ratio + lower_ratio) * 0.5;
        el.size.height = length_original * try_ratio;
        float sum_value = computeRotatedRectSum(el, binary_img);
        if (sum_value > sum_value_target) {
            upper_ratio = try_ratio;
        } else {
            lower_ratio = try_ratio;
        }
    }
    try_ratio = (upper_ratio + lower_ratio) * 0.5;
    length = length_original * try_ratio;
    el.size.height = length;
} */

// 高效计算旋转矩形内白色像素面积的辅助函数
float Light::computeRotatedRectSum(const cv::RotatedRect& rect, const cv::Mat& gray_img) {
    // 获取旋转矩形的四个顶点
    cv::Point2f vertices[4];
    rect.points(vertices);
    
    // 计算最小外接矩形（ROI）
    cv::Rect boundRect = cv::boundingRect(std::vector<cv::Point2f>(vertices, vertices + 4));
    boundRect &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);  // 确保在图像范围内
    
    // 如果ROI无效，返回0
    if (boundRect.width <= 0 || boundRect.height <= 0) {
        return 0;
    }
    
    // 超采样比例，可以提高掩码的精度
    const int scale_factor = 4;  // 4x超采样
    
    // 创建超采样掩码
    cv::Mat high_res_mask = cv::Mat::zeros(boundRect.height * scale_factor, 
                                           boundRect.width * scale_factor, 
                                           CV_32FC1);
    
    // 将顶点坐标转换为超采样ROI局部坐标
    std::vector<cv::Point2f> scaled_polyPoints;
    for (int i = 0; i < 4; i++) {
        scaled_polyPoints.push_back(cv::Point2f(
            (vertices[i].x - boundRect.x) * scale_factor,
            (vertices[i].y - boundRect.y) * scale_factor
        ));
    }
    
    // 在超采样掩码上填充旋转矩形，值为1
    cv::fillConvexPoly(high_res_mask, 
                      std::vector<cv::Point>(scaled_polyPoints.begin(), scaled_polyPoints.end()), 
                      cv::Scalar(1.0));
    
    // 将超采样掩码下采样回原始分辨率，得到比例值
    cv::Mat proportion_mask;
    cv::resize(high_res_mask, proportion_mask, 
               cv::Size(boundRect.width, boundRect.height), 
               0, 0, cv::INTER_AREA);
    
    // 提取ROI区域并转换为浮点型以便乘法
    cv::Mat roi = gray_img(boundRect);
    cv::Mat roi_float;
    roi.convertTo(roi_float, CV_32FC1, 1.0/255.0);  // 归一化到[0,1]
    
    // 计算加权和：掩码比例值 * 二值图像值
    cv::Mat weighted_roi;
    cv::multiply(roi_float, proportion_mask, weighted_roi);
    
    // 计算总和
    float sum_value = cv::sum(weighted_roi)[0];
    
    return sum_value;
}

void Light::correctLengthAndWidth(const cv::Mat& gray_img) {
    // 原始面积
    cv::RotatedRect temp_el = el;
    temp_el.size.height *= 1.2;
    temp_el.size.width *= 2.0;

    float original_sum = computeRotatedRectSum(temp_el, gray_img);
    float sum_value_target = original_sum * 0.95;
    
    // 保存原始参数
    float length_original = temp_el.size.height;
    float width_original = temp_el.size.width;
    cv::Point2f center_original = temp_el.center;
    float angle_normal = temp_el.angle + 90.0;
    
    // 二分查找参数
    int binarySearchFrequency = 10;
    
    // 1. 分别优化长度方向的两端
    float front_ratio = 1.0f;  // 前端收缩比例
    float back_ratio = 1.0f;   // 后端收缩比例
    
    // 优化前端（沿着长度方向的正面）
    float upper_front = 1.0f;
    float lower_front = 0.5f;
    for (int i = 0; i < binarySearchFrequency; i++) {
        front_ratio = (upper_front + lower_front) * 0.5f;
        
        // 只收缩前端，保持后端不变
        cv::RotatedRect test_rect = temp_el;
        float new_length = length_original * front_ratio;
        
        // 计算新的中心点（前端收缩，中心点向后移动）
        double angle_rad = angle_normal * CV_PI / 180.0;
        cv::Point2f direction(std::cos(angle_rad), std::sin(angle_rad));
        cv::Point2f length_offset = direction * (length_original - new_length) * 0.5f;
        
        test_rect.center = center_original - length_offset;
        test_rect.size.height = new_length;
        
        float sum_value = computeRotatedRectSum(test_rect, gray_img);
        
        if (sum_value > sum_value_target) {
            upper_front = front_ratio;
        } else {
            lower_front = front_ratio;
        }
    }
    front_ratio = (upper_front + lower_front) * 0.5f;
    
    // 优化后端（沿着长度方向的背面）
    float upper_back = 1.0f;
    float lower_back = 0.5f;
    for (int i = 0; i < binarySearchFrequency; i++) {
        back_ratio = (upper_back + lower_back) * 0.5f;
        
        cv::RotatedRect test_rect = temp_el;
        float new_length = length_original * back_ratio;
        
        // 计算新的中心点（后端收缩，中心点向前移动）
        double angle_rad = angle_normal * CV_PI / 180.0;
        cv::Point2f direction(std::cos(angle_rad), std::sin(angle_rad));
        cv::Point2f length_offset = direction * (length_original - new_length) * 0.5f;
        
        test_rect.center = center_original + length_offset;
        test_rect.size.height = new_length;
        
        float sum_value = computeRotatedRectSum(test_rect, gray_img);
        
        if (sum_value > sum_value_target) {
            upper_back = back_ratio;
        } else {
            lower_back = back_ratio;
        }
    }
    back_ratio = (upper_back + lower_back) * 0.5f;
    
    // 2. 分别优化宽度方向的两侧
    float left_ratio = 1.0f;   // 左侧收缩比例
    float right_ratio = 1.0f;  // 右侧收缩比例
    
    // 优化左侧（垂直于长度方向的左侧）
    float upper_left = 1.0f;
    float lower_left = 0.0f;
    for (int i = 0; i < binarySearchFrequency; i++) {
        left_ratio = (upper_left + lower_left) * 0.5f;
        
        cv::RotatedRect test_rect = temp_el;
        float new_width = width_original * left_ratio;
        
        // 计算新的中心点（左侧收缩，中心点向右移动）
        double angle_rad = (angle_normal + 90) * CV_PI / 180.0;  // 垂直方向
        cv::Point2f direction(std::cos(angle_rad), std::sin(angle_rad));
        cv::Point2f width_offset = direction * (width_original - new_width) * 0.5f;
        
        test_rect.center = center_original - width_offset;
        test_rect.size.width = new_width;
        
        float sum_value = computeRotatedRectSum(test_rect, gray_img);
        
        if (sum_value > sum_value_target) {
            upper_left = left_ratio;
        } else {
            lower_left = left_ratio;
        }
    }
    left_ratio = (upper_left + lower_left) * 0.5f;
    
    // 优化右侧（垂直于长度方向的右侧）
    float upper_right = 1.0f;
    float lower_right = 0.0f;
    for (int i = 0; i < binarySearchFrequency; i++) {
        right_ratio = (upper_right + lower_right) * 0.5f;
        
        cv::RotatedRect test_rect = temp_el;
        float new_width = width_original * right_ratio;
        
        // 计算新的中心点（右侧收缩，中心点向左移动）
        double angle_rad = (angle_normal + 90) * CV_PI / 180.0;  // 垂直方向
        cv::Point2f direction(std::cos(angle_rad), std::sin(angle_rad));
        cv::Point2f width_offset = direction * (width_original - new_width) * 0.5f;
        
        test_rect.center = center_original + width_offset;
        test_rect.size.width = new_width;
        
        float sum_value = computeRotatedRectSum(test_rect, gray_img);
        
        if (sum_value > sum_value_target) {
            upper_right = right_ratio;
        } else {
            lower_right = right_ratio;
        }
    }
    right_ratio = (upper_right + lower_right) * 0.5f;
    
    // 3. 应用最终的优化结果
    // 计算平均收缩比例并应用
    float final_length_ratio = (front_ratio + back_ratio - 1.0);
    float final_width_ratio = (left_ratio + right_ratio - 1.0);
    
    length = length_original * final_length_ratio;
    width = width_original * final_width_ratio;
    
    // 计算最终的中心点偏移（综合考虑两端收缩）
    double angle_rad = angle_normal * CV_PI / 180.0;
    cv::Point2f length_direction(std::cos(angle_rad), std::sin(angle_rad));
    cv::Point2f width_direction(std::cos(angle_rad + CV_PI/2), std::sin(angle_rad + CV_PI/2));
    
    cv::Point2f length_offset = length_direction * (length_original * (back_ratio - front_ratio) * 0.5f);
    cv::Point2f width_offset = width_direction * (width_original * (right_ratio - left_ratio) * 0.5f);
    
    el.center = center_original - length_offset - width_offset;
    el.size.height = length;
    el.size.width = width;
}

/************************* LightBarDetector类实现 *************************/

LightBarDetector::LightBarDetector(const Params& params, std::shared_ptr<YAML::Node> config_file_ptr)
    : params(params), enemy_color(params.enemy_color), config_file_ptr(config_file_ptr) {
        mean_color_diff_THRESHOLD_BLUE = (*config_file_ptr)["mean_color_diff_THRESHOLD_BLUE"].as<float>();
        mean_color_diff_THRESHOLD_RED = (*config_file_ptr)["mean_color_diff_THRESHOLD_RED"].as<float>();
        color_rect_expand_FACTOR = (*config_file_ptr)["color_rect_expand_FACTOR"].as<float>(); 
        binary_img_THRESHOLD = (*config_file_ptr)["binary_img_THRESHOLD"].as<uint8_t>(); 
        subtract_value = (*config_file_ptr)["subtract_value"].as<uint8_t>(); 
        THRES_MAX_COLOR_RED = (*config_file_ptr)["THRES_MAX_COLOR_RED"].as<int>(); 
        THRES_MAX_COLOR_BLUE = (*config_file_ptr)["THRES_MAX_COLOR_BLUE"].as<int>(); 
        light_bar_max_angle = (*config_file_ptr)["light_bar_max_angle"].as<float>();
        show_windows_ = (*config_file_ptr)["SHOW_WINDOWS"] ? (*config_file_ptr)["SHOW_WINDOWS"].as<bool>() : false;
    }

void LightBarDetector::setEnemyColor(int color) {
    enemy_color = static_cast<Params::EnemyColor>(color);
    params.enemy_color = enemy_color;
}

struct alignas(64) LightDetectThreadInfo { // 64字节对齐
    cv::RotatedRect* lightRect;
    bool is_true_light = false;
    Light light;
};

float LightBarDetector::calculateAccurateAngleByOLS(const cv::Mat& gray_img, const cv::RotatedRect& rotatedRect) {
    // 1. 获取旋转矩形的边界框
    cv::Rect boundingRect = rotatedRect.boundingRect();
    boundingRect = boundingRect & cv::Rect(0, 0, gray_img.cols, gray_img.rows);
    if (boundingRect.area() == 0) return rotatedRect.angle;
    
    // 2. 截取ROI区域
    cv::Mat roi = gray_img(boundingRect);
    
    // 3. 创建旋转矩形的掩码
    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::Point2f centerInRoi(rotatedRect.center.x - boundingRect.x, 
                           rotatedRect.center.y - boundingRect.y);
    cv::RotatedRect rectInRoi(centerInRoi, rotatedRect.size, rotatedRect.angle);
    
    cv::Point2f vertices[4];
    rectInRoi.points(vertices);
    std::vector<cv::Point> poly;
    for (int i = 0; i < 4; ++i) poly.push_back(vertices[i]);
    cv::fillConvexPoly(mask, poly, cv::Scalar(255));
    
    // 4. 收集加权像素点
    std::vector<double> xs, ys, ws;
    for (int y = 0; y < roi.rows; ++y) {
        const uchar* imgRow = roi.ptr<uchar>(y);
        const uchar* maskRow = mask.ptr<uchar>(y);
        for (int x = 0; x < roi.cols; ++x) {
            if (maskRow[x] > 0 && imgRow[x] > 0) {
                xs.push_back(x);
                ys.push_back(y);
                ws.push_back(imgRow[x]); // 亮度作为权重
            }
        }
    }
    
    if (xs.size() < 2) return rotatedRect.angle;
    
    // 5. 加权最小二乘法
    double sumW = 0, sumX = 0, sumY = 0, sumXX = 0, sumXY = 0, sumYY = 0;
    
    for (size_t i = 0; i < xs.size(); ++i) {
        double w = ws[i];
        double x = xs[i];
        double y = ys[i];
        
        sumW += w;
        sumX += w * x;
        sumY += w * y;
        sumXX += w * x * x;
        sumXY += w * x * y;
        sumYY += w * y * y;
    }
    
    double meanX = sumX / sumW;
    double meanY = sumY / sumW;
    
    double Sxx = sumXX / sumW - meanX * meanX;
    double Sxy = sumXY / sumW - meanX * meanY;
    double Syy = sumYY / sumW - meanY * meanY;
    
    // 6. 计算主方向角度（使用数值更稳定的方法）
    double angle = 0.0;
    
    // 检查数值稳定性
    if (std::abs(Sxx) > std::abs(Syy)) {
        // 以x为自变量更稳定
        if (std::abs(Sxx) > 1e-6) {
            double slope = Sxy / Sxx;
            angle = std::atan(slope) * 180.0 / CV_PI;
        }
    } else {
        // 以y为自变量更稳定
        if (std::abs(Syy) > 1e-6) {
            double slope = Sxy / Syy;  // x关于y的斜率
            angle = 90.0 - std::atan(slope) * 180.0 / CV_PI;  // 转换为y关于x的斜率
        }
    }
    
    // 7. 调整角度范围（与PCA方法对齐，得到法线方向）
    angle += 90.0;
    while (angle > 90) angle -= 180;
    while (angle < -90) angle += 180;
    
    return static_cast<float>(angle);
}

void LightBarDetector::detectLights(cv::Mat& img) {
    lights.clear();  // 清除上一帧的检测结果

    // 1. 提取二值化图片
    cv::Mat binary_img = binaryImg(img);

    cv::Mat color_diff;
    color_diff = extractColorChannelDiff(img);

    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Mat subtract_gray_img;
    cv::subtract(gray_img, subtract_value, subtract_gray_img);
    cv::max(subtract_gray_img, 0, subtract_gray_img);  // 截断到0

    // 2. 检测可能的灯条
    std::vector<cv::RotatedRect> detectedRects = detectLightRects(binary_img);

    // 3. 移除颜色错误的灯条，只保留目标颜色的灯条

    // 进行多线程优化
    int lightRectsNum = detectedRects.size();
    std::vector<LightDetectThreadInfo> lightDetectThreadInfos(lightRectsNum);
    for (size_t i = 0; i < lightRectsNum; ++i) {
        lightDetectThreadInfos[i].lightRect = &detectedRects[i];
    }


    // 并行灯条处理：线程池替代 std::execution::par
    ::utils::threadPool().parallel_for_each(lightDetectThreadInfos.begin(), lightDetectThreadInfos.end(),
    [&](LightDetectThreadInfo& lightDetectThreadInfo) {

        cv::RotatedRect& rect = *lightDetectThreadInfo.lightRect;

        // 1. 计算角度并标准化到[-90, 90]
        if (rect.size.width > rect.size.height) {
            rect.angle += 90;  // 确保角度始终表示长边的方向
        }
        while (rect.angle > 90) rect.angle -= 180;
        while (rect.angle < -90) rect.angle += 180;

        // 2. 计算长宽（确保length始终为较长边）
        float length = std::max(rect.size.width, rect.size.height);
        float width = std::min(rect.size.width, rect.size.height);
        rect.size.height = length;
        rect.size.width = width;

        // 3. 使用加权最小二乘法修正角度
        rect.angle = calculateAccurateAngleByOLS(subtract_gray_img, cv::RotatedRect(
            rect.center, 
            cv::Size2f(rect.size.width * 2.0, 
                    rect.size.height * 1.2),
            rect.angle
        ));

        // 3.5 移除倾向角度过大的旋转矩形
        if (fabs(rect.angle) > light_bar_max_angle) {
            lightDetectThreadInfo.is_true_light = false;
            return;
        }

        // 4. 将检测到的旋转矩形转换为Light对象
        Light temp_light = Light(rect);
                
        if (enemy_color != Params::BOTH) {
            // 2. 获取扩张后的旋转矩形
            cv::RotatedRect expandedRect = rectExpand(rect, color_rect_expand_FACTOR);

            // 3. 获取矩形范围内通道差值图像的均值
            float mean_color_diff = calculateMeanInRotatedRect(color_diff, expandedRect);

            // 4. 移除小于阈值的图像
            tools::logger()->debug("mean_color_diff: {:.2f}", mean_color_diff);
            float mean_color_diff_THRESHOLD;
            if (params.enemy_color == Params::BLUE) {
                mean_color_diff_THRESHOLD = mean_color_diff_THRESHOLD_BLUE;
            } else {
                mean_color_diff_THRESHOLD = mean_color_diff_THRESHOLD_RED;
            }
            if (mean_color_diff < mean_color_diff_THRESHOLD) {
                lightDetectThreadInfo.is_true_light = false;
                return;
            }
        }


        // 5. 修正在拟合旋转矩形时造成的长度误差
        temp_light.correctLengthAndWidth(subtract_gray_img);

        lightDetectThreadInfo.light = temp_light;
        lightDetectThreadInfo.is_true_light = true;
    });
    
    // 统计结果
    for (const auto& lightDetectThreadInfo : lightDetectThreadInfos) {
        if (lightDetectThreadInfo.is_true_light) {
            lights.push_back(lightDetectThreadInfo.light);
        }
    }
    //cv::cvtColor(binary_img, img, cv::COLOR_GRAY2BGR);

    if (show_windows_) {
        // cv::imshow("Light Bar Debug: binary_img", binary_img);
        // cv::imshow("Light Bar Debug: color_diff", color_diff);
        // cv::imshow("Light Bar Debug: gray_img", gray_img);
        // cv::imshow("Light Bar Debug: subtract_gray_img", subtract_gray_img);
    }
}

cv::Mat LightBarDetector::binaryImg(const cv::Mat& img) {
    // 1. 获取灰度图
    
    // 1. 分离BGR通道
    std::vector<cv::Mat> channels;
    cv::split(img, channels);  // channels[0]=B, [1]=G, [2]=R

    // 2. 根据敌方颜色提取对应的通道
    cv::Mat gray_img;
    switch (enemy_color) {
        case Params::RED:
            // 红色装甲板：R通道
            gray_img = channels[2];
            break;
        case Params::BLUE:
            // 蓝色装甲板：B通道
            gray_img = channels[0];
            break;
        case Params::BOTH:
            // 识别两者：R通道和B通道最大值
            gray_img = cv::max(channels[0], channels[2]);
            break;
        default:
            // 默认情况：灰度图
            cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
            break;
    }

    // cv::Mat gray_img;
    // cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    
    // 1. 获取二值图
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, binary_img_THRESHOLD, 255, cv::THRESH_BINARY);

    return binary_img;
}

cv::RotatedRect LightBarDetector::rectExpand(const cv::RotatedRect& rect, float factor) {
    return cv::RotatedRect(
        rect.center, 
        cv::Size2f(rect.size.width * factor, 
                  rect.size.height * factor),
        rect.angle
    );
}

float LightBarDetector::calculateMeanInRotatedRect(const cv::Mat& grayImage, const cv::RotatedRect& rect) {
    cv::Rect boundingBox = rect.boundingRect();
    boundingBox &= cv::Rect(0, 0, grayImage.cols, grayImage.rows);
    if (boundingBox.width <= 0 || boundingBox.height <= 0) return 0.0f;

    // 创建与 boundingBox 等大的掩码
    cv::Mat mask(boundingBox.size(), CV_8UC1, cv::Scalar(0));

    // 获取旋转矩形的顶点并平移到掩码坐标系
    cv::Point2f vertices[4];
    rect.points(vertices);
    std::vector<cv::Point> intVertices(4);
    for (int i = 0; i < 4; ++i) {
        intVertices[i] = cv::Point(static_cast<int>(vertices[i].x - boundingBox.x),
                                   static_cast<int>(vertices[i].y - boundingBox.y));
    }
    cv::fillConvexPoly(mask, intVertices, cv::Scalar(255));

    // 对 ROI 和掩码求均值
    cv::Scalar meanVal = cv::mean(grayImage(boundingBox), mask);
    return meanVal[0];
}

cv::Mat LightBarDetector::extractColorChannelDiff(const cv::Mat& img) {
    // 1. 分离BGR通道
    std::vector<cv::Mat> channels;
    cv::split(img, channels);  // channels[0]=B, [1]=G, [2]=R

    // 2. 根据敌方颜色提取对应的通道差值
    cv::Mat color_diff;
    switch (enemy_color) {
        case Params::RED:
            // 红色装甲板：R通道减B通道
            cv::subtract(channels[2], channels[0], color_diff);
            //cv::threshold(color_diff, color_diff, THRES_MAX_COLOR_RED, 255, cv::THRESH_BINARY);
            break;
        case Params::BLUE:
            // 蓝色装甲板：B通道减R通道
            cv::subtract(channels[0], channels[2], color_diff);
            //cv::threshold(color_diff, color_diff, THRES_MAX_COLOR_BLUE, 255, cv::THRESH_BINARY);
            break;
        case Params::BOTH:
            // 识别两者：上述两者最大值
            {
            cv::Mat color_diff_1, color_diff_2;
            cv::subtract(channels[2], channels[0], color_diff_1);
            cv::subtract(channels[0], channels[2], color_diff_2);
            color_diff = cv::max(color_diff_1, color_diff_2);
            }
            break;
        default:
            // 默认情况：G通道减R通道
            cv::subtract(channels[1], channels[0], color_diff);
            break;
    }
    return color_diff;
}

std::vector<cv::RotatedRect> LightBarDetector::detectLightRects(const cv::Mat& img) {
    std::vector<cv::RotatedRect> rects;
    
    // 1. 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 2. 遍历所有轮廓
    for (const auto& contour : contours) {
        // 检查轮廓点数是否足够拟合椭圆
        if (contour.size() < 5) continue;
        
        // 检查轮廓面积
        float area = cv::contourArea(contour);
        if (area < params.light_min_area) continue;
        if (area > params.light_max_area) continue;

        // 3. 拟合旋转矩形
        cv::RotatedRect rect = cv::fitEllipse(contour);

        // 4. 标准化宽高和角度
        float width = std::min(rect.size.width, rect.size.height);
        float length = std::max(rect.size.width, rect.size.height);
        float angle = rect.angle;
        
        if (rect.size.width > rect.size.height) {
            std::swap(width, length);
            angle += 90;
        }

        // 5. 标准化角度到[-90, 90]
        while (angle > 90) angle -= 180;
        while (angle < -90) angle += 180;

        // 6. 检查几何约束条件
        float ratio = length / width;
        if (ratio >= params.min_light_wh_ratio &&
            ratio <= params.max_light_wh_ratio &&
            length >= params.min_light_height &&
            std::abs(angle) <= params.light_max_tilt_angle) {
            
            // 创建标准化后的旋转矩形
            cv::RotatedRect newRect(rect.center, cv::Size2f(width, length), angle);
            rects.push_back(newRect);
        }
    }

    return rects;
}

void LightBarDetector::processLights() {
    // filterLights();    // 过滤不合格的灯条
    // updateLights();    // 更新灯条状态（用于追踪）
}

void LightBarDetector::filterLights() {
    // 移除重叠的灯条，只保留较大的灯条
    for (size_t i = 0; i < lights.size(); ++i) {
        for (size_t j = i + 1; j < lights.size(); ++j) {
            if (isOverlap(lights[i], lights[j])) {
                // 保留较大的灯条
                if (lights[i].length * lights[i].width < lights[j].length * lights[j].width) {
                    lights.erase(lights.begin() + i);
                    --i;
                    break;
                } else {
                    lights.erase(lights.begin() + j);
                    --j;
                }
            }
        }
    }
}

bool LightBarDetector::isOverlap(const Light& light1, const Light& light2) {
    // 获取两个灯条的旋转矩形交集区域
    cv::Rect rect1 = light1.el.boundingRect();
    cv::Rect rect2 = light2.el.boundingRect();
    
    // 计算两个矩形的交集
    cv::Rect intersection = rect1 & rect2;
    
    // 如果交集面积大于一定阈值，则认为是重叠的
    return intersection.area() > (rect1.area() * 0.5) || intersection.area() > (rect2.area() * 0.5);
}

void LightBarDetector::updateLights() {
    // 预留用于实现灯条追踪功能
    // TODO: 实现灯条追踪逻辑
}
