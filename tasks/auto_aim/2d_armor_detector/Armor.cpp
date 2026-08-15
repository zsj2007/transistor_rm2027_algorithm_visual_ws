// Armor.cpp
#include "2d_armor_detector/Armor.h"

namespace ArmorType {
    std::vector<std::string> ArmorTypeStrings = {
        "Hero",
        "Engineer",
        "Infantry1",
        "Infantry2",
        "Infantry3",
        "Sentry",
        "Outpost",
        "Base",
        "Middle(should not be used)",
        "Nearest(should not be used)"
    };
}

// 带参数的构造函数
Armor::Armor(const cv::RotatedRect& left, const cv::RotatedRect& right, std::shared_ptr<YAML::Node> config_file_ptr, int history_frame_identifier) 
    : leftLight(left), rightLight(right), confidence(0.0f) {
    corners_expand_ratio = (*config_file_ptr)["corners_expand_ratio"].as<float>();
    corners_narrow_ratio = (*config_file_ptr)["corners_narrow_ratio"].as<float>();
    height_correct_ratio_small = (*config_file_ptr)["height_correct_ratio_small"].as<float>();
    width_correct_ratio_small = (*config_file_ptr)["width_correct_ratio_small"].as<float>();
    height_correct_ratio_large = (*config_file_ptr)["height_correct_ratio_large"].as<float>();
    width_correct_ratio_large = (*config_file_ptr)["width_correct_ratio_large"].as<float>();
    is_ture_yolo_armor_brightness_ratio = (*config_file_ptr)["is_ture_yolo_armor_brightness_ratio"].as<float>();
    calculateROI();

    if (history_frame_identifier != -1) {
        delayed_result.is_delayed_result = true;
        delayed_result.history_frame_identifier = history_frame_identifier;
    }
}

// ROI计算函数
void Armor::calculateROI() {
    // 更新corners向量
    corners.clear();
    calculateCorners();
    // RCLCPP_DEBUG(node->get_logger(), "----------Armor Debug Flag----------");

    // 计算ROI
    roi = cv::boundingRect(corners);
}

void Armor::calculateCorners() {
    // 获取左右灯条的顶点
    cv::Point2f left_vertices[4], right_vertices[4];
    leftLight.points(left_vertices);
    rightLight.points(right_vertices);
    
    // 找到左右灯条的中心点
    cv::Point2f left_center = leftLight.center;
    cv::Point2f right_center = rightLight.center;

    // 获取左右灯条的顶点转换为相对中心点的坐标
    cv::Vec2f relative_left_vertices[4], relative_right_vertices[4];
    for (int i = 0; i < 4; i+=1) {
        relative_left_vertices[i] = pointToVec(left_vertices[i] - left_center);
        relative_right_vertices[i] = pointToVec(right_vertices[i] - right_center);
    }

    // 获取左灯条中心点指向右灯条中心点的向量及垂直其向上的向量，并单位化
    cv::Vec2f d_center_vector = right_center - left_center;
    cv::Vec2f vertical_d_center_vector = cv::Vec2f(d_center_vector[1], -d_center_vector[0]);
    d_center_vector = cv::normalize(d_center_vector);
    vertical_d_center_vector = cv::normalize(vertical_d_center_vector);

    // 获取左右灯条长边及短边的单位方向向量，并调整方向为向右或向上
    float rad_left = -leftLight.angle * M_PI / 180.0;
    float rad_right = -rightLight.angle * M_PI / 180.0;
    cv::Vec2f left_length_direction = cv::Vec2f(std::sin(rad_left), std::cos(rad_left));
    cv::Vec2f left_width_direction = cv::Vec2f(-left_length_direction[1], left_length_direction[0]);
    cv::Vec2f right_length_direction = cv::Vec2f(std::sin(rad_right), std::cos(rad_right));
    cv::Vec2f right_width_direction = cv::Vec2f(-right_length_direction[1], right_length_direction[0]);
    if (left_length_direction.dot(vertical_d_center_vector) < 0) left_length_direction = -left_length_direction;
    if (left_width_direction.dot(d_center_vector) < 0) left_width_direction = -left_width_direction;
    if (right_length_direction.dot(vertical_d_center_vector) < 0) right_length_direction = -right_length_direction;
    if (right_width_direction.dot(d_center_vector) < 0) right_width_direction = -right_width_direction;

    // 将顶点相对坐标拆分为沿长边和短边两部分
    cv::Vec2f horizontal_relative_left_vertices[4], horizontal_relative_right_vertices[4],
                vertical_relative_left_vertices[4], vertical_relative_right_vertices[4];
    for (int i = 0; i < 4; i+=1) {
        horizontal_relative_left_vertices[i] = left_width_direction * relative_left_vertices[i].dot(left_width_direction);
        horizontal_relative_right_vertices[i] = right_width_direction * relative_right_vertices[i].dot(right_width_direction);
        vertical_relative_left_vertices[i] = left_length_direction * relative_left_vertices[i].dot(left_length_direction);
        vertical_relative_right_vertices[i] = right_length_direction * relative_right_vertices[i].dot(right_length_direction);
    }


    // 获取灯条顶点坐标中靠近装甲板中心的四个点的相对中心点坐标的沿长边和短边两部分
    cv::Vec2f left_up_horizontal, left_up_vertical, left_down_horizontal, left_down_vertical, 
                right_up_horizontal, right_up_vertical, right_down_horizontal, right_down_vertical;
    for (int i = 0; i < 4; i+=1) {
        if (horizontal_relative_left_vertices[i].dot(left_width_direction) >= 0)
        {
            if (vertical_relative_left_vertices[i].dot(left_length_direction) >= 0)
            {
                left_up_horizontal = horizontal_relative_left_vertices[i];
                left_up_vertical = vertical_relative_left_vertices[i];
            }
            else
            {
                left_down_horizontal = horizontal_relative_left_vertices[i];
                left_down_vertical = vertical_relative_left_vertices[i];
            }
        }
        if (horizontal_relative_right_vertices[i].dot(right_width_direction) <= 0)
        {
            if (vertical_relative_right_vertices[i].dot(right_length_direction) >= 0)
            {
                right_up_horizontal = horizontal_relative_right_vertices[i];
                right_up_vertical = vertical_relative_right_vertices[i];
            }
            else
            {
                right_down_horizontal = horizontal_relative_right_vertices[i];
                right_down_vertical = vertical_relative_right_vertices[i];
            }
        }
    }

    // 获取灯条短边中心点作为灯条顶点，用于pnp
    light_bar_corners.push_back(left_center + vecToPoint(left_up_vertical));
    light_bar_corners.push_back(left_center + vecToPoint(left_down_vertical));
    light_bar_corners.push_back(right_center + vecToPoint(right_down_vertical));
    light_bar_corners.push_back(right_center + vecToPoint(right_up_vertical));

    // 沿长边部分使用小装甲板比例获得装甲板高度相对坐标
    left_up_vertical *= ArmorConstants::SMALL_HEIGHT_RATIO;
    left_down_vertical *= ArmorConstants::SMALL_HEIGHT_RATIO;
    right_up_vertical *= ArmorConstants::SMALL_HEIGHT_RATIO;
    right_down_vertical *= ArmorConstants::SMALL_HEIGHT_RATIO;

    std::vector<cv::Point2f> corners_biased;

    // 按从左上角开始逆时针排序
    corners_biased.push_back(left_center + vecToPoint(left_up_horizontal + left_up_vertical));
    corners_biased.push_back(left_center + vecToPoint(left_down_horizontal + left_down_vertical));
    corners_biased.push_back(right_center + vecToPoint(right_down_horizontal + right_down_vertical));
    corners_biased.push_back(right_center + vecToPoint(right_up_horizontal + right_up_vertical));

    // 计算中心
    center = computeIntersection(corners_biased);

    // 拆分为装甲板中心坐标下纵向和横向坐标，修正坐标并输出
    for (int i = 0; i < 4; i+=1) {
        cv::Vec2f center_to_corner = pointToVec(corners_biased[i] - center);
        cv::Vec2f center_to_corner_height = vertical_d_center_vector * center_to_corner.dot(vertical_d_center_vector);
        cv::Vec2f center_to_corner_width = d_center_vector * center_to_corner.dot(d_center_vector);
        corners.push_back(center + 
            vecToPoint(center_to_corner_height * height_correct_ratio_small + center_to_corner_width * width_correct_ratio_small));
        corners_large.push_back(center + 
            vecToPoint(center_to_corner_height * height_correct_ratio_large + center_to_corner_width * width_correct_ratio_large));
    }

    // 计算扩大后角点坐标
    for (int i = 0; i < 4; i+=1) {
        corners_expanded.push_back(center + corners_expand_ratio * (corners[i] - center));
    }

    // 计算缩小后角点坐标
    for (int i = 0; i < 4; i+=1) {
        corners_narrowed.push_back(center + corners_narrow_ratio * (corners[i] - center));
    }
}

cv::Point2f Armor::vecToPoint(const cv::Vec2f& vec) {
    return cv::Point2f(vec[0], vec[1]);
}

cv::Vec2f Armor::pointToVec(const cv::Point2f& point) {
    return cv::Vec2f(point.x, point.y);
}

cv::Point2f Armor::computeIntersection(const std::vector<cv::Point2f>& corners) {
    // 提取对角线端点
    cv::Point2f A1 = corners[0]; // 左上角
    cv::Point2f A2 = corners[2]; // 右下角
    cv::Point2f B1 = corners[1]; // 左下角
    cv::Point2f B2 = corners[3]; // 右上角

    // 计算向量
    cv::Point2f a = A2 - A1; // 对角线1的向量
    cv::Point2f b = B2 - B1; // 对角线2的向量
    cv::Point2f c = B1 - A1; // 从A1指向B1的向量

    // 计算叉积
    float cross_ab = a.cross(b); // a × b
    float cross_cb = c.cross(b); // c × b

    // 检查对角线是否平行
    if (std::fabs(cross_ab) < 1e-6) {
        throw std::runtime_error("Diagonals are parallel, no intersection.");
    }

    // 计算参数t
    float t = cross_cb / cross_ab;

    // 计算交点坐标
    cv::Point2f P = A1 + t * a;

    return P;
}

bool Armor::is_true_yolo_armor(cv::Mat& frame) {
    // 将图像转换为灰度图
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 1. 计算左右灯条范围内的亮度均值
    // 创建灯条区域的掩膜
    cv::Mat light_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    
    // 获取左右灯条的顶点
    cv::Point2f left_vertices[4], right_vertices[4];
    leftLight.points(left_vertices);
    rightLight.points(right_vertices);
    
    // 填充左灯条多边形
    std::vector<cv::Point> left_poly(4);
    for (int i = 0; i < 4; ++i) {
        left_poly[i] = left_vertices[i];
    }
    cv::fillPoly(light_mask, std::vector<std::vector<cv::Point>>{left_poly}, cv::Scalar(255));
    
    // 填充右灯条多边形
    std::vector<cv::Point> right_poly(4);
    for (int i = 0; i < 4; ++i) {
        right_poly[i] = right_vertices[i];
    }
    cv::fillPoly(light_mask, std::vector<std::vector<cv::Point>>{right_poly}, cv::Scalar(255));
    
    // 计算灯条区域的平均亮度
    cv::Scalar light_mean_scalar = cv::mean(gray, light_mask);
    double light_brightness_mean = light_mean_scalar[0];  // a

    // 2. 计算corners_narrowed范围内的0.95分位值
    // 创建装甲板缩小区域的掩膜
    cv::Mat armor_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    
    // 填充装甲板缩小区域多边形
    std::vector<cv::Point> armor_poly(4);
    for (int i = 0; i < 4; ++i) {
        armor_poly[i] = corners_narrowed[i];
    }
    cv::fillPoly(armor_mask, std::vector<std::vector<cv::Point>>{armor_poly}, cv::Scalar(255));
    
    // 提取装甲板区域内的像素值
    std::vector<double> armor_pixel_values;
    for (int y = 0; y < gray.rows; ++y) {
        for (int x = 0; x < gray.cols; ++x) {
            if (armor_mask.at<uchar>(y, x) > 0) {
                armor_pixel_values.push_back(static_cast<double>(gray.at<uchar>(y, x)));
            }
        }
    }
    
    // 如果没有像素，返回false
    if (armor_pixel_values.empty()) {
        return false;
    }
    
    // 计算0.95分位值
    std::sort(armor_pixel_values.begin(), armor_pixel_values.end());
    size_t index_95 = static_cast<size_t>(armor_pixel_values.size() * 0.95);
    if (index_95 >= armor_pixel_values.size()) {
        index_95 = armor_pixel_values.size() - 1;
    }
    double armor_brightness_95percentile = armor_pixel_values[index_95];  // b

    // 3. 比较并返回结果
    return light_brightness_mean > armor_brightness_95percentile * is_ture_yolo_armor_brightness_ratio;
}
