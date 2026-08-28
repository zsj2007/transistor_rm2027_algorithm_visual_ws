#include "utils/DataProcessFuncs.h"

#include <limits>


LinearRegressionResult linearRegression(const std::vector<double>& x, const std::vector<double>& y) {
    LinearRegressionResult result;
    int n = x.size();
    
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    for (int i = 0; i < n; i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
    }
    
    double denominator = n * sum_xx - sum_x * sum_x;
    if (std::abs(denominator) < 1e-10) {
        result.a = 0.0;
        result.b = 0.0;
    } else {
        result.b = (n * sum_xy - sum_x * sum_y) / denominator;
        result.a = (sum_y - result.b * sum_x) / n;
    }
    
    return result;
}

double variance(const std::vector<double>& signal) {
    double signal_mean = 0.0;
    int n = signal.size();
    
    for (double val : signal) {
        signal_mean += val;
    }
    signal_mean /= n;
    
    double var = 0.0;
    for (double val : signal) {
        var += (val - signal_mean) * (val - signal_mean);
    }
    
    return var / n;
}

double variance(const std::deque<double>& signal) {
    return variance(std::vector<double>(signal.begin(), signal.end()));
}

float variance(const std::vector<float>& signal) {
    float signal_mean = 0.0;
    int n = signal.size();
    
    for (float val : signal) {
        signal_mean += val;
    }
    signal_mean /= n;
    
    float var = 0.0;
    for (float val : signal) {
        var += (val - signal_mean) * (val - signal_mean);
    }
    
    return var / n;
}

float variance(const std::deque<float>& signal) {
    return variance(std::vector<float>(signal.begin(), signal.end()));
}

std::vector<double> linearInterpolation(const std::vector<double>& data, int refineMultiple) {
    int result_len = (data.size() - 1) * refineMultiple + 1;
    std::vector<double> result(result_len, 0.0);
    
    for (int result_i = 0; result_i < result_len; result_i++) {
        int origin_i = result_i / refineMultiple;
        int result_i_left_part = result_i - origin_i * refineMultiple;
        
        if (result_i_left_part == 0) {
            result[result_i] = data[origin_i];
        } else {
            double weight_high = static_cast<double>(result_i_left_part) / refineMultiple;
            double weight_low = 1.0 - weight_high;
            result[result_i] = weight_low * data[origin_i] + weight_high * data[origin_i + 1];
        }
    }
    
    return result;
}

double meanSquaredError(const std::vector<double>& pred_value, const std::vector<double>& true_value) {
    double result = 0.0;
    size_t value_num = std::min(pred_value.size(), true_value.size());
    if (value_num == 0) {
        return 0.0;
    }
    for (size_t value_index = 0; value_index < value_num; value_index++) {
        double value_error = pred_value[value_index] - true_value[value_index];
        result += value_error * value_error;
    }
    return result / static_cast<double>(value_num);
}

double variancePoint3f(const std::vector<cv::Point3f>& points) {
    int n = points.size();
    cv::Point3d points_mean = {0.0, 0.0, 0.0};
    
    for (cv::Point3d point : points) {
        points_mean += point;
    }
    points_mean /= n;
    
    double var = 0.0;
    for (cv::Point3d point : points) {
        var += (point.x - points_mean.x) * (point.x - points_mean.x) + 
               (point.y - points_mean.y) * (point.y - points_mean.y) + 
               (point.z - points_mean.z) * (point.z - points_mean.z);
    }
    
    return var / n;
}

double meanSquaredErrorPoint3f(const std::vector<cv::Point3f>& pred_points, const std::vector<cv::Point3f>& true_points) {
    double result = 0.0;
    size_t value_num = std::min(pred_points.size(), true_points.size());
    if (value_num == 0) {
        return 0.0;
    }
    for (size_t value_index = 0; value_index < value_num; value_index++) {
        double value_error_x = pred_points[value_index].x - true_points[value_index].x;
        double value_error_y = pred_points[value_index].y - true_points[value_index].y;
        double value_error_z = pred_points[value_index].z - true_points[value_index].z;
        result += value_error_x * value_error_x + value_error_y * value_error_y + value_error_z * value_error_z;
    }
    return result / static_cast<double>(value_num);
}

std::pair<int, int> findTwoSmallestIndices(const std::vector<double>& nums) {
    if (nums.size() < 2) {
        // 处理元素不足的情况
        return {-1, -1};
    }
    
    int min1 = 0;  // 最小值的索引
    int min2 = 1;  // 第二小值的索引
    
    // 初始化，确保min1对应较小的值
    if (nums[1] < nums[0]) {
        std::swap(min1, min2);
    }
    
    // 从第三个元素开始遍历
    for (int i = 2; i < nums.size(); i++) {
        if (nums[i] < nums[min1]) {
            // 当前元素比最小值还小
            min2 = min1;  // 原来的最小值变成第二小
            min1 = i;     // 更新最小值索引
        } else if (nums[i] < nums[min2]) {
            // 当前元素比最小值大，但比第二小值小
            min2 = i;
        }
    }
    
    return {min1, min2};
}

// 把装甲板相对正对方向的角度乘以旋转方向，使正反转都统一为从出现到消失递增。
double directedArmorVisibilityAngle(
    const cv::Point2d& camera_to_center_direction,
    double armor_yaw,
    int rotation_direction) {
    const double direction_norm = cv::norm(camera_to_center_direction);
    if (direction_norm <= 1e-12 || !std::isfinite(direction_norm) ||
        !std::isfinite(armor_yaw)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const cv::Point2d unit_camera_to_center =
        camera_to_center_direction / direction_norm;
    const double camera_facing_yaw = std::atan2(
        -unit_camera_to_center.x, unit_camera_to_center.y);
    const int direction = rotation_direction >= 0 ? 1 : -1;
    const double yaw_difference =
        (armor_yaw - camera_facing_yaw) * static_cast<double>(direction);
    const double wrapped_difference = std::atan2(
        std::sin(yaw_difference), std::cos(yaw_difference));
    return wrapped_difference + M_PI / 2.0;
}

// 使用左闭右开区间避免 45 度和 135 度边界同时属于两个区域。
ArmorVisibilityRegion classifyArmorVisibilityRegion(double directed_angle) {
    if (!std::isfinite(directed_angle) ||
        directed_angle < 0.0 || directed_angle >= M_PI) {
        return ArmorVisibilityRegion::Invisible;
    }
    if (directed_angle < M_PI / 4.0) {
        return ArmorVisibilityRegion::Appearing;
    }
    if (directed_angle < 3.0 * M_PI / 4.0) {
        return ArmorVisibilityRegion::GoldenShooting;
    }
    return ArmorVisibilityRegion::Disappearing;
}

// 枚举值就是区域优先级；若当前板与候选板同级，保留当前板抑制无意义切换。
int selectArmorByVisibilityRegion(
    const std::vector<ArmorVisibilityRegion>& regions,
    int current_id) {
    const bool current_valid =
        current_id >= 0 &&
        static_cast<std::size_t>(current_id) < regions.size() &&
        regions[static_cast<std::size_t>(current_id)] !=
            ArmorVisibilityRegion::Invisible;
    int best_id = current_valid ? current_id : -1;
    int best_priority = current_valid
        ? static_cast<int>(regions[static_cast<std::size_t>(current_id)])
        : static_cast<int>(ArmorVisibilityRegion::Invisible);

    for (std::size_t id = 0; id < regions.size(); ++id) {
        const int priority = static_cast<int>(regions[id]);
        if (priority > best_priority) {
            best_priority = priority;
            best_id = static_cast<int>(id);
        }
    }
    return best_id;
}
