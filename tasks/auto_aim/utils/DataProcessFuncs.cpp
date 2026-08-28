#include "utils/DataProcessFuncs.h"


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

// 将相机到车辆中心的方向与装甲板外法线夹角映射到 [0, 1] 的正对损失。
// 输入方向退化或非有限时返回中性损失 0.5，防止无效几何主导选板。
double normalizedArmorFacingLoss(
    const cv::Point2d& camera_to_center_direction, double armor_yaw) {
    const double direction_norm = cv::norm(camera_to_center_direction);
    if (direction_norm <= 1e-12 || !std::isfinite(direction_norm)) {
        return 0.5;
    }
    const cv::Point2d unit_direction =
        camera_to_center_direction / direction_norm;
    const cv::Point2d armor_normal{
        std::sin(armor_yaw), -std::cos(armor_yaw)};
    const double cosine = std::clamp(
        unit_direction.dot(armor_normal), -1.0, 1.0);
    // 正对相机时 cosine=-1、损失为 0；背对相机时损失为 1。
    return 0.5 * (1.0 + cosine);
}

// 总损失仅由归一化正对损失和切板惩罚组成，不引入角速度等第三项。
// 以当前板作为初始最优项，使浮点损失相同时保持当前板而不发生切换。
int selectArmorByFacingAndSwitchPenalty(
    const std::vector<double>& facing_losses,
    int previous_id,
    double switch_penalty) {
    const bool previous_valid =
        previous_id >= 0 &&
        static_cast<std::size_t>(previous_id) < facing_losses.size() &&
        std::isfinite(facing_losses[static_cast<std::size_t>(previous_id)]);
    const double penalty = std::max(0.0, switch_penalty);
    int best_id = previous_valid ? previous_id : -1;
    double best_loss = previous_valid
        ? facing_losses[static_cast<std::size_t>(previous_id)]
        : std::numeric_limits<double>::infinity();

    for (std::size_t id = 0; id < facing_losses.size(); ++id) {
        if (!std::isfinite(facing_losses[id])) continue;
        const double total_loss = facing_losses[id] +
            (previous_valid && static_cast<int>(id) != previous_id
                 ? penalty
                 : 0.0);
        if (total_loss + 1e-12 < best_loss) {
            best_loss = total_loss;
            best_id = static_cast<int>(id);
        }
    }
    return best_id;
}
