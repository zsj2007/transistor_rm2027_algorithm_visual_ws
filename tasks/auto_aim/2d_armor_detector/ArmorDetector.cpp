// ArmorDetector.cpp
#include "2d_armor_detector/ArmorDetector.h"
#include "utils/ThreadPool.h"

struct alignas(64) ArmorDetectThreadInfo { // 64字节对齐
    const Light* leftLight;
    const Light* rightLight;
    bool is_true_armor = false;
    Armor armor;
};

std::vector<Armor> ArmorDetector::detectArmors(const std::vector<Light>& lights) {
    std::vector<Armor> armors;

    // 进行多线程优化
    int maxArmorsNum = lights.size() * (lights.size() - 1) / 2;
    std::vector<ArmorDetectThreadInfo> armorDetectThreadInfos(maxArmorsNum);
    // 2. 遍历所有可能的灯条对
    size_t thread_index = 0;
    for (size_t i = 0; i < lights.size(); i++) {
        for (size_t j = i + 1; j < lights.size(); j++) {
            const Light& l1 = lights[i];
            const Light& l2 = lights[j];
            
            armorDetectThreadInfos[thread_index].leftLight = &((l1.el.center.x < l2.el.center.x) ? l1 : l2);
            armorDetectThreadInfos[thread_index].rightLight = &((l1.el.center.x < l2.el.center.x) ? l2 : l1);
            thread_index += 1;
        }
    }
    
    // 并行配对检测：线程池替代 std::execution::par
    ::utils::threadPool().parallel_for_each(armorDetectThreadInfos.begin(), armorDetectThreadInfos.end(),
    [&](ArmorDetectThreadInfo& armorDetectThreadInfo) {
        const Light& leftLight = *armorDetectThreadInfo.leftLight;
        const Light& rightLight = *armorDetectThreadInfo.rightLight;
        if (isArmorPair(leftLight, rightLight)) {
            Armor armor(leftLight.el, rightLight.el, config_file_ptr);
            armor.confidence = getArmorConfidence(leftLight, rightLight);
            
            // 只添加置信度足够高的装甲板
            if (armor.confidence >= min_armor_confidence) {
                armorDetectThreadInfo.is_true_armor = true;
                armorDetectThreadInfo.armor = armor;
            }
        }
    });

    // 统计结果
    for (size_t i = 0; i < maxArmorsNum; i++) {
        if (armorDetectThreadInfos[i].is_true_armor) {
            armors.push_back(armorDetectThreadInfos[i].armor);
        }
    }

    // 根据置信度排序
    std::sort(armors.begin(), armors.end(),
        [](const Armor& a1, const Armor& a2) {
            return a1.confidence > a2.confidence;
        });

    return armors;
}


// 修改装甲板配对条件
bool ArmorDetector::isArmorPair(const Light& l1, const Light& l2) {

    // 获取灯条平均长方向向量
    float rad_l1 = -l1.el.angle * M_PI / 180.0;
    float rad_l2 = -l2.el.angle * M_PI / 180.0;
    cv::Vec2f l1_length_direction = cv::Vec2f(std::sin(rad_l1), std::cos(rad_l1));
    cv::Vec2f l2_length_direction = cv::Vec2f(std::sin(rad_l2), std::cos(rad_l2));
    if (std::abs(rad_l1 - rad_l2) > M_PI/2)
    {
        l2_length_direction = -l2_length_direction;
    }
    cv::Vec2f average_length_direction = l1_length_direction + l2_length_direction;
    average_length_direction /= cv::norm(average_length_direction);

    // 1. 检查灯条间距
    cv::Vec2f d_center_vector = cv::Vec2f(l1.el.center - l2.el.center);
    float distance = cv::norm(d_center_vector);
    float avg_light_height = (l1.length + l2.length) / 2.0f;
    
    float expected_small_distance = ArmorConstants::SMALL_DISTANCE_RATIO * avg_light_height;
    float expected_large_distance = ArmorConstants::LARGE_DISTANCE_RATIO * avg_light_height;
    
    bool distance_match = false;
    if (std::abs(distance - expected_small_distance) / expected_small_distance <= max_expected_small_distance_mismatch_ratio ||
        std::abs(distance - expected_large_distance) / expected_large_distance <= max_expected_large_distance_mismatch_ratio) {
        distance_match = true;
    }
    
    if (!distance_match) return false;

    // 2. 检查灯条在长度方向上的错位
    float length_direction_mismatch = std::abs(d_center_vector.dot(average_length_direction));
    float max_length_direction_mismatch = max_length_direction_mismatch_ratio * avg_light_height;
    if (length_direction_mismatch > max_length_direction_mismatch)
    {
        return false;
    }

    // 3. 检查灯条平行度
    float angleDiff = std::abs(l1.angle - l2.angle);
    if (angleDiff > max_angle_diff) {
        return false;
    }
    
    // 4. 检查灯条高度比例
    float heightDiff = std::abs(l1.length - l2.length);
    if (heightDiff / std::min(l1.length, l2.length) > max_height_diff_ratio) {
        return false;
    }
    
    return true;
}

float ArmorDetector::getArmorConfidence(const Light& l1, const Light& l2) {
    // 1. 角度差得分
    float angleDiff = std::abs(l1.angle - l2.angle);
    float angleScore = 1.0f - (angleDiff / max_angle_diff * 1.5f);
    
    // 2. 高度差得分
    float heightDiff = std::abs(l1.length - l2.length);
    float averageHeight = (l1.length + l2.length) / 2;
    float heightScore = 1.0f - (heightDiff / averageHeight);
    
    // 3. 距离得分
    float distance = cv::norm(l1.el.center - l2.el.center);
    float expectedDistance_small = averageHeight * ArmorConstants::SMALL_DISTANCE_RATIO; // 理想的灯条间距
    float expectedDistance_large = averageHeight * ArmorConstants::LARGE_DISTANCE_RATIO;
    float distanceScore = 1.0f - std::min(
        std::abs(distance - expectedDistance_small) / expectedDistance_small,
        std::abs(distance - expectedDistance_large) / expectedDistance_large
    );
    
    // 综合评分
    return (angleScore + heightScore + distanceScore) / 3.0f;
}
