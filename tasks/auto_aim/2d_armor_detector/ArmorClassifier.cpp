// ArmorCLassifier.cpp
#include "2d_armor_detector/ArmorClassifier.h"
#include "utils/ThreadPool.h"

#include "tools/logger.hpp"

/* #include <iostream>
#include <sstream>
#include <string>
// DEBUG */

ArmorClassifier::ArmorClassifier(std::shared_ptr<YAML::Node> config_file_ptr, fs::path ws_dir_path) 
    : ws_dir_path(ws_dir_path) {
    
    MAX_ROI_SAVE_COUNT = (*config_file_ptr)["MAX_ROI_SAVE_COUNT"].as<int>();

    IS_ARMOR_THRESHOLD = (*config_file_ptr)["IS_ARMOR_THRESHOLD"].as<float>();
    IS_LARGE_THRESHOLD = (*config_file_ptr)["IS_LARGE_THRESHOLD"].as<float>();
    CLASSIFY_THRESHOLD = (*config_file_ptr)["CLASSIFY_THRESHOLD"].as<float>();
    INPUT_HEIGHT = (*config_file_ptr)["INPUT_HEIGHT"].as<int>();
    INPUT_WIDTH = (*config_file_ptr)["INPUT_WIDTH"].as<int>();
    filter_armor_class_mask_ =
        (*config_file_ptr)["FILTER_ARMOR_CLASS"] ? (*config_file_ptr)["FILTER_ARMOR_CLASS"].as<int>() : 0;
    fix_armor_class_ =
        (*config_file_ptr)["FIX_ARMOR_CLASS"] ? (*config_file_ptr)["FIX_ARMOR_CLASS"].as<int>() : -1;

    shm_python_classifier = std::make_shared<SharedMemoryClassifier>(config_file_ptr);
    armor_tracker = std::make_shared<ArmorTracker>(config_file_ptr);
}

cv::Mat ArmorClassifier::preprocessROI(const cv::Mat& img, const Armor& armor) {
    cv::Mat normalized;  // 将声明移到函数开始

    // 提取ROI
    cv::Mat roi_img = UnwarpUtils::unwarpQuadrilateral(img, armor.corners_expanded);
    
    
    // 图像预处理
    cv::Mat blurred;
    cv::GaussianBlur(roi_img, blurred, cv::Size(3, 3), 0);
    
    cv::Mat padded;
    int padding = 2;
    cv::copyMakeBorder(blurred, padded, padding, padding, padding, padding, 
                      cv::BORDER_REPLICATE);

    cv::Mat resized;
    cv::resize(padded, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
    
    // cv::imshow("Classifier DEBUG", resized);
    // 如果已经保存了1000张图片，直接返回处理后的图像而不保存
    if (roi_save_count >= MAX_ROI_SAVE_COUNT) {
        return resized;
    }
    
    // 保存处理后的图像（用于神经网络输入的标准化图像）
    if (roi_save_count < MAX_ROI_SAVE_COUNT) {
        // 创建保存目录
        fs::create_directories(ws_dir_path / "network_input_images");
        
        // 生成文件名（00001.jpg 格式）
        std::ostringstream filename;
        filename << std::setw(5) << std::setfill('0') << (roi_save_count.fetch_add(1) + 1) << ".jpg";
        
        fs::path full_file_path = ws_dir_path / "network_input_images/" / filename.str();
        cv::imwrite(full_file_path, resized);
        
        if (roi_save_count == MAX_ROI_SAVE_COUNT) {
            std::cout << "Reached maximum number of saved images (2000)" << std::endl;
        }
    }
    
    return resized;
}

struct alignas(64) RoiImageThreadInfo { // 64字节对齐
    const Armor* armor;
    size_t armor_index;
};

std::vector<ArmorResult> ArmorClassifier::classify(
    const cv::Mat& img, const std::vector<Armor>& armors, const cv::Point2f& ground_stable_point) {
    
    int process_armors_count = armors.size();
    process_armors_count = std::min(process_armors_count, 100);
    std::vector<cv::Mat> roi_images(process_armors_count);
    std::vector<RoiImageThreadInfo> roiImageThreadInfos(process_armors_count);
    std::vector<std::vector<float>> pytorch_results;
    
    
    armor_tracker -> preProcess(ground_stable_point);


    for (size_t i = 0; i < process_armors_count; ++i) {
        roiImageThreadInfos[i].armor = &armors[i];
        roiImageThreadInfos[i].armor_index = i;
    }
    // 并行 ROI 预处理：线程池替代 std::execution::par
    ::utils::threadPool().parallel_for_each(roiImageThreadInfos.begin(), roiImageThreadInfos.end(),
    [&](RoiImageThreadInfo& roiImageThreadInfo) {
        roi_images[roiImageThreadInfo.armor_index] = preprocessROI(img, *roiImageThreadInfo.armor);
    });
    if (process_armors_count > 0) {
        pytorch_results = shm_python_classifier->processImages(roi_images);
        if (pytorch_results.size() != static_cast<size_t>(process_armors_count)) {
            // 分类器降级（Python 端无响应/超时）：放弃本轮分类，防止越界
            return {};
        }
    }
    
    for (size_t i = 0; i < process_armors_count; ++i) {

        auto armor = armors[i];
        // 计算当前装甲板中心点
        cv::Point2f current_center = armor.center;
        
        // 获取多输出头结果
        float is_armor_probability;
        float is_large_probability;
        float reserved_3_probability;
        float reserved_4_probability;
        std::vector<float> classify_probabilities(8);
        int current_number;
        float classify_confidence;
        
        is_armor_probability = pytorch_results[i][0];
        is_large_probability = pytorch_results[i][1];
        reserved_3_probability = pytorch_results[i][2];
        reserved_4_probability = pytorch_results[i][3];
        std::copy(pytorch_results[i].begin() + 4, pytorch_results[i].begin() + 12, classify_probabilities.begin());
        
        auto classify_max_it = std::max_element(classify_probabilities.begin(), classify_probabilities.end());
        if (classify_max_it != classify_probabilities.end()) {
            classify_confidence = *classify_max_it;
            current_number = std::distance(classify_probabilities.begin(), classify_max_it);
        }

        tools::logger()->debug(
            "ArmorClassifier Debug: {:.2f} | {:.2f} | {:.2f} | {:.2f} | {:.2f} | {}",
            is_armor_probability, is_large_probability, reserved_3_probability, reserved_4_probability,
            classify_confidence, current_number);

        //is_armor_probability = 1.0; // DEBUG
        //is_large_probability = 0.0;
        //not_screen_probability = 1.0;
        //not_slant_probability = 1.0;
        //current_number = 1;
        //classify_confidence = 1.0;

        bool is_ture_armor = (is_armor_probability >= IS_ARMOR_THRESHOLD) &&
                             (classify_confidence >= CLASSIFY_THRESHOLD);

        bool not_slant = true;


        if ((((filter_armor_class_mask_ >> current_number) & 0x01) == 1)) {
            is_ture_armor = false;
        }
        if (fix_armor_class_ >= 0) {
            current_number = fix_armor_class_;
        }

        if (is_ture_armor) {
            bool is_large = is_large_probability > IS_LARGE_THRESHOLD;
            float armor_type_confidence = 1.0 - is_large_probability;
            if (is_large)
            {
                armor_type_confidence = is_large_probability;
                armor.corners = armor.corners_large;
            }
            float confidence = std::pow(
                std::abs(is_armor_probability * armor_type_confidence * classify_confidence) + 1e-6, 
                1.0 / 3.0
            );

            armor_tracker -> addArmor(armor, current_number, is_large, not_slant, confidence);

        }
    }

    std::vector<ArmorResult> results = armor_tracker -> afterProcess();

    return results;
}
