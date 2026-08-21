// ArmorSolver.cpp
#include "3d_processing/ArmorSolver.h"

#include "tools/logger.hpp"
#include <fstream>
#include <sstream>
#include <mutex>
#include <string>
#include <iomanip>
#include <cctype>   // std::isdigit
#include <cmath>

// 取得当前源文件所在目录（__FILE__ 是编译期绝对/相对路径）
static inline std::string __src_dir__() {
  std::string p = __FILE__;
  size_t pos = p.find_last_of("/\\");
  return (pos == std::string::npos) ? std::string(".") : p.substr(0, pos);
}

namespace {
std::mutex      g_ypr_mtx;
std::ofstream   g_ypr_ofs;
std::size_t     g_line_no = 0;      // 当前行对应的 frame 编号（会自动从现有文件续上）
bool            g_inited  = false;
std::string     g_log_path;

// 从已存在的文件里读到“最后一个 frame 编号”
static std::size_t read_last_index(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) return 0;
  std::size_t last = 0;
  std::string line;
  while (std::getline(ifs, line)) {
    // 行形如：frame123: pnp(...), ba(...)
    if (line.rfind("frame", 0) == 0) {
      std::size_t i = 5; // 跳过 "frame"
      std::size_t num = 0; bool any = false;
      while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i]))) {
        any = true;
        num = num * 10 + (line[i] - '0');
        ++i;
      }
      if (any) last = num;
    }
  }
  return last;
}

// 确保日志就绪：定位路径（同目录优先），读最后编号，再以追加模式打开
static void ensure_log_ready() {
  if (g_inited) return;
  std::lock_guard<std::mutex> lk(g_ypr_mtx);

  if (g_inited) return; // 双重检查
  // 1) 同目录优先
  std::string path_same_dir = __src_dir__() + "/ypr_log.txt";
  // 先读最后编号
  std::size_t last_idx = read_last_index(path_same_dir);

  g_ypr_ofs.open(path_same_dir, std::ios::out | std::ios::app);
  if (g_ypr_ofs.good()) {
    g_log_path = path_same_dir;
    g_line_no  = last_idx;
    g_inited   = true;
    return;
  }

  // 2) 回退到运行目录
  std::string path_run_dir = "ypr_log.txt";
  last_idx = read_last_index(path_run_dir);
  g_ypr_ofs.clear();
  g_ypr_ofs.open(path_run_dir, std::ios::out | std::ios::app);
  if (g_ypr_ofs.good()) {
    g_log_path = path_run_dir;
    g_line_no  = last_idx;
    g_inited   = true;
    return;
  }

  // 3) 仍失败就保持未初始化（不会崩，只是写不进去）
}

// 追加一行：frameN: pnp(p,y,r), ba(p,y,r)
// 这里默认输出**弧度**；若想用“度”，把 val() 改成返回 x*180/M_PI。
static inline void append_ypr(double pnp_pitch, double pnp_yaw, double pnp_roll,
                              double  ba_pitch, double  ba_yaw, double  ba_roll) {
  if (!g_inited) ensure_log_ready();
  std::lock_guard<std::mutex> lk(g_ypr_mtx);
  if (!g_ypr_ofs.is_open()) return; // 打不开就静默返回（不影响主流程）

  auto val = [](double x){ return x; }; // 输出弧度；若需“度”，用：return x * 180.0 / M_PI;

  ++g_line_no; // 自动：1,2,3,…（会从已有文件的最后编号继续递增）
  g_ypr_ofs << "frame" << g_line_no << ": "
            << "pnp(" << std::fixed << std::setprecision(2)
            << val(pnp_pitch) << ", " << val(pnp_yaw) << ", " << val(pnp_roll) << "), "
            << "ba("  << val(ba_pitch)  << ", " << val(ba_yaw)  << ", " << val(ba_roll)  << ")\n";
  g_ypr_ofs.flush();
}
} // namespace

double getYawFromRvec(const cv::Mat& rvec) {
    if (rvec.empty()) return 0.0;
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat); // 从旋转向量得到旋转矩阵

    // 根据OpenCV相机坐标系从旋转矩阵直接计算Yaw角
    // Yaw是绕Y轴的旋转，一个稳健的计算方法如下：
    // yaw = atan2(-R(2,0), sqrt(R(0,0)^2 + R(1,0)^2))
    double yaw = std::atan2(-rmat.at<double>(2, 0),
                           std::sqrt(std::pow(rmat.at<double>(0, 0), 2) +
                                     std::pow(rmat.at<double>(1, 0), 2)));
    return yaw;
}


std::vector<double> getNormalYawPitchRollFromRvec(const cv::Mat& rvec) {
    std::vector<double> result = {0.0, 0.0, 0.0};
    if (rvec.empty()) return result;
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat); // 从旋转向量得到旋转矩阵

    // 此处获得的欧拉角为RestFrame中定义的坐标系的欧拉角，即：
    // x：向右，y：向前，z：向上
    // yaw：绕z轴 从上方看逆时针 x轴转向y轴，pitch：绕x轴 抬头 y轴转向z轴，roll：绕y轴 从画面看顺时针 z轴转向x轴
    double yaw, pitch, roll;
    pitch = std::asin(-rmat.at<double>(1, 2));
    const float epsilon = 1e-6;
    if (std::abs(std::cos(pitch)) > epsilon) {
        yaw = std::atan2(-rmat.at<double>(0, 2), rmat.at<double>(2, 2));
        roll = std::atan2(rmat.at<double>(1, 0), rmat.at<double>(1, 1));
    } else {
        roll = 0.0f;
        yaw = std::atan2(rmat.at<double>(2, 0), rmat.at<double>(0, 0));
    }
    result = {yaw, pitch, roll};
    return result;
}


void ArmorSolver::initCameraMatrix(std::shared_ptr<YAML::Node> config_file_ptr) {
    const YAML::Node& camera_matrix_Node = (*config_file_ptr)["camera_matrix"];
    tools::logger()->info(
        "camera_matrix_config: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]",
        camera_matrix_Node[0][0].as<double>(), camera_matrix_Node[0][1].as<double>(), camera_matrix_Node[0][2].as<double>(),
        camera_matrix_Node[1][0].as<double>(), camera_matrix_Node[1][1].as<double>(), camera_matrix_Node[1][2].as<double>(),
        camera_matrix_Node[2][0].as<double>(), camera_matrix_Node[2][1].as<double>(), camera_matrix_Node[2][2].as<double>());
    // 相机内参矩阵
    camera_matrix = (cv::Mat_<double>(3, 3) << 
        camera_matrix_Node[0][0].as<double>(), camera_matrix_Node[0][1].as<double>(), camera_matrix_Node[0][2].as<double>(), 
        camera_matrix_Node[1][0].as<double>(), camera_matrix_Node[1][1].as<double>(), camera_matrix_Node[1][2].as<double>(), 
        camera_matrix_Node[2][0].as<double>(), camera_matrix_Node[2][1].as<double>(), camera_matrix_Node[2][2].as<double>());
    
    const YAML::Node& dist_coeffs_Node = (*config_file_ptr)["dist_coeffs"];
    tools::logger()->info(
        "dist_coeffs_config: {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}",
        dist_coeffs_Node[0].as<double>(), dist_coeffs_Node[1].as<double>(), dist_coeffs_Node[2].as<double>(),
        dist_coeffs_Node[3].as<double>(), dist_coeffs_Node[4].as<double>());
    // 畸变系数
    dist_coeffs = (cv::Mat_<double>(1, 5) << 
        dist_coeffs_Node[0].as<double>(), dist_coeffs_Node[1].as<double>(), dist_coeffs_Node[2].as<double>(), 
        dist_coeffs_Node[3].as<double>(), dist_coeffs_Node[4].as<double>());
    
    // 初始化ba指针
    std::array<double,9> Karr = {
    camera_matrix.at<double>(0,0), camera_matrix.at<double>(0,1), camera_matrix.at<double>(0,2),
    camera_matrix.at<double>(1,0), camera_matrix.at<double>(1,1), camera_matrix.at<double>(1,2),
    camera_matrix.at<double>(2,0), camera_matrix.at<double>(2,1), camera_matrix.at<double>(2,2)};
    std::vector<double> distv;
    distv.reserve(dist_coeffs.total());
     for (int i = 0; i < dist_coeffs.total(); ++i)
    distv.push_back(dist_coeffs.at<double>(i));
    ba_ = std::make_unique<fyt::auto_aim::BaSolver>(Karr, distv);
}

void ArmorSolver::initArmorPoints() {
    // 使用小装甲板尺寸初始化（因为在初始化阶段我们还不知道具体是哪种装甲板）
    const float HALF_WIDTH = ArmorConstants::SMALL_ARMOR_WIDTH / 2.0f;   // 67.5mm
    const float HALF_HEIGHT = ArmorConstants::SMALL_ARMOR_HEIGHT / 2.0f; // 62.5mm
    
    armor_points_3d = {
        cv::Point3f(-HALF_WIDTH, -HALF_HEIGHT, 0.0f),  // 左上
        cv::Point3f(-HALF_WIDTH, HALF_HEIGHT, 0.0f),   // 右上
        cv::Point3f(HALF_WIDTH, HALF_HEIGHT, 0.0f),    // 右下
        cv::Point3f(HALF_WIDTH, -HALF_HEIGHT, 0.0f)    // 左下
    };
}

cv::Point2f ArmorSolver::project3DToPixel(const cv::Point3f& world_point) const {
    // 确保相机参数已初始化0
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        throw std::runtime_error("Camera parameters not initialized!");
    }

    // 从枪口坐标系换回相机坐标系
    cv::Point3f cam_world_point;
    cam_world_point.x = world_point.x - delta_x_;
    cam_world_point.y = world_point.y + delta_z_;
    cam_world_point.z = world_point.z - delta_y_;

    // 将3D点转换为OpenCV输入格式
    std::vector<cv::Point3f> object_points = {cam_world_point};
    std::vector<cv::Point2f> image_points;

    // 使用solvePnP投影
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // 假设无旋转
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // 假设无平移
    
    // 直接使用projectPoints进行投影
    cv::projectPoints(object_points, rvec, tvec, 
                     camera_matrix, dist_coeffs, 
                     image_points);

    return image_points[0];
}

// 修改solveArmor函数实现
AimResult ArmorSolver::solveArmor(const ArmorResult& armor_result, const double last_pitch_rad_, const double last_yaw_rad_) const {
    
    AimResult result;
    result.valid = false;
    
    const Armor armor = armor_result.armor;
    int number = armor_result.number;
    
    // 计算相机到水平系的旋转矩阵
    Eigen::Matrix3d R_imu_camera = ba_ -> RPYTorotationMatrix(Eigen::Vector3d(last_pitch_rad_, last_yaw_rad_, 0));

    // Eigen::Matrix3d R_imu_camera = ba_ -> RPYTorotationMatrix(Eigen::Vector3d(2, 1, 0));
    // auto rpy_wc = ba_ -> rotationMatrixToRPY(R_imu_camera);
    // RCLCPP_DEBUG(logger_p, "\n YPR WC: (%.2f, %.2f, %.2f)" , rpy_wc[0], rpy_wc[1], rpy_wc[2]);
    tools::logger()->debug("camera yaw&pitch: ({:.2f}, {:.2f})", last_pitch_rad_, last_yaw_rad_);

    try {
        bool is_large_armor = armor_result.is_large;
        
        // float half_width = is_large_armor ? 
        //     ArmorConstants::LARGE_ARMOR_WIDTH / 2.0f :
        //     ArmorConstants::SMALL_ARMOR_WIDTH / 2.0f;
            
        // float half_height = is_large_armor ? 
        //     ArmorConstants::LARGE_ARMOR_HEIGHT / 2.0f :
        //     ArmorConstants::SMALL_ARMOR_HEIGHT / 2.0f;

        // 改为使用灯条顶点
        float half_width = is_large_armor ? 
            ArmorConstants::LARGE_ARMOR_LIGHT_DISTANCE / 2.0f :
            ArmorConstants::SMALL_ARMOR_LIGHT_DISTANCE / 2.0f;
        float half_height = ArmorConstants::PNP_LIGHT_HEIGHT / 2.0f;
            
        std::vector<cv::Point3f> armor_points_3d = {
            cv::Point3f(-half_width, -half_height, 0.0f),
            cv::Point3f(-half_width, half_height, 0.0f),
            cv::Point3f(half_width, half_height, 0.0f),
            cv::Point3f(half_width, -half_height, 0.0f)
        };

        cv::Mat rvec, tvec;
        bool solve_success = cv::solvePnP(armor_points_3d, armor_result.armor.light_bar_corners, // armor_result.corners, 
                                        camera_matrix, dist_coeffs, 
                                        rvec, tvec, false, cv::SOLVEPNP_IPPE);

        if (solve_success) {

            tools::logger()->debug("solvePnP success");

            result.yaw = getYawFromRvec(rvec); // <<--- 计算并填充yaw
            tools::logger()->debug("yaw getfromRvec: {:.2f}", result.yaw);

            result.normal_euler_angles = getNormalYawPitchRollFromRvec(rvec);
            tools::logger()->debug(
                "NormalYawPitchRoll: ({:.2f}, {:.2f}, {:.2f})",
                result.normal_euler_angles[0], result.normal_euler_angles[1], result.normal_euler_angles[2]);
             
            //转化为ba需要的参数
            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);
            Eigen::Matrix3d R = fyt::utils::cvToEigen(rmat);

            // 现打印ba优化之前的参数
            auto rpy_before = ba_ -> rotationMatrixToRPY(R);

            tools::logger()->debug(
                "HUHU YPR PNP: ({:.2f}, {:.2f}, {:.2f})",
                result.normal_euler_angles[0], result.normal_euler_angles[1], result.normal_euler_angles[2]);
            tools::logger()->debug("YPR PNP: ({:.2f}, {:.2f}, {:.2f})", rpy_before[0], rpy_before[1], rpy_before[2]);
            // RCLCPP_DEBUG(logger_p, "pitch before ba: %.2f" , rpy_before[0]);
            // RCLCPP_DEBUG(logger_p, "yaw before ba: %.2f" , rpy_before[1]);
            // RCLCPP_DEBUG(logger_p, "roll before ba: %.2f" , rpy_before[2]);

            Eigen::Vector3d t = fyt::utils::cvToEigen(tvec);

            R = ba_-> solveBa(armor_result, t, R, R_imu_camera);
            // 将优化后的旋转矩阵转化为RPY
            auto rpy = ba_ -> rotationMatrixToRPY(R);

            // 打印优化之后的参数
            tools::logger()->debug("RPY BA: ({:.2f}, {:.2f}, {:.2f})", rpy[0], rpy[1], rpy[2]);

            // append_ypr(result.normal_euler_angles[0], result.normal_euler_angles[1], result.normal_euler_angles[2],
            // rpy[0],  rpy[1],  rpy[2]);
            
            // 填充所有的result
            result.valid = true;
            //result.yaw = rpy[2]; // <<--- 填充ba优化后的yaw
            result.position = cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            result.rvec = rvec.clone(); // <<--- 填充rvec

            result.ba_global_ypr = {rpy[0], rpy[1], rpy[2]};

        } else {
            std::cerr << "PnP solve failed!" << std::endl;
            return result;
        }
        
        // 设置位置信息（相机坐标系下的三维位置）
        result.position = cv::Point3f(tvec.at<double>(0),
                                    tvec.at<double>(1),
                                    tvec.at<double>(2));
        
        // 计算距离
        result.distance = cv::norm(result.position);

        // 修正为枪口坐标系
        result.position.x += delta_x_;
        result.position.y -= delta_z_;
        result.position.z += delta_y_;
        
        // 标记解算成功
        result.valid = true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in solveArmor: " << e.what() << std::endl;
    }
    
    return result;
}

double ArmorSolver::getMaxFOVAngle(int width, int height) const {
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        tools::logger()->error("Camera parameters not initialized!");
        return -1.0;
    }

    // 检查缓存
    std::string cache_key = makeCacheKey(width, height);
    auto it = fov_cache_.find(cache_key);
    if (it != fov_cache_.end()) {
        return it->second;
    }

    // 准备四个角点（可进一步在边上增加采样点，这里以角点为例）
    std::vector<cv::Point2f> corners;
    corners.emplace_back(0.0f, 0.0f);                      // 左上
    corners.emplace_back(static_cast<float>(width - 1), 0.0f); // 右上
    corners.emplace_back(0.0f, static_cast<float>(height - 1)); // 左下
    corners.emplace_back(static_cast<float>(width - 1), static_cast<float>(height - 1)); // 右下

    // 可选：增加四条边中点，防止切向畸变导致最大夹角不在角点
    // 为提高精度（尤其畸变严重时），可取消注释以下代码
    // /*
    corners.emplace_back(static_cast<float>(width/2), 0.0f);
    corners.emplace_back(static_cast<float>(width/2), static_cast<float>(height-1));
    corners.emplace_back(0.0f, static_cast<float>(height/2));
    corners.emplace_back(static_cast<float>(width-1), static_cast<float>(height/2));
    // */

    // 去畸变，得到归一化平面上的理想坐标（射线方向向量，z=1）
    std::vector<cv::Point2f> normalized_points;
    // 关键：不传入新的相机矩阵 P，输出即为归一化坐标
    cv::undistortPoints(corners, normalized_points, camera_matrix, dist_coeffs,
                        cv::noArray(), cv::noArray()); // 注意：第四个参数为空，输出直接是归一化坐标

    if (normalized_points.size() != corners.size()) {
        tools::logger()->error("undistortPoints failed, size mismatch");
        return -1.0;
    }

    // 将归一化坐标转换为 3D 射线方向 (x, y, 1)
    std::vector<cv::Point3f> rays;
    rays.reserve(normalized_points.size());
    for (const auto& p : normalized_points) {
        rays.emplace_back(p.x, p.y, 1.0f);
        std::cout << "Normalized point: (" << p.x << ", " << p.y << ") -> Ray: (" << p.x << ", " << p.y << ", 1)" << std::endl;
    }

    // 计算所有点对之间的最大夹角
    double max_angle = 0.0;
    const size_t n = rays.size();
    for (size_t i = 0; i < n; ++i) {
        const cv::Point3f& d1 = rays[i];
        double norm1 = std::sqrt(d1.x*d1.x + d1.y*d1.y + 1.0); // 注意 z=1
        for (size_t j = i+1; j < n; ++j) {
            const cv::Point3f& d2 = rays[j];
            double dot = d1.x*d2.x + d1.y*d2.y + 1.0; // d1.z=d2.z=1
            double norm2 = std::sqrt(d2.x*d2.x + d2.y*d2.y + 1.0);
            double cos_val = dot / (norm1 * norm2);
            // 避免数值误差
            if (cos_val > 1.0) cos_val = 1.0;
            if (cos_val < -1.0) cos_val = -1.0;
            double angle = std::acos(cos_val);
            if (angle > max_angle) max_angle = angle;
        }
    }

    // 缓存结果
    fov_cache_[cache_key] = max_angle;
    tools::logger()->info(
        "Computed max FOV angle: {:.3f} deg ({:.4f} rad) for resolution {}x{}",
        max_angle * 180.0 / M_PI, max_angle, width, height);
    return max_angle;
}
/*最后，把所有相关代码压缩成这一张图

你现在如果只想抓住项目里的 PnP 核心思路，记这张就够了：

                    YAML
                     │
           ┌─────────┴──────────┐
           ↓                    ↓
     camera_matrix          dist_coeffs
           │                    │
           └─────────┬──────────┘
                     │
Stage1               │
   │                 │
   │ light_bar_corners
   ▼
装甲板图像中的 4 个 2D 点
                     │
真实大小              │
   │                  │
large/small armor     │
   ↓                  │
构造 4 个真实 3D 点 ──┘
          │
          ▼
   ┌──────────────┐
   │ cv::solvePnP │
   │     IPPE     │
   └──────┬───────┘
          │
     ┌────┴─────┐
     ▼          ▼
   tvec        rvec
     │           │
     │       Rodrigues
     │           ↓
     │           R
     │           │
     │       提取 YPR
     │           │
     │       + IMU姿态
     │           │
     │          BA
     │           ↓
     │      优化后姿态
     │
     ↓
相机坐标3D位置
     ↓
计算 distance
     ↓
相机→枪口偏移
     ↓
result.position
     ↓
Stage2 RestFrame
     ↓
世界/稳定坐标
     ↓
Tracker / Predictor
一句话概括

你们这套 PnP 的主要思路就是：用“已知真实尺寸的灯条四点”和“图像中检测到的灯条四点”，结合相机标定参数，通过 solvePnP(IPPE) 求出装甲板相对于相机的 3D 位置 tvec 和姿态 rvec；位置再做枪口偏移和世界坐标转换，姿态则进一步结合 IMU 并通过 BA 优化，最后交给追踪和预测模块。

其中你当前最值得抓牢的不是 BA 的数学细节，而是 3D真实点 + 2D检测点 + 相机内参 → rvec/tvec 这条主链。后面的代码基本都是围绕 PnP 输出做坐标系变换、姿态优化和结果使用。*/
