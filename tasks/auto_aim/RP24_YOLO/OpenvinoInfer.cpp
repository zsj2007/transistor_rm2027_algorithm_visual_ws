#include "RP24_YOLO/OpenvinoInfer.h"

#include <algorithm>
#include <chrono>

OpenvinoInfer::OpenvinoInfer(string model_path_xml, string model_path_bin, string device,
                             int infer_threads, int num_streams, int input_size){
    input_shape = {1, static_cast<unsigned long>(input_size), static_cast<unsigned long>(input_size), 3};
    // 限制 CPU 推理并行度：默认会占满所有逻辑核（16）且 TBB arena 大量自旋空转。
    // 单流 + 少量线程，避免 TBB 空转成为最大 CPU 热点。
    // [实验结论] 8 线程单流实测反而更慢（~17ms vs 4 线程 ~10ms），
    // 小模型同步开销主导，单流加线程是死路。
    // 提速方向：多流(num_streams>1) + 多 InferRequest 并发推理（请求池）。
    // infer_threads 为总线程数 = 流数 × 每流线程数（如 8 = 2流 × 4线程）。
    const int n_requests = std::max(1, num_streams);
    try {
        core.set_property("CPU", ov::inference_num_threads(infer_threads));
        core.set_property("CPU", ov::num_streams(n_requests));
    } catch (const std::exception& e) {
        std::cerr << "[OpenvinoInfer] set_property warning: " << e.what() << std::endl;
    }
    model = core.read_model(model_path_xml, model_path_bin);
    // Step . Inizialize Preprocessing for the model
    ppp = new ov::preprocess::PrePostProcessor(model);
    // Specify input image format
    ppp->input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR); 
    //NHWC:batchsize,height,width,channels
    // Specify preprocess pipeline to input image without resizing
    ppp->input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255., 255., 255.});
    //  Specify model's input layout
    ppp->input().model().set_layout("NCHW");
    // Specify output results format
    ppp->output().tensor().set_element_type(ov::element::f32);
    // Embed above steps in the graph
    model = ppp->build();

    compiled_model = core.compile_model(model, device);

    // 请求池：每请求独立输出缓冲，可并发推理（多帧并行）。流数<=1 时退化为单请求串行。
    num_requests_ = n_requests;
    infer_requests_.reserve(num_requests_);
    for (int i = 0; i < num_requests_; ++i) {
        infer_requests_.push_back(compiled_model.create_infer_request());
    }
    request_busy_.assign(num_requests_, 0);
}

std::vector<Object> OpenvinoInfer::infer(const cv::Mat& img, int detect_color,
                                         double* wait_ms, double* infer_ms)
{
    // 防御：经 header 内联构造函数创建的对象也要保证有请求可用
    if (infer_requests_.empty()) {
        std::lock_guard<std::mutex> lk(request_mtx_);
        if (infer_requests_.empty()) {
            num_requests_ = 1;
            infer_requests_.push_back(compiled_model.create_infer_request());
            request_busy_.assign(1, 0);
        }
    }

    // 1. 领一个空闲 InferRequest（都忙则等待；多请求 = 多帧并行推理）
    const auto wait_t0 = std::chrono::steady_clock::now();
    ov::InferRequest req;
    int slot = -1;
    {
        std::unique_lock<std::mutex> lk(request_mtx_);
        request_cv_.wait(lk, [this, &slot]() {
            for (int i = 0; i < num_requests_; ++i) {
                if (!request_busy_[i]) {
                    slot = i;
                    request_busy_[i] = 1;
                    return true;
                }
            }
            return false;
        });
        req = infer_requests_[slot];
    }
    if (wait_ms) {
        *wait_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - wait_t0).count();
    }

    const auto infer_t0 = std::chrono::steady_clock::now();

    // 2. 推理 + 解码（全部局部变量，不写共享成员；输出缓冲属于 req，互不干扰）
    // 按输出布局自动分派解码分支：
    //   - FasterNet-P345_pose：输出 [1, 29, N]（N=8400@640 / 5376@512），channels-first
    //     （0-3 box cxcywh、4..20 类 logits、21..28 关键点；类别 0-8 蓝 9-16 红）
    //   - 旧 0526 模型：输出 [1, 25200, 22]，anchors-first
    //     （0-7 关键点、8 置信度、9-12 颜色、13-21 类别）
    std::vector<Object> objects;
    std::vector<Object> tmp_objects;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    float conf_threshold = 0.65;
    float nms_threshold  = 0.45;

    uchar* input_data = (uchar*)img.data;
    ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);
    req.set_input_tensor(input_tensor);
    req.infer();

    auto output = req.get_output_tensor(0);
    ov::Shape output_shape = output.get_shape();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());

    // 布局判定：channels-first（通道数 < 锚点数） vs anchors-first（锚点数 > 特征维）
    const bool channels_first = (output_shape.size() >= 3 && output_shape[1] < output_shape[2]);

    if (channels_first) {
        // ============ FasterNet-P345_pose（[1,29,N] channels-first）============
        const int n_anchors  = output_buffer.cols;
        const int n_channels = output_buffer.rows;
        const int nc = n_channels - 4 - 8;   // 类别数 = 29 - 4 - 8 = 17
        const int kClsStart = 4;
        const int kKptStart = 4 + nc;
        constexpr int kBlueClasses = 9;      // 类别 0-8 蓝色，9-16 红色

        for (int j = 0; j < n_anchors; ++j) {
            // 无 objectness 分支：取最大类得分（sigmoid 后）作为置信度
            float max_score = -1e9f;
            int best_cls = -1;
            for (int c = 0; c < nc; ++c) {
                float s = sigmoid(output_buffer.at<float>(kClsStart + c, j));
                if (s > max_score) { max_score = s; best_cls = c; }
            }
            if (best_cls < 0 || max_score < conf_threshold) continue;

            // 敌方颜色过滤：类别前缀 B(0-8)/R(9-16)；detect_color: 0=敌方蓝 1=敌方红
            const bool is_blue = (best_cls < kBlueClasses);
            if (detect_color == 0 && !is_blue) continue;
            if (detect_color == 1 && is_blue)  continue;

            const float cx = output_buffer.at<float>(0, j);
            const float cy = output_buffer.at<float>(1, j);
            const float w  = output_buffer.at<float>(2, j);
            const float h  = output_buffer.at<float>(3, j);

            Object obj;
            obj.prob  = max_score;
            obj.color = is_blue ? 1 : 0;   // 沿用旧约定：blue:1, red:0
            obj.label = best_cls;
            for (int k = 0; k < 8; ++k) {
                obj.landmarks[k] = output_buffer.at<float>(kKptStart + k, j);
            }
            // 关键点顺序假设与旧模型一致：[左灯条上, 左灯条下, 右灯条上, 右灯条下]
            obj.length = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[2],
                                              obj.landmarks[1] - obj.landmarks[3]));
            obj.width  = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[4],
                                              obj.landmarks[1] - obj.landmarks[5]));
            obj.ratio  = obj.length / obj.width;

            // box 输出 cxcywh（输入空间）→ 左上角 + 宽高
            cv::Rect rect((int)(cx - w * 0.5f), (int)(cy - h * 0.5f), (int)w, (int)h);
            obj.rect = rect;
            objects.push_back(obj);
            boxes.push_back(rect);
            confidences.push_back(max_score);
        }
    } else {
        // ============ 旧 0526 模型（[1,25200,22] anchors-first）============
        for (int i = 0; i < output_buffer.rows; i++) {
            // 通过置信度阈值筛选
            float confidence = output_buffer.at<float>(i, 8);
            confidence = sigmoid(confidence);
            if (confidence < conf_threshold) continue;

            // 颜色和类别独热向量
            cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);  //color
            cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22); //num
            cv::Point class_id, color_id;
            double score_color, score_num;
            cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
            cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
            // None 或者 Purple 丢掉
            if (color_id.x == 2 || color_id.x == 3) continue;
            else if (detect_color == 0 && color_id.x == 1) continue;   // detect blue
            else if (detect_color == 1 && color_id.x == 0) continue;   // detect red

            Object obj;
            obj.prob = confidence;
            obj.color = color_id.x;
            obj.label = class_id.x;
            obj.landmarks[0] = output_buffer.at<float>(i, 0);
            obj.landmarks[1] = output_buffer.at<float>(i, 1);
            obj.landmarks[2] = output_buffer.at<float>(i, 2);
            obj.landmarks[3] = output_buffer.at<float>(i, 3);
            obj.landmarks[4] = output_buffer.at<float>(i, 4);
            obj.landmarks[5] = output_buffer.at<float>(i, 5);
            obj.landmarks[6] = output_buffer.at<float>(i, 6);
            obj.landmarks[7] = output_buffer.at<float>(i, 7);
            obj.length = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[6])-cv::Point2f(obj.landmarks[1]-obj.landmarks[7]));
            obj.width = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[2])-cv::Point2f(obj.landmarks[1]-obj.landmarks[3]));
            obj.ratio = obj.length / obj.width;

            // landmarks 为左上逆时针：用 4 点包围盒做矩形
            std::vector<cv::Point2f> points;
            points.push_back(cv::Point2f(obj.landmarks[0], obj.landmarks[1]));
            points.push_back(cv::Point2f(obj.landmarks[6], obj.landmarks[7]));
            points.push_back(cv::Point2f(obj.landmarks[4], obj.landmarks[5]));
            points.push_back(cv::Point2f(obj.landmarks[2], obj.landmarks[3]));
            float min_x = points[0].x, max_x = points[0].x;
            float min_y = points[0].y, max_y = points[0].y;
            for (size_t p = 1; p < points.size(); ++p) {
                min_x = std::min(min_x, points[p].x);
                max_x = std::max(max_x, points[p].x);
                min_y = std::min(min_y, points[p].y);
                max_y = std::max(max_y, points[p].y);
            }
            cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
            obj.rect = rect;
            objects.push_back(obj);
            boxes.push_back(rect);
            confidences.push_back(score_num);
        }
    }

    // NMS（输入空间坐标）
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
    for (int valid_index : indices) {
        if (valid_index < (int)objects.size()) {
            tmp_objects.push_back(objects[valid_index]);
        }
    }

    // 3. 归还请求
    {
        std::lock_guard<std::mutex> lk(request_mtx_);
        request_busy_[slot] = 0;
    }
    request_cv_.notify_one();

    if (infer_ms) {
        *infer_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - infer_t0).count();
    }

    return tmp_objects;
}
