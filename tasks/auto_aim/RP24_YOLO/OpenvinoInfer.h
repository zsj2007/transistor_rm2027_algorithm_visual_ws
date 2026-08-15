//
// Created by nuc on 23-5-7.
//

#ifndef OPENVINO_TEST_OPENVINOINFER_H
#define OPENVINO_TEST_OPENVINOINFER_H

#include <condition_variable>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>

//用前先根据模型修改好要修改的地方

using namespace cv;
using namespace std;


struct Object
{
    cv::Rect_<float> rect; //
    float landmarks[8]; //4个关键点
    int label;
    float prob;
    int color;      //blue:1 , red:0
    double length;
    double width;
    double ratio;   //length s/ width
};

class OpenvinoInfer {
public:
    // 模型输入尺寸：仅旧版内联构造函数使用（legacy），
    // 运行时尺寸由构造参数 input_size 决定（配置 RP24_YOLO_input_size，全仓库唯一真源）
    const int IMAGE_HEIGHT = 640;
    const int IMAGE_WIDTH = 640;
    double ans;
    std::shared_ptr<ov::Model> model;
    ov::Core core;
    ov::preprocess::PrePostProcessor *ppp;
    ov::CompiledModel compiled_model;
    ov::Shape input_shape;

    OpenvinoInfer(){}
    // infer_threads: CPU 推理总线程数（= 流数 × 每流线程数，如 8 = 2流 × 4线程）；
    // num_streams: 执行流数 = 并发推理请求数（1 = 单请求串行）；
    // input_size: 模型输入边长（唯一尺寸来源，配置 RP24_YOLO_input_size）
    OpenvinoInfer(string model_path_xml, string model_path_bin, string device,
                  int infer_threads = 4, int num_streams = 1, int input_size = 640);
    // 线程安全：从请求池领一个空闲 InferRequest，推理+解码后返回结果，支持多帧并行。
    // wait_ms / infer_ms（可选输出）：等待空闲请求耗时 / 纯推理+解码耗时。
    std::vector<Object> infer(const cv::Mat& img, int detect_color,
                              double* wait_ms = nullptr, double* infer_ms = nullptr);

    // ---- 多帧并行推理请求池（每请求独立输出缓冲，可并发） ----
    std::vector<ov::InferRequest> infer_requests_;
    std::vector<char> request_busy_;      // 1 = 占用中，受 request_mtx_ 保护
    std::mutex request_mtx_;
    std::condition_variable request_cv_;
    int num_requests_ = 1;

    OpenvinoInfer(string model_path, string device){
        input_shape = {1, 1, static_cast<unsigned long>(IMAGE_HEIGHT), static_cast<unsigned long>(IMAGE_WIDTH)};
        // input_shape = { 1, static_cast<unsigned long>(IMAGE_HEIGHT), static_cast<unsigned long>(IMAGE_WIDTH),1};
        model = core.read_model(model_path);
        // Step . Inizialize Preprocessing for the model
        ppp = new ov::preprocess::PrePostProcessor(model);
        // Specify input image format
        ppp->input().tensor().set_shape(input_shape).set_element_type(ov::element::u8).set_layout("NHWC");
        ppp->input().preprocess().resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);

        //  Specify model's input layout
        ppp->input().model().set_layout("NHWC");
        // Specify output results format
        ppp->output().tensor().set_element_type(ov::element::f32);
        // Embed above steps in the graph
        model = ppp->build();

        compiled_model = core.compile_model(model, device);
    }

    void infer(Mat img, float* detections){
        // Step 3. Read input image
        // resize image
        resize(img,img,Size(IMAGE_WIDTH,IMAGE_HEIGHT));
        if(img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);


        // Step 5. Create tensor from image
        int rows = img.rows;
        int cols = img.cols;
        uchar* input_data = (uchar *)img.data; // 创建一个新的float数组

        ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);


        // Step 6. Create an infer request for model inference
        ov::InferRequest infer_request = compiled_model.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();


        //Step 7. Retrieve inference results
        const ov::Tensor &output_tensor = infer_request.get_output_tensor();
        ov::Shape output_shape = output_tensor.get_shape();
        copy(output_tensor.data<float>(), output_tensor.data<float>() + 9, detections);
    }
    double sigmoid(double x) {
        if(x>0)
            return 1.0 / (1.0 + exp(-x));
        else
            return exp(x) / (1.0 + exp(x));
    }

        double cal_iou(const cv::Rect& r1, const cv::Rect& r2)
    {
        float x_left = std::fmax(r1.x, r2.x);
        float y_top = std::fmax(r1.y, r2.y); 
        float x_right = std::fmin(r1.x + r1.width, r2.x + r2.width);
        float y_bottom = std::fmin(r1.y + r1.height, r2.y + r2.height);

        if (x_right < x_left || y_bottom < y_top) {
            return 0.0; 
        }

        double in_area = (x_right - x_left) * (y_bottom - y_top);
        double un_area = r1.area() + r2.area() - in_area; 

        return in_area / un_area;
    }

    double meaning(float x, int len){
        if(len == 0) ans = x;
        else{
            ans = (len * ans + x) / (len+1);
        }
        return ans;
    }

    ~OpenvinoInfer(){
        delete ppp;
    }
};

#endif //OPENVINO_TEST_OPENVINOINFER_H
