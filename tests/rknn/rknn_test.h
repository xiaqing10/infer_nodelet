#ifndef RKNN_TEST_H
#define RKNN_TEST_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "rknn_api.h"

struct RknnDetection {
    int class_id;
    float confidence;
    cv::Rect box;
};

class RknnTest {
public:
    RknnTest(const std::string& model_path, int num_classes = 10);
    ~RknnTest();

    std::vector<RknnDetection> runInference(const cv::Mat& img);
    void runInferenceOnly(const cv::Mat& img);
    cv::Mat drawResults(const cv::Mat& img, const std::vector<RknnDetection>& detections);

    void setYoloV5(bool v5) { use_yolov5_ = v5; }
    void setUseLetterbox(bool lb) { use_letterbox_ = lb; }
    void setUseSigmoid(bool sg) { use_sigmoid_ = sg; }
    int getNumOutputs() const { return num_outputs_; }
    void setCoreMask(const std::string& mask_str);

private:
    rknn_context ctx_ = 0;
    int net_w_ = 640;
    int net_h_ = 640;
    int num_classes_ = 10;
    float conf_threshold_ = 0.45f;
    float nms_threshold_ = 0.45f;
    int max_det_ = 300;
    bool use_yolov5_ = false;
    bool use_letterbox_ = false;
    bool use_sigmoid_ = false;
    int num_outputs_ = 1;
    int pad_left_ = 0;
    int pad_top_ = 0;
    float letterbox_scale_ = 1.0f;
    rknn_core_mask core_mask_ = RKNN_NPU_CORE_AUTO;
    bool is_quant_ = false;
    int output_per_branch_ = 0;
    std::vector<rknn_tensor_attr> output_attrs_;

    void letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty);
    void nms(std::vector<RknnDetection>& dets, float nmsConfidence);
    static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

    std::vector<RknnDetection> decodeYoloV8Merged(const float* data, int box_num, int nout,
                                                   float scale, int tx, int ty, int img_w, int img_h);
    std::vector<RknnDetection> decodeYoloV8DFL(rknn_output* outputs,
                                                float scale_x, float scale_y,
                                                int img_w, int img_h);
};

#endif // RKNN_TEST_H