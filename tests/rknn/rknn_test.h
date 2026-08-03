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
    int getNumOutputs() const { return num_outputs_; }

private:
    rknn_context ctx_ = 0;
    int net_w_ = 640;
    int net_h_ = 640;
    int num_classes_ = 10;
    float conf_threshold_ = 0.45f;
    float nms_threshold_ = 0.45f;
    int max_det_ = 300;
    bool use_yolov5_ = false;
    int num_outputs_ = 1;

    void letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty);
    void nms(std::vector<RknnDetection>& dets, float nmsConfidence);

    std::vector<RknnDetection> decodeYoloV8(const float* data, int box_num, int nout,
                                             float scale, int tx, int ty, int img_w, int img_h);
};

#endif // RKNN_TEST_H