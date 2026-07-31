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

    cv::Mat drawResults(const cv::Mat& img, const std::vector<RknnDetection>& detections);

private:
    rknn_context ctx_ = 0;
    int net_w_ = 640;
    int net_h_ = 640;
    int num_classes_ = 10;
    float conf_threshold_ = 0.35f;
    float nms_threshold_ = 0.45f;
    int max_det_ = 300;

    void letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty);
    void nms(std::vector<RknnDetection>& dets, float nmsConfidence);
};

#endif // RKNN_TEST_H