#ifndef YOLOV8_DET_RKNN_H
#define YOLOV8_DET_RKNN_H

#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "rknn_api.h"

struct YoloV8Box {
    float x1, y1, x2, y2;
    float score;
    int class_id;
};

using YoloV8BoxVec = std::vector<YoloV8Box>;

class YoloV8_det {
public:
    int batch_size = 4;
    int m_net_h = 640;
    int m_net_w = 640;
    float m_confThreshold = 0.35;
    float m_nmsThreshold = 0.45;
    int m_class_num = 10;

    int Init(const std::string& model_path);
    int Detect(const std::vector<cv::Mat>& batch_imgs, std::vector<YoloV8BoxVec>& boxes);
    void draw_result(cv::Mat& img, YoloV8BoxVec& result);
    ~YoloV8_det();

private:
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs = nullptr;
    rknn_tensor_attr* output_attrs = nullptr;
    int max_det = 300;

    void letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty);
    void NMS(YoloV8BoxVec& dets, float nmsConfidence);
    void clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h);
};

#endif // YOLOV8_DET_RKNN_H