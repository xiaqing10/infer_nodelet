#ifndef YOLOV8_DET_RKNN_H
#define YOLOV8_DET_RKNN_H

#include <iostream>
#include <vector>
#include <string>
#include <set>
#include "opencv2/opencv.hpp"
#include "rknn_api.h"

#define OBJ_NUMB_MAX_SIZE 256

struct YoloV8Box {
    float x1, y1, x2, y2;
    float score;
    int class_id;
};

using YoloV8BoxVec = std::vector<YoloV8Box>;

typedef struct _BOX_RECT {
    int left;
    int right;
    int top;
    int bottom;
} BOX_RECT;

typedef struct _detect_result_t {
    int class_id;
    float prop;
    BOX_RECT box;
    char name[24];
} detect_result_t;

typedef struct _detect_result_group_t {
    int count;
    detect_result_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_group_t;

class YoloV8_det {
public:
    int m_net_h = 640;
    int m_net_w = 640;
    float m_confThreshold = 0.35;
    float m_nmsThreshold = 0.45;
    int m_class_num = 10;
    bool use_sigmoid_ = false;
    bool use_letterbox_ = false;

    int Init(const std::string& model_path);
    int Detect(const cv::Mat& img, YoloV8BoxVec& boxes);
    void draw_result(cv::Mat& img, YoloV8BoxVec& result);
    ~YoloV8_det();

private:
    rknn_context ctx = 0;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs = nullptr;
    rknn_tensor_attr* output_attrs = nullptr;
    int max_det = 300;
    int num_outputs_ = 1;

    int pad_left_ = 0;
    int pad_top_ = 0;
    float letterbox_scale_ = 1.0f;

    static constexpr int anchor0[6] = {10, 13, 16, 30, 33, 23};
    static constexpr int anchor1[6] = {30, 61, 62, 45, 59, 119};
    static constexpr int anchor2[6] = {116, 90, 156, 198, 373, 326};

    void letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty);
    void NMS(YoloV8BoxVec& dets, float nmsConfidence);
    void clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h);

    static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
    static float apply_activation(float x, bool use_sig) { return use_sig ? sigmoid(x) : x; }
    static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale);
    static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale);

    int process(int8_t* input, const int* anchor, int grid_h, int grid_w,
                int stride, std::vector<float>& boxes, std::vector<float>& objProbs,
                std::vector<int>& classId, float threshold, int32_t zp, float scale,
                int num_classes, bool use_sigmoid);
    int nms(int validCount, std::vector<float>& outputLocations, std::vector<int>& classIds,
            std::vector<int>& order, int filterId, float threshold);
    int quick_sort_indice_inverse(std::vector<float>& input, int left, int right, std::vector<int>& indices);
    int post_process(int8_t* input0, int8_t* input1, int8_t* input2,
                     int model_in_h, int model_in_w,
                     int num_classes, float conf_threshold, float nms_threshold,
                     BOX_RECT pads, float scale_w, float scale_h,
                     std::vector<int32_t>& qnt_zps, std::vector<float>& qnt_scales,
                     detect_result_group_t* group, bool use_sigmoid);
};

#endif