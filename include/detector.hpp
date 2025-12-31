//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//

#ifndef YOLOV8_DET_H
#define YOLOV8_DET_H

#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "utils.hpp"
// Define USE_OPENCV for enabling OPENCV related funtions in bm_wrapper.hpp
#define USE_OPENCV 1
#include "bm_wrapper.hpp"
#define DEBUG 0

struct YoloV8Box {
    float x1, y1, x2, y2;
    float score;
    int class_id;
};

typedef struct DetectorRetData{
    int label;
    float confidence;
    int xmin;
    int ymin;
    int xmax;
    int ymax;

    cv::Mat          boxMask;
}DetectorRetData;

typedef struct DetectorRetDatas{
    std::vector<DetectorRetData> data;
}DetectorRetDatas;


using YoloV8BoxVec = std::vector<YoloV8Box>;

class YoloV8_det {
    void *bmrt = NULL;
    const bm_net_info_t *netinfo = NULL;
    std::vector<std::string> network_names;
    bm_misc_info misc_info;

    // configuration
    bool agnostic = false;
    float m_confThreshold = 0.35;
    float m_nmsThreshold = 0.45;
    std::vector<std::string> m_class_names;
    int m_class_num = -1;
    int m_net_h, m_net_w;
    int max_det = 300;
    int max_wh = 7680;  // (pixels) maximum box width and height
    bmcv_convert_to_attr converto_attr;
    TimeStamp tmp_ts;
    bool is_output_transposed = true;

private:
    int pre_process(const std::vector<bm_image>& images, 
                    bm_tensor_t& input_tensor,
                    std::vector<std::pair<int, int>>& txy_batch, 
                    std::vector<std::pair<float, float>>& ratios_batch);
    int forward(bm_tensor_t& input_tensor, std::vector<bm_tensor_t>& output_tensors);
    float* get_cpu_data(bm_tensor_t* tensor, float scale);
    int post_process(const std::vector<bm_image>& input_images, 
                     std::vector<bm_tensor_t>& output_tensors, 
                     const std::vector<std::pair<int, int>>& txy_batch, 
                     const std::vector<std::pair<float, float>>& ratios_batch,
                     std::vector<YoloV8BoxVec>& boxes);
    static float get_aspect_scaled_ratio(int src_w, int src_h, int dst_w, int dst_h, bool* alignWidth);
    int argmax(float* data, int num);
    void xywh2xyxy(YoloV8BoxVec& xyxyboxes, std::vector<std::vector<float>> box);
    void NMS(YoloV8BoxVec& dets, float nmsConfidence);
    void clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h);
public:
    int Init(std::string bmodel_file );
    bm_handle_t handle;
    int batch_size = -1;
    TimeStamp* m_ts = NULL;
    int Detect(const std::vector<bm_image>& images, std::vector<YoloV8BoxVec>& boxes, int index);
    void draw_result(cv::Mat& img, YoloV8BoxVec& result);

    ~YoloV8_det(){
    if (bmrt!=NULL) {
        bmrt_destroy(bmrt);
        bmrt = NULL;
    }  
    bm_dev_free(handle);
};

};

#endif  //! YOLOV8_DET_H