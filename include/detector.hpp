#ifndef YOLOV8_DET_H
#define YOLOV8_DET_H

#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#if USE_CAMBRICON
#include "cambricon/detector_cambricon.hpp"
#elif USE_SOPHON
#include "sophon/detector_sophon.hpp"
#elif USE_ASCEND
#include "ascend/detector_ascend.hpp"
#endif

typedef struct DetectorRetData{
    int label;
    float confidence;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
}DetectorRetData;

typedef struct DetectorRetDatas{
    std::vector<DetectorRetData> data;
}DetectorRetDatas;

struct POINT_2D_S {
  float cx, cy;
};

class Detector{
public:
    void init(std::string model, int device_id, int num_class, int stride);
    std::vector<DetectorRetData> inference(cv::Mat &img);

private:
#if USE_CAMBRICON
    cn::Net net;
    float nms_thred = 0.5;
    float conf_thred = 0.25;
    int num_cls = -1;
    int feature_stride = -1;
    void yolov8_nms(std::vector<std::vector<float>>& boxes);
    float letterbox(const cv::Mat& image, cv::Mat& out_image, const cv::Size& new_shape, const cv::Scalar& color);
#elif USE_SOPHON
    YoloV8_det yolo_det;
#elif USE_ASCEND
    YoloV8_det yolo_det;
#endif
};

#endif
