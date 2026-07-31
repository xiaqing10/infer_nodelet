#ifndef BATCH_PIPELINE_BASE_H
#define BATCH_PIPELINE_BASE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>

// Forward-declare: full definition in detector.hpp
struct DetectorRetData;

// 每个相机 frame 入队
struct BatchFrameData {
    cv::Mat mat;
    int camera_id;
    int frame_width;
    int frame_height;
    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
};

// 每个相机的推理结果（帧+检测结果），由 processResult 线程消费
struct CameraResult {
    cv::Mat frame;
    std::vector<DetectorRetData> detections;
    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
};

// 一次 batch 推理的结果（平台无关），由 post 线程消费
struct InferResult {
    std::vector<cv::Mat> frames;
    std::vector<int> camera_ids;
    std::vector<double> img_time_secs;
    std::vector<double> img_time_nsecs;
    std::vector<std::vector<DetectorRetData>> detections;  // per-frame detections
    int batch_size;
};

// 平台无关的 batch 流水线接口
class BatchPipeline {
public:
    virtual ~BatchPipeline() = default;

    virtual int init(const std::string& model_path) = 0;

    // 预处理 + 推理：输入一批帧，输出检测结果
    virtual int preprocessAndInfer(
        const std::vector<BatchFrameData>& batch_frames,
        InferResult& result) = 0;

    virtual int getBatchSize() const = 0;
};

#endif // BATCH_PIPELINE_BASE_H