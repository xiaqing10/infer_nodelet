#ifndef BATCH_PIPELINE_BASE_H
#define BATCH_PIPELINE_BASE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>

#if USE_SOPHON
#include "sophon/detector_sophon.hpp"
#endif
#if USE_NVIDIA
#include "nvidia/detector_nvidia.hpp"
#endif

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
    // 本机接收到该帧时的本地墙钟（ms），用于在发布点计算处理延时（与数据源时钟无关）
    int64_t receive_local_ms = 0;
    // ROI 增强元数据：-1 = 完整帧；>=0 = 该 ROI 槽位所属完整帧在本 batch 中的槽位索引
    int roi_parent_slot = -1;
    int roi_offset_x = 0;
    int roi_offset_y = 0;
};

// 每个相机的推理结果（帧+检测结果），由 processResult 线程消费
struct CameraResult {
    cv::Mat frame;
    std::vector<DetectorRetData> detections;
    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
    int64_t receive_local_ms = 0;
};

// 一次 batch 推理的结果（平台无关），由 post 线程消费
struct InferResult {
    std::vector<cv::Mat> frames;
    std::vector<int> camera_ids;
    std::vector<double> img_time_secs;
    std::vector<double> img_time_nsecs;
    std::vector<int64_t> receive_local_ms_list;
    std::vector<std::vector<DetectorRetData>> detections;  // per-frame detections
    int batch_size;
    // ROI 槽位元数据（与 frames/camera_ids 对齐，-1 表示完整帧）
    std::vector<int> roi_parent_slots;
    std::vector<int> roi_offset_xs;
    std::vector<int> roi_offset_ys;
#if USE_SOPHON
    // Sophon 中间结果：由 pre 线程产出，post 线程消费（用于流水线重叠）
    std::vector<bm_image> input_images;
    std::vector<bm_tensor_t> output_tensors;
    std::vector<std::pair<int, int>> txy_batch;
    std::vector<std::pair<float, float>> ratios_batch;
#endif
#if USE_NVIDIA
    // NVIDIA 中间结果：由 pre 线程产出，post 线程消费。
    // 每批独立携带 letterbox 参数与原始输出，避免 post 线程读取检测器共享缓冲
    // 而被下一个 batch 的 pre 覆盖（轨迹闪烁根因）。
    std::vector<PreParam> nvidia_pparams;
    std::vector<float> nvidia_output;
#endif
};

// 平台无关的 batch 流水线接口
class BatchPipeline {
public:
    virtual ~BatchPipeline() = default;

    virtual int init(const std::string& model_path) = 0;

    // 预处理 + 前向推理：输入一批帧，输出中间结果（由 post 线程消费）
    virtual int preprocessAndForward(
        std::vector<BatchFrameData>& batch_frames,
        InferResult& result) = 0;

    // 后处理：从中间结果生成检测框
    virtual int postProcess(InferResult& result) = 0;

    virtual int getBatchSize() const = 0;

    // 检测置信度阈值与检测器 NMS IoU 阈值（由上层从 yaml 配置注入，需在 init 前调用）
    virtual void setDetectThresholds(float conf_threshold, float nms_threshold) = 0;
};

#endif // BATCH_PIPELINE_BASE_H