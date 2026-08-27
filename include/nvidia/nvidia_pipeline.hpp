#ifndef NVIDIA_PIPELINE_H
#define NVIDIA_PIPELINE_H

#include "batch_pipeline_base.hpp"
#include "detector_nvidia.hpp"
#include <memory>
#include <mutex>

class NvidiaPipeline : public BatchPipeline {
public:
    int init(const std::string& model_path) override;
    int preprocessAndForward(
        std::vector<BatchFrameData>& batch_frames,
        InferResult& result) override;
    int postProcess(InferResult& result) override;
    int getBatchSize() const override { return detector_.getBatchSize(); }

    void setNumLabels(int n) { num_labels_ = n; }
    YoloV8_det& getDetector() { return detector_; }

    void setRoiEnabled(bool e) { roi_enabled_ = e; }
    void setRoiHeightRatio(float r) { roi_height_ratio_ = r; }
    void setRoiWidthRatio(float r) { roi_width_ratio_ = r; }
    void setNmsIou(float iou) { nms_iou_threshold_ = iou; }
    void setDetectThresholds(float conf, float nms) override { conf_threshold_ = conf; nms_threshold_ = nms; }

    bool roiEnabled() const { return roi_enabled_; }
    float roiHeightRatio() const { return roi_height_ratio_; }
    float roiWidthRatio() const { return roi_width_ratio_; }
    float nmsIou() const { return nms_iou_threshold_; }
    float confThreshold() const { return conf_threshold_; }
    float nmsThreshold() const { return nms_threshold_; }

private:
    // NVIDIA 检测器 pre/post 共享内部缓冲，需串行化防止两个 worker 并发访问
    mutable std::mutex mtx_;
    YoloV8_det detector_;
    int num_labels_ = 10;

    // ROI 放大检测增强（方案A：ROI 抠图混入同一 batch）。
    // roi_enabled_ = 批内任一相机在 camera_roi 中启用 ROI（roi_active），
    // 用于门控"主图+ROI 合并去重"；ROI 本身完全按相机单独配置（g_camera_roi）。
    bool roi_enabled_ = false;
    float roi_height_ratio_ = 0.5f;  // 保留默认值（未使用，ROI 尺寸已按相机配置）
    float roi_width_ratio_ = 0.5f;   // 保留默认值（未使用）
    float nms_iou_threshold_ = 0.5f; // 主图+ROI 合并去重阈值
    float conf_threshold_ = 0.25f;   // 检测置信度阈值（默认与检测器一致）
    float nms_threshold_ = 0.65f;    // 检测器 NMS IoU 阈值（默认与检测器一致）
};

#endif // NVIDIA_PIPELINE_H
