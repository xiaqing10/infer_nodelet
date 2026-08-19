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

    bool roiEnabled() const { return roi_enabled_; }
    float roiHeightRatio() const { return roi_height_ratio_; }
    float roiWidthRatio() const { return roi_width_ratio_; }
    float nmsIou() const { return nms_iou_threshold_; }

private:
    // NVIDIA 检测器 pre/post 共享内部缓冲，需串行化防止两个 worker 并发访问
    mutable std::mutex mtx_;
    YoloV8_det detector_;
    int num_labels_ = 10;

    // ROI 放大检测增强（方案A：ROI 抠图混入同一 batch）
    bool roi_enabled_ = false;
    float roi_height_ratio_ = 0.5f;  // ROI 高度 = 原图高 * 比例，取 [0,0.5] 上半区域
    float roi_width_ratio_ = 0.5f;   // ROI 保留宽度 = 原图宽 * 比例，水平居中，取 [0.25,0.75]
    float nms_iou_threshold_ = 0.5f; // 主图+ROI 合并去重阈值
};

#endif // NVIDIA_PIPELINE_H
