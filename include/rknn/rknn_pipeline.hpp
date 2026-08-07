#ifndef RKNN_PIPELINE_H
#define RKNN_PIPELINE_H

#include "batch_pipeline_base.hpp"
#include "detector_rknn.hpp"
#include <memory>
#include <atomic>
#include <future>

#define RKNN_NUM_CORES 3

class RknnPipeline : public BatchPipeline {
public:
    int init(const std::string& model_path) override;
    int init(const std::string& model_path, const std::string& model_type);
    int preprocessAndInfer(
        const std::vector<BatchFrameData>& batch_frames,
        InferResult& result) override;
    int getBatchSize() const override { return RKNN_NUM_CORES; }

    YoloV8_det& getDetector(int core_id) { return detectors_[core_id]; }

private:
    YoloV8_det detectors_[RKNN_NUM_CORES];
};

#endif