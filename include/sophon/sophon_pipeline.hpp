#ifndef SOPHON_PIPELINE_H
#define SOPHON_PIPELINE_H

#include "batch_pipeline_base.hpp"
#include "detector_sophon.hpp"
#include "bm_wrapper_sophon.hpp"
#include <memory>

class SophonPipeline : public BatchPipeline {
public:
    int init(const std::string& model_path) override;
    int preprocessAndForward(
        std::vector<BatchFrameData>& batch_frames,
        InferResult& result) override;
    int postProcess(InferResult& result) override;
    int getBatchSize() const override { return detector_.batch_size; }

    void setDetectThresholds(float conf, float nms) override { detector_.setDetectThresholds(conf, nms); }

    YoloV8_det& getDetector() { return detector_; }

private:
    YoloV8_det detector_;
};

#endif // SOPHON_PIPELINE_H