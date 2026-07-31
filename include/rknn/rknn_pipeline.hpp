#ifndef RKNN_PIPELINE_H
#define RKNN_PIPELINE_H

#include "batch_pipeline_base.hpp"
#include "detector_rknn.hpp"
#include <memory>

class RknnPipeline : public BatchPipeline {
public:
    int init(const std::string& model_path) override;
    int preprocessAndInfer(
        const std::vector<BatchFrameData>& batch_frames,
        InferResult& result) override;
    int getBatchSize() const override { return detector_.batch_size; }

    YoloV8_det& getDetector() { return detector_; }

private:
    YoloV8_det detector_;
};

#endif // RKNN_PIPELINE_H