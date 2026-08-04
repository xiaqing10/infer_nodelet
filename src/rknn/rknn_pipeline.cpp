#include "rknn_pipeline.hpp"
#include "detector.hpp"

int RknnPipeline::init(const std::string& model_path) {
    return detector_.Init(model_path);
}

int RknnPipeline::preprocessAndInfer(
    const std::vector<BatchFrameData>& batch_frames,
    InferResult& result) {
    int n = batch_frames.size();
    if (n == 0) return 0;

    result.detections.resize(n);
    result.frames.resize(n);
    result.camera_ids.resize(n);
    result.img_time_secs.resize(n);
    result.img_time_nsecs.resize(n);
    result.batch_size = n;

    for (int i = 0; i < n; i++) {
        YoloV8BoxVec boxes;
        int ret = detector_.Detect(batch_frames[i].mat, boxes);
        if (ret != 0) {
            return ret;
        }

        for (auto& box : boxes) {
            DetectorRetData d;
            d.label = box.class_id;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            result.detections[i].push_back(d);
        }

        result.frames[i] = batch_frames[i].mat.clone();
        result.camera_ids[i] = batch_frames[i].camera_id;
        result.img_time_secs[i] = batch_frames[i].img_time_sec;
        result.img_time_nsecs[i] = batch_frames[i].img_time_nsec;
    }

    return 0;
}