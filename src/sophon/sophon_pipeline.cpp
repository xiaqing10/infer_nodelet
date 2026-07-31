#include "sophon_pipeline.hpp"
#include "detector.hpp"
#include "log_macros.h"

int SophonPipeline::init(const std::string& model_path) {
    return detector_.Init(model_path);
}

int SophonPipeline::preprocessAndInfer(
    const std::vector<BatchFrameData>& batch_frames,
    InferResult& result) {
    int n = batch_frames.size();
    std::vector<bm_image> batch_imgs(n);
    for (int i = 0; i < n; i++) {
        cv::bmcv::toBMI(const_cast<cv::Mat&>(batch_frames[i].mat), &batch_imgs[i]);
    }

    bm_tensor_t input_tensor;
    std::vector<bm_tensor_t> output_tensors;
    output_tensors.resize(detector_.getOutputNum());
    std::vector<std::pair<int, int>> txy_batch;
    std::vector<std::pair<float, float>> ratios_batch;

    int ret = detector_.pre_process(batch_imgs, input_tensor, txy_batch, ratios_batch);
    if (ret != 0) {
        LOG_ERROR("[sophon] pre_process failed");
        for (auto& img : batch_imgs) bm_image_destroy(img);
        return ret;
    }

    ret = detector_.forward(input_tensor, output_tensors);
    if (ret != 0) {
        LOG_ERROR("[sophon] forward failed");
        for (auto& img : batch_imgs) bm_image_destroy(img);
        return ret;
    }

    std::vector<YoloV8BoxVec> boxes;
    ret = detector_.post_process(batch_imgs, output_tensors, txy_batch, ratios_batch, boxes);
    if (ret != 0) {
        LOG_ERROR("[sophon] post_process failed");
        for (auto& img : batch_imgs) bm_image_destroy(img);
        return ret;
    }

    result.detections.resize(n);
    for (int i = 0; i < n; i++) {
        for (auto& box : boxes[i]) {
            DetectorRetData d;
            d.label = box.class_id;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            result.detections[i].push_back(d);
        }
    }

    result.frames.resize(n);
    result.camera_ids.resize(n);
    result.img_time_secs.resize(n);
    result.img_time_nsecs.resize(n);
    result.batch_size = n;

    for (int i = 0; i < n; i++) {
        result.frames[i] = batch_frames[i].mat.clone();
        result.camera_ids[i] = batch_frames[i].camera_id;
        result.img_time_secs[i] = batch_frames[i].img_time_sec;
        result.img_time_nsecs[i] = batch_frames[i].img_time_nsec;
    }

    for (auto& img : batch_imgs) {
        bm_image_destroy(img);
    }

    return 0;
}