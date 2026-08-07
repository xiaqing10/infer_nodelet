#include "rknn_pipeline.hpp"
#include "detector.hpp"

static const rknn_core_mask kCoreMasks[RKNN_NUM_CORES] = {
    RKNN_NPU_CORE_0,
    RKNN_NPU_CORE_1,
    RKNN_NPU_CORE_2
};

int RknnPipeline::init(const std::string& model_path) {
    return init(model_path, "auto");
}

int RknnPipeline::init(const std::string& model_path, const std::string& model_type) {
    for (int i = 0; i < RKNN_NUM_CORES; i++) {
        detectors_[i].m_model_type = model_type;
        int ret = detectors_[i].Init(model_path, kCoreMasks[i]);
        if (ret != 0) {
            std::cerr << "RknnPipeline: failed to init detector on core " << i << std::endl;
            return ret;
        }
    }
    std::cout << "RknnPipeline: initialized " << RKNN_NUM_CORES
              << " detectors on cores 0/1/2, model_type=" << model_type << std::endl;
    return 0;
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

    std::vector<std::thread> threads(n);
    for (int i = 0; i < n; i++) {
        int core_id = i % RKNN_NUM_CORES;
        YoloV8_det& det = detectors_[core_id];

        cv::Mat frame = batch_frames[i].mat.clone();
        int camera_id = batch_frames[i].camera_id;
        double img_time_sec = batch_frames[i].img_time_sec;
        double img_time_nsec = batch_frames[i].img_time_nsec;

        threads[i] = std::thread([&det, frame, camera_id, img_time_sec, img_time_nsec, i, &result]() {
            YoloV8BoxVec boxes;
            int ret = det.Detect(frame, boxes);
            if (ret != 0) return;

            for (auto& box : boxes) {
                DetectorRetData d;
                d.label = box.class_id + 1;
                d.confidence = box.score;
                d.xmin = (int)box.x1;
                d.ymin = (int)box.y1;
                d.xmax = (int)box.x2;
                d.ymax = (int)box.y2;
                result.detections[i].push_back(d);
            }

            result.frames[i] = frame;
            result.camera_ids[i] = camera_id;
            result.img_time_secs[i] = img_time_sec;
            result.img_time_nsecs[i] = img_time_nsec;
        });
    }

    for (int i = 0; i < n; i++) {
        threads[i].join();
    }

    return 0;
}