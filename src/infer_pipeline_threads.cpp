#include "log_macros.h"
#include <infer.h>

#if USE_RKNN
void rknn_infer_thread(int core_id) {
    LOG_INFO("[rknn_infer] thread started, core=%d", core_id);
    YoloV8_det& det = static_cast<RknnPipeline*>(g_pipeline.get())->getDetector(core_id);

    while (ros::ok()) {
        BatchFrameData fd;
        if (!g_infer_queue.ConsumeSync(fd)) {
            continue;
        }

        YoloV8BoxVec boxes;
        int ret = det.Detect(fd.mat, boxes);
        if (ret != 0) continue;

        CameraResult cr;
        cr.frame = std::move(fd.mat);
        cr.img_time_sec = fd.img_time_sec;
        cr.img_time_nsec = fd.img_time_nsec;
        for (auto& box : boxes) {
            DetectorRetData d;
            d.label = box.class_id + 1;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            cr.detections.push_back(d);
        }

        g_result_queues[fd.camera_id].Produce(std::move(cr));
    }
}
#elif USE_SOPHON
void batch_preprocess_thread(int batch_size) {
    LOG_INFO("[batch_pre] thread started, batch_size=%d", batch_size);
    int num_cameras = g_cam_frame_queues.size();
    int round_start = 0;

    while (ros::ok()) {
        std::vector<BatchFrameData> batch_frames;

        // Round-robin: take at most one frame per camera per round, capped at batch_size
        for (int i = 0; i < num_cameras && (int)batch_frames.size() < batch_size; i++) {
            int c = (round_start + i) % num_cameras;
            BatchFrameData fd;
            if (g_cam_frame_queues[c].Consume(fd)) {
                batch_frames.push_back(std::move(fd));
            }
        }
        if (!batch_frames.empty()) {
            round_start = (batch_frames.back().camera_id + 1) % num_cameras;
        }
        if (batch_frames.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        InferResult ir;
        int ret = g_pipeline->preprocessAndForward(batch_frames, ir);
        if (ret != 0) {
            LOG_ERROR("[batch_pre] preprocessAndForward failed");
            continue;
        }

        g_post_queue.Produce(std::move(ir));
    }
}

void batch_postprocess_thread() {
    LOG_INFO("[batch_post] thread started");
    while (ros::ok()) {
        InferResult ir;
        if (!g_post_queue.ConsumeSync(ir)) {
            continue;
        }

        int ret = g_pipeline->postProcess(ir);
        if (ret != 0) {
            LOG_ERROR("[batch_post] postProcess failed");
            continue;
        }

        for (int i = 0; i < ir.batch_size; i++) {
            int cam_id = ir.camera_ids[i];
            CameraResult cr;
            cr.frame = std::move(ir.frames[i]);
            cr.detections = std::move(ir.detections[i]);
            cr.img_time_sec = ir.img_time_secs[i];
            cr.img_time_nsec = ir.img_time_nsecs[i];
            g_result_queues[cam_id].Produce(std::move(cr));
        }
    }
}
#endif
