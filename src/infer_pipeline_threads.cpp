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

// 全局共享的颜色分类 worker：消费所有路相机提交的车俩 crop，路由结果回对应路
void color_infer_thread() {
    LOG_INFO("[color_infer] thread started");
    while (ros::ok()) {
        ModelInputData input_data;
        if (!g_color_queue.ConsumeSync(input_data)) {
            continue;
        }
        ClsRetData cls_ret = g_color_model->inference(input_data.im);
        VehicleColorResult vcr;
        vcr.tracker_id = input_data.tracker_id;
        vcr.vehicle_color = cls_ret.label;
        vcr.confidence = cls_ret.confidence;
        vcr.camera_id = input_data.camera_id;
        if (input_data.camera_id >= 0 && input_data.camera_id < (int)g_color_result_queues.size()) {
            g_color_result_queues[input_data.camera_id].Produce(std::move(vcr));
        }
    }
}

// 全局共享的抛洒物检测 worker：消费所有路相机提交的（帧+同帧det框），过滤后路由回对应路
void abandon_infer_thread() {
    LOG_INFO("[abandon_infer] thread started");
    while (ros::ok()) {
        AbandonInputData input_data;
        if (!g_abandon_queue.ConsumeSync(input_data)) {
            continue;
        }

        cv::Mat img = input_data.im;
        auto results = g_abandon_model->inference(img);

        std::vector<DetectorRetData> valid_results;
        valid_results.reserve(results.size());
        std::copy_if(results.begin(), results.end(), std::back_inserter(valid_results),
            [&img](const auto& obj) {
                return obj.xmin >= 0 && obj.ymin >= 0 &&
                       obj.xmax < img.cols && obj.ymax < img.rows;
            });

        std::vector<DetectorRetData> filtered;
        for (const auto& new_obj : valid_results) {
            bool match_found = std::any_of(input_data.data.begin(), input_data.data.end(),
                [&new_obj](const auto& existing_obj) {
                    return CalculateOverlap(
                        new_obj.xmin, new_obj.ymin, new_obj.xmax, new_obj.ymax,
                        existing_obj.xmin, existing_obj.ymin, existing_obj.xmax, existing_obj.ymax
                    ) > 0.1f;
                });
            if (!match_found) {
                filtered.push_back(new_obj);
            }
        }

        DetectorRetDatas results_data{{std::move(filtered)}};
        if (input_data.camera_id >= 0 && input_data.camera_id < (int)g_abandon_result_queues.size()) {
            g_abandon_result_queues[input_data.camera_id].Produce(std::move(results_data));
        }
    }
}
#endif
