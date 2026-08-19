#include "log_macros.h"
#include <infer.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <unordered_map>
#include <algorithm>

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
        cr.receive_local_ms = fd.receive_local_ms;
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
#elif USE_SOPHON || USE_NVIDIA
void batch_preprocess_thread(int batch_size, int max_full) {
    LOG_INFO("[batch_pre] thread started, batch_size=%d max_full=%d", batch_size, max_full);
    int num_cameras = g_cam_frame_queues.size();
    int round_start = 0;

#if USE_NVIDIA
    // ROI 启用时：每个完整帧追加一个 ROI 槽位，总槽数 = 2×完整帧。
    // 必须限制完整帧 ≤ batch_size/2，否则总 batch 超过 engine 最大 batch，
    // 导致输入 device 缓冲（按 max batch 分配）被写越界 -> cudaMemcpyAsync invalid argument
    // 故对可配 max_full 做安全钳制：ROI 开时 ≤ batch_size/2，ROI 关时 ≤ batch_size。
    // roi_active：全局开关开启，或任一相机在 camera_roi 中单独启用（per-camera 可覆盖全局关闭）
    bool global_roi = static_cast<NvidiaPipeline*>(g_pipeline.get())->roiEnabled();
    bool any_per_cam_roi = false;
    for (const auto& c : g_camera_roi) {
        if (c.enabled) { any_per_cam_roi = true; break; }
    }
    bool roi_active = global_roi || any_per_cam_roi;
    if (roi_active) {
        max_full = std::min(max_full, batch_size / 2);
    } else {
        max_full = std::min(max_full, batch_size);
    }
    LOG_INFO("[batch_pre] max_full after cap=%d roi_active=%d", max_full, (int)roi_active);
#else
    max_full = std::min(max_full, batch_size);
#endif

#if USE_NVIDIA
    long long nvidia_total_ms = 0, nvidia_pre_ms = 0;
    int nvidia_count = 0;
    auto nvidia_t0 = std::chrono::steady_clock::now();
#endif

    while (ros::ok()) {
        std::vector<BatchFrameData> batch_frames;

#if USE_NVIDIA
        {
            // 诊断：打印取帧前各相机队列的实时缓存帧数，判断是否有路积压/空转
            std::string qs;
            for (int ci = 0; ci < num_cameras; ci++) {
                if (ci) qs += " ";
                qs += "cam" + std::to_string(ci) + "=" + std::to_string((int)g_cam_frame_queues[ci].size());
            }
            LOG_DEBUG("[batch-probe] pre-round queues: %s", qs.c_str());
        }
#endif

        // Round-robin: 首轮把各相机已排队的帧尽量一次取够（每相机最多 1 帧，可多轮），
        // 直到攒满 max_full 或一轮遍历没有新帧为止，避免随后无谓的攒批等待。
        // 相机总是源源不断喂帧（队列常深），首轮即可吃满 max_full，无需再等 35ms。
        {
            bool sweep_got;
            do {
                sweep_got = false;
                for (int i = 0; i < num_cameras && (int)batch_frames.size() < max_full; i++) {
                    int c = (round_start + i) % num_cameras;
                    BatchFrameData fd;
                    if (g_cam_frame_queues[c].Consume(fd)) {
                        batch_frames.push_back(std::move(fd));
#if USE_NVIDIA
                        g_frame_cnt_pre[fd.camera_id]++;
#endif
                        sweep_got = true;
                    }
                }
            } while (sweep_got && (int)batch_frames.size() < max_full);
        }
#if USE_NVIDIA
        {
            // 诊断：打印首轮取帧后的批组成（哪些相机有数据），确认是否每路都能取到
            std::string bm;
            for (auto& bf : batch_frames) {
                if (!bm.empty()) bm += " ";
                bm += std::to_string(bf.camera_id);
            }
            LOG_DEBUG("[batch-probe] first-round got=%zu batch=[%s]", batch_frames.size(), bm.c_str());
        }
#endif
        if (!batch_frames.empty()) {
            round_start = (batch_frames.back().camera_id + 1) % num_cameras;
        }
#if USE_NVIDIA
        // NVIDIA 攒批等待：首轮已尽力吃满队列后仍未攒满（说明确实缺帧），
        // 才在 GPU 推理期间（约 35ms）继续从各相机补充帧，提高 batch 利用率。
        // 攒满 max_full 或超时后立即提交。队列满时首轮已攒满，不会进入此等待。
        // 仅 NVIDIA 生效，sophon 保持原逻辑。
        if (!batch_frames.empty() && (int)batch_frames.size() < max_full) {
            auto wait_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(35);
            while ((int)batch_frames.size() < max_full &&
                   std::chrono::steady_clock::now() < wait_until) {
                bool got = false;
                for (int i = 0; i < num_cameras && (int)batch_frames.size() < max_full; i++) {
                    int c = (round_start + i) % num_cameras;
                    BatchFrameData fd;
                    if (g_cam_frame_queues[c].Consume(fd)) {
                        batch_frames.push_back(std::move(fd));
#if USE_NVIDIA
                        g_frame_cnt_pre[fd.camera_id]++;
#endif
                        got = true;
                    }
                }
                if (!got) {
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                }
            }
            round_start = (batch_frames.back().camera_id + 1) % num_cameras;
        }
        LOG_DEBUG("[batch-probe] final batch=%d", (int)batch_frames.size());
#endif
        if (batch_frames.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

#if USE_NVIDIA
        // ROI 放大检测增强（方案A）：为每个完整帧追加一个 ROI 抠图槽位，混入同一 batch。
        // ROI 尺寸与左上角位置由 per-camera 配置决定（g_camera_roi，按 camera_id 索引），
        // 未配置的相机回退到全局值。坐标为浅拷贝视图，映射回原图（offset_x/offset_y）在 post 阶段完成。
        {
            auto* np = static_cast<NvidiaPipeline*>(g_pipeline.get());
            if (roi_active) {
                size_t base_cnt = batch_frames.size();
                std::vector<BatchFrameData> roi_slots;
                roi_slots.reserve(base_cnt);
                for (size_t i = 0; i < base_cnt; i++) {
                    const auto& fd = batch_frames[i];
                    if (fd.roi_parent_slot >= 0) continue;  // 只对完整帧生成 ROI
                    // 取该相机的 ROI 配置（per-camera）；越界或未配置时回退到全局值
                    const CameraRoiConfig* rc = (fd.camera_id >= 0 && fd.camera_id < (int)g_camera_roi.size())
                                                    ? &g_camera_roi[fd.camera_id] : nullptr;
                    bool enabled = rc ? rc->enabled : np->roiEnabled();
                    float h_ratio = rc ? rc->height_ratio : np->roiHeightRatio();
                    float w_ratio = rc ? rc->width_ratio : np->roiWidthRatio();
                    float y_ratio = rc ? rc->y_ratio : 0.0f;
                    float x_ratio = rc ? rc->x_ratio : 0.5f;
                    if (!enabled) continue;  // 该相机未启用 ROI，不生成槽位
                    int roi_h = (int)(fd.mat.rows * h_ratio);
                    int roi_w = (int)(fd.mat.cols * w_ratio);
                    if (roi_h < 8 || roi_w < 8) continue;
                    // ROI 框左上角（分数偏移），并钳制到帧内
                    int roi_x = std::max(0, std::min((int)(fd.mat.cols * x_ratio), fd.mat.cols - roi_w));
                    int roi_y = std::max(0, std::min((int)(fd.mat.rows * y_ratio), fd.mat.rows - roi_h));
                    BatchFrameData roifd;
                    roifd.mat = fd.mat(cv::Rect(roi_x, roi_y, roi_w, roi_h));  // 浅拷贝视图
                    roifd.camera_id = fd.camera_id;
                    roifd.roi_parent_slot = (int)i;  // 所属完整帧在本 batch 中的槽位索引
                    roifd.roi_offset_x = roi_x;      // 映射回原图的水平偏移
                    roifd.roi_offset_y = roi_y;      // 映射回原图的纵向偏移
                    // ROI 调试：每相机 2s 存一张"裁剪的 ROI 图"与"带 ROI 框的原图"，便于确认裁剪内容是否正确
                    if (g_roi_save_debug) {
                        static std::mutex roi_mtx;
                        static std::unordered_map<int, std::chrono::steady_clock::time_point> roi_last;
                        static bool roi_dir_ok = false;
                        std::lock_guard<std::mutex> lg(roi_mtx);
                        if (!roi_dir_ok) {
                            std::string cmd = "mkdir -p " + g_roi_save_path;
                            if (system(cmd.c_str()) == 0) {
                                roi_dir_ok = true;
                            }
                        }
                        auto now = std::chrono::steady_clock::now();
                        auto& last = roi_last[fd.camera_id];
                        if (now - last >= std::chrono::seconds(2)) {
                            last = now;
                            char ts[64];
                            auto tnow = std::chrono::system_clock::now();
                            std::time_t tt = std::chrono::system_clock::to_time_t(tnow);
                            struct tm tmv;
                            localtime_r(&tt, &tmv);
                            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
                            std::string base = g_roi_save_path + "/cam" + std::to_string(fd.camera_id) + "_" + ts;
                            if (roifd.mat.rows > 0 && roifd.mat.cols > 0) {
                                cv::imwrite(base + "_roi.jpg", roifd.mat);
                            }
                            cv::Mat orig = fd.mat.clone();
                            if (!orig.empty()) {
                                cv::rectangle(orig, cv::Rect(roi_x, roi_y, roi_w, roi_h), cv::Scalar(0, 255, 255), 3);
                                cv::imwrite(base + "_orig.jpg", orig);
                            }
                            LOG_INFO("[roi-debug] saved cam=%d roi=%dx%d at x=%d y=%d -> %s_*.jpg",
                                     fd.camera_id, roi_w, roi_h, roi_x, roi_y, base.c_str());
                        }
                    }
                    roi_slots.push_back(std::move(roifd));
                }
                for (auto& r : roi_slots) {
                    batch_frames.push_back(std::move(r));
                }
            }
        }
#endif

        InferResult ir;
        auto t_pre = std::chrono::steady_clock::now();
        int ret = g_pipeline->preprocessAndForward(batch_frames, ir);
        auto t_end = std::chrono::steady_clock::now();
#if USE_NVIDIA
        nvidia_pre_ms += std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_pre).count();
        nvidia_count++;
        auto now = std::chrono::steady_clock::now();
        if (now - nvidia_t0 >= std::chrono::milliseconds(5000)) {
            nvidia_total_ms += std::chrono::duration_cast<std::chrono::milliseconds>(now - nvidia_t0).count();
            LOG_DEBUG("[nvidia-batch] pre_thread: busy=%lldms window=%lldms batches=%d", nvidia_pre_ms, nvidia_total_ms, nvidia_count);
            for (size_t ci = 0; ci < g_frame_cnt_input.size(); ci++) {
                double win_s = nvidia_total_ms / 1000.0;
                if (win_s <= 0) win_s = 1.0;
                LOG_DEBUG("[nvidia-cnt] cam=%zu input=%.1ffps pre=%.1ffps post=%.1ffps (in=%lld pre=%lld post=%lld)",
                         ci,
                         g_frame_cnt_input[ci] / win_s,
                         g_frame_cnt_pre[ci] / win_s,
                         g_frame_cnt_post[ci] / win_s,
                         g_frame_cnt_input[ci], g_frame_cnt_pre[ci], g_frame_cnt_post[ci]);
                g_frame_cnt_input[ci] = 0;
                g_frame_cnt_pre[ci] = 0;
                g_frame_cnt_post[ci] = 0;
            }
            nvidia_pre_ms = 0;
            nvidia_total_ms = 0;
            nvidia_count = 0;
            nvidia_t0 = now;
        }
#endif
        if (ret != 0) {
            LOG_ERROR("[batch_pre] preprocessAndForward failed");
            continue;
        }

        g_post_queue.Produce(std::move(ir));
    }
}

void batch_postprocess_thread() {
    LOG_INFO("[batch_post] thread started");
#if USE_NVIDIA
    long long nvidia_post_ms = 0;
    int nvidia_count = 0;
    auto nvidia_t0 = std::chrono::steady_clock::now();
#endif
    while (ros::ok()) {
        InferResult ir;
        if (!g_post_queue.ConsumeSync(ir)) {
            continue;
        }

        auto t_post = std::chrono::steady_clock::now();
        int ret = g_pipeline->postProcess(ir);
        auto t_end = std::chrono::steady_clock::now();
#if USE_NVIDIA
        nvidia_post_ms += std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_post).count();
        nvidia_count++;
        auto now = std::chrono::steady_clock::now();
        if (now - nvidia_t0 >= std::chrono::milliseconds(5000)) {
            LOG_DEBUG("[nvidia-batch] post_thread: busy=%lldms window=%lldms batches=%d", nvidia_post_ms,
                     std::chrono::duration_cast<std::chrono::milliseconds>(now - nvidia_t0).count(), nvidia_count);
            nvidia_post_ms = 0;
            nvidia_count = 0;
            nvidia_t0 = now;
        }
#endif
        if (ret != 0) {
            LOG_ERROR("[batch_post] postProcess failed");
            continue;
        }

        for (int i = 0; i < ir.batch_size; i++) {
#if USE_NVIDIA
            // ROI 槽位不产生独立结果（已合并进所属完整帧）
            if (ir.roi_parent_slots[i] >= 0) continue;
#endif
            int cam_id = ir.camera_ids[i];
            CameraResult cr;
            cr.frame = std::move(ir.frames[i]);
            cr.detections = std::move(ir.detections[i]);
            cr.img_time_sec = ir.img_time_secs[i];
            cr.img_time_nsec = ir.img_time_nsecs[i];
            if (i < (int)ir.receive_local_ms_list.size())
                cr.receive_local_ms = ir.receive_local_ms_list[i];
#if USE_NVIDIA
            g_frame_cnt_post[cam_id]++;
#endif
            g_result_queues[cam_id].Produce(std::move(cr));
        }
    }
}

#if USE_SOPHON
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
#endif // USE_SOPHON
#endif
