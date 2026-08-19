#include "nvidia_pipeline.hpp"
#include "detector.hpp"
#include "log_macros.h"
#include <chrono>
#include <algorithm>

int NvidiaPipeline::init(const std::string& model_path) {
    return detector_.Init(model_path, num_labels_);
}

namespace {
// 主图 + ROI 结果合并后的 NMS 去重（按 IoU，分类无关）。
// 方案1：同一目标（IoU>阈值）时，完整帧框（from_roi=false）优先于 ROI 框（from_roi=true），
// 与置信度无关——ROI 框可能不完整（只框住目标一部分），完整帧框通常更完整，应被优先保留；
// ROI 仅补充完整帧漏检的目标。故排序时完整帧框在前，ROI 框在后，NMS 抑制后到的重叠框。
std::vector<DetectorRetData> nmsMerge(const std::vector<DetectorRetData>& dets, float iou_thres) {
    std::vector<DetectorRetData> result;
    std::vector<int> order(dets.size());
    for (size_t i = 0; i < dets.size(); i++) order[i] = (int)i;
    std::sort(order.begin(), order.end(), [&dets](int a, int b) {
        // 完整帧框优先；同来源时按置信度从高到低
        if (dets[a].from_roi != dets[b].from_roi)
            return dets[a].from_roi < dets[b].from_roi;  // false(完整帧) 排前
        return dets[a].confidence > dets[b].confidence;
    });
    std::vector<bool> suppressed(dets.size(), false);
    for (size_t oi = 0; oi < order.size(); oi++) {
        int i = order[oi];
        if (suppressed[i]) continue;
        result.push_back(dets[i]);
        for (size_t oj = oi + 1; oj < order.size(); oj++) {
            int j = order[oj];
            if (suppressed[j]) continue;
            const auto& a = dets[i];
            const auto& b = dets[j];
            int ix1 = std::max(a.xmin, b.xmin), iy1 = std::max(a.ymin, b.ymin);
            int ix2 = std::min(a.xmax, b.xmax), iy2 = std::min(a.ymax, b.ymax);
            int iw = ix2 - ix1, ih = iy2 - iy1;
            if (iw <= 0 || ih <= 0) continue;
            float inter = (float)(iw * ih);
            float aarea = (float)((a.xmax - a.xmin) * (a.ymax - a.ymin));
            float barea = (float)((b.xmax - b.xmin) * (b.ymax - b.ymin));
            float iou = inter / (aarea + barea - inter + 1e-6f);
            if (iou > iou_thres) suppressed[j] = true;
        }
    }
    return result;
}

// NVIDIA batch 流水线阶段耗时统计（仅 NVIDIA 平台，用于定位帧率瓶颈）
struct NvidiaTiming {
    int count = 0;
    long long frames = 0;
    double pre_ms = 0.0;
    double forward_ms = 0.0;
    double post_ms = 0.0;
    std::chrono::steady_clock::time_point last_print;

    // 最近一个 batch 的 pre/forward 耗时（供 postProcess 组合出完整单 batch 耗时）
    double last_pre_ms = 0.0;
    double last_forward_ms = 0.0;
    int last_batch = 0;

    NvidiaTiming() : last_print(std::chrono::steady_clock::now()) {}

    void print_if_due() {
        auto now = std::chrono::steady_clock::now();
        if (now - last_print < std::chrono::milliseconds(5000)) return;
        if (count > 0) {
            double avg_frames = (double)frames / count;
            LOG_DEBUG("[nvidia-batch] avg_frames=%.1f  pre=%.2fms  forward=%.2fms  post=%.2fms  (batches=%d)",
                     avg_frames, pre_ms / count, forward_ms / count, post_ms / count, count);
        }
        count = 0;
        frames = 0;
        pre_ms = forward_ms = post_ms = 0.0;
        last_print = now;
    }
};
NvidiaTiming g_nvidia_timing;
}

int NvidiaPipeline::preprocessAndForward(
    std::vector<BatchFrameData>& batch_frames,
    InferResult& result) {
    std::lock_guard<std::mutex> lock(mtx_);

    int n = (int)batch_frames.size();
    std::vector<cv::Mat> images(n);
    for (int i = 0; i < n; i++) {
        images[i] = batch_frames[i].mat;
    }

    auto t_pre = std::chrono::steady_clock::now();
    int ret = detector_.pre_process(images);
    if (ret != 0) {
        LOG_ERROR("[nvidia] pre_process failed");
        return ret;
    }
    auto t_fwd = std::chrono::steady_clock::now();

    ret = detector_.forward();
    if (ret != 0) {
        LOG_ERROR("[nvidia] forward failed");
        return ret;
    }
    auto t_end = std::chrono::steady_clock::now();

    // 立即导出本 batch 的中间数据到 result，随队列传给 post 线程。
    // 必须在 forward 之后、下一个 batch 的 pre_process 覆盖共享缓冲之前完成。
    detector_.export_batch_data(result.nvidia_pparams, result.nvidia_output);

    g_nvidia_timing.count++;
    g_nvidia_timing.frames += n;
    g_nvidia_timing.pre_ms += std::chrono::duration<double, std::milli>(t_fwd - t_pre).count();
    g_nvidia_timing.forward_ms += std::chrono::duration<double, std::milli>(t_end - t_fwd).count();
    g_nvidia_timing.last_pre_ms = std::chrono::duration<double, std::milli>(t_fwd - t_pre).count();
    g_nvidia_timing.last_forward_ms = std::chrono::duration<double, std::milli>(t_end - t_fwd).count();
    g_nvidia_timing.last_batch = n;

    result.frames.resize(n);
    result.camera_ids.resize(n);
    result.img_time_secs.resize(n);
    result.img_time_nsecs.resize(n);
    result.receive_local_ms_list.resize(n);
    result.detections.clear();
    result.batch_size = n;
    result.roi_parent_slots.resize(n);
    result.roi_offset_xs.resize(n);
    result.roi_offset_ys.resize(n);

    for (int i = 0; i < n; i++) {
        result.frames[i] = std::move(batch_frames[i].mat);
        result.camera_ids[i] = batch_frames[i].camera_id;
        result.img_time_secs[i] = batch_frames[i].img_time_sec;
        result.img_time_nsecs[i] = batch_frames[i].img_time_nsec;
        result.receive_local_ms_list[i] = batch_frames[i].receive_local_ms;
        result.roi_parent_slots[i] = batch_frames[i].roi_parent_slot;
        result.roi_offset_xs[i] = batch_frames[i].roi_offset_x;
        result.roi_offset_ys[i] = batch_frames[i].roi_offset_y;
    }

    return 0;
}

int NvidiaPipeline::postProcess(InferResult& result) {
    // 使用 result 内携带的 per-batch 数据（nvidia_pparams / nvidia_output），
    // 不再访问检测器内部共享缓冲，因此无需持有 mtx_，可与 pre 线程并行。
    int n = result.batch_size;
    std::vector<YoloV8BoxVec> boxes;
    auto t_post = std::chrono::steady_clock::now();
    int ret = detector_.post_process(result.nvidia_pparams, result.nvidia_output, boxes);
    if (ret != 0) {
        LOG_ERROR("[nvidia] post_process failed");
        return ret;
    }
    auto t_end = std::chrono::steady_clock::now();
    double post_ms = std::chrono::duration<double, std::milli>(t_end - t_post).count();
    g_nvidia_timing.post_ms += post_ms;
    g_nvidia_timing.print_if_due();

    // 每 20 个 batch 打印一次完整单 batch 耗时（pre+forward+post），便于观察波动与平均单帧耗时
    static int batch_log_cnt = 0;
    batch_log_cnt++;
    if (batch_log_cnt % 20 == 1) {
        double pre = g_nvidia_timing.last_pre_ms;
        double fwd = g_nvidia_timing.last_forward_ms;
        double total = pre + fwd + post_ms;
        int b = g_nvidia_timing.last_batch > 0 ? g_nvidia_timing.last_batch : n;
        LOG_DEBUG("[nvidia-batch-per] batch=%d  pre=%.2fms  forward=%.2fms  post=%.2fms  total=%.2fms  avg_per_frame=%.2fms",
                 b, pre, fwd, post_ms, total, total / b);
    }

    // 先按槽位解码检测框
    std::vector<std::vector<DetectorRetData>> slot_dets(n);
    for (int i = 0; i < n && i < (int)boxes.size(); i++) {
        bool from_roi = result.roi_parent_slots[i] >= 0;
        for (auto& box : boxes[i]) {
            DetectorRetData d;
            d.label = box.class_id + 1;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            d.from_roi = from_roi;
            slot_dets[i].push_back(d);
        }
    }

    // 合并：完整帧槽位作为基准，ROI 槽位坐标映射回原图后并入所属相机
    result.detections.assign(n, std::vector<DetectorRetData>());
    // 原图坐标下的最终检测（按相机），用于合并后的 NMS 去重
    std::vector<std::vector<DetectorRetData>> cam_dets(n);

    for (int i = 0; i < n; i++) {
        int parent = result.roi_parent_slots[i];
        if (parent < 0) {
            // 完整帧：检测结果按原图坐标直接作为该相机基准
            result.detections[i] = std::move(slot_dets[i]);
            cam_dets[i] = result.detections[i];
        } else if (parent >= 0 && parent < n) {
            // ROI 槽位：坐标 + 偏移映射回原图，并入父相机（父相机为 batch 槽位索引）
            int ox = result.roi_offset_xs[i];
            int oy = result.roi_offset_ys[i];
            for (auto& d : slot_dets[i]) {
                d.xmin += ox; d.xmax += ox;
                d.ymin += oy; d.ymax += oy;
                cam_dets[parent].push_back(d);
            }
        }
    }

    // 对每个相机做 NMS 去重（主图 + ROI 结果合并后）
    if (roi_enabled_) {
        for (int i = 0; i < n; i++) {
            if (result.roi_parent_slots[i] >= 0) continue;  // 只对完整帧做
            result.detections[i] = nmsMerge(cam_dets[i], nms_iou_threshold_);
        }
    }

    return 0;
}
