#include "rknn_test.h"
#include "rknn_api.h"
#include "postprocess.h"
#include "preprocess.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iostream>

static const std::vector<cv::Scalar> kColors = {
    {255, 0, 0},     {0, 255, 0},     {0, 0, 255},
    {255, 255, 0},   {255, 0, 255},   {0, 255, 255},
    {128, 128, 0},   {128, 0, 128},   {0, 128, 128},
    {64, 128, 255}
};

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

RknnTest::RknnTest(const std::string& model_path, int num_classes)
    : num_classes_(num_classes) {
    FILE* fp = fopen(model_path.c_str(), "rb");
    if (!fp) {
        std::cerr << "Failed to open model: " << model_path << std::endl;
        return;
    }
    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    void* model_data = malloc(model_size);
    fread(model_data, 1, model_size, fp);
    fclose(fp);

    int ret = rknn_init(&ctx_, model_data, model_size, 0, NULL);
    free(model_data);
    if (ret < 0) {
        std::cerr << "rknn_init failed: " << ret << std::endl;
        return;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        std::cerr << "rknn_query IO num failed: " << ret << std::endl;
        return;
    }
    num_outputs_ = io_num.n_output;
    std::cout << "Number of outputs: " << num_outputs_ << std::endl;

    rknn_tensor_attr input_attr;
    memset(&input_attr, 0, sizeof(input_attr));
    input_attr.index = 0;
    ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
    if (ret < 0) {
        std::cerr << "rknn_query input attr failed: " << ret << std::endl;
        return;
    }

    int h_idx = (input_attr.fmt == RKNN_TENSOR_NCHW) ? 2 : 1;
    int w_idx = (input_attr.fmt == RKNN_TENSOR_NCHW) ? 3 : 2;
    net_h_ = input_attr.dims[h_idx];
    net_w_ = input_attr.dims[w_idx];

    for (int i = 0; i < num_outputs_; i++) {
        rknn_tensor_attr out_attr;
        memset(&out_attr, 0, sizeof(out_attr));
        out_attr.index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
        if (ret == 0) {
            std::cout << "  Output[" << i << "] dims: "
                      << out_attr.dims[0] << "x" << out_attr.dims[1] << "x"
                      << out_attr.dims[2] << "x" << out_attr.dims[3]
                      << " fmt=" << (out_attr.fmt == RKNN_TENSOR_NCHW ? "NCHW" : "NHWC")
                      << std::endl;
            // Auto-detect num_classes from first output
            if (i == 0) {
                int cls_from_model = out_attr.dims[1] - 5;
                if (cls_from_model > 0 && cls_from_model < 100) {
                    num_classes_ = cls_from_model;
                }
            }
        }
    }

    std::cout << "Input dims: " << input_attr.dims[0] << "x" << input_attr.dims[1] << "x"
              << input_attr.dims[2] << "x" << input_attr.dims[3] << std::endl;
    std::cout << "RKNN model loaded: " << net_w_ << "x" << net_h_
              << " classes=" << num_classes_
              << " fmt=" << (input_attr.fmt == RKNN_TENSOR_NCHW ? "NCHW" : "NHWC")
              << std::endl;

    rknn_sdk_version sdk_ver;
    ret = rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &sdk_ver, sizeof(sdk_ver));
    if (ret == 0) {
        std::cout << "SDK version: " << sdk_ver.api_version
                  << " (driver: " << sdk_ver.drv_version << ")" << std::endl;
    }
}

RknnTest::~RknnTest() {
    if (ctx_) {
        rknn_destroy(ctx_);
    }
}

void RknnTest::letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty) {
    int src_w = img.cols;
    int src_h = img.rows;
    float r = std::min((float)net_w_ / src_w, (float)net_h_ / src_h);
    scale = r;
    int new_w = (int)(src_w * r);
    int new_h = (int)(src_h * r);
    tx = (net_w_ - new_w) / 2;
    ty = (net_h_ - new_h) / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));
    out = cv::Mat(net_h_, net_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(tx, ty, new_w, new_h)));
}

std::vector<RknnDetection> RknnTest::runInference(const cv::Mat& img) {
    std::vector<RknnDetection> detections;
    if (!ctx_ || img.empty()) return detections;

    float scale;
    int tx, ty;
    cv::Mat letterboxed;
    letterbox(img, letterboxed, scale, tx, ty);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_w_ * net_h_ * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = letterboxed.data;

    int ret = rknn_inputs_set(ctx_, 1, inputs);
    if (ret < 0) {
        std::cerr << "rknn_inputs_set failed: " << ret << std::endl;
        return detections;
    }

    ret = rknn_run(ctx_, nullptr);
    if (ret < 0) {
        std::cerr << "rknn_run failed: " << ret << std::endl;
        return detections;
    }

    std::vector<rknn_output> outputs(num_outputs_);
    memset(outputs.data(), 0, sizeof(rknn_output) * num_outputs_);
    for (int i = 0; i < num_outputs_; i++) {
        outputs[i].want_float = 0;
        outputs[i].is_prealloc = 0;
    }

    ret = rknn_outputs_get(ctx_, num_outputs_, outputs.data(), NULL);
    if (ret < 0) {
        std::cerr << "rknn_outputs_get failed: " << ret << std::endl;
        return detections;
    }

    if (use_yolov5_) {
        // Read output scales/zps for dequantization
        std::vector<float> out_scales(num_outputs_);
        std::vector<int32_t> out_zps(num_outputs_);
        for (int i = 0; i < num_outputs_; i++) {
            rknn_tensor_attr out_attr;
            memset(&out_attr, 0, sizeof(out_attr));
            out_attr.index = i;
            rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
            out_scales[i] = out_attr.scale;
            out_zps[i] = out_attr.zp;
        }

        // Use the original post_process function from the user's project
        BOX_RECT pads;
        pads.left = tx;
        pads.top = ty;
        pads.right = net_w_ - (net_w_ - (int)(img.cols * scale)) / 2 - (int)(img.cols * scale);
        pads.bottom = net_h_ - (net_h_ - (int)(img.rows * scale)) / 2 - (int)(img.rows * scale);

        float scale_w = scale;
        float scale_h = scale;

        detect_result_group_t detect_result_group;
        post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf,
                     net_h_, net_w_,
                     conf_threshold_, nms_threshold_,
                     pads, scale_w, scale_h,
                     out_zps, out_scales, &detect_result_group);

        for (int i = 0; i < detect_result_group.count; i++) {
            detect_result_t* det = &detect_result_group.results[i];
            RknnDetection d;
            d.class_id = det->class_id;
            d.confidence = det->prop;
            d.box.x = det->box.left;
            d.box.y = det->box.top;
            d.box.width = det->box.right - det->box.left;
            d.box.height = det->box.bottom - det->box.top;
            detections.push_back(d);
        }
    } else {
        // Single output detect (YOLOv8 style)
        float* data = (float*)outputs[0].buf;
        rknn_tensor_attr out_attr;
        memset(&out_attr, 0, sizeof(out_attr));
        out_attr.index = 0;
        rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
        int box_num = out_attr.dims[2];
        int nout = out_attr.dims[1];
        detections = decodeYoloV8(data, box_num, nout, scale, tx, ty, img.cols, img.rows);
    }

    rknn_outputs_release(ctx_, num_outputs_, outputs.data());
    return detections;
}

void RknnTest::runInferenceOnly(const cv::Mat& img) {
    if (!ctx_ || img.empty()) return;

    float scale;
    int tx, ty;
    cv::Mat letterboxed;
    letterbox(img, letterboxed, scale, tx, ty);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_w_ * net_h_ * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = letterboxed.data;

    rknn_inputs_set(ctx_, 1, inputs);
    rknn_run(ctx_, nullptr);

    std::vector<rknn_output> outputs(num_outputs_);
    memset(outputs.data(), 0, sizeof(rknn_output) * num_outputs_);
    for (int i = 0; i < num_outputs_; i++) {
        outputs[i].want_float = 0;
        outputs[i].is_prealloc = 0;
    }
    rknn_outputs_get(ctx_, num_outputs_, outputs.data(), NULL);
    rknn_outputs_release(ctx_, num_outputs_, outputs.data());
}

std::vector<RknnDetection> RknnTest::decodeYoloV8(
    const float* data, int box_num, int nout,
    float scale, int tx, int ty, int img_w, int img_h) {
    std::vector<RknnDetection> raw_dets;
    for (int i = 0; i < box_num; i++) {
        const float* ptr = data + i * nout;
        const float* cls_conf = ptr + 4;
        for (int j = 0; j < num_classes_; j++) {
            float score = sigmoid(cls_conf[j]);
            if (score > conf_threshold_) {
                RknnDetection d;
                d.class_id = j;
                d.confidence = score;
                float cx = ptr[0];
                float cy = ptr[1];
                float w = ptr[2];
                float h = ptr[3];
                d.box = cv::Rect(
                    (int)(cx - w / 2),
                    (int)(cy - h / 2),
                    (int)w,
                    (int)h
                );
                raw_dets.push_back(d);
            }
        }
    }

    nms(raw_dets, nms_threshold_);
    if ((int)raw_dets.size() > max_det_) {
        raw_dets.erase(raw_dets.begin(), raw_dets.begin() + (raw_dets.size() - max_det_));
    }

    float inv_scale = 1.0f / scale;
    for (auto& d : raw_dets) {
        d.box.x = (int)((d.box.x - tx) * inv_scale);
        d.box.y = (int)((d.box.y - ty) * inv_scale);
        d.box.width = (int)(d.box.width * inv_scale);
        d.box.height = (int)(d.box.height * inv_scale);

        d.box.x = std::max(0, std::min(d.box.x, img_w - 1));
        d.box.y = std::max(0, std::min(d.box.y, img_h - 1));
        d.box.width = std::max(1, std::min(d.box.width, img_w - d.box.x));
        d.box.height = std::max(1, std::min(d.box.height, img_h - d.box.y));
    }

    return raw_dets;
}

void RknnTest::nms(std::vector<RknnDetection>& dets, float nmsConfidence) {
    int length = dets.size();
    if (length == 0) return;

    std::sort(dets.begin(), dets.end(), [](const RknnDetection& a, const RknnDetection& b) {
        return a.confidence < b.confidence;
    });

    std::vector<float> areas(length);
    for (int i = 0; i < length; i++) {
        areas[i] = (float)(dets[i].box.width * dets[i].box.height);
    }

    int index = length - 1;
    while (index > 0) {
        int i = 0;
        while (i < index) {
            float left = std::max((float)dets[index].box.x, (float)dets[i].box.x);
            float top = std::max((float)dets[index].box.y, (float)dets[i].box.y);
            float right = std::min((float)(dets[index].box.x + dets[index].box.width),
                                   (float)(dets[i].box.x + dets[i].box.width));
            float bottom = std::min((float)(dets[index].box.y + dets[index].box.height),
                                    (float)(dets[i].box.y + dets[i].box.height));
            float overlap = std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
            if (overlap / (areas[index] + areas[i] - overlap) > nmsConfidence) {
                areas.erase(areas.begin() + i);
                dets.erase(dets.begin() + i);
                index--;
            } else {
                i++;
            }
        }
        index--;
    }
}

cv::Mat RknnTest::drawResults(const cv::Mat& img, const std::vector<RknnDetection>& detections) {
    cv::Mat out = img.clone();
    for (auto& d : detections) {
        cv::Scalar color = kColors[d.class_id % kColors.size()];
        cv::rectangle(out, d.box, color, 2);

        std::string label = "cls" + std::to_string(d.class_id) + " "
                          + std::to_string(d.confidence).substr(0, 4);
        int baseline;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Rect textBox(d.box.x, d.box.y - textSize.height - 5,
                         textSize.width + 4, textSize.height + 4);
        cv::rectangle(out, textBox, color, cv::FILLED);
        cv::putText(out, label,
                    cv::Point(d.box.x + 2, d.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    return out;
}