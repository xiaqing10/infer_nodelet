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
    size_t nread = fread(model_data, 1, model_size, fp);
    (void)nread;
    fclose(fp);

    int ret = rknn_init(&ctx_, model_data, model_size, 0, NULL);
    free(model_data);
    if (ret < 0) {
        std::cerr << "rknn_init failed: " << ret << std::endl;
        return;
    }

    ret = rknn_set_core_mask(ctx_, core_mask_);
    if (ret < 0) {
        std::cerr << "rknn_set_core_mask failed: " << ret << ", fallback to auto" << std::endl;
    } else {
        std::cout << "RKNN core mask set successfully" << std::endl;
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

    output_attrs_.resize(num_outputs_);
    for (int i = 0; i < num_outputs_; i++) {
        memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
        output_attrs_[i].index = i;
        ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
        if (ret == 0) {
            auto& oa = output_attrs_[i];
            std::cout << "  Output[" << i << "] dims: "
                      << oa.dims[0] << "x" << oa.dims[1] << "x"
                      << oa.dims[2] << "x" << oa.dims[3]
                      << " fmt=" << (oa.fmt == RKNN_TENSOR_NCHW ? "NCHW" : "NHWC")
                      << " type=" << (oa.type == RKNN_TENSOR_INT8 ? "INT8" : "FP32")
                      << " qnt=" << (oa.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ? "AFFINE" : "NONE")
                      << " zp=" << oa.zp << " scale=" << oa.scale
                      << std::endl;
            if (i == 0) {
                if (oa.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && oa.type == RKNN_TENSOR_INT8) {
                    is_quant_ = true;
                }
                if (num_outputs_ >= 3) {
                    int cls_from_model = output_attrs_[1].dims[1];
                    if (cls_from_model > 0 && cls_from_model < 100) {
                        num_classes_ = cls_from_model;
                    }
                }
            }
        }
    }
    if (num_outputs_ >= 3) {
        output_per_branch_ = num_outputs_ / 3;
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

void RknnTest::setCoreMask(const std::string& mask_str) {
    if (mask_str == "auto") {
        core_mask_ = RKNN_NPU_CORE_AUTO;
    } else if (mask_str == "0") {
        core_mask_ = RKNN_NPU_CORE_0;
    } else if (mask_str == "1") {
        core_mask_ = RKNN_NPU_CORE_1;
    } else if (mask_str == "2") {
        core_mask_ = RKNN_NPU_CORE_2;
    } else if (mask_str == "0_1") {
        core_mask_ = RKNN_NPU_CORE_0_1;
    } else if (mask_str == "0_1_2") {
        core_mask_ = RKNN_NPU_CORE_0_1_2;
    } else {
        std::cerr << "Unknown core mask: " << mask_str << ", using default" << std::endl;
    }
    std::cout << "RKNN core mask set to: " << mask_str << std::endl;
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

    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    cv::Mat model_input;
    float scale_x = (float)img.cols / net_w_;
    float scale_y = (float)img.rows / net_h_;

    if (use_letterbox_) {
        // Letterbox: keep aspect ratio + gray padding
        letterbox(rgb, model_input, letterbox_scale_, pad_left_, pad_top_);
        // Coordinates in 640x640 space → map back to original: (x - pad) / letterbox_scale
        scale_x = 1.0f / letterbox_scale_;
        scale_y = 1.0f / letterbox_scale_;
    } else {
        // Direct resize (same as Python convert.py)
        cv::resize(rgb, model_input, cv::Size(net_w_, net_h_));
        pad_left_ = 0;
        pad_top_ = 0;
        letterbox_scale_ = 1.0f;
        // Coordinates in 640x640 space → map back to original: x * (img_w / 640)
        scale_x = (float)img.cols / net_w_;
        scale_y = (float)img.rows / net_h_;
    }

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_w_ * net_h_ * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = model_input.data;

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
        outputs[i].want_float = is_quant_ ? 0 : 1;
        outputs[i].is_prealloc = 0;
    }

    ret = rknn_outputs_get(ctx_, num_outputs_, outputs.data(), NULL);
    if (ret < 0) {
        std::cerr << "rknn_outputs_get failed: " << ret << std::endl;
        return detections;
    }

    if (use_yolov5_) {
        std::vector<float> out_scales(num_outputs_);
        std::vector<int32_t> out_zps(num_outputs_);
        int num_classes = num_classes_;
        for (int i = 0; i < num_outputs_; i++) {
            rknn_tensor_attr out_attr;
            memset(&out_attr, 0, sizeof(out_attr));
            out_attr.index = i;
            rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
            out_scales[i] = out_attr.scale;
            out_zps[i] = out_attr.zp;
            if (i == 0) {
                int channels = out_attr.dims[1];
                num_classes = channels / 3 - 5;
                if (num_classes < 1 || num_classes > 100) num_classes = num_classes_;
            }
        }

        BOX_RECT pads;
        memset(&pads, 0, sizeof(pads));
        float scale_w = 1.0f;
        float scale_h = 1.0f;

        if (use_letterbox_) {
            pads.left = pad_left_;
            pads.top = pad_top_;
            pads.right = pad_left_;
            pads.bottom = pad_top_;
            scale_w = 1.0f / letterbox_scale_;
            scale_h = 1.0f / letterbox_scale_;
        }

        detect_result_group_t detect_result_group;
        post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf,
                     net_h_, net_w_, num_classes,
                     conf_threshold_, nms_threshold_,
                     pads, scale_w, scale_h,
                     out_zps, out_scales, &detect_result_group, use_sigmoid_);

        for (int i = 0; i < detect_result_group.count; i++) {
            detect_result_t* det = &detect_result_group.results[i];
            RknnDetection d;
            d.class_id = det->class_id;
            d.confidence = det->prop;
            d.box.x = (int)(det->box.left * scale_x);
            d.box.y = (int)(det->box.top * scale_y);
            d.box.width = (int)((det->box.right - det->box.left) * scale_x);
            d.box.height = (int)((det->box.bottom - det->box.top) * scale_y);
            detections.push_back(d);
        }
    } else {
        if (num_outputs_ >= 3) {
            detections = decodeYoloV8DFL(outputs.data(), scale_x, scale_y, img.cols, img.rows);
        } else {
            rknn_tensor_attr out_attr;
            memset(&out_attr, 0, sizeof(out_attr));
            out_attr.index = 0;
            rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
            int box_num = out_attr.dims[2];
            int nout = out_attr.dims[1];
            int nc = nout - 4;
            if (nc > 0 && nc < 100) num_classes_ = nc;

            bool is_quant = (out_attr.type == RKNN_TENSOR_INT8);

            std::vector<float> dequant_buf;
            if (is_quant) {
                dequant_buf.resize(box_num * nout);
                int8_t* i8_data = (int8_t*)outputs[0].buf;
                float scale = out_attr.scale;
                int32_t zp = out_attr.zp;
                for (int i = 0; i < box_num * nout; i++) {
                    dequant_buf[i] = ((float)i8_data[i] - (float)zp) * scale;
                }
            }
            const float* data = is_quant ? dequant_buf.data() : (float*)outputs[0].buf;

            detections = decodeYoloV8Merged(data, box_num, nout, 1.0f, 0, 0, img.cols, img.rows);
            float inv_scale_w = (float)img.cols / net_w_;
            float inv_scale_h = (float)img.rows / net_h_;
            for (auto& d : detections) {
                d.box.x = (int)(d.box.x * inv_scale_w);
                d.box.y = (int)(d.box.y * inv_scale_h);
                d.box.width = (int)(d.box.width * inv_scale_w);
                d.box.height = (int)(d.box.height * inv_scale_h);
            }
        }
    }

    rknn_outputs_release(ctx_, num_outputs_, outputs.data());
    return detections;
}

void RknnTest::runInferenceOnly(const cv::Mat& img) {
    if (!ctx_ || img.empty()) return;

    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    cv::Mat model_input;
    if (use_letterbox_) {
        letterbox(rgb, model_input, letterbox_scale_, pad_left_, pad_top_);
    } else {
        cv::resize(rgb, model_input, cv::Size(net_w_, net_h_));
    }

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_w_ * net_h_ * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = model_input.data;

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

std::vector<RknnDetection> RknnTest::decodeYoloV8Merged(
    const float* data, int box_num, int nout,
    float scale, int tx, int ty, int img_w, int img_h) {
    std::vector<RknnDetection> raw_dets;
    int nc = nout - 4;
    int grids = box_num;
    int stride = 640 / (int)std::sqrt(grids);
    int grid_h = (int)std::sqrt(grids);
    int grid_w = grid_h;

    for (int i = 0; i < box_num; i++) {
        const float* ptr = data + i * nout;
        float cx = (sigmoid(ptr[0]) * 2.0f - 0.5f + (i % grid_w)) * stride;
        float cy = (sigmoid(ptr[1]) * 2.0f - 0.5f + (i / grid_w)) * stride;
        float w = std::pow(sigmoid(ptr[2]) * 2.0f, 2) * stride;
        float h = std::pow(sigmoid(ptr[3]) * 2.0f, 2) * stride;

        const float* cls_conf = ptr + 4;
        for (int j = 0; j < nc; j++) {
            float score = sigmoid(cls_conf[j]);
            if (score > conf_threshold_) {
                RknnDetection d;
                d.class_id = j;
                d.confidence = score;
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

static inline int32_t clip_f(float val, float min, float max) {
    float f = val <= min ? min : (val >= max ? max : val);
    return (int32_t)f;
}

static void compute_dfl(const float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; b++) {
        float exp_t[16];
        float exp_sum = 0;
        float acc_sum = 0;
        for (int i = 0; i < dfl_len; i++) {
            exp_t[i] = std::exp(tensor[i + b * dfl_len]);
            exp_sum += exp_t[i];
        }
        for (int i = 0; i < dfl_len; i++) {
            acc_sum += exp_t[i] / exp_sum * i;
        }
        box[b] = acc_sum;
    }
}

static int process_yolov8_branch_i8(
    int8_t* box_tensor, int32_t box_zp, float box_scale,
    int8_t* score_tensor, int32_t score_zp, float score_scale,
    int8_t* score_sum_tensor, int32_t score_sum_zp, float score_sum_scale,
    int grid_h, int grid_w, int stride, int dfl_len, int num_classes,
    float threshold,
    std::vector<float>& boxes, std::vector<float>& objProbs, std::vector<int>& classId) {

    int validCount = 0;
    int grid_len = grid_h * grid_w;
    int8_t score_thres = (int8_t)(clip_f(threshold / score_scale + score_zp, -128, 127));
    int8_t score_sum_thres = (int8_t)(clip_f(threshold / score_sum_scale + score_sum_zp, -128, 127));

    for (int i = 0; i < grid_h; i++) {
        for (int j = 0; j < grid_w; j++) {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            if (score_sum_tensor != nullptr) {
                if (score_sum_tensor[offset] < score_sum_thres) continue;
            }

            int8_t max_score = -score_zp;
            for (int c = 0; c < num_classes; c++) {
                int idx = offset + c * grid_len;
                if (score_tensor[idx] > score_thres && score_tensor[idx] > max_score) {
                    max_score = score_tensor[idx];
                    max_class_id = c;
                }
            }

            if (max_score > score_thres) {
                int box_offset = offset;
                float dfl_input[4 * 16];
                for (int k = 0; k < dfl_len * 4; k++) {
                    dfl_input[k] = ((float)box_tensor[box_offset] - (float)box_zp) * box_scale;
                    box_offset += grid_len;
                }
                float box[4];
                compute_dfl(dfl_input, dfl_len, box);

                float x1 = (-box[0] + j + 0.5f) * stride;
                float y1 = (-box[1] + i + 0.5f) * stride;
                float x2 = (box[2] + j + 0.5f) * stride;
                float y2 = (box[3] + i + 0.5f) * stride;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(x2 - x1);
                boxes.push_back(y2 - y1);

                objProbs.push_back(((float)max_score - (float)score_zp) * score_scale);
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

static int process_yolov8_branch_fp32(
    float* box_tensor, float* score_tensor, float* score_sum_tensor,
    int grid_h, int grid_w, int stride, int dfl_len, int num_classes,
    float threshold,
    std::vector<float>& boxes, std::vector<float>& objProbs, std::vector<int>& classId) {

    int validCount = 0;
    int grid_len = grid_h * grid_w;

    for (int i = 0; i < grid_h; i++) {
        for (int j = 0; j < grid_w; j++) {
            int offset = i * grid_w + j;
            int max_class_id = -1;

            if (score_sum_tensor != nullptr) {
                if (score_sum_tensor[offset] < threshold) continue;
            }

            float max_score = 0;
            for (int c = 0; c < num_classes; c++) {
                int idx = offset + c * grid_len;
                if (score_tensor[idx] > threshold && score_tensor[idx] > max_score) {
                    max_score = score_tensor[idx];
                    max_class_id = c;
                }
            }

            if (max_score > threshold) {
                int box_offset = offset;
                float dfl_input[4 * 16];
                for (int k = 0; k < dfl_len * 4; k++) {
                    dfl_input[k] = box_tensor[box_offset];
                    box_offset += grid_len;
                }
                float box[4];
                compute_dfl(dfl_input, dfl_len, box);

                float x1 = (-box[0] + j + 0.5f) * stride;
                float y1 = (-box[1] + i + 0.5f) * stride;
                float x2 = (box[2] + j + 0.5f) * stride;
                float y2 = (box[3] + i + 0.5f) * stride;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(x2 - x1);
                boxes.push_back(y2 - y1);

                objProbs.push_back(max_score);
                classId.push_back(max_class_id);
                validCount++;
            }
        }
    }
    return validCount;
}

std::vector<RknnDetection> RknnTest::decodeYoloV8DFL(
    rknn_output* outputs, float scale_x, float scale_y, int img_w, int img_h) {

    std::vector<RknnDetection> detections;
    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;
    int validCount = 0;

    int dfl_len = output_attrs_[0].dims[1] / 4;

    for (int i = 0; i < 3; i++) {
        int box_idx = i * output_per_branch_;
        int score_idx = i * output_per_branch_ + 1;

        void* score_sum = nullptr;
        int32_t score_sum_zp = 0;
        float score_sum_scale = 1.0f;
        if (output_per_branch_ == 3) {
            int sum_idx = i * output_per_branch_ + 2;
            score_sum = outputs[sum_idx].buf;
            score_sum_zp = output_attrs_[sum_idx].zp;
            score_sum_scale = output_attrs_[sum_idx].scale;
        }

        int grid_h = output_attrs_[box_idx].dims[2];
        int grid_w = output_attrs_[box_idx].dims[3];
        int stride = net_h_ / grid_h;

        if (is_quant_) {
            validCount += process_yolov8_branch_i8(
                (int8_t*)outputs[box_idx].buf,
                output_attrs_[box_idx].zp, output_attrs_[box_idx].scale,
                (int8_t*)outputs[score_idx].buf,
                output_attrs_[score_idx].zp, output_attrs_[score_idx].scale,
                (int8_t*)score_sum, score_sum_zp, score_sum_scale,
                grid_h, grid_w, stride, dfl_len, num_classes_,
                conf_threshold_,
                filterBoxes, objProbs, classId);
        } else {
            validCount += process_yolov8_branch_fp32(
                (float*)outputs[box_idx].buf,
                (float*)outputs[score_idx].buf,
                (float*)score_sum,
                grid_h, grid_w, stride, dfl_len, num_classes_,
                conf_threshold_,
                filterBoxes, objProbs, classId);
        }
    }

    if (validCount <= 0) return detections;

    std::vector<int> indexArray(validCount);
    for (int i = 0; i < validCount; i++) indexArray[i] = i;

    // Sort by confidence descending
    for (int i = 0; i < validCount; i++) {
        for (int j = i + 1; j < validCount; j++) {
            if (objProbs[indexArray[i]] < objProbs[indexArray[j]]) {
                std::swap(indexArray[i], indexArray[j]);
            }
        }
    }

    // Per-class NMS
    for (int i = 0; i < validCount; i++) {
        int n = indexArray[i];
        if (n == -1) continue;
        for (int j = i + 1; j < validCount; j++) {
            int m = indexArray[j];
            if (m == -1) continue;
            if (classId[n] != classId[m]) continue;

            float xmin0 = filterBoxes[n * 4 + 0];
            float ymin0 = filterBoxes[n * 4 + 1];
            float xmax0 = xmin0 + filterBoxes[n * 4 + 2];
            float ymax0 = ymin0 + filterBoxes[n * 4 + 3];
            float xmin1 = filterBoxes[m * 4 + 0];
            float ymin1 = filterBoxes[m * 4 + 1];
            float xmax1 = xmin1 + filterBoxes[m * 4 + 2];
            float ymax1 = ymin1 + filterBoxes[m * 4 + 3];

            float w = std::max(0.0f, std::min(xmax0, xmax1) - std::max(xmin0, xmin1));
            float h = std::max(0.0f, std::min(ymax0, ymax1) - std::max(ymin0, ymin1));
            float iou = w * h / ((xmax0 - xmin0) * (ymax0 - ymin0) + (xmax1 - xmin1) * (ymax1 - ymin1) - w * h);

            if (iou > nms_threshold_) {
                indexArray[j] = -1;
            }
        }
    }

    // Letterbox: get pad from letterbox_scale_ and pad_left_/pad_top_
    float lx = (float)pad_left_;
    float ly = (float)pad_top_;
    float ls = letterbox_scale_;

    if (!use_letterbox_) {
        lx = 0;
        ly = 0;
        ls = 1.0f;
    }

    for (int i = 0; i < validCount && (int)detections.size() < max_det_; i++) {
        int n = indexArray[i];
        if (n == -1) continue;

        float x1 = filterBoxes[n * 4 + 0];
        float y1 = filterBoxes[n * 4 + 1];
        float w = filterBoxes[n * 4 + 2];
        float h = filterBoxes[n * 4 + 3];

        if (use_letterbox_) {
            x1 = (x1 - lx) / ls;
            y1 = (y1 - ly) / ls;
            w = w / ls;
            h = h / ls;
        } else {
            x1 = x1 * scale_x;
            y1 = y1 * scale_y;
            w = w * scale_x;
            h = h * scale_y;
        }

        x1 = clip_f(x1, 0, img_w);
        y1 = clip_f(y1, 0, img_h);
        w = clip_f(w, 1, img_w - (int)x1);
        h = clip_f(h, 1, img_h - (int)y1);

        RknnDetection d;
        d.class_id = classId[n];
        d.confidence = objProbs[n];
        d.box = cv::Rect((int)x1, (int)y1, (int)w, (int)h);
        detections.push_back(d);
    }

    return detections;
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