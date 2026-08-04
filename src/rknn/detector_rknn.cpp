#include "detector_rknn.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

static const std::vector<std::vector<int>> colors = {
    {255, 0, 0},     {255, 85, 0},    {255, 170, 0},   {255, 255, 0}, {170, 255, 0}, {85, 255, 0},  {0, 255, 0},
    {0, 255, 85},    {0, 255, 170},   {0, 255, 255},   {0, 170, 255}, {0, 85, 255},  {0, 0, 255},   {85, 0, 255},
    {170, 0, 255},   {255, 0, 255},   {255, 0, 170},   {255, 0, 85},  {255, 0, 0},   {255, 0, 255}, {255, 85, 255},
    {255, 170, 255}, {255, 255, 255}, {170, 255, 255}, {85, 255, 255}};

static inline int32_t __clip(float val, float min, float max) {
    float f = val <= min ? min : (val >= max ? max : val);
    return (int32_t)f;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0,
                              float xmin1, float ymin1, float xmax1, float ymax1) {
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) +
              (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
    return u <= 0.f ? 0.f : (i / u);
}

int8_t YoloV8_det::qnt_f32_to_affine(float f32, int32_t zp, float scale) {
    float dst_val = (f32 / scale) + zp;
    return (int8_t)__clip(dst_val, -128, 127);
}

float YoloV8_det::deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
    return ((float)qnt - (float)zp) * scale;
}

int YoloV8_det::Init(const std::string& model_path) {
    FILE* fp = fopen(model_path.c_str(), "rb");
    if (!fp) {
        std::cerr << "Failed to open model: " << model_path << std::endl;
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    size_t model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    void* model_data = malloc(model_size);
    size_t nread = fread(model_data, 1, model_size, fp);
    (void)nread;
    fclose(fp);

    int ret = rknn_init(&ctx, model_data, model_size, 0, NULL);
    free(model_data);
    if (ret < 0) {
        std::cerr << "rknn_init failed: " << ret << std::endl;
        return ret;
    }

    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        std::cerr << "rknn_query IO num failed: " << ret << std::endl;
        return ret;
    }

    input_attrs = new rknn_tensor_attr[io_num.n_input];
    memset(input_attrs, 0, sizeof(rknn_tensor_attr) * io_num.n_input);
    for (uint32_t i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
        if (ret < 0) {
            std::cerr << "rknn_query input attr failed: " << ret << std::endl;
            return ret;
        }
    }

    output_attrs = new rknn_tensor_attr[io_num.n_output];
    memset(output_attrs, 0, sizeof(rknn_tensor_attr) * io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
        if (ret < 0) {
            std::cerr << "rknn_query output attr failed: " << ret << std::endl;
            return ret;
        }
    }

    num_outputs_ = io_num.n_output;

    int h_idx = input_attrs[0].fmt == RKNN_TENSOR_NCHW ? 2 : 1;
    int w_idx = input_attrs[0].fmt == RKNN_TENSOR_NCHW ? 3 : 2;
    m_net_h = input_attrs[0].dims[h_idx];
    m_net_w = input_attrs[0].dims[w_idx];

    // Auto-detect num_classes from first output: dims[1] = 3 * (5 + num_classes)
    if (io_num.n_output > 0) {
        int channels = output_attrs[0].dims[1];
        int cls = channels / 3 - 5;
        if (cls > 0 && cls < 100) {
            m_class_num = cls;
        }
    }

    std::cout << "RKNN YOLOv5 model loaded: size=" << m_net_w << "x" << m_net_h
              << " classes=" << m_class_num
              << " outputs=" << io_num.n_output
              << " fmt=" << (input_attrs[0].fmt == RKNN_TENSOR_NCHW ? "NCHW" : "NHWC")
              << std::endl;

    return 0;
}

void YoloV8_det::letterbox(const cv::Mat& img, cv::Mat& out, float& scale, int& tx, int& ty) {
    int src_w = img.cols;
    int src_h = img.rows;
    float r = std::min((float)m_net_w / src_w, (float)m_net_h / src_h);
    scale = r;
    int new_w = (int)(src_w * r);
    int new_h = (int)(src_h * r);
    tx = (m_net_w - new_w) / 2;
    ty = (m_net_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h));
    out = cv::Mat(m_net_h, m_net_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(tx, ty, new_w, new_h)));
}

int YoloV8_det::process(int8_t* input, const int* anchor, int grid_h, int grid_w,
                        int stride, std::vector<float>& boxes, std::vector<float>& objProbs,
                        std::vector<int>& classId, float threshold, int32_t zp, float scale,
                        int num_classes, bool use_sigmoid) {
    int box_size = 5 + num_classes;
    int validCount = 0;
    int grid_len = grid_h * grid_w;
    int8_t thres_i8 = qnt_f32_to_affine(threshold, zp, scale);
    for (int a = 0; a < 3; a++) {
        for (int i = 0; i < grid_h; i++) {
            for (int j = 0; j < grid_w; j++) {
                int8_t box_confidence = input[(box_size * a + 4) * grid_len + i * grid_w + j];
                if (box_confidence >= thres_i8) {
                    int offset = (box_size * a) * grid_len + i * grid_w + j;
                    int8_t* in_ptr = input + offset;
                    float box_x = apply_activation(deqnt_affine_to_f32(*in_ptr, zp, scale), use_sigmoid) * 2.0 - 0.5;
                    float box_y = apply_activation(deqnt_affine_to_f32(in_ptr[grid_len], zp, scale), use_sigmoid) * 2.0 - 0.5;
                    float box_w = apply_activation(deqnt_affine_to_f32(in_ptr[2 * grid_len], zp, scale), use_sigmoid) * 2.0;
                    float box_h = apply_activation(deqnt_affine_to_f32(in_ptr[3 * grid_len], zp, scale), use_sigmoid) * 2.0;
                    box_w = box_w * box_w;
                    box_h = box_h * box_h;
                    box_x = (box_x + j) * (float)stride;
                    box_y = (box_y + i) * (float)stride;
                    box_w = box_w * (float)anchor[a * 2];
                    box_h = box_h * (float)anchor[a * 2 + 1];
                    box_x -= (box_w / 2.0);
                    box_y -= (box_h / 2.0);

                    int8_t maxClassProbs = in_ptr[5 * grid_len];
                    int maxClassId = 0;
                    for (int k = 1; k < num_classes; ++k) {
                        int8_t prob = in_ptr[(5 + k) * grid_len];
                        if (prob > maxClassProbs) {
                            maxClassId = k;
                            maxClassProbs = prob;
                        }
                    }
                    if (maxClassProbs > thres_i8) {
                        float obj_conf = apply_activation(deqnt_affine_to_f32(box_confidence, zp, scale), use_sigmoid);
                        float cls_conf = apply_activation(deqnt_affine_to_f32(maxClassProbs, zp, scale), use_sigmoid);
                        objProbs.push_back(cls_conf * obj_conf);
                        classId.push_back(maxClassId);
                        validCount++;
                        boxes.push_back(box_x);
                        boxes.push_back(box_y);
                        boxes.push_back(box_w);
                        boxes.push_back(box_h);
                    }
                }
            }
        }
    }
    return validCount;
}

int YoloV8_det::quick_sort_indice_inverse(std::vector<float>& input, int left, int right,
                                           std::vector<int>& indices) {
    float key;
    int key_index;
    int low = left;
    int high = right;
    if (left < right) {
        key_index = indices[left];
        key = input[left];
        while (low < high) {
            while (low < high && input[high] <= key) {
                high--;
            }
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key) {
                low++;
            }
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = key_index;
        quick_sort_indice_inverse(input, left, low - 1, indices);
        quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

int YoloV8_det::nms(int validCount, std::vector<float>& outputLocations, std::vector<int>& classIds,
                     std::vector<int>& order, int filterId, float threshold) {
    for (int i = 0; i < validCount; ++i) {
        int n = order[i];
        if (n == -1) {
            continue;
        }
        for (int j = i + 1; j < validCount; ++j) {
            int m = order[j];
            if (m == -1) {
                continue;
            }
            float xmin0 = outputLocations[n * 4 + 0];
            float ymin0 = outputLocations[n * 4 + 1];
            float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
            float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];
            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
            float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

            float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);
            if (iou > threshold) {
                order[j] = -1;
            }
        }
    }
    return 0;
}

int YoloV8_det::post_process(int8_t* input0, int8_t* input1, int8_t* input2,
                              int model_in_h, int model_in_w,
                              int num_classes, float conf_threshold, float nms_threshold,
                              BOX_RECT pads, float scale_w, float scale_h,
                              std::vector<int32_t>& qnt_zps, std::vector<float>& qnt_scales,
                              detect_result_group_t* group, bool use_sigmoid) {
    memset(group, 0, sizeof(detect_result_group_t));

    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;

    int stride0 = 8;
    int grid_h0 = model_in_h / stride0;
    int grid_w0 = model_in_w / stride0;
    int validCount0 = process(input0, (int*)anchor0, grid_h0, grid_w0, stride0,
                              filterBoxes, objProbs, classId, conf_threshold,
                              qnt_zps[0], qnt_scales[0], num_classes, use_sigmoid);

    int stride1 = 16;
    int grid_h1 = model_in_h / stride1;
    int grid_w1 = model_in_w / stride1;
    int validCount1 = process(input1, (int*)anchor1, grid_h1, grid_w1, stride1,
                              filterBoxes, objProbs, classId, conf_threshold,
                              qnt_zps[1], qnt_scales[1], num_classes, use_sigmoid);

    int stride2 = 32;
    int grid_h2 = model_in_h / stride2;
    int grid_w2 = model_in_w / stride2;
    int validCount2 = process(input2, (int*)anchor2, grid_h2, grid_w2, stride2,
                              filterBoxes, objProbs, classId, conf_threshold,
                              qnt_zps[2], qnt_scales[2], num_classes, use_sigmoid);

    int validCount = validCount0 + validCount1 + validCount2;
    if (validCount <= 0) {
        return 0;
    }

    std::vector<int> indexArray;
    for (int i = 0; i < validCount; ++i) {
        indexArray.push_back(i);
    }

    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    nms(validCount, filterBoxes, classId, indexArray, 0, nms_threshold);

    int last_count = 0;
    group->count = 0;
    for (int i = 0; i < validCount; ++i) {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
            continue;
        }
        int n = indexArray[i];

        float x1 = filterBoxes[n * 4 + 0] - pads.left;
        float y1 = filterBoxes[n * 4 + 1] - pads.top;
        float x2 = x1 + filterBoxes[n * 4 + 2];
        float y2 = y1 + filterBoxes[n * 4 + 3];
        int id = classId[n];
        float obj_conf = objProbs[i];

        group->results[last_count].box.left = (int)(std::max(0.0f, std::min(x1, (float)model_in_w)) / scale_w);
        group->results[last_count].box.top = (int)(std::max(0.0f, std::min(y1, (float)model_in_h)) / scale_h);
        group->results[last_count].box.right = (int)(std::max(0.0f, std::min(x2, (float)model_in_w)) / scale_w);
        group->results[last_count].box.bottom = (int)(std::max(0.0f, std::min(y2, (float)model_in_h)) / scale_h);
        group->results[last_count].prop = obj_conf;
        group->results[last_count].class_id = id;

        last_count++;
    }
    group->count = last_count;

    return 0;
}

int YoloV8_det::Detect(const cv::Mat& img, YoloV8BoxVec& boxes) {
    if (!ctx || img.empty()) return -1;

    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

    cv::Mat model_input;
    float scale_x = (float)img.cols / m_net_w;
    float scale_y = (float)img.rows / m_net_h;

    if (use_letterbox_) {
        letterbox(rgb, model_input, letterbox_scale_, pad_left_, pad_top_);
        scale_x = 1.0f / letterbox_scale_;
        scale_y = 1.0f / letterbox_scale_;
    } else {
        cv::resize(rgb, model_input, cv::Size(m_net_w, m_net_h));
        pad_left_ = 0;
        pad_top_ = 0;
        letterbox_scale_ = 1.0f;
        scale_x = (float)img.cols / m_net_w;
        scale_y = (float)img.rows / m_net_h;
    }

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = m_net_w * m_net_h * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = model_input.data;

    int ret = rknn_inputs_set(ctx, 1, inputs);
    if (ret < 0) {
        std::cerr << "rknn_inputs_set failed: " << ret << std::endl;
        return ret;
    }

    ret = rknn_run(ctx, nullptr);
    if (ret < 0) {
        std::cerr << "rknn_run failed: " << ret << std::endl;
        return ret;
    }

    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        outputs[i].want_float = 0;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
    if (ret < 0) {
        std::cerr << "rknn_outputs_get failed: " << ret << std::endl;
        return ret;
    }

    std::vector<float> out_scales(io_num.n_output);
    std::vector<int32_t> out_zps(io_num.n_output);
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        out_scales[i] = output_attrs[i].scale;
        out_zps[i] = output_attrs[i].zp;
    }

    int num_classes = m_class_num;
    // Re-derive num_classes from output channels
    {
        int channels = output_attrs[0].dims[1];
        int cls = channels / 3 - 5;
        if (cls > 0 && cls < 100) num_classes = cls;
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
                 m_net_h, m_net_w, num_classes,
                 m_confThreshold, m_nmsThreshold,
                 pads, scale_w, scale_h,
                 out_zps, out_scales, &detect_result_group, use_sigmoid_);

    boxes.clear();
    for (int i = 0; i < detect_result_group.count; i++) {
        detect_result_t* det = &detect_result_group.results[i];
        YoloV8Box box;
        box.class_id = det->class_id;
        box.score = det->prop;
        box.x1 = (float)det->box.left * scale_x;
        box.y1 = (float)det->box.top * scale_y;
        box.x2 = (float)det->box.right * scale_x;
        box.y2 = (float)det->box.bottom * scale_y;
        boxes.push_back(box);
    }

    if ((int)boxes.size() > max_det) {
        boxes.erase(boxes.begin(), boxes.begin() + (boxes.size() - max_det));
    }

    rknn_outputs_release(ctx, io_num.n_output, outputs);

    return 0;
}

void YoloV8_det::NMS(YoloV8BoxVec& dets, float nmsConfidence) {
    int length = dets.size();
    if (length == 0) return;

    std::sort(dets.begin(), dets.end(), [](const YoloV8Box& a, const YoloV8Box& b) {
        return a.score < b.score;
    });

    std::vector<float> areas(length);
    for (int i = 0; i < length; i++) {
        areas[i] = (dets[i].x2 - dets[i].x1) * (dets[i].y2 - dets[i].y1);
    }

    int index = length - 1;
    while (index > 0) {
        int i = 0;
        while (i < index) {
            float left = std::max(dets[index].x1, dets[i].x1);
            float top = std::max(dets[index].y1, dets[i].y1);
            float right = std::min(dets[index].x2, dets[i].x2);
            float bottom = std::min(dets[index].y2, dets[i].y2);
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

void YoloV8_det::clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h) {
    for (auto& box : yolobox_vec) {
        box.x1 = std::max(0.0f, std::min(box.x1, (float)src_w));
        box.y1 = std::max(0.0f, std::min(box.y1, (float)src_h));
        box.x2 = std::max(0.0f, std::min(box.x2, (float)src_w));
        box.y2 = std::max(0.0f, std::min(box.y2, (float)src_h));
    }
}

void YoloV8_det::draw_result(cv::Mat& img, YoloV8BoxVec& result) {
    for (auto& box : result) {
        if (box.score < 0.25) continue;
        cv::Scalar color(colors[box.class_id % 25][0],
                         colors[box.class_id % 25][1],
                         colors[box.class_id % 25][2]);
        cv::Rect bound = {(int)box.x1, (int)box.y1,
                          (int)(box.x2 - box.x1), (int)(box.y2 - box.y1)};
        cv::rectangle(img, bound, color, 2);
        std::string label = std::to_string(box.class_id) + " " + std::to_string(box.score).substr(0, 4);
        cv::putText(img, label, cv::Point((int)box.x1, (int)box.y1),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
}

YoloV8_det::~YoloV8_det() {
    if (ctx) {
        rknn_destroy(ctx);
    }
    delete[] input_attrs;
    delete[] output_attrs;
}