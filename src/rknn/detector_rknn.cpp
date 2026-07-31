#include "detector_rknn.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

static const std::vector<std::vector<int>> colors = {
    {255, 0, 0},     {255, 85, 0},    {255, 170, 0},   {255, 255, 0}, {170, 255, 0}, {85, 255, 0},  {0, 255, 0},
    {0, 255, 85},    {0, 255, 170},   {0, 255, 255},   {0, 170, 255}, {0, 85, 255},  {0, 0, 255},   {85, 0, 255},
    {170, 0, 255},   {255, 0, 255},   {255, 0, 170},   {255, 0, 85},  {255, 0, 0},   {255, 0, 255}, {255, 85, 255},
    {255, 170, 255}, {255, 255, 255}, {170, 255, 255}, {85, 255, 255}};

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

int YoloV8_det::Init(const std::string& model_path) {
    int ret = rknn_init(&ctx, (void*)model_path.c_str(), 0, 0, NULL);
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

    // NCHW: [batch, 3, 640, 640]
    batch_size = input_attrs[0].dims[0];
    int h_idx = input_attrs[0].fmt == RKNN_TENSOR_NCHW ? 2 : 1;
    int w_idx = input_attrs[0].fmt == RKNN_TENSOR_NCHW ? 3 : 2;
    m_net_h = input_attrs[0].dims[h_idx];
    m_net_w = input_attrs[0].dims[w_idx];

    // YOLOv8 输出: [batch, 84, 8400] (NCHW: class=0, dims[1]=84, dims[2]=8400)
    int cls_dim = output_attrs[0].fmt == RKNN_TENSOR_NCHW ? 1 : 2;
    m_class_num = output_attrs[0].dims[cls_dim] - 4;

    std::cout << "RKNN model loaded: batch=" << batch_size
              << " size=" << m_net_w << "x" << m_net_h
              << " classes=" << m_class_num
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

int YoloV8_det::Detect(const std::vector<cv::Mat>& batch_imgs, std::vector<YoloV8BoxVec>& boxes) {
    int n = batch_imgs.size();
    int img_size = m_net_w * m_net_h;
    int plane_size = img_size * 3;  // 3 channels

    // NCHW: float data[plane_size * n]
    std::vector<float> input_data(plane_size * n);
    std::vector<float> scales(n);
    std::vector<int> txs(n), tys(n);

    for (int i = 0; i < n; i++) {
        cv::Mat letterboxed, rgb;
        letterbox(batch_imgs[i], letterboxed, scales[i], txs[i], tys[i]);
        cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);

        // NCHW: R plane, G plane, B plane
        float* dst = input_data.data() + i * plane_size;
        for (int c = 0; c < 3; c++) {
            float* plane = dst + c * img_size;
            for (int y = 0; y < m_net_h; y++) {
                for (int x = 0; x < m_net_w; x++) {
                    plane[y * m_net_w + x] = rgb.at<cv::Vec3b>(y, x)[c] / 255.0f;
                }
            }
        }
    }

    // Set inputs
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_FLOAT32;
    inputs[0].size = plane_size * n * sizeof(float);
    inputs[0].fmt = RKNN_TENSOR_NCHW;
    inputs[0].buf = input_data.data();

    int ret = rknn_inputs_set(ctx, 1, inputs);
    if (ret < 0) {
        std::cerr << "rknn_inputs_set failed: " << ret << std::endl;
        return ret;
    }

    // Run inference
    ret = rknn_run(ctx, nullptr);
    if (ret < 0) {
        std::cerr << "rknn_run failed: " << ret << std::endl;
        return ret;
    }

    // Get outputs
    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        outputs[i].want_float = 1;
        outputs[i].is_prealloc = 0;
    }
    ret = rknn_get_outputs(ctx, outputs, NULL);
    if (ret < 0) {
        std::cerr << "rknn_get_outputs failed: " << ret << std::endl;
        return ret;
    }

    // Post-process: output shape [batch, 84, 8400] (NCHW)
    float* output_data = (float*)(outputs[0].buf);
    int box_num = output_attrs[0].dims[2];  // 8400
    int nout = output_attrs[0].dims[1];     // 84 = 4 + class_num

    boxes.clear();
    boxes.resize(n);

    for (int b = 0; b < n; b++) {
        YoloV8BoxVec& batch_boxes = boxes[b];
        float* batch_data = output_data + b * box_num * nout;

        for (int i = 0; i < box_num; i++) {
            float* ptr = batch_data + i * nout;  // NCHW: [nout, box_num] -> ptr = row i of nout

            float* cls_conf = ptr + 4;
            for (int j = 0; j < m_class_num; j++) {
                float score = sigmoid(cls_conf[j]);
                if (score > m_confThreshold) {
                    YoloV8Box box;
                    box.score = score;
                    box.class_id = j + 1;

                    float cx = ptr[0];
                    float cy = ptr[1];
                    float w = ptr[2];
                    float h = ptr[3];

                    box.x1 = cx - w / 2;
                    box.y1 = cy - h / 2;
                    box.x2 = cx + w / 2;
                    box.y2 = cy + h / 2;
                    batch_boxes.push_back(box);
                }
            }
        }

        NMS(batch_boxes, m_nmsThreshold);

        if ((int)batch_boxes.size() > max_det) {
            batch_boxes.erase(batch_boxes.begin(), batch_boxes.begin() + (batch_boxes.size() - max_det));
        }

        // Map back to original image coordinates
        float inv_scale = 1.0f / scales[b];
        for (auto& box : batch_boxes) {
            box.x1 = (box.x1 - txs[b]) * inv_scale;
            box.y1 = (box.y1 - tys[b]) * inv_scale;
            box.x2 = (box.x2 - txs[b]) * inv_scale;
            box.y2 = (box.y2 - tys[b]) * inv_scale;
        }
        clip_boxes(batch_boxes, batch_imgs[b].cols, batch_imgs[b].rows);
    }

    rknn_release_outputs(ctx, io_num.n_output, outputs);

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