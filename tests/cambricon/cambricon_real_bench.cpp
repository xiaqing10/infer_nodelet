#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "cnrt.h"
#include "Net.hpp"

static float letterbox(const cv::Mat& image, cv::Mat& out_image, const cv::Size& new_shape, const cv::Scalar& color) {
    cv::Size shape = image.size();
    float r = std::min((float)new_shape.height / (float)shape.height, (float)new_shape.width / (float)shape.width);
    int new_unpad[2]{(int)std::round((float)shape.width * r), (int)std::round((float)shape.height * r)};
    cv::Mat tmp;
    if (shape.width != new_unpad[0] || shape.height != new_unpad[1])
        cv::resize(image, tmp, cv::Size(new_unpad[0], new_unpad[1]));
    else
        tmp = image.clone();
    float dw = new_shape.width - new_unpad[0];
    float dh = new_shape.height - new_unpad[1];
    dw /= 2.0f; dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(tmp, out_image, top, bottom, left, right, cv::BORDER_CONSTANT, color);
    return 1.0f / r;
}

static void yolov8_nms(std::vector<std::vector<float>>& boxes, float nms_thred) {
    int length = boxes.size();
    int index = length - 1;
    sort(boxes.begin(), boxes.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    std::vector<float> areas(length);
    for (int i = 0; i < length; i++) {
        float width = boxes[i][4] - boxes[i][2];
        float height = boxes[i][5] - boxes[i][3];
        areas[i] = width * height;
    }
    while (index > 0) {
        int i = 0;
        while (i < index) {
            float left = std::max(boxes[index][2], boxes[i][2]);
            float top = std::max(boxes[index][3], boxes[i][3]);
            float right = std::min(boxes[index][4], boxes[i][4]);
            float bottom = std::min(boxes[index][5], boxes[i][5]);
            float overlap = std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
            float union_area = areas[index] + areas[i] - overlap;
            if (overlap / union_area > nms_thred) {
                areas.erase(areas.begin() + i);
                boxes.erase(boxes.begin() + i);
                index--;
            } else { i++; }
        }
        index--;
    }
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <model.cambricon> <image1.jpg> <image2.jpg> [num_iters] [num_instances] [num_classes] [stride]" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::vector<std::string> image_paths = {argv[2], argv[3]};
    int num_iters = 50;
    int num_instances = 2;
    int num_classes = 10;
    int stride = 3;

    if (argc >= 5) num_iters = std::stoi(argv[4]);
    if (argc >= 6) num_instances = std::stoi(argv[5]);
    if (argc >= 7) num_classes = std::stoi(argv[6]);
    if (argc >= 8) stride = std::stoi(argv[7]);

    std::vector<cv::Mat> ref_images;
    for (auto& p : image_paths) {
        cv::Mat img = cv::imread(p);
        if (img.empty()) { std::cerr << "Failed to load: " << p << std::endl; return 1; }
        ref_images.push_back(img);
    }

    std::cout << "=== Cambricon Realistic Benchmark ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Images: " << image_paths[0] << ", " << image_paths[1] << std::endl;
    std::cout << "Iters per instance: " << num_iters << std::endl;
    std::cout << "Max instances: " << num_instances << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;
    std::cout << "Stride: " << stride << std::endl;
    std::cout << std::endl;

    for (int n = 1; n <= num_instances; n++) {
        std::vector<cn::Net> nets(n);
        for (int i = 0; i < n; i++) {
            int dev_id = i % 2;
            nets[i].init(model_path, dev_id, num_classes);
        }

        std::vector<std::thread> threads;
        auto t0 = std::chrono::steady_clock::now();
        {
            for (int i = 0; i < n; i++) {
                threads.emplace_back([&nets, i, &ref_images, num_iters, num_classes, stride]() {
                    cv::Mat img = ref_images[i % ref_images.size()].clone();
                    int h = nets[i].input_dims(1);
                    int w = nets[i].input_dims(2);
                    int box_size = nets[i].output_dims(1, 0) * nets[i].output_dims(3, 0);
                    int cls_size = nets[i].output_dims(1, 1) * nets[i].output_dims(3, 1);

                    float* input = (float*)malloc(h * w * 3 * sizeof(float));
                    float* box_output = (float*)malloc(box_size * sizeof(float));
                    float* cls_output = (float*)malloc(cls_size * sizeof(float));

                    for (int iter = 0; iter < num_iters; iter++) {
                        cv::Mat resized;
                        float scale = letterbox(img, resized, cv::Size(w, h), cv::Scalar(114, 114, 114));
                        cv::Mat rgb;
                        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
                        cv::Mat img_cvt(h, w, CV_32FC3, input);
                        rgb.convertTo(img_cvt, CV_32F);
                        img_cvt /= 255.0f;

                        nets[i].input_from(input);
                        nets[i].infer();
                        nets[i].output_to(box_output, 0);
                        nets[i].output_to(cls_output, 1);

                        // postprocess (simplified)
                        std::vector<int> stride_list;
                        if (stride == 4) stride_list = {4, 8, 16, 32};
                        else stride_list = {8, 16, 32};

                        std::vector<std::vector<float>> tmp_result;
                        float* box_ptr = box_output;
                        float* cls_ptr = cls_output;
                        int grid_h = 0, grid_w = 0;
                        for (int n_s = 0; n_s < 3; n_s++) {
                            grid_h = grid_w = 640 / stride_list[n_s];
                            for (int k = 0; k < grid_h; k++) {
                                for (int j = 0; j < grid_w; j++) {
                                    const float* td = box_ptr + (k * grid_w + j) * 4;
                                    const float* tc = cls_ptr + (k * grid_w + j) * num_classes;
                                    int topClass = 0;
                                    for (int c = 1; c < num_classes; c++) {
                                        if (tc[c] > tc[topClass]) topClass = c;
                                    }
                                    if (tc[topClass] < 0.25f) continue;
                                    float cx = (j + 0.5f - td[0]) * stride_list[n_s];
                                    float cy = (k + 0.5f - td[1]) * stride_list[n_s];
                                    float bw = (td[2] + td[0]) * stride_list[n_s];
                                    float bh = (td[3] + td[1]) * stride_list[n_s];
                                    float x1 = std::max(0.0f, cx - bw * 0.5f);
                                    float y1 = std::max(0.0f, cy - bh * 0.5f);
                                    float x2 = std::min(cx + bw * 0.5f, 640.0f);
                                    float y2 = std::min(cy + bh * 0.5f, 640.0f);
                                    if (bw < 5 || bh < 5) continue;
                                    std::vector<float> sr = {(float)topClass, tc[topClass], x1, y1, x2, y2};
                                    tmp_result.push_back(sr);
                                }
                            }
                            box_ptr += grid_h * grid_w * 4;
                            cls_ptr += grid_h * grid_w * num_classes;
                        }
                        if (!tmp_result.empty()) {
                            yolov8_nms(tmp_result, 0.5f);
                            int x_offset = (w * scale - img.cols) / 2;
                            int y_offset = (h * scale - img.rows) / 2;
                            for (auto& sr : tmp_result) {
                                sr[2] = sr[2] * scale - x_offset;
                                sr[3] = sr[3] * scale - y_offset;
                                sr[4] = sr[4] * scale - x_offset;
                                sr[5] = sr[5] * scale - y_offset;
                            }
                        }
                    }
                    free(input);
                    free(box_output);
                    free(cls_output);
                });
            }
            for (auto& t : threads) t.join();
        }
        auto t1 = std::chrono::steady_clock::now();

        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        int total_iters = num_iters * n;
        double avg_us = (double)total_us / total_iters;
        double fps = 1000000.0 / avg_us;

        std::cout << "--- " << n << " instance(s) parallel ---" << std::endl;
        for (int i = 0; i < n; i++) std::cout << "  Instance " << i << " -> devId " << (i % 2) << std::endl;
        std::cout << "Total: " << total_us / 1000 << " ms for " << total_iters << " iters" << std::endl;
        std::cout << "Avg:   " << avg_us << " us / iter" << std::endl;
        std::cout << "FPS:   " << fps << std::endl;
        std::cout << std::endl;
    }

    return 0;
}