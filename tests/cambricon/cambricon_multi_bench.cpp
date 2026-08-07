#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "cnrt.h"
#include "Net.hpp"

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <model.cambricon> <image.jpg> [num_iters] [num_instances] [num_classes] [stride]" << std::endl;
    std::cout << "  model.cambricon : path to Cambricon model file" << std::endl;
    std::cout << "  image.jpg       : path to input image" << std::endl;
    std::cout << "  num_iters       : number of inference iterations per instance (default: 50)" << std::endl;
    std::cout << "  num_instances   : number of parallel instances to test (default: 2)" << std::endl;
    std::cout << "  num_classes     : number of classes (default: 10)" << std::endl;
    std::cout << "  stride          : feature stride (default: 3)" << std::endl;
}

static float letterbox(const cv::Mat& image, cv::Mat& out_image, const cv::Size& new_shape, const cv::Scalar& color) {
    cv::Size shape = image.size();
    float r = std::min((float)new_shape.height / (float)shape.height, (float)new_shape.width / (float)shape.width);
    int new_unpad[2]{(int)std::round((float)shape.width * r), (int)std::round((float)shape.height * r)};
    cv::Mat tmp;
    if (shape.width != new_unpad[0] || shape.height != new_unpad[1]) {
        cv::resize(image, tmp, cv::Size(new_unpad[0], new_unpad[1]));
    } else {
        tmp = image.clone();
    }
    float dw = new_shape.width - new_unpad[0];
    float dh = new_shape.height - new_unpad[1];
    dw /= 2.0f;
    dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(tmp, out_image, top, bottom, left, right, cv::BORDER_CONSTANT, color);
    return 1.0f / r;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];
    int num_iters = 50;
    int num_instances = 2;
    int num_classes = 10;
    int stride = 3;

    if (argc >= 4) num_iters = std::stoi(argv[3]);
    if (argc >= 5) num_instances = std::stoi(argv[4]);
    if (argc >= 6) num_classes = std::stoi(argv[5]);
    if (argc >= 7) stride = std::stoi(argv[6]);

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }

    std::cout << "=== Cambricon Multi-instance Parallel Benchmark ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << " (" << img.cols << "x" << img.rows << ")" << std::endl;
    std::cout << "Iters per instance: " << num_iters << std::endl;
    std::cout << "Max instances: " << num_instances << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;
    std::cout << "Stride: " << stride << std::endl;
    std::cout << std::endl;

    // Pre-load model data
    std::ifstream file(model_path, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Failed to open model: " << model_path << std::endl;
        return 1;
    }
    size_t model_size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> model_data(model_size);
    file.read(model_data.data(), model_size);
    file.close();

    // Preprocess image once
    cv::Mat resized;
    float scale = letterbox(img, resized, cv::Size(640, 640), cv::Scalar(114, 114, 114));
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // Test 1, 2, ..., num_instances
    for (int n = 1; n <= num_instances; n++) {
        std::vector<cn::Net> nets(n);
        for (int i = 0; i < n; i++) {
            int dev_id = i % 2;
            nets[i].init(model_path, dev_id, num_classes);
        }

        if (n > 1) {
            // Warmup
            std::vector<std::thread> warmup_threads;
            for (int i = 0; i < n; i++) {
                warmup_threads.emplace_back([&nets, i, &rgb]() {
                    int h = nets[i].input_dims(1);
                    int w = nets[i].input_dims(2);
                    float* input = (float*)malloc(h * w * 3 * sizeof(float));
                    cv::Mat img_cvt(h, w, CV_32FC3, input);
                    rgb.convertTo(img_cvt, CV_32F);
                    img_cvt /= 255.0f;
                    nets[i].input_from(input);
                    nets[i].infer();
                    free(input);
                });
            }
            for (auto& t : warmup_threads) t.join();
        }

        // Timed parallel inference
        auto t0 = std::chrono::steady_clock::now();
        {
            std::vector<std::thread> threads;
            for (int i = 0; i < n; i++) {
                threads.emplace_back([&nets, i, &rgb, num_iters]() {
                    int h = nets[i].input_dims(1);
                    int w = nets[i].input_dims(2);
                    int box_size = nets[i].output_dims(1, 0) * nets[i].output_dims(3, 0);
                    int cls_size = nets[i].output_dims(1, 1) * nets[i].output_dims(3, 1);
                    float* input = (float*)malloc(h * w * 3 * sizeof(float));
                    float* box_output = (float*)malloc(box_size * sizeof(float));
                    float* cls_output = (float*)malloc(cls_size * sizeof(float));

                    for (int iter = 0; iter < num_iters; iter++) {
                        cv::Mat img_cvt(h, w, CV_32FC3, input);
                        rgb.convertTo(img_cvt, CV_32F);
                        img_cvt /= 255.0f;
                        nets[i].input_from(input);
                        nets[i].infer();
                        nets[i].output_to(box_output, 0);
                        nets[i].output_to(cls_output, 1);
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
        for (int i = 0; i < n; i++) {
            std::cout << "  Instance " << i << " -> devId " << (i % 2) << std::endl;
        }
        std::cout << "Total: " << total_us / 1000 << " ms for " << total_iters << " iters" << std::endl;
        std::cout << "Avg:   " << avg_us << " us / iter" << std::endl;
        std::cout << "FPS:   " << fps << std::endl;
        std::cout << std::endl;
    }

    return 0;
}