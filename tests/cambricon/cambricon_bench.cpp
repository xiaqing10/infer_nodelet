#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "detector.hpp"

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " <model.cambricon> <image.jpg> [num_iters] [num_classes] [stride]" << std::endl;
    std::cout << "  model.cambricon : path to Cambricon model file" << std::endl;
    std::cout << "  image.jpg       : path to input image" << std::endl;
    std::cout << "  num_iters       : number of inference iterations (default: 100)" << std::endl;
    std::cout << "  num_classes     : number of classes (default: 10)" << std::endl;
    std::cout << "  stride          : feature stride (default: 3)" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];
    int num_iters = 100;
    int num_classes = 10;
    int stride = 3;

    if (argc >= 4) num_iters = std::stoi(argv[3]);
    if (argc >= 5) num_classes = std::stoi(argv[4]);
    if (argc >= 6) stride = std::stoi(argv[5]);

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }

    std::cout << "=== Cambricon YOLOv8 Benchmark ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << " (" << img.cols << "x" << img.rows << ")" << std::endl;
    std::cout << "Iterations: " << num_iters << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;
    std::cout << "Stride: " << stride << std::endl;
    std::cout << std::endl;

    Detector detector;
    detector.init(model_path, 0, num_classes, stride);
    std::cout << "Model loaded" << std::endl;
    std::cout << std::endl;

    // Warmup
    int warmup = std::min(20, num_iters / 2);
    for (int i = 0; i < warmup; i++) {
        cv::Mat tmp = img.clone();
        detector.inference(tmp);
    }

    // Timed: end-to-end (preprocess + inference + postprocess)
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < num_iters; i++) {
        cv::Mat tmp = img.clone();
        detector.inference(tmp);
    }
    auto t1 = std::chrono::steady_clock::now();

    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    double avg_ms = (double)total_ms / num_iters;
    double fps = 1000.0 / avg_ms;

    std::cout << "--- Results (end-to-end: preprocess + inference + postprocess) ---" << std::endl;
    std::cout << "Total time: " << total_ms << " ms for " << num_iters << " iters" << std::endl;
    std::cout << "Average:    " << avg_ms << " ms / iter" << std::endl;
    std::cout << "FPS:        " << fps << std::endl;
    std::cout << std::endl;

    // Run one final inference to verify correctness
    std::vector<DetectorRetData> detections = detector.inference(img);
    std::cout << "Final run detections: " << detections.size() << std::endl;
    for (size_t i = 0; i < std::min((size_t)5, detections.size()); i++) {
        auto& d = detections[i];
        std::cout << "  [" << i << "] class=" << d.label
                  << " conf=" << d.confidence
                  << " box=[" << d.xmin << "," << d.ymin
                  << "," << d.xmax << "x" << d.ymax << "]"
                  << std::endl;
    }

    return 0;
}