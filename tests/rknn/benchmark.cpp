#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include "rknn_test.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " [options] <model.rknn> <image.jpg> [num_iters] [num_classes]" << std::endl;
        std::cout << "  --core-mask MASK : set NPU core mask (auto/0/1/2/0_1/0_1_2, default: auto)" << std::endl;
        std::cout << "  model.rknn   : path to RKNN model file" << std::endl;
        std::cout << "  image.jpg    : path to input image" << std::endl;
        std::cout << "  num_iters    : number of inference iterations (default: 100)" << std::endl;
        std::cout << "  num_classes  : number of classes (default: auto-detect)" << std::endl;
        return 1;
    }

    std::string core_mask = "auto";
    int arg_idx = 1;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--core-mask") == 0) {
            if (i + 1 < argc) {
                core_mask = argv[++i];
                arg_idx = i + 1;
            } else {
                std::cerr << "--core-mask requires an argument" << std::endl;
                return 1;
            }
        } else {
            arg_idx = i;
            break;
        }
    }

    if (argc < arg_idx + 2) {
        std::cerr << "Missing model/image arguments" << std::endl;
        return 1;
    }

    std::string model_path = argv[arg_idx];
    std::string image_path = argv[arg_idx + 1];
    int num_iters = 100;
    int num_classes = 10;

    if (argc >= arg_idx + 3) num_iters = std::stoi(argv[arg_idx + 2]);
    if (argc >= arg_idx + 4) num_classes = std::stoi(argv[arg_idx + 3]);

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }

    std::cout << "=== RKNN YOLOv5 Benchmark ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << " (" << img.cols << "x" << img.rows << ")" << std::endl;
    std::cout << "Iterations: " << num_iters << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;
    std::cout << "Core mask: " << core_mask << std::endl;
    std::cout << std::endl;

    RknnTest test(model_path, num_classes);
    test.setCoreMask(core_mask);
    // Auto-detect: 3 outputs -> YOLOv5, 1 output -> YOLOv8
    test.setYoloV5(test.getNumOutputs() >= 3);

    // Warmup: run a few iterations to stabilize NPU clock
    int warmup = std::min(20, num_iters / 2);
    for (int i = 0; i < warmup; i++) {
        test.runInference(img);
    }

    // Timed: end-to-end (preprocess + inference + postprocess)
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < num_iters; i++) {
        test.runInference(img);
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

    // Timed: inference only (rknn_run)
    auto t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < num_iters; i++) {
        test.runInferenceOnly(img);
    }
    auto t3 = std::chrono::steady_clock::now();

    auto infer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
    double avg_infer_ms = (double)infer_ms / num_iters;
    double infer_fps = 1000.0 / avg_infer_ms;

    std::cout << "--- Results (inference only: rknn_run) ---" << std::endl;
    std::cout << "Total time: " << infer_ms << " ms for " << num_iters << " iters" << std::endl;
    std::cout << "Average:    " << avg_infer_ms << " ms / iter" << std::endl;
    std::cout << "FPS:        " << infer_fps << std::endl;
    std::cout << std::endl;

    // Run one final inference to verify correctness with output
    auto detections = test.runInference(img);
    std::cout << "Final run detections: " << detections.size() << std::endl;
    for (size_t i = 0; i < std::min((size_t)5, detections.size()); i++) {
        auto& d = detections[i];
        std::cout << "  [" << i << "] class=" << d.class_id
                  << " conf=" << d.confidence
                  << " box=[" << d.box.x << "," << d.box.y
                  << "," << d.box.width << "x" << d.box.height << "]"
                  << std::endl;
    }

    cv::Mat result = test.drawResults(img, detections);
    std::string output_path = "benchmark_output.jpg";
    cv::imwrite(output_path, result);
    std::cout << "Result saved to: " << output_path << std::endl;

    return 0;
}