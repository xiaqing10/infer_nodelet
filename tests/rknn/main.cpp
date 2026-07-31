#include <iostream>
#include <string>
#include "rknn_test.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <model.rknn> <image.jpg> [output.jpg] [num_classes]"
                  << std::endl;
        std::cout << "  model.rknn   : path to RKNN model file" << std::endl;
        std::cout << "  image.jpg    : path to input image" << std::endl;
        std::cout << "  output.jpg   : path to save output image (default: output.jpg)" << std::endl;
        std::cout << "  num_classes  : number of classes (default: auto-detect from model)" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];
    std::string output_path = "output.jpg";
    int num_classes = 10;
    if (argc >= 4) {
        std::string arg3 = argv[3];
        // Check if arg3 is a number (num_classes) or a filename
        bool is_num = !arg3.empty() && arg3.find_first_not_of("0123456789") == std::string::npos;
        if (is_num) {
            num_classes = std::stoi(arg3);
        } else {
            output_path = arg3;
        }
    }
    if (argc >= 5) {
        num_classes = std::stoi(argv[4]);
    }

    std::cout << "=== RKNN YOLO Test ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << std::endl;
    std::cout << "Output: " << output_path << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }
    std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;

    RknnTest test(model_path, num_classes);

    auto t0 = std::chrono::steady_clock::now();
    std::vector<RknnDetection> detections = test.runInference(img);
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    std::cout << "Inference time: " << elapsed << " ms" << std::endl;
    std::cout << "Detections: " << detections.size() << std::endl;

    for (size_t i = 0; i < detections.size(); i++) {
        auto& d = detections[i];
        std::cout << "  [" << i << "] class=" << d.class_id
                  << " conf=" << d.confidence
                  << " box=[" << d.box.x << "," << d.box.y
                  << "," << d.box.width << "x" << d.box.height << "]"
                  << std::endl;
    }

    cv::Mat result = test.drawResults(img, detections);
    cv::imwrite(output_path, result);
    std::cout << "Result saved to: " << output_path << std::endl;

    return 0;
}