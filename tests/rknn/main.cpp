#include <iostream>
#include <string>
#include <cstring>
#include "rknn_test.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " [options] <model.rknn> <image.jpg> [output.jpg] [num_classes]"
                  << std::endl;
        std::cout << "  --yolov5,-y5 : use YOLOv5 postprocessing (default: auto-detect)" << std::endl;
        std::cout << "  --letterbox  : use letterbox preprocessing (default: direct resize)" << std::endl;
        std::cout << "  --sigmoid    : apply sigmoid after dequant (default: off)" << std::endl;
        std::cout << "  --core-mask MASK : set NPU core mask (auto/0/1/2/0_1/0_1_2, default: auto)" << std::endl;
        std::cout << "  model.rknn   : path to RKNN model file" << std::endl;
        std::cout << "  image.jpg    : path to input image" << std::endl;
        std::cout << "  output.jpg   : path to save output image (default: output.jpg)" << std::endl;
        std::cout << "  num_classes  : number of classes (default: auto-detect from model)" << std::endl;
        return 1;
    }

    bool use_yolov5 = false;
    bool use_letterbox = false;
    bool use_sigmoid = false;
    std::string core_mask = "auto";
    int arg_idx = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--yolov5") == 0 || strcmp(argv[i], "-y5") == 0) {
            use_yolov5 = true;
        } else if (strcmp(argv[i], "--letterbox") == 0) {
            use_letterbox = true;
        } else if (strcmp(argv[i], "--sigmoid") == 0) {
            use_sigmoid = true;
        } else if (strcmp(argv[i], "--core-mask") == 0) {
            if (i + 1 < argc) {
                core_mask = argv[++i];
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
    std::string output_path = "output.jpg";
    int num_classes = 10;

    int remaining = argc - (arg_idx + 2);
    if (remaining >= 1) {
        std::string arg3 = argv[arg_idx + 2];
        bool is_num = !arg3.empty() && arg3.find_first_not_of("0123456789") == std::string::npos;
        if (is_num) {
            num_classes = std::stoi(arg3);
        } else {
            output_path = arg3;
        }
    }
    if (remaining >= 2) {
        num_classes = std::stoi(argv[arg_idx + 3]);
    }

    std::cout << "=== RKNN YOLO Test ===" << std::endl;
    std::cout << "Model: " << model_path << std::endl;
    std::cout << "Image: " << image_path << std::endl;
    std::cout << "Output: " << output_path << std::endl;
    std::cout << "Mode: " << (use_yolov5 ? "YOLOv5" : "auto-detect") << std::endl;
    std::cout << "Preprocess: " << (use_letterbox ? "letterbox" : "direct resize") << std::endl;
    std::cout << "Sigmoid: " << (use_sigmoid ? "on" : "off") << std::endl;
    std::cout << "Core mask: " << core_mask << std::endl;
    std::cout << "Num classes: " << num_classes << std::endl;

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }
    std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;

    RknnTest test(model_path, num_classes);
    test.setCoreMask(core_mask);
    test.setYoloV5(use_yolov5);
    test.setUseLetterbox(use_letterbox);
    test.setUseSigmoid(use_sigmoid);

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