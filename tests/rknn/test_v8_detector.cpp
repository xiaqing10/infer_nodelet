#include <iostream>
#include <string>
#include <chrono>
#include "detector_rknn.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <model.rknn> <image.jpg> [num_classes]" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::string img_path = argv[2];
    int num_classes = 10;
    if (argc >= 4) num_classes = std::stoi(argv[3]);

    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << img_path << std::endl;
        return 1;
    }
    std::cout << "Image: " << img.cols << "x" << img.rows << std::endl;

    YoloV8_det det;
    det.m_model_type = "v8";
    std::cout << "model_type=" << det.m_model_type << std::endl;

    int ret = det.Init(model_path, RKNN_NPU_CORE_AUTO);
    if (ret != 0) {
        std::cerr << "Init failed: " << ret << std::endl;
        return 1;
    }

    std::cout << "Init done: type=" << det.m_model_type
              << " classes=" << det.m_class_num
              << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    YoloV8BoxVec boxes;
    ret = det.Detect(img, boxes);
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    if (ret != 0) {
        std::cerr << "Detect failed: " << ret << std::endl;
        return 1;
    }

    std::cout << "Inference time: " << elapsed << " ms" << std::endl;
    std::cout << "Detections: " << boxes.size() << std::endl;

    for (size_t i = 0; i < boxes.size(); i++) {
        auto& b = boxes[i];
        std::cout << "  [" << i << "] class=" << b.class_id
                  << " conf=" << b.score
                  << " box=[" << (int)b.x1 << "," << (int)b.y1
                  << "," << (int)(b.x2 - b.x1) << "x" << (int)(b.y2 - b.y1) << "]"
                  << std::endl;
    }

    return 0;
}