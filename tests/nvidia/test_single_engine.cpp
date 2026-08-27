// 单图推理测试（NVIDIA）：把一张图片喂给同一 engine + 阈值，检测结果画框保存到本地。
//
// 用途：判断线上"偶发整帧无框"到底是模型/阈值问题，还是批量流水线（取帧/同步）问题。
//   - 若本测试对同一张图能检出目标  => 线上偶发无框是流水线问题，不是模型问题。
//   - 若本测试也检不出目标            => 模型本身在此阈值下检不出该目标。
//
// 用法：
//   test_single_engine <engine_path> <num_labels> <conf_thresh> <nms_thresh> <img1> [img2 ...]
//   输出：与输入同目录的 <basename>.out.jpg（画框图），并在终端打印每张的检测框数。
//
// 编译（远程，需 TensorRT/CUDA/OpenCV）：
//   g++ -std=c++17 -O2 -o test_single_engine \
//       tests/nvidia/test_single_engine.cpp \
//       src/nvidia/detector_nvidia.cpp \
//       -Iinclude -Iinclude/nvidia \
//       -I/usr/local/cuda/include -I/usr/include/opencv4 \
//       -L/usr/local/cuda/targets/aarch64-linux/lib \
//       -lnvinfer -lnvinfer_plugin -lcudart \
//       -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_dnn -lpthread

#include "detector_nvidia.hpp"
#include "log_macros.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

static cv::Scalar labelColor(int class_id) {
    cv::Scalar c;
    switch (class_id) {
        case 0:  c = cv::Scalar(255, 0, 0);   break;  // BGR
        case 1:  c = cv::Scalar(0, 255, 0);   break;
        case 2:  c = cv::Scalar(0, 165, 255); break;
        case 3:  c = cv::Scalar(255, 255, 0); break;
        case 4:  c = cv::Scalar(255, 0, 255); break;
        case 5:  c = cv::Scalar(0, 255, 255); break;
        case 6:  c = cv::Scalar(128, 0, 255); break;
        case 7:  c = cv::Scalar(0, 128, 255); break;
        case 8:  c = cv::Scalar(255, 128, 0); break;
        default: c = cv::Scalar(0, 255, 128); break;
    }
    return c;
}

static std::string outPathFor(const std::string& in) {
    size_t dot = in.find_last_of('.');
    std::string base = (dot == std::string::npos) ? in : in.substr(0, dot);
    return base + ".out.jpg";
}

int main(int argc, char** argv) {
    if (argc < 6) {
        fprintf(stderr,
                "usage: %s <engine_path> <num_labels> <conf> <nms> <img1> [img2 ...]\n",
                argv[0]);
        return 1;
    }

    std::string engine_path = argv[1];
    int  num_labels = atoi(argv[2]);
    float conf = (float)atof(argv[3]);
    float nms  = (float)atof(argv[4]);

    YoloV8_det det;
    det.setDetectThresholds(conf, nms);
    if (det.Init(engine_path, num_labels) != 0) {
        LOG_ERROR("engine init failed: %s", engine_path.c_str());
        return 1;
    }
    LOG_INFO("engine loaded: %s  labels=%d conf=%.3f nms=%.3f",
             engine_path.c_str(), num_labels, conf, nms);

    int nimg = argc - 5;
    for (int k = 0; k < nimg; k++) {
        std::string img_path = argv[5 + k];
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            LOG_ERROR("cannot read image: %s", img_path.c_str());
            continue;
        }

        YoloV8BoxVec boxes;
        int ret = det.Detect(img, boxes, 0);
        if (ret != 0) {
            LOG_ERROR("Detect failed for %s", img_path.c_str());
            continue;
        }

        LOG_INFO("[%s] boxes=%zu", img_path.c_str(), boxes.size());
        cv::Mat out = img.clone();
        for (auto& b : boxes) {
            int cls = b.class_id + 1;
            cv::rectangle(out,
                          cv::Rect(cv::Point((int)b.x1, (int)b.y1),
                                   cv::Point((int)b.x2, (int)b.y2)),
                          labelColor(b.class_id), 2);
            char txt[64];
            snprintf(txt, sizeof(txt), "cls%d %.2f", cls, b.score);
            cv::putText(out, txt, cv::Point((int)b.x1, (int)b.y1 - 4),
                        cv::FONT_HERSHEY_PLAIN, 1.2, labelColor(b.class_id), 2);
        }

        std::string out_path = outPathFor(img_path);
        if (!cv::imwrite(out_path, out)) {
            LOG_ERROR("failed to write %s", out_path.c_str());
        } else {
            LOG_INFO("saved: %s", out_path.c_str());
        }
    }

    return 0;
}
