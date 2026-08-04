#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>
#include <cstring>
#include <dirent.h>

#include "opencv2/opencv.hpp"
#include "acl/acl.h"
#include "AclLiteModel.h"
#include "AclLiteUtils.h"

using namespace std;

struct YoloV8Box {
    float x1, y1, x2, y2;
    float score;
    int class_id;
};

using YoloV8BoxVec = vector<YoloV8Box>;

static const vector<vector<int>> colors = {
    {255, 0, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0},
    {170, 255, 0}, {85, 255, 0}, {0, 255, 0}, {0, 255, 85},
    {0, 255, 170}, {0, 255, 255}, {0, 170, 255}, {0, 85, 255},
    {0, 0, 255}, {85, 0, 255}, {170, 0, 255}, {255, 0, 255},
    {255, 0, 170}, {255, 0, 85}, {255, 0, 0}, {255, 0, 255},
    {255, 85, 255}, {255, 170, 255}, {255, 255, 255}, {170, 255, 255},
    {85, 255, 255}};

static const vector<string> class_names = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"};

static float get_aspect_scaled_ratio(int src_w, int src_h, int dst_w, int dst_h, bool* pIsAligWidth) {
    float ratio;
    float r_w = (float)dst_w / src_w;
    float r_h = (float)dst_h / src_h;
    if (r_h > r_w) {
        *pIsAligWidth = true;
        ratio = r_w;
    } else {
        *pIsAligWidth = false;
        ratio = r_h;
    }
    return ratio;
}

static void nms(YoloV8BoxVec& dets, float nmsConfidence) {
    if (dets.empty()) return;

    sort(dets.begin(), dets.end(), [](const YoloV8Box& a, const YoloV8Box& b) {
        return a.score > b.score;
    });

    vector<float> areas(dets.size());
    for (size_t i = 0; i < dets.size(); i++) {
        float width = dets[i].x2 - dets[i].x1;
        float height = dets[i].y2 - dets[i].y1;
        areas[i] = width * height;
    }

    for (size_t i = 0; i < dets.size(); i++) {
        if (dets[i].score == 0) continue;
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (dets[j].score == 0) continue;
            if (dets[i].class_id != dets[j].class_id) continue;

            float xx1 = max(dets[i].x1, dets[j].x1);
            float yy1 = max(dets[i].y1, dets[j].y1);
            float xx2 = min(dets[i].x2, dets[j].x2);
            float yy2 = min(dets[i].y2, dets[j].y2);

            float w = max(0.0f, xx2 - xx1);
            float h = max(0.0f, yy2 - yy1);
            float inter = w * h;
            float iou = inter / (areas[i] + areas[j] - inter);

            if (iou > nmsConfidence) {
                dets[j].score = 0;
            }
        }
    }

    dets.erase(remove_if(dets.begin(), dets.end(),
                         [](const YoloV8Box& box) { return box.score == 0; }),
               dets.end());
}

static void clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h) {
    for (auto& box : yolobox_vec) {
        box.x1 = max(0.0f, min(box.x1, (float)src_w));
        box.y1 = max(0.0f, min(box.y1, (float)src_h));
        box.x2 = max(0.0f, min(box.x2, (float)src_w));
        box.y2 = max(0.0f, min(box.y2, (float)src_h));
    }
}

static void draw_result(cv::Mat& img, const YoloV8BoxVec& result) {
    for (size_t i = 0; i < result.size(); i++) {
        int left = (int)result[i].x1;
        int top = (int)result[i].y1;
        int right = (int)result[i].x2;
        int bottom = (int)result[i].y2;
        int label = result[i].class_id;
        float score = result[i].score;

        const auto& color = colors[label % colors.size()];
        cv::rectangle(img, cv::Point(left, top), cv::Point(right, bottom),
                      cv::Scalar(color[0], color[1], color[2]), 3);

        string text = class_names[label] + " " + to_string(score).substr(0, 4);
        int baseline;
        cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_COMPLEX, 0.8, 1, &baseline);
        cv::rectangle(img, cv::Point(left, top - textSize.height - 5),
                      cv::Point(left + textSize.width + 5, top),
                      cv::Scalar(color[0], color[1], color[2]), -1);
        cv::putText(img, text, cv::Point(left, top - 5),
                    cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255), 1);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <image_path> <model_path>" << endl;
        return -1;
    }

    string image_path = argv[1];
    string model_path = argv[2];

    const int32_t modelWidth = 640;
    const int32_t modelHeight = 640;
    const float confThreshold = 0.35;
    const float nmsThreshold = 0.45;
    const int classNum = 10;
    const int maxDet = 300;
    const int num_features = 4 + classNum;  // 4 bbox + classNum 类别分数

    // 1. Read image
    cv::Mat srcImage = cv::imread(image_path);
    if (srcImage.empty()) {
        cerr << "[ERROR] Failed to read image: " << image_path << endl;
        return -1;
    }
    int srcWidth = srcImage.cols;
    int srcHeight = srcImage.rows;
    cout << "[INFO] Image loaded: " << srcWidth << "x" << srcHeight << endl;

    // 2. Init ACL
    aclError ret = aclInit(nullptr);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclInit failed: " << ret << endl;
        return -1;
    }
    cout << "[INFO] aclInit ok" << endl;

    ret = aclrtSetDevice(0);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtSetDevice failed: " << ret << endl;
        aclFinalize();
        return -1;
    }
    cout << "[INFO] Set device 0 ok" << endl;

    aclrtContext context = nullptr;
    ret = aclrtCreateContext(&context, 0);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtCreateContext failed: " << ret << endl;
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }
    cout << "[INFO] Create context ok" << endl;

    ret = aclrtSetCurrentContext(context);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtSetCurrentContext failed: " << ret << endl;
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }

    aclrtStream stream = nullptr;
    ret = aclrtCreateStream(&stream);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtCreateStream failed: " << ret << endl;
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }

    aclrtRunMode runMode;
    aclrtGetRunMode(&runMode);

    // 3. Load model
    AclLiteModel model;
    ret = model.Init(model_path);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] Model init failed: " << model_path << endl;
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }
    cout << "[INFO] Model loaded: " << model_path << endl;

    // 4. Allocate device input buffer
    uint32_t imageDataSize = modelWidth * modelHeight * 3;
    void* imageDataBuf = nullptr;
    ret = aclrtMalloc(&imageDataBuf, imageDataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtMalloc failed: " << ret << endl;
        model.DestroyResource();
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }

    // 5. Create model input (reuse for all inference)
    ret = model.CreateInput(imageDataBuf, imageDataSize);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] CreateInput failed: " << ret << endl;
        aclrtFree(imageDataBuf);
        model.DestroyResource();
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }

    // 6. Pre-process (same as infer_nodelet)
    bool isAlignWidth = false;
    float ratio = get_aspect_scaled_ratio(srcWidth, srcHeight, modelWidth, modelHeight, &isAlignWidth);

    int tx1 = 0, ty1 = 0;
    if (isAlignWidth) {
        int resized_h = (int)(srcHeight * ratio);
        ty1 = (modelHeight - resized_h) / 2;
    } else {
        int resized_w = (int)(srcWidth * ratio);
        tx1 = (modelWidth - resized_w) / 2;
    }

    cout << "[INFO] Letterbox: ratio=" << ratio << " tx1=" << tx1 << " ty1=" << ty1 << endl;

    cv::Mat rgbImage;
    cv::cvtColor(srcImage, rgbImage, cv::COLOR_BGR2RGB);

    // letterbox: resize keeping aspect ratio, pad with 114
    int resized_w, resized_h;
    if (isAlignWidth) {
        resized_w = modelWidth;
        resized_h = (int)(srcHeight * ratio);
    } else {
        resized_w = (int)(srcWidth * ratio);
        resized_h = modelHeight;
    }

    cv::Mat resizedImage;
    cv::resize(rgbImage, resizedImage, cv::Size(resized_w, resized_h));

    cv::Mat canvas(modelHeight, modelWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat roi = canvas(cv::Rect(tx1, ty1, resized_w, resized_h));
    resizedImage.copyTo(roi);

    aclrtMemcpyKind policy = (runMode == ACL_HOST) ?
                             ACL_MEMCPY_HOST_TO_DEVICE : ACL_MEMCPY_DEVICE_TO_DEVICE;
    ret = aclrtMemcpy(imageDataBuf, imageDataSize,
                      canvas.ptr<uint8_t>(), imageDataSize, policy);
    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] aclrtMemcpy to device failed: " << ret << endl;
        model.DestroyInput();
        aclrtFree(imageDataBuf);
        model.DestroyResource();
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }
    cout << "[INFO] Image data copied to device" << endl;

    // 7. Inference
    vector<InferenceOutput> inferOutputs;
    auto infer_start = chrono::system_clock::now();
    ret = model.Execute(inferOutputs);
    auto infer_end = chrono::system_clock::now();
    auto infer_ms = chrono::duration_cast<chrono::milliseconds>(infer_end - infer_start).count();

    if (ret != ACL_SUCCESS) {
        cerr << "[ERROR] Model execute failed: " << ret << endl;
        model.DestroyInput();
        aclrtFree(imageDataBuf);
        model.DestroyResource();
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }
    cout << "[INFO] Inference done: " << infer_ms << " ms" << endl;

    // 8. Post-process (same as infer_nodelet)
    if (inferOutputs.empty()) {
        cerr << "[ERROR] No inference outputs" << endl;
        model.DestroyInput();
        aclrtFree(imageDataBuf);
        model.DestroyResource();
        aclrtDestroyStream(stream);
        aclrtDestroyContext(context);
        aclrtResetDevice(0);
        aclFinalize();
        return -1;
    }

    float* classBuff = static_cast<float*>(inferOutputs[0].data.get());
    size_t total_elements = inferOutputs[0].size / sizeof(float);
    size_t num_boxes = total_elements / num_features;

    cout << "[INFO] Output: total_elements=" << total_elements
         << " num_boxes=" << num_boxes << " num_features=" << num_features << endl;

    YoloV8BoxVec raw_boxes;

    for (size_t i = 0; i < num_boxes; ++i) {
        float cx = classBuff[0 * num_boxes + i];
        float cy = classBuff[1 * num_boxes + i];
        float w  = classBuff[2 * num_boxes + i];
        float h  = classBuff[3 * num_boxes + i];

        bool need_normalize = false;
        if (cx > 1.0f || cy > 1.0f || w > 1.0f || h > 1.0f) {
            need_normalize = true;
        }

        float normalized_cx, normalized_cy, normalized_w, normalized_h;

        if (need_normalize) {
            normalized_cx = cx / modelWidth;
            normalized_cy = cy / modelHeight;
            normalized_w = w / modelWidth;
            normalized_h = h / modelHeight;
        } else {
            normalized_cx = cx;
            normalized_cy = cy;
            normalized_w = w;
            normalized_h = h;
        }

        if (normalized_cx < 0 || normalized_cx > 1 ||
            normalized_cy < 0 || normalized_cy > 1 ||
            normalized_w <= 0 || normalized_w > 1 ||
            normalized_h <= 0 || normalized_h > 1) {
            continue;
        }

        float x1 = (normalized_cx - normalized_w / 2.0f) * modelWidth;
        float y1 = (normalized_cy - normalized_h / 2.0f) * modelHeight;
        float x2 = (normalized_cx + normalized_w / 2.0f) * modelWidth;
        float y2 = (normalized_cy + normalized_h / 2.0f) * modelHeight;

        float maxValue = 0;
        size_t maxIndex = 0;

        for (size_t j = 0; j < (size_t)classNum; ++j) {
            float value = classBuff[(4 + j) * num_boxes + i];
            if (value > maxValue) {
                maxIndex = j;
                maxValue = value;
            }
        }

        if (maxValue > 1.0f) {
            maxValue = 1.0f / (1.0f + expf(-maxValue));
        }

        if (maxValue > confThreshold) {
            // Debug: print raw bbox values before letterbox reverse
            if (need_normalize) {
                cout << "[DEBUG] box[" << i << "] raw(cx,cy,w,h)=(" << cx << "," << cy << ","
                     << w << "," << h << ") norm=(" << normalized_cx << "," << normalized_cy << ","
                     << normalized_w << "," << normalized_h << ") cls=" << maxIndex << " score=" << maxValue << endl;
            }
            YoloV8Box box;
            box.x1 = x1;
            box.y1 = y1;
            box.x2 = x2;
            box.y2 = y2;
            box.score = maxValue;
            box.class_id = (int)maxIndex;
            raw_boxes.push_back(box);
        }
    }

    cout << "[INFO] Before NMS: " << raw_boxes.size() << " boxes" << endl;

    nms(raw_boxes, nmsThreshold);

    if (raw_boxes.size() > (size_t)maxDet) {
        raw_boxes.erase(raw_boxes.begin() + maxDet, raw_boxes.end());
    }

    for (auto& box : raw_boxes) {
        box.x1 = round((box.x1 - tx1) / ratio);
        box.y1 = round((box.y1 - ty1) / ratio);
        box.x2 = round((box.x2 - tx1) / ratio);
        box.y2 = round((box.y2 - ty1) / ratio);
    }

    clip_boxes(raw_boxes, srcWidth, srcHeight);

    cout << "[INFO] After NMS: " << raw_boxes.size() << " boxes" << endl;
    for (size_t i = 0; i < raw_boxes.size(); i++) {
        cout << "  [" << i << "] class=" << raw_boxes[i].class_id
             << " (" << (raw_boxes[i].class_id < (int)class_names.size() ? class_names[raw_boxes[i].class_id] : "unknown") << ")"
             << " score=" << raw_boxes[i].score
             << " box=(" << raw_boxes[i].x1 << "," << raw_boxes[i].y1
             << "," << raw_boxes[i].x2 << "," << raw_boxes[i].y2 << ")" << endl;
    }

    // 9. Draw and save result
    cv::Mat resultImage = srcImage.clone();
    draw_result(resultImage, raw_boxes);
    cv::imwrite("out.jpg", resultImage);
    cout << "[INFO] Result saved to out.jpg" << endl;

    // 10. Cleanup
    model.DestroyInput();
    aclrtFree(imageDataBuf);
    model.DestroyResource();
    aclrtDestroyStream(stream);
    aclrtDestroyContext(context);
    aclrtResetDevice(0);
    aclFinalize();

    cout << "[INFO] Done" << endl;
    return 0;
}