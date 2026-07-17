#include "detector_ascend.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

using namespace std;

const std::vector<std::vector<int>> colors = {
    {255, 0, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0},
    {170, 255, 0}, {85, 255, 0}, {0, 255, 0}, {0, 255, 85},
    {0, 255, 170}, {0, 255, 255}, {0, 170, 255}, {0, 85, 255},
    {0, 0, 255}, {85, 0, 255}, {170, 0, 255}, {255, 0, 255},
    {255, 0, 170}, {255, 0, 85}, {255, 0, 0}, {255, 0, 255},
    {255, 85, 255}, {255, 170, 255}, {255, 255, 255}, {170, 255, 255},
    {85, 255, 255}};

const std::vector<std::string> coco_labels = {
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
    "toothbrush"
};

#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)

YoloV8_det::YoloV8_det(int deviceId) : deviceId_(deviceId) {
    std::cout << "[INFO] YoloV8_det constructor, deviceId: " << deviceId_ << std::endl;
}

YoloV8_det::YoloV8_det(YoloV8_det&& other) noexcept
    : models_(std::move(other.models_)),
      context_(other.context_),
      stream_(other.stream_),
      runMode_(other.runMode_),
      g_imageDataBuf_(other.g_imageDataBuf_),
      g_imageDataSize_(other.g_imageDataSize_),
      modelPath_(std::move(other.modelPath_)),
      modelWidth_(other.modelWidth_),
      modelHeight_(other.modelHeight_),
      confThreshold_(other.confThreshold_),
      nmsThreshold_(other.nmsThreshold_),
      classNum_(other.classNum_),
      max_det_(other.max_det_),
      modelCount_(other.modelCount_),
      srcWidth_(other.srcWidth_),
      srcHeight_(other.srcHeight_),
      inferOutputs_(std::move(other.inferOutputs_)),
      txy_batch(std::move(other.txy_batch)),
      ratios_batch(std::move(other.ratios_batch)),
      deviceId_(other.deviceId_),
      is_initialized_(other.is_initialized_) {
    other.context_ = nullptr;
    other.stream_ = nullptr;
    other.runMode_ = ACL_HOST;
    other.g_imageDataBuf_ = nullptr;
    other.g_imageDataSize_ = 0;
    other.is_initialized_ = false;
}

YoloV8_det& YoloV8_det::operator=(YoloV8_det&& other) noexcept {
    if (this != &other) {
        Cleanup();

        models_ = std::move(other.models_);
        context_ = other.context_;
        stream_ = other.stream_;
        runMode_ = other.runMode_;
        g_imageDataBuf_ = other.g_imageDataBuf_;
        g_imageDataSize_ = other.g_imageDataSize_;
        modelPath_ = std::move(other.modelPath_);
        modelWidth_ = other.modelWidth_;
        modelHeight_ = other.modelHeight_;
        confThreshold_ = other.confThreshold_;
        nmsThreshold_ = other.nmsThreshold_;
        classNum_ = other.classNum_;
        max_det_ = other.max_det_;
        modelCount_ = other.modelCount_;
        srcWidth_ = other.srcWidth_;
        srcHeight_ = other.srcHeight_;
        inferOutputs_ = std::move(other.inferOutputs_);
        txy_batch = std::move(other.txy_batch);
        ratios_batch = std::move(other.ratios_batch);
        deviceId_ = other.deviceId_;
        is_initialized_ = other.is_initialized_;

        other.context_ = nullptr;
        other.stream_ = nullptr;
        other.runMode_ = ACL_HOST;
        other.g_imageDataBuf_ = nullptr;
        other.g_imageDataSize_ = 0;
        other.is_initialized_ = false;
    }
    return *this;
}

YoloV8_det::~YoloV8_det() {
    Cleanup();
    std::cout << "[INFO] YoloV8_det destructor, deviceId: " << deviceId_ << std::endl;
}

void YoloV8_det::Cleanup() {
    if (is_initialized_) {
        if (!models_.empty()) {
            for (auto& model : models_) {
                if (model) {
                    model->DestroyResource();
                }
            }
            models_.clear();
        }

        if (g_imageDataBuf_ != nullptr) {
            aclrtFree(g_imageDataBuf_);
            g_imageDataBuf_ = nullptr;
            g_imageDataSize_ = 0;
            std::cout << "[INFO] Input buffer freed for device " << deviceId_ << std::endl;
        }

        if (stream_ != nullptr) {
            aclrtDestroyStream(stream_);
            stream_ = nullptr;
            std::cout << "[INFO] Stream destroyed for device " << deviceId_ << std::endl;
        }

        if (context_ != nullptr) {
            aclrtDestroyContext(context_);
            context_ = nullptr;
            std::cout << "[INFO] Context destroyed for device " << deviceId_ << std::endl;
        }

        aclrtResetDevice(deviceId_);
        std::cout << "[INFO] Device " << deviceId_ << " reset" << std::endl;

        AclGlobalManager::Release();

        is_initialized_ = false;
        std::cout << "[INFO] Detector cleaned up for device " << deviceId_ << std::endl;
    }
}

int YoloV8_det::SetCurrentContext() {
    if (context_ == nullptr) {
        std::cerr << "[ERROR] Context is null for device " << deviceId_ << std::endl;
        return -1;
    }

    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Set current context failed for device " << deviceId_
                  << ", error: " << ret << std::endl;
        return -1;
    }

    return 0;
}

int YoloV8_det::Init(const std::string& model_path, int device_id) {
    if (device_id >= 0) {
        deviceId_ = device_id;
    }

    return InitInternal(model_path);
}

int YoloV8_det::InitInternal(const std::string& model_path) {
    if (is_initialized_) {
        std::cout << "[WARN] Detector already initialized for device " << deviceId_ << std::endl;
        return 0;
    }

    modelPath_ = model_path;

    std::cout << "\n=== YoloV8_det::Init Start ===" << std::endl;
    std::cout << "Device ID: " << deviceId_ << std::endl;
    std::cout << "Model path: " << modelPath_ << std::endl;

    int ret = AclGlobalManager::Init();
    if (ret != 0) {
        std::cerr << "[ERROR] ACL global initialization failed" << std::endl;
        return -1;
    }

    aclError acl_ret = aclrtSetDevice(deviceId_);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Set device " << deviceId_ << " failed, error: " << acl_ret << std::endl;
        AclGlobalManager::Release();
        return -1;
    }
    std::cout << "[INFO] Open device " << deviceId_ << " ok" << std::endl;

    acl_ret = aclrtCreateContext(&context_, deviceId_);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Create context failed for device " << deviceId_
                  << ", error: " << acl_ret << std::endl;
        aclrtResetDevice(deviceId_);
        AclGlobalManager::Release();
        return -1;
    }
    std::cout << "[INFO] Create context ok for device " << deviceId_ << std::endl;

    acl_ret = aclrtSetCurrentContext(context_);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Set current context failed for device " << deviceId_
                  << ", error: " << acl_ret << std::endl;
        aclrtDestroyContext(context_);
        context_ = nullptr;
        aclrtResetDevice(deviceId_);
        AclGlobalManager::Release();
        return -1;
    }

    acl_ret = aclrtCreateStream(&stream_);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Create stream failed for device " << deviceId_
                  << ", error: " << acl_ret << std::endl;
        aclrtDestroyContext(context_);
        context_ = nullptr;
        aclrtResetDevice(deviceId_);
        AclGlobalManager::Release();
        return -1;
    }
    std::cout << "[INFO] Create stream ok for device " << deviceId_ << std::endl;

    acl_ret = aclrtGetRunMode(&runMode_);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Get runMode failed for device " << deviceId_
                  << ", error: " << acl_ret << std::endl;
        aclrtDestroyStream(stream_);
        stream_ = nullptr;
        aclrtDestroyContext(context_);
        context_ = nullptr;
        aclrtResetDevice(deviceId_);
        AclGlobalManager::Release();
        return -1;
    }

    std::cout << "Running in " << (runMode_ == ACL_HOST ? "HOST" : "DEVICE")
              << " mode on device " << deviceId_ << std::endl;

    g_imageDataSize_ = RGBU8_IMAGE_SIZE(modelWidth_, modelHeight_);
    acl_ret = aclrtMalloc(&g_imageDataBuf_, g_imageDataSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (acl_ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Malloc device input buffer failed on device " << deviceId_
                  << ", error: " << acl_ret << std::endl;
        aclrtDestroyStream(stream_);
        stream_ = nullptr;
        aclrtDestroyContext(context_);
        context_ = nullptr;
        aclrtResetDevice(deviceId_);
        AclGlobalManager::Release();
        return -1;
    }
    std::cout << "[INFO] Allocated device input buffer: " << g_imageDataSize_ << " bytes" << std::endl;

    models_.clear();
    for (int i = 0; i < modelCount_; i++) {
        auto model = std::make_unique<AclLiteModel>();

        AclLiteError model_ret = model->Init(modelPath_.c_str());
        if (model_ret != ACL_SUCCESS) {
            std::cerr << "[ERROR] Model " << i << " init failed on device " << deviceId_
                      << ", error: " << model_ret << std::endl;
            for (auto& m : models_) {
                m->DestroyResource();
            }
            models_.clear();

            aclrtFree(g_imageDataBuf_);
            g_imageDataBuf_ = nullptr;
            aclrtDestroyStream(stream_);
            stream_ = nullptr;
            aclrtDestroyContext(context_);
            context_ = nullptr;
            aclrtResetDevice(deviceId_);
            AclGlobalManager::Release();
            return -1;
        }

        model_ret = model->CreateInput(g_imageDataBuf_, g_imageDataSize_);
        if (model_ret != ACL_SUCCESS) {
            std::cerr << "[ERROR] Model " << i << " CreateInput failed on device " << deviceId_
                      << ", error: " << model_ret << std::endl;
            for (auto& m : models_) {
                m->DestroyResource();
            }
            models_.clear();

            aclrtFree(g_imageDataBuf_);
            g_imageDataBuf_ = nullptr;
            aclrtDestroyStream(stream_);
            stream_ = nullptr;
            aclrtDestroyContext(context_);
            context_ = nullptr;
            aclrtResetDevice(deviceId_);
            AclGlobalManager::Release();
            return -1;
        }

        models_.push_back(std::move(model));
        std::cout << "Model " << i << " loaded successfully on device " << deviceId_ << std::endl;
    }

    is_initialized_ = true;

    std::cout << "YOLOv8 model initialized successfully on device " << deviceId_ << "!" << std::endl;
    std::cout << "Model size: " << modelWidth_ << "x" << modelHeight_ << std::endl;
    std::cout << "Model count: " << modelCount_ << std::endl;
    std::cout << "Confidence threshold: " << confThreshold_ << std::endl;
    std::cout << "NMS threshold: " << nmsThreshold_ << std::endl;
    std::cout << "Class number: " << classNum_ << std::endl;
    std::cout << "=== YoloV8_det::Init Complete ===\n" << std::endl;

    return 0;
}

float YoloV8_det::get_aspect_scaled_ratio(int src_w, int src_h, int dst_w, int dst_h, bool* pIsAligWidth) {
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

int YoloV8_det::pre_process(const cv::Mat& srcImage) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    if (srcImage.empty()) {
        std::cerr << "[ERROR] Source image is empty" << std::endl;
        return -1;
    }

    if (SetCurrentContext() != 0) {
        return -1;
    }

    srcWidth_ = srcImage.cols;
    srcHeight_ = srcImage.rows;

    txy_batch.clear();
    ratios_batch.clear();

    bool isAlignWidth = false;
    float ratio = get_aspect_scaled_ratio(srcWidth_, srcHeight_, modelWidth_, modelHeight_, &isAlignWidth);

    int tx1 = 0, ty1 = 0;
    if (isAlignWidth) {
        int resized_h = srcHeight_ * ratio;
        ty1 = (modelHeight_ - resized_h) / 2;
    } else {
        int resized_w = srcWidth_ * ratio;
        tx1 = (modelWidth_ - resized_w) / 2;
    }

    txy_batch.push_back(std::make_pair(tx1, ty1));
    ratios_batch.push_back(std::make_pair(ratio, ratio));

    cv::Mat rgbImage;
    cv::cvtColor(srcImage, rgbImage, cv::COLOR_BGR2RGB);

    int resized_w, resized_h;
    if (isAlignWidth) {
        resized_w = modelWidth_;
        resized_h = (int)(srcHeight_ * ratio);
    } else {
        resized_w = (int)(srcWidth_ * ratio);
        resized_h = modelHeight_;
    }

    cv::Mat resizedImage;
    cv::resize(rgbImage, resizedImage, cv::Size(resized_w, resized_h));

    cv::Mat canvas(modelHeight_, modelWidth_, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat roi = canvas(cv::Rect(tx1, ty1, resized_w, resized_h));
    resizedImage.copyTo(roi);

    aclrtMemcpyKind policy = (runMode_ == ACL_HOST) ?
                             ACL_MEMCPY_HOST_TO_DEVICE : ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError ret = aclrtMemcpy(g_imageDataBuf_, g_imageDataSize_,
                               canvas.ptr<uint8_t>(), g_imageDataSize_, policy);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Copy resized image data to device failed on device " << deviceId_
                  << ", error: " << ret << std::endl;
        return -1;
    }

    return 0;
}

int YoloV8_det::pre_process_batch(const std::vector<cv::Mat>& images) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    if (images.empty()) {
        std::cerr << "[ERROR] No images provided" << std::endl;
        return -1;
    }
    return pre_process(images[0]);
}

int YoloV8_det::detect(std::vector<InferenceOutput>& inferOutputs, int index) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    if (index < 0 || index >= models_.size()) {
        std::cerr << "[ERROR] Invalid model index: " << index << std::endl;
        return -1;
    }

    if (SetCurrentContext() != 0) {
        return -1;
    }

    AclLiteError ret = models_[index]->Execute(inferOutputs);
    if (ret != ACL_SUCCESS) {
        std::cerr << "[ERROR] Execute model failed on device " << deviceId_
                  << ", errorCode is " << ret << std::endl;
        return -1;
    }

    return 0;
}

int YoloV8_det::argmax(float* data, int num) {
    float max_value = 0.0;
    int max_index = 0;
    for (int i = 0; i < num; ++i) {
        float value = data[i];
        if (value > max_value) {
            max_value = value;
            max_index = i;
        }
    }
    return max_index;
}

void YoloV8_det::NMS(YoloV8BoxVec& dets, float nmsConfidence) {
    if (dets.empty()) return;

    std::sort(dets.begin(), dets.end(), [](const YoloV8Box& a, const YoloV8Box& b) {
        return a.score > b.score;
    });

    std::vector<float> areas(dets.size());
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

            float xx1 = std::max(dets[i].x1, dets[j].x1);
            float yy1 = std::max(dets[i].y1, dets[j].y1);
            float xx2 = std::min(dets[i].x2, dets[j].x2);
            float yy2 = std::min(dets[i].y2, dets[j].y2);

            float w = std::max(0.0f, xx2 - xx1);
            float h = std::max(0.0f, yy2 - yy1);
            float inter = w * h;
            float iou = inter / (areas[i] + areas[j] - inter);

            if (iou > nmsConfidence) {
                dets[j].score = 0;
            }
        }
    }

    dets.erase(std::remove_if(dets.begin(), dets.end(),
                             [](const YoloV8Box& box) { return box.score == 0; }),
               dets.end());
}

void YoloV8_det::clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h) {
    for (auto& box : yolobox_vec) {
        box.x1 = std::max(0.0f, std::min(box.x1, (float)src_w));
        box.y1 = std::max(0.0f, std::min(box.y1, (float)src_h));
        box.x2 = std::max(0.0f, std::min(box.x2, (float)src_w));
        box.y2 = std::max(0.0f, std::min(box.y2, (float)src_h));
    }
}

int YoloV8_det::post_process(const std::vector<InferenceOutput>& inferOutputs,
                             YoloV8BoxVec& boxes) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    if (inferOutputs.empty()) {
        std::cerr << "[ERROR] No inference outputs on device " << deviceId_ << std::endl;
        return -1;
    }

    uint32_t outputDataBufId = 0;
    float *classBuff = static_cast<float *>(inferOutputs[outputDataBufId].data.get());
    size_t total_elements = inferOutputs[outputDataBufId].size / sizeof(float);
    size_t num_features = 4 + classNum_;
    size_t num_boxes = total_elements / num_features;

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
            normalized_cx = cx / modelWidth_;
            normalized_cy = cy / modelHeight_;
            normalized_w = w / modelWidth_;
            normalized_h = h / modelHeight_;
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

        float x1 = (normalized_cx - normalized_w / 2.0f) * modelWidth_;
        float y1 = (normalized_cy - normalized_h / 2.0f) * modelHeight_;
        float x2 = (normalized_cx + normalized_w / 2.0f) * modelWidth_;
        float y2 = (normalized_cy + normalized_h / 2.0f) * modelHeight_;

        float maxValue = 0;
        size_t maxIndex = 0;

        for (size_t j = 0; j < (size_t)classNum_; ++j) {
            float value = classBuff[(4 + j) * num_boxes + i];
            if (value > maxValue) {
                maxIndex = j;
                maxValue = value;
            }
        }

        if (maxValue > 1.0f) {
            maxValue = 1.0f / (1.0f + expf(-maxValue));
        }

        if (maxValue > confThreshold_) {
            YoloV8Box box;
            box.x1 = x1;
            box.y1 = y1;
            box.x2 = x2;
            box.y2 = y2;
            box.score = maxValue;
            box.class_id = maxIndex;
            raw_boxes.push_back(box);
        }
    }

    NMS(raw_boxes, nmsThreshold_);

    if (raw_boxes.size() > max_det_) {
        raw_boxes.erase(raw_boxes.begin() + max_det_, raw_boxes.end());
    }

    if (!txy_batch.empty() && !ratios_batch.empty()) {
        int tx1 = txy_batch[0].first;
        int ty1 = txy_batch[0].second;
        float ratio_x = ratios_batch[0].first;
        float ratio_y = ratios_batch[0].second;

        for (auto& box : raw_boxes) {
            box.x1 = std::round((box.x1 - tx1) / ratio_x);
            box.y1 = std::round((box.y1 - ty1) / ratio_y);
            box.x2 = std::round((box.x2 - tx1) / ratio_x);
            box.y2 = std::round((box.y2 - ty1) / ratio_y);
        }
    }

    clip_boxes(raw_boxes, srcWidth_, srcHeight_);
    boxes = raw_boxes;

    return 0;
}

int YoloV8_det::post_process_batch(const std::vector<InferenceOutput>& inferOutputs,
                                   std::vector<YoloV8BoxVec>& boxes_batch) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    YoloV8BoxVec boxes;
    int ret = post_process(inferOutputs, boxes);
    if (ret == 0) {
        boxes_batch.push_back(boxes);
    }
    return ret;
}

int YoloV8_det::Detect(const cv::Mat& image, YoloV8BoxVec& boxes, int index) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    aclrtSetDevice(deviceId_);

    if (SetCurrentContext() != 0) {
        return -1;
    }

    int ret = pre_process(image);
    if (ret != 0) {
        std::cerr << "[ERROR] Pre-process failed on device " << deviceId_ << std::endl;
        return -1;
    }

    std::vector<InferenceOutput> inferOutputs;
    ret = detect(inferOutputs, index);
    if (ret != 0) {
        std::cerr << "[ERROR] Inference failed on device " << deviceId_ << std::endl;
        return -1;
    }

    ret = post_process(inferOutputs, boxes);
    if (ret != 0) {
        std::cerr << "[ERROR] Post-process failed on device " << deviceId_ << std::endl;
        return -1;
    }

    return 0;
}

int YoloV8_det::DetectBatch(const std::vector<cv::Mat>& images,
                            std::vector<YoloV8BoxVec>& boxes_batch, int index) {
    if (!is_initialized_) {
        std::cerr << "[ERROR] Detector not initialized" << std::endl;
        return -1;
    }

    boxes_batch.clear();

    for (const auto& image : images) {
        YoloV8BoxVec boxes;
        int ret = Detect(image, boxes, index);
        if (ret != 0) {
            std::cerr << "[ERROR] Detect failed for one image on device " << deviceId_ << std::endl;
            return -1;
        }
        boxes_batch.push_back(boxes);
    }

    return 0;
}

void YoloV8_det::draw_result(cv::Mat& img, YoloV8BoxVec& result) {
    for (const auto& box : result) {
        if (box.score < confThreshold_) continue;

        int left = box.x1;
        int top = box.y1;
        int right = box.x2;
        int bottom = box.y2;

        int color_idx = box.class_id % colors.size();
        cv::Scalar color(colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);

        cv::rectangle(img, cv::Point(left, top), cv::Point(right, bottom), color, 2);

        std::string label;
        if (box.class_id < coco_labels.size()) {
            label = coco_labels[box.class_id];
        } else {
            label = "class_" + std::to_string(box.class_id);
        }
        label += ": " + std::to_string(box.score).substr(0, 4);

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        cv::rectangle(img,
                      cv::Point(left, top - text_size.height - 5),
                      cv::Point(left + text_size.width, top),
                      color, cv::FILLED);

        cv::putText(img, label,
                    cv::Point(left, top - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

void YoloV8_det::Release() {
    Cleanup();
}