#ifndef YOLOV8_DET_ASCEND_H
#define YOLOV8_DET_ASCEND_H

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <mutex>
#include "opencv2/opencv.hpp"
#include "acl/acl.h"
#include "AclLiteUtils.h"
#include "AclLiteModel.h"
#include <chrono>
#include "utils.hpp"

#define DEBUG 0

struct YoloV8Box {
    float x1, y1, x2, y2;
    float score;
    int class_id;
};

using YoloV8BoxVec = std::vector<YoloV8Box>;

class AclGlobalManager {
private:
    static std::mutex mutex_;
    static bool is_initialized_;
    static int ref_count_;

public:
    static int Init() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!is_initialized_) {
            aclError ret = aclInit(nullptr);
            if (ret != ACL_SUCCESS) {
                std::cerr << "[ERROR] ACL global init failed, errorcode: " << ret << std::endl;
                return -1;
            }
            std::cout << "[INFO] ACL global init ok" << std::endl;
            is_initialized_ = true;
        }

        ref_count_++;
        return 0;
    }

    static void Release() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (is_initialized_) {
            ref_count_--;
            if (ref_count_ <= 0) {
                aclError ret = aclFinalize();
                if (ret != ACL_SUCCESS) {
                    std::cerr << "[ERROR] ACL global finalize failed, errorcode: " << ret << std::endl;
                } else {
                    std::cout << "[INFO] ACL global finalize ok" << std::endl;
                }
                is_initialized_ = false;
                ref_count_ = 0;
            }
        }
    }

    static bool IsInitialized() { return is_initialized_; }
};

inline std::mutex AclGlobalManager::mutex_;
inline bool AclGlobalManager::is_initialized_ = false;
inline int AclGlobalManager::ref_count_ = 0;

class YoloV8_det {
private:
    std::vector<std::unique_ptr<AclLiteModel>> models_;

    aclrtContext context_ = nullptr;
    aclrtStream stream_ = nullptr;

    aclrtRunMode runMode_;

    void* g_imageDataBuf_ = nullptr;
    uint32_t g_imageDataSize_ = 0;

    std::string modelPath_;
    int32_t modelWidth_ = 640;
    int32_t modelHeight_ = 640;
    float confThreshold_ = 0.35;
    float nmsThreshold_ = 0.45;
    int classNum_ = 10;
    int max_det_ = 300;
    int modelCount_ = 1;

    int srcWidth_;
    int srcHeight_;
    std::vector<InferenceOutput> inferOutputs_;

    std::vector<std::pair<int, int>> txy_batch;
    std::vector<std::pair<float, float>> ratios_batch;

    int deviceId_ = 0;

    bool is_initialized_ = false;

public:
    explicit YoloV8_det(int deviceId = 0);

    YoloV8_det(const YoloV8_det&) = delete;
    YoloV8_det& operator=(const YoloV8_det&) = delete;
    YoloV8_det(YoloV8_det&& other) noexcept;
    YoloV8_det& operator=(YoloV8_det&& other) noexcept;

    ~YoloV8_det();

    int batch_size = 1;

    int Init(const std::string& model_path, int device_id = -1);

    bool IsInitialized() const { return is_initialized_; }
    int GetDeviceId() const { return deviceId_; }
    const std::string& GetModelPath() const { return modelPath_; }

    int Detect(const cv::Mat& image, YoloV8BoxVec& boxes, int index = 0);
    int DetectBatch(const std::vector<cv::Mat>& images,
                    std::vector<YoloV8BoxVec>& boxes_batch, int index = 0);

    void draw_result(cv::Mat& img, YoloV8BoxVec& result);
    void Release();

private:
    int InitInternal(const std::string& model_path);
    void Cleanup();
    int SetCurrentContext();

    int pre_process(const cv::Mat& srcImage);
    int pre_process_batch(const std::vector<cv::Mat>& images);

    int detect(std::vector<InferenceOutput>& inferOutputs, int index);

    int post_process(const std::vector<InferenceOutput>& inferOutputs,
                     YoloV8BoxVec& boxes);
    int post_process_batch(const std::vector<InferenceOutput>& inferOutputs,
                          std::vector<YoloV8BoxVec>& boxes_batch);

    static float get_aspect_scaled_ratio(int src_w, int src_h, int dst_w, int dst_h, bool* alignWidth);
    void NMS(YoloV8BoxVec& dets, float nmsConfidence);
    void clip_boxes(YoloV8BoxVec& yolobox_vec, int src_w, int src_h);
    int argmax(float* data, int num);
};

#endif
