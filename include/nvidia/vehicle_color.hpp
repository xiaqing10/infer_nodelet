#ifndef VEHICLE_COLOR_NVIDIA_H
#define VEHICLE_COLOR_NVIDIA_H

#include <numeric>
#include <time.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <string>
#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "opencv2/opencv.hpp"

#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)\
    {\
        cudaError_t error_code = callstr;\
        if (error_code != cudaSuccess) {\
            std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__;\
            assert(0);\
        }\
    }
#endif

typedef struct ClsRetData{
    int label;
    float confidence;
}ClsRetData;

// 数值稳定的 softmax 函数
static std::vector<float> softmax(float *prob, int n) {
    std::vector<float> res;
    // 数值稳定的 softmax：先减去最大值
    float max_val = prob[0];
    for (int i = 1; i < n; i++) {
        if (prob[i] > max_val) max_val = prob[i];
    }

    float sum = 0.0f;
    float t;
    for (int i = 0; i < n; i++) {
        t = expf(prob[i] - max_val);
        res.push_back(t);
        sum += t;
    }
    for (int i = 0; i < n; i++) {
        res[i] /= sum;
    }
    return res;
}

// topk 函数
static std::vector<int> topk(const std::vector<float>& vec, int k) {
    std::vector<int> topk_index;
    std::vector<size_t> vec_index(vec.size());
    std::iota(vec_index.begin(), vec_index.end(), 0);

    std::sort(vec_index.begin(), vec_index.end(), [&vec](size_t index_1, size_t index_2) { 
        return vec[index_1] > vec[index_2]; 
    });

    int k_num = std::min<int>(vec.size(), k);

    for (int i = 0; i < k_num; ++i) {
        topk_index.push_back(vec_index[i]);
    }

    return topk_index;
}

class VehicleColorDetector {
public:
    VehicleColorDetector() {}
    ~VehicleColorDetector() {
        if (context) context->destroy();
        if (engine) engine->destroy();
        if (runtime) runtime->destroy();
        if (stream) cudaStreamDestroy(stream);
        // 正确释放GPU显存
        if (buffers[0]) cudaFree(buffers[0]);
        if (buffers[1]) cudaFree(buffers[1]);
    }

    int init(std::string engine_file_path) {
        std::ifstream file(engine_file_path, std::ios::binary);
        assert(file.good());
        file.seekg(0, std::ios::end);
        auto size = file.tellg();
        file.seekg(0, std::ios::beg);
        char* trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
        initLibNvInferPlugins(&gLogger, "");
        runtime = nvinfer1::createInferRuntime(gLogger);
        assert(runtime != nullptr);

        engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        delete[] trtModelStream;
        context = engine->createExecutionContext();

        assert(context != nullptr);
        cudaStreamCreate(&stream);
        assert(engine->getNbBindings() == 2);

        // 打印binding信息便于调试
        std::string input_name = engine->getBindingName(0);
        std::string output_name = engine->getBindingName(1);
        std::cout << "Input binding: " << input_name << std::endl;
        std::cout << "Output binding: " << output_name << std::endl;

        const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
        const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(inputIndex == 0);
        assert(outputIndex == 1);
        CUDA_CHECK(cudaMalloc((void**)&buffers[inputIndex], 1 * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&buffers[outputIndex], 1 * OUTPUT_SIZE * sizeof(float)));

        std::cout << "Model initialized: " << INPUT_W << "x" << INPUT_H << ", " << OUTPUT_SIZE << " classes" << std::endl;

        return 0;
    }

    ClsRetData inference(cv::Mat &img) {
        // resize到网络输入尺寸
        cv::resize(img, img, cv::Size(INPUT_W, INPUT_H));
        
        // 预处理：BGR -> RGB，CHW格式，ImageNet归一化
        int i = 0;
        for (int row = 0; row < INPUT_H; ++row) {
            uchar* uc_pixel = img.data + row * img.step;
            for (int col = 0; col < INPUT_W; ++col) {
                // CHW格式
                data[i] = ((float)uc_pixel[2] / 255.0 - 0.485) / 0.229;  // R
                data[i + INPUT_H * INPUT_W] = ((float)uc_pixel[1] / 255.0 - 0.456) / 0.224;  // G
                data[i + 2 * INPUT_H * INPUT_W] = ((float)uc_pixel[0] / 255.0 - 0.406) / 0.225;  // B
                uc_pixel += 3;
                ++i;
            }
        }

        CUDA_CHECK(cudaMemcpyAsync(buffers[0], data, 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context->enqueue(1, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(prob, buffers[1], 1 * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);

        // 使用数值稳定的softmax
        auto res = softmax(prob, OUTPUT_SIZE);
        auto topk_idx = topk(res, 1);
	
	/*
        // 调试功能：保存高置信度图片
        for (auto idx : topk_idx) {
            if (res[idx] > 0.95) {
                char filename[256] = {0};
                snprintf(filename, sizeof(filename), "/tmp/videos/%d___%f_vehicle.jpg", idx + 1, res[idx]);
                std::cout << "save image to " << filename << std::endl;
                cv::imwrite(filename, img);
            }
        }
	*/

        ClsRetData ret_data;
        ret_data.label = topk_idx[0] + 1;  // 输出 1-8
        ret_data.confidence = res[topk_idx[0]];
        return ret_data;
    }

private:
    const char* INPUT_BLOB_NAME = "data";
    const char* OUTPUT_BLOB_NAME = "prob";

    int INPUT_W = 128;
    int INPUT_H = 128;
    int INPUT_C = 3;
    int OUTPUT_SIZE = 8;
    float data[3 * 128 * 128];
    float prob[8];

    void* buffers[2];

    nvinfer1::ICudaEngine*       engine  = nullptr;
    nvinfer1::IRuntime*          runtime = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t                 stream  = nullptr;
    Logger                       gLogger{nvinfer1::ILogger::Severity::kERROR};
};

#endif // VEHICLE_COLOR_NVIDIA_H
