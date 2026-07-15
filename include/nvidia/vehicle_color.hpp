#ifndef VEHICLE_COLOR_NVIDIA_H
#define VEHICLE_COLOR_NVIDIA_H

#include <numeric>
#include <time.h>
#include <iostream>
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

class VehicleColorDetector {
public:
    VehicleColorDetector() {}
    ~VehicleColorDetector() {
        if (context) context->destroy();
        if (engine) engine->destroy();
        if (runtime) runtime->destroy();
        if (stream) cudaStreamDestroy(stream);
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

        const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
        const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(inputIndex == 0);
        assert(outputIndex == 1);
        CUDA_CHECK(cudaMalloc((void**)&buffers[inputIndex], 1 * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&buffers[outputIndex], 1 * OUTPUT_SIZE * sizeof(float)));

        return 0;
    }

    ClsRetData inference(cv::Mat &img) {
        cv::resize(img, img, cv::Size(INPUT_W, INPUT_H));
        int i = 0;
        for (int row = 0; row < INPUT_H; ++row) {
            uchar* uc_pixel = img.data + row * img.step;
            for (int col = 0; col < INPUT_W; ++col) {
                data[i] = ((float)uc_pixel[2] / 255.0 - 0.485) / 0.229;
                data[i + INPUT_H * INPUT_W] = ((float)uc_pixel[1] / 255.0 - 0.456) / 0.224;
                data[i + 2 * INPUT_H * INPUT_W] = ((float)uc_pixel[0] / 255.0 - 0.406) / 0.225;
                uc_pixel += 3;
                ++i;
            }
        }

        CUDA_CHECK(cudaMemcpyAsync(buffers[0], data, 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context->enqueue(1, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(prob, buffers[1], 1 * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);

        std::vector<float> res;
        float sum = 0.0f;
        for (int j = 0; j < OUTPUT_SIZE; j++) {
            float t = expf(prob[j]);
            res.push_back(t);
            sum += t;
        }
        for (int j = 0; j < OUTPUT_SIZE; j++) {
            res[j] /= sum;
        }

        int max_idx = 0;
        for (int j = 1; j < OUTPUT_SIZE; j++) {
            if (res[j] > res[max_idx]) max_idx = j;
        }

        ClsRetData ret_data;
        ret_data.label = max_idx + 1;
        ret_data.confidence = res[max_idx];
        return ret_data;
    }

private:
    const char* INPUT_BLOB_NAME = "input.1";
    const char* OUTPUT_BLOB_NAME = "191";

    int INPUT_W = 224;
    int INPUT_H = 224;
    int INPUT_C = 3;
    int OUTPUT_SIZE = 6;
    float data[3 * 224 * 224];
    float prob[6];

    void* buffers[2];

    nvinfer1::ICudaEngine*       engine  = nullptr;
    nvinfer1::IRuntime*          runtime = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t                 stream  = nullptr;
    Logger                       gLogger{nvinfer1::ILogger::Severity::kERROR};
};

#endif