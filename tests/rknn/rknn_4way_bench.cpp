#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include "rknn_api.h"
#include <opencv2/opencv.hpp>

static const rknn_core_mask kCoreMasks[3] = {
    RKNN_NPU_CORE_0,
    RKNN_NPU_CORE_1,
    RKNN_NPU_CORE_2
};

static unsigned char* load_model(const std::string& path, uint32_t* size) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return nullptr;
    *size = file.tellg();
    file.seekg(0, std::ios::beg);
    unsigned char* data = new unsigned char[*size];
    file.read(reinterpret_cast<char*>(data), *size);
    file.close();
    return data;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <model.rknn> <image.jpg> [num_iters]" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::string image_path = argv[2];
    int num_iters = 100;
    if (argc >= 4) num_iters = std::stoi(argv[3]);

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) { std::cerr << "Failed to load image" << std::endl; return 1; }

    uint32_t model_size = 0;
    unsigned char* model_data = load_model(model_path, &model_size);
    if (!model_data) { std::cerr << "Failed to load model" << std::endl; return 1; }

    int num_instances = 3;
    std::vector<rknn_context> ctxs(num_instances);
    for (int i = 0; i < num_instances; i++) {
        int ret = rknn_init(&ctxs[i], model_data, model_size, 0, NULL);
        if (ret < 0) { std::cerr << "rknn_init failed for " << i << std::endl; return 1; }
        ret = rknn_set_core_mask(ctxs[i], kCoreMasks[i]);
        if (ret < 0) { std::cerr << "rknn_set_core_mask failed for " << i << std::endl; return 1; }
        std::cout << "Instance " << i << " bound to core " << i << std::endl;
    }
    delete[] model_data;

    rknn_input_output_num io_num;
    rknn_query(ctxs[0], RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    rknn_tensor_attr input_attr;
    input_attr.index = 0;
    rknn_query(ctxs[0], RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
    int net_w = input_attr.dims[2];
    int net_h = input_attr.dims[1];

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(net_w, net_h));

    rknn_input inputs[1];
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_w * net_h * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = resized.data;

    int num_cameras = 4;
    int camera_to_instance[4] = {0, 1, 2, 0};

    for (int i = 0; i < 10; i++) {
        int idx = camera_to_instance[i % num_cameras];
        rknn_inputs_set(ctxs[idx], 1, inputs);
        rknn_run(ctxs[idx], NULL);
    }

    {
        std::cout << "\n=== 4 cameras, 3 instances (instance 0 shared by 2 cameras) ===" << std::endl;
        int parallel_iters = num_iters;
        auto pt0 = std::chrono::steady_clock::now();
        {
            std::vector<std::thread> threads;
            for (int c = 0; c < num_cameras; c++) {
                threads.emplace_back([&ctxs, &inputs, c, &camera_to_instance, parallel_iters]() {
                    int idx = camera_to_instance[c];
                    for (int i = 0; i < parallel_iters; i++) {
                        rknn_inputs_set(ctxs[idx], 1, inputs);
                        rknn_run(ctxs[idx], NULL);
                    }
                });
            }
            for (auto& th : threads) th.join();
        }
        auto pt1 = std::chrono::steady_clock::now();
        auto total_par_us = std::chrono::duration_cast<std::chrono::microseconds>(pt1 - pt0).count();
        int total_par_iters = parallel_iters * num_cameras;
        double avg_par_us = (double)total_par_us / total_par_iters;
        double par_fps = 1000000.0 / avg_par_us;
        std::cout << "Total: " << total_par_us / 1000 << " ms for " << total_par_iters << " iters" << std::endl;
        std::cout << "Avg:   " << avg_par_us << " us / iter" << std::endl;
        std::cout << "FPS:   " << par_fps << std::endl;
        std::cout << "Per camera: " << par_fps / num_cameras << " FPS" << std::endl;
    }

    for (auto& ctx : ctxs) rknn_destroy(ctx);
    return 0;
}