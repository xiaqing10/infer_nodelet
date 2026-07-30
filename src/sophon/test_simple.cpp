#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include "detector_sophon.hpp"
#include "shm/shmmem.h"

static YoloV8_det g_detector;
static int g_frame = 0;
static double g_sum = 0;

struct alignas(1) Packet {
    int32_t magic;
    int32_t size;
    int16_t pixfmt;
    int16_t type;
    int32_t nalu;
    int32_t width;
    int32_t height;
    int64_t dts;
    char    reserved[32];
    char    data[0];
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_simple");
    ros::NodeHandle private_nh("~");

    std::string shm_name;
    std::string bmodel = "/home/files/catkin_ws/yolov8_bmcv_origin/yolov8s_int8_4b.bmodel";
    private_nh.param("test/shm_name", shm_name, std::string(""));
    private_nh.param("bmodel", bmodel, bmodel);

    if (shm_name.empty()) {
        printf("ERROR: shm_name not set. Use _test/shm_name:=/_k7_855_test_downstream_extra1_camera_image_raw.decoder\n");
        return 1;
    }

    printf("SHM name: %s\n", shm_name.c_str());
    printf("Bmodel: %s\n", bmodel.c_str());

    auto shm_reader = std::make_shared<ehawkeye::modules::units::shmmem>(shm_name, 30, false);
    printf("shmmem constructed: %s\n", shm_reader->path().c_str());

    g_detector.Init(bmodel);

    // Warmup / benchmark mode: wait for first frame, then loop 100 times on same image
    void* data = nullptr;
    int size = 0;
    int result = shm_reader->nocopyRead((void**)&data, size);
    if (result < 0 || !data) {
        printf("ERROR: failed to read first frame\n");
        return 1;
    }
    auto* pkt = static_cast<Packet*>(data);
    printf("pkt: magic=0x%x size=%d width=%d height=%d\n", pkt->magic, pkt->size, pkt->width, pkt->height);

    if (pkt->width <= 0 || pkt->height <= 0) {
        printf("ERROR: invalid frame dimensions\n");
        return 1;
    }

    cv::Mat mat = cv::Mat(pkt->height, pkt->width, CV_8UC3, pkt->data).clone();

    bm_image bmimg;
    cv::bmcv::toBMI(mat, &bmimg);
    std::vector<bm_image> batch = {bmimg};
    std::vector<YoloV8BoxVec> boxes;

    // Warmup
    printf("Warmup...\n");
    for (int i = 0; i < 10; i++) {
        g_detector.Detect(batch, boxes, 0);
    }

    // Benchmark
    printf("Benchmark 100 loops...\n");
    double sum = 0;
    int loops = 100;
    for (int i = 0; i < loops; i++) {
        auto t0 = std::chrono::steady_clock::now();
        g_detector.Detect(batch, boxes, 0);
        auto t1 = std::chrono::steady_clock::now();
        int us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        sum += us;
        if (i < 10 || (i+1) % 10 == 0)
            printf("loop=%d detect=%dus boxes=%zu\n", i+1, us, boxes[0].size());
    }
    printf("=== Avg: %.0fus/loop over %d loops ===\n", sum / loops, loops);

    bm_image_destroy(bmimg);

    // Now enter normal loop
    while (ros::ok()) {
        void* data2 = nullptr;
        int size2 = 0;
        int result2 = shm_reader->nocopyRead((void**)&data2, size2);
        if (result2 >= 0 && data2) {
            auto* pkt2 = static_cast<Packet*>(data2);
            cv::Mat mat2 = cv::Mat(pkt2->height, pkt2->width, CV_8UC3, pkt2->data).clone();
            bm_image bmimg2;
            cv::bmcv::toBMI(mat2, &bmimg2);
            std::vector<bm_image> batch2 = {bmimg2};
            std::vector<YoloV8BoxVec> boxes2;
            auto t0 = std::chrono::steady_clock::now();
            g_detector.Detect(batch2, boxes2, 0);
            auto t1 = std::chrono::steady_clock::now();
            int us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            g_sum += us;
            g_frame++;
            printf("frame=%d detect=%dus boxes=%zu avg=%dus\n", g_frame, us, boxes2[0].size(), (int)(g_sum / g_frame));
            bm_image_destroy(bmimg2);
        } else {
            usleep(1000);
        }
    }

    return 0;
}