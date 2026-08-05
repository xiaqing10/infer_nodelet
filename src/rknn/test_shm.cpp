#include <ros/ros.h>
#include <cstdio>
#include <string>
#include "shm/shmmem.h"

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
    ros::init(argc, argv, "test_shm");
    ros::NodeHandle private_nh("~");

    std::string shm_name;
    private_nh.param("test/shm_name", shm_name, std::string(""));

    if (shm_name.empty()) {
        printf("ERROR: shm_name not set. Use _test/shm_name:=/...\n");
        return 1;
    }

    printf("SHM name: %s\n", shm_name.c_str());

    auto shm_reader = std::make_shared<ehawkeye::modules::units::shmmem>(shm_name, 30, false);
    printf("shmmem constructed: %s\n", shm_reader->path().c_str());

    int timeout_ms = 3000;
    if (argc > 1) {
        timeout_ms = std::stoi(argv[1]);
    }

    printf("Waiting for SHM data (timeout=%d ms)...\n", timeout_ms);

    void* data = nullptr;
    int size = 0;
    int result = shm_reader->nocopyRead((void**)&data, size);

    if (result < 0 || !data) {
        printf("RESULT: NO DATA (timeout, no frame available)\n");
        return 1;
    }

    auto* pkt = static_cast<Packet*>(data);
    printf("RESULT: DATA FOUND\n");
    printf("  magic=0x%x size=%d width=%d height=%d pixfmt=%d dts=%ld\n",
           pkt->magic, pkt->size, pkt->width, pkt->height, pkt->pixfmt, (long)pkt->dts);

    return 0;
}