#ifndef ENC_VIDEO_HPP
#define ENC_VIDEO_HPP

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>

#include "BlockQueue.hpp"
#include "check.hpp"
#include "cn_video_enc.h"
#include "cnrt.h"

namespace cn
{
    class EncVideo
    {
    private:
        uint32_t getChannelNum(cncodecPixelFormat fmt);

    public:
        enum class Mode {
            CPU,
            MLU
        }; 
        cnvideoEncoder _encoder;
        cnvideoEncCreateInfo _info;
        cnrtDev_t _dev;
        BlockQueue<cnvideoEncOutput*> *_que;
        sem_t _finish;
        EncVideo(int width=1920, int height=1080, cncodecPixelFormat fmt = CNCODEC_PIX_FMT_NV12, cncodecType type = CNCODEC_H264, int devId=0, int coreId=CNVIDEOENC_INSTANCE_AUTO);
        ~EncVideo();
        int send(void *data0, void *data1, Mode mode=Mode::CPU, u64_t pts=0);
        cnvideoEncOutput* get();
        int keep(cnvideoEncOutput* out);
        int free(cnvideoEncOutput* out);
        static int event_callback(cncodecCbEventType event, void *context, void *data);
    };
}
#endif