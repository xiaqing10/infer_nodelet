#ifndef DECVIDEO_HPP
#define DECVIDEO_HPP

#include <iostream>
#include <semaphore.h>
#include <string.h>
#include <mutex>

#include "BlockQueue.hpp"
#include "check.hpp"
#include "MluMat.hpp"
#include "cn_video_dec.h"
#include "cnrt.h"

namespace cn
{
    class DecVideo
    {
    public:
        cnvideoDecoder _decoder;
        cnvideoDecCreateInfo _info;
        BlockQueue<MluMat> *_que;
        DecVideo(cncodecType type = CNCODEC_H264, cncodecPixelFormat fmt=CNCODEC_PIX_FMT_NV12, int devId = 0, int coreId = CNVIDEODEC_INSTANCE_AUTO);
        ~DecVideo();
        int send(uint8_t *buf, int len, u64_t pts=0, u32_t flags=0);
        MluMat get();
        static int event_callback(cncodecCbEventType event, void *context, void *data);
    };
}

#endif