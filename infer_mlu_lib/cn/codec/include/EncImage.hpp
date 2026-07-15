#ifndef ENC_IMAGE_HPP
#define ENC_IMAGE_HPP

#include <iostream>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>

#include "check.hpp"
#include "cn_jpeg_enc.h"
#include "cnrt.h"

namespace cn
{
    class EncImage
    {

    public:
        enum class Mode {
            CPU,
            MLU
        }; 
        cnjpegEncoder _encoder;
        cnjpegEncCreateInfo _info;
        cnrtDev_t _dev;
        cnjpegEncInput _input;
        cnjpegEncOutput _output;

        EncImage(cncodecPixelFormat fmt=CNCODEC_PIX_FMT_NV12,int width=1920, int height=1080, int devId=0, int coreId=CNJPEGENC_INSTANCE_AUTO);
        ~EncImage();
        cnjpegEncOutput* run(void *y, void *uv, Mode mode=Mode::MLU, u64_t pts=0);
    };
}

#endif