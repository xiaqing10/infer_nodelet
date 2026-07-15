#ifndef DEC_IMAGE_HPP
#define DEC_IMAGE_HPP

#include <iostream>
#include <string.h>

#include "check.hpp"
#include "cn_jpeg_dec.h"
#include "cnrt.h"

namespace cn
{
    class DecImage
    {
    private:
        cnjpegDecoder _decoder;
        cnjpegDecCreateInfo _info;
        cnrtDev_t _dev;
        cncodecFrame _frame;

    public:
        DecImage(cncodecPixelFormat fmt=CNCODEC_PIX_FMT_NV12, int iw_max=1920, int ih_max=1080, int devId=0, int coreId=CNJPEGDEC_INSTANCE_AUTO);
        ~DecImage();
        cncodecFrame* run(uint8_t *buf, int len);
    };
}

#endif