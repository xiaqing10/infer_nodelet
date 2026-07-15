#ifndef MAT_HPP
#define MAT_HPP

#include <iostream>
#include <string.h>
#include <vector>

#include "check.hpp"
#include "cn_video_dec.h"
#include "cnrt.h"

namespace cn
{
    class Mat
    {
    private:
        cnvideoDecoder _decoder;
        cncodecFrame *_frame;
        int _dev;
        int _width;
        int _height;
        int _stride;
        void *_y;
        void *_uv;
    public:
        Mat(cnvideoDecoder decoder, cncodecFrame *frame);
        Mat(int dev, int width, int height, int stride);
        ~Mat();
        int dev();
        int width();
        int height();
        int stride();
        void* y();
        void* uv();
        int set(void *y, void *uv, int dev);
        static void* data(void *data_dev, int width, int height, int channel);
    };
}
#endif