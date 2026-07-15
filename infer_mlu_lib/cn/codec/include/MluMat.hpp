#ifndef MLUMAT_HPP
#define MLUMAT_HPP

#include "Mat.hpp"
#include <memory>

namespace cn
{
    class MluMat
    {
    private:
        std::shared_ptr<Mat> _mat;
        bool _valid;
    public:
        MluMat();
        MluMat(cnvideoDecoder decoder, cncodecFrame *frame);
        MluMat(int dev, int width, int height, int stride);
        int dev();
        int width();
        int height();
        int stride();
        void* y();
        void* uv();
        int set(void *y, void *uv, int dev);
        static void* data(void *data_dev, int width, int height, int channel);
        bool valid();
    };
}

#endif