#include "MluMat.hpp"

using namespace std;
using namespace cn;

MluMat::MluMat()
{
    _mat = nullptr;
    _valid = false;
}

MluMat::MluMat(cnvideoDecoder decoder, cncodecFrame *frame)
{
    _mat = std::make_shared<Mat>(decoder, frame);
    _valid = true;
}

MluMat::MluMat(int dev, int width, int height, int stride)
{
    _mat = std::make_shared<Mat>(dev, width, height, stride);
    _valid = true;
}

int MluMat::dev()
{
    return _mat->dev();
}

int MluMat::width()
{
    return _mat->width();
}

int MluMat::height()
{
    return _mat->height();
}

int MluMat::stride()
{
    return _mat->stride();
}

void* MluMat::y()
{
    return _mat->y();
}

void* MluMat::uv()
{
    return _mat->uv();
}

int MluMat::set(void *y, void *uv, int dev)
{
    return _mat->set(y,uv,dev);
}

void* MluMat::data(void *data_dev, int width, int height, int channel)
{
    return Mat::data(data_dev, width, height, channel);
}

bool MluMat::valid()
{
    return _valid;
}