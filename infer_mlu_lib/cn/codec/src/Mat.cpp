#include"Mat.hpp"

using namespace std;
using namespace cn;

Mat::Mat(cnvideoDecoder decoder, cncodecFrame *frame)
{
    //std::cout<<"Mat0: "<<this<<std::endl;
    _decoder = decoder;
    _frame = frame;
    CN_CHECK(cnvideoDecAddReference(_decoder, _frame));
    _dev = _frame->deviceId;
    _width = _frame->width;
    _height = _frame->height;
    _stride = _frame->stride[0];
    _y = (void*)(_frame->plane[0].addr);
    _uv = (void*)(_frame->plane[1].addr);
}

Mat::Mat(int dev, int width, int height, int stride)
{
    //std::cout<<"Mat1: "<<this<<std::endl;
    _decoder = nullptr;
    _frame = nullptr;
    _dev = dev;
    _width = width;
    _height = height;
    _stride = stride;
    cnrtDev_t device;
    CN_CHECK(cnrtGetDeviceHandle(&device, _dev));
    CN_CHECK(cnrtSetCurrentDevice(device));
    CN_CHECK(cnrtMalloc(&(_y), _stride*_height));
    CN_CHECK(cnrtMalloc(&(_uv), _stride*_height/2));
}

Mat::~Mat()
{
    if(_decoder!=nullptr && _frame!=nullptr)
    {
        CN_CHECK(cnvideoDecReleaseReference(_decoder, _frame));
        //std::cout<<"~Mat0: "<<this<<std::endl;
    }
    else if(_decoder==nullptr && _frame==nullptr && _y!=nullptr && _uv!=nullptr)
    {
        cnrtDev_t device;
        CN_CHECK(cnrtGetDeviceHandle(&device, _dev));
        CN_CHECK(cnrtSetCurrentDevice(device));
        CN_CHECK(cnrtFree(_y));
        CN_CHECK(cnrtFree(_uv));
        //std::cout<<"~Mat1: "<<this<<std::endl;
    }
}

int Mat::dev()
{
    return _dev;
}

int Mat::width()
{
    return _width;
}

int Mat::height()
{
    return _height;
}

int Mat::stride()
{
    return _stride;
}

void* Mat::y()
{
    return _y;
}

void* Mat::uv()
{
    return _uv;
}

int Mat::set(void *y, void *uv, int dev)
{
    int ret = -1;
    if(_decoder==nullptr && _frame==nullptr && _y!=nullptr && _uv!=nullptr)
    {
        if(dev<0)
        {
            ret += CN_CHECK(cnrtMemcpy(_y, y, _stride*_height, CNRT_MEM_TRANS_DIR_HOST2DEV));
            ret += CN_CHECK(cnrtMemcpy(_uv, uv, _stride*_height/2, CNRT_MEM_TRANS_DIR_HOST2DEV));
        }
        else if(dev==_dev)
        {
            ret += CN_CHECK(cnrtMemcpy(_y, y, _stride*_height, CNRT_MEM_TRANS_DIR_DEV2DEV));
            ret += CN_CHECK(cnrtMemcpy(_uv, uv, _stride*_height/2, CNRT_MEM_TRANS_DIR_DEV2DEV));
        }
        else if(dev!=_dev)
        {
            ret += CN_CHECK(cnrtMemcpyPeer(_y, _dev, y, dev, _stride*_height));
            ret += CN_CHECK(cnrtMemcpyPeer(_uv, _dev, uv, dev, _stride*_height/2));
        }
    }
    return ret;
}

void* Mat::data(void *data_dev, int width, int height, int channel)
{
    int size = width*height*channel;
    void *data_host = malloc(size);
    CN_CHECK(cnrtMemcpy(data_host, data_dev, size, CNRT_MEM_TRANS_DIR_DEV2HOST));
    return data_host;
}