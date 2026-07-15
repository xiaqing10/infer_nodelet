#include "BaseOp.hpp"

using namespace std;
using namespace cn;

BaseOp::BaseOp(int devId)
{
    cout<<"BaseOp: "<<this<<endl;
    CN_CHECK(cnrtInit(0));
    CN_CHECK(cnrtGetDeviceHandle(&_dev, devId));
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtCreateQueue(&_queue));
    CN_CHECK(cncvCreate(&_handle));
    CN_CHECK(cncvSetQueue(_handle, _queue));
}

BaseOp::~BaseOp()
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cncvDestroy(_handle));
    CN_CHECK(cnrtDestroyQueue(_queue));
    cnrtDestroy();
    cout<<"~BaseOp: "<<this<<endl;
}

uint32_t BaseOp::getChannelNum(cncvPixelFormat fmt)
{
    uint32_t channel_num = 0;
    switch (fmt)
    {
        case CNCV_PIX_FMT_NV12:
        case CNCV_PIX_FMT_NV21:
            channel_num = 1;
            break;

        case CNCV_PIX_FMT_RGB:
        case CNCV_PIX_FMT_BGR:
            channel_num = 3;
            break;

        case CNCV_PIX_FMT_ARGB:
        case CNCV_PIX_FMT_ABGR:
        case CNCV_PIX_FMT_BGRA:
        case CNCV_PIX_FMT_RGBA:
            channel_num = 4;
            break;

        default:
            channel_num = 0;
            break;
    }
    return channel_num;
}

uint32_t BaseOp::getDepthNum(cncvDepth_t depth)
{
    uint32_t depth_num = 0;
    switch (depth)
    {
        case CNCV_DEPTH_8U:
            depth_num = 1;
            break;

        case CNCV_DEPTH_16F:
            depth_num = 2;
            break;

        case CNCV_DEPTH_32F:
            depth_num = 4;
            break;

        default:
            depth_num = 0;
            break;
    }
    return depth_num;
}