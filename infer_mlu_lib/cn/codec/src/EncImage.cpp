#include "EncImage.hpp"

using namespace std;
using namespace cn;

#define ALIGN(a, size)   ((a+size-1) & (~ (size-1)))

EncImage::EncImage(cncodecPixelFormat fmt, int width, int height, int devId, int coreId)
{
    cout<<"EncImage: "<<this<<endl;
    CN_CHECK(cnrtInit(0));
    CN_CHECK(cnrtGetDeviceHandle(&_dev, devId));
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    memset(&_info, 0, sizeof(cnjpegEncCreateInfo));
    _info.deviceId = devId;
    _info.instance = (cnjpegEncInstance)coreId;
    _info.pixelFmt = fmt;
    _info.colorSpace = CNCODEC_COLOR_SPACE_INVALID;
    _info.width = width;
    _info.height = height;
    _info.inputBufNum = 1;
    _info.outputBufNum = 1;
    _info.allocType = CNCODEC_BUF_ALLOC_LIB;
    _info.userContext = (void*)this;
    CN_CHECK(cnjpegEncCreate(&_encoder, CNJPEGENC_RUN_MODE_SYNC, NULL, &_info));

    _input.frame.width = width;
    _input.frame.height = height;
    _input.frame.colorSpace = CNCODEC_COLOR_SPACE_INVALID;
    _input.frame.deviceId = devId;
    _input.frame.pixelFmt = fmt;
    _input.frame.planeNum = 2;
    _input.frame.stride[0] = ALIGN(width,64);
    _input.frame.stride[1] = ALIGN(width,64);
    _input.frame.plane[0].size = _input.frame.stride[0]*ALIGN(height,16);
    _input.frame.plane[1].size = _input.frame.stride[1]*ALIGN(height,16)/2;
    CN_CHECK(cnrtMalloc((void **)(&(_input.frame.plane[0].addr)), _input.frame.plane[0].size));
    CN_CHECK(cnrtMalloc((void **)(&(_input.frame.plane[1].addr)), _input.frame.plane[1].size));

    _output.streamBuffer.size = 10000000;
    CN_CHECK(cnrtMalloc((void **)(&_output.streamBuffer.addr), _output.streamBuffer.size));
}

EncImage::~EncImage()
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtFree((void*)_output.streamBuffer.addr));
    CN_CHECK(cnrtFree((void*)_input.frame.plane[0].addr));
    CN_CHECK(cnrtFree((void*)_input.frame.plane[1].addr));
    CN_CHECK(cnjpegEncDestroy(_encoder));
    cnrtDestroy();
    cout<<"~EncImage:"<<this<<endl;
}

cnjpegEncOutput* EncImage::run(void *y, void *uv, Mode mode, u64_t pts)
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    int y_size = _info.width * _info.height;
    int uv_size = _info.width * _info.height / 2;

    if(mode==Mode::MLU)
    {
        CN_CHECK(cnrtMemcpy((void*)_input.frame.plane[0].addr, y, y_size, CNRT_MEM_TRANS_DIR_DEV2DEV));
        CN_CHECK(cnrtMemcpy((void*)_input.frame.plane[1].addr, uv, uv_size, CNRT_MEM_TRANS_DIR_DEV2DEV));
    }
    else if(mode==Mode::CPU)
    {
        CN_CHECK(cnrtMemcpy((void*)_input.frame.plane[0].addr, y, y_size, CNRT_MEM_TRANS_DIR_HOST2DEV));
        CN_CHECK(cnrtMemcpy((void*)_input.frame.plane[1].addr, uv, uv_size, CNRT_MEM_TRANS_DIR_HOST2DEV));
    }
    _input.pts = pts;
    cnjpegEncParameters enFrameParam = {100, 0, 0};
    if(CN_CHECK(cnjpegEncSyncEncode(_encoder, &_output, &_input, &enFrameParam, 5000)))
    {
        return nullptr;
    }
    return &_output;
}