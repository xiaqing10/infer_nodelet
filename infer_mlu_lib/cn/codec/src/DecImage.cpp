#include"DecImage.hpp"

using namespace std;
using namespace cn;

#define ALIGN(a, size)   ((a+size-1) & (~ (size-1)))

DecImage::DecImage(cncodecPixelFormat fmt, int iw_max, int ih_max, int devId, int coreId)
{
    cout<<"DecImage: "<<this<<endl;

    CN_CHECK(cnrtInit(0));
    CN_CHECK(cnrtGetDeviceHandle(&_dev, devId));
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    memset(&_info, 0, sizeof(cnjpegDecCreateInfo));
    _info.pixelFmt = fmt;
    _info.width = iw_max;
    _info.height = ih_max;
    _info.deviceId = devId;
    _info.instance = coreId;
    _info.colorSpace = CNCODEC_COLOR_SPACE_INVALID;
    _info.inputBufNum = 1;
    _info.outputBufNum = 1;
    _info.allocType = CNCODEC_BUF_ALLOC_LIB;
    CN_CHECK(cnjpegDecCreate(&_decoder, CNJPEGDEC_RUN_MODE_SYNC, NULL, &_info));

    CN_CHECK(cnrtMallocFrameBuffer((void **)(&(_frame.plane[0].addr)), ALIGN(iw_max, 64)*ALIGN(ih_max, 16)));
    CN_CHECK(cnrtMallocFrameBuffer((void **)(&(_frame.plane[1].addr)), ALIGN(iw_max, 64)*ALIGN(ih_max, 16)/2));
}

DecImage::~DecImage()
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtFree((void*)_frame.plane[0].addr));
    CN_CHECK(cnrtFree((void*)_frame.plane[1].addr));
    CN_CHECK(cnjpegDecDestroy(_decoder));
    cnrtDestroy();
    cout<<"~DecImage:"<<this<<endl;
}

cncodecFrame* DecImage::run(uint8_t *buf, int len)
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    cnjpegDecImageInfo imgInfo;
    CN_CHECK(cnjpegDecGetImageInfo(_decoder, &imgInfo, buf, len));
    if(imgInfo.width>_info.width||imgInfo.height>_info.height)
    {
        cout<<"iw_max="<<_info.width<<";ih_max="<<_info.height<<";width="<<imgInfo.width<<";height="<<imgInfo.height<<endl;
        exit(1);
    }

    cnjpegDecInput input;
    input.streamBuffer = buf;
    input.streamLength = len;

    _frame.width = imgInfo.width;
    _frame.height = imgInfo.height;
    _frame.pixelFmt = _info.pixelFmt;
    _frame.colorSpace = _info.colorSpace;
    _frame.deviceId = _info.deviceId;
    _frame.planeNum = 2;
    _frame.stride[0] = ALIGN(imgInfo.width, 64);
    _frame.stride[1] = ALIGN(imgInfo.width, 64);
    _frame.plane[0].size = _frame.stride[0]*ALIGN(imgInfo.height, 16);
    _frame.plane[1].size = _frame.stride[1]*ALIGN(imgInfo.height, 16)/2;

    cnjpegDecOutput output;
    output.frame = _frame;
    if(CN_CHECK(cnjpegDecSyncDecode(_decoder, &output, &input, 5000))==0)
    {
        return &_frame;
    }
    else
    {
        return nullptr;
    }
}