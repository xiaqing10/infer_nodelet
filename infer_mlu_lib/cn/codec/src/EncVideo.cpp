#include "EncVideo.hpp"

using namespace std;
using namespace cn;

int EncVideo::event_callback(cncodecCbEventType event, void *context, void *data)
{
    //cout<<"EncVideo::event_callback event = "<<event<<endl;
    EncVideo *thiz = (EncVideo*)context;

    switch (event)
    {     
        case CNCODEC_CB_EVENT_NEW_FRAME:
        {
            cnvideoEncOutput *out = (cnvideoEncOutput*)data;
            thiz->_que->lock();
            if(thiz->_que->full())
            {
                CN_CHECK(thiz->free(thiz->_que->pop()));
                cout<<"lost enc frame..."<<endl;
            }
            CN_CHECK(thiz->keep(out));
            thiz->_que->push(out);
            thiz->_que->unlock();
            break;
        }
            
        case CNCODEC_CB_EVENT_EOS:
        {
            thiz->_que->lock();
            if(thiz->_que->full())
            {
                CN_CHECK(thiz->free(thiz->_que->pop()));
                cout<<"lost enc frame..."<<endl;
            }
            thiz->_que->push(nullptr);
            thiz->_que->unlock();
            CN_CHECK(sem_post(&thiz->_finish));
            break;
        }

        case CNCODEC_CB_EVENT_OUT_OF_MEMORY:
        case CNCODEC_CB_EVENT_ABORT_ERROR:
        default:
        {
            log("error enc event: "+to_string(event));
            break;
        }
            
    }

    return 0;

}

EncVideo::EncVideo(int width, int height, cncodecPixelFormat fmt, cncodecType type, int devId, int coreId)
{
    cout<<"EncVideo: "<<this<<endl;
    CN_CHECK(cnrtInit(0));
    memset(&_info, 0, sizeof(cnvideoEncCreateInfo));
    _info.deviceId = devId;
    _info.instance = (cnvideoEncInstance)coreId;
    _info.pixelFmt = fmt;
    _info.codec = type;
    _info.colorSpace = CNCODEC_COLOR_SPACE_INVALID;
    _info.width = width;
    _info.height = height;
    _info.inputBufNum = 2;
    _info.outputBufNum = 8;
    _info.allocType = CNCODEC_BUF_ALLOC_LIB;
    //_info.allocType = CNCODEC_BUF_ALLOC_LIB_USER_SPECIFY_BUF_ALIGNMENT;
    //_info.extCfg.bufCfg.frameBufAlignment = 1;//default:1
    _info.userContext = (void*)this;
    
    // if(type==CNCODEC_H264)
    // {
    //     //_info.uCfg.h264.insertSpsPpsWhenIDR = 1;
    // }
    CN_CHECK(cnvideoEncCreate(&_encoder, event_callback, &_info));

    CN_CHECK(cnrtGetDeviceHandle(&_dev, devId));

    _que = new BlockQueue<cnvideoEncOutput*>(5);
    CN_CHECK(sem_init(&_finish, 0, 0));

}

EncVideo::~EncVideo()
{
    CN_CHECK(sem_destroy(&_finish));
    _que->lock();
    while (!_que->empty())
    {
        CN_CHECK(free(_que->pop()));
    }
    _que->unlock();
    delete _que;
    CN_CHECK(cnvideoEncDestroy(_encoder));
    cnrtDestroy();
    cout<<"~EncVideo:"<<this<<endl;
}

int EncVideo::send(void *data0, void *data1, Mode mode, u64_t pts)
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    int data0_size = _info.width * _info.height * getChannelNum(_info.pixelFmt);
    int data1_size = _info.width * _info.height * getChannelNum(_info.pixelFmt) / 2;

    cnvideoEncInput input;
    memset(&input, 0, sizeof(input));
    CN_CHECK(cnvideoEncWaitAvailInputBuf(_encoder, &input.frame, 5000));

    if(mode==Mode::MLU)
    {
        if(data0!=nullptr)
        {
            CN_CHECK(cnrtMemcpy((void*)input.frame.plane[0].addr, data0, data0_size, CNRT_MEM_TRANS_DIR_DEV2DEV));
        }
        if(data1!=nullptr)
        {
            CN_CHECK(cnrtMemcpy((void*)input.frame.plane[1].addr, data1, data1_size, CNRT_MEM_TRANS_DIR_DEV2DEV));
        }
    }
    else if(mode==Mode::CPU)
    {
        if(data0!=nullptr)
        {
            CN_CHECK(cnrtMemcpy((void*)input.frame.plane[0].addr, data0, data0_size, CNRT_MEM_TRANS_DIR_HOST2DEV));
        }
        if(data1!=nullptr)
        {
            CN_CHECK(cnrtMemcpy((void*)input.frame.plane[1].addr, data1, data1_size, CNRT_MEM_TRANS_DIR_HOST2DEV));
        }
    }
    input.pts = pts;
    if(data0==nullptr && data1==nullptr)
    {
        input.flags |= CNVIDEOENC_FLAG_EOS;
        CN_CHECK(cnvideoEncFeedFrame(_encoder, &input, 1000));
        return CN_CHECK(sem_wait(&_finish));
    }
    else
    {
        return CN_CHECK(cnvideoEncFeedFrame(_encoder, &input, 1000));
    }
}

cnvideoEncOutput* EncVideo::get()
{
    cnvideoEncOutput* out = nullptr;
    _que->lock();
    //if(!_que->empty())
    {
        out = _que->pop();
    }
    _que->unlock();
    return out;
}

int EncVideo::keep(cnvideoEncOutput *out)
{
    return CN_CHECK(cnvideoEncAddReference(_encoder, &out->streamBuffer));
}

int EncVideo::free(cnvideoEncOutput *out)
{
    return CN_CHECK(cnvideoEncReleaseReference(_encoder, &out->streamBuffer));
}

uint32_t EncVideo::getChannelNum(cncodecPixelFormat fmt)
{
    uint32_t channel_num = 0;
    switch (fmt)
    {
        case CNCODEC_PIX_FMT_NV12:
        case CNCODEC_PIX_FMT_NV21:
            channel_num = 1;
            break;

        case CNCODEC_PIX_FMT_ARGB:
        case CNCODEC_PIX_FMT_ABGR:
        case CNCODEC_PIX_FMT_BGRA:
        case CNCODEC_PIX_FMT_RGBA:
            channel_num = 4;
            break;

        default:
            channel_num = 0;
            break;
    }
    return channel_num;
}