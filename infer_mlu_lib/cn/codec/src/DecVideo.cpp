#include"DecVideo.hpp"

using namespace std;
using namespace cn;

int DecVideo::event_callback(cncodecCbEventType event, void *context, void *data)
{
    //cout<<"DecVideo call back event = "<<event<<endl;
    DecVideo* thiz = (DecVideo*)context;

    switch (event)
    {
        case CNCODEC_CB_EVENT_SEQUENCE:
        {
            cnvideoDecSequenceInfo* seqInfo = (cnvideoDecSequenceInfo*)data;
            cout<<seqInfo->codec<<";"<<seqInfo->width<<";"<<seqInfo->height<<";"<<seqInfo->minInputBufNum<<";"<<seqInfo->minOutputBufNum<<endl;
            thiz->_info.width = seqInfo->width;
            thiz->_info.height = seqInfo->height;
            thiz->_info.inputBufNum = max(seqInfo->minInputBufNum,thiz->_info.inputBufNum);
            thiz->_info.outputBufNum = max(seqInfo->minOutputBufNum,thiz->_info.outputBufNum);
            CN_CHECK(cnvideoDecStart(thiz->_decoder, &(thiz->_info)));
            break;
        }
            
        case CNCODEC_CB_EVENT_NEW_FRAME:
        {
            cnvideoDecOutput *out = (cnvideoDecOutput*)data;
            thiz->_que->lock();
            if(thiz->_que->full())
            {
                thiz->_que->pop();
                cout<<"lost frame..."<<endl;
            }
            thiz->_que->push(MluMat(thiz->_decoder, &(out->frame)));
            thiz->_que->unlock();
            break;
        }
            
        case CNCODEC_CB_EVENT_EOS:
        {
            break;
        }

        case CNCODEC_CB_EVENT_OUT_OF_MEMORY:
        case CNCODEC_CB_EVENT_ABORT_ERROR:
        default:
        {
            log("error event: "+to_string(event));
            break;
        }
            
    }
    return 0;
}

DecVideo::DecVideo(cncodecType type, cncodecPixelFormat fmt, int devId, int coreId)
{
    cout<<"DecVideo: "<<this<<endl;

    CN_CHECK(cnrtInit(0));
    memset(&_info, 0, sizeof(cnvideoDecCreateInfo));
    _info.width = 1920;
    _info.height = 1080;
    _info.pixelFmt = fmt;
    _info.deviceId = devId;
    _info.instance = coreId;
    _info.codec = type;
    _info.colorSpace = CNCODEC_COLOR_SPACE_INVALID;
    _info.userContext = (void*)this;
    _info.inputBufNum = 2;
    _info.outputBufNum = 8;
    _info.progressive = true;//逐行扫描（progressive）;隔行扫描（interlaced）
    _info.allocType = CNCODEC_BUF_ALLOC_LIB;
    CN_CHECK(cnvideoDecCreate(&_decoder, event_callback, &_info));

    int align = 1;//width default:128; height:2
    CN_CHECK(cnvideoDecSetAttributes(_decoder,CNVIDEO_DEC_ATTR_OUT_BUF_ALIGNMENT,&align));

    _que = new BlockQueue<MluMat>(_info.outputBufNum-2);
}

DecVideo::~DecVideo()
{
    _que->lock();
    while (!_que->empty())
    {
        _que->pop();
    }
    _que->unlock();
    delete _que;
    CN_CHECK(cnvideoDecStop(_decoder));
    CN_CHECK(cnvideoDecDestroy(_decoder));
    cnrtDestroy();
    cout<<"~DecVideo:"<<this<<endl;
}

int DecVideo::send(uint8_t *buf, int len, u64_t pts, u32_t flags)//CNVIDEODEC_FLAG_END_OF_FRAME; CNVIDEODEC_FLAG_TIMESTAMP
{
    cnvideoDecInput input = {0};
    input.streamBuf = buf;
    input.streamLength = len;
    input.flags = flags;
    input.pts = pts;
    return CN_CHECK(cnvideoDecFeedData(_decoder, &input, 2000));
}

MluMat DecVideo::get()
{
    MluMat mat;
    _que->lock();
    if(!_que->empty())
    {
        mat = _que->pop();
    }
    _que->unlock();
    return mat;
}