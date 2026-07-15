#include "ResizeConvert.hpp"

using namespace std;
using namespace cn;

ResizeConvert::ResizeConvert(uint32_t iw_max, uint32_t ih_max, uint32_t ow_max, uint32_t oh_max, uint32_t batch_max, int devId):BaseOp(devId)
{
    cout<<"ResizeConvert: "<<this<<endl;
    _batch = batch_max;

    _iDes = (cncvImageDescriptor *)malloc(_batch * sizeof(cncvImageDescriptor));
    _oDes = (cncvImageDescriptor *)malloc(_batch * sizeof(cncvImageDescriptor));
    for (int i = 0; i < _batch; i++) 
    {
        _iDes[i].pixel_fmt = CNCV_PIX_FMT_NV12;
        _iDes[i].width = iw_max;
        _iDes[i].height = ih_max;
        _iDes[i].stride[0] = _iDes[i].width*getChannelNum(_iDes[i].pixel_fmt);
        _iDes[i].stride[1] = _iDes[i].width*getChannelNum(_iDes[i].pixel_fmt);
        _iDes[i].depth = CNCV_DEPTH_8U;
        _iDes[i].color_space = CNCV_COLOR_SPACE_BT_601;

        _oDes[i].pixel_fmt = CNCV_PIX_FMT_RGBA;
        _oDes[i].width = ow_max;
        _oDes[i].height = oh_max;
        _oDes[i].stride[0] = _oDes[i].width*getChannelNum(_oDes[i].pixel_fmt);
        _oDes[i].depth = CNCV_DEPTH_8U;
        _oDes[i].color_space = CNCV_COLOR_SPACE_BT_601;
    }

    _iRect = (cncvRect *)malloc(_batch * sizeof(cncvRect));
    _oRect = (cncvRect *)malloc(_batch * sizeof(cncvRect));
    for (int i = 0; i < _batch; i++) 
    {
        _iRect[i].x = 0;
        _iRect[i].y = 0;
        _iRect[i].w = iw_max;
        _iRect[i].h = ih_max;

        _oRect[i].x = 0;
        _oRect[i].y = 0;
        _oRect[i].w = ow_max;
        _oRect[i].h = oh_max;
    }

    _workspaceSize = 0;
    _workspace = nullptr;
    getWorkSpace();

    _y_tmp = (void **)malloc(_batch * sizeof(void *));
    _uv_tmp = (void **)malloc(_batch * sizeof(void *));
    _dst_tmp = (void **)malloc(_batch * sizeof(void *));

    CN_CHECK(cnrtMalloc((void **)&_y, _batch * sizeof(void *)));
    CN_CHECK(cnrtMalloc((void **)&_uv, _batch * sizeof(void *)));
    CN_CHECK(cnrtMalloc((void **)&_dst, _batch * sizeof(void *)));

    CN_CHECK(cnrtMalloc(&_dst_mem, ow_max*oh_max*getChannelNum(CNCV_PIX_FMT_RGBA)*batch_max));
}

ResizeConvert::~ResizeConvert()
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtFree(_dst_mem));
    CN_CHECK(cnrtFree(_y));
    CN_CHECK(cnrtFree(_uv));
    CN_CHECK(cnrtFree(_dst));
    std::free(_y_tmp);
    std::free(_uv_tmp);
    std::free(_dst_tmp);
    CN_CHECK(cnrtFree(_workspace));
    std::free(_iRect);
    std::free(_oRect);
    std::free(_iDes);
    std::free(_oDes);
    cout<<"~ResizeConvert: "<<this<<endl;
}

void* ResizeConvert::run(void* y[], void* uv[], uint32_t batch, uint32_t iw, uint32_t ih, cncvPixelFormat ifmt, cncvRect iRect[],  uint32_t ow, uint32_t oh, cncvPixelFormat ofmt, cncvRect oRect[])
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    _batch = batch;
    for (int i = 0; i < _batch; i++)
    {
        _iDes[i].pixel_fmt = ifmt;
        _iDes[i].width = iw;
        _iDes[i].height = ih;
        _iDes[i].stride[0] = _iDes[i].width*getChannelNum(_iDes[i].pixel_fmt);
        _iDes[i].stride[1] = _iDes[i].width*getChannelNum(_iDes[i].pixel_fmt);

        _oDes[i].pixel_fmt = ofmt;
        _oDes[i].width = ow;
        _oDes[i].height = oh;
        _oDes[i].stride[0] = _oDes[i].width*getChannelNum(_oDes[i].pixel_fmt);

        if(iRect==nullptr)
        {
            _iRect[i].x = 0;
            _iRect[i].y = 0;
            _iRect[i].w = iw;
            _iRect[i].h = ih;
        }
        else
        {
            _iRect[i] = iRect[i];
        }

        if(oRect==nullptr)
        {
            _oRect[i].x = 0;
            _oRect[i].y = 0;
            _oRect[i].w = ow;
            _oRect[i].h = oh;
        }
        else
        {
            _oRect[i] = oRect[i];
        }
        
        _y_tmp[i] = y[i];
        _uv_tmp[i] = uv[i];
        _dst_tmp[i] = (u8_t*)_dst_mem+ow*oh*getChannelNum(ofmt)*i;
    }

    getWorkSpace();

    CN_CHECK(cnrtMemcpy(_y, _y_tmp, _batch * sizeof(void*), CNRT_MEM_TRANS_DIR_HOST2DEV));
    CN_CHECK(cnrtMemcpy(_uv, _uv_tmp, _batch * sizeof(void*), CNRT_MEM_TRANS_DIR_HOST2DEV));
    CN_CHECK(cnrtMemcpy(_dst, _dst_tmp, _batch * sizeof(void *),CNRT_MEM_TRANS_DIR_HOST2DEV));

    CN_CHECK(cnrtMemset(_dst_mem, 0, ow*oh*getChannelNum(ofmt)*_batch));

    if(CN_CHECK(cncvResizeConvert(_handle,
                        _batch,
                        _iDes,
                        _iRect,
                        _y,
                        _uv,
                        _oDes,
                        _oRect,
                        _dst,
                        _workspaceSize,
                        _workspace,
                        CNCV_INTER_BILINEAR)))
    {
    #if CNCV_DEBUG_SHOW
        cout<<"rc_error:handle="<<_handle<<";queue="<<_queue<<";workspaceSize="<<_workspaceSize<<";workspace="<<_workspace<<";batch="<<_batch<<endl;
        cout<<"iw="<<iw<<";ih="<<ih<<";ifmt="<<ifmt<<endl;
        cout<<"ow="<<ow<<";oh="<<oh<<";ofmt="<<ofmt<<endl;
        for(int i=0; i<_batch; i++)
        {
            cout<<"iRect"<<i<<":x="<<_iRect[i].x<<";y="<<_iRect[i].y<<";w="<<_iRect[i].w<<";h="<<_iRect[i].h<<endl;
            cout<<"oRect"<<i<<":x="<<_oRect[i].x<<";y="<<_oRect[i].y<<";w="<<_oRect[i].w<<";h="<<_oRect[i].h<<endl;
        }
        exit(-1);
    #endif
        return nullptr;
    }
    CN_CHECK(cnrtSyncQueue(_queue));

    return _dst_mem;
}

void* ResizeConvert::run(void *y, void *uv, uint32_t iw, uint32_t ih, cncvPixelFormat ifmt, cncvRect iRect, uint32_t ow, uint32_t oh, cncvPixelFormat ofmt, cncvRect oRect)
{
    void *y_batch[1];
    y_batch[0] = y;
    void *uv_batch[1];
    uv_batch[0] = uv;
    cncvRect iRect_batch[1];
    iRect_batch[0]=iRect;
    cncvRect oRect_batch[1];
    oRect_batch[0]=oRect;
    void *result = nullptr;
    CN_CHECK((result=run(y_batch,uv_batch,1,iw,ih,ifmt,iRect_batch,ow,oh,ofmt,oRect_batch))==nullptr);
    return result;
}

void ResizeConvert::getWorkSpace()
{
    size_t workspaceSize = 0;
    CN_CHECK(cncvGetResizeConvertWorkspaceSize(_batch,
                                            _iDes,
                                            _iRect,
                                            _oDes,
                                            _oRect,
                                            &workspaceSize));
    if(workspaceSize>_workspaceSize)
    {
        _workspaceSize = workspaceSize;
        cout<<"work space size = "<<_workspaceSize<<endl;
        if(_workspace!=nullptr)
        {
            CN_CHECK(cnrtFree(_workspace));
        }
        CN_CHECK(cnrtMalloc(&_workspace, _workspaceSize));
    }
}