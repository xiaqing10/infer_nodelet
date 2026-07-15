#include "ConvertTo.hpp"

using namespace std;
using namespace cn;

ConvertTo::ConvertTo(uint32_t w_max, uint32_t h_max, uint32_t batch_max, int devId):BaseOp(devId)
{
    cout<<"ConvertTo: "<<this<<endl;
    
    _src_tmp = (void **)malloc(batch_max * sizeof(void *));
    _dst_tmp = (void **)malloc(batch_max * sizeof(void *));

    CN_CHECK(cnrtMalloc((void **)&_src, batch_max * sizeof(void *)));
    CN_CHECK(cnrtMalloc((void **)&_dst, batch_max * sizeof(void *)));

    CN_CHECK(cnrtMalloc(&_dst_mem, w_max*h_max*getChannelNum(CNCV_PIX_FMT_RGBA)*batch_max*getDepthNum(CNCV_DEPTH_32F)));
}

ConvertTo::~ConvertTo()
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtFree(_dst_mem));
    CN_CHECK(cnrtFree(_src));
    CN_CHECK(cnrtFree(_dst));
    std::free(_src_tmp);
    std::free(_dst_tmp);
    cout<<"~ConvertTo: "<<this<<endl;
}

void* ConvertTo::run(void *src_mem, uint32_t batch, uint32_t w, uint32_t h, cncvPixelFormat fmt, cncvDepth_t depth_in, cncvDepth_t depth_out, float alpha, float beta)
{
    CN_CHECK(cnrtSetCurrentDevice(_dev));

    cncvImageDescriptor src_desc;
    src_desc.pixel_fmt = fmt;
    src_desc.width = w;
    src_desc.height = h;
    src_desc.stride[0] = w*getChannelNum(fmt)*getDepthNum(depth_in);
    src_desc.depth = depth_in;

    cncvImageDescriptor dst_desc;
    dst_desc.pixel_fmt = fmt;
    dst_desc.width = w;
    dst_desc.height = h;
    dst_desc.stride[0] = w*getChannelNum(fmt)*getDepthNum(depth_out);
    dst_desc.depth = depth_out;

    for (int i = 0; i < batch; i++) 
    {
        _src_tmp[i] = (uint8_t*)src_mem+w*h*getChannelNum(fmt)*getDepthNum(depth_in)*i;
        _dst_tmp[i] = (uint8_t*)_dst_mem+w*h*getChannelNum(fmt)*getDepthNum(depth_out)*i;
    }
    
    CN_CHECK(cnrtMemcpy(_src, _src_tmp, batch * sizeof(void*), CNRT_MEM_TRANS_DIR_HOST2DEV));
    CN_CHECK(cnrtMemcpy(_dst, _dst_tmp, batch * sizeof(void *),CNRT_MEM_TRANS_DIR_HOST2DEV));

    CN_CHECK(cnrtMemset(_dst_mem, 0, w*h*getChannelNum(fmt)*getDepthNum(depth_out)*batch));

    if(CN_CHECK(cncvConvertTo(_handle,
                        batch,
                        src_desc,
                        _src,
                        dst_desc,
                        _dst,
                        alpha,
                        beta)))
    {
    #if CNCV_DEBUG_SHOW
        cout<<"ct_error:handle="<<_handle<<";queue="<<_queue<<";batch="<<batch<<";alpha="<<alpha<<";beta="<<beta<<endl;
        cout<<"w="<<w<<";h="<<h<<";fmt="<<fmt<<endl;
        cout<<"depth_in="<<depth_in<<";depth_out="<<depth_out<<endl;
        exit(-1);
    #endif
        return nullptr;
    }
    CN_CHECK(cnrtSyncQueue(_queue));

    return _dst_mem;

}