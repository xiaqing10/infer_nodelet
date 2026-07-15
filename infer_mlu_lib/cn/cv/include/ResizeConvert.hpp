#ifndef RESIZE_CONVERT_HPP
#define RESIZE_CONVERT_HPP

#include "BaseOp.hpp"

namespace cn
{
    class ResizeConvert : public BaseOp
    {
    public:
        ResizeConvert(uint32_t iw_max=1920, uint32_t ih_max=1080, uint32_t ow_max=1920, uint32_t oh_max=1080, uint32_t batch_max=1, int devId=0);
        ~ResizeConvert();
        void* run(void *y[], void *uv[], uint32_t batch, uint32_t iw, uint32_t ih, cncvPixelFormat ifmt, cncvRect iRect[],  uint32_t ow, uint32_t oh, cncvPixelFormat ofmt, cncvRect oRect[]);
        void* run(void *y, void *uv, uint32_t iw, uint32_t ih, cncvPixelFormat ifmt, cncvRect iRect, uint32_t ow, uint32_t oh, cncvPixelFormat ofmt, cncvRect oRect);
    private:
        uint32_t _batch;
        cncvImageDescriptor *_iDes, *_oDes;
        cncvRect *_iRect, *_oRect;
        size_t _workspaceSize;
        void *_workspace;

        void **_y_tmp;
        void **_uv_tmp;
        void **_dst_tmp;

        void **_y;
        void **_uv;
        void **_dst;

        void *_dst_mem;

        void getWorkSpace();

    };
}
#endif