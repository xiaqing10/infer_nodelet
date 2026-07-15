#ifndef MEAN_STD_HPP
#define MEAN_STD_HPP

#include "BaseOp.hpp"

namespace cn
{
    class MeanStd : public BaseOp
    {
    public:
        MeanStd(uint32_t w_max=1920, uint32_t h_max=1080, uint32_t batch_max=1, int devId=0);
        ~MeanStd();
        void* run(void *src_mem, uint32_t batch, uint32_t w, uint32_t h, cncvPixelFormat fmt, cncvDepth_t depth_in, cncvDepth_t depth_out, float *mean, float *std);
    private:

        size_t _workspaceSize;
        void *_workspace;

        void **_src_tmp;
        void **_dst_tmp;

        void **_src;
        void **_dst;

        void *_dst_mem;
    };
}
#endif