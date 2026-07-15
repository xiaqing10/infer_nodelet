#ifndef CONVERT_TO_HPP
#define CONVERT_TO_HPP

#include "BaseOp.hpp"

namespace cn
{
    class ConvertTo : public BaseOp
    {
    public:
        ConvertTo(uint32_t w_max=1920, uint32_t h_max=1080, uint32_t batch_max=4, int devId=0);
        ~ConvertTo();
        void* run(void *src_mem, uint32_t batch, uint32_t w, uint32_t h, cncvPixelFormat fmt, cncvDepth_t depth_in, cncvDepth_t depth_out, float alpha=1.0, float beta=0.0);
    private:
        void **_src_tmp;
        void **_dst_tmp;

        void **_src;
        void **_dst;

        void *_dst_mem;
    };
}
#endif