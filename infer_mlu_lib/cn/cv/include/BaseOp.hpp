#ifndef BASE_OP_HPP
#define BASE_OP_HPP

#include <vector>
#include <iostream>
#include <string.h>
#include <memory>

#define CNCV_DEBUG_SHOW (1)
#if CNCV_DEBUG_SHOW
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "check.hpp"
#include "cncv.h"

namespace cn
{
    class BaseOp
    {
    public:
        BaseOp(int devId=0);
        ~BaseOp();
    protected:
        cnrtDev_t _dev;
        cnrtQueue_t _queue = nullptr;
        cncvHandle_t _handle = nullptr;
        uint32_t getChannelNum(cncvPixelFormat fmt);
        uint32_t getDepthNum(cncvDepth_t depth);
    };
}
#endif