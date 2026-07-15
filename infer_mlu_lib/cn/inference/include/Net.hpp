#ifndef NET_HPP
#define NET_HPP

#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <vector>
#include <queue>
#include <string>
#include <fstream>
#include <thread> 
#include <sys/time.h>
#include <mutex>
#include <typeinfo>
#include <unistd.h>
#include <condition_variable>

#include "check.hpp"
#include "cnrt.h"

namespace cn
{
    class Net
    {
    private:
        cnrtModel_t _mdl;
       // int devi;
//	cnrtFunction_t func;
	cnrtDev_t _dev;
        cnrtQueue_t _queue;
        cnrtRuntimeContext_t _ctx;
        cnrtInvokeParam_t _invoke_param;
        unsigned int _affinity;
        void **_params;

        int _iNum;
        int64_t* _iDataSize;
        cnrtDataType_t* _iDataType;
        std::vector<int> _iDimNum;
        std::vector<int*> _iDimVal;
        std::vector<void*> _iMlu;
        std::vector<void*> _iCpu;

        int _oNum;
        int64_t* _oDataSize;
        cnrtDataType_t* _oDataType;
        std::vector<int> _oDimNum;
        std::vector<int*> _oDimVal;
        std::vector<void*> _oMlu;
        std::vector<void*> _oCpu;

    public:
        void  init(std::string path, int devId=0, int cls=-1);
        void Net_del();
        int input_dims(int i, int idx=0);
        int output_dims(int i, int idx=0);
        int input_set(void *data, int idx=0);
        int input_from(void *data, int idx=0, bool bNCHW2NHWC=false);
        int output_to(void *data, int idx=0, bool bNHWC2NCHW=false);
        int infer();
    };
}

#endif
