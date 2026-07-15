#include "Net.hpp"

using namespace std;
using namespace cn;

void   Net::init(std::string path, int devId, int cls)
{
    //cout<<"Net: "<<this<<"; "<<path<<endl;
    CN_CHECK(cnrtInit(0));

    CN_CHECK(cnrtLoadModel(&_mdl, path.c_str()));

    cnrtFunction_t func;
    CN_CHECK(cnrtCreateFunction(&func));
    CN_CHECK(cnrtExtractFunction(&func, _mdl, "subnet0"));

    CN_CHECK(cnrtGetDeviceHandle(&_dev, devId));
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    if(cls>=0)
    {
        CN_CHECK(cnrtSetCurrentChannel((cnrtChannelType_t)cls));
        _affinity = 1<<cls;
        _invoke_param.invoke_param_type = CNRT_INVOKE_PARAM_TYPE_0;
        _invoke_param.cluster_affinity.affinity = &_affinity;
    }
    else
    {
        _affinity = 0;
    }

    CN_CHECK(cnrtGetInputDataSize(&_iDataSize, &_iNum, func));
    CN_CHECK(cnrtGetInputDataType(&_iDataType, &_iNum, func));
    _iMlu.resize(_iNum);
    _iCpu.resize(_iNum);
    for(int i=0;i<_iNum;i++)
    {
        int *dimVal;
        int dimNum;
        CN_CHECK(cnrtGetInputDataShape(&dimVal,&dimNum,i,func));
        _iDimNum.push_back(dimNum);
        _iDimVal.push_back(dimVal);

        CN_CHECK(cnrtMalloc(&(_iMlu[i]), _iDataSize[i]));
        _iCpu[i] = malloc(_iDataSize[i]);
    }

    CN_CHECK(cnrtGetOutputDataSize(&_oDataSize, &_oNum, func));
    CN_CHECK(cnrtGetOutputDataType(&_oDataType, &_oNum, func));
    _oMlu.resize(_oNum);
    _oCpu.resize(_oNum);
    for(int i=0;i<_oNum;i++)
    {
        int *dimVal;
        int dimNum;
        CN_CHECK(cnrtGetOutputDataShape(&dimVal,&dimNum,i,func));
        _oDimNum.push_back(dimNum);
        _oDimVal.push_back(dimVal);

        CN_CHECK(cnrtMalloc(&(_oMlu[i]), _oDataSize[i]));
        _oCpu[i] = malloc(_oDataSize[i]);
    }

    int core_num = 0;
    CN_CHECK(cnrtQueryModelParallelism(_mdl, &core_num));
    cout<<"core_num="<<core_num<<endl;

    cout<<"iNum="<<_iNum<<endl;
    for(int i=0;i<_iNum;i++)
    {
        cout<<"iDataSize="<<_iDataSize[i]<<";iDataType="<<_iDataType[i]<<";iDataCnt="<<_iDataSize[i]/cnrtDataTypeSize(_iDataType[i])<<endl;
        for(int j=0;j<_iDimNum[i];j++)
        {
            cout<<_iDimVal[i][j]<<";";
        }
        cout<<endl;
    }
    cout<<"oNum="<<_oNum<<endl;
    for(int i=0;i<_oNum;i++)
    {
        cout<<"oDataSize="<<_oDataSize[i]<<";oDataType="<<_oDataType[i]<<";oDataCnt="<<_oDataSize[i]/cnrtDataTypeSize(_oDataType[i])<<endl;
        for(int j=0;j<_oDimNum[i];j++)
        {
            cout<<_oDimVal[i][j]<<";";
        }
        cout<<endl;
    }

    CN_CHECK(cnrtSetDeviceFlag(CNRT_QUEUE_SYNC_BLOCK));
    CN_CHECK(cnrtCreateQueue(&_queue));

    CN_CHECK(cnrtCreateRuntimeContext(&_ctx, func, nullptr));
    CN_CHECK(cnrtSetRuntimeContextDeviceId(_ctx, devId));
    CN_CHECK(cnrtInitRuntimeContext(_ctx, nullptr));

    _params = (void**)malloc((_iNum+_oNum)*sizeof(void*));
    for (int i = 0; i < _iNum; i++)
    {
        _params[i] = _iMlu[i];
    }
    for (int i = 0; i < _oNum; i++)
    {
        _params[_iNum + i] = _oMlu[i];
    }

}

void Net::Net_del()
{
    cout<<"~Net: "<<this<<endl;
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    CN_CHECK(cnrtDestroyQueue(_queue));
    CN_CHECK(cnrtDestroyRuntimeContext(_ctx));
    free(_iDataSize);
    free(_iDataType);
    for(int i=0;i<_iNum;i++)
    {
        free(_iDimVal[i]);
        CN_CHECK(cnrtFree(_iMlu[i]));
        free(_iCpu[i]);
    }
    free(_oDataSize);
    free(_oDataType);
    for(int i=0;i<_oNum;i++)
    {
        free(_oDimVal[i]);
        CN_CHECK(cnrtFree(_oMlu[i]));
        free(_oCpu[i]);
    }
    free(_params);
    CN_CHECK(cnrtUnloadModel(_mdl));
    cnrtDestroy();
}

int Net::input_dims(int i, int idx)
{
    return _iDimVal[idx][i];
}

int Net::output_dims(int i, int idx)
{
    return _oDimVal[idx][i];
}

int Net::input_set(void *data, int idx)
{
    _params[idx] = data;
    return 0;
}

int Net::input_from(void *data, int idx, bool bNCHW2NHWC)
{
    _params[idx] = _iMlu[idx];
    int ret = 0;
    int dimOrder[4] = {0,2,3,1};//NCHW->NHWC
    int dimValue[4];
    if(bNCHW2NHWC)
    {
        dimValue[0] = _iDimVal[idx][0];//N
        dimValue[1] = _iDimVal[idx][3];//C
        dimValue[2] = _iDimVal[idx][1];//H
        dimValue[3] = _iDimVal[idx][2];//W
    }
    if(_iDataType[idx]==CNRT_FLOAT16)
    {
        if(bNCHW2NHWC)
        {
            ret += CN_CHECK(cnrtTransOrderAndCast(data,CNRT_FLOAT32,_iCpu[idx],CNRT_FLOAT16,nullptr,4,dimValue,dimOrder));
        }
        else
        {
            ret += CN_CHECK(cnrtCastDataType(data, CNRT_FLOAT32, _iCpu[idx], CNRT_FLOAT16, _iDataSize[idx]/2, nullptr));
        }
        ret += CN_CHECK(cnrtMemcpy(_iMlu[idx], _iCpu[idx], _iDataSize[idx], CNRT_MEM_TRANS_DIR_HOST2DEV));
    }
    else
    {
        if(bNCHW2NHWC)
        {
            ret += CN_CHECK(cnrtTransDataOrder(data,_iDataType[idx],_iCpu[idx],4,dimValue,dimOrder));
            ret += CN_CHECK(cnrtMemcpy(_iMlu[idx], _iCpu[idx], _iDataSize[idx], CNRT_MEM_TRANS_DIR_HOST2DEV));
        }
        else
        {
            ret += CN_CHECK(cnrtMemcpy(_iMlu[idx], data, _iDataSize[idx], CNRT_MEM_TRANS_DIR_HOST2DEV));
        } 
    }
    
    return ret;
}

int Net::output_to(void *data, int idx, bool bNHWC2NCHW)
{
    int ret = 0;
    int dimOrder[4] = {0,3,1,2};//NHWC->NCHW
    int dimValue[4];
    if(bNHWC2NCHW)
    {
        dimValue[0] = _oDimVal[idx][0];//N
        dimValue[1] = _oDimVal[idx][1];//H
        dimValue[2] = _oDimVal[idx][2];//W
        dimValue[3] = _oDimVal[idx][3];//C
    }
    if(_oDataType[idx]==CNRT_FLOAT16)
    {
        ret += CN_CHECK(cnrtMemcpy(_oCpu[idx], _oMlu[idx], _oDataSize[idx], CNRT_MEM_TRANS_DIR_DEV2HOST));
        if(bNHWC2NCHW)
        {
            ret += CN_CHECK(cnrtTransOrderAndCast(_oCpu[idx],CNRT_FLOAT16,data,CNRT_FLOAT32,nullptr,4,dimValue,dimOrder));
        }
        else
        {
            ret += CN_CHECK(cnrtCastDataType(_oCpu[idx], CNRT_FLOAT16, data, CNRT_FLOAT32, _oDataSize[idx]/2, nullptr));
        } 
    }
    else
    {
        if(bNHWC2NCHW)
        {
            ret += CN_CHECK(cnrtMemcpy(_oCpu[idx], _oMlu[idx], _oDataSize[idx], CNRT_MEM_TRANS_DIR_DEV2HOST));
            ret += CN_CHECK(cnrtTransDataOrder(_oCpu[idx],_oDataType[idx],data,4,dimValue,dimOrder));
        }
        else
        {
            ret = CN_CHECK(cnrtMemcpy(data, _oMlu[idx], _oDataSize[idx], CNRT_MEM_TRANS_DIR_DEV2HOST));
        }
    }
    
    return ret;
}

int Net::infer()
{
    int ret = 0;
    CN_CHECK(cnrtSetCurrentDevice(_dev));
    if(_affinity==0)
    {
        ret = CN_CHECK(cnrtInvokeRuntimeContext(_ctx, _params, _queue, nullptr));
    }
    else
    {
        ret = CN_CHECK(cnrtInvokeRuntimeContext(_ctx, _params, _queue, &_invoke_param));
    }
    CN_CHECK(cnrtSyncQueue(_queue));

    return ret;
}
