/*************************************************************************
 * Copyright (C) [2019] by Cambricon, Inc.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *************************************************************************/
#ifndef _CN_VIDEO_ENC_H_
#define _CN_VIDEO_ENC_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_common.h"

typedef void *cnvideoEncoder;

typedef enum
{
    CNVIDEOENC_INSTANCE_AUTO = (u32_t)(-1),   /* ecoder instance id is not set, will be specified by cncodec */
    CNVIDEOENC_INSTANCE_0 = 0,
    CNVIDEOENC_INSTANCE_1,
    CNVIDEOENC_INSTANCE_2,
    CNVIDEOENC_INSTANCE_3,
    CNVIDEOENC_INSTANCE_4,
    CNVIDEOENC_INSTANCE_5,
    CNVIDEOENC_INSTANCE_INVALID
} cnvideoEncInstance;

typedef enum
{
    CNVIDEOENC_PROFILE_H264_BASELINE = 0,
    CNVIDEOENC_PROFILE_H264_MAIN,
    CNVIDEOENC_PROFILE_H264_HIGH,
    CNVIDEOENC_PROFILE_H264_HIGH_10,
    CNVIDEOENC_PROFILE_H265_MAIN,
    CNVIDEOENC_PROFILE_H265_MAIN_STILL,
    CNVIDEOENC_PROFILE_H265_MAIN_INTRA,
    CNVIDEOENC_PROFILE_H265_MAIN_10,
    CNVIDEOENC_PROFILE_MAX
} cnvideoEncProfile;

typedef enum
{
    CNVIDEOENC_LEVEL_H264_1 = 0,
    CNVIDEOENC_LEVEL_H264_1B,
    CNVIDEOENC_LEVEL_H264_11,
    CNVIDEOENC_LEVEL_H264_12,
    CNVIDEOENC_LEVEL_H264_13,
    CNVIDEOENC_LEVEL_H264_2,
    CNVIDEOENC_LEVEL_H264_21,
    CNVIDEOENC_LEVEL_H264_22,
    CNVIDEOENC_LEVEL_H264_3,
    CNVIDEOENC_LEVEL_H264_31,
    CNVIDEOENC_LEVEL_H264_32,
    CNVIDEOENC_LEVEL_H264_4,
    CNVIDEOENC_LEVEL_H264_41,
    CNVIDEOENC_LEVEL_H264_42,
    CNVIDEOENC_LEVEL_H264_5,
    CNVIDEOENC_LEVEL_H264_51,

    CNVIDEOENC_LEVEL_H265_MAIN_1,
    CNVIDEOENC_LEVEL_H265_HIGH_1,
    CNVIDEOENC_LEVEL_H265_MAIN_2,
    CNVIDEOENC_LEVEL_H265_HIGH_2,
    CNVIDEOENC_LEVEL_H265_MAIN_21,
    CNVIDEOENC_LEVEL_H265_HIGH_21,
    CNVIDEOENC_LEVEL_H265_MAIN_3,
    CNVIDEOENC_LEVEL_H265_HIGH_3,
    CNVIDEOENC_LEVEL_H265_MAIN_31,
    CNVIDEOENC_LEVEL_H265_HIGH_31,
    CNVIDEOENC_LEVEL_H265_MAIN_4,
    CNVIDEOENC_LEVEL_H265_HIGH_4,
    CNVIDEOENC_LEVEL_H265_MAIN_41,
    CNVIDEOENC_LEVEL_H265_HIGH_41,
    CNVIDEOENC_LEVEL_H265_MAIN_5,
    CNVIDEOENC_LEVEL_H265_HIGH_5,
    CNVIDEOENC_LEVEL_H265_MAIN_51,
    CNVIDEOENC_LEVEL_H265_HIGH_51,
    CNVIDEOENC_LEVEL_H265_MAIN_52,
    CNVIDEOENC_LEVEL_H265_HIGH_52,
    CNVIDEOENC_LEVEL_H265_MAIN_6,
    CNVIDEOENC_LEVEL_H265_HIGH_6,
    CNVIDEOENC_LEVEL_H265_MAIN_61,
    CNVIDEOENC_LEVEL_H265_HIGH_61,
    CNVIDEOENC_LEVEL_H265_MAIN_62,
    CNVIDEOENC_LEVEL_H265_HIGH_62,
    CNVIDEOENC_LEVEL_MAX
} cnvideoEncLevel;

typedef enum
{
    CNVIDEOENC_RATE_CTRL_VBR = 0,
    CNVIDEOENC_RATE_CTRL_CBR,
    CNVIDEOENC_RATE_CTRL_CQP,
    CNVIDEOENC_RATE_CTRL_MODE_MAX
} cnvideoEncRateCtrlMode;

typedef enum
{
    CNVIDEO_ENC_ATTR_LEVEL = 0,             /* set stream's level */
    CNVIDEO_ENC_ATTR_PROFILE,               /* set stream's profile(see cnvideoEncLevel) */
    CNVIDEO_ENC_ATTR_FRAME_RATE,            /* set stream's framerate */
    CNVIDEO_ENC_ATTR_RATE_CTRL_MODES,       /* set cnvideoEncRateCtrlMode */
    CNVIDEO_ENC_ATTR_H264_I_FRAME_QP,       /* set i frame's qp, only available when no bitrate control */
    CNVIDEO_ENC_ATTR_H264_P_FRAME_QP,       /* set p frame's qp, only available when no bitrate control */
    CNVIDEO_ENC_ATTR_H264_B_FRAME_QP,       /* set b frame's qp, only available when no bitrate control */
    CNVIDEO_ENC_ATTR_QP_RANGE,              /* set qp range. only available when enabling bitrate control */
    CNVIDEO_ENC_ATTR_VBV,                   /* set encoding's virtual buffer size */
    CNVIDEO_ENC_ATTR_REF_FRAMES,            /* set maximum reference frame number for encoder */
    CNVIDEO_ENC_ATTR_SLICE_INTRA_REFRESH,   /* Set infra refresh interval for encoder */
    CNVIDEO_ENC_ATTR_NUM_BFRAMES,           /* set b frame's number */
    CNVIDEO_ENC_ATTR_INSERT_SPS_PPS_AT_IDR, /* insert sps&pps when idr */
    CNVIDEO_ENC_ATTR_H264_ENTROPY_MODE,     /* set entropy mode(cavlc or cabac) when encoding to h264 */
    CNVIDEO_ENC_ATTR_GOP_SIZE,              /* set gop's length */
    CNVIDEO_ENC_ATTR_IDR_INTERVAL,          /* set idr interval */
    CNVIDEO_ENC_ATTR_ROI,                   /* set encode regions of interest */
    CNVIDEO_ENC_ATTR_TYPE_MAX,
} cnvideoEncAttribute;

typedef enum
{
    CNVIDEOENC_ENTROPY_MODE_CAVLC = 0,
    CNVIDEOENC_ENTROPY_MODE_CABAC,
    CNVIDEOENC_ENTROPY_MODE_MAX
} cnvideoEncEntropyMode;

typedef enum
{
    CNVIDEOENC_SLICE_MODE_SINGLE = 0,
    CNVIDEOENC_SLICE_MODE_MAX_MB,
    CNVIDEOENC_SLICE_MODE_MAX
} cnvideoEncSliceMode;

typedef enum
{
    CNVIDEOENC_FLAG_EOS               = (1U << 0),   /* indicates end of the input stream */
    CNVIDEOENC_FLAG_FORCE_INTRA       = (1U << 1),   /* force slice in intra */
    CNVIDEOENC_FLAG_FORCE_IDR         = (1U << 2),   /* force slice in idr */
    CNVIDEOENC_FLAG_INSERT_SPSPPS     = (1U << 3),   /* insert sps/pps in current frame */
    CNVIDEOENC_FLAG_INVALID_FRAME     = (1U << 4),   /* indicates invalid input frame*/
} cnvideoEncFlag;

/**
 * H265 CU(coded Unit) size type
 */
typedef enum
{
    CNVIDEOENC_H265_CUSIZE_8X8 = 0,
    CNVIDEOENC_H265_CUSIZE_16X16,
    CNVIDEOENC_H265_CUSIZE_32X32,
    CNVIDEOENC_H265_CUSIZE_64X64,
    CNVIDEOENC_H265_CUSIZE_INVALID
}cnvideoEncH265CUSize;

typedef enum
{
    CNVIDEOENC_GOP_TYPE_BIDIRECTIONAL = 0,
    CNVIDEOENC_GOP_TYPE_LOW_DELAY,
    CNVIDEOENC_GOP_TYPE_PYRAMID,
    CNVIDEOENC_GOP_TYPE_MAX
} cnvideoEncGopType;

typedef struct
{
    u32_t                    gopLength;             /* [in] specify GOP length */
    u32_t                    constIQP;              /* [in] specify QP for I frame */
    u32_t                    constPQP;              /* [in] specify QP for P frame */
    u32_t                    constBQP;              /* [in] specify QP for B frame */
    u32_t                    minIQP;                /* [in] specify min QP for I frame */
    u32_t                    minPQP;                /* [in] specify min QP for P frame */
    u32_t                    minBQP;                /* [in] specify min QP for B frame */
    u32_t                    maxIQP;                /* [in] specify max QP for I frame */
    u32_t                    maxPQP;                /* [in] specify max QP for P frame */
    u32_t                    maxBQP;                /* [in] specify Max QP for B frame */
    u32_t                    initIQP;               /* [in] specify initial QP for I frame */
    u32_t                    initPQP;               /* [in] specify initial QP for P frame */
    u32_t                    initBQP;               /* [in] specify initial QP for B frame */
    u32_t                    targetBitrate;         /* [in] specify target bitrate(in bits/sec) */
    u32_t                    peakBitrate;           /* [in] specify peak bitrate(in bits/sec)，Available when VBR */
    u32_t                    virtualBufferSize;     /* [in] specify VBV(HRD) buffer size，0, use default. */
    u32_t                    enableQPRange;         /* [in] 1，min(I/B/P)QP, maxXQP work , 0, not work*/
    cnvideoEncRateCtrlMode   rcMode;                /* [in] specify rate control Mode */
} cnvideoEncRateCtrl;

typedef struct
{
    cnvideoEncProfile        profile;               /* [in]: specify stream's profile */
    cnvideoEncLevel          level;                 /* [in]: specify stream's level */
    u32_t                    insertSpsPpsWhenIDR;   /* [in]: 1, insert SPS/PPS before IDR， 0 not */
    u32_t                    IframeInterval;        /* [in]: Intra frame's interval */
    u32_t                    IDRInterval;           /* [in]: IDR frame's interval */
    u32_t                    enableIR;              /* [in]: 1 enable Encode's intra refresh, 0 not */
    u32_t                    IRCount;               /* [in]: MB count of Intra refresh */
    u32_t                    maxRefFramesNum;       /* [in]: Max ref number our encoder can support */
    u32_t                    BFramesNum;            /* [in]: B frame number encoder can support */
    u32_t                    horizontalRange;       /* [in]: horizontal search range for motion vector calculation*/
    u32_t                    verticalRange;         /* [in]: vertical search range for motion vector calculation*/
    u32_t                    cabacInitIDC;          /* [in]: CABAC initial table, 0,1,2 for H264 and 0,1 for HEVC*/
    u32_t                    maxMBPerSlice;         /* [in]: max MB in one Slice*/
    cnvideoEncEntropyMode    entropyMode;           /* [in]: CAVLC or CABAC*/
    cnvideoEncSliceMode      sliceMode;             /* [in]: Slice mode of a encoder session, single/MaxMB */
    cnvideoEncGopType        gopType;               /* [in]: Specify gop type, see cnvideoEncSliceMode*/
    u32_t                    chromaFormatIDC;       /* [in]: Specify chromaIDC 1 yuv420, 3 yuv444 */
    u32_t                    bitDepth;              /* [in]: Specify bit depth 1 8bit, 2 10bit, 3 12bit */
    u32_t                    enableROI;             /* [in]: enable or disable roi encode*/
} cnvideoEncH264Config;

typedef struct
{
    cnvideoEncProfile        profile;               /* [in]: specify stream's profile */
    cnvideoEncLevel          level;                 /* [in]: specify stream's level */
    u32_t                    tier;                  /* [in]: specify stream's level tier */
    cnvideoEncH265CUSize     minCU;                 /* [in]: specify stream's min CU(code unit) size*/
    cnvideoEncH265CUSize     maxCU;                 /* [in]: specify stream's max CU size*/
    u32_t                    insertSpsPpsWhenIDR;   /* [in]: 1 insert SPSPPS before IDR， 0 not */
    u32_t                    IframeInterval;        /* [in]: Intra frame's interval */
    u32_t                    IDRInterval;           /* [in]: IDR frame's interval */
    u32_t                    enableIR;              /* [in]: 1 enable Encode's intra refresh, 0 not */
    u32_t                    IRCount;               /* [in]: MB count of Intra refresh */
    u32_t                    maxRefFramesNum;       /* [in]: Max Ref frame num of encoder session */
    u32_t                    BFramesNum;            /* [in]: Max B frame num of encode session */
    u32_t                    horizontalRange;       /* [in]: horizontal search range for motion vector calculation*/
    u32_t                    verticalRange;         /* [in]: vertical search range for motion vector calculation*/
    u32_t                    cabacInitIDC;          /* [in]: CABAC initial table, 0,1,2 for H264 and 0,1 for HEVC*/
    u32_t                    maxMBPerSlice;         /* [in]: max MB in one Slice*/
    cnvideoEncSliceMode      sliceMode;             /* [in]: Slice mode of a encoder session, single/MaxMB */
    cnvideoEncGopType        gopType;               /* [in]: Specify gop type, see cnvideoEncSliceMode*/
    u32_t                    chromaFormatIDC;       /* [in]: Specify chromaIDC 1 yuv420, 3 yuv444 */
    u32_t                    bitDepth;              /* [in]: Specify bit depth 1 8bit, 2 10bit, 3 12bit*/
    u32_t                    enableROI;             /* [in]: enable or disable roi encode*/
} cnvideoEncH265Config;

typedef struct
{
    u32_t                    deviceId;                        /*[in] must, default 0 */
    cnvideoEncInstance       instance;                        /*[in] must, default auto */
    cncodecType              codec;                           /*[in] must，only support 264 H265 */
    cncodecPixelFormat       pixelFmt;                        /*[in] must，input buffer format */
    cncodecColorSpace        colorSpace;                      /*[in] must, color standard */
    u32_t                    width;                           /*[in] must, specify video width */
    u32_t                    height;                          /*[in] must, specify video height */
    u32_t                    fpsNumerator;                    /*[in] option，specify frameRateNum，default 30 */
    u32_t                    fpsDenominator;                  /*[in] option，specify frameRateDen, default 1，frame rate = frameRateNum / frameRateDen ). */
    cnvideoEncRateCtrl       rateCtrl;                        /*[in] set parameter for rate control */
    union
    {
        cnvideoEncH264Config h264;                            /*[in] set encode config for H265 encoder */
        cnvideoEncH265Config h265;                            /*[in] set encode config for H264 encoder */
    } uCfg;
    u32_t                    inputBufNum;                     /*[in] must，input buffer number */
    u32_t                    outputBufNum;                    /*[in] must，output buffer number */
    cncodecFrame             *inputBuf;                       /*[in] option, input buffer info when allocType is CNCODEC_BUF_ALLOC_APP，it is must */
    cncodecDevMemory         *outputBuf;                      /*[in] option, output buffer info when allocType is CNCODEC_BUF_ALLOC_APP，it is must */
    cncodecBufAllocType      allocType;                       /*[in] must, specify memory allocate type, app or lib allocates buffer */
    u32_t                    suggestedLibAllocBitStrmBufSize; /*[in] output bitstream buffer size for lib internal allocate buffer type */
    void*                    userContext;                     /*[in] user context */
    union
    {
        u8_t                  memChannel;                     /*[in] specify memory channel when allocType is CNCODEC_BUF_ALLOC_LIB_USER_ASSIGN_MEM_CH */
        cncodecExtBufCfg      bufCfg;
        u64_t                 reserved;
    } extCfg;
    u64_t                    privData;
} cnvideoEncCreateInfo;

typedef struct
{
    cncodecDevMemory    streamBuffer;
    u32_t               streamLength;
    u32_t               dataOffset;
    u64_t               pts;
    u32_t               flags;
    cncodecSliceType    sliceType;
    u64_t               privateData;
    u64_t               reserved;
} cnvideoEncOutput;

typedef struct
{
    u16_t mbxLeft;                /* macroblock x left edge   (inclusive) */
    u16_t mbxRight;               /* macroblock x right edge  (exclusive) */
    u16_t mbyTop;                 /* macroblock y top edge    (inclusive) */
    u16_t mbyBottom;              /* macroblock y bottom edge (exclusive) */
    i16_t qpDelta;                /* QP delta value for this region */
} cnvideoEncRegion;

/**
 * Regions of interest when encoding h264 and h265
 */
typedef struct
{
    u32_t            count;       /*number of regions */
    cnvideoEncRegion region[16];  /*regions info*/
} cnvideoEncRegions;

typedef struct
{
    cncodecFrame     frame;       /* [in]: input frame for encoder */
    u64_t            pts;         /* [in]: PTS of input frame */
    u32_t            flags;       /* [in]: See cnvideoEncFlag. */
    u32_t            reserved;
} cnvideoEncInput;

/**
 * @brief create encoder
 *
 * @param handle[out], return encoder's handle
 * @param cb[in], give event callback to encoder
 * @param createInfo[in], information for creating encoder
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncCreate(cnvideoEncoder *handle,
                                              pfnCncodecEventCallback cb,
                                              cnvideoEncCreateInfo *createInfo);

/**
 * @brief destroy encoder
 *
 * @param handle[in], encoder's handle
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncDestroy(cnvideoEncoder handle);

/**
 * @brief abort encoder
 *
 * @param handle[in], encoder's handle
 * @return CNCODEC_SUCCESS if success, else fail.*/
extern CNCODEC_DLL_API i32_t cnvideoEncAbort(cnvideoEncoder handle);

/**
 * @brief, set encoder's attribute
 *
 * @param handle[in], encoder's handle
 * @param attribute[in], specify the attribute to be set
 * @param value[in], set the value for specified attribute
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncSetAttributes(cnvideoEncoder handle,
                                                     cnvideoEncAttribute attribute,
                                                     void *value);

/**
 * @brief, feed frame to encoder
 *
 * @param handle[in], encoder's handle
 * @param input[in], pass frame to encoder, frame related info will be given to encoder too
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncFeedFrame(cnvideoEncoder handle,
                                                 cnvideoEncInput *input,
                                                 i32_t timeoutMs);

/**
 * @brief, wait for available input buffer
 *
 * @param handle[in], encoder's handle
 * @param buffer[out], return an available input buffer pointer
 * @timeout [in], timeout in millsecond for waiting
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncWaitAvailInputBuf(cnvideoEncoder handle,
                                                         cncodecFrame *buffer,
                                                         i32_t timeoutMs);

/**
 * @brief,Add reference counter for video encode output buffer
 *
 * @param handle[in], encode's handle
 * @param frame[in], stream buffer to add ref
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncAddReference(cnvideoEncoder handle, cncodecDevMemory *streamBuf);

/**
 * @brief,Release reference counter for video encode output buffer
 *
 * @param handle[in], encode's handle
 * @param frame[in], stream buffer to release reference
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncReleaseReference(cnvideoEncoder handle, cncodecDevMemory *streamBuf);

/**
 * @brief, get the optimal memory channle for an encode instance
 *
 * @param instance[in], encode instance
 * @return the optimal memory channel.
 */
extern CNCODEC_DLL_API u32_t cnvideoEncQueryOptimalMemChannel(cnvideoEncInstance instance);

/**
 * @brief, get parameter set data directly based on encode create info
 *
 * @param createInfo[in] information for creating encoder
  *@param timeout[in] timeout in millsecond for waiting
 * @return parameter set data if success, NULL if failed.
 */
extern CNCODEC_DLL_API cnvideoEncOutput *cnvideoEncGetParamSet(cnvideoEncCreateInfo *createInfo,
                                                               i32_t timeoutMs);
/**
 * @brief, release parameter set data
 *
 * @param paramSetData[in] parameter set data
 * @return if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncReleaseParamSet(cnvideoEncOutput *paramSetData);

/**
 * @brief, allocate an encode instance which has least loading
 *
 * @param deviceId[in], device id
 * @param is_decode_only[in], 1 means run enode only, 0 means run decode and decode simultaneously
 * @return the optimal encode instance
 */
extern CNCODEC_DLL_API u32_t cnvideoEncAllocOptimalInstance(u32_t deviceId, i32_t is_encode_only);

/**
 * @brief, release encode instance
 *
 * @param deviceId[in], device id
 * @param instance[in], instance id
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncReleaseInstance(u32_t deviceId, u32_t instance);

/**
 * @brief, get the count of available input buffer
 *
 * @param handle[in], encoder's handle
 * @return the count of available input buffer.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncQueryAvailInputBufCnt(cnvideoEncoder handle);

/**
 * @brief, get the count of available output buffer
 *
 * @param handle[in], encoder's handle
 * @return the count of available output buffer.
 */
extern CNCODEC_DLL_API i32_t cnvideoEncQueryAvailOutputBufCnt(cnvideoEncoder handle);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_VIDEO_ENC_H_ */
