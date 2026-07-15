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
#ifndef _CN_VIDEO_DEC_H_
#define _CN_VIDEO_DEC_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_common.h"

typedef void *cnvideoDecoder;

typedef enum
{
    CNVIDEODEC_INSTANCE_AUTO = (u32_t)(-1),   /* decoder instance id is not set, will be specified by cncodec */
    CNVIDEODEC_INSTANCE_0 = 0,
    CNVIDEODEC_INSTANCE_1,
    CNVIDEODEC_INSTANCE_2,
    CNVIDEODEC_INSTANCE_3,
    CNVIDEODEC_INSTANCE_4,
    CNVIDEODEC_INSTANCE_5,
    CNVIDEODEC_INSTANCE_6,
    CNVIDEODEC_INSTANCE_7,
    CNVIDEODEC_INSTANCE_INVALID,
} cnvideoDecInstance;


typedef enum
{
    CNVIDEODEC_FLAG_EOS           = (1U << 0), /*set when last packet for this stream  */
    CNVIDEODEC_FLAG_TIMESTAMP     = (1U << 1), /*set when timestamp is valid */
    CNVIDEODEC_FLAG_DISCONTINUITY = (1U << 2), /*set when discontinuity is signal */
    CNVIDEODEC_FLAG_END_OF_FRAME  = (1u << 3), /*can be set when the input buffer contains an integral frame data */
} cnvideoDecFlag;

typedef struct
{
    u32_t  bitDepthMinus8;  /* In, Specify BitDepth*/
    u8_t   supported;       /* Out, 0 not supported, otherwise supported*/
    u32_t  maxWidth;        /* Out, Maximum supported decode Width, in Pixel*/
    u32_t  maxHeight;       /* Out, Maximum supported decode Height, in Pixel*/
    u16_t  minWidth;        /* Out, Minimum supported decode Width, in Pixel*/
    u16_t  minHeight;       /* Out, Minimum supported decode Height, in Pixel*/
}cnvideoDecCapability;

/**
 * structure for video Sequence Information
 */
typedef struct
{
    cncodecType codec;            /* Out, compress type */
    u32_t       width;            /* Out, coded frame width, in pixel */
    u32_t       height;           /* Out, coded frame height, in pixel */
    u32_t       minInputBufNum;   /* Out, minmum in buffer is required for decode */
    u32_t       minOutputBufNum;  /* Out, minmum out buffer is requied for decode, including Refs buffer + Cache buffer */

    /* Display area, example:
     * coded_width = 1920, coded_height = 1088
     * Crop_area = { 0,0,1920,1080 }
     */
    struct
    {
        u32_t  left;              /* Out, left position of crop area */
        u32_t  top;               /* Out, Top position of crop area */
        u32_t  right;             /* Out, right position of crop area */
        u32_t  bottom;            /* Out, Bottom position of crop area */
    } cropArea;

    struct
    {
        u32_t  x;
        u32_t  y;
    } aspectRatio;                /* Out, Aspect ratio */

    struct
    {
        u32_t   numerator;
        u32_t   denominator;
    } fps;                        /* Out, stream's framerate, Numerator / Denominator */
} cnvideoDecSequenceInfo;

typedef struct
{
    u32_t                     deviceId;                        /* In, Specify which deviceId to use */
    u32_t                     instance;                        /* In, See cnvideoDecInstance */
    cncodecType               codec;                           /* In, See cnvideoDecoder type */
    cncodecPixelFormat        pixelFmt;                        /* In, Specify Video output format */
    cncodecColorSpace         colorSpace;                      /* In, Specify Video output Color Standard */
    u32_t                     width;                           /* In, Specify video Width, in pixel */
    u32_t                     height;                          /* In, Specify video Height, in Pixel */
    u32_t                     bitDepthMinus8;                  /* In, Specify stream bit depth, value "bitdepth - 8" */
    u32_t                     progressive;                     /* In, specify progressive or interlaced */
    u32_t                     inputBufNum;                     /* In, Number of input surfaces */
    u32_t                     outputBufNum;                    /* In, Number of output surfaces */
    cncodecDevMemory          *inputBuf;                       /* In, specify input buffer information for decoder */
    cncodecFrame              *outputBuf;                      /* In, specify output frame information for decoder */
    cncodecBufAllocType       allocType;                       /* In, specify memory allocate type, app or lib allocates buffer */
    u32_t                     suggestedLibAllocBitStrmBufSize; /* In, input buffer size for lib internal allocate buffer type */
    void*                     userContext;                     /* In, user context  */
    union
    {
        u8_t                  memChannel;                      /* In, specify memory channel when allocType is CNCODEC_BUF_ALLOC_LIB_USER_ASSIGN_MEM_CH */
        cncodecExtBufCfg      bufCfg;
        u64_t                 reserved;
    } extCfg;
    u64_t                     privData;
} cnvideoDecCreateInfo;

typedef struct
{
    cncodecFrame      frame;        /* Out, specify frame information for decoder output*/
    cncodecSliceType  srcScliceType; /* Out, specify nalu type for decoder input, not used now*/
    u32_t             mbErrorCount;  /* Out, specify error mb counts found when decoding, not used now*/
    u64_t             pts;           /* Out, Presentation time for frame */
    u32_t             flags;         /* Out, Flag for this frame, EOS/Timestamp */
    u64_t             reserved;
    u64_t             privData;
} cnvideoDecOutput;

typedef struct
{
    u8_t     *streamBuf;      /* In, Specify bitstream buffer information for decoder input*/
    u32_t    streamLength;    /* In, Specify bitstream size */
    u64_t    pts;             /* In, frame's Presentation time stamp, available when CNVideoPacket_Timestamp is set */
    u32_t    flags;           /* In, See cnvideoDecFlag */
    u64_t    reserved;
    u64_t    privData;
}cnvideoDecInput;

typedef struct {
    u64_t frameCount;         /* Out, total decoded frame count*/
    u64_t frameNumber;        /* Out, error frame number */
    u64_t pts;                /* Out, pts of input buffer */
    u64_t reserved[4];
} cnvideoDecStreamCorruptInfo;

typedef enum {
    CNVIDEO_DEC_ATTR_OUT_BUF_ALIGNMENT = 0U, /* Set output buffer alignment when lib allocates buffer*/
    CNVIDEO_DEC_ATTR_INVALID_CACHE_BEFORE_QUEUE_OUT_BUF, /* invalid cache for output buffer before send it to hardware*/
    CNVIDEO_DEC_ATTR_ENABLE_NONBLOCK_DEC_STOP, /* Enable non-block stop, i.e cnvideoDecStop will stop decode directly without waiting output buffer back */
    CNVIDEO_DEC_ATTR_MAX
} cnvideoDecAttribute;

typedef enum {
    CNVIDEODEC_DECODE_ORDER    =  0,
    CNVIDEODEC_DISPLAY_ORDER   =  1,
} cnvideoDecOutputOrder;

typedef struct {
    cnvideoDecOutputOrder outputFrameOrder;
    u64_t reserved;
} cnvideoDecExtConfig;

/**
 * @brief Get codec detail info
 *
 * @param codec[in], specify which codec is choosen to get caps
 * @param CodecCaps[out], return capability for this codec
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecGetCaps(cncodecType codec, cnvideoDecCapability *caps);

/**
 * @brief create decoder
 *
 * @param handle[out], return decode's handle
 * @param cb[in], set event callback to decode
 * @param createInfo[in], specify decode's parameter
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecCreate(cnvideoDecoder *handle,
                                              pfnCncodecEventCallback cb,
                                              cnvideoDecCreateInfo *createInfo);

/**
 * @brief create decoder
 *
 * @param handle[out], return decode's handle
 * @param cb[in], set event callback to decode
 * @param createInfo[in], specify decode's parameter
 * @param ext_cfg[in], specify decode's ext config
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecCreateEx(cnvideoDecoder *handle,
                                              pfnCncodecEventCallback cb,
                                              cnvideoDecCreateInfo *createInfo,
                                              cnvideoDecExtConfig *ext_cfg);


/**
 * @brief Destroy decoder
 *
 * @param handle[in], decode handle to be destroy
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecDestroy(cnvideoDecoder handle);

/**
 * @brief Abort decoder
 *
 * @param handle[in], decode handle to be abort.
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecAbort(cnvideoDecoder handle);

/**
 * @brief start decode, shall be called when sequence callback be called
 *
 * @param handle[in], decode handle to be started
 * @param CreateInfo[in], config decoder again according to sequence information.
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecStart(cnvideoDecoder handle,
                                             cnvideoDecCreateInfo *startInfo);

/**
 * @brief, stop decode, invoked when eos callback get called
 *
 * @param handle[in], decode handle to be stopped
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecStop(cnvideoDecoder handle);


/**
 * @brief, feed data to decoder
 *
 * @param handle[in], decode's handle
 * @param input[in], Specify bitstream buffer feeded to decoder
 * @param timeout[in], in ms, -1 wait infinitely
 * @return if CNCODEC_TIMEOUT timeout, else if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecFeedData(cnvideoDecoder handle,
                                                cnvideoDecInput *input,
                                                i32_t timeoutMs);
/**
 * @brief,Add refernce counter for video decode output buffer
 *
 * @param handle[in], decode's handle
 * @param frame[in], frame buffer to add ref
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecAddReference(cnvideoDecoder handle, cncodecFrame *frame);

/**
 * @brief,Release refernce counter for video decode output buffer
 *
 * @param handle[in], decode's handle
 * @param frame[in], frame buffer to  release reference
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecReleaseReference(cnvideoDecoder handle, cncodecFrame *frame);

/**
 * @brief, set decoder's attribute
 *
 * @param handle[in], decoder's handle
 * @param attribute[in], specify the attribute to be set
 * @param value[in], set the value for specified attribute
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecSetAttributes(cnvideoDecoder handle,
                                                     cnvideoDecAttribute attribute,
                                                     void *value);

/**
 * @brief, query available input buffer
 *
 * @param handle[in], decoder's handle
 * @return true if input buffer is available, else false.
 */
extern CNCODEC_DLL_API bool cnvideoDecQueryAvailInputBuf(cnvideoDecoder handle);

/**
 * @brief, get the count of available input buffer
 *
 * @param handle[in], decoder's handle
 * @return the count of available input buffer.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecQueryAvailInputBufCnt(cnvideoDecoder handle);

/**
 * @brief, get the count of available output buffer
 *
 * @param handle[in], decoder's handle
 * @return the count of available output buffer.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecQueryAvailOutputBufCnt(cnvideoDecoder handle);

/**
 * @brief, get the optimal memory channle for a decode instance
 *
 * @param instance[in], decode instance
 * @return the optimal memory channel.
 */
extern CNCODEC_DLL_API u32_t cnvideoDecQueryOptimalMemChannel(cnvideoDecInstance instance);

/**
 * @brief, allocate a decode instance which has least loading
 *
 * @param deviceId[in], device id
 * @param is_decode_only[in], 1 means run decode only, 0 means run decode and decode simultaneously
 * @return the optimal decode instance
 */
extern CNCODEC_DLL_API u32_t cnvideoDecAllocOptimalInstance(u32_t deviceId, i32_t is_decode_only);

/**
 * @brief, release decode instance
 *
 * @param deviceId[in], instance id
 * @param instance[in], decode instance
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnvideoDecReleaseInstance(u32_t deviceId, u32_t instance);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif
