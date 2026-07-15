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

#ifndef _CN_JPEG_DEC_H_
#define _CN_JPEG_DEC_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_common.h"

typedef void* cnjpegDecoder;

typedef enum
{
    CNJPEGDEC_INSTANCE_AUTO = (u32_t)(-1),    /* jpeg decode instanceid will be set automatically inside cncodec lib*/
    CNJPEGDEC_INSTANCE_0 = 0,
    CNJPEGDEC_INSTANCE_1,
    CNJPEGDEC_INSTANCE_2,
    CNJPEGDEC_INSTANCE_3,
    CNJPEGDEC_INSTANCE_4,
    CNJPEGDEC_INSTANCE_5,
    CNJPEGDEC_INSTANCE_INVALID,
} cnjpegDecInstance;

typedef enum
{
    CNJPEGDEC_RUN_MODE_SYNC = 0,   /* run synchronized jpeg decode */
    CNJPEGDEC_RUN_MODE_ASYNC,      /* run asynchronized jpeg decode */
    CNJPEGDEC_RUN_MODE_INVALID,
} cnjpegDecRunMode;

typedef enum
{
    CNJPEGDEC_FLAG_EOS           = (1U << 0), /*set when last packet for this stream  */
    CNJPEGDEC_FLAG_TIMESTAMP     = (1U << 1), /*set when timestamp is valid */
    CNJPEGDEC_FLAG_DISCONTINUITY = (1U << 2), /*set when discontinuity is signal */
} cnjpegDecFlag;

typedef struct
{
    u32_t type;
    u32_t length;
    void* data;
} cnjpegAppMarker;

#define CNJPEG_DEC_MAX_SUPPORT_APP_MARK_NUM (16U)

typedef struct
{
    u32_t           width;    /* output picture width */
    u32_t           height;   /* output picture height */
    u32_t           markNum;  /* the number of App merkers */
    cnjpegAppMarker mark[CNJPEG_DEC_MAX_SUPPORT_APP_MARK_NUM];
} cnjpegDecImageInfo;

typedef struct
{
    u32_t               deviceId;                        /* [in] Specify which card device to use */
    u32_t               instance;                        /* [in] See cnjpegDecInstance */
    cncodecPixelFormat  pixelFmt;                        /*[in], Specify JPEG output format */
    cncodecColorSpace   colorSpace;                      /*[in], Specify JPEG output Color Standard */
    u32_t               width;                           /*[in], Specify JPEG Width, in pixel */
    u32_t               height;                          /*[in], Specify JPEG Height, in Pixel */
    u32_t               bitDepthMinus8;                  /*[in], Specify JPEG bit depth, value "bitdepth - 8" */
    u32_t               inputBufNum;                     /* [in] Number of input surfaces */
    u32_t               outputBufNum;                    /* [in] Number of output surfaces */
    cncodecDevMemory    *inputBuf;                       /* [in] for app allcoated buffer case, app set input mjpeg data feed used bit stream buffer */
    cncodecFrame        *outputBuf;                      /* [in] for app allcoated buffer case, app set output frame buffer used bit stream buffer */
    cncodecBufAllocType allocType;                       /* [in] See CNVideo_MemoryAllocType, if Use, APP should allocate memory */
    u32_t               suggestedLibAllocBitStrmBufSize; /* [in] input bitstream buffer size for lib internal allocate buffer type */
    u32_t               enablePreparse;                  /* [in] enable internal parse mjpeg boundary to guarnatee integrated jpeg stream decoding */
    void*               userContext;                     /* [in] user context for async decode callback */
    union
    {
        u8_t            memChannel;                      /* [in] specify memory channel when allocType is CNCODEC_BUF_ALLOC_LIB_USER_ASSIGN_MEM_CH */
        cncodecExtBufCfg bufCfg;
        u64_t           reserved;
    } extCfg;
    u64_t               privData;
} cnjpegDecCreateInfo;

typedef struct
{
    cncodecFrame frame;    /* Out, frame data */
    u32_t        flags;    /* Out, Flag for this frame, EOS/Timestamp */
    u64_t        pts;      /* Out, Presentation time for frame */
    u32_t        result;   /* 0-success, non-zero value indicated decode fail */
    u64_t        reserved;
    u64_t        privData;
} cnjpegDecOutput;

typedef struct
{
    u8_t  *streamBuffer;   /* In, jpeg stream Buffer */
    u32_t streamLength;    /* In, validate jpeg stream length */
    u32_t flags;           /* In, See CNVideoPacketFlag */
    u64_t pts;             /* In, Presentation time stamp */
    u64_t reserved;
    u64_t privData;
} cnjpegDecInput;

typedef struct
{
    u32_t maxWidth;        /* Out, Maximum supported decode Width, in Pixel*/
    u32_t maxHeight;       /* Out, Maximum supported decode Height, in Pixel*/
    u32_t minWidth;        /* Out, Minimum supported decode Width, in Pixel*/
    u32_t minHeight;       /* Out, Minimum supported decode Height, in Pixel*/
    u64_t reserved[16];    /* reserved for future extension */
} cnjpegDecCapability;

/**
 * @brief Get codec detail info
 *
 * @param caps[out], return capability for jpeg decode
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecGetCapability(cnjpegDecCapability *caps);

/**
 * @brief create jpeg decoder
 *
 * @param handle[out], return decode's handle
 * @param mode[in], select decode running mode-sync or async
 * @param cb[in], set event callback to decode
 * @param createInfo[in], specify decode's parameters
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecCreate(cnjpegDecoder *handle,
                                             cnjpegDecRunMode mode,
                                             pfnCncodecEventCallback cb,
                                             cnjpegDecCreateInfo *createInfo);

/**
 * @brief Destroy decoder
 *
 * @param handle[in], decode handle to be destroy
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecDestroy(cnjpegDecoder handle);

/**
 * @brief Abort decoder
 *
 * @param handle[in], decode handle to be abort.
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecAbort(cnjpegDecoder handle);

/**
 * @brief parse Jpeg Header info
 *
 * @param handle[in], decode handle to be started
 * @param info[out], jpeg header marks/resolution info
 * @param streamStart[in], jpeg bit stream start virtual addr
 * @param streamLength[in],  jpeg bit stream length
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecGetImageInfo(cnjpegDecoder handle,
                                                   cnjpegDecImageInfo *info,
                                                   void* streamStart,
                                                   u32_t streamLength);
/**
 * @brief,  synchronized jpeg decode
 *
 * @param handle[in], decode's handle
 * @param output[out], frame data buffer info after decode done
 * @param input[in], input bitstream data info
 * @return if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecSyncDecode(cnjpegDecoder handle,
                                                 cnjpegDecOutput *output,
                                                 cnjpegDecInput *input,
                                                 i32_t timeoutMs);

/**
 * @brief, feed data to decoder for asynchrnozied decode
 *
 * @param handle[in], decode's handle
 * @param input[in], Specify bitstream buffer feeded to decoder
 * @param timeout[in], in ms, -1 wait infinitely
 * @return if CNCODEC_TIMEOUT timeout, else if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecFeedData(cnjpegDecoder handle,
                                               cnjpegDecInput *input,
                                               i32_t timeoutMs);

/**
 * @brief,Add refernce counter for jpeg decode output buffer
 *
 * @param handle[in], decode's handle
 * @param frame[in], frame buffer to add ref
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecAddReference(cnjpegDecoder handle, cncodecFrame *frame);

/**
 * @brief,Add refernce counter for jpeg decode output buffer
 *
 * @param handle[in], decode's handle
 * @param frame[in], frame buffer to  release reference
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegDecReleaseReference(cnjpegDecoder handle, cncodecFrame *frame);

/**
 * @brief, get the optimal memory channle for a decode instance
 *
 * @param instance[in], decode instance
 * @return the optimal memory channel.
 */
extern CNCODEC_DLL_API u32_t cnjpegDecGetOptimalMemChannel(cnjpegDecInstance instance);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_JPEG_DEC_H_ */
