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

#ifndef _CN_JPEG_ENC_H_
#define _CN_JPEG_ENC_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_common.h"

typedef  void*  cnjpegEncoder;

typedef enum
{
    CNJPEGENC_INSTANCE_AUTO = (u32_t)(-1),/* jpeg decode instanceid will be set automatically inside cncodec lib*/
    CNJPEGENC_INSTANCE_0 = 0,
    CNJPEGENC_INSTANCE_1,
    CNJPEGENC_INSTANCE_2,
    CNJPEGENC_INSTANCE_3,
    CNJPEGENC_INSTANCE_4,
    CNJPEGENC_INSTANCE_5,
    CNJPEGENC_INSTANCE_INVALID,
} cnjpegEncInstance;

typedef enum
{
    CNJPEGENC_RUN_MODE_SYNC = 0,
    CNJPEGENC_RUN_MODE_ASYNC,
    CNJPEGENC_RUN_MODE_INVALID,
}cnjpegEncRunMode;

typedef enum
{
    CNJPEGENC_FLAG_EOS           = (1U << 0),   /* indicates end of the input stream */
    CNJPEGENC_FLAG_TIMESTAMP     = (1U << 1),   /* indicates PTS is delivered with the frame */
    CNJPEGENC_FLAG_INVALID_FRAME = (1U << 2),   /* indicates invalid input frame */
} cnjpegEncFlag;

typedef struct
{
    u8_t length[16];    /*Huffman Code Bit length*/
    u8_t *values;       /*Huffman coding table */
} cnjpegEncHuffTbl;

typedef struct
{
    cnjpegEncHuffTbl lumaDC;   /*user config Huffman table of Luma DC Bits*/
    cnjpegEncHuffTbl lumaAC;   /*user config Huffman table of Luma AC Bits*/
    cnjpegEncHuffTbl chromaDC; /*user config Huffman table of Chroma DC Bits*/
    cnjpegEncHuffTbl chromaAC; /*user config Huffman table of Chroma AC Bits*/
} cnjpegEncHuffmanTables;

typedef struct
{
    u8_t   lumaQuant[64];     /*user config luma Quantize Table*/
    u8_t   chromaQuant[64];   /*user config chroma Quantize Table*/
} cnjpegEncQuantTables;

typedef enum
{
   CNJPEGENC_ATTR_USER_QUANT_TABLES = 0,    /* corresponding value type pointr cnjpegEncQuantTables */
   CNJPEGENC_ATTR_USER_HUFFMAN_TABLES,      /* corresponding value type pointr cnjpegEncHuffmanTables */
   CNJPEGENC_ATTR_USER_PICTURE_QUALITY,     /* corresponding value type pointr u32_t */
   CNJPEGENC_ATTR_RESTART_INTERVAL,         /* corresponding value type pointr u32_t */
   CNJPEGENC_ATTR_DEFAULT_QUANT_TABLES,
   CNJPEGENC_ATTR_DEFAULT_HUFFMAN_TABLES,
   CNJPEGENC_ATTR_TYPE_MAX,
} cnjpegEncAttributeSetType;

typedef struct
{
    u32_t  quality;           /*user config jpeg encode quality*/
    u32_t  restartInterval;   /*user config jpeg encode restart i32_terval*/
    u64_t  metaData;
} cnjpegEncParameters;

typedef struct
{
    u32_t                   deviceId;                        /*[in] must, default 0 */
    u32_t                   instance;                        /*[in] must, default auto */
    cncodecPixelFormat      pixelFmt;                        /*[in] must，input buffer format */
    cncodecColorSpace       colorSpace;                      /*[in] must, color standard */
    u32_t                   width;                           /*[in] must, specify image width */
    u32_t                   height;                          /*[in] must, specify image height */
    u32_t                   inputBufNum;                     /*[in] must，input buffer number */
    u32_t                   outputBufNum;                    /*[in] must，output buffer number */
    cncodecFrame            *inputBuf;                       /*[in] use app allocated buffer to encode, shall be valid for async decode and app-alloc buffer mode */
    cncodecDevMemory        *outputBuf;                      /*[in] use app allocated buffer to encode, shall be valid for async decode and app-alloc buffer mode */
    cncodecBufAllocType     allocType;                       /*[in] option, specify memory allocate type, lib-alloc or app-alloc */
    u32_t                   suggestedLibAllocBitStrmBufSize; /*[in] output bitstream buffer size for lib internal allocate buffer type */
    void*                   userContext;                     /*[in] user context for async decode callback */
    union
    {
        u8_t                memChannel;                      /*[in] specify memory channel when allocType is CNCODEC_BUF_ALLOC_LIB_USER_ASSIGN_MEM_CH */
        cncodecExtBufCfg    bufCfg;
        u64_t               reserved;
    } extCfg;
    u64_t                   privData;
} cnjpegEncCreateInfo;

typedef struct
{
    cncodecDevMemory streamBuffer;
    u32_t            streamLength;
    u32_t            dataOffset;
    u32_t            result;
    u64_t            pts;
    u32_t            flags;
    u32_t            reserved;
    u64_t            privData;
} cnjpegEncOutput;

typedef struct
{
    cncodecFrame frame;
    u64_t        pts;              /* Out, Presentation time for frame */
    u32_t        flags;            /* Out, Flag for this frame, EOS/Timestamp */
    u64_t        reserved;
    u64_t        privData;
} cnjpegEncInput;

/**
 * @brief create jpeg encoder
 *
 * @param handle[out], return encoder's handle
 * @param mode[in], select decode running mode-sync or async
 * @param cb[in], give event callback to encoder
 * @param createInfo[in], information for creating encoder
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncCreate(cnjpegEncoder *handle,
                                             cnjpegEncRunMode mode,
                                             pfnCncodecEventCallback cb,
                                             cnjpegEncCreateInfo *createInfo);

/**
 * @brief destroy encoder
 *
 * @param handle[in], encoder's handle
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncDestroy(cnjpegEncoder handle);

/**
 * @brief abort encoder
 *
 * @param handle[in], encoder's handle
 * @return CNCODEC_SUCCESS if success, else fail.*/
extern CNCODEC_DLL_API i32_t cnjpegEncAbort(cnjpegEncoder handle);

/**
 * @brief, set encoder's attribute
 *
 * @param handle[in], encoder's handle
 * @param attribute[in], specify the attribute to be set
 * @param value[in], set the value for specified attribute
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncSetAttribute(cnjpegEncoder handle,
                                                   cnjpegEncAttributeSetType attribute,
                                                   void *value);
/**
 * @brief, synchronized jpeg encode
 * @param handle[in], encoder's handle
 * @param output[out], output bitstream data info
 * @param input[in], input frame data buffer info
 * @param params[in], jpeg encode params-e.g. quality
 * @return if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncSyncEncode(cnjpegEncoder handle,
                                                 cnjpegEncOutput *output,
                                                 cnjpegEncInput *input,
                                                 cnjpegEncParameters *params,
                                                 i32_t timeoutMs);

/**
 * @brief, feed data to encode for asynchrnozied decode
 * @param handle[in], decode's handle
 * @param input[in], input frame data buffer info
 * @param params[in], jpeg encode params-e.g. quality
 * @return if CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncFeedFrame(cnjpegEncoder handle,
                                                cnjpegEncInput *input,
                                                cnjpegEncParameters *params,
                                                i32_t timeoutMs);

/**
 * @brief, wait for available input buffer
 *
 * @param handle[in], encoder's handle
 * @param buffer[out], return an available input buffer
 * @timeout [in], timeout in millsecond for waiting
 * @return CNCODEC_SUCCESS if success, else fail.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncWaitAvailInputBuf(cnjpegEncoder handle,
                                                        cncodecFrame* buffer,
                                                        i32_t timeoutMs);

/**
 * @brief,Add reference counter for jpeg encode output buffer
 *
 * @param handle[in], encode's handle
 * @param frame[in], stream buffer to add ref
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncAddReference(cnjpegEncoder handle, cncodecDevMemory *streamBuf);

/**
 * @brief,Release reference counter for jpeg encode output buffer
 *
 * @param handle[in], encode's handle
 * @param frame[in], stream buffer to release reference
 * @return if  CNCODEC_SUCCESS success, else failed.
 */
extern CNCODEC_DLL_API i32_t cnjpegEncReleaseReference(cnjpegEncoder handle, cncodecDevMemory *streamBuf);

/**
 * @brief, get the optimal memory channle for an encode instance
 *
 * @param instance[in], encode instance
 * @return the optimal memory channel.
 */
extern CNCODEC_DLL_API u32_t cnjpegEncGetOptimalMemChannel(cnjpegEncInstance instance);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_JPEG_ENC_H__ */
