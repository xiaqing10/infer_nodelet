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
#ifndef _CN_CODEC_COMMON_H_
#define _CN_CODEC_COMMON_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_define.h"

#define CNCODEC_MAJOR_VERSION    1
#define CNCODEC_MINOR_VERSION    8
#define CNCODEC_PATCH_VERSION    112

#define CNCODEC_VERSION (CNCODEC_MAJOR_VERSION * 10000 + CNCODEC_MINOR_VERSION * 100 + CNCODEC_PATCH_VERSION)

#define CN_MAX_SURFACE_PLANAR_NUM  (6U)

typedef enum
{
    CNCODEC_SUCCESS        = 0,  /* success */
    CNCODEC_INVALID_PARAM  = 1,  /* parameter error */
    CNCODEC_NOT_SUPPORT    = 2,  /* function not implemented */
    CNCODEC_INVALID_HANDLE = 3,  /* invalid handle passed to codec */
    CNCODEC_TIMEOUT        = 4,  /* time out */
    CNCODEC_MALLOC_FAIL    = 5,  /* internal buffer allocation fail */
    CNCODEC_BUFFER_ERROR   = 6,  /* buffer error */
    CNCODEC_JPU_ENC_FAIL   = 7,  /* JPU encode jpeg picture failed */
    CNCODEC_JPU_DEC_FAIL   = 8,  /* JPU decode jpeg picture failed */
    CNCODEC_CREATE_FAIL    = 9,  /* encode/decode create failure */
    CNCODEC_OVER_CAPACITY  = 10, /* over capacity error */
    CNCODEC_CONVERT_FAIL   = 11, /* convert to phy ID failed*/
    CNCODEC_ERR_UNKNOWN    = 999 /* unknown error */
} cncodecRetCode;

typedef enum
{
    CNCODEC_MPEG2 = 0,
    CNCODEC_MPEG4,
    CNCODEC_H264,
    CNCODEC_HEVC,
    CNCODEC_VP8,
    CNCODEC_VP9,
    CNCODEC_AVS,
    CNCODEC_JPEG,
    CNCODEC_TOTAL_COUNT, /* Total codec type count */
} cncodecType;

typedef enum
{
    CNCODEC_PIX_FMT_NV12 = 0,     /* SEMI-PLANAR Y4-U1V1 */
    CNCODEC_PIX_FMT_NV21,         /* SEMI-PLANAR Y4-V1U1 */
    CNCODEC_PIX_FMT_I420,         /* PLANAR Y4-U1-V1 */
    CNCODEC_PIX_FMT_YV12,         /* PLANAR Y4-V1-U1 */
    CNCODEC_PIX_FMT_YUYV,         /* 8 BIT PACKED Y2U1Y2V1 */
    CNCODEC_PIX_FMT_UYVY,         /* 8 BIT PACKED U1Y2V1Y2 */
    CNCODEC_PIX_FMT_YVYU,         /* 8 BIT PACKED Y2V1Y2U1 */
    CNCODEC_PIX_FMT_VYUY,         /* 8 BIT PACKED V1Y2U1Y2 */
    CNCODEC_PIX_FMT_P010,         /* 10 BIT SEMI-PLANAR Y4-U1V1. */
    CNCODEC_PIX_FMT_YUV420_10BIT, /* 10 BIT PLANAR Y4-U1-V1. */
    CNCODEC_PIX_FMT_YUV444_10BIT, /* 10 BIT PLANAR Y4-U4-V4.*/
    CNCODEC_PIX_FMT_ARGB,         /* PACKED A8R8G8B8. */
    CNCODEC_PIX_FMT_ABGR,         /* PACKED A8B8G8R8. */
    CNCODEC_PIX_FMT_BGRA,         /* PACKED B8G8R8A8. */
    CNCODEC_PIX_FMT_RGBA,         /* PACKED A8B8G8R8. */
    CNCODEC_PIX_FMT_AYUV,         /* 8 BIT PACKED A8Y8U8V8. */
    CNCODEC_PIX_FMT_RGB565,       /* 8 BIT PACKED R5G6B5. */
    CNCODEC_PIX_FMT_RAW,          /* RAW DMA BUFFER. */
    CNCODEC_PIX_FMT_TOTAL_COUNT
} cncodecPixelFormat;

typedef enum
{
    CNCODEC_BUF_ALLOC_LIB = 0, /* CNCODEC WILL PREPARE MEMORY */
    CNCODEC_BUF_ALLOC_APP,     /* CNCODEC WILL USE MEMORY FROM APP */
    CNCODEC_BUF_ALLOC_LIB_USER_SPECIFY_MEM_CH, /* CNCODEC WILL PREPARE MEMORY AND APP CAN SPECIFY MEM CHANNEL */
    CNCODEC_BUF_ALLOC_LIB_USER_SPECIFY_BUF_ALIGNMENT, /* CNCODEC WILL PREPARE MEMORY AND APP CAN SPECIFY MEM FRAME BUF ALIGNMENT */
    CNCODEC_BUF_ALLOC_LIB_APPLY_ALL_EXT_CFG,   /* CNCODEC WILL PREPARE MEMORY AND APP CAN SPECIFY MEM CHANNEL AND BUF STRIDE */
    CNCODEC_JPEG_BUF_ALLOC_APP_SPECIFY_BUF_ALIGNMENT, /* CNCODEC WILL USE MEMORY FROM APP, APP CAN SPECIFY ALIGNMENT*/
                                                    /*   THIS MODE IS ONLY FOR JPEG*/
    CNCODEC_BUF_ALLOC_INVALID_TYPE,
} cncodecBufAllocType;

typedef enum
{
    CNCODEC_SLICE_NALU_P = 0,
    CNCODEC_SLICE_NALU_B,
    CNCODEC_SLICE_NALU_I,
    CNCODEC_SLICE_NALU_IDR,         /*when enable insertSpsPpsWhenIDR, vps/sps/pps will be inserted into IDR frame*/
    CNCODEC_SLICE_NALU_BI,
    CNCODEC_SLICE_NALU_SPS,
    CNCODEC_SLICE_NALU_PPS,
    CNCODEC_SLICE_NALU_SEI,
    CNCODEC_SLICE_NALU_SKIPPED,
    CNCODEC_SLICE_NALU_INTRA_REFRESH,
    CNCODEC_SLICE_NALU_VPS,
    CNCODEC_SLICE_H264_SPS_PPS,     /* consist of sps & pps*/
    CNCODEC_SLICE_HEVC_VPS_SPS_PPS, /* consist of vps & sps & pps */
    CNCODEC_SLICE_UNKNOWN_TYPE,
    CNCODEC_SLICE_TYPE_MAX
} cncodecSliceType;

typedef enum
{
    CNCODEC_CB_EVENT_NEW_FRAME  = 0, /* for both decode/encode, notify sequence callback  */
    CNCODEC_CB_EVENT_SEQUENCE,       /* video decode got resolution change, notify sequence callback  */
    CNCODEC_CB_EVENT_EOS,            /* for both decode/encode, notify eos is reqched */
    CNCODEC_CB_EVENT_SW_RESET,       /* software reset in cncodec library, ALL CODEC INSTANCE WILL DOWN*/
    CNCODEC_CB_EVENT_HW_RESET,       /* cambricon hardware reset, ALL CODEC INSTANCE WILL DOWN */
    CNCODEC_CB_EVENT_OUT_OF_MEMORY,  /* no enough memory for decoder or encoder*/
    CNCODEC_CB_EVENT_ABORT_ERROR,    // cncodec occurs abort error,
                                     // possible causes: out of memory, unsupported code stream or invaid buffer
    CNCODEC_CB_EVENT_STREAM_CORRUPT, /* stream corrupt, discard frame */
    CNCODEC_CB_EVENT_MAX,
} cncodecCbEventType;

typedef enum
{
    CNCODEC_COLOR_SPACE_BT_709 = 0,   /* ITU BT 709 color standard */
    CNCODEC_COLOR_SPACE_BT_601,       /* ITU BT.601 color standard */
    CNCODEC_COLOR_SPACE_BT_2020,      /* ITU BT 2020 color standard */
    CNCODEC_COLOR_SPACE_BT_601_ER,    /* ITU BT 601 color standard extend range */
    CNCODEC_COLOR_SPACE_BT_709_ER,    /* ITU BT 709 color standard extend range */
    CNCODEC_COLOR_SPACE_INVALID,
} cncodecColorSpace;

typedef struct
{
    u64_t addr;        /* Device addr for memory buffer  */
    u32_t size;        /* total size of buffer memory */
    u64_t reserved;    /* reserved field */
    u64_t privData;    /* private Data */
} cncodecDevMemory;

#define CNCODEC_FRAME_MAX_PLANE_NUM (6U)
typedef struct
{
    cncodecPixelFormat   pixelFmt;                            /* Specify SurfaceFormat */
    cncodecColorSpace    colorSpace;                          /* color standard */
    u32_t                width;                               /* Specify Frame width, in pixel */
    u32_t                height;                              /* Specify Frame height, in pixel */
    u32_t                planeNum;                            /* The number of surface plane of current frame */
    cncodecDevMemory     plane[CNCODEC_FRAME_MAX_PLANE_NUM];  /* The information of each surface of current frame */
    u32_t                stride[CNCODEC_FRAME_MAX_PLANE_NUM]; /* Stride for current plane */
    u32_t                channel;                             /* planar device memory located memory channel  */
    u32_t                deviceId;                            /* planar device memory affiliated deviceId device */
    u64_t                reserved;
    u64_t                privData;
} cncodecFrame;

typedef struct
{
    u32_t  left;
    u32_t  top;
    u32_t  right;
    u32_t  bottom;
} cncodecRectangle;

typedef enum
{
    CNCODEC_Filter_BiLinear = 0x0,	/* Enable Bi-Linear filtering */
    CNCODEC_Filter_BiCubic,		/* Enable media quality filtering */
    CNCODEC_Filter_Lanczos2,		/* Enable best zoom out quality filtering Lanczos, window size=2 filtering */
    CNCODEC_Filter_Lanczos3,		/* Enable best zoom in quality  filtering Lanczos, window size=3 filtering */
    CNCODEC_NumFilters
} cncodecFilter;

typedef struct
{
    u32_t instance;	/*Choose hardware to use, 0, 1 or default 2*/
    u32_t batchNum;	/*Use for batch transform, at least 1 batch, maximum 256*/
    u32_t cardId;	/*Choose card to use when connect to multiple cards. Default is 0*/
    u32_t mode;		/*0 for normal transform, 1 for grey transform only. Default is 0*/
    u32_t cropNum;	/*Use for crop transform, maximum is 256 crops*/
} cncodecInMsg;

typedef struct
{
    double swPerf;	/*Transform software performance*/
    u64_t hwPerf;	/*Transform hardware performance*/
    u64_t metaData;	/*Reserved*/
} cncodecOutMsg;

typedef struct
{
    cncodecInMsg inMsg;
    cncodecOutMsg outMsg;
} cncodecWorkInfo;

typedef struct listHead
{
    struct listHead *next;
    struct listHead *prev;
} listHead;

typedef struct
{
    cncodecFrame cncodecParam;
    listHead listHead;
    u32_t seq;
} cncodecBatchParam;

typedef struct
{
    cncodecRectangle cncodecRect;
    listHead listHead;
    u32_t seq;
} cncodecCropParam;

typedef struct {
    u8_t reserved;
    u32_t frameBufAlignment;      /* [in]: set frame buffer aligment when lib allocates buffer*/
} cncodecExtBufCfg;

/**
 * decoder callbacks
 * The decoder will call these during decode, when a picture is decoded, or sequence info parsed, or EOS
 * First argument "void *pUserData" is used for bypass userptr
 */
typedef i32_t (*pfnCncodecEventCallback)(cncodecCbEventType, void*, void*);

/**
 * \brief Returns the version information for the cncodec library.
 *
 * @return pointer to version info
 */
extern CNCODEC_DLL_API u8_t* cncodecGetVersion(void);

/**
 * \brief Image transform size and format API
 *
 * @param dstFrame[in], contains destinate image information
 * @param dstRect[in],  designate crop destinate image position
 * @param srcFrame[in], contains source image information
 * @param srcRect[in],  designate crop source image position
 * @param filter[in],   designate transform filter, default is CNCODEC_NumFilters
 * @param workInfo[in/out], transfer in instance, mode etc; transfer out performance information
 * @return CNCODEC_SUCCESS if success, else failed
 */
extern CNCODEC_DLL_API i32_t cncodecImageTransform(cncodecFrame *dstFrame, cncodecRectangle *dstRect,
                                                   cncodecFrame *srcFrame, cncodecRectangle *srcRect,
                                                   cncodecFilter filter, cncodecWorkInfo *workInfo);

/**
 * \brief DtoD copy API
 *
 * @param devDstAddr[in], destination data address
 * @param devSrcAddr[in], source data address
 * @param size[in], data size
 * @param workInfo[in/out], transfer in instance, transfer out performance information
 * @return CNCODEC_SUCCESS if success, else failed
 */
extern CNCODEC_DLL_API i32_t cncodecMemcpyDtoD(void *devDstAddr, void *devSrcAddr,
                                               i64_t size, cncodecWorkInfo *workInfo);


/**
 * \brief Image batch transform API
 *
 * @param dstBatchList[in], consist a list of destinate images
 * @param srcBatchList[in], consist a list of source images
 * @param filter[in], designate transform filter, default is CNCODEC_NumFilters
 * @param workInfo[in/out], transfer in instance, mode etc; transfer out performance information
 * @return CNCODEC_SUCCESS if success, else failed
 */
extern CNCODEC_DLL_API i32_t cncodecBatchTransform(listHead *dstBatchList, listHead *dstCropList,
                                                   listHead *srcBatchList, listHead *srcCropList,
                                                   cncodecFilter filter, cncodecWorkInfo *workInfo);

/**
 * @param deviceId[in], specify the deviceId id to reset
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cncodecReset(i32_t deviceId);

/**
 * @param totDecNum[in], total decode channel num on this device(card)
 * @param totEncNum[in], total encode channel num on this device(card)
 * @param deviceId[in], specify the deviceId id to reset
 * @return CNCODEC_SUCCESS if success, else failed.
 */
extern CNCODEC_DLL_API i32_t cncodecQueryVideoDeviceLoading(u32_t *totDecNum, u32_t *totEncNum, i32_t deviceId);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_CODEC_COMMON_H_ */
