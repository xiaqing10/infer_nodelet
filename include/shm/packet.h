#ifndef __EHAWKEYE_MODULES_COMMON_PACKAGE_H__
#define __EHAWKEYE_MODULES_COMMON_PACKAGE_H__

#include <stdint.h>
#include <memory>

namespace ehawkeye {
namespace modules  {
namespace common   {

enum format : int16_t {
	VIDEO_PIX_FMT_YUYV422 = 0,
	VIDEO_PIX_FMT_YUV420P,
	VIDEO_PIX_FMT_NV12,
	VIDEO_PIX_FMT_NV16,
	VIDEO_PIX_FMT_RGB24,
	VIDEO_PIX_FMT_BGR24,
	VIDEO_PIX_FMT_ARGB,
	VIDEO_PIX_FMT_RGBA,
	VIDEO_PIX_FMT_ABGR,
	VIDEO_PIX_FMT_BGRA,
    VIDEO_PIX_FMT_JPEG,
	VIDEO_PIX_FMT_NB,

	AUDIO_CODEC_FMT_NONE = 32,
	AUDIO_CODEC_FMT_H264,
	AUDIO_CODEC_FMT_H265,
	AUDIO_CODEC_FMT_MJPEG,
	AUDIO_CODEC_FMT_NB,

	VIDEO_CODEC_FMT_NONE = 64,
	VIDEO_CODEC_FMT_H264,
	VIDEO_CODEC_FMT_H265,
	VIDEO_CODEC_FMT_MJPEG,
	VIDEO_CODEC_FMT_NB,
};

enum type : int16_t {
	MEMORY_TYPE_COMMON,
	MEMORY_TYPE_SHM
};

struct alignas(1) packet {
	int32_t magic;
    int32_t size;
	int16_t pixfmt;
    int16_t type;
	int32_t nalu;
    int32_t width;
    int32_t height;
    int64_t dts;
	char    reserved[32];
    char    data[0];
};

} // namespace common
} // namespace modules
} // namespace ehawkeye
#endif//__EHAWKEYE_MODULES_COMMON_PACKAGE_H__